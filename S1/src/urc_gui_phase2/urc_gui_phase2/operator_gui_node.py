"""Entry point: the operator GUI, with rclpy driven from a Qt QTimer.

This is the single combined S1 GUI. It supersedes urc_rover_console's battery_gui.py (left in
place, now redundant). Run it alongside two simulators: rover_sim_node (NavSatFix on /rover/fix)
and urc_rover_console's rover_simulator (JSON on /rover/telemetry).

Everything runs on the Qt main thread. A QTimer periodically calls the
executor's spin_once(timeout_sec=0), which runs any ROS callbacks that are
ready and returns immediately. Because callbacks execute on the same thread
as the widgets, they can update the GUI directly -- no worker thread, no
cross-thread signals, no locks.

Trade-off (defend this honestly): a callback that blocks freezes the GUI, and
ROS latency is bounded by the timer period. Callbacks must therefore stay
short. The timer also keeps ticking while a modal dialog is open, because
QDialog.exec_() runs a nested Qt event loop.
"""

import json
import math
import sys
import time
from typing import Callable, Optional

from geometry_msgs.msg import PoseStamped
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QGridLayout, QHBoxLayout, QLabel, QMainWindow, QPushButton, QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy._rclpy_pybind11 import RCLError  # rclpy does not re-export it
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from urc_gui_phase2.coordinate_convert import LocalFrame
from urc_gui_phase2.map_widget import OfflineMapWidget
from urc_gui_phase2.mission_controller import MissionController
from urc_gui_phase2.mission_model import TargetType
from urc_gui_phase2.mission_panel import ADD_MODE_OFF_TEXT, USER_ERRORS, MissionPanel
from urc_gui_phase2.telemetry_monitor import TelemetryMonitor

SPIN_PERIOD_MS = 10
# spin_once() runs at most one ready callback per call, so drain a bounded
# number per tick. The bound stops a message flood from starving Qt painting.
MAX_SPINS_PER_TICK = 10

FIX_TOPIC = '/rover/fix'                      # sensor_msgs/NavSatFix, rover -> GUI
ACTIVE_TARGET_TOPIC = '/mission/active_target'  # geometry_msgs/PoseStamped, GUI -> rover
LOCAL_FRAME_ID = 'map'                        # ENU, origin = rover spawn point
TELEMETRY_TOPIC = '/rover/telemetry'          # std_msgs/String (JSON), rover -> GUI
COMMAND_TOPIC = '/operator/command'           # std_msgs/String, GUI -> rover
STALE_AFTER_S = 2.5

# (button text, command string, objectName for the stylesheet). START/ABORT are the strings
# rover_simulator.py actually handles. STOP/RESET use the requested names; the simulator
# ignores them until it grows handlers.
COMMAND_BUTTONS = [
    ('START NEXT TARGET', 'START_NEXT_TARGET', ''),
    ('STOP', 'STOP_MISSION', ''),
    ('ABORT AND RETURN', 'ABORT_AND_RETURN', 'danger'),
    ('RESET', 'RESET_MISSION', ''),
]

STYLESHEET = """
    QWidget { background:#f8fafc; color:#1e293b; font:14px 'DejaVu Sans'; }
    QGroupBox { background:white; border:1px solid #cbd5e1; border-radius:6px;
                margin-top:18px; padding:12px 8px 8px 8px; font-weight:700; }
    QGroupBox::title { subcontrol-origin:margin; left:12px; padding:0 5px; }
    QLabel#title { font-size:20px; font-weight:800; }
    QLabel#clock { font:700 21px monospace; padding:6px 14px; }
    QLabel#banner { background:#e2e8f0; border:1px solid #94a3b8; padding:12px;
                    font-size:18px; font-weight:800; }
    QLabel#waiting { background:#475569; color:white; padding:9px; font-weight:700; }
    QLabel#active { background:#15803d; color:white; padding:9px; font-weight:700; }
    QLabel#stale { background:#b91c1c; color:white; padding:9px; font-weight:700; }
    QLabel#value { font-weight:700; }
    QPushButton { min-height:44px; padding:0 18px; background:#2563eb; color:white;
                  border:0; border-radius:5px; font-weight:700; }
    QPushButton#danger { background:#b91c1c; }
    QProgressBar { min-height:22px; text-align:center; }
    QProgressBar::chunk { background:#2563eb; }
"""


def _value_label(text='--'):
    label = QLabel(text)
    label.setObjectName('value')
    return label


class OperatorGuiNode(Node):
    """ROS side of the GUI. Holds no widgets and no Qt.

    * subscribes NavSatFix on /rover/fix and hands (lat, lon) to `on_fix`
    * subscribes /rover/telemetry (JSON in a std_msgs/String) and hands the parsed dict to
      `on_telemetry`; a message that is not valid JSON is reported via `on_telemetry_error`
    * publishes operator commands (std_msgs/String) on /operator/command
    * publishes the active waypoint as a PoseStamped in the local ENU frame
      ("map", x=east, y=north, metres) on /mission/active_target. Publishing
      nothing is published while there is no active target or no frame yet.

    Parameters spawn_lat_deg / spawn_lon_deg give the ENU origin (same names as
    rover_sim_node). If unset (NaN), the window anchors the frame at the
    first valid fix.
    """

    def __init__(self):
        super().__init__('operator_gui_node')
        self.declare_parameter('spawn_lat_deg', float('nan'))
        self.declare_parameter('spawn_lon_deg', float('nan'))
        self.on_fix: Optional[Callable[[float, float], None]] = None
        self.on_telemetry: Optional[Callable[[dict], None]] = None
        self.on_telemetry_error: Optional[Callable[[], None]] = None
        self._target_pub = self.create_publisher(PoseStamped, ACTIVE_TARGET_TOPIC, 10)
        self._command_pub = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.create_subscription(NavSatFix, FIX_TOPIC, self._on_navsat, 10)
        self.create_subscription(String, TELEMETRY_TOPIC, self._on_telemetry, 10)

    def spawn_origin(self):
        lat = self.get_parameter('spawn_lat_deg').value
        lon = self.get_parameter('spawn_lon_deg').value
        if math.isnan(lat) or math.isnan(lon):
            return None
        return lat, lon

    def _on_navsat(self, msg: NavSatFix) -> None:
        # status < 0 is NO_FIX; NaN means the receiver has no solution.
        if msg.status.status < 0 or not (math.isfinite(msg.latitude)
                                         and math.isfinite(msg.longitude)):
            return
        if self.on_fix is not None:
            self.on_fix(msg.latitude, msg.longitude)

    def _on_telemetry(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            if self.on_telemetry_error is not None:
                self.on_telemetry_error()
            return
        if self.on_telemetry is not None:
            self.on_telemetry(data)

    def publish_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self._command_pub.publish(msg)

    def publish_active_target(self, east_m: float, north_m: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = LOCAL_FRAME_ID
        msg.pose.position.x = float(east_m)
        msg.pose.position.y = float(north_m)
        msg.pose.orientation.w = 1.0
        self._target_pub.publish(msg)


class OperatorWindow(QMainWindow):
    """Status header + nav row + command buttons, over map + mission panel (one MissionController)."""

    def __init__(self, node: OperatorGuiNode, controller: MissionController):
        super().__init__()
        self._node = node
        self._ctl = controller
        self.setWindowTitle('URC Phase 2 - Navigation GUI')
        self.resize(1300, 800)
        self._telemetry_monitor = TelemetryMonitor(stale_after_seconds=STALE_AFTER_S)
        self._start_time = time.monotonic()

        self._map = OfflineMapWidget()
        self._panel = MissionPanel(controller)
        self._panel.setFixedWidth(420)
        central = QWidget()
        root = QVBoxLayout(central)
        root.addLayout(self._build_header())
        self._banner = QLabel('WAITING FOR ROVER STATUS')
        self._banner.setObjectName('banner')
        self._banner.setAlignment(Qt.AlignCenter)
        root.addWidget(self._banner)
        root.addLayout(self._build_nav_row())
        body = QHBoxLayout()
        body.addWidget(self._map, 1)
        body.addWidget(self._panel)
        root.addLayout(body, 1)
        root.addLayout(self._build_commands())
        self.setCentralWidget(central)
        self.setStyleSheet(STYLESHEET)

        # controller -> map
        controller.missionChanged.connect(lambda: self._map.set_waypoints(controller.model))
        controller.roverMoved.connect(self._map.set_rover_position)
        controller.frameChanged.connect(self._map.set_local_frame)
        controller.selectionChanged.connect(self._on_controller_selection)
        # map -> controller (a map click selects; panel and map stay in step)
        self._map.selectionChanged.connect(self._on_map_selection)
        # a click on empty map proposes a new waypoint, but only while the panel's add mode is
        # armed; the controller validates it. Add mode also drives the map cursor.
        self._map.mapClickedForNewWaypoint.connect(self._on_map_click_add)
        self._panel.addModeChanged.connect(self._map.set_crosshair_cursor)
        # controller -> ROS
        controller.activeTargetChanged.connect(self._publish_target)
        controller.frameChanged.connect(lambda _f: self._publish_target(None))
        # ROS -> controller
        node.on_fix = self._on_fix
        node.on_telemetry = self._on_telemetry
        node.on_telemetry_error = self._on_telemetry_error

        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._update_clock_and_link)
        self._ui_timer.start(250)

        self._map.set_waypoints(controller.model)
        if controller.frame is not None:
            self._map.set_local_frame(controller.frame)

    def _build_header(self) -> QHBoxLayout:
        header = QHBoxLayout()
        title = QLabel('AUTONOMOUS NAVIGATION — ROVER OPERATIONS CONSOLE')
        title.setObjectName('title')
        self._clock = QLabel('00:00')
        self._clock.setObjectName('clock')
        self._link = QLabel('WAITING FOR TELEMETRY')
        self._link.setObjectName('waiting')
        header.addWidget(title, 1)
        header.addWidget(QLabel('MISSION TIME'))
        header.addWidget(self._clock)
        header.addWidget(self._link)
        return header

    def _build_nav_row(self) -> QGridLayout:
        self._target = _value_label()
        self._nav_state = _value_label()
        self._distance = _value_label()
        self._accuracy = _value_label()
        row = QGridLayout()
        for col, (name, field) in enumerate([
                ('Current target', self._target), ('Rover state', self._nav_state),
                ('Distance', self._distance), ('GNSS accuracy', self._accuracy)]):
            row.addWidget(QLabel(name), 0, 2 * col)
            row.addWidget(field, 0, 2 * col + 1)
            row.setColumnStretch(2 * col + 1, 1)
        return row

    def _build_commands(self) -> QHBoxLayout:
        commands = QHBoxLayout()
        self._feedback = QLabel('No operator command sent')
        commands.addWidget(self._feedback, 1)
        for text, command, object_name in COMMAND_BUTTONS:
            button = QPushButton(text)
            if object_name:
                button.setObjectName(object_name)
            button.clicked.connect(lambda _checked=False, c=command: self._send_command(c))
            commands.addWidget(button)
        return commands

    def _send_command(self, command: str) -> None:
        self._node.publish_command(command)
        self._feedback.setText(f"Command sent: {command.replace('_', ' ')}")

    def _update_clock_and_link(self) -> None:
        elapsed = int(time.monotonic() - self._start_time)
        self._clock.setText(f'{elapsed // 60:02d}:{elapsed % 60:02d}')
        state, age = self._telemetry_monitor.connection_state()
        if state == 'waiting':
            self._link.setObjectName('waiting')
            self._link.setText('WAITING FOR TELEMETRY')
        elif state == 'active':
            self._link.setObjectName('active')
            self._link.setText(f'LINK ACTIVE · {age:.1f}s')
        else:
            self._link.setObjectName('stale')
            self._link.setText(f'TELEMETRY STALE · {age:.1f}s')
        self._link.style().unpolish(self._link)
        self._link.style().polish(self._link)

    def _on_telemetry(self, data: dict) -> None:
        self._telemetry_monitor.mark_received()
        try:
            nav = data['navigation']
            target, state = nav['target'], nav['state']
            distance, accuracy = nav['distance_m'], nav['gnss_accuracy_m']
        except (KeyError, TypeError):
            self._link.setText('INVALID TELEMETRY MESSAGE')
            return
        self._target.setText(target)
        self._nav_state.setText(state)
        self._distance.setText(f'{distance:.1f} m')
        self._accuracy.setText(f'±{accuracy:.1f} m')
        self._banner.setText('TARGET REACHED — ROVER STOPPED' if state == 'ARRIVED'
                             else f'{state} — {target}')
        if state == 'RETURNING':
            self._feedback.setText('Rover acknowledged abort and is returning')

    def _on_telemetry_error(self) -> None:
        self._telemetry_monitor.mark_received()
        self._link.setText('INVALID TELEMETRY MESSAGE')

    def _on_fix(self, lat: float, lon: float) -> None:
        if self._ctl.frame is None:
            # No spawn parameters: the first fix is the spawn point.
            self._ctl.set_frame(LocalFrame(lat, lon))
        self._ctl.set_rover_fix(lat, lon)

    def _on_map_selection(self, wp_id) -> None:
        if wp_id != self._ctl.selected_id:
            self._ctl.select(wp_id)

    def _on_map_click_add(self, lat: float, lon: float) -> None:
        """Add a GNSS waypoint where the map was clicked, if add mode is armed.

        Add mode is one-shot: it turns itself off after a successful add, so a second stray
        click cannot place another waypoint. A rejected add keeps it armed so the operator can
        click again; the rejection is reported, not raised.
        """
        if not self._panel.add_mode:
            self._panel.report(f"Not adding a waypoint: press '{ADD_MODE_OFF_TEXT}' first.", ok=None)
            return
        try:
            wp = self._ctl.add_waypoint(self._ctl.default_waypoint_name(), lat, lon, TargetType.GNSS)
        except USER_ERRORS as exc:  # InvalidCoordinateError, incl. FrameRangeError
            self._panel.report(f'Waypoint not added at {lat:.6f}, {lon:.6f}: {exc}', ok=False)
            return
        self._panel.set_add_mode(False)
        self._ctl.select(wp.id)
        self._panel.report(f'Added {wp.name} at {lat:.6f}, {lon:.6f}', ok=True)

    def _on_controller_selection(self, wp_id) -> None:
        if wp_id != self._map.selected_waypoint_id:
            self._map.select_waypoint(wp_id)

    def _publish_target(self, _waypoint) -> None:
        try:
            enu = self._ctl.active_target_enu()
        except Exception as exc:  # out-of-range target: report, don't crash a Qt slot
            self._node.get_logger().error(f'cannot publish active target: {exc}')
            return
        if enu is not None:
            self._node.publish_active_target(enu.east_m, enu.north_m)


def main():
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)

    node = OperatorGuiNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # A timer event already queued when exec_() returns can still fire after timer.stop();
    # this flag keeps it from calling into a ROS context that is being shut down.
    shutting_down = False

    def spin_ros():
        if shutting_down:
            return
        try:
            for _ in range(MAX_SPINS_PER_TICK):
                executor.spin_once(timeout_sec=0)
        except RCLError:
            return  # context already gone (shutdown race); nothing left to spin

    timer = QTimer()
    timer.timeout.connect(spin_ros)
    timer.start(SPIN_PERIOD_MS)

    origin = node.spawn_origin()
    controller = MissionController(frame=LocalFrame(*origin) if origin else None)
    window = OperatorWindow(node, controller)
    window.show()

    exit_code = app.exec_()

    # No spin thread to join: stop the timer, then tear ROS down in order.
    shutting_down = True
    timer.stop()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
