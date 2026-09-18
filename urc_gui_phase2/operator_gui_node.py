"""Entry point: the operator GUI, with rclpy driven from a Qt QTimer.

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

import math
import sys
from typing import Callable, Optional

from geometry_msgs.msg import PoseStamped
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QHBoxLayout, QMainWindow, QWidget
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from urc_gui_phase2.coordinate_convert import LocalFrame
from urc_gui_phase2.map_widget import OfflineMapWidget
from urc_gui_phase2.mission_controller import MissionController
from urc_gui_phase2.mission_panel import MissionPanel

SPIN_PERIOD_MS = 10
# spin_once() runs at most one ready callback per call, so drain a bounded
# number per tick. The bound stops a message flood from starving Qt painting.
MAX_SPINS_PER_TICK = 10

FIX_TOPIC = '/rover/fix'                      # sensor_msgs/NavSatFix, rover -> GUI
ACTIVE_TARGET_TOPIC = '/mission/active_target'  # geometry_msgs/PoseStamped, GUI -> rover
LOCAL_FRAME_ID = 'map'                        # ENU, origin = rover spawn point


class OperatorGuiNode(Node):
    """ROS side of the GUI. Holds no widgets and no Qt.

    * subscribes NavSatFix on /rover/fix and hands (lat, lon) to `on_fix`
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
        self._target_pub = self.create_publisher(PoseStamped, ACTIVE_TARGET_TOPIC, 10)
        self.create_subscription(NavSatFix, FIX_TOPIC, self._on_navsat, 10)

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

    def publish_active_target(self, east_m: float, north_m: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = LOCAL_FRAME_ID
        msg.pose.position.x = float(east_m)
        msg.pose.position.y = float(north_m)
        msg.pose.orientation.w = 1.0
        self._target_pub.publish(msg)


class OperatorWindow(QMainWindow):
    """Map + mission panel, both driven by one MissionController."""

    def __init__(self, node: OperatorGuiNode, controller: MissionController):
        super().__init__()
        self._node = node
        self._ctl = controller
        self.setWindowTitle('URC Phase 2 - Navigation GUI')
        self.resize(1300, 800)

        self._map = OfflineMapWidget()
        self._panel = MissionPanel(controller)
        self._panel.setFixedWidth(420)
        central = QWidget()
        layout = QHBoxLayout(central)
        layout.addWidget(self._map, 1)
        layout.addWidget(self._panel)
        self.setCentralWidget(central)

        # controller -> map
        controller.missionChanged.connect(lambda: self._map.set_waypoints(controller.model))
        controller.roverMoved.connect(self._map.set_rover_position)
        controller.frameChanged.connect(self._map.set_local_frame)
        controller.selectionChanged.connect(self._on_controller_selection)
        # map -> controller (a map click selects; panel and map stay in step)
        self._map.selectionChanged.connect(self._on_map_selection)
        # controller -> ROS
        controller.activeTargetChanged.connect(self._publish_target)
        controller.frameChanged.connect(lambda _f: self._publish_target(None))
        # ROS -> controller
        node.on_fix = self._on_fix

        self._map.set_waypoints(controller.model)
        if controller.frame is not None:
            self._map.set_local_frame(controller.frame)

    def _on_fix(self, lat: float, lon: float) -> None:
        if self._ctl.frame is None:
            # No spawn parameters: the first fix is the spawn point.
            self._ctl.set_frame(LocalFrame(lat, lon))
        self._ctl.set_rover_fix(lat, lon)

    def _on_map_selection(self, wp_id) -> None:
        if wp_id != self._ctl.selected_id:
            self._ctl.select(wp_id)

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

    def spin_ros():
        for _ in range(MAX_SPINS_PER_TICK):
            executor.spin_once(timeout_sec=0)

    timer = QTimer()
    timer.timeout.connect(spin_ros)
    timer.start(SPIN_PERIOD_MS)

    origin = node.spawn_origin()
    controller = MissionController(frame=LocalFrame(*origin) if origin else None)
    window = OperatorWindow(node, controller)
    window.show()

    exit_code = app.exec_()

    # No spin thread to join: stop the timer, then tear ROS down in order.
    timer.stop()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
