import json
import math
import sys
import time

import rclpy
from std_msgs.msg import String
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)
from urc_rover_console.telemetry_monitor import TelemetryMonitor
from urc_gui_phase2.map_widget import OfflineMapWidget
from urc_gui_phase2.mission_controller import MissionController
from urc_gui_phase2.mission_model import TargetType
from urc_gui_phase2.mission_panel import (
    ADD_MODE_OFF_TEXT,
    USER_ERRORS,
    MissionPanel,
)
from urc_gui_phase2.coordinate_convert import LocalFrame
from urc_gui_phase2.operator_gui_node import OperatorGuiNode
from urc_gui_phase2.rover_track import RoverTrack

DATA_STALE_AFTER_S = 2.5


class RoverConsole(QMainWindow):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.telemetry_monitor = TelemetryMonitor(DATA_STALE_AFTER_S)
        self.fix_monitor = TelemetryMonitor(DATA_STALE_AFTER_S)
        self.status_monitor = TelemetryMonitor(DATA_STALE_AFTER_S)
        self.start_time = time.monotonic()
        self.rover_track = RoverTrack()
        self._last_mission_state = None
        self._current_mission_state = "UNKNOWN"
        self._completing_arrival = False
        self._safety_stop_sent = False
        origin = self.node.spawn_origin()

        if origin is None:
            frame = None
        else:
            frame = LocalFrame(*origin)

        self.mission_controller = MissionController(frame=frame)
        self.setWindowTitle("URC Autonomous Navigation Rover Operations Console")
        self.resize(1500, 900)
        self.build_ui()
        self.node.on_fix = self.on_rover_fix

        self.subscription = node.create_subscription(
            String, "/rover/telemetry", self.update_telemetry, 10)
        self.status_subscription = node.create_subscription(
            String, "/mission/status", self.update_mission_status, 10)
        self.command_publisher = node.create_publisher(String, "/operator/command", 10)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self.ros_timer.start(50)
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.update_clock_and_link)
        self.ui_timer.start(250)

    def build_ui(self):
        page = QWidget()
        root = QVBoxLayout(page)

        # Application header
        header = QHBoxLayout()

        title = QLabel("S1 AUTONOMOUS NAVIGATION CONSOLE")
        title.setObjectName("title")

        self.clock = QLabel("00:00")
        self.clock.setObjectName("clock")

        self.link = QLabel("WAITING FOR TELEMETRY")
        self.link.setObjectName("waiting")

        header.addWidget(title, 1)
        header.addWidget(QLabel("MISSION TIME"))
        header.addWidget(self.clock)
        header.addWidget(self.link)

        root.addLayout(header)

        # Prominent mission-state banner
        self.banner = QLabel("WAITING FOR ROVER STATUS")
        self.banner.setObjectName("banner")
        self.banner.setAlignment(Qt.AlignCenter)
        root.addWidget(self.banner)

        # Compact navigation information
        status_row = QHBoxLayout()

        self.target = QLabel("Target: --")
        self.nav_state = QLabel("State: --")
        self.distance = QLabel("Distance: --")
        self.accuracy = QLabel("GNSS accuracy: --")

        for label in (
            self.target,
            self.nav_state,
            self.distance,
            self.accuracy,
        ):
            label.setObjectName("status_value")
            status_row.addWidget(label)

        root.addLayout(status_row)

        # Phase II map and mission editor
        integration_area = QWidget()
        integration_layout = QHBoxLayout(integration_area)
        integration_layout.setContentsMargins(0, 0, 0, 0)

        self.map_widget = OfflineMapWidget()

        self.mission_panel = MissionPanel(
            self.mission_controller
        )
        self.mission_panel.setFixedWidth(420)

        integration_layout.addWidget(self.map_widget, 1)
        integration_layout.addWidget(self.mission_panel)

        root.addWidget(integration_area, 1)

        self.connect_mission_components()

        # Mission controls
        commands = QHBoxLayout()

        self.feedback = QLabel("No operator command sent")

        self.start_button = QPushButton("START")
        # START remains unavailable until both safety-critical ROS streams have
        # produced fresh data. STOP, ABORT, and RESET remain usable at all times.
        self.start_button.setEnabled(False)
        self.start_button.setToolTip(
            "Waiting for fresh GNSS and mission-status data"
        )
        self.stop_button = QPushButton("STOP")
        self.abort_button = QPushButton("ABORT")
        self.reset_button = QPushButton("RESET")

        self.abort_button.setObjectName("danger")

        self.start_button.clicked.connect(
            lambda: self.send_command("START_MISSION")
        )
        self.stop_button.clicked.connect(
            lambda: self.send_command("STOP_MISSION")
        )
        self.abort_button.clicked.connect(
            lambda: self.send_command("ABORT_MISSION")
        )
        self.reset_button.clicked.connect(
            lambda: self.send_command("RESET_MISSION")
        )

        commands.addWidget(self.feedback, 1)
        commands.addWidget(self.start_button)
        commands.addWidget(self.stop_button)
        commands.addWidget(self.abort_button)
        commands.addWidget(self.reset_button)

        root.addLayout(commands)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        self.setCentralWidget(scroll)

        self.setStyleSheet("""
            QWidget {
                background: #f8fafc;
                color: #1e293b;
                font: 14px 'DejaVu Sans';
            }

            QLabel#title {
                font-size: 20px;
                font-weight: 800;
            }

            QLabel#clock {
                font: 700 21px monospace;
                padding: 6px 14px;
            }

            QLabel#banner {
                background: #e2e8f0;
                border: 1px solid #94a3b8;
                padding: 12px;
                font-size: 18px;
                font-weight: 800;
            }

            QLabel#waiting {
                background: #475569;
                color: white;
                padding: 9px;
                font-weight: 700;
            }

            QLabel#active {
                background: #15803d;
                color: white;
                padding: 9px;
                font-weight: 700;
            }

            QLabel#stale {
                background: #b91c1c;
                color: white;
                padding: 9px;
                font-weight: 700;
            }

            QLabel#degraded {
                background: #b45309;
                color: white;
                padding: 9px;
                font-weight: 700;
            }

            QLabel#status_value {
                background: white;
                border: 1px solid #cbd5e1;
                padding: 10px;
                font-weight: 700;
            }

            QLabel#integration_area {
                background: white;
                border: 2px dashed #94a3b8;
                color: #64748b;
                font-size: 20px;
                font-weight: 700;
            }

            QPushButton {
                min-height: 44px;
                padding: 0 18px;
                background: #2563eb;
                color: white;
                border: 0;
                border-radius: 5px;
                font-weight: 700;
            }

            QPushButton#danger {
                background: #b91c1c;
            }
        """)

    def connect_mission_components(self):
        """Connect the mission controller to the map."""

        self.mission_controller.missionChanged.connect(
            self.refresh_map_waypoints
        )

        self.mission_controller.selectionChanged.connect(
            self.on_controller_selection
        )

        self.map_widget.selectionChanged.connect(
            self.on_map_selection
        )
        self.map_widget.mapClickedForNewWaypoint.connect(
            self.on_map_click_add
        )

        self.mission_panel.addModeChanged.connect(
            self.map_widget.set_crosshair_cursor
        )

        self.mission_controller.roverMoved.connect(
            self.on_rover_moved
        )

        self.mission_controller.frameChanged.connect(
            self.map_widget.set_local_frame
        )

        self.mission_controller.activeTargetChanged.connect(
            self.publish_active_target
        )

        if self.mission_controller.frame is not None:
            self.map_widget.set_local_frame(
                self.mission_controller.frame
            )
        self.refresh_map_waypoints()

    def on_map_click_add(self, latitude, longitude):
        """Create a GNSS waypoint from an armed map click."""

        if not self.mission_panel.add_mode:
            self.mission_panel.report(
                f"Not adding a waypoint: press "
                f"'{ADD_MODE_OFF_TEXT}' first.",
                ok=None,
            )
            return

        try:
            waypoint = self.mission_controller.add_waypoint(
                self.mission_controller.default_waypoint_name(),
                latitude,
                longitude,
                TargetType.GNSS,
            )
        except USER_ERRORS as error:
            self.mission_panel.report(
                f"Waypoint not added at "
                f"{latitude:.6f}, {longitude:.6f}: {error}",
                ok=False,
            )
            return

        self.mission_panel.set_add_mode(False)
        self.mission_controller.select(waypoint.id)

        self.mission_panel.report(
            f"Added {waypoint.name} at "
            f"{latitude:.6f}, {longitude:.6f}",
            ok=True,
        )

    def on_rover_fix(self, latitude, longitude):
        """Process a valid WGS 84 position received from the rover."""
        self.fix_monitor.mark_received()

        if self.mission_controller.frame is None:
            self.mission_controller.set_frame(
                LocalFrame(latitude, longitude)
            )

        self.mission_controller.set_rover_fix(
            latitude,
            longitude,
        )

    def on_rover_moved(self, latitude, longitude):
        """Update marker, stable heading, and traveled path from a GNSS fix."""
        self.map_widget.set_rover_position(latitude, longitude)
        rover_enu = self.mission_controller.rover_enu()
        if rover_enu is None:
            return

        changed = self.rover_track.add_fix(
            latitude,
            longitude,
            rover_enu.east_m,
            rover_enu.north_m,
        )
        if not changed:
            return

        self.map_widget.set_rover_path(self.rover_track.points)
        if self.rover_track.heading_deg is not None:
            self.map_widget.set_rover_heading(
                self.rover_track.heading_deg
            )

    def publish_active_target(self, _waypoint):
        """Publish the active target in the local REP 103 frame."""

        active_waypoint = self.mission_controller.active_target()

        if active_waypoint is None:
            # Completing the final waypoint deliberately leaves the simulator
            # in ARRIVED so the success state remains visible. Other removals
            # still cancel the rover's target immediately.
            if self._completing_arrival:
                return False
            cancel_message = String()
            cancel_message.data = "CANCEL_TARGET"
            self.command_publisher.publish(cancel_message)
            return False

        if self.mission_controller.frame is None:
            self.mission_panel.report(
                "Target cannot be sent until a rover position establishes "
                "the local coordinate frame.",
                ok=False,
            )
            return False

        try:
            target = self.mission_controller.active_target_enu()
        except USER_ERRORS as error:
            self.mission_panel.report(
                f"Target could not be converted: {error}",
                ok=False,
            )
            return False

        self.node.publish_active_target(
            target.east_m,
            target.north_m,
        )
        return True

    def refresh_map_waypoints(self):
        """Redraw map markers using the current mission."""

        self.map_widget.set_waypoints(
            self.mission_controller.model
        )

    def on_map_selection(self, waypoint_id):
        """Select a waypoint when its map marker is clicked."""

        if waypoint_id != self.mission_controller.selected_id:
            self.mission_controller.select(waypoint_id)

    def on_controller_selection(self, waypoint_id):
        """Highlight the selected mission waypoint on the map."""

        if waypoint_id != self.map_widget.selected_waypoint_id:
            self.map_widget.select_waypoint(waypoint_id)

    def send_command(self, command):
        if command == "START_MISSION":
            failures = self.critical_data_failures()
            if failures:
                text = (
                    "Start blocked: waiting for fresh "
                    f"{', '.join(failures)} data."
                )
                self.feedback.setText(text)
                self.mission_panel.report(text, ok=False)
                return False

            if self.mission_controller.active_target() is None:
                text = "Start blocked: set an active waypoint first."
                self.feedback.setText(text)
                self.mission_panel.report(text, ok=False)
                return False

            if not self.publish_active_target(
                self.mission_controller.active_target()
            ):
                self.feedback.setText(
                    "Start blocked: active target could not be sent."
                )
                return False

        message = String()
        message.data = command
        self.command_publisher.publish(message)

        if command == "RESET_MISSION":
            # STOP and ABORT preserve the track for review; RESET explicitly
            # starts a new run and therefore clears historical map overlays.
            self.rover_track.reset()
            self.map_widget.clear_rover_track()

        self.feedback.setText(f"Command sent: {command.replace('_', ' ')}")
        return True

    def update_clock_and_link(self):
        elapsed = int(time.monotonic() - self.start_time)
        self.clock.setText(f"{elapsed // 60:02d}:{elapsed % 60:02d}")

        streams = self.data_stream_states()
        failures = self.critical_data_failures(streams)
        diagnostics_state, diagnostics_age = streams["diagnostics"]

        if failures:
            stale = [
                name for name in failures
                if streams[name][0] == "stale"
            ]
            self.link.setObjectName("stale" if stale else "waiting")
            problem = "STALE" if stale else "MISSING"
            self.link.setText(f"{problem}: {', '.join(failures).upper()}")
        elif diagnostics_state != "active":
            # Power/diagnostic loss is visible but does not invalidate fresh
            # position and mission-state data used for autonomous motion.
            self.link.setObjectName("degraded")
            detail = (
                f"{diagnostics_age:.1f}s"
                if diagnostics_age is not None else "missing"
            )
            self.link.setText(f"NAV DATA ACTIVE · DIAGNOSTICS {detail}")
        else:
            self.link.setObjectName("active")
            self.link.setText("ROS DATA ACTIVE")

        ready = not failures
        self.start_button.setEnabled(ready)
        self.start_button.setToolTip(
            "" if ready else
            f"Waiting for fresh {', '.join(failures)} data"
        )

        if failures:
            failure_text = ", ".join(failures).upper()
            if self._current_mission_state == "NAVIGATING":
                self.send_safety_stop_once(failures)
            if self._safety_stop_sent:
                self.banner.setText(
                    f"SAFETY STOP — {failure_text} DATA UNAVAILABLE"
                )
            else:
                self.banner.setText(
                    f"DATA UNAVAILABLE — {failure_text}"
                )
        else:
            # A recovered stream permits one future safety stop if another
            # distinct outage occurs.
            self._safety_stop_sent = False

        self.link.style().unpolish(self.link)
        self.link.style().polish(self.link)

    def data_stream_states(self):
        """Return health for each ROS input used by the combined console."""
        return {
            "gnss": self.fix_monitor.connection_state(),
            "mission status": self.status_monitor.connection_state(),
            "diagnostics": self.telemetry_monitor.connection_state(),
        }

    def critical_data_failures(self, streams=None):
        """List safety-critical streams that are missing or stale."""
        streams = streams if streams is not None else self.data_stream_states()
        return [
            name for name in ("gnss", "mission status")
            if streams[name][0] != "active"
        ]

    def send_safety_stop_once(self, failures):
        """Request one fail-safe stop for the current critical-data outage."""
        if self._safety_stop_sent:
            return
        message = String()
        message.data = "STOP_MISSION"
        self.command_publisher.publish(message)
        self._safety_stop_sent = True
        self.feedback.setText(
            "Safety STOP sent because fresh "
            f"{', '.join(failures)} data became unavailable."
        )

    def update_telemetry(self, message):
        """Update diagnostics that are not owned by the navigation simulator."""
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            self.link.setText("INVALID TELEMETRY MESSAGE")
            return

        try:
            nav = data["navigation"]
            accuracy = float(nav["gnss_accuracy_m"])
            if not math.isfinite(accuracy) or accuracy < 0.0:
                raise ValueError("invalid GNSS accuracy")
        except (KeyError, TypeError, ValueError):
            self.link.setText("INCOMPLETE TELEMETRY MESSAGE")
            return

        # Invalid payloads do not refresh stream health; receiving bytes is
        # not the same as receiving trustworthy diagnostic information.
        self.telemetry_monitor.mark_received()
        self.accuracy.setText(f"GNSS accuracy: ±{accuracy:.1f} m")

    def update_mission_status(self, message):
        """Display the state produced by the node that actually moves the rover."""
        valid_states = {
            "IDLE",
            "READY",
            "NAVIGATING",
            "STOPPED",
            "ARRIVED",
            "ABORTED",
        }

        try:
            data = json.loads(message.data)
            state = data["state"]
            has_target = data["has_target"]
            distance = data["distance_m"]
            if state not in valid_states or not isinstance(has_target, bool):
                raise ValueError("invalid state or target flag")
            if distance is not None:
                distance = float(distance)
                if not math.isfinite(distance) or distance < 0.0:
                    raise ValueError("invalid target distance")
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self.feedback.setText("INVALID MISSION STATUS MESSAGE")
            return

        self.status_monitor.mark_received()
        self._current_mission_state = state
        active = self.mission_controller.active_target()
        target_name = active.name if has_target and active is not None else "--"
        self.target.setText(f"Target: {target_name}")
        self.nav_state.setText(f"State: {state}")
        self.distance.setText(
            "Distance: --" if distance is None else f"Distance: {distance:.1f} m"
        )

        if state == "ARRIVED":
            self.banner.setText("TARGET REACHED — ROVER STOPPED")
        elif state == "READY":
            self.banner.setText(f"READY — {target_name} — PRESS START")
        else:
            self.banner.setText(f"{state} — {target_name}")

        # Status is published repeatedly. Only the transition into ARRIVED may
        # modify the mission, otherwise the same waypoint would be completed
        # again on every simulator tick.
        entered_arrived = (
            state == "ARRIVED" and self._last_mission_state != "ARRIVED"
        )
        self._last_mission_state = state
        if entered_arrived:
            self.complete_arrived_waypoint()

        if state == "ABORTED":
            self.feedback.setText("Rover acknowledged abort command")

    def complete_arrived_waypoint(self):
        """Complete one reached target and prepare, but do not start, the next."""
        arrived = self.mission_controller.active_target()
        if arrived is None:
            return

        self._completing_arrival = True
        try:
            next_waypoint = self.mission_controller.complete_active()
        except USER_ERRORS as error:
            self.mission_panel.report(
                f"Reached target could not be completed: {error}",
                ok=False,
            )
            return
        finally:
            self._completing_arrival = False

        if next_waypoint is None:
            text = f"Reached {arrived.name}; mission complete."
        else:
            text = (
                f"Reached {arrived.name}; {next_waypoint.name} is ready. "
                f"Press START to continue."
            )
        self.feedback.setText(text)
        self.mission_panel.report(text, ok=True)


def main(args=None):
    rclpy.init(args=args)
    node = OperatorGuiNode()
    app = QApplication(sys.argv)
    window = RoverConsole(node); window.show()
    try:
        code = app.exec_()
    finally:
        node.destroy_node(); rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
