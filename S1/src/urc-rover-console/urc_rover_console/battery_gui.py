import json
import sys
import time

import rclpy
from rclpy.node import Node
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


class RoverConsole(QMainWindow):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.telemetry_monitor = TelemetryMonitor(stale_after_seconds=2.5)
        self.start_time = time.monotonic()
        self.mission_controller = MissionController()
        self.setWindowTitle("URC Autonomous Navigation Rover Operations Console")
        self.resize(1500, 900)
        self.build_ui()

        self.subscription = node.create_subscription(
            String, "/rover/telemetry", self.update_telemetry, 10)
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
        message = String(); message.data = command
        self.command_publisher.publish(message)
        self.feedback.setText(f"Command sent: {command.replace('_', ' ')}")

    def update_clock_and_link(self):
        elapsed = int(time.monotonic() - self.start_time)
        self.clock.setText(f"{elapsed // 60:02d}:{elapsed % 60:02d}")

        state, age = self.telemetry_monitor.connection_state()

        if state == "waiting":
            self.link.setObjectName("waiting")
            self.link.setText("WAITING FOR TELEMETRY")
        elif state == "active":
            self.link.setObjectName("active")
            self.link.setText(f"LINK ACTIVE · {age:.1f}s")
        else:
            self.link.setObjectName("stale")
            self.link.setText(f"TELEMETRY STALE · {age:.1f}s")

        self.link.style().unpolish(self.link)
        self.link.style().polish(self.link)

    def update_telemetry(self, message):
        self.telemetry_monitor.mark_received()

        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            self.link.setText("INVALID TELEMETRY MESSAGE")
            return

        try:
            nav = data["navigation"]

            target = nav["target"]
            state = nav["state"]
            distance = nav["distance_m"]
            accuracy = nav["gnss_accuracy_m"]
        except (KeyError, TypeError):
            self.link.setText("INCOMPLETE TELEMETRY MESSAGE")
            return

        self.target.setText(f"Target: {target}")
        self.nav_state.setText(f"State: {state}")
        self.distance.setText(f"Distance: {distance:.1f} m")
        self.accuracy.setText(f"GNSS accuracy: ±{accuracy:.1f} m")

        if state == "ARRIVED":
            self.banner.setText("TARGET REACHED — ROVER STOPPED")
        else:
            self.banner.setText(f"{state} — {target}")

        if state in ("RETURNING", "ABORTED"):
            self.feedback.setText(
                "Rover acknowledged abort command"
            )


def main(args=None):
    rclpy.init(args=args)
    node = Node("rover_operations_console")
    app = QApplication(sys.argv)
    window = RoverConsole(node); window.show()
    try:
        code = app.exec_()
    finally:
        node.destroy_node(); rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
