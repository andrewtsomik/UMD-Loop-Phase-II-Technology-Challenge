import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication, QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QListWidget, QMainWindow, QProgressBar, QPushButton, QScrollArea,
    QVBoxLayout, QWidget,
)


def value_label(text="--"):
    label = QLabel(text)
    label.setObjectName("value")
    return label


def add_row(layout, row, name, widget):
    layout.addWidget(QLabel(name), row, 0)
    layout.addWidget(widget, row, 1)


class RoverConsole(QMainWindow):
    TARGETS = ["GNSS Location 1", "GNSS Location 2", "ArUco Post 1",
               "ArUco Post 2", "Orange Mallet", "Rock Pick Hammer", "Water Bottle"]

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.last_telemetry = None
        self.start_time = time.monotonic()
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

    def panel(self, title):
        box = QGroupBox(title)
        grid = QGridLayout(box)
        grid.setColumnStretch(1, 1)
        return box, grid

    def build_ui(self):
        page = QWidget()
        root = QVBoxLayout(page)

        header = QHBoxLayout()
        title = QLabel("AUTONOMOUS NAVIGATION — ROVER OPERATIONS CONSOLE")
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

        self.banner = QLabel("WAITING FOR ROVER STATUS")
        self.banner.setObjectName("banner")
        self.banner.setAlignment(Qt.AlignCenter)
        root.addWidget(self.banner)

        body = QGridLayout()
        root.addLayout(body, 1)

        mission_box, mission = self.panel("MISSION PROGRESS")
        self.targets = QListWidget()
        self.targets.addItems(self.TARGETS)
        mission.addWidget(self.targets, 0, 0, 1, 2)
        self.target = value_label("--")
        self.nav_state = value_label("--")
        self.distance = value_label("--")
        self.accuracy = value_label("--")
        add_row(mission, 1, "Current target", self.target)
        add_row(mission, 2, "Rover state", self.nav_state)
        add_row(mission, 3, "Distance", self.distance)
        add_row(mission, 4, "GNSS accuracy", self.accuracy)
        body.addWidget(mission_box, 0, 0, 2, 1)

        power_box, power = self.panel("POWER AND ENVIRONMENT")
        self.battery = QProgressBar(); self.battery.setRange(0, 100)
        self.voltage = value_label(); self.current = value_label(); self.runtime = value_label()
        self.temperature = value_label(); self.humidity = value_label()
        power.addWidget(QLabel("Battery"), 0, 0); power.addWidget(self.battery, 0, 1)
        add_row(power, 1, "Pack voltage", self.voltage)
        add_row(power, 2, "Current draw", self.current)
        add_row(power, 3, "Estimated runtime", self.runtime)
        add_row(power, 4, "Electronics temperature", self.temperature)
        add_row(power, 5, "Humidity", self.humidity)
        body.addWidget(power_box, 0, 1)

        comms_box, comms = self.panel("COMMS AND LINK HEALTH")
        self.signal = value_label(); self.packet_loss = value_label(); self.latency = value_label()
        self.topic_count = value_label()
        add_row(comms, 0, "Signal quality", self.signal)
        add_row(comms, 1, "Packet loss", self.packet_loss)
        add_row(comms, 2, "Round-trip latency", self.latency)
        add_row(comms, 3, "Telemetry topics", self.topic_count)
        body.addWidget(comms_box, 0, 2)

        safety_box, safety = self.panel("SAFETY AND STABILITY")
        self.pitch = value_label(); self.roll = value_label(); self.yaw = value_label()
        self.acceleration = value_label(); self.estop = value_label()
        add_row(safety, 0, "Front-to-back tilt", self.pitch)
        add_row(safety, 1, "Left-to-right tilt", self.roll)
        add_row(safety, 2, "IMU yaw rate", self.yaw)
        add_row(safety, 3, "Acceleration", self.acceleration)
        add_row(safety, 4, "E-stop circuit", self.estop)
        body.addWidget(safety_box, 0, 3)

        mobility_box = QGroupBox("MOBILITY DIAGNOSTICS — WHEELS AND STEERING")
        mobility = QHBoxLayout(mobility_box)
        self.wheels = {}
        for key, name in [("front_left", "FRONT LEFT"), ("front_right", "FRONT RIGHT"),
                          ("rear_left", "REAR LEFT"), ("rear_right", "REAR RIGHT")]:
            wheel_box, wheel = self.panel(name)
            fields = [value_label() for _ in range(4)]
            for row, (label, field) in enumerate(zip(
                    ["Velocity", "Current", "Steering", "Temperature"], fields)):
                add_row(wheel, row, label, field)
            self.wheels[key] = fields
            mobility.addWidget(wheel_box)
        body.addWidget(mobility_box, 1, 1, 1, 2)

        subsystem_box, subsystem = self.panel("SUBSYSTEM CHECKS")
        self.subsystems = {}
        for row, (key, name) in enumerate([
                ("ros_link", "ROS link"), ("power_bus", "Power bus"),
                ("gnss_receiver", "GNSS receiver"), ("mobility_motors", "Mobility motors"),
                ("thermal_state", "Thermal state")]):
            field = value_label(); self.subsystems[key] = field
            add_row(subsystem, row, name, field)
        body.addWidget(subsystem_box, 1, 3)

        commands = QHBoxLayout()
        self.feedback = QLabel("No operator command sent")
        start = QPushButton("START NEXT TARGET")
        abort = QPushButton("ABORT AND RETURN"); abort.setObjectName("danger")
        start.clicked.connect(lambda: self.send_command("START_NEXT_TARGET"))
        abort.clicked.connect(lambda: self.send_command("ABORT_AND_RETURN"))
        commands.addWidget(self.feedback, 1); commands.addWidget(start); commands.addWidget(abort)
        root.addLayout(commands)

        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(page)
        self.setCentralWidget(scroll)
        self.setStyleSheet("""
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
        """)

    def send_command(self, command):
        message = String(); message.data = command
        self.command_publisher.publish(message)
        self.feedback.setText(f"Command sent: {command.replace('_', ' ')}")

    def update_clock_and_link(self):
        elapsed = int(time.monotonic() - self.start_time)
        self.clock.setText(f"{elapsed // 60:02d}:{elapsed % 60:02d}")
        if self.last_telemetry is None:
            return
        age = time.monotonic() - self.last_telemetry
        if age <= 2.5:
            self.link.setObjectName("active"); self.link.setText(f"LINK ACTIVE · {age:.1f}s")
        else:
            self.link.setObjectName("stale"); self.link.setText(f"TELEMETRY STALE · {age:.1f}s")
        self.link.style().unpolish(self.link); self.link.style().polish(self.link)

    def update_telemetry(self, message):
        self.last_telemetry = time.monotonic()
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            self.link.setText("INVALID TELEMETRY MESSAGE")
            return
        nav, power, comms = data["navigation"], data["power"], data["comms"]
        safety, mobility = data["safety"], data["mobility"]
        self.target.setText(nav["target"]); self.nav_state.setText(nav["state"])
        self.distance.setText(f'{nav["distance_m"]:.1f} m')
        self.accuracy.setText(f'±{nav["gnss_accuracy_m"]:.1f} m')
        completed = nav["completed_targets"]
        for i in range(self.targets.count()):
            prefix = "✓ " if i < completed else ("▶ " if i == completed else "○ ")
            self.targets.item(i).setText(prefix + self.TARGETS[i])
        self.banner.setText("TARGET REACHED — ROVER STOPPED" if nav["state"] == "ARRIVED"
                            else f'{nav["state"]} — {nav["target"]}')
        self.battery.setValue(round(power["battery_percent"]))
        self.battery.setFormat(f'{power["battery_percent"]:.1f}%')
        self.voltage.setText(f'{power["pack_voltage_v"]:.2f} V')
        self.current.setText(f'{power["current_a"]:.2f} A')
        self.runtime.setText(f'{power["runtime_minutes"]} min')
        self.temperature.setText(f'{power["electronics_temp_c"]:.1f} °C')
        self.humidity.setText(f'{power["humidity_percent"]:.1f}%')
        self.signal.setText(f'{comms["signal_quality_percent"]:.1f}%')
        self.packet_loss.setText(f'{comms["packet_loss_percent"]:.1f}%')
        self.latency.setText(f'{comms["latency_ms"]:.0f} ms')
        self.topic_count.setText(f'{comms["topics_received"]} / {comms["topics_expected"]}')
        self.pitch.setText(f'{safety["pitch_deg"]:.1f}° · SAFE')
        self.roll.setText(f'{safety["roll_deg"]:.1f}° · SAFE')
        self.yaw.setText(f'{safety["yaw_rate_deg_s"]:.1f}°/s')
        self.acceleration.setText(f'{safety["acceleration_m_s2"]:.2f} m/s²')
        self.estop.setText(safety["estop_status"])
        for key, fields in self.wheels.items():
            wheel = mobility[key]
            values = [f'{wheel["velocity_m_s"]:.2f} m/s', f'{wheel["current_a"]:.1f} A',
                      f'{wheel["steering_deg"]:.1f}°', f'{wheel["temperature_c"]:.1f} °C']
            for field, value in zip(fields, values): field.setText(value)
        for key, field in self.subsystems.items(): field.setText(data["subsystems"][key])
        if nav["state"] == "RETURNING":
            self.feedback.setText("Rover acknowledged abort and is returning")


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
