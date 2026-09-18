import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RoverSimulator(Node):
    def __init__(self):
        super().__init__("rover_simulator")

        self.battery_percentage = 100.0
        self.navigation_state = "NAVIGATING"
        self.current_target = "GNSS Target 1"
        self.distance_to_target = 24.0
        self.completed_targets = 0
        self.tick = 0


        # This topic contains the complete dashboard telemetry.
        self.telemetry_publisher = self.create_publisher(
            String,
            "/rover/telemetry",
            10,
        )

        self.command_subscription = self.create_subscription(
            String,
            "/operator/command",
            self.receive_command,
            10,
        )

        self.timer = self.create_timer(1.0, self.publish_telemetry)
        self.get_logger().info("Rover simulator started")

    def publish_telemetry(self):
        self.tick += 1

        if self.navigation_state == "NAVIGATING":
            self.distance_to_target = max(
                0.0,
                self.distance_to_target - 0.8,
            )

            if self.distance_to_target == 0.0:
                self.navigation_state = "ARRIVED"

        elif self.navigation_state == "RETURNING":
            self.distance_to_target = max(
                0.0,
                self.distance_to_target - 1.0,
            )

            if self.distance_to_target == 0.0:
                self.navigation_state = "STOPPED"

        movement = math.sin(self.tick / 3.0)

        telemetry = {
            "navigation": {
                "state": self.navigation_state,
                "target": self.current_target,
                "distance_m": round(self.distance_to_target, 1),
                "gnss_accuracy_m": 0.7,
                "completed_targets": self.completed_targets,
                "total_targets": 3,
            },
            "power": {
                "battery_percent": round(
                    self.battery_percentage,
                    1,
                ),
                "pack_voltage_v": round(
                    22.0
                    + (self.battery_percentage / 100.0) * 3.4,
                    2,
                ),
                "current_a": round(5.8 + abs(movement), 2),
                "runtime_minutes": round(
                    self.battery_percentage * 0.6,
                ),
                "electronics_temp_c": round(
                    41.0 + abs(movement),
                    1,
                ),
                "humidity_percent": 18.0,
            },
            "comms": {
                "signal_quality_percent": round(
                    78.0 + movement * 4.0,
                    1,
                ),
                "packet_loss_percent": round(
                    1.2 + abs(movement) * 0.4,
                    1,
                ),
                "latency_ms": round(
                    84.0 + abs(movement) * 12.0,
                ),
                "topics_received": 5,
                "topics_expected": 5,
            },
            "safety": {
                "pitch_deg": round(2.1 + movement, 1),
                "roll_deg": round(1.3 - movement / 2.0, 1),
                "yaw_rate_deg_s": round(
                    4.8 + movement,
                    1,
                ),
                "acceleration_m_s2": round(
                    0.18 + abs(movement) / 10.0,
                    2,
                ),
                "estop_status": "ARMED",
            },
            "mobility": {
                "front_left": {
                    "velocity_m_s": round(1.17 + movement / 20, 2),
                    "current_a": 2.3,
                    "steering_deg": 4.6,
                    "temperature_c": 39.0,
                },
                "front_right": {
                    "velocity_m_s": round(1.20 - movement / 20, 2),
                    "current_a": 2.2,
                    "steering_deg": 4.4,
                    "temperature_c": 38.0,
                },
                "rear_left": {
                    "velocity_m_s": round(1.16 + movement / 20, 2),
                    "current_a": 2.4,
                    "steering_deg": 4.7,
                    "temperature_c": 40.0,
                },
                "rear_right": {
                    "velocity_m_s": round(1.19 - movement / 20, 2),
                    "current_a": 2.3,
                    "steering_deg": 4.5,
                    "temperature_c": 39.0,
                },
            },
            "subsystems": {
                "ros_link": "PASS",
                "power_bus": "PASS",
                "gnss_receiver": "PASS",
                "mobility_motors": "PASS",
                "thermal_state": "PASS",
            },
        }

        telemetry_message = String()
        telemetry_message.data = json.dumps(telemetry)
        self.telemetry_publisher.publish(telemetry_message)

        self.get_logger().info(
            f"Battery: {self.battery_percentage:.1f}% | "
            f"State: {self.navigation_state} | "
            f"Distance: {self.distance_to_target:.1f} m"
        )

        self.battery_percentage -= 0.5

        if self.battery_percentage < 20.0:
            self.battery_percentage = 100.0

    def receive_command(self, message):
        self.get_logger().info(
            f"Command received: {message.data}"
        )

        if message.data == "ABORT_AND_RETURN":
            self.navigation_state = "RETURNING"
            self.distance_to_target = 15.0
            self.get_logger().warning(
                "Attempt aborted. Returning to previous target."
            )

        elif message.data == "START_NEXT_TARGET":
            self.navigation_state = "NAVIGATING"
            self.distance_to_target = 24.0
            self.get_logger().info(
                "Navigation to the next target started."
            )


def main(args=None):
    rclpy.init(args=args)
    node = RoverSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
