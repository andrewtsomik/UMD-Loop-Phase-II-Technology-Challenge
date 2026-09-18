from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="urc_rover_console",
                executable="rover_simulator",
                name="rover_simulator",
                output="screen",
            ),
            Node(
                package="urc_rover_console",
                executable="battery_gui",
                name="rover_operations_console",
                output="screen",
            ),
        ]
    )
