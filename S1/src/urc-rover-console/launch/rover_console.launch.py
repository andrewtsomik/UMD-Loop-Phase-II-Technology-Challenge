from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    spawn_latitude = ParameterValue(
        LaunchConfiguration("spawn_lat_deg"),
        value_type=float,
    )

    spawn_longitude = ParameterValue(
        LaunchConfiguration("spawn_lon_deg"),
        value_type=float,
    )

    obstacle_enabled = ParameterValue(
        LaunchConfiguration("obstacle_enabled"),
        value_type=bool,
    )

    obstacle_east = ParameterValue(
        LaunchConfiguration("obstacle_east_m"),
        value_type=float,
    )

    obstacle_north = ParameterValue(
        LaunchConfiguration("obstacle_north_m"),
        value_type=float,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "spawn_lat_deg",
                default_value="38.4058",
                description="WGS 84 latitude associated with rover spawn",
            ),
            DeclareLaunchArgument(
                "spawn_lon_deg",
                default_value="-110.7919",
                description="WGS 84 longitude associated with rover spawn",
            ),
            DeclareLaunchArgument(
                "obstacle_enabled",
                default_value="true",
                description="Enable the simulated circular obstacle",
            ),
            DeclareLaunchArgument(
                "obstacle_east_m",
                default_value="15.0",
                description="Obstacle center east of spawn in REP 103 metres",
            ),
            DeclareLaunchArgument(
                "obstacle_north_m",
                default_value="0.0",
                description="Obstacle center north of spawn in REP 103 metres",
            ),
            Node(
                package="urc_rover_console",
                executable="rover_simulator",
                name="telemetry_simulator",
                output="screen",
            ),
            Node(
                package="urc_gui_phase2",
                executable="rover_sim_node",
                name="coordinate_rover_simulator",
                output="screen",
                parameters=[
                    {
                        "spawn_lat_deg": spawn_latitude,
                        "spawn_lon_deg": spawn_longitude,
                        "speed_mps": 3.0,
                        "obstacle_enabled": obstacle_enabled,
                        "obstacle_east_m": obstacle_east,
                        "obstacle_north_m": obstacle_north,
                        "obstacle_radius_m": 3.0,
                        "obstacle_clearance_m": 2.0,
                    }
                ],
            ),
            Node(
                package="urc_rover_console",
                executable="battery_gui",
                name="rover_operations_console",
                output="screen",
                parameters=[
                    {
                        "spawn_lat_deg": spawn_latitude,
                        "spawn_lon_deg": spawn_longitude,
                    }
                ],
            ),
        ]
    )
