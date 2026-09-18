# URC Rover Operations Console

A ROS 2 and PyQt operator dashboard prototype for the 2026 University Rover Challenge Autonomous Navigation Mission. The project demonstrates live rover telemetry, mission-state monitoring, operator commands, and visible communication-failure handling.

This project was created for the UMD Loop Fall 2026 Phase 1 Software Challenge S2.

## Features

- Live navigation state, target, distance, and GNSS accuracy
- Mission progress across seven URC autonomous-navigation targets
- Battery, voltage, current, runtime, temperature, and humidity monitoring
- Signal quality, packet loss, latency, and telemetry-topic status
- Rover pitch, roll, yaw rate, acceleration, and E-stop status
- Per-wheel velocity, current, steering angle, and temperature
- Subsystem health checks
- Mission timer and prominent navigation-state banner
- Operator commands for starting the next target and aborting an attempt
- Visible stale-telemetry warning with automatic recovery
- One-command startup through a ROS 2 launch file

## System Architecture

The package contains two ROS 2 nodes:

1. `rover_simulator` generates simulated rover telemetry and responds to operator commands.
2. `battery_gui` runs the PyQt rover operations console, displays telemetry, and publishes commands.

The simulator publishes a JSON-formatted `std_msgs/String` message for rapid prototyping. A production system would normally use separate strongly typed ROS 2 interfaces for validation, fault isolation, and independent update rates.

## ROS 2 Interfaces

| Topic | Message type | Direction | Purpose |
| --- | --- | --- | --- |
| `/rover/telemetry` | `std_msgs/String` | Simulator to GUI | Combined navigation and rover-health telemetry |
| `/rover/battery_percentage` | `std_msgs/Float32` | Simulator to subscribers | Battery topic retained from the first prototype |
| `/rover/navigation_state` | `std_msgs/String` | Simulator to subscribers | Navigation state retained from the communication test |
| `/operator/command` | `std_msgs/String` | GUI to simulator | `START_NEXT_TARGET` and `ABORT_AND_RETURN` commands |

## Requirements

- Ubuntu 24.04 LTS
- ROS 2 Jazzy Jalisco desktop installation
- Python 3
- PyQt5
- `colcon` and the standard ROS 2 development tools

Install PyQt5 if it is not already available:

```bash
sudo apt update
sudo apt install python3-pyqt5 -y
```

## Installation

Create a ROS 2 workspace and clone the repository into its `src` directory:

```bash
mkdir -p ~/umd_loop_ws/src
cd ~/umd_loop_ws/src
git clone https://github.com/nmaisheri/urc-rover-console.git
```

Install declared dependencies:

```bash
cd ~/umd_loop_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Build and load the workspace:

```bash
colcon build --symlink-install --packages-select urc_rover_console
source install/setup.bash
```

## Running the Project

Start both the simulator and dashboard:

```bash
ros2 launch urc_rover_console rover_console.launch.py
```

The dashboard should open automatically and begin displaying simulated telemetry. Press `Ctrl+C` in the launch terminal to stop both nodes.

## Demonstration Tests

### Operator command

1. Start the system using the launch command.
2. Confirm that the rover state is `NAVIGATING`.
3. Select **Abort and Return**.
4. Confirm that the simulator receives `ABORT_AND_RETURN`.
5. Confirm that the dashboard changes to `RETURNING`.

### Stale telemetry

1. Start the system and confirm that the link indicator is active.
2. Stop the launch process or simulator.
3. Confirm that the dashboard reports stale telemetry after approximately 2.5 seconds.
4. Restart the simulator and confirm that the active indication returns automatically.

## Current Limitations

- All rover and environmental measurements are simulated.
- The combined telemetry message uses JSON rather than custom ROS 2 message definitions.
- The stale-data monitor currently evaluates the combined telemetry stream instead of tracking each subsystem independently.
- The prototype does not yet connect to physical rover hardware or Gazebo.
- The simulator models mission behavior for interface testing rather than realistic rover dynamics.

## License

This project is released under the MIT License.
