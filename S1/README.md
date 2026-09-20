# Phase II S1 - Autonomous Navigation GUI

This directory contains the UMD Loop Fall 2026 Phase II S1 submission: a
custom PyQt and ROS 2 command-and-control interface for a simulated University
Rover Challenge autonomous-navigation mission.

The system accepts WGS 84 coordinates, converts them to a local REP-103 East-
North-Up frame, displays the rover and mission on an offline MDRS map, manages
GNSS waypoints, simulates autonomous travel through an obstacle course, handles
stale ROS data safely, and exports a structured mission log.

The implementation is a functional simulation prototype. It does not use Nav2,
Zenoh, a Gazebo terrain world, or physical rover hardware. See
[Unfinished objectives](#unfinished-objectives) for the exact scope.

## Completed functionality

- Coordinate entry in decimal degrees (DD), degrees-decimal-minutes (DDM), and
  degrees-minutes-seconds (DMS)
- Validation and visible rejection of invalid coordinates
- WGS 84 to local ENU conversion and inverse conversion
- Association of the supplied spawn coordinate with local `(0, 0)`
- Offline raster map of the Mars Desert Research Station area
- Rover position, heading, traveled path, destination, route, and mission state
- GNSS target creation through coordinate entry or a map click
- Target editing, deletion, reordering, activation, saving, and loading
- GUI controls for start, stop, abort, reset, and log export
- Simulated autonomous movement with a documented 1 m arrival tolerance
- Five-obstacle presentation course with multi-leg avoidance routing
- Missing and stale telemetry detection with a one-time safety stop
- Unreachable-target detection
- Exportable CSV log containing events, commands, conversions, and target results
- One-command ROS 2 launch for the complete demonstration

## Repository layout

```text
S1/
├── README.md
└── src/
    ├── urc_gui_phase2/          Coordinate, mission, map, planner, and rover sim
    │   ├── docs/OFFLINE_MAP.md  Offline-map construction and verification
    │   ├── tiles/mdrs/          699 committed local map tiles
    │   ├── tools/               Tile and offline-verification utilities
    │   └── test/                Unit and GUI integration tests
    └── urc-rover-console/       Combined operations console and launch file
        ├── launch/
        ├── test/
        └── urc_rover_console/
```

## Architecture

The project is divided into independently testable responsibilities:

- `battery_gui.py` provides the combined PyQt operator console.
- `mission_model.py` owns waypoint data, ordering, state, and JSON persistence.
- `mission_controller.py` connects mission operations to Qt signals without
  depending on map rendering or rover control.
- `coordinate_convert.py` parses DD/DDM/DMS input and converts WGS 84 to and
  from the local ENU frame.
- `map_widget.py` renders the offline map, rover track, targets, obstacle
  course, safety boundaries, and planned route.
- `rover_sim_node.py` owns the authoritative simulated position and navigation
  state.
- `obstacle_planner.py` calculates deterministic routes around separated
  circular obstacles.
- `telemetry_monitor.py` classifies each important ROS stream as waiting,
  active, or stale.
- `mission_event_log.py` records structured events and exports them to CSV.

ROS callbacks are processed by a bounded `SingleThreadedExecutor` from a Qt
timer. This keeps widget updates on the Qt main thread while limiting the number
of callbacks handled during each GUI cycle.

## ROS 2 interfaces

| Topic | Message type | Direction | Purpose |
| --- | --- | --- | --- |
| `/rover/fix` | `sensor_msgs/NavSatFix` | Rover sim to GUI | WGS 84 rover position |
| `/mission/active_target` | `geometry_msgs/PoseStamped` | GUI to rover sim | Active ENU target in the `map` frame |
| `/mission/status` | `std_msgs/String` | Rover sim to GUI | Authoritative mission state, route, and obstacles |
| `/operator/command` | `std_msgs/String` | GUI to simulators | Start, stop, abort, reset, or cancel |
| `/rover/telemetry` | `std_msgs/String` | Telemetry sim to GUI | Simulated rover and subsystem telemetry |

Position and target messages use ROS timestamps. The target uses the local
`map` frame and GNSS fixes use the `gps` frame. The prototype uses standard ROS
2 DDS communication with the default reliable queue-depth-10 configuration.

## Requirements

- Ubuntu 24.04 LTS
- ROS 2 Jazzy Jalisco desktop
- Python 3
- PyQt5 and PyQt WebEngine
- `pyproj`
- `pyqtlet2`
- `colcon`, `rosdep`, and standard ROS development tools

## Fresh VM installation

Source ROS and create a workspace:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/umd_loop_ws/src
cd ~/umd_loop_ws/src
```

Clone this repository from `main`:

```bash
git clone --branch main \
  https://github.com/andrewtsomik/UMD-Loop-Phase-II-Technology-Challenge.git
```

Install the system dependencies:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pyqt5 \
  python3-pyqt5.qtwebengine \
  python3-pyproj \
  python3-pip
```

Install `pyqtlet2`, which does not have a ROS dependency key:

```bash
python3 -m pip install --user --break-system-packages pyqtlet2
```

Install declared ROS dependencies and build both S1 packages:

```bash
cd ~/umd_loop_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select urc_gui_phase2 urc_rover_console
source install/setup.bash
```

## Updating an existing VM checkout

```bash
source /opt/ros/jazzy/setup.bash
cd ~/umd_loop_ws/src/UMD-Loop-Phase-II-Technology-Challenge
git switch main
git pull origin main

cd ~/umd_loop_ws
colcon build --symlink-install \
  --packages-select urc_gui_phase2 urc_rover_console
source install/setup.bash
```

## Running the system

Start the complete simulation and operator console:

```bash
ros2 launch urc_rover_console rover_console.launch.py
```

The presentation default is 8 m/s. Override it when needed:

```bash
ros2 launch urc_rover_console rover_console.launch.py speed_mps:=10.0
```

The obstacle course can be disabled for a direct-route demonstration:

```bash
ros2 launch urc_rover_console rover_console.launch.py obstacle_enabled:=false
```

For every new terminal, source ROS and the workspace before using ROS commands:

```bash
source /opt/ros/jazzy/setup.bash
source ~/umd_loop_ws/install/setup.bash
```

## Recommended presentation demonstration

The default spawn coordinate is:

```text
Latitude:  38.405800
Longitude: -110.791900
```

For a route that crosses the presentation obstacle course, create a target
approximately 50 m east of the spawn point:

```text
Latitude:  38.405800
Longitude: -110.791327
```

Suggested demonstration sequence:

1. Start the launch file and wait for all three data indicators to become active.
2. Show coordinate entry and conversion in DD, DDM, and DMS.
3. Enter an invalid coordinate and show that the GUI rejects it.
4. Add the suggested eastward target and activate it.
5. Point out the five obstacles, orange safety boundaries, and planned route.
6. Press **START** and show the `AVOIDING` state, rover heading, and traveled path.
7. Press **STOP**, then **START**, to demonstrate safe pause and resume.
8. Allow the rover to reach the target within the 1 m tolerance.
9. Demonstrate **RESET** or **ABORT**.
10. Export the mission log and show its commands, conversions, state changes,
    planner activity, and target result.

To demonstrate stale-data handling, stop one simulator during navigation. After
approximately 2.5 seconds, the GUI identifies the missing or stale stream and
sends a one-time safety stop.

## Mission states and commands

The coordinate rover reports these authoritative states:

| State | Meaning |
| --- | --- |
| `IDLE` | No active movement or target |
| `READY` | A valid target and route are available |
| `NAVIGATING` | Traveling on the final direct leg |
| `AVOIDING` | Following a multi-leg obstacle detour |
| `STOPPED` | Operator stopped movement; target is retained |
| `ARRIVED` | Rover is within the 1 m arrival tolerance |
| `ABORTED` | Mission was aborted and the target was cleared |
| `UNREACHABLE` | The obstacle planner rejected the target |

Supported operator commands are `START_MISSION`, `STOP_MISSION`,
`ABORT_MISSION`, `RESET_MISSION`, and `CANCEL_TARGET`.

## Offline map operation

The map uses committed local tiles under `urc_gui_phase2/tiles/mdrs`. The GUI
blocks nonlocal browser requests, so normal map operation does not require an
internet connection.

Detailed tile-building, attribution, verification, and coverage information is
available in [`src/urc_gui_phase2/docs/OFFLINE_MAP.md`](src/urc_gui_phase2/docs/OFFLINE_MAP.md).

OpenStreetMap data is copyright OpenStreetMap contributors and is available
under the Open Database License. Keep the displayed attribution when showing
or redistributing the map.

## Tests

After building and sourcing the workspace, run the package tests with:

```bash
cd ~/umd_loop_ws
colcon test --packages-select urc_gui_phase2 urc_rover_console
colcon test-result --verbose
```

The final non-ROS repository review ran 116 supported tests successfully, with
two environment-dependent tests skipped. A complete ROS, Qt, launch, and visual
test should still be performed inside the presentation VM.

## Safety and failure behavior

- A mission cannot start without an active target and fresh critical ROS data.
- Invalid fixes, target coordinates, route points, and status payloads are rejected.
- Missing or stale GNSS and mission-status streams trigger a one-time stop.
- Stop retains the current target so the mission can resume.
- Abort clears the target and route.
- Reset clears the route and returns the simulated rover to spawn.
- A target inside an obstacle safety boundary is reported as unreachable.
- Event logs use atomic replacement so an interrupted export does not leave a
  partially written final CSV.

## Unfinished objectives

The following Phase II objectives are not implemented and should be stated
explicitly during the presentation:

- Replacing Fast DDS with a ROS 2 Jazzy-compatible Zenoh setup
- Comparing Zenoh behavior and configuration with DDS
- Nav2 localization, global planning, and local planning
- A Gazebo simulation of the MDRS/Utah terrain
- USGS elevation-data conversion into an occupancy grid
- A complete ROS `tf2` transform tree
- Explicit per-topic QoS profiles and radio-network experiments
- Physical GNSS, IMU, motor-controller, emergency-stop, and radio integration
- Real-rover field testing

The current obstacle avoidance is a custom deterministic 2D demonstration for
separated circular obstacles. It is not a substitute for a perception-driven
Nav2 cost map or a production rover motion controller.

## Changes required for real hardware

A real deployment would replace the simulation nodes with hardware drivers and
add GNSS/IMU/odometry sensor fusion, calibrated transforms, physical motor and
E-stop interfaces, live obstacle sensing, a production navigation stack,
per-topic QoS policies, radio heartbeats, authentication, persistent onboard
logging, and field validation of coordinate accuracy and stopping distance.

## License

The `urc_rover_console` package is released under the MIT License. The
`urc_gui_phase2` package declares Apache-2.0. Offline map data attribution and
licensing are documented separately in the offline-map guide.
