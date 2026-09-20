"""Simulated rover: the state-producing half of the sim/GUI split.

Nothing is hardcoded: no default waypoints, no scripted path. The spawn point
is a parameter (spawn_lat_deg / spawn_lon_deg) and is also the origin of the
local ENU frame (see coordinate_convert.py). Without it the sim refuses to
start publishing rather than invent a site.

Publishes sensor_msgs/NavSatFix on /rover/fix and authoritative JSON mission
state on /mission/status. Subscribes to the operator's active target
(geometry_msgs/PoseStamped, ENU metres, frame 'map') and drives toward it at
`speed_mps`. A configurable course of circular local-frame obstacles is
inflated by a safety clearance; blocked paths receive a multi-leg detour.
"""

import json
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from urc_gui_phase2.coordinate_convert import LocalFrame
from urc_gui_phase2.obstacle_planner import (
    CircleObstacle,
    UnreachableTargetError,
    plan_route_around_obstacles,
    route_length,
)

PERIOD_S = 0.2
ARRIVAL_RADIUS_M = 1.0
MISSION_STATUS_TOPIC = '/mission/status'


class RoverSimNode(Node):
    def __init__(self):
        super().__init__('rover_sim_node')
        self.declare_parameter('spawn_lat_deg', float('nan'))
        self.declare_parameter('spawn_lon_deg', float('nan'))
        self.declare_parameter('speed_mps', 1.0)
        self.declare_parameter('obstacle_enabled', True)
        self.declare_parameter('obstacle_east_m', 8.0)
        self.declare_parameter('obstacle_north_m', 0.0)
        self.declare_parameter('obstacle_radius_m', 3.0)
        self.declare_parameter('obstacle_clearance_m', 2.0)
        self.declare_parameter('obstacle_2_east_m', 16.0)
        self.declare_parameter('obstacle_2_north_m', 5.0)
        self.declare_parameter('obstacle_2_radius_m', 2.5)
        self.declare_parameter('obstacle_3_east_m', 24.0)
        self.declare_parameter('obstacle_3_north_m', -4.0)
        self.declare_parameter('obstacle_3_radius_m', 2.8)
        self.declare_parameter('obstacle_4_east_m', 32.0)
        self.declare_parameter('obstacle_4_north_m', 4.0)
        self.declare_parameter('obstacle_4_radius_m', 2.4)
        self.declare_parameter('obstacle_5_east_m', 40.0)
        self.declare_parameter('obstacle_5_north_m', -3.0)
        self.declare_parameter('obstacle_5_radius_m', 2.6)
        lat = self.get_parameter('spawn_lat_deg').value
        lon = self.get_parameter('spawn_lon_deg').value
        self._speed = float(self.get_parameter('speed_mps').value)
        self._east = self._north = 0.0
        self._target = None
        self._route = []
        self._avoidance_planned = False
        self._planning_error = None
        self._motion_enabled = False
        self._state = 'IDLE'
        self._frame = None
        if math.isnan(lat) or math.isnan(lon):
            self.get_logger().error('spawn_lat_deg/spawn_lon_deg not set; publishing nothing')
            return
        if not math.isfinite(self._speed) or self._speed <= 0.0:
            self.get_logger().error(
                'speed_mps must be a positive finite number; publishing nothing'
            )
            return
        self._obstacle_clearance = float(
            self.get_parameter('obstacle_clearance_m').value
        )
        self._obstacles = []
        if self.get_parameter('obstacle_enabled').value:
            try:
                obstacle_parameters = [
                    'obstacle',
                    'obstacle_2',
                    'obstacle_3',
                    'obstacle_4',
                    'obstacle_5',
                ]
                for prefix in obstacle_parameters:
                    self._obstacles.append(CircleObstacle(
                        self.get_parameter(f'{prefix}_east_m').value,
                        self.get_parameter(f'{prefix}_north_m').value,
                        self.get_parameter(f'{prefix}_radius_m').value,
                    ))
                if (
                    not math.isfinite(self._obstacle_clearance)
                    or self._obstacle_clearance < 0.0
                ):
                    raise ValueError(
                        'obstacle clearance must be finite and non-negative'
                    )
            except (TypeError, ValueError) as error:
                self.get_logger().error(f'Invalid obstacle configuration: {error}')
                return
        self._frame = LocalFrame(lat, lon)
        self._pub = self.create_publisher(NavSatFix, '/rover/fix', 10)
        # This status is authoritative because it is calculated by the same
        # node that owns the rover position and applies movement commands.
        self._status_pub = self.create_publisher(
            String,
            MISSION_STATUS_TOPIC,
            10,
        )
        self.create_subscription(PoseStamped, '/mission/active_target', self._on_target, 10)
        self.create_subscription(
            String,
            '/operator/command',
            self._on_command,
            10,
        )
        self.create_timer(PERIOD_S, self._tick)

    def _on_target(self, msg: PoseStamped) -> None:
        """Store a destination without automatically starting motion."""
        if msg.header.frame_id != 'map':
            self.get_logger().warning(
                f'Ignoring target in unexpected frame '
                f'{msg.header.frame_id!r}'
            )
            return

        east_m = msg.pose.position.x
        north_m = msg.pose.position.y
        if not (math.isfinite(east_m) and math.isfinite(north_m)):
            self.get_logger().warning(
                'Ignoring target with non-finite map coordinates'
            )
            return

        goal = (east_m, north_m)
        try:
            route = (
                (goal,) if not self._obstacles else
                plan_route_around_obstacles(
                    (self._east, self._north),
                    goal,
                    self._obstacles,
                    self._obstacle_clearance,
                )
            )
        except UnreachableTargetError as error:
            self._motion_enabled = False
            self._target = None
            self._route = []
            self._avoidance_planned = False
            self._planning_error = str(error)
            self._state = 'UNREACHABLE'
            self.get_logger().warning(
                f'Target rejected by obstacle planner: {error}'
            )
            return

        self._target = goal
        self._route = list(route)
        self._avoidance_planned = len(route) > 1
        self._planning_error = None
        if self._motion_enabled:
            self._state = (
                'AVOIDING' if self._avoidance_planned else 'NAVIGATING'
            )
        else:
            self._state = 'READY'

        self.get_logger().info(
            f'Target received: east={self._target[0]:.2f} m, '
            f'north={self._target[1]:.2f} m; '
            f'route points={len(self._route)}'
        )

    def _on_command(self, msg: String) -> None:
        """Apply an operator command to the rover's motion state."""
        command = msg.data

        if command == 'START_MISSION':
            if self._target is None:
                self._motion_enabled = False
                self._state = (
                    'UNREACHABLE' if self._planning_error else 'IDLE'
                )
                self.get_logger().warning(
                    'Start ignored because no target is assigned'
                )
            else:
                self._motion_enabled = True
                self._state = (
                    'AVOIDING' if len(self._route) > 1 else 'NAVIGATING'
                )
                self.get_logger().info('Motion enabled')

        elif command == 'STOP_MISSION':
            self._motion_enabled = False
            self._state = 'STOPPED' if self._target is not None else 'IDLE'
            self.get_logger().info(
                'Rover stopped; target retained'
            )

        elif command == 'ABORT_MISSION':
            self._motion_enabled = False
            self._target = None
            self._route = []
            self._avoidance_planned = False
            self._planning_error = None
            self._state = 'ABORTED'
            self.get_logger().warning(
                'Mission aborted; target cleared'
            )

        elif command == 'RESET_MISSION':
            self._motion_enabled = False
            self._target = None
            self._route = []
            self._avoidance_planned = False
            self._planning_error = None
            self._east = 0.0
            self._north = 0.0
            self._state = 'IDLE'
            self.get_logger().info(
                'Rover reset to spawn position'
            )

        elif command == 'CANCEL_TARGET':
            self._motion_enabled = False
            self._target = None
            self._route = []
            self._avoidance_planned = False
            self._planning_error = None
            self._state = 'IDLE'
            self.get_logger().info(
                'Active target cancelled'
            )

    def _tick(self) -> None:
        if self._motion_enabled and self._target is not None and self._route:
            route_point = self._route[0]
            de = route_point[0] - self._east
            dn = route_point[1] - self._north
            dist = math.hypot(de, dn)
            final_leg = len(self._route) == 1

            if final_leg and dist <= ARRIVAL_RADIUS_M:
                self._motion_enabled = False
                self._route = []
                self._state = 'ARRIVED'
                self.get_logger().info(
                    'Target reached within '
                    f'{ARRIVAL_RADIUS_M:.1f} m tolerance'
                )
            else:
                step = min(self._speed * PERIOD_S, dist)
                if dist <= step:
                    # Snap exactly to an intermediate planner waypoint, then
                    # continue toward the next route point on the next tick.
                    self._east, self._north = route_point
                    self._route.pop(0)
                    if not self._route:
                        self._motion_enabled = False
                        self._state = 'ARRIVED'
                    elif len(self._route) == 1:
                        self._state = 'NAVIGATING'
                elif dist > 0.0:
                    self._east += de / dist * step
                    self._north += dn / dist * step
                    self._state = (
                        'AVOIDING' if len(self._route) > 1 else 'NAVIGATING'
                    )

        geo = self._frame.to_wgs84(self._east, self._north)
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.latitude, msg.longitude, msg.altitude = geo.lat_deg, geo.lon_deg, geo.alt_m
        self._pub.publish(msg)

        # JSON keeps this small status channel independent of the GUI while
        # still making every field explicit and easy to inspect with ros2 topic.
        distance_m = None
        if self._target is not None:
            if self._route:
                distance_m = route_length(
                    (self._east, self._north),
                    self._route,
                )
            else:
                distance_m = math.hypot(
                    self._target[0] - self._east,
                    self._target[1] - self._north,
                )
        obstacles_data = [
            {
                'east_m': obstacle.east_m,
                'north_m': obstacle.north_m,
                'radius_m': obstacle.radius_m,
                'clearance_m': self._obstacle_clearance,
            }
            for obstacle in self._obstacles
        ]
        status = String()
        status.data = json.dumps(
            {
                'state': self._state,
                'has_target': self._target is not None,
                'distance_m': (
                    None if distance_m is None else round(distance_m, 2)
                ),
                'avoidance_planned': self._avoidance_planned,
                'route': [
                    {'east_m': east, 'north_m': north}
                    for east, north in self._route
                ],
                'obstacle': obstacles_data[0] if obstacles_data else None,
                'obstacles': obstacles_data,
                'planning_error': self._planning_error,
            }
        )
        self._status_pub.publish(status)


def main():
    rclpy.init()
    node = RoverSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
