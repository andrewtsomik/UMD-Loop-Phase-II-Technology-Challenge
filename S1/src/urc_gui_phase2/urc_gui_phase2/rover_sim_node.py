"""Simulated rover: the state-producing half of the sim/GUI split.

Nothing is hardcoded: no default waypoints, no scripted path. The spawn point
is a parameter (spawn_lat_deg / spawn_lon_deg) and is also the origin of the
local ENU frame (see coordinate_convert.py). Without it the sim refuses to
start publishing rather than invent a site.

Publishes sensor_msgs/NavSatFix on /rover/fix and authoritative JSON mission
state on /mission/status. Subscribes to the operator's active target
(geometry_msgs/PoseStamped, ENU metres, frame 'map') and drives straight
toward it at `speed_mps`, stopping on arrival.
"""

import json
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from urc_gui_phase2.coordinate_convert import LocalFrame

PERIOD_S = 0.2
ARRIVAL_RADIUS_M = 1.0
MISSION_STATUS_TOPIC = '/mission/status'


class RoverSimNode(Node):
    def __init__(self):
        super().__init__('rover_sim_node')
        self.declare_parameter('spawn_lat_deg', float('nan'))
        self.declare_parameter('spawn_lon_deg', float('nan'))
        self.declare_parameter('speed_mps', 1.0)
        lat = self.get_parameter('spawn_lat_deg').value
        lon = self.get_parameter('spawn_lon_deg').value
        self._speed = float(self.get_parameter('speed_mps').value)
        self._east = self._north = 0.0
        self._target = None
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

        self._target = (east_m, north_m)
        if not self._motion_enabled:
            self._state = 'READY'

        self.get_logger().info(
            f'Target received: east={self._target[0]:.2f} m, '
            f'north={self._target[1]:.2f} m'
        )

    def _on_command(self, msg: String) -> None:
        """Apply an operator command to the rover's motion state."""
        command = msg.data

        if command == 'START_MISSION':
            if self._target is None:
                self._motion_enabled = False
                self._state = 'IDLE'
                self.get_logger().warning(
                    'Start ignored because no target is assigned'
                )
            else:
                self._motion_enabled = True
                self._state = 'NAVIGATING'
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
            self._state = 'ABORTED'
            self.get_logger().warning(
                'Mission aborted; target cleared'
            )

        elif command == 'RESET_MISSION':
            self._motion_enabled = False
            self._target = None
            self._east = 0.0
            self._north = 0.0
            self._state = 'IDLE'
            self.get_logger().info(
                'Rover reset to spawn position'
            )

        elif command == 'CANCEL_TARGET':
            self._motion_enabled = False
            self._target = None
            self._state = 'IDLE'
            self.get_logger().info(
                'Active target cancelled'
            )

    def _tick(self) -> None:
        if self._motion_enabled and self._target is not None:
            de = self._target[0] - self._east
            dn = self._target[1] - self._north
            dist = math.hypot(de, dn)

            if dist > ARRIVAL_RADIUS_M:
                step = min(self._speed * PERIOD_S, dist)
                self._east += de / dist * step
                self._north += dn / dist * step
            else:
                self._motion_enabled = False
                self._state = 'ARRIVED'
                self.get_logger().info(
                    'Target reached within '
                    f'{ARRIVAL_RADIUS_M:.1f} m tolerance'
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
            distance_m = math.hypot(
                self._target[0] - self._east,
                self._target[1] - self._north,
            )
        status = String()
        status.data = json.dumps(
            {
                'state': self._state,
                'has_target': self._target is not None,
                'distance_m': (
                    None if distance_m is None else round(distance_m, 2)
                ),
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
