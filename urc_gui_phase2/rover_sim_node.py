"""Simulated rover: the state-producing half of the sim/GUI split.

Skeleton only. Unlike Phase 1, nothing here is hardcoded: no default
waypoints, no scripted path, no baked-in site or mission name. The spawn point
is a parameter, and it is also the origin of the local ENU frame (see
coordinate_convert.py).

TODO:
  - publish rover GNSS as sensor_msgs/NavSatFix and pose as nav_msgs/Odometry
    (standard messages, not JSON strings)
  - accept the operator's mission over a typed interface (to be designed)
  - move toward received waypoints instead of following a canned curve
"""

import rclpy
from rclpy.node import Node


class RoverSimNode(Node):
    def __init__(self):
        super().__init__('rover_sim_node')
        # No default: the operator/launch config must supply the spawn point,
        # otherwise the sim refuses to invent one.
        self.declare_parameter('spawn_lat_deg', float('nan'))
        self.declare_parameter('spawn_lon_deg', float('nan'))
        self.get_logger().info('rover_sim_node skeleton started (publishes nothing yet)')


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
