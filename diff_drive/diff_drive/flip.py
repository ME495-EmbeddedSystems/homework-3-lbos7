"""A Node for controlling the ddrive robot in Gazebo."""

from geometry_msgs.msg import Twist, Vector3

import rclpy
from rclpy.node import Node

class Flip(Node):

    def __init__(self):
        super().__init__('flip')
        self.tmr = self.create_timer(1/100, self.timer_callback)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def timer_callback(self):
        pass


def main(args=None):
    """Entrypoint for the flip ROS node."""
    rclpy.init(args=args)
    node = Flip()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)
