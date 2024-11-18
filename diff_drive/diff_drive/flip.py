"""A Node for controlling the ddrive robot in Gazebo."""

from enum import Enum, auto

from geometry_msgs.msg import Twist, Vector3

import rclpy
from rclpy.node import Node



class Flip(Node):
    """
    Node for controlling the ddrive robotf.

    Publishes
    ---------
    cmd_vel : geometry_msgs/msg/Twist - the speed the turtle in turtlesim

    """

    def __init__(self):
        super().__init__('flip')
        self.tmr = self.create_timer(1/100, self.timer_callback)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vel = 2.0
        self.count = 0

    def timer_callback(self):
        """
        Timer callback for controlling the turtle robot.

        Publishes commands at a set frequency paramter 

        """
        if self.count > 300:
            self.vel *= -1
            self.count = 0

        self.vel_pub.publish(Twist(linear=Vector3(x=self.vel)))
        self.count += 1


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
