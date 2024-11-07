"""A Node for controlling the ddrive robot in Gazebo."""

from enum import Enum, auto

from geometry_msgs.msg import TransformStamped, Twist, Vector3

import rclpy
from rclpy.node import Node

import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration

    """

    NORMAL = auto(),
    FLIPPED = auto(),
    STOPPED = auto()


class Flip(Node):

    def __init__(self):
        super().__init__('flip')
        self.tmr = self.create_timer(1/100, self.timer_callback)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.tf_broadcaster = TransformBroadcaster(self, 10)
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)

        self.state = State.STOPPED
        self.vel = 6.5

        odom_base_tf = TransformStamped()
        odom_base_tf.header.stamp = self.get_clock().now().to_msg()
        odom_base_tf.header.frame_id = 'odom'
        odom_base_tf.child_frame_id = 'base_link'
        odom_base_tf.transform.translation.x = 0.0
        self.tf_broadcaster.sendTransform(odom_base_tf)

        self.start_time = self.get_clock().now().to_msg()

    def timer_callback(self):
        if self.state != State.STOPPED:
            self.vel_pub.publish(
                Twist(
                    linear=Vector3(
                        x=self.vel
                    )
                )
            )
            try:
                tf_odom_base = self.buffer.lookup_transform('odom',
                                                            'base_link',
                                                            rclpy.time.Time())
            except tf2_ros.LookupException:
                pass
            except tf2_ros.ConnectivityException:
                pass
            except tf2_ros.ExtrapolationException:
                pass

            tf_odom_base = self.buffer.lookup_transform('odom',
                                                        'base_link',
                                                        rclpy.time.Time())
            if tf_odom_base.transform.translation.x >= 3.0:
                self.vel = -6.5
            elif tf_odom_base.transform.translation.x <= -3.0:
                self.vel = 6.5

            odom_base_tf = TransformStamped()
            odom_base_tf.header.stamp = self.get_clock().now().to_msg()
            odom_base_tf.header.frame_id = 'odom'
            odom_base_tf.child_frame_id = 'base_link'
            odom_base_tf.transform.translation.x = \
                tf_odom_base.transform.translation.x
            self.tf_broadcaster.sendTransform(odom_base_tf)

        odom_base_tf = TransformStamped()
        odom_base_tf.header.stamp = self.get_clock().now().to_msg()
        odom_base_tf.header.frame_id = 'odom'
        odom_base_tf.child_frame_id = 'base_link'
        odom_base_tf.transform.translation.x = 0.0
        self.tf_broadcaster.sendTransform(odom_base_tf)

        if self.get_clock().now().to_msg().sec - self.start_time.sec > 5:
            self.state = State.NORMAL


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
