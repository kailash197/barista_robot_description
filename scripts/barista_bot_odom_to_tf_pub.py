#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class BaristaBotOdomToTF(Node):

    def __init__(self):
        super().__init__('odom_to_tf_broadcaster_node')

        # Declare parameters with default values
        self.declare_parameter("robot_base_frame", "rick/base_plate_link")
        self.declare_parameter("odom_topic", "/rick/odom")

        # Get parameters from the command line or use defaults
        self._robot_base_frame = self.get_parameter("robot_base_frame").get_parameter_value().string_value
        self._odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

        # Create a new `TransformStamped` object.
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "world"
        self.transform_stamped.child_frame_id = self._robot_base_frame

        self.subscriber = self.create_subscription(
            Odometry,
            self._odom_topic,
            self.odom_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.br = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info(f"odom_to_tf_broadcaster_node ready!")
        self.get_logger().info(f"Using robot_base_frame: {self._robot_base_frame}")
        self.get_logger().info(f"Listening to odometry topic: {self._odom_topic}")

    def odom_callback(self, msg):
        self.barista_bot_odom = msg
        self.get_logger().debug('Odom VALUE: "%s"' % str(self.barista_bot_odom))
        self.broadcast_new_tf()

    def broadcast_new_tf(self):
        time_header = self.barista_bot_odom.header
        position = self.barista_bot_odom.pose.pose.position
        orientation = self.barista_bot_odom.pose.pose.orientation

        self.transform_stamped.header.stamp = time_header.stamp
        self.transform_stamped.transform.translation.x = position.x
        self.transform_stamped.transform.translation.y = position.y
        self.transform_stamped.transform.translation.z = position.z
        self.transform_stamped.transform.rotation.x = orientation.x
        self.transform_stamped.transform.rotation.y = orientation.y
        self.transform_stamped.transform.rotation.z = orientation.z
        self.transform_stamped.transform.rotation.w = orientation.w

        self.br.sendTransform(self.transform_stamped)


def main(args=None):
    rclpy.init(args=args)
    odom_to_tf_obj = BaristaBotOdomToTF()
    rclpy.spin(odom_to_tf_obj)


if __name__ == '__main__':
    main()
