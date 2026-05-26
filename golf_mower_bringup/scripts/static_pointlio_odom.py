#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class StaticPointlioOdom(Node):
    def __init__(self):
        super().__init__('static_pointlio_odom')
        self.declare_parameter('odom_topic', '/pointlio/odom')
        self.declare_parameter('parent_frame', 'camera_init')
        self.declare_parameter('child_frame', 'aft_mapped')
        self.declare_parameter('publish_rate', 30.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        period = 1.0 / max(publish_rate, 1.0)
        self.timer = self.create_timer(period, self.publish_odom)
        self.get_logger().info(
            f'Publishing static odometry on {self.odom_topic} '
            f'({self.parent_frame} -> {self.child_frame})')

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame
        msg.child_frame_id = self.child_frame
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 1.0e-4
        msg.pose.covariance[7] = 1.0e-4
        msg.pose.covariance[14] = 1.0e-4
        msg.pose.covariance[21] = 1.0e-4
        msg.pose.covariance[28] = 1.0e-4
        msg.pose.covariance[35] = 1.0e-4
        msg.twist.covariance[0] = 1.0e-3
        msg.twist.covariance[7] = 1.0e-3
        msg.twist.covariance[14] = 1.0e-3
        msg.twist.covariance[21] = 1.0e-3
        msg.twist.covariance[28] = 1.0e-3
        msg.twist.covariance[35] = 1.0e-3
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = StaticPointlioOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
