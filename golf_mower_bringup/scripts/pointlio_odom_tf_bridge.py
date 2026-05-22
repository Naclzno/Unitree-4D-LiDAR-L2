#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class PointlioOdomTfBridge(Node):
    def __init__(self):
        super().__init__('pointlio_odom_tf_bridge')

        self.declare_parameter('odom_topic', '/pointlio/odom')
        self.declare_parameter('parent_frame', 'camera_init')
        self.declare_parameter('child_frame', 'aft_mapped')
        self.declare_parameter('use_odom_frame_ids', True)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.use_odom_frame_ids = self.get_parameter('use_odom_frame_ids').value

        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            50)

        self.get_logger().info(
            f'Bridging {self.odom_topic} odometry to TF '
            f'({self.parent_frame} -> {self.child_frame})')

    def odom_callback(self, msg):
        parent_frame = self.parent_frame
        child_frame = self.child_frame
        if self.use_odom_frame_ids:
            parent_frame = msg.header.frame_id or parent_frame
            child_frame = msg.child_frame_id or child_frame

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = PointlioOdomTfBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
