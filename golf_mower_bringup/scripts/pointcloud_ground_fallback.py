#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


def point_count(msg: PointCloud2) -> int:
    return int(msg.width) * int(msg.height)


class PointCloudGroundFallback(Node):
    def __init__(self):
        super().__init__('pointcloud_ground_fallback')

        self.raw_cloud_topic = self.declare_parameter(
            'raw_cloud_topic', '/unilidar/cloud').value
        self.ground_cloud_topic = self.declare_parameter(
            'ground_cloud_topic', '/ground_segmentation/ground').value
        self.output_cloud_topic = self.declare_parameter(
            'output_cloud_topic', '/golf_mower/ground_cloud_for_elevation').value
        self.min_ground_points = int(self.declare_parameter(
            'min_ground_points', 100).value)

        self.latest_raw = None
        self.fallback_active = False

        self.publisher = self.create_publisher(PointCloud2, self.output_cloud_topic, 10)
        self.create_subscription(
            PointCloud2, self.raw_cloud_topic, self.raw_callback, 10)
        self.create_subscription(
            PointCloud2, self.ground_cloud_topic, self.ground_callback, 10)

        self.get_logger().info(
            f'publishing elevation input on {self.output_cloud_topic}; '
            f'using {self.ground_cloud_topic} when it has at least '
            f'{self.min_ground_points} points, otherwise falling back to '
            f'{self.raw_cloud_topic}')

    def raw_callback(self, msg: PointCloud2):
        self.latest_raw = msg

    def ground_callback(self, msg: PointCloud2):
        ground_points = point_count(msg)
        if ground_points >= self.min_ground_points:
            if self.fallback_active:
                self.get_logger().info('ground cloud recovered; leaving raw fallback mode')
                self.fallback_active = False
            self.publisher.publish(msg)
            return

        if self.latest_raw is None:
            self.get_logger().warn(
                'ground cloud is too small, but no raw cloud is available yet',
                throttle_duration_sec=5.0)
            return

        if not self.fallback_active:
            self.get_logger().warn(
                f'ground cloud has only {ground_points} points; '
                'using raw cloud for elevation mapping until Patchwork++ recovers')
            self.fallback_active = True
        self.publisher.publish(self.latest_raw)


def main():
    rclpy.init()
    node = PointCloudGroundFallback()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
