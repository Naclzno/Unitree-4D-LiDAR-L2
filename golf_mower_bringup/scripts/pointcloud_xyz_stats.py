#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointCloudXyzStats(Node):
    def __init__(self):
        super().__init__('pointcloud_xyz_stats')
        self.cloud_topic = self.declare_parameter('cloud_topic', '/unilidar/cloud').value
        self.print_every_n = int(self.declare_parameter('print_every_n', 30).value)
        self.frame_count = 0
        self.create_subscription(PointCloud2, self.cloud_topic, self.callback, 10)
        self.get_logger().info(f'listening to {self.cloud_topic}')

    def callback(self, msg: PointCloud2):
        self.frame_count += 1
        if self.print_every_n > 0 and self.frame_count % self.print_every_n != 0:
            return

        fields = {field.name: field.offset for field in msg.fields}
        if not all(name in fields for name in ('x', 'y', 'z')):
            self.get_logger().warn('cloud does not contain x/y/z fields')
            return

        count = int(msg.width) * int(msg.height)
        if count == 0:
            self.get_logger().warn('empty point cloud')
            return

        xs = []
        ys = []
        zs = []
        step = msg.point_step
        unpack = struct.Struct('<fff').unpack_from

        for i in range(count):
            base = i * step
            try:
                x = struct.unpack_from('<f', msg.data, base + fields['x'])[0]
                y = struct.unpack_from('<f', msg.data, base + fields['y'])[0]
                z = struct.unpack_from('<f', msg.data, base + fields['z'])[0]
            except struct.error:
                break
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                xs.append(x)
                ys.append(y)
                zs.append(z)

        if not xs:
            self.get_logger().warn('no finite x/y/z points')
            return

        def summary(values):
            values_sorted = sorted(values)
            n = len(values_sorted)
            return (
                values_sorted[0],
                values_sorted[n // 20],
                sum(values_sorted) / n,
                values_sorted[(19 * n) // 20],
                values_sorted[-1],
            )

        sx = summary(xs)
        sy = summary(ys)
        sz = summary(zs)
        self.get_logger().info(
            f'points={len(xs)} frame={msg.header.frame_id} | '
            f'x[min,p05,mean,p95,max]=[{sx[0]:.2f} {sx[1]:.2f} {sx[2]:.2f} {sx[3]:.2f} {sx[4]:.2f}] | '
            f'y=[{sy[0]:.2f} {sy[1]:.2f} {sy[2]:.2f} {sy[3]:.2f} {sy[4]:.2f}] | '
            f'z=[{sz[0]:.2f} {sz[1]:.2f} {sz[2]:.2f} {sz[3]:.2f} {sz[4]:.2f}]'
        )


def main():
    rclpy.init()
    node = PointCloudXyzStats()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
