#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def point_count(msg: PointCloud2) -> int:
    return int(msg.width) * int(msg.height)


def xyz_points(msg: PointCloud2):
    points = []
    for point in point_cloud2.read_points(
        msg, field_names=('x', 'y', 'z'), skip_nans=True
    ):
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((x, y, z))
    return points


def covered_cells(points, cell_size: float) -> int:
    return len({
        (math.floor(x / cell_size), math.floor(y / cell_size))
        for x, y, _ in points
    })


def recover_local_low_surface(
    points,
    cell_size: float,
    height_threshold: float,
    expected_ground_z: float,
    ground_z_tolerance: float,
):
    cell_min_z = {}
    for x, y, z in points:
        key = (math.floor(x / cell_size), math.floor(y / cell_size))
        if key not in cell_min_z or z < cell_min_z[key]:
            cell_min_z[key] = z

    valid_cells = {
        key for key, min_z in cell_min_z.items()
        if abs(min_z - expected_ground_z) <= ground_z_tolerance
    }
    recovered = []
    for x, y, z in points:
        key = (math.floor(x / cell_size), math.floor(y / cell_size))
        if key in valid_cells and z <= cell_min_z[key] + height_threshold:
            recovered.append((x, y, z))
    return recovered


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
        self.min_ground_ratio = float(self.declare_parameter(
            'min_ground_ratio', 0.02).value)
        self.min_ground_cells = int(self.declare_parameter(
            'min_ground_cells', 20).value)
        self.failure_frames_before_recovery = int(self.declare_parameter(
            'failure_frames_before_recovery', 3).value)
        self.recovery_cell_size = float(self.declare_parameter(
            'recovery_cell_size', 0.20).value)
        self.recovery_height_threshold = float(self.declare_parameter(
            'recovery_height_threshold', 0.15).value)
        self.sensor_height = float(self.declare_parameter(
            'sensor_height', 0.75).value)
        self.recovery_ground_z_tolerance = float(self.declare_parameter(
            'recovery_ground_z_tolerance', 0.45).value)
        self.min_recovered_points = int(self.declare_parameter(
            'min_recovered_points', 100).value)
        self.min_recovered_cells = int(self.declare_parameter(
            'min_recovered_cells', 20).value)

        self.latest_raw = None
        self.consecutive_failures = 0
        self.recovery_active = False

        self.publisher = self.create_publisher(PointCloud2, self.output_cloud_topic, 10)
        self.create_subscription(
            PointCloud2, self.raw_cloud_topic, self.raw_callback, 10)
        self.create_subscription(
            PointCloud2, self.ground_cloud_topic, self.ground_callback, 10)

        self.get_logger().info(
            f'publishing elevation input on {self.output_cloud_topic}; '
            f'using quality-checked {self.ground_cloud_topic}; after '
            f'{self.failure_frames_before_recovery} consecutive failures, recovering '
            f'a local low surface from {self.raw_cloud_topic}. Raw clouds are never '
            'published directly as ground.')

    def raw_callback(self, msg: PointCloud2):
        self.latest_raw = msg

    def ground_callback(self, msg: PointCloud2):
        ground = xyz_points(msg)
        raw_count = point_count(self.latest_raw) if self.latest_raw is not None else 0
        ratio = len(ground) / max(raw_count, 1)
        cells = covered_cells(ground, self.recovery_cell_size)
        quality_ok = (
            len(ground) >= self.min_ground_points
            and ratio >= self.min_ground_ratio
            and cells >= self.min_ground_cells
        )

        if quality_ok:
            if self.recovery_active or self.consecutive_failures:
                self.get_logger().info(
                    'Patchwork++ ground quality recovered; leaving low-surface recovery mode')
            self.consecutive_failures = 0
            self.recovery_active = False
            self.publisher.publish(msg)
            return

        self.consecutive_failures += 1
        if self.latest_raw is None:
            self.get_logger().warn(
                'Ground quality is insufficient, but no raw cloud is available; '
                'holding the previous elevation map',
                throttle_duration_sec=5.0)
            return

        if self.consecutive_failures < self.failure_frames_before_recovery:
            self.get_logger().warn(
                f'Ground quality insufficient: points={len(ground)}, ratio={ratio:.3f}, '
                f'cells={cells}; holding the previous elevation map '
                f'({self.consecutive_failures}/{self.failure_frames_before_recovery})',
                throttle_duration_sec=2.0)
            return

        raw = xyz_points(self.latest_raw)
        recovered = recover_local_low_surface(
            raw,
            self.recovery_cell_size,
            self.recovery_height_threshold,
            -self.sensor_height,
            self.recovery_ground_z_tolerance,
        )
        recovered_cells = covered_cells(recovered, self.recovery_cell_size)
        if (
            len(recovered) < self.min_recovered_points
            or recovered_cells < self.min_recovered_cells
        ):
            self.get_logger().error(
                f'Ground recovery rejected: points={len(recovered)}, '
                f'cells={recovered_cells}. Holding the previous elevation map; '
                'check lidar pose, axis convention, and sensor_height.',
                throttle_duration_sec=5.0)
            return

        if not self.recovery_active:
            self.get_logger().warn(
                f'Entering local low-surface recovery mode: points={len(recovered)}, '
                f'cells={recovered_cells}.')
        self.recovery_active = True
        recovered_msg = point_cloud2.create_cloud_xyz32(
            self.latest_raw.header,
            recovered,
        )
        self.publisher.publish(recovered_msg)


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
