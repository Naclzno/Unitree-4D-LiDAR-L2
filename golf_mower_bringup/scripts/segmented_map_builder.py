#!/usr/bin/env python3

import math
import os
import struct
from typing import Dict, Iterable, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def _rotation_matrix(qx: float, qy: float, qz: float, qw: float):
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _transform_point(point, transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    rot = _rotation_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
    x, y, z, intensity = point
    return (
        rot[0][0] * x + rot[0][1] * y + rot[0][2] * z + translation.x,
        rot[1][0] * x + rot[1][1] * y + rot[1][2] * z + translation.y,
        rot[2][0] * x + rot[2][1] * y + rot[2][2] * z + translation.z,
        intensity,
    )


def _voxel_key(x: float, y: float, z: float, resolution: float):
    return (
        int(math.floor(x / resolution)),
        int(math.floor(y / resolution)),
        int(math.floor(z / resolution)),
    )


def _write_binary_pcd(path: str, points: Iterable[Tuple[float, float, float, float]]):
    points = list(points)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z intensity\n'
        'SIZE 4 4 4 4\n'
        'TYPE F F F F\n'
        'COUNT 1 1 1 1\n'
        f'WIDTH {len(points)}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(points)}\n'
        'DATA binary\n'
    )
    with open(path, 'wb') as pcd:
        pcd.write(header.encode('utf-8'))
        for point in points:
            pcd.write(struct.pack('<ffff', *point))


class SegmentedMapBuilder(Node):
    def __init__(self):
        super().__init__('segmented_map_builder')

        self.declare_parameter('ground_topic', '/ground_segmentation/ground')
        self.declare_parameter('nonground_topic', '/ground_segmentation/nonground')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('output_dir', '/tmp/golf_mower_segmented_maps')
        self.declare_parameter('voxel_resolution', 0.05)
        self.declare_parameter('save_period_sec', 10.0)
        self.declare_parameter('max_points_per_cloud', 0)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.output_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))
        self.voxel_resolution = float(self.get_parameter('voxel_resolution').value)
        self.max_points_per_cloud = int(self.get_parameter('max_points_per_cloud').value)

        self.ground: Dict[Tuple[int, int, int], Tuple[float, float, float, float]] = {}
        self.nonground: Dict[Tuple[int, int, int], Tuple[float, float, float, float]] = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('ground_topic').value),
            lambda msg: self._cloud_callback(msg, self.ground, 'ground'),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('nonground_topic').value),
            lambda msg: self._cloud_callback(msg, self.nonground, 'nonground'),
            qos_profile_sensor_data,
        )

        save_period = float(self.get_parameter('save_period_sec').value)
        if save_period > 0.0:
            self.timer = self.create_timer(save_period, self.save_maps)
        else:
            self.timer = None

        self.get_logger().info(
            f'Accumulating segmented maps in {self.target_frame}; output_dir={self.output_dir}')

    def _cloud_callback(self, msg: PointCloud2, store, label: str):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'No transform {self.target_frame} <- {msg.header.frame_id}: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        field_names = [field.name for field in msg.fields]
        if 'intensity' in field_names:
            read_fields = ('x', 'y', 'z', 'intensity')
        else:
            read_fields = ('x', 'y', 'z')

        added = 0
        for idx, raw_point in enumerate(
            point_cloud2.read_points(msg, field_names=read_fields, skip_nans=True)
        ):
            if self.max_points_per_cloud > 0 and idx >= self.max_points_per_cloud:
                break
            if len(raw_point) == 3:
                point = (float(raw_point[0]), float(raw_point[1]), float(raw_point[2]), 0.0)
            else:
                point = (
                    float(raw_point[0]),
                    float(raw_point[1]),
                    float(raw_point[2]),
                    float(raw_point[3]),
                )
            transformed = _transform_point(point, transform)
            key = _voxel_key(
                transformed[0],
                transformed[1],
                transformed[2],
                self.voxel_resolution,
            )
            if key not in store:
                store[key] = transformed
                added += 1

        if added > 0:
            self.get_logger().info(
                f'{label}: added={added}, total={len(store)}',
                throttle_duration_sec=5.0,
            )

    def save_maps(self):
        ground_path = os.path.join(self.output_dir, 'ground_map.pcd')
        nonground_path = os.path.join(self.output_dir, 'nonground_map.pcd')
        _write_binary_pcd(ground_path, self.ground.values())
        _write_binary_pcd(nonground_path, self.nonground.values())
        self.get_logger().info(
            f'Saved ground={len(self.ground)} to {ground_path}; '
            f'nonground={len(self.nonground)} to {nonground_path}')


def main(args=None):
    rclpy.init(args=args)
    node = SegmentedMapBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_maps()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
