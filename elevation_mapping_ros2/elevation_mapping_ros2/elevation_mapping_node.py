#!/usr/bin/env python3

import os
import pickle
from typing import Dict, Tuple

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import tf2_ros
from tf_transformations import quaternion_matrix


def encode_layer_to_multiarray(array: np.ndarray) -> Float32MultiArray:
    arr = np.asarray(array, dtype=np.float32)
    rows, cols = arr.shape
    msg = Float32MultiArray()
    msg.layout = MultiArrayLayout()
    msg.layout.dim.append(
        MultiArrayDimension(label='column_index', size=cols, stride=rows * cols)
    )
    msg.layout.dim.append(
        MultiArrayDimension(label='row_index', size=rows, stride=rows)
    )
    msg.data = arr.flatten(order='F').tolist()
    return msg


def pointcloud2_xyz_f32(msg: PointCloud2) -> np.ndarray:
    if msg.is_bigendian:
        raise ValueError('PointCloud2 big-endian data is not supported')

    fields: Dict[str, PointField] = {field.name: field for field in msg.fields}
    missing = {'x', 'y', 'z'}.difference(fields)
    if missing:
        raise ValueError(f'PointCloud2 missing required fields: {sorted(missing)}')

    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.datatype != PointField.FLOAT32 or field.count != 1:
            raise ValueError(
                f"PointCloud2 field '{name}' must be FLOAT32 count=1, "
                f'got datatype={field.datatype} count={field.count}'
            )

    dtype = np.dtype({
        'names': ('x', 'y', 'z'),
        'formats': (np.float32, np.float32, np.float32),
        'offsets': (fields['x'].offset, fields['y'].offset, fields['z'].offset),
        'itemsize': msg.point_step,
    })
    points = np.frombuffer(msg.data, dtype=dtype)
    xyz = np.stack((points['x'], points['y'], points['z']), axis=-1).astype(
        np.float32, copy=False
    )
    return xyz[np.isfinite(xyz).all(axis=1)]


def transform_points(points: np.ndarray, transform_msg) -> np.ndarray:
    translation = transform_msg.transform.translation
    rotation = transform_msg.transform.rotation
    matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
    matrix[0, 3] = translation.x
    matrix[1, 3] = translation.y
    matrix[2, 3] = translation.z

    homogeneous = np.ones((points.shape[0], 4), dtype=np.float32)
    homogeneous[:, :3] = points
    transformed = homogeneous @ matrix.T
    return transformed[:, :3].astype(np.float32, copy=False)


class ElevationMappingNode(Node):
    def __init__(self):
        super().__init__(
            'elevation_mapping_node',
            automatically_declare_parameters_from_overrides=True,
            allow_undeclared_parameters=True,
        )

        self.map_frame = self._param_str('map_frame', 'map')
        self.base_frame = self._param_str('base_frame', 'base_link')
        self.resolution = self._param_float('resolution', 0.10)
        self.map_length = self._param_float('map_length', 20.0)
        self.sensor_noise_factor = self._param_float('sensor_noise_factor', 0.05)
        self.mahalanobis_thresh = self._param_float('mahalanobis_thresh', 2.0)
        self.outlier_variance = self._param_float('outlier_variance', 0.01)
        self.wall_num_thresh = int(self._param_float('wall_num_thresh', 20))
        self.max_ray_length = self._param_float('max_ray_length', 10.0)
        self.cleanup_step = self._param_float('cleanup_step', 0.1)
        self.cleanup_cos_thresh = self._param_float('cleanup_cos_thresh', 0.1)
        self.enable_visibility_cleanup = self._param_bool(
            'enable_visibility_cleanup', True
        )
        self.enable_edge_sharpen = self._param_bool('enable_edge_sharpen', True)
        self.max_variance = self._param_float('max_variance', 100.0)
        self.initial_variance = self._param_float('initial_variance', 1000.0)
        self.time_variance = self._param_float('time_variance', 0.0001)
        self.time_interval = self._param_float('time_interval', 0.1)
        self.update_pose_fps = self._param_float('update_pose_fps', 10.0)
        self.update_variance_fps = self._param_float('update_variance_fps', 5.0)
        self.dilation_size = int(self._param_float('dilation_size', 3))
        self.use_neural_traversability = self._param_bool(
            'use_neural_traversability', True
        )
        self.min_valid_distance = self._param_float('min_valid_distance', 0.2)
        self.max_height_range = self._param_float('max_height_range', 4.0)
        self.ramped_height_range_a = self._param_float('ramped_height_range_a', 0.3)
        self.ramped_height_range_b = self._param_float('ramped_height_range_b', 1.0)
        self.ramped_height_range_c = self._param_float('ramped_height_range_c', 0.2)
        self.slope_obstacle_threshold = self._param_float(
            'slope_obstacle_threshold', 0.20
        )
        self.variance_obstacle_threshold = self._param_float(
            'variance_obstacle_threshold', 0.05
        )
        self.publish_empty_maps = self._param_bool('publish_empty_maps', True)

        self.rows = max(1, int(round(self.map_length / self.resolution)))
        self.cols = self.rows
        self.internal_rows = self.rows + 2
        self.internal_cols = self.cols + 2
        self.actual_map_length = self.rows * self.resolution

        self.publishers_cfg = self._read_publishers()
        self.subscriber_topics = self._read_pointcloud_topics()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.elevation = np.zeros((self.internal_rows, self.internal_cols), dtype=np.float32)
        self.variance = np.full(
            (self.internal_rows, self.internal_cols),
            self.initial_variance,
            dtype=np.float32,
        )
        self.is_valid = np.zeros((self.internal_rows, self.internal_cols), dtype=np.float32)
        self.traversability = np.ones((self.internal_rows, self.internal_cols), dtype=np.float32)
        self.time = np.zeros((self.internal_rows, self.internal_cols), dtype=np.float32)
        self.upper_bound = np.zeros((self.internal_rows, self.internal_cols), dtype=np.float32)
        self.is_upper_bound = np.zeros((self.internal_rows, self.internal_cols), dtype=np.float32)
        self.normal_x = np.zeros((self.internal_rows, self.internal_cols), dtype=np.float32)
        self.normal_y = np.zeros((self.internal_rows, self.internal_cols), dtype=np.float32)
        self.normal_z = np.ones((self.internal_rows, self.internal_cols), dtype=np.float32)
        self.center_x = 0.0
        self.center_y = 0.0
        self.center_z = 0.0
        self.traversability_weights = self._load_traversability_weights()
        self.layers = self._build_publish_layers()
        self.last_stamp = None

        self.map_publishers = {}
        self.publish_timers = []
        for key, cfg in self.publishers_cfg.items():
            self.map_publishers[key] = self.create_publisher(
                GridMap, f'/{self.get_name()}/{key}', 10
            )
            fps = max(float(cfg.get('fps', 1.0)), 0.01)
            self.publish_timers.append(
                self.create_timer(1.0 / fps, lambda k=key: self.publish_map(k))
            )

        qos_profile = QoSPresetProfiles.get_from_short_key('sensor_data')
        self.pointcloud_subscriptions = [
            self.create_subscription(
                PointCloud2,
                topic,
                self.pointcloud_callback,
                qos_profile,
            )
            for topic in self.subscriber_topics
        ]
        self.pose_timer = self.create_timer(
            1.0 / max(self.update_pose_fps, 0.01), self.update_pose
        )
        self.variance_timer = self.create_timer(
            1.0 / max(self.update_variance_fps, 0.01), self.update_variance
        )

        self.get_logger().info(
            'CPU elevation mapping active: '
            f'topics={self.subscriber_topics}, map_frame={self.map_frame}, '
            f'base_frame={self.base_frame}, map={self.rows}x{self.cols}, '
            f'resolution={self.resolution:.3f}, '
            f'neural_traversability={self.traversability_weights is not None}'
        )

    def _param_float(self, name: str, default: float) -> float:
        return float(self.get_parameter(name).value) if self.has_parameter(name) else default

    def _param_bool(self, name: str, default: bool) -> bool:
        return bool(self.get_parameter(name).value) if self.has_parameter(name) else default

    def _param_str(self, name: str, default: str) -> str:
        return str(self.get_parameter(name).value) if self.has_parameter(name) else default

    def _load_traversability_weights(self):
        if not self.use_neural_traversability:
            return None

        weight_file = self._param_str('weight_file', '')
        candidates = []
        if weight_file:
            candidates.append(weight_file)
        for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
            if not prefix:
                continue
            candidates.append(
                os.path.join(
                    prefix,
                    'share',
                    'elevation_mapping_cupy',
                    'config',
                    'core',
                    'weights.dat',
                )
            )
        candidates.extend([
            '/home/ubuntu/unilidar_sdk2/elevation_mapping_cupy/install/elevation_mapping_cupy/share/elevation_mapping_cupy/config/core/weights.dat',
            '/home/ubuntu/unilidar_sdk2/elevation_mapping_cupy/elevation_mapping_cupy/config/core/weights.dat',
        ])

        for path in candidates:
            if not path or not os.path.exists(path):
                continue
            try:
                with open(path, 'rb') as file:
                    weights = pickle.load(file)
                return {
                    'w1': np.asarray(weights['conv1.weight'], dtype=np.float32),
                    'w2': np.asarray(weights['conv2.weight'], dtype=np.float32),
                    'w3': np.asarray(weights['conv3.weight'], dtype=np.float32),
                    'w_out': np.asarray(weights['conv_final.weight'], dtype=np.float32),
                }
            except Exception as exc:
                self.get_logger().warn(
                    f'Failed to load traversability weights from {path}: {exc}',
                    throttle_duration_sec=5.0,
                )
        self.get_logger().warn(
            'Neural traversability weights not found; using slope/variance fallback.'
        )
        return None

    def _read_pointcloud_topics(self):
        params = self.get_parameters_by_prefix('subscribers')
        grouped = {}
        for param_name, param_value in params.items():
            parts = param_name.split('.')
            if len(parts) < 2:
                continue
            key, sub_key = parts[:2]
            grouped.setdefault(key, {})[sub_key] = param_value.value

        topics = []
        for cfg in grouped.values():
            if cfg.get('data_type') == 'pointcloud' and cfg.get('topic_name'):
                topics.append(str(cfg['topic_name']))

        return topics or ['/golf_mower/ground_cloud_for_elevation']

    def _read_publishers(self):
        params = self.get_parameters_by_prefix('publishers')
        grouped = {}
        for param_name, param_value in params.items():
            parts = param_name.split('.')
            if len(parts) < 2:
                continue
            key, sub_key = parts[:2]
            grouped.setdefault(key, {})[sub_key] = param_value.value

        if grouped:
            return grouped

        return {
            'elevation_map_raw': {
                'layers': ['elevation', 'traversability', 'variance'],
                'basic_layers': ['elevation'],
                'fps': 3.0,
            },
            'elevation_map_filter': {
                'layers': ['min_filter', 'smooth', 'inpaint', 'elevation', 'traversability', 'variance'],
                'basic_layers': ['min_filter'],
                'fps': 2.0,
            },
        }

    def _empty_layers(self) -> Dict[str, np.ndarray]:
        nan_layer = np.full((self.rows, self.cols), np.nan, dtype=np.float32)
        return {
            'elevation': nan_layer.copy(),
            'variance': nan_layer.copy(),
            'traversability': nan_layer.copy(),
            'min_filter': nan_layer.copy(),
            'smooth': nan_layer.copy(),
            'inpaint': nan_layer.copy(),
        }

    def _lookup_pose(self, stamp):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time.from_msg(stamp),
                timeout=Duration(seconds=0.05),
            )
            return transform
        except Exception:
            try:
                return self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
            except Exception:
                return None

    def _move_to_pose(self, transform) -> None:
        if transform is None:
            return

        translation = transform.transform.translation
        target = np.array(
            [translation.x, translation.y, translation.z],
            dtype=np.float32,
        )
        center = np.array([self.center_x, self.center_y, self.center_z], dtype=np.float32)
        delta = target - center
        delta_pixel = np.rint(delta[:2] / self.resolution).astype(np.int32)
        if np.any(delta_pixel != 0):
            self.center_x += float(delta_pixel[0] * self.resolution)
            self.center_y += float(delta_pixel[1] * self.resolution)
            self._shift_map_xy(-int(delta_pixel[1]), -int(delta_pixel[0]))
        if abs(float(delta[2])) > 1e-6:
            self.center_z += float(delta[2])
            self.elevation -= float(delta[2])
            self.upper_bound -= float(delta[2])

    def update_pose(self) -> None:
        transform = self._lookup_pose(
            self.last_stamp if self.last_stamp is not None else self.get_clock().now().to_msg()
        )
        self._move_to_pose(transform)

    def update_variance(self) -> None:
        self.variance += self.time_variance * self.is_valid
        self.variance = np.minimum(self.variance, self.max_variance).astype(np.float32, copy=False)
        self.time += self.time_interval

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        try:
            points = pointcloud2_xyz_f32(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return

        if points.size == 0:
            if self.publish_empty_maps:
                self.layers = self._build_publish_layers()
            return

        raw_points = points
        frame_id = msg.header.frame_id
        if frame_id != self.map_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    frame_id,
                    rclpy.time.Time.from_msg(msg.header.stamp),
                    timeout=Duration(seconds=0.05),
                )
            except Exception as exc:
                self.get_logger().warn(
                    f'No TF {self.map_frame} <- {frame_id}: {exc}',
                    throttle_duration_sec=2.0,
                )
                return
            points = transform_points(points, transform)

        self._move_to_pose(self._lookup_pose(msg.header.stamp))
        self.last_stamp = msg.header.stamp
        self._fuse_points(points, raw_points)
        self.layers = self._build_publish_layers()

    def _fuse_points(self, map_points: np.ndarray, raw_points: np.ndarray) -> None:
        sensor_z = self.center_z

        dx = map_points[:, 0] - self.center_x
        dy = map_points[:, 1] - self.center_y
        dz = map_points[:, 2] - sensor_z
        horizontal_dist = np.hypot(dx, dy)
        ramped_limit = (
            np.maximum(horizontal_dist - self.ramped_height_range_b, 0.0)
            * self.ramped_height_range_a
            + self.ramped_height_range_c
        )
        good = (
            (horizontal_dist >= self.min_valid_distance)
            & (dz <= self.max_height_range)
            & (dz <= ramped_limit)
        )
        map_points = map_points[good]
        raw_points = raw_points[good]
        if map_points.size == 0:
            return

        col = np.floor(
            (map_points[:, 0] - self.center_x) / self.resolution
            + 0.5 * (self.internal_cols - 1)
            + 0.5
        ).astype(np.int32)
        row = np.floor(
            (map_points[:, 1] - self.center_y) / self.resolution
            + 0.5 * (self.internal_rows - 1)
            + 0.5
        ).astype(np.int32)
        inside = (
            (0 < row)
            & (row < self.internal_rows - 1)
            & (0 < col)
            & (col < self.internal_cols - 1)
        )
        row = row[inside]
        col = col[inside]
        inside_points = map_points[inside]
        z = (inside_points[:, 2] - self.center_z).astype(np.float32, copy=False)
        raw = raw_points[inside]

        if z.size == 0:
            return

        if self.enable_visibility_cleanup:
            self._visibility_cleanup(inside_points, z)

        point_var = self.sensor_noise_factor * np.sum(raw[:, :3] * raw[:, :3], axis=1)
        point_var = np.maximum(point_var.astype(np.float32, copy=False), 1e-6)

        map_h = self.elevation[row, col]
        map_v = self.variance[row, col]
        outlier = np.abs(map_h - z) > (map_v * self.mahalanobis_thresh)
        if np.any(outlier):
            np.add.at(self.variance, (row[outlier], col[outlier]), self.outlier_variance)

        accept = ~outlier
        if self.enable_edge_sharpen:
            flat_all = row * self.internal_cols + col
            point_count = np.bincount(
                flat_all,
                minlength=self.internal_rows * self.internal_cols,
            ).astype(np.float32)
            count_at_cell = point_count[flat_all]
            edge_reject = (
                (count_at_cell > max(self.wall_num_thresh, 0))
                & (z < map_h - map_v * self.mahalanobis_thresh / np.maximum(count_at_cell, 1.0))
            )
            accept &= ~edge_reject
        if not np.any(accept):
            return

        ar = row[accept]
        ac = col[accept]
        ah = map_h[accept]
        av = map_v[accept]
        az = z[accept]
        pv = point_var[accept]
        new_h = (ah * pv + az * av) / (av + pv)
        new_v = (av * pv) / (av + pv)

        flat = ar * self.internal_cols + ac
        size = self.internal_rows * self.internal_cols
        count = np.bincount(flat, minlength=size).astype(np.float32)
        h_sum = np.bincount(flat, weights=new_h, minlength=size).astype(np.float32)
        v_sum = np.bincount(flat, weights=new_v, minlength=size).astype(np.float32)
        touched = count.reshape((self.internal_rows, self.internal_cols)) > 0
        h_mean = np.divide(
            h_sum,
            count,
            out=np.zeros_like(h_sum, dtype=np.float32),
            where=count > 0,
        ).reshape((self.internal_rows, self.internal_cols))
        v_mean = np.divide(
            v_sum,
            count,
            out=np.zeros_like(v_sum, dtype=np.float32),
            where=count > 0,
        ).reshape((self.internal_rows, self.internal_cols))

        reset = touched & (v_mean > self.max_variance)
        update = touched & ~reset
        self.elevation[update] = h_mean[update]
        self.variance[update] = v_mean[update]
        self.is_valid[update] = 1.0
        self.time[update] = 0.0
        self.upper_bound[update] = h_mean[update]
        self.is_upper_bound[update] = 0.0

        self.elevation[reset] = 0.0
        self.variance[reset] = self.initial_variance
        self.is_valid[reset] = 0.0
        self.upper_bound[reset] = 0.0
        self.is_upper_bound[reset] = 0.0
        self._update_traversability()

    def _visibility_cleanup(self, map_points: np.ndarray, relative_z: np.ndarray) -> None:
        if map_points.size == 0:
            return

        sensor = np.array(
            [self.center_x, self.center_y, self.center_z],
            dtype=np.float32,
        )
        ray_step = max(self.resolution / np.sqrt(2.0), 1e-6)
        max_ray_length = max(self.max_ray_length, ray_step)

        for point, _z_rel in zip(map_points, relative_z):
            ray = point - sensor
            full_length = float(np.linalg.norm(ray))
            if full_length < self.min_valid_distance:
                continue

            direction = ray / full_length
            ray_length = min(full_length, max_ray_length)
            last_index = (-1, -1)
            s = ray_step
            while s < ray_length:
                probe = sensor + direction * s
                row, col = self._map_index(float(probe[0]), float(probe[1]))
                if (row, col) == last_index:
                    s += ray_step
                    continue
                last_index = (row, col)
                if not self._inside_internal(row, col):
                    s += ray_step
                    continue

                d = float(np.sum((point - probe) * (point - probe)))
                if d < 0.1:
                    s += ray_step
                    continue

                nz = float(probe[2] - self.center_z)
                if self.is_valid[row, col] < 0.5:
                    if nz < self.upper_bound[row, col] or self.is_upper_bound[row, col] < 0.5:
                        self.upper_bound[row, col] = nz
                        self.is_upper_bound[row, col] = 1.0
                    s += ray_step
                    continue

                if self.time[row, col] < 0.5:
                    s += ray_step
                    continue

                height_threshold = nz + 0.01 - min(float(self.variance[row, col]), 1.0) * 0.05
                if self.elevation[row, col] <= height_threshold:
                    s += ray_step
                    continue

                ray_dot_normal = (
                    float(direction[0]) * float(self.normal_x[row, col])
                    + float(direction[1]) * float(self.normal_y[row, col])
                    + float(direction[2]) * float(self.normal_z[row, col])
                )
                if abs(ray_dot_normal) < self.cleanup_cos_thresh:
                    s += ray_step
                    continue

                cleanup_amount = self.cleanup_step / max(ray_length / max_ray_length, 1e-3)
                self.is_valid[row, col] = max(0.0, self.is_valid[row, col] - cleanup_amount)
                self.variance[row, col] = min(
                    self.max_variance,
                    self.variance[row, col] + self.outlier_variance,
                )
                if nz < self.upper_bound[row, col] or self.is_upper_bound[row, col] < 0.5:
                    self.upper_bound[row, col] = nz
                    self.is_upper_bound[row, col] = 1.0
                s += ray_step

    def _map_index(self, x: float, y: float) -> Tuple[int, int]:
        col = int(
            np.floor(
                (x - self.center_x) / self.resolution
                + 0.5 * (self.internal_cols - 1)
                + 0.5
            )
        )
        row = int(
            np.floor(
                (y - self.center_y) / self.resolution
                + 0.5 * (self.internal_rows - 1)
                + 0.5
            )
        )
        return row, col

    def _inside_internal(self, row: int, col: int) -> bool:
        return (
            0 < row < self.internal_rows - 1
            and 0 < col < self.internal_cols - 1
        )

    def _build_publish_layers(self) -> Dict[str, np.ndarray]:
        valid = self.is_valid[1:-1, 1:-1] > 0.5
        elevation = np.where(valid, self.elevation[1:-1, 1:-1] + self.center_z, np.nan)
        variance = self.variance[1:-1, 1:-1].copy()
        smooth = self._nanmean_3x3(elevation)
        min_filter = self._nanmin_3x3(elevation)
        inpaint = np.where(np.isfinite(elevation), elevation, smooth)
        traversability_valid = (
            (self.is_valid[1:-1, 1:-1] + self.is_upper_bound[1:-1, 1:-1]) > 0.5
        )
        traversability = np.where(
            traversability_valid,
            self.traversability[1:-1, 1:-1],
            np.nan,
        )
        layers = {
            'elevation': elevation,
            'variance': variance,
            'is_valid': np.where(valid, 1.0, np.nan).astype(np.float32),
            'traversability': traversability,
            'time': self.time[1:-1, 1:-1].copy(),
            'upper_bound': np.where(
                (self.is_upper_bound[1:-1, 1:-1] > 0.5) | valid,
                self.upper_bound[1:-1, 1:-1] + self.center_z,
                np.nan,
            ),
            'is_upper_bound': np.where(
                self.is_upper_bound[1:-1, 1:-1] > 0.5,
                self.is_upper_bound[1:-1, 1:-1],
                np.nan,
            ).astype(np.float32),
            'min_filter': min_filter,
            'smooth': smooth,
            'inpaint': inpaint,
        }
        return {name: value.astype(np.float32, copy=False) for name, value in layers.items()}

    def _update_traversability(self) -> None:
        mask = ((self.is_valid + self.is_upper_bound) > 0.5).astype(np.float32)
        dilated = self._dilate_height(self.upper_bound, mask, max(self.dilation_size, 0))
        self._update_normals(dilated, mask)
        if self.traversability_weights is None:
            valid = self.is_valid[1:-1, 1:-1] > 0.5
            elevation = np.where(valid, self.elevation[1:-1, 1:-1] + self.center_z, np.nan)
            self.traversability[1:-1, 1:-1] = np.nan_to_num(
                self._traversability_fallback(
                    elevation,
                    self.variance[1:-1, 1:-1],
                    valid,
                ),
                nan=1.0,
            )
            return

        out = self._neural_traversability(dilated)
        self.traversability[3:-3, 3:-3] = out

    def _update_normals(self, height: np.ndarray, mask: np.ndarray) -> None:
        self.normal_x.fill(0.0)
        self.normal_y.fill(0.0)
        self.normal_z.fill(1.0)

        if height.shape[0] < 3 or height.shape[1] < 3:
            return

        center = height[1:-1, 1:-1]
        right = height[1:-1, 2:]
        down = height[2:, 1:-1]
        valid = (
            (mask[1:-1, 1:-1] > 0.5)
            & (mask[1:-1, 2:] > 0.5)
            & (mask[2:, 1:-1] > 0.5)
        )
        if not np.any(valid):
            return

        dzdx = (right - center) / self.resolution
        dzdy = (down - center) / self.resolution
        nx = -dzdx
        ny = -dzdy
        nz = np.ones_like(nx, dtype=np.float32)
        norm = np.sqrt(nx * nx + ny * ny + nz * nz)
        norm = np.maximum(norm, 1e-6)

        normal_x = self.normal_x[1:-1, 1:-1]
        normal_y = self.normal_y[1:-1, 1:-1]
        normal_z = self.normal_z[1:-1, 1:-1]
        normal_x[valid] = (nx / norm)[valid]
        normal_y[valid] = (ny / norm)[valid]
        normal_z[valid] = (nz / norm)[valid]

    def _dilate_height(
        self,
        height: np.ndarray,
        mask: np.ndarray,
        radius: int,
    ) -> np.ndarray:
        out = height.copy()
        if radius <= 0:
            return out

        missing = mask < 0.5
        if not np.any(missing):
            return out

        nearest = np.full_like(height, np.nan, dtype=np.float32)
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                shifted_height = self._shift_internal(height, dy, dx)
                shifted_mask = self._shift_internal(mask, dy, dx)
                take = np.isnan(nearest) & (shifted_mask > 0.5)
                nearest[take] = shifted_height[take]
        out[missing & np.isfinite(nearest)] = nearest[missing & np.isfinite(nearest)]
        return out

    def _neural_traversability(self, elevation: np.ndarray) -> np.ndarray:
        weights = self.traversability_weights
        assert weights is not None

        out1 = self._conv3x3(elevation, weights['w1'][:, 0], dilation=1)[:, 2:-2, 2:-2]
        out2 = self._conv3x3(elevation, weights['w2'][:, 0], dilation=2)[:, 1:-1, 1:-1]
        out3 = self._conv3x3(elevation, weights['w3'][:, 0], dilation=3)
        features = np.concatenate((out1, out2, out3), axis=0)
        response = np.sum(np.abs(features) * weights['w_out'][0, :, 0, 0, None, None], axis=0)
        return np.exp(-response).astype(np.float32, copy=False)

    def _conv3x3(
        self,
        data: np.ndarray,
        kernels: np.ndarray,
        dilation: int,
    ) -> np.ndarray:
        radius = dilation
        out_rows = data.shape[0] - 2 * radius
        out_cols = data.shape[1] - 2 * radius
        outputs = np.zeros((kernels.shape[0], out_rows, out_cols), dtype=np.float32)
        for ky in range(3):
            row_slice = slice(ky * dilation, ky * dilation + out_rows)
            for kx in range(3):
                col_slice = slice(kx * dilation, kx * dilation + out_cols)
                window = data[row_slice, col_slice]
                outputs += kernels[:, ky, kx, None, None] * window[None, :, :]
        return outputs

    def _nanmean_3x3(self, data: np.ndarray) -> np.ndarray:
        total = np.zeros_like(data, dtype=np.float32)
        count = np.zeros_like(data, dtype=np.float32)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                shifted = self._shift(data, dy, dx)
                mask = np.isfinite(shifted)
                total[mask] += shifted[mask]
                count[mask] += 1.0
        out = np.full_like(data, np.nan, dtype=np.float32)
        mask = count > 0
        out[mask] = total[mask] / count[mask]
        return out

    def _nanmin_3x3(self, data: np.ndarray) -> np.ndarray:
        stacked = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                stacked.append(self._shift(data, dy, dx))
        with np.errstate(all='ignore'):
            return np.nanmin(np.stack(stacked, axis=0), axis=0).astype(np.float32)

    def _shift(self, data: np.ndarray, dy: int, dx: int) -> np.ndarray:
        out = np.full_like(data, np.nan, dtype=np.float32)
        src_y0 = max(0, -dy)
        src_y1 = self.rows - max(0, dy)
        src_x0 = max(0, -dx)
        src_x1 = self.cols - max(0, dx)
        dst_y0 = max(0, dy)
        dst_y1 = self.rows - max(0, -dy)
        dst_x0 = max(0, dx)
        dst_x1 = self.cols - max(0, -dx)
        out[dst_y0:dst_y1, dst_x0:dst_x1] = data[src_y0:src_y1, src_x0:src_x1]
        return out

    def _traversability_fallback(
        self,
        elevation: np.ndarray,
        variance: np.ndarray,
        valid: np.ndarray,
    ) -> np.ndarray:
        traversability = np.full_like(elevation, np.nan, dtype=np.float32)
        finite = np.isfinite(elevation) & valid
        if not np.any(finite):
            return traversability

        gy, gx = np.gradient(np.nan_to_num(elevation, nan=0.0), self.resolution)
        slope = np.hypot(gx, gy)
        score = np.ones_like(elevation, dtype=np.float32)
        score -= np.clip(slope / max(self.slope_obstacle_threshold, 1e-6), 0.0, 1.0) * 0.7
        score -= np.clip(variance / max(self.variance_obstacle_threshold, 1e-6), 0.0, 1.0) * 0.3
        traversability[finite] = np.clip(score[finite], 0.0, 1.0)
        return traversability

    def _shift_internal(self, data: np.ndarray, dy: int, dx: int) -> np.ndarray:
        rows, cols = data.shape
        out = np.full_like(data, np.nan, dtype=np.float32)
        src_y0 = max(0, -dy)
        src_y1 = rows - max(0, dy)
        src_x0 = max(0, -dx)
        src_x1 = cols - max(0, dx)
        dst_y0 = max(0, dy)
        dst_y1 = rows - max(0, -dy)
        dst_x0 = max(0, dx)
        dst_x1 = cols - max(0, -dx)
        out[dst_y0:dst_y1, dst_x0:dst_x1] = data[src_y0:src_y1, src_x0:src_x1]
        return out

    def _shift_map_xy(self, row_shift: int, col_shift: int) -> None:
        if row_shift == 0 and col_shift == 0:
            return

        arrays = (
            self.elevation,
            self.variance,
            self.is_valid,
            self.traversability,
            self.time,
            self.upper_bound,
            self.is_upper_bound,
            self.normal_x,
            self.normal_y,
            self.normal_z,
        )
        for data in arrays:
            data[:] = np.roll(data, shift=(row_shift, col_shift), axis=(0, 1))
            self._pad_shifted(data, row_shift, col_shift)
        self.variance[self.is_valid < 0.5] = self.initial_variance
        invalid = (self.is_valid + self.is_upper_bound) < 0.5
        self.normal_x[invalid] = 0.0
        self.normal_y[invalid] = 0.0
        self.normal_z[invalid] = 1.0

    def _pad_shifted(self, data: np.ndarray, row_shift: int, col_shift: int) -> None:
        if row_shift > 0:
            data[:row_shift, :] = 0.0
        elif row_shift < 0:
            data[row_shift:, :] = 0.0
        if col_shift > 0:
            data[:, :col_shift] = 0.0
        elif col_shift < 0:
            data[:, col_shift:] = 0.0

    def _to_grid_map_convention(self, data: np.ndarray) -> np.ndarray:
        return np.flip(np.flip(data.T, axis=0), axis=1).astype(np.float32, copy=False)

    def publish_map(self, key: str) -> None:
        cfg = self.publishers_cfg.get(key, {})
        layers = list(cfg.get('layers', []))
        basic_layers = list(cfg.get('basic_layers', []))
        if not layers:
            return

        self.layers = self._build_publish_layers()

        msg = GridMap()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.last_stamp if self.last_stamp is not None else self.get_clock().now().to_msg()
        msg.info.resolution = self.resolution
        msg.info.length_x = self.actual_map_length
        msg.info.length_y = self.actual_map_length
        msg.info.pose.position.x = float(self.center_x)
        msg.info.pose.position.y = float(self.center_y)
        msg.info.pose.position.z = 0.0
        msg.info.pose.orientation.w = 1.0
        msg.layers = layers
        msg.basic_layers = basic_layers
        msg.data = [
            encode_layer_to_multiarray(
                self.layers.get(layer, np.full((self.rows, self.cols), np.nan, dtype=np.float32))
            )
            for layer in layers
        ]
        msg.outer_start_index = 0
        msg.inner_start_index = 0
        self.map_publishers[key].publish(msg)


def main():
    rclpy.init()
    node = ElevationMappingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
