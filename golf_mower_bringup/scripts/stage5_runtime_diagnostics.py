#!/usr/bin/env python3

import math
import statistics

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from grid_map_msgs.msg import GridMap
except ImportError:
    GridMap = None


class Stage5RuntimeDiagnostics(Node):
    def __init__(self):
        super().__init__('stage5_runtime_diagnostics')

        self.declare_parameter('cloud_topic', '/unilidar/cloud')
        self.declare_parameter('imu_topic', '/unilidar/imu')
        self.declare_parameter('odom_topic', '/pointlio/odom')
        self.declare_parameter('ground_topic', '/ground_segmentation/ground')
        self.declare_parameter('elevation_input_topic', '/golf_mower/ground_cloud_for_elevation')
        self.declare_parameter('static_map_topic', '/map')
        self.declare_parameter('elevation_map_raw_topic', '/elevation_mapping_node/elevation_map_raw')
        self.declare_parameter('elevation_map_filter_topic', '/elevation_mapping_node/elevation_map_filter')
        self.declare_parameter('traversability_grid_topic', '/elevation/traversability_grid')
        self.declare_parameter('registered_cloud_topic', '/pointlio/cloud_registered')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_init_frame', 'camera_init')
        self.declare_parameter('pointlio_body_frame', 'aft_mapped')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'unilidar_lidar')
        self.declare_parameter('imu_frame', 'unilidar_imu')
        self.declare_parameter('imu_window_size', 300)
        self.declare_parameter('cloud_print_every_n', 10)
        self.declare_parameter('ground_print_every_n', 10)
        self.declare_parameter('odom_check_interval', 0.2)
        self.declare_parameter('odom_jump_distance', 0.1)
        self.declare_parameter('odom_jump_speed', 1.0)
        self.declare_parameter('tf_check_period', 1.0)
        self.declare_parameter('graph_check_period', 5.0)
        self.declare_parameter('topic_timeout_sec', 30.0)

        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.ground_topic = self.get_parameter('ground_topic').value
        self.elevation_input_topic = self.get_parameter('elevation_input_topic').value
        self.static_map_topic = self.get_parameter('static_map_topic').value
        self.elevation_map_raw_topic = self.get_parameter('elevation_map_raw_topic').value
        self.elevation_map_filter_topic = self.get_parameter('elevation_map_filter_topic').value
        self.traversability_grid_topic = self.get_parameter('traversability_grid_topic').value
        self.registered_cloud_topic = self.get_parameter('registered_cloud_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.camera_init_frame = self.get_parameter('camera_init_frame').value
        self.pointlio_body_frame = self.get_parameter('pointlio_body_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.imu_window_size = max(10, self.get_parameter('imu_window_size').value)
        self.cloud_print_every_n = max(1, self.get_parameter('cloud_print_every_n').value)
        self.ground_print_every_n = max(1, self.get_parameter('ground_print_every_n').value)
        self.odom_check_interval = max(0.01, self.get_parameter('odom_check_interval').value)
        self.odom_jump_distance = max(0.0, self.get_parameter('odom_jump_distance').value)
        self.odom_jump_speed = max(0.0, self.get_parameter('odom_jump_speed').value)
        self.tf_check_period = max(0.2, self.get_parameter('tf_check_period').value)
        self.graph_check_period = max(1.0, self.get_parameter('graph_check_period').value)
        self.topic_timeout_sec = max(5.0, self.get_parameter('topic_timeout_sec').value)

        self.cloud_count = 0
        self.ground_count = 0
        self.elevation_input_count = 0
        self.imu_samples = []
        self.last_cloud_stamp = None
        self.last_cloud_receipt_time = None
        self.last_imu_stamp = None
        self.last_imu_receipt_time = None
        self.last_odom_check = None
        self.last_odom_stamp = None
        self.last_odom_receipt_time = None
        self.last_map_stamp = None
        self.last_map_receipt_time = None
        self.last_elevation_raw_stamp = None
        self.last_elevation_raw_receipt_time = None
        self.last_elevation_filter_stamp = None
        self.last_elevation_filter_receipt_time = None
        self.last_traversability_stamp = None
        self.last_traversability_receipt_time = None
        self.warned_missing_grid_map_msg = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_callback, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 200)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 50)
        self.create_subscription(PointCloud2, self.ground_topic, self.ground_callback, 10)
        self.create_subscription(PointCloud2, self.elevation_input_topic, self.elevation_input_callback, 10)
        self.create_subscription(OccupancyGrid, self.static_map_topic, self.static_map_callback, 10)
        if GridMap is not None:
            self.create_subscription(GridMap, self.elevation_map_raw_topic, self.elevation_raw_callback, 10)
            self.create_subscription(GridMap, self.elevation_map_filter_topic, self.elevation_filter_callback, 10)
        else:
            self.warned_missing_grid_map_msg = True
            self.get_logger().warn('grid_map_msgs is not importable; GridMap topics will not be inspected')
        self.create_subscription(
            OccupancyGrid,
            self.traversability_grid_topic,
            self.traversability_grid_callback,
            10)
        self.create_timer(self.tf_check_period, self.tf_timer_callback)
        self.create_timer(self.graph_check_period, self.graph_timer_callback)
        self.create_timer(5.0, self.topic_timeout_callback)

        self.get_logger().info(
            'listening '
            f'cloud={self.cloud_topic}, imu={self.imu_topic}, odom={self.odom_topic}, '
            f'ground={self.ground_topic}, elevation_input={self.elevation_input_topic}, '
            f'map={self.static_map_topic}, '
            f'elevation_raw={self.elevation_map_raw_topic}, elevation_filter={self.elevation_map_filter_topic}, '
            f'registered_cloud={self.registered_cloud_topic}, '
            f'traversability={self.traversability_grid_topic}'
        )

    @staticmethod
    def stamp_to_float(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    @staticmethod
    def point_count(msg):
        return msg.width * msg.height

    def cloud_callback(self, msg):
        self.last_cloud_receipt_time = self.now_sec()
        self.cloud_count += 1
        if self.cloud_count % self.cloud_print_every_n != 0:
            return

        field_names = [field.name for field in msg.fields]
        required = {'x', 'y', 'z', 'time', 'ring'}
        missing = sorted(required - set(field_names))
        if missing:
            self.get_logger().error(
                f'cloud frame={msg.header.frame_id} fields={field_names}; '
                f'missing Point-LIO fields={missing}')
            return

        times = []
        rings = []
        xyz_count = 0
        for point in point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z', 'time', 'ring'), skip_nans=True
        ):
            x, y, z, t, ring = point
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                xyz_count += 1
                times.append(float(t))
                rings.append(int(ring))

        stamp = self.stamp_to_float(msg.header.stamp)
        self.last_cloud_stamp = stamp
        if not times:
            self.get_logger().warn(
                f'cloud frame={msg.header.frame_id} stamp={stamp:.9f} has no valid x/y/z/time/ring points')
            return

        imu_delta = 'n/a'
        if self.last_imu_stamp is not None:
            imu_delta = f'{self.last_imu_stamp - stamp:.6f}s'
        self.get_logger().info(
            'cloud '
            f'points={xyz_count} frame={msg.header.frame_id} stamp={stamp:.9f} '
            f'time_span={max(times) - min(times):.9f}s '
            f'ring=[{min(rings)}, {max(rings)}] imu_delta={imu_delta}'
        )

    def imu_callback(self, msg):
        self.last_imu_receipt_time = self.now_sec()
        self.last_imu_stamp = self.stamp_to_float(msg.header.stamp)
        acc = msg.linear_acceleration
        gyro = msg.angular_velocity
        acc_norm = math.sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z)
        gyro_norm = math.sqrt(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z)
        self.imu_samples.append((acc.x, acc.y, acc.z, acc_norm, gyro.x, gyro.y, gyro.z, gyro_norm))
        if len(self.imu_samples) > self.imu_window_size:
            self.imu_samples.pop(0)
        if len(self.imu_samples) == self.imu_window_size:
            mean = [statistics.fmean(sample[i] for sample in self.imu_samples) for i in range(8)]
            cloud_delta = 'n/a'
            if self.last_cloud_stamp is not None:
                cloud_delta = f'{self.last_imu_stamp - self.last_cloud_stamp:.6f}s'
            level = self.get_logger().info
            if mean[3] < 8.5 or mean[3] > 10.5:
                level = self.get_logger().warn
            level(
                'imu window '
                f'stamp={self.last_imu_stamp:.9f} '
                f'acc_mean=[{mean[0]:.4f}, {mean[1]:.4f}, {mean[2]:.4f}] '
                f'acc_norm_mean={mean[3]:.4f} '
                f'gyro_mean=[{mean[4]:.4f}, {mean[5]:.4f}, {mean[6]:.4f}] '
                f'gyro_norm_mean={mean[7]:.4f} cloud_delta={cloud_delta}'
            )
            self.imu_samples.clear()

    def odom_callback(self, msg):
        self.last_odom_receipt_time = self.now_sec()
        stamp = self.stamp_to_float(msg.header.stamp)
        self.last_odom_stamp = stamp
        pos = msg.pose.pose.position
        if self.last_odom_check is None:
            self.last_odom_check = (stamp, pos.x, pos.y, pos.z)
            return

        last_stamp, last_x, last_y, last_z = self.last_odom_check
        dt = stamp - last_stamp
        if dt < self.odom_check_interval:
            return

        dist = math.sqrt((pos.x - last_x) ** 2 + (pos.y - last_y) ** 2 + (pos.z - last_z) ** 2)
        speed = dist / dt if dt > 0.0 else float('inf')
        if dist > self.odom_jump_distance and speed > self.odom_jump_speed:
            self.get_logger().warn(
                f'odom jump: dt={dt:.4f}s dist={dist:.3f}m speed={speed:.3f}m/s '
                f'frame={msg.header.frame_id}->{msg.child_frame_id} '
                f'pos=[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]')
        self.last_odom_check = (stamp, pos.x, pos.y, pos.z)

    def ground_callback(self, msg):
        self.ground_count += 1
        if self.ground_count % self.ground_print_every_n == 0:
            self.get_logger().info(
                f'patchwork ground points={self.point_count(msg)} frame={msg.header.frame_id} '
                f'stamp={self.stamp_to_float(msg.header.stamp):.9f}')

    def elevation_input_callback(self, msg):
        self.elevation_input_count += 1
        if self.elevation_input_count % self.ground_print_every_n == 0:
            self.get_logger().info(
                f'elevation input points={self.point_count(msg)} frame={msg.header.frame_id} '
                f'stamp={self.stamp_to_float(msg.header.stamp):.9f}')

    def static_map_callback(self, msg):
        self.last_map_receipt_time = self.now_sec()
        stamp = self.stamp_to_float(msg.header.stamp)
        if self.last_map_stamp == stamp:
            return
        self.last_map_stamp = stamp
        known = sum(1 for value in msg.data if value >= 0)
        occupied = sum(1 for value in msg.data if value > 50)
        self.get_logger().info(
            f'static map frame={msg.header.frame_id} stamp={stamp:.9f} '
            f'size={msg.info.width}x{msg.info.height} resolution={msg.info.resolution:.3f} '
            f'known={known} occupied={occupied}')

    def elevation_raw_callback(self, msg):
        self.last_elevation_raw_receipt_time = self.now_sec()
        stamp = self.stamp_to_float(msg.header.stamp)
        if self.last_elevation_raw_stamp == stamp:
            return
        self.last_elevation_raw_stamp = stamp
        self.get_logger().info(
            f'elevation raw frame={msg.header.frame_id} stamp={stamp:.9f} '
            f'layers={list(msg.layers)} size={msg.info.length_x:.2f}x{msg.info.length_y:.2f} '
            f'resolution={msg.info.resolution:.3f}')

    def elevation_filter_callback(self, msg):
        self.last_elevation_filter_receipt_time = self.now_sec()
        stamp = self.stamp_to_float(msg.header.stamp)
        if self.last_elevation_filter_stamp == stamp:
            return
        self.last_elevation_filter_stamp = stamp
        self.get_logger().info(
            f'elevation filter frame={msg.header.frame_id} stamp={stamp:.9f} '
            f'layers={list(msg.layers)} size={msg.info.length_x:.2f}x{msg.info.length_y:.2f} '
            f'resolution={msg.info.resolution:.3f}')

    def traversability_grid_callback(self, msg):
        self.last_traversability_receipt_time = self.now_sec()
        stamp = self.stamp_to_float(msg.header.stamp)
        if self.last_traversability_stamp == stamp:
            return
        self.last_traversability_stamp = stamp
        known = sum(1 for value in msg.data if value >= 0)
        occupied = sum(1 for value in msg.data if value > 50)
        self.get_logger().info(
            f'traversability grid frame={msg.header.frame_id} stamp={stamp:.9f} '
            f'size={msg.info.width}x{msg.info.height} resolution={msg.info.resolution:.3f} '
            f'known={known} occupied={occupied}')

    def topic_timeout_callback(self):
        now = self.now_sec()
        checks = [
            ('cloud', self.last_cloud_receipt_time, self.last_cloud_stamp),
            ('imu', self.last_imu_receipt_time, self.last_imu_stamp),
            ('odom', self.last_odom_receipt_time, self.last_odom_stamp),
            ('static map', self.last_map_receipt_time, self.last_map_stamp),
            ('elevation raw GridMap', self.last_elevation_raw_receipt_time, self.last_elevation_raw_stamp),
            ('elevation filter GridMap', self.last_elevation_filter_receipt_time, self.last_elevation_filter_stamp),
            ('traversability OccupancyGrid', self.last_traversability_receipt_time, self.last_traversability_stamp),
        ]
        for name, receipt_time, header_stamp in checks:
            if receipt_time is None:
                self.get_logger().warn(f'No {name} message received yet')
            elif now - receipt_time > self.topic_timeout_sec:
                self.get_logger().warn(
                    f'{name} message is stale: receipt_age={now - receipt_time:.1f}s '
                    f'last_header_stamp={header_stamp:.9f}')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def graph_timer_callback(self):
        node_names = set(self.get_node_names())
        interesting_nodes = [
            'unitree_lidar_ros2_node',
            'laserMapping',
            'elevation_mapping_node',
            'ground_segmentation',
            'pointcloud_ground_fallback',
            'rviz2_outdoor_elevation',
            'controller_server',
        ]
        present = [name for name in interesting_nodes if name in node_names]
        missing = [name for name in interesting_nodes if name not in node_names]
        self.get_logger().info(
            f'nodes present={present} missing={missing}')

        for topic in [
            self.cloud_topic,
            self.registered_cloud_topic,
            self.ground_topic,
            self.elevation_input_topic,
            self.elevation_map_raw_topic,
            self.elevation_map_filter_topic,
            self.traversability_grid_topic,
        ]:
            self.log_topic_endpoints(topic)

    def log_topic_endpoints(self, topic):
        publishers = self.get_publishers_info_by_topic(topic)
        subscriptions = self.get_subscriptions_info_by_topic(topic)
        pub_text = ', '.join(self.endpoint_summary(info) for info in publishers) or 'none'
        sub_text = ', '.join(self.endpoint_summary(info) for info in subscriptions) or 'none'
        self.get_logger().info(
            f'topic {topic}: publishers={len(publishers)} [{pub_text}] '
            f'subscriptions={len(subscriptions)} [{sub_text}]')

    @staticmethod
    def endpoint_summary(info):
        qos = info.qos_profile
        reliability = getattr(qos.reliability, 'name', str(qos.reliability))
        durability = getattr(qos.durability, 'name', str(qos.durability))
        return (
            f'{info.node_name}({info.node_namespace}) '
            f'qos={reliability}/{durability}/depth={qos.depth}'
        )

    def tf_timer_callback(self):
        pairs = [
            (self.map_frame, self.camera_init_frame),
            (self.camera_init_frame, self.pointlio_body_frame),
            (self.pointlio_body_frame, self.base_frame),
            (self.base_frame, self.lidar_frame),
            (self.base_frame, self.imu_frame),
            (self.map_frame, self.base_frame),
            (self.map_frame, self.lidar_frame),
        ]
        for target, source in pairs:
            self.check_tf(target, source)

    def check_tf(self, target, source):
        try:
            transform = self.tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except TransformException as exc:
            self.get_logger().warn(f'No TF {target} <- {source}: {exc}')
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        dist_xy = math.hypot(t.x, t.y)
        level = self.get_logger().info
        if (target, source) in [
            (self.camera_init_frame, self.pointlio_body_frame),
            (self.map_frame, self.base_frame),
        ] and (dist_xy > 5.0 or abs(t.z) > 2.0):
            level = self.get_logger().warn
        level(
            f'TF {target} <- {source}: '
            f'trans=[{t.x:.3f}, {t.y:.3f}, {t.z:.3f}] '
            f'xy={dist_xy:.3f} yaw={math.degrees(yaw):.2f}deg '
            f'stamp={self.stamp_to_float(transform.header.stamp):.9f}')


def main():
    rclpy.init()
    node = Stage5RuntimeDiagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
