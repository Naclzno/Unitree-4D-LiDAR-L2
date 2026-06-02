#!/usr/bin/env python3

import math
import statistics

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2


class PointlioInputDiagnostics(Node):
    def __init__(self):
        super().__init__('pointlio_input_diagnostics')

        self.declare_parameter('cloud_topic', '/unilidar/cloud')
        self.declare_parameter('imu_topic', '/unilidar/imu')
        self.declare_parameter('odom_topic', '/pointlio/odom')
        self.declare_parameter('cloud_print_every_n', 10)
        self.declare_parameter('imu_window_size', 300)
        self.declare_parameter('odom_check_interval', 0.2)
        self.declare_parameter('odom_jump_distance', 0.1)
        self.declare_parameter('odom_jump_speed', 1.0)

        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cloud_print_every_n = max(1, self.get_parameter('cloud_print_every_n').value)
        self.imu_window_size = max(10, self.get_parameter('imu_window_size').value)
        self.odom_check_interval = max(0.01, self.get_parameter('odom_check_interval').value)
        self.odom_jump_distance = max(0.0, self.get_parameter('odom_jump_distance').value)
        self.odom_jump_speed = max(0.0, self.get_parameter('odom_jump_speed').value)

        self.cloud_count = 0
        self.imu_samples = []
        self.last_cloud_stamp = None
        self.last_imu_stamp = None
        self.last_odom_check = None

        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_callback, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 200)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 50)

        self.get_logger().info(
            f'listening cloud={self.cloud_topic}, imu={self.imu_topic}, odom={self.odom_topic}'
        )

    def cloud_callback(self, msg):
        self.cloud_count += 1
        if self.cloud_count % self.cloud_print_every_n != 0:
            return

        field_names = [field.name for field in msg.fields]
        required = {'x', 'y', 'z', 'time', 'ring'}
        missing = sorted(required - set(field_names))
        if missing:
            self.get_logger().error(
                f'cloud fields={field_names}; missing fields required by Point-LIO: {missing}'
            )
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

        if not times:
            self.get_logger().warn('cloud has no valid points with time/ring')
            return

        t_min = min(times)
        t_max = max(times)
        t_span = t_max - t_min
        ring_min = min(rings)
        ring_max = max(rings)
        header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_cloud_stamp = header_stamp
        imu_delta_text = 'imu_delta=n/a'
        if self.last_imu_stamp is not None:
            imu_delta_text = f'imu_delta={self.last_imu_stamp - header_stamp:.6f}s'

        self.get_logger().info(
            'cloud '
            f'points={xyz_count} frame={msg.header.frame_id} stamp={header_stamp:.9f} '
            f'fields={field_names} '
            f'time[min,max,span]=[{t_min:.9f}, {t_max:.9f}, {t_span:.9f}] '
            f'ring[min,max]=[{ring_min}, {ring_max}] {imu_delta_text}'
        )

    def imu_callback(self, msg):
        self.last_imu_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        acc = msg.linear_acceleration
        gyro = msg.angular_velocity
        acc_norm = math.sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z)
        gyro_norm = math.sqrt(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z)
        self.imu_samples.append((acc.x, acc.y, acc.z, acc_norm, gyro.x, gyro.y, gyro.z, gyro_norm))
        if len(self.imu_samples) > self.imu_window_size:
            self.imu_samples.pop(0)
        if len(self.imu_samples) == self.imu_window_size:
            mean = [statistics.fmean(sample[i] for sample in self.imu_samples) for i in range(8)]
            cloud_delta_text = 'cloud_delta=n/a'
            if self.last_cloud_stamp is not None:
                cloud_delta_text = f'cloud_delta={self.last_imu_stamp - self.last_cloud_stamp:.6f}s'
            self.get_logger().info(
                'imu window '
                f'stamp={self.last_imu_stamp:.9f} '
                f'acc_mean=[{mean[0]:.4f}, {mean[1]:.4f}, {mean[2]:.4f}] '
                f'acc_norm_mean={mean[3]:.4f} '
                f'gyro_mean=[{mean[4]:.4f}, {mean[5]:.4f}, {mean[6]:.4f}] '
                f'gyro_norm_mean={mean[7]:.4f} {cloud_delta_text}'
            )
            self.imu_samples.clear()

    def odom_callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.pose.position
        if self.last_odom_check is None:
            self.last_odom_check = (stamp, pos.x, pos.y, pos.z)
            return

        last_stamp, last_x, last_y, last_z = self.last_odom_check
        dt = stamp - last_stamp
        if dt < self.odom_check_interval:
            return

        dist = math.sqrt((pos.x - last_x) ** 2 + (pos.y - last_y) ** 2 + (pos.z - last_z) ** 2)
        if dt > 0.0:
            speed = dist / dt
            if dist > self.odom_jump_distance and speed > self.odom_jump_speed:
                self.get_logger().warn(
                    f'odom jump: dt={dt:.4f}s dist={dist:.3f}m speed={speed:.3f}m/s '
                    f'pos=[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]'
                )
        self.last_odom_check = (stamp, pos.x, pos.y, pos.z)


def main():
    rclpy.init()
    node = PointlioInputDiagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
