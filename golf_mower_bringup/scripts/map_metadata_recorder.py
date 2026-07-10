#!/usr/bin/env python3

import json
import math
import os
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class MapMetadataRecorder(Node):
    def __init__(self):
        super().__init__('map_metadata_recorder')

        self.declare_parameter('fix_topic', '/fix')
        self.declare_parameter('odom_topic', '/pointlio/odom')
        self.declare_parameter('output_path', '/tmp/golf_mower_map_metadata.yaml')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('pointlio_map_frame', 'camera_init')
        self.declare_parameter('pointlio_body_frame', 'aft_mapped')
        self.declare_parameter('yaw_map_to_enu', 0.0)
        self.declare_parameter('write_period_sec', 2.0)
        self.declare_parameter('overwrite', False)

        self.fix: Optional[NavSatFix] = None
        self.odom: Optional[Odometry] = None
        self.written = False
        self.output_path = os.path.expanduser(str(self.get_parameter('output_path').value))

        self.create_subscription(
            NavSatFix,
            str(self.get_parameter('fix_topic').value),
            self.fix_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            20,
        )
        self.timer = self.create_timer(
            max(float(self.get_parameter('write_period_sec').value), 0.1),
            self.try_write,
        )

        self.get_logger().info(
            f'Recording map metadata to {self.output_path}; waiting for '
            f'{self.get_parameter("fix_topic").value} and {self.get_parameter("odom_topic").value}')

    def fix_callback(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            self.get_logger().warn('Ignoring NavSatFix without fix status', throttle_duration_sec=5.0)
            return
        if not _valid_lat_lon(msg.latitude, msg.longitude):
            self.get_logger().warn('Ignoring invalid NavSatFix lat/lon', throttle_duration_sec=5.0)
            return
        self.fix = msg
        self.try_write()

    def odom_callback(self, msg: Odometry):
        self.odom = msg
        self.try_write()

    def try_write(self):
        if self.written and not bool(self.get_parameter('overwrite').value):
            return
        if self.fix is None or self.odom is None:
            return

        data = {
            'version': 1,
            'map_frame': str(self.get_parameter('map_frame').value),
            'pointlio_map_frame': str(self.get_parameter('pointlio_map_frame').value),
            'pointlio_body_frame': str(self.get_parameter('pointlio_body_frame').value),
            'datum': {
                'latitude': float(self.fix.latitude),
                'longitude': float(self.fix.longitude),
                'altitude': float(self.fix.altitude),
                'frame_id': self.fix.header.frame_id,
                'stamp': _stamp_to_float(self.fix.header.stamp),
            },
            'alignment': {
                'yaw_map_to_enu': float(self.get_parameter('yaw_map_to_enu').value),
                'note': 'yaw_map_to_enu is the yaw of the offline map x-axis in ENU radians.',
            },
            'mapping_start_pointlio_pose': {
                'frame_id': self.odom.header.frame_id,
                'child_frame_id': self.odom.child_frame_id,
                'x': float(self.odom.pose.pose.position.x),
                'y': float(self.odom.pose.pose.position.y),
                'z': float(self.odom.pose.pose.position.z),
                'yaw': _yaw_from_quaternion(self.odom.pose.pose.orientation),
                'stamp': _stamp_to_float(self.odom.header.stamp),
            },
        }

        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        with open(self.output_path, 'w', encoding='utf-8') as metadata:
            json.dump(data, metadata, indent=2, ensure_ascii=False)
            metadata.write('\n')
        self.written = True
        self.get_logger().info(
            'Wrote map metadata: '
            f'datum=[{data["datum"]["latitude"]:.9f}, {data["datum"]["longitude"]:.9f}], '
            f'yaw_map_to_enu={data["alignment"]["yaw_map_to_enu"]:.6f}')


def _valid_lat_lon(lat: float, lon: float) -> bool:
    return math.isfinite(lat) and math.isfinite(lon) and not (abs(lat) < 1e-12 and abs(lon) < 1e-12)


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    rclpy.init()
    node = MapMetadataRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
