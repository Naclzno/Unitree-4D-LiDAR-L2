#!/usr/bin/env python3

import json
import math
import os
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


EARTH_RADIUS_M = 6378137.0


class RtkMapLocalizer(Node):
    def __init__(self):
        super().__init__('rtk_map_localizer')

        self.declare_parameter('metadata_path', '/tmp/golf_mower_map_metadata.yaml')
        self.declare_parameter('fix_topic', '/fix')
        self.declare_parameter('odom_topic', '/pointlio/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_init_frame', 'camera_init')
        self.declare_parameter('map_to_camera_init_z', 0.0)
        self.declare_parameter('map_to_camera_init_yaw', 0.0)
        self.declare_parameter('use_heading_topic', False)
        self.declare_parameter('heading_topic', '/um981/heading')
        self.declare_parameter('heading_offset_deg', 0.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('publish_once', True)
        self.declare_parameter('min_fix_status', int(NavSatStatus.STATUS_FIX))

        self.metadata = _load_metadata(os.path.expanduser(str(self.get_parameter('metadata_path').value)))
        self.datum_lat = float(self.metadata['datum']['latitude'])
        self.datum_lon = float(self.metadata['datum']['longitude'])
        self.yaw_map_to_enu = float(self.metadata.get('alignment', {}).get('yaw_map_to_enu', 0.0))

        self.fix: Optional[NavSatFix] = None
        self.odom: Optional[Odometry] = None
        self.heading_deg: Optional[float] = None
        self.sent_once = False
        self.broadcaster = TransformBroadcaster(self)

        self.create_subscription(NavSatFix, str(self.get_parameter('fix_topic').value), self.fix_callback, 10)
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value), self.odom_callback, 20)
        if bool(self.get_parameter('use_heading_topic').value):
            self.create_subscription(
                Float64,
                str(self.get_parameter('heading_topic').value),
                self.heading_callback,
                10,
            )
        self.timer = self.create_timer(
            1.0 / max(float(self.get_parameter('publish_rate').value), 0.1),
            self.publish_transform,
        )

        self.get_logger().info(
            'RTK map localizer active: '
            f'datum=[{self.datum_lat:.9f}, {self.datum_lon:.9f}], '
            f'yaw_map_to_enu={self.yaw_map_to_enu:.6f}, '
            f'use_heading_topic={bool(self.get_parameter("use_heading_topic").value)}.')

    def fix_callback(self, msg: NavSatFix):
        min_status = int(self.get_parameter('min_fix_status').value)
        if msg.status.status < min_status:
            self.get_logger().warn(
                f'Ignoring fix with status={msg.status.status}, required >= {min_status}',
                throttle_duration_sec=5.0,
            )
            return
        if not _valid_lat_lon(msg.latitude, msg.longitude):
            self.get_logger().warn('Ignoring invalid NavSatFix lat/lon', throttle_duration_sec=5.0)
            return
        self.fix = msg

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def heading_callback(self, msg: Float64):
        if not math.isfinite(float(msg.data)):
            self.get_logger().warn('Ignoring invalid UM981 heading', throttle_duration_sec=5.0)
            return
        self.heading_deg = float(msg.data) % 360.0

    def publish_transform(self):
        if self.sent_once and bool(self.get_parameter('publish_once').value):
            return
        if self.fix is None or self.odom is None:
            return

        map_x, map_y = self.fix_to_map_xy(self.fix.latitude, self.fix.longitude)
        yaw = self.resolve_map_yaw()
        if yaw is None:
            return
        odom_x = float(self.odom.pose.pose.position.x)
        odom_y = float(self.odom.pose.pose.position.y)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        tx = map_x - (cos_yaw * odom_x - sin_yaw * odom_y)
        ty = map_y - (sin_yaw * odom_x + cos_yaw * odom_y)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(self.get_parameter('map_frame').value)
        transform.child_frame_id = str(self.get_parameter('camera_init_frame').value)
        transform.transform.translation.x = tx
        transform.transform.translation.y = ty
        transform.transform.translation.z = float(self.get_parameter('map_to_camera_init_z').value)
        transform.transform.rotation.z = math.sin(0.5 * yaw)
        transform.transform.rotation.w = math.cos(0.5 * yaw)
        self.broadcaster.sendTransform(transform)

        if not self.sent_once:
            self.get_logger().info(
                f'Published initial map -> camera_init: x={tx:.3f}, y={ty:.3f}, yaw={yaw:.6f}; '
                f'fix map position=[{map_x:.3f}, {map_y:.3f}]')
        self.sent_once = True

    def resolve_map_yaw(self) -> Optional[float]:
        if not bool(self.get_parameter('use_heading_topic').value):
            return float(self.get_parameter('map_to_camera_init_yaw').value)
        if self.heading_deg is None:
            self.get_logger().warn(
                'Waiting for UM981 heading before publishing map -> camera_init',
                throttle_duration_sec=5.0,
            )
            return None
        heading = (self.heading_deg + float(self.get_parameter('heading_offset_deg').value)) % 360.0
        return heading_deg_to_map_yaw(heading, self.yaw_map_to_enu)

    def fix_to_map_xy(self, lat: float, lon: float):
        datum_lat_rad = math.radians(self.datum_lat)
        east = math.radians(lon - self.datum_lon) * EARTH_RADIUS_M * math.cos(datum_lat_rad)
        north = math.radians(lat - self.datum_lat) * EARTH_RADIUS_M

        cos_yaw = math.cos(self.yaw_map_to_enu)
        sin_yaw = math.sin(self.yaw_map_to_enu)
        map_x = cos_yaw * east + sin_yaw * north
        map_y = -sin_yaw * east + cos_yaw * north
        return map_x, map_y


def heading_deg_to_map_yaw(heading_deg: float, yaw_map_to_enu: float) -> float:
    heading_rad = math.radians(heading_deg)
    east = math.sin(heading_rad)
    north = math.cos(heading_rad)

    cos_yaw = math.cos(yaw_map_to_enu)
    sin_yaw = math.sin(yaw_map_to_enu)
    map_x = cos_yaw * east + sin_yaw * north
    map_y = -sin_yaw * east + cos_yaw * north
    return math.atan2(map_y, map_x)


def _load_metadata(path: str):
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path, 'r', encoding='utf-8') as metadata:
        return json.load(metadata)


def _valid_lat_lon(lat: float, lon: float) -> bool:
    return math.isfinite(lat) and math.isfinite(lon) and not (abs(lat) < 1e-12 and abs(lon) < 1e-12)


def main():
    rclpy.init()
    node = RtkMapLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
