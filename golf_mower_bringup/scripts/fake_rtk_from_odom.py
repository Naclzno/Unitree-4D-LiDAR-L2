#!/usr/bin/env python3

import math
import json
import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


EARTH_RADIUS_M = 6378137.0


class FakeRtkFromOdom(Node):
    def __init__(self):
        super().__init__('fake_rtk_from_odom')

        self.declare_parameter('odom_topic', '/pointlio/odom')
        self.declare_parameter('fix_topic', '/fix')
        self.declare_parameter('frame_id', 'rtk_antenna')
        self.declare_parameter('datum_lat_deg', 39.771981522833336)
        self.declare_parameter('datum_lon_deg', 116.353032825)
        self.declare_parameter('datum_alt_m', 79.0)
        self.declare_parameter('metadata_path', '')
        self.declare_parameter('position_covariance_m2', 0.04)
        self.declare_parameter('publish_no_fix_until_odom', False)

        metadata_path = os.path.expanduser(str(self.get_parameter('metadata_path').value))
        datum_lat = float(self.get_parameter('datum_lat_deg').value)
        datum_lon = float(self.get_parameter('datum_lon_deg').value)
        datum_alt = float(self.get_parameter('datum_alt_m').value)
        if metadata_path and os.path.exists(metadata_path):
            with open(metadata_path, 'r', encoding='utf-8') as metadata:
                data = json.load(metadata)
            datum = data.get('datum', {})
            datum_lat = float(datum['latitude'])
            datum_lon = float(datum['longitude'])
            datum_alt = float(datum.get('altitude', 0.0))

        self.datum_lat = datum_lat
        self.datum_lon = datum_lon
        self.datum_alt = datum_alt
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.covariance = float(self.get_parameter('position_covariance_m2').value)

        self.publisher = self.create_publisher(
            NavSatFix,
            str(self.get_parameter('fix_topic').value),
            10,
        )
        self.subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            20,
        )

        self.get_logger().info(
            'Publishing fake RTK NavSatFix from odometry: '
            f'odom={self.get_parameter("odom_topic").value}, '
            f'fix={self.get_parameter("fix_topic").value}, '
            f'datum=[{self.datum_lat:.9f}, {self.datum_lon:.9f}, {self.datum_alt:.3f}]'
        )

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        lat, lon = self.local_xy_to_lat_lon(x, y)

        fix = NavSatFix()
        fix.header.stamp = msg.header.stamp
        fix.header.frame_id = self.frame_id
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = self.datum_alt + z
        fix.position_covariance = [
            self.covariance, 0.0, 0.0,
            0.0, self.covariance, 0.0,
            0.0, 0.0, max(self.covariance, 1.0),
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.publisher.publish(fix)

    def local_xy_to_lat_lon(self, x_m: float, y_m: float):
        datum_lat_rad = math.radians(self.datum_lat)
        d_lat = y_m / EARTH_RADIUS_M
        d_lon = x_m / (EARTH_RADIUS_M * max(math.cos(datum_lat_rad), 1e-9))
        return (
            self.datum_lat + math.degrees(d_lat),
            self.datum_lon + math.degrees(d_lon),
        )


def main():
    rclpy.init()
    node = FakeRtkFromOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
