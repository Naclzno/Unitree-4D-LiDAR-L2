from __future__ import annotations

import math
import threading
from typing import Any

from um981.models import GnssFix, ImuSample
from um981.sdk import DEFAULT_COMMANDS, Um981Serial


def main() -> None:
    import rclpy

    rclpy.init()
    wrapper = Um981RosNode()
    try:
        rclpy.spin(wrapper.node)
    finally:
        wrapper.stop()
        wrapper.destroy_node()
        rclpy.shutdown()


class Um981RosNode:
    def __init__(self) -> None:
        import rclpy
        from geometry_msgs.msg import Vector3Stamped
        from sensor_msgs.msg import Imu, NavSatFix
        from std_msgs.msg import Float64, String

        self._rclpy = rclpy
        self._Imu = Imu
        self._NavSatFix = NavSatFix
        self._Float64 = Float64
        self._String = String
        self._Vector3Stamped = Vector3Stamped

        self.node = rclpy.create_node("um981_node")
        self.node.declare_parameter("port", "/dev/ttyUSB0")
        self.node.declare_parameter("baud", 115200)
        self.node.declare_parameter("timeout", 0.2)
        self.node.declare_parameter("read_size", 4096)
        self.node.declare_parameter("frame_id", "gnss_link")
        self.node.declare_parameter("imu_frame_id", "imu_link")
        self.node.declare_parameter("heading_topic", "/um981/heading")
        self.node.declare_parameter("attitude_topic", "/um981/ins_attitude")
        self.node.declare_parameter("fix_quality_topic", "/um981/fix_quality")
        self.node.declare_parameter("require_rtk_fixed", True)
        self.node.declare_parameter("allow_rtk_float", False)
        self.node.declare_parameter("min_satellites", 10)
        self.node.declare_parameter("max_hdop", 1.5)
        self.node.declare_parameter("hdop_to_horizontal_stddev_m", 1.0)
        self.node.declare_parameter("commands", list(DEFAULT_COMMANDS))

        self.fix_pub = self.node.create_publisher(NavSatFix, "/fix", 10)
        self.imu_pub = self.node.create_publisher(Imu, "/imu/data_raw", 50)
        self.heading_pub = self.node.create_publisher(Float64, str(self._param("heading_topic")), 10)
        self.fix_quality_pub = self.node.create_publisher(
            String,
            str(self._param("fix_quality_topic")),
            10,
        )
        self.attitude_pub = self.node.create_publisher(
            Vector3Stamped,
            str(self._param("attitude_topic")),
            10,
        )

        self._receiver: Um981Serial | None = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop, name="um981-reader", daemon=True)
        self._thread.start()

    def get_logger(self) -> Any:
        return self.node.get_logger()

    def destroy_node(self) -> None:
        self.node.destroy_node()

    def stop(self) -> None:
        self._stop_event.set()
        if self._receiver is not None:
            self._receiver.close()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _param(self, name: str) -> Any:
        return self.node.get_parameter(name).value

    def _reader_loop(self) -> None:
        port = str(self._param("port"))
        baud = int(self._param("baud"))
        timeout = float(self._param("timeout"))
        read_size = int(self._param("read_size"))
        commands = list(self._param("commands") or [])

        try:
            self._receiver = Um981Serial(
                port=port,
                baud=baud,
                timeout=timeout,
                read_size=read_size,
                commands=commands,
            )
            self._receiver.open()
            self.get_logger().info(f"opened UM981 serial port {port} at {baud} baud")
            for observation in self._receiver.iter_observations(include_unknown=False):
                if self._stop_event.is_set():
                    break
                self._publish_observation(observation.payload)
        except Exception as exc:
            self.get_logger().error(f"UM981 reader stopped: {exc}")

    def _publish_observation(self, payload: Any) -> None:
        if isinstance(payload, GnssFix):
            self.fix_pub.publish(self._fix_msg(payload))
            self._publish_attitude(payload)
        elif isinstance(payload, ImuSample):
            self.imu_pub.publish(self._imu_msg(payload))

    def _stamp(self) -> Any:
        return self.node.get_clock().now().to_msg()

    def _fix_msg(self, fix: GnssFix) -> Any:
        from sensor_msgs.msg import NavSatStatus

        msg = self._NavSatFix()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = str(self._param("frame_id"))
        msg.status.service = NavSatStatus.SERVICE_GPS
        accepted, reason = self._fix_is_accepted(fix)
        msg.status.status = NavSatStatus.STATUS_FIX if accepted else NavSatStatus.STATUS_NO_FIX
        msg.latitude = float(fix.lat_deg or 0.0)
        msg.longitude = float(fix.lon_deg or 0.0)
        msg.altitude = float(fix.height_m or 0.0)

        hdop_scale = float(self._param("hdop_to_horizontal_stddev_m"))
        if fix.hdop is not None and math.isfinite(float(fix.hdop)) and hdop_scale > 0.0:
            horizontal_stddev = float(fix.hdop) * hdop_scale
            horizontal_variance = horizontal_stddev ** 2
            vertical_variance = (2.0 * horizontal_stddev) ** 2
            msg.position_covariance = [
                horizontal_variance, 0.0, 0.0,
                0.0, horizontal_variance, 0.0,
                0.0, 0.0, vertical_variance,
            ]
            msg.position_covariance_type = self._NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            msg.position_covariance_type = self._NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._publish_fix_quality(fix, accepted, reason)
        return msg

    def _fix_is_accepted(self, fix: GnssFix) -> tuple[bool, str]:
        if not _valid_lat_lon(fix.lat_deg, fix.lon_deg):
            return False, "invalid_lat_lon"

        quality = str(fix.fix_quality or "unknown").strip().lower()
        require_rtk_fixed = bool(self._param("require_rtk_fixed"))
        allow_rtk_float = bool(self._param("allow_rtk_float"))
        if require_rtk_fixed:
            accepted_qualities = {"rtk_fixed"}
            if allow_rtk_float:
                accepted_qualities.add("rtk_float")
            if quality not in accepted_qualities:
                return False, f"quality={quality}"

        min_satellites = int(self._param("min_satellites"))
        if min_satellites > 0:
            if fix.satellites_used is None or int(fix.satellites_used) < min_satellites:
                return False, f"satellites={fix.satellites_used}<{min_satellites}"

        max_hdop = float(self._param("max_hdop"))
        if max_hdop > 0.0:
            if fix.hdop is None or not math.isfinite(float(fix.hdop)) or float(fix.hdop) > max_hdop:
                return False, f"hdop={fix.hdop}>{max_hdop}"
        return True, "accepted"

    def _publish_fix_quality(self, fix: GnssFix, accepted: bool, reason: str) -> None:
        quality = self._String()
        quality.data = (
            f"accepted={str(accepted).lower()} reason={reason} "
            f"source={fix.source} quality={fix.fix_quality or 'unknown'} "
            f"satellites={fix.satellites_used if fix.satellites_used is not None else 'unknown'} "
            f"hdop={fix.hdop if fix.hdop is not None else 'unknown'}"
        )
        self.fix_quality_pub.publish(quality)

    def _imu_msg(self, sample: ImuSample) -> Any:
        msg = self._Imu()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = str(self._param("imu_frame_id"))
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = sample.gyro_radps.x
        msg.angular_velocity.y = sample.gyro_radps.y
        msg.angular_velocity.z = sample.gyro_radps.z
        msg.linear_acceleration.x = sample.accel_mps2.x
        msg.linear_acceleration.y = sample.accel_mps2.y
        msg.linear_acceleration.z = sample.accel_mps2.z
        return msg

    def _publish_attitude(self, fix: GnssFix) -> None:
        if fix.heading_deg is None:
            return
        if not math.isfinite(float(fix.heading_deg)):
            return

        heading = self._Float64()
        heading.data = _normalize_heading_deg(float(fix.heading_deg))
        self.heading_pub.publish(heading)

        attitude = self._Vector3Stamped()
        attitude.header.stamp = self._stamp()
        attitude.header.frame_id = str(self._param("imu_frame_id"))
        attitude.vector.x = float(fix.roll_deg or 0.0)
        attitude.vector.y = float(fix.pitch_deg or 0.0)
        attitude.vector.z = heading.data
        self.attitude_pub.publish(attitude)

def _valid_lat_lon(lat: float | None, lon: float | None) -> bool:
    if lat is None or lon is None:
        return False
    return math.isfinite(lat) and math.isfinite(lon) and not (abs(lat) < 1e-12 and abs(lon) < 1e-12)


def _normalize_heading_deg(value: float) -> float:
    return value % 360.0
