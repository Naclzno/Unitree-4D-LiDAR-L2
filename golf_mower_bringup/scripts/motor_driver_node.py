#!/usr/bin/env python3
"""Translate /cmd_vel into the documented V2 motor-controller serial protocol.

The protocol does not define a signed differential-speed encoding. This node
therefore uses only its documented discrete motion commands and intentionally
rejects reverse turns instead of inventing an unsafe byte encoding.
"""

import os
import termios
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


FRAME_HEAD = 0xAA
FRAME_TAIL = 0x55
CMD_FORWARD = 0x01
CMD_BACKWARD = 0x02
CMD_STOP = 0x03
CMD_BRAKE = 0x04
CMD_ARC_LEFT = 0x05
CMD_ARC_RIGHT = 0x06
CMD_SPIN_LEFT = 0x07
CMD_SPIN_RIGHT = 0x08
CMD_SET_SPEED = 0x0A
CMD_EMERGENCY_STOP = 0xFF


def make_frame(command: int, data: List[int] | None = None) -> bytes:
    """Build AA LEN CMD DATA... CHECKSUM 55 using the documented checksum."""
    payload = data or []
    if not 0 <= command <= 0xFF or any(not 0 <= value <= 0xFF for value in payload):
        raise ValueError('command and data must be byte values')
    frame = [FRAME_HEAD, 1 + len(payload), command, *payload]
    frame.extend([sum(frame) & 0xFF, FRAME_TAIL])
    return bytes(frame)


class MotorDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('motor_driver')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('port', '')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('enabled', False)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('require_fresh_cmd_after_arm', True)
        self.declare_parameter('command_rate_hz', 10.0)
        self.declare_parameter('cmd_vel_timeout_sec', 0.50)
        self.declare_parameter('stop_mode', 'brake')
        self.declare_parameter('max_linear_speed_mps', 0.45)
        self.declare_parameter('max_angular_speed_radps', 0.70)
        self.declare_parameter('min_speed_percent', 10)
        self.declare_parameter('max_speed_percent', 30)
        self.declare_parameter('linear_deadband_mps', 0.03)
        self.declare_parameter('angular_deadband_radps', 0.10)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.require_fresh_cmd_after_arm = bool(
            self.get_parameter('require_fresh_cmd_after_arm').value)
        requested_enabled = bool(self.get_parameter('enabled').value)
        # Physical output must always be armed explicitly through the service.
        # `enabled:=true` is retained only to make dry-run protocol tests easy.
        self.armed = requested_enabled if self.dry_run else False
        self.command_rate_hz = float(self.get_parameter('command_rate_hz').value)
        self.timeout_sec = float(self.get_parameter('cmd_vel_timeout_sec').value)
        self.stop_mode = str(self.get_parameter('stop_mode').value).lower()
        self.max_linear = float(self.get_parameter('max_linear_speed_mps').value)
        self.max_angular = float(self.get_parameter('max_angular_speed_radps').value)
        self.min_percent = int(self.get_parameter('min_speed_percent').value)
        self.max_percent = int(self.get_parameter('max_speed_percent').value)
        self.linear_deadband = float(self.get_parameter('linear_deadband_mps').value)
        self.angular_deadband = float(self.get_parameter('angular_deadband_radps').value)

        if self.command_rate_hz <= 0.0 or self.timeout_sec <= 0.0:
            raise ValueError('command_rate_hz and cmd_vel_timeout_sec must be positive')
        if self.max_linear <= 0.0 or self.max_angular <= 0.0:
            raise ValueError('maximum linear and angular speeds must be positive')
        if not 0 <= self.min_percent <= self.max_percent <= 100:
            raise ValueError('speed percentages must satisfy 0 <= min <= max <= 100')
        if self.stop_mode not in ('stop', 'brake', 'emergency_stop'):
            raise ValueError('stop_mode must be stop, brake, or emergency_stop')

        self.serial_fd: Optional[int] = None
        self.last_cmd: Optional[Twist] = None
        self.last_cmd_monotonic: Optional[float] = None
        self.last_speed_percent: Optional[int] = None
        self.last_sent_signature: Optional[Tuple[int, int]] = None
        self.last_motion_name = 'BRAKE'
        self.waiting_for_fresh_cmd = False

        self.status_pub = self.create_publisher(String, '/motor_driver/status', 10)
        self.create_subscription(
            Twist, str(self.get_parameter('cmd_vel_topic').value), self.cmd_vel_callback, 10)
        self.enable_service = self.create_service(SetBool, '/motor_driver/enable', self.enable_callback)
        self.timer = self.create_timer(1.0 / self.command_rate_hz, self.control_tick)

        if requested_enabled and not self.dry_run:
            self.get_logger().warn(
                'enabled:=true ignored for physical output; call /motor_driver/enable after '
                'verifying the port and lifting the drive wheels')
        self.publish_status('started')

    def cmd_vel_callback(self, message: Twist) -> None:
        self.last_cmd = message
        self.last_cmd_monotonic = time.monotonic()
        if self.armed:
            self.waiting_for_fresh_cmd = False

    def enable_callback(self, request: SetBool.Request, response: SetBool.Response) -> None:
        if request.data:
            if self.dry_run:
                response.success = False
                response.message = 'dry_run=true; refusing to arm physical motor output'
                return
            try:
                self.open_serial()
                self.armed = True
                self.clear_command()
                self.waiting_for_fresh_cmd = self.require_fresh_cmd_after_arm
                if not self.send_stop():
                    raise OSError('failed to send initial brake command')
                response.success = True
                response.message = (
                    'motor driver armed; brake command sent; waiting for a fresh command'
                    if self.waiting_for_fresh_cmd else 'motor driver armed; brake command sent')
                self.publish_status('armed')
            except OSError as exc:
                self.armed = False
                response.success = False
                response.message = f'failed to open motor serial port: {exc}'
                self.publish_status('serial_open_failed')
            return

        stop_sent = self.send_stop()
        self.armed = False
        self.clear_command()
        self.close_serial()
        response.success = stop_sent
        response.message = (
            'motor driver disarmed and stop command sent'
            if stop_sent else 'motor driver disarmed, but stop command could not be sent')
        self.publish_status('disarmed')

    def open_serial(self) -> None:
        if self.serial_fd is not None:
            return
        if not self.port:
            raise OSError('port parameter is empty')
        baud_constant = getattr(termios, f'B{self.baudrate}', None)
        if baud_constant is None:
            raise OSError(f'unsupported termios baudrate {self.baudrate}')
        fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY)
        try:
            attributes = termios.tcgetattr(fd)
            attributes[0] = termios.IGNPAR
            attributes[1] = 0
            attributes[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            attributes[3] = 0
            attributes[4] = baud_constant
            attributes[5] = baud_constant
            attributes[6][termios.VMIN] = 0
            attributes[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attributes)
            termios.tcflush(fd, termios.TCIOFLUSH)
            self.serial_fd = fd
        except Exception:
            os.close(fd)
            raise

    def close_serial(self) -> None:
        if self.serial_fd is not None:
            os.close(self.serial_fd)
            self.serial_fd = None

    def clear_command(self) -> None:
        self.last_cmd = None
        self.last_cmd_monotonic = None
        self.last_sent_signature = None
        self.last_speed_percent = None

    def speed_percent(self, linear_x: float, angular_z: float) -> int:
        ratio = max(abs(linear_x) / self.max_linear, abs(angular_z) / self.max_angular)
        ratio = max(0.0, min(1.0, ratio))
        return int(round(self.min_percent + ratio * (self.max_percent - self.min_percent)))

    def select_motion(self, linear_x: float, angular_z: float) -> Tuple[int, str]:
        linear_active = abs(linear_x) >= self.linear_deadband
        angular_active = abs(angular_z) >= self.angular_deadband
        if not linear_active and not angular_active:
            return self.stop_command(), 'STOP'
        if not linear_active:
            return (CMD_SPIN_LEFT, 'SPIN_LEFT') if angular_z > 0.0 else (CMD_SPIN_RIGHT, 'SPIN_RIGHT')
        if linear_x < 0.0:
            if angular_active:
                return self.stop_command(), 'REJECTED_REVERSE_TURN'
            return CMD_BACKWARD, 'BACKWARD'
        if not angular_active:
            return CMD_FORWARD, 'FORWARD'
        return (CMD_ARC_LEFT, 'ARC_LEFT') if angular_z > 0.0 else (CMD_ARC_RIGHT, 'ARC_RIGHT')

    def stop_command(self) -> int:
        return {
            'stop': CMD_STOP,
            'brake': CMD_BRAKE,
            'emergency_stop': CMD_EMERGENCY_STOP,
        }[self.stop_mode]

    def control_tick(self) -> None:
        if not self.armed:
            return
        if self.waiting_for_fresh_cmd:
            return
        stale = self.last_cmd_monotonic is None or (
            time.monotonic() - self.last_cmd_monotonic > self.timeout_sec)
        if stale:
            if self.last_sent_signature is not None:
                stop_sent = self.send_stop()
                self.last_sent_signature = None
                if stop_sent:
                    self.publish_status('cmd_vel_timeout')
            return
        assert self.last_cmd is not None
        command, name = self.select_motion(self.last_cmd.linear.x, self.last_cmd.angular.z)
        if command in (CMD_STOP, CMD_BRAKE, CMD_EMERGENCY_STOP):
            if self.last_sent_signature is not None:
                stop_sent = self.send_stop()
                self.last_sent_signature = None
                if not stop_sent:
                    return
            if self.last_motion_name != name:
                self.last_motion_name = name
                event = 'command_rejected' if name == 'REJECTED_REVERSE_TURN' else 'stop_command_sent'
                if name == 'REJECTED_REVERSE_TURN':
                    self.get_logger().warn(
                        'reverse turning is unsupported by the documented protocol; braking instead')
                self.publish_status(event)
            return

        percent = self.speed_percent(self.last_cmd.linear.x, self.last_cmd.angular.z)
        signature = (command, percent)
        if signature != self.last_sent_signature:
            if percent != self.last_speed_percent:
                if not self.send_frame(CMD_SET_SPEED, [percent]):
                    return
                self.last_speed_percent = percent
            if not self.send_frame(command):
                return
            self.last_sent_signature = signature
            self.last_motion_name = name
            self.publish_status('command_sent')

    def send_stop(self) -> bool:
        sent = self.send_frame(self.stop_command())
        self.last_motion_name = 'STOP'
        self.last_speed_percent = None
        return sent

    def send_frame(self, command: int, data: Optional[List[int]] = None) -> bool:
        frame = make_frame(command, data)
        if self.dry_run:
            self.get_logger().info(f'dry-run TX: {frame.hex(" ").upper()}')
            return True
        if self.serial_fd is None:
            self.get_logger().error('motor serial port is not open; disarming driver')
            self.armed = False
            self.publish_status('serial_not_open')
            return False
        try:
            remaining = memoryview(frame)
            while remaining:
                written = os.write(self.serial_fd, remaining)
                if written <= 0:
                    raise OSError('serial write returned zero bytes')
                remaining = remaining[written:]
            termios.tcdrain(self.serial_fd)
            return True
        except OSError as exc:
            self.get_logger().error(f'motor serial write failed: {exc}; disarming driver')
            self.armed = False
            self.waiting_for_fresh_cmd = False
            self.close_serial()
            self.publish_status('serial_write_failed')
            return False

    def publish_status(self, event: str) -> None:
        status = String()
        age = -1.0 if self.last_cmd_monotonic is None else time.monotonic() - self.last_cmd_monotonic
        status.data = (
            f'event={event} armed={str(self.armed).lower()} dry_run={str(self.dry_run).lower()} '
            f'motion={self.last_motion_name} cmd_age_sec={age:.3f} port={self.port or "unset"}')
        self.status_pub.publish(status)

    def destroy_node(self) -> bool:
        try:
            if self.armed:
                self.send_stop()
        finally:
            self.close_serial()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
