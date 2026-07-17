#!/usr/bin/env python3
"""Gate navigation velocity commands before they reach the physical motor driver."""

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


class SafetyControllerNode(Node):
    """Publish zero velocity unless command, odometry, and E-stop checks all pass."""

    def __init__(self) -> None:
        super().__init__('safety_controller')
        self.declare_parameter('input_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('output_cmd_vel_topic', '/motor/cmd_vel')
        self.declare_parameter('odom_topic', '/pointlio/odom')
        self.declare_parameter('emergency_stop_topic', '/emergency_stop')
        self.declare_parameter('enabled', False)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('cmd_vel_timeout_sec', 0.50)
        self.declare_parameter('odom_timeout_sec', 0.50)
        self.declare_parameter('require_odom', True)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_vel_timeout_sec').value)
        self.odom_timeout_sec = float(self.get_parameter('odom_timeout_sec').value)
        self.require_odom = bool(self.get_parameter('require_odom').value)
        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        if self.cmd_timeout_sec <= 0.0 or self.odom_timeout_sec <= 0.0:
            raise ValueError('timeout parameters must be positive')

        self.enabled = bool(self.get_parameter('enabled').value)
        self.estop_signal_active = False
        self.estop_latched = False
        self.last_cmd: Optional[Twist] = None
        self.last_cmd_time: Optional[float] = None
        self.last_odom_time: Optional[float] = None
        self.last_state = ''

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('output_cmd_vel_topic').value), 10)
        self.status_pub = self.create_publisher(String, '/safety_controller/status', 10)
        self.create_subscription(
            Twist,
            str(self.get_parameter('input_cmd_vel_topic').value),
            self.cmd_callback,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            20,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('emergency_stop_topic').value),
            self.estop_callback,
            10,
        )
        self.enable_service = self.create_service(
            SetBool, '/safety_controller/enable', self.enable_callback)
        self.reset_estop_service = self.create_service(
            Trigger, '/safety_controller/reset_emergency_stop', self.reset_estop_callback)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.control_tick)

        self.publish_zero()
        self.publish_status('started')

    def cmd_callback(self, message: Twist) -> None:
        self.last_cmd = message
        self.last_cmd_time = time.monotonic()

    def odom_callback(self, _: Odometry) -> None:
        self.last_odom_time = time.monotonic()

    def estop_callback(self, message: Bool) -> None:
        self.estop_signal_active = bool(message.data)
        if not self.estop_signal_active:
            return
        self.estop_latched = True
        self.enabled = False
        self.clear_command()
        self.publish_zero()
        self.publish_status('emergency_stop_latched')

    def enable_callback(self, request: SetBool.Request, response: SetBool.Response) -> None:
        if not request.data:
            self.enabled = False
            self.clear_command()
            self.publish_zero()
            response.success = True
            response.message = 'safety controller disabled and zero velocity published'
            self.publish_status('disabled')
            return

        if self.estop_latched:
            response.success = False
            response.message = 'emergency stop is latched; clear the input and call reset_emergency_stop first'
            return

        self.enabled = True
        # Do not pass a command received before this explicit enable request.
        self.clear_command()
        self.publish_zero()
        response.success = True
        response.message = 'safety controller enabled; waiting for a fresh command and odometry'
        self.publish_status('enabled_waiting_for_fresh_data')

    def reset_estop_callback(self, _: Trigger.Request, response: Trigger.Response) -> None:
        if self.estop_signal_active:
            response.success = False
            response.message = 'emergency stop input is still active'
            return
        self.estop_latched = False
        self.enabled = False
        self.clear_command()
        self.publish_zero()
        response.success = True
        response.message = 'emergency stop latch cleared; controller remains disabled'
        self.publish_status('emergency_stop_reset')

    def clear_command(self) -> None:
        self.last_cmd = None
        self.last_cmd_time = None

    def control_tick(self) -> None:
        reason = self.block_reason()
        if reason is not None:
            self.publish_zero()
            self.publish_status(reason)
            return

        assert self.last_cmd is not None
        self.cmd_pub.publish(self.last_cmd)
        self.publish_status('active')

    def block_reason(self) -> Optional[str]:
        if self.estop_latched:
            return 'emergency_stop_latched'
        if not self.enabled:
            return 'disabled'

        now = time.monotonic()
        if self.require_odom:
            if self.last_odom_time is None:
                return 'waiting_for_odom'
            if now - self.last_odom_time > self.odom_timeout_sec:
                return 'odom_timeout'
        if self.last_cmd is None or self.last_cmd_time is None:
            return 'waiting_for_fresh_cmd_vel'
        if now - self.last_cmd_time > self.cmd_timeout_sec:
            return 'cmd_vel_timeout'
        if not self.valid_twist(self.last_cmd):
            return 'invalid_cmd_vel'
        return None

    @staticmethod
    def valid_twist(message: Twist) -> bool:
        return all(math.isfinite(value) for value in (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        ))

    def publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())

    def publish_status(self, state: str) -> None:
        if state == self.last_state:
            return
        self.last_state = state
        status = String()
        cmd_age = -1.0 if self.last_cmd_time is None else time.monotonic() - self.last_cmd_time
        odom_age = -1.0 if self.last_odom_time is None else time.monotonic() - self.last_odom_time
        status.data = (
            f'state={state} enabled={str(self.enabled).lower()} '
            f'estop_latched={str(self.estop_latched).lower()} '
            f'cmd_age_sec={cmd_age:.3f} odom_age_sec={odom_age:.3f}')
        self.status_pub.publish(status)

    def destroy_node(self) -> bool:
        self.enabled = False
        self.publish_zero()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SafetyControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
