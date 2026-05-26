#!/usr/bin/env python3

import sys

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from nav2_msgs.action import Spin
from rclpy.action import ActionClient
from rclpy.node import Node


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class Nav2SpinActionDemo(Node):
    def __init__(self):
        super().__init__('nav2_send_spin_action')
        self.declare_parameter('target_yaw', 0.8)
        self.declare_parameter('time_allowance', 10.0)
        self.target_yaw = float(self.get_parameter('target_yaw').value)
        self.time_allowance = float(self.get_parameter('time_allowance').value)
        self.action_client = ActionClient(self, Spin, 'spin')

    def run(self):
        self.get_logger().info('Waiting for Nav2 spin action server...')
        if not self.action_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('spin action server is not available.')
            return 1

        goal = Spin.Goal()
        goal.target_yaw = self.target_yaw
        goal.time_allowance = Duration(
            sec=int(self.time_allowance),
            nanosec=int((self.time_allowance % 1.0) * 1.0e9),
        )

        self.get_logger().info(
            f'Sending Nav2 Spin goal: target_yaw={self.target_yaw:.3f} rad')
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Spin goal was rejected by Nav2.')
            return 1

        self.get_logger().info('Spin goal accepted. Watch /cmd_vel for angular.z output.')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        status_name = STATUS_NAMES.get(result.status, str(result.status))
        self.get_logger().info(f'Nav2 spin finished with status: {status_name} ({result.status})')
        return 0 if result.status == GoalStatus.STATUS_SUCCEEDED else 1


def main():
    rclpy.init()
    node = Nav2SpinActionDemo()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
