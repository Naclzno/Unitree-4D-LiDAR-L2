#!/usr/bin/env python3

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half_yaw = yaw * 0.5
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(half_yaw),
        'w': math.cos(half_yaw),
    }


class Nav2SpinGoalDemo(Node):
    def __init__(self):
        super().__init__('nav2_send_spin_goal')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('yaw_delta', 0.8)

        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.yaw_delta = float(self.get_parameter('yaw_delta').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def run(self):
        self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose action server is not available.')
            return 1

        self.get_logger().info(
            f'Waiting for TF {self.global_frame} -> {self.base_frame}...')
        transform = None
        deadline = self.get_clock().now() + Duration(seconds=10.0)
        while rclpy.ok() and self.get_clock().now() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.global_frame,
                    self.base_frame,
                    rclpy.time.Time())
                break
            except TransformException as exc:
                self.get_logger().warn(
                    f'TF not ready yet: {exc}', throttle_duration_sec=1.0)
                rclpy.spin_once(self, timeout_sec=0.1)

        if transform is None:
            self.get_logger().error('Could not get current robot pose from TF.')
            return 1

        current_yaw = yaw_from_quaternion(transform.transform.rotation)
        target_yaw = current_yaw + self.yaw_delta
        target_q = quaternion_from_yaw(target_yaw)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = transform.transform.translation.x
        goal.pose.pose.position.y = transform.transform.translation.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = target_q['x']
        goal.pose.pose.orientation.y = target_q['y']
        goal.pose.pose.orientation.z = target_q['z']
        goal.pose.pose.orientation.w = target_q['w']

        self.get_logger().info(
            f'Sending in-place yaw goal: current={current_yaw:.3f}, target={target_yaw:.3f}')
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by Nav2.')
            return 1

        self.get_logger().info('Goal accepted. Watch /cmd_vel for Nav2 output.')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f'Nav2 goal finished with status: {result.status}')
        return 0


def main():
    rclpy.init()
    node = Nav2SpinGoalDemo()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
