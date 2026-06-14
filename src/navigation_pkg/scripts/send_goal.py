#!/usr/bin/env python3
import argparse
import math
import sys
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node


STATUS_TEXT = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class GoalSender(Node):
    def __init__(self):
        super().__init__('navigation_goal_sender')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._last_feedback_time = self.get_clock().now()

    def send(
        self,
        x: float,
        y: float,
        yaw: float,
        frame_id: str,
        server_timeout: float,
        result_timeout: Optional[float],
    ) -> int:
        self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...')
        if not self._client.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error('navigate_to_pose action server is not available.')
            return 2

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'Sending goal: frame={frame_id}, x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad'
        )
        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by Nav2.')
            return 3

        self.get_logger().info('Goal accepted; navigating...')
        result_future = goal_handle.get_result_async()
        started = self.get_clock().now()

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            if result_timeout is None:
                continue
            elapsed = self.get_clock().now() - started
            if elapsed > Duration(seconds=result_timeout):
                self.get_logger().warn('Result timeout reached; canceling goal.')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                return 4

        result = result_future.result()
        status = result.status
        status_text = STATUS_TEXT.get(status, str(status))
        nav_result = result.result
        error_code = getattr(nav_result, 'error_code', 0)
        error_msg = getattr(nav_result, 'error_msg', '')

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded.')
            return 0

        self.get_logger().error(
            f'Navigation finished with status={status_text}, '
            f'error_code={error_code}, error_msg="{error_msg}"'
        )
        return 5

    def _feedback_callback(self, feedback_msg):
        now = self.get_clock().now()
        if now - self._last_feedback_time < Duration(seconds=1.0):
            return
        self._last_feedback_time = now
        feedback = feedback_msg.feedback
        distance_remaining = getattr(feedback, 'distance_remaining', float('nan'))
        eta = getattr(feedback, 'estimated_time_remaining', None)
        eta_sec = eta.sec + eta.nanosec * 1e-9 if eta is not None else float('nan')
        self.get_logger().info(
            f'distance_remaining={distance_remaining:.2f} m, eta={eta_sec:.1f} s'
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description='Send a Nav2 NavigateToPose goal in the map frame.'
    )
    parser.add_argument('x', type=float, help='Goal x in meters.')
    parser.add_argument('y', type=float, help='Goal y in meters.')
    parser.add_argument(
        'yaw',
        nargs='?',
        type=float,
        default=0.0,
        help='Goal yaw. Radians by default; use --degrees for degrees.',
    )
    parser.add_argument('--degrees', action='store_true', help='Interpret yaw as degrees.')
    parser.add_argument('--frame', default='map', help='Goal frame id. Default: map.')
    parser.add_argument(
        '--server-timeout',
        type=float,
        default=30.0,
        help='Seconds to wait for navigate_to_pose action server.',
    )
    parser.add_argument(
        '--result-timeout',
        type=float,
        default=None,
        help='Optional seconds to wait for final result before canceling.',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    yaw = math.radians(args.yaw) if args.degrees else args.yaw

    rclpy.init()
    node = GoalSender()
    try:
        exit_code = node.send(
            args.x,
            args.y,
            yaw,
            args.frame,
            args.server_timeout,
            args.result_timeout,
        )
    except KeyboardInterrupt:
        node.get_logger().warn('Interrupted.')
        exit_code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
