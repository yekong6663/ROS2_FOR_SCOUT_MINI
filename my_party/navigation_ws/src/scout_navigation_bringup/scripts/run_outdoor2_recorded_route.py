#!/usr/bin/env python3
"""Run the currently approved outdoor_02 recorded-point route.

Only point 1 is enabled for the first field test.  Later recorded points stay
documented in maps/outdoor_02/README.md and can be appended after validation.
This node controls navigation only; it never starts the manipulator pipeline.
"""

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


ROUTE = [
    (1, 21.056, 1.207, 0.383, "前进到 manner coffer"),
]


class Outdoor2RecordedRoute(Node):
    """Visit each currently enabled outdoor_02 point in order."""

    def __init__(self):
        super().__init__("run_outdoor2_recorded_route")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("server_timeout", 30.0)
        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._goal_handle = None
        self._last_feedback_time = 0.0
        self._active_point = 0

    def _feedback(self, feedback_message):
        now = time.monotonic()
        if now - self._last_feedback_time < 1.0:
            return
        self._last_feedback_time = now
        distance = feedback_message.feedback.distance_remaining
        self.get_logger().info(
            f"目标点 {self._active_point}: remaining {distance:.2f} m"
        )

    def _wait_for_server(self):
        timeout = float(self.get_parameter("server_timeout").value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._client.wait_for_server(timeout_sec=1.0):
                return True
            self.get_logger().info("Waiting for /navigate_to_pose...")
        return False

    def _navigate(self, point_number, x, y, yaw, description):
        self._active_point = point_number
        self._last_feedback_time = 0.0
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = str(self.get_parameter("frame_id").value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)

        self.get_logger().info(
            f"前往 outdoor_02 目标点 {point_number}（{description}）: "
            f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback
        )
        rclpy.spin_until_future_complete(self, send_future)
        if send_future.result() is None or not send_future.result().accepted:
            self.get_logger().error(f"Nav2 rejected 目标点 {point_number}")
            return False

        self._goal_handle = send_future.result()
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        if result_future.result() is None:
            self.get_logger().error(f"目标点 {point_number} ended without a result")
            return False

        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"目标点 {point_number} failed with action status {status}; preparing retry"
            )
            return False

        self.get_logger().info(f"已到达 outdoor_02 目标点 {point_number}")
        self._goal_handle = None
        return True

    def run(self):
        if not self._wait_for_server():
            self.get_logger().error(
                "Nav2 is unavailable; start and localize outdoor_02 first"
            )
            return 2
        for point in ROUTE:
            attempt = 1
            while rclpy.ok():
                if self._navigate(*point):
                    break
                self._goal_handle = None
                self.get_logger().warning(
                    f"目标点 {point[0]} 第 {attempt} 次未成功；立即重试同一目标，路线不会停止"
                )
                attempt += 1
        self.get_logger().info("当前 outdoor_02 测试路线完成")
        return 0

    def cancel(self):
        if self._goal_handle is None:
            return
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)


def main():
    rclpy.init()
    node = Outdoor2RecordedRoute()
    try:
        result = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.cancel()
        result = 130
    except Exception as error:
        node.get_logger().error(f"outdoor_02 route failed: {error}")
        result = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
