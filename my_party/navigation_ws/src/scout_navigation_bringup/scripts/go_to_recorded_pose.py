#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class GoToRecordedPose(Node):
    """Send one NavigateToPose goal, wait for completion, and exit."""

    def __init__(self):
        super().__init__("go_to_recorded_pose")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 4.034532)
        self.declare_parameter("y", 6.797518)
        self.declare_parameter("yaw", 1.416757)
        self.declare_parameter("server_timeout", 30.0)

        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._goal_handle = None
        self._last_feedback_time = 0.0

    def _feedback_callback(self, feedback_message):
        now = time.monotonic()
        if now - self._last_feedback_time < 1.0:
            return
        self._last_feedback_time = now

        feedback = feedback_message.feedback
        self.get_logger().info(
            f"Navigation in progress; distance remaining: "
            f"{feedback.distance_remaining:.2f} m"
        )

    def _wait_for_server(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._client.wait_for_server(timeout_sec=1.0):
                return True
            self.get_logger().info("Waiting for /navigate_to_pose...")
        return False

    def run(self):
        frame_id = str(self.get_parameter("frame_id").value)
        x = float(self.get_parameter("x").value)
        y = float(self.get_parameter("y").value)
        yaw = float(self.get_parameter("yaw").value)
        timeout = float(self.get_parameter("server_timeout").value)

        if not self._wait_for_server(timeout):
            self.get_logger().error(
                "Timed out waiting for Nav2. Start and localize the navigation "
                "system before running this command."
            )
            return 2

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)

        self.get_logger().info(
            f"Sending navigation goal in {frame_id}: "
            f"x={x:.6f}, y={y:.6f}, yaw={yaw:.6f} rad"
        )
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout)
        if not send_future.done() or send_future.result() is None:
            self.get_logger().error("Timed out while sending the navigation goal")
            return 3

        self._goal_handle = send_future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error("Nav2 rejected the navigation goal")
            return 4

        self.get_logger().info("Goal accepted")
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error("Navigation ended without a result")
            return 5

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Recorded pose reached successfully")
            return 0
        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning("Navigation goal was canceled")
            return 6

        self.get_logger().error(f"Navigation failed with action status {status}")
        return 7

    def cancel(self):
        if self._goal_handle is None:
            return
        self.get_logger().warning("Canceling the active navigation goal")
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)


def main():
    rclpy.init()
    node = GoToRecordedPose()
    try:
        return_code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.cancel()
        return_code = 130
    except Exception as error:
        node.get_logger().error(f"Failed to navigate to the recorded pose: {error}")
        return_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return return_code


if __name__ == "__main__":
    sys.exit(main())
