#!/usr/bin/env python3

"""Visit two recorded outdoor poses and return to the pose at startup."""

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class OutdoorRecordedRoute(Node):
    """Run a fixed route only after Nav2 and map-frame localization are ready."""

    def __init__(self):
        super().__init__("run_outdoor_recorded_route")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("server_timeout", 30.0)
        self.declare_parameter("transform_timeout", 30.0)
        self.declare_parameter("return_to_start", True)

        # Recorded on outdoor_01 in the map frame.
        self.declare_parameter("point_1_x", 19.61)
        self.declare_parameter("point_1_y", 43.44)
        self.declare_parameter("point_1_yaw", -0.155)
        self.declare_parameter("point_2_x", 20.90)
        self.declare_parameter("point_2_y", 16.38)
        self.declare_parameter("point_2_yaw", -2.408)

        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._goal_handle = None
        self._last_feedback_time = 0.0

    def _wait_for_server(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._client.wait_for_server(timeout_sec=1.0):
                return True
            self.get_logger().info("Waiting for /navigate_to_pose...")
        return False

    def _read_start_pose(self, frame_id, base_frame, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            # TransformListener uses this node's executor.  Process incoming
            # /tf messages before querying its buffer; otherwise a standalone
            # route node never receives map-to-base transforms while waiting.
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                transform = self._tf_buffer.lookup_transform(
                    frame_id, base_frame, Time(), timeout=Duration(seconds=0.1)
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                yaw = math.atan2(
                    2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                    1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
                )
                return (translation.x, translation.y, yaw)
            except TransformException:
                pass
        return None

    def _feedback_callback(self, feedback_message):
        now = time.monotonic()
        if now - self._last_feedback_time < 1.0:
            return
        self._last_feedback_time = now
        self.get_logger().info(
            f"{feedback_message.feedback.distance_remaining:.2f} m remaining"
        )

    def _navigate(self, name, x, y, yaw, frame_id, timeout):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)

        self.get_logger().info(
            f"Navigating to {name}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout)
        if not send_future.done() or send_future.result() is None:
            self.get_logger().error(f"Timed out while sending {name}")
            return False

        self._goal_handle = send_future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error(f"Nav2 rejected {name}")
            return False

        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self._goal_handle = None
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error(f"Navigation ended without a result at {name}")
            return False
        if result_future.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"{name} failed with action status {result_future.result().status}; "
                "route stopped without continuing to the next point"
            )
            return False
        self.get_logger().info(f"Reached {name}")
        return True

    def run(self):
        frame_id = str(self.get_parameter("frame_id").value)
        base_frame = str(self.get_parameter("base_frame").value)
        server_timeout = float(self.get_parameter("server_timeout").value)
        transform_timeout = float(self.get_parameter("transform_timeout").value)

        if not self._wait_for_server(server_timeout):
            self.get_logger().error("Nav2 is unavailable; start navigation first")
            return 2

        start_pose = self._read_start_pose(
            frame_id, base_frame, transform_timeout
        )
        if start_pose is None:
            self.get_logger().error(
                "No valid map-to-base transform; route will not start"
            )
            return 3

        self.get_logger().info(
            f"Captured return pose: x={start_pose[0]:.3f}, "
            f"y={start_pose[1]:.3f}, yaw={start_pose[2]:.3f}"
        )
        points = [
            (
                "recorded point 1",
                float(self.get_parameter("point_1_x").value),
                float(self.get_parameter("point_1_y").value),
                float(self.get_parameter("point_1_yaw").value),
            ),
            (
                "recorded point 2",
                float(self.get_parameter("point_2_x").value),
                float(self.get_parameter("point_2_y").value),
                float(self.get_parameter("point_2_yaw").value),
            ),
        ]
        if bool(self.get_parameter("return_to_start").value):
            points.append(("startup position", *start_pose))

        for name, x, y, yaw in points:
            if not self._navigate(name, x, y, yaw, frame_id, server_timeout):
                return 4
        self.get_logger().info("Recorded route completed")
        return 0

    def cancel(self):
        if self._goal_handle is not None:
            self.get_logger().warning("Canceling active navigation goal")
            cancel_future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)


def main():
    rclpy.init()
    node = OutdoorRecordedRoute()
    try:
        return_code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.cancel()
        return_code = 130
    except Exception as error:
        node.get_logger().error(f"Recorded route failed: {error}")
        return_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return return_code


if __name__ == "__main__":
    sys.exit(main())
