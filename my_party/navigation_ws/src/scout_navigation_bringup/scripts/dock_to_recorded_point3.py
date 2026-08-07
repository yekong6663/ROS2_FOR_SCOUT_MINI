#!/usr/bin/env python3

"""Navigate to a staging pose, then creep straight into recorded point 3.

The final creep deliberately does not consult Nav2's local costmap.  It is for
the pre-verified parking approach where a side obstacle is represented too
conservatively in the local map.  It publishes to /cmd_vel_nav, the existing
input of the velocity smoother; the Scout base topic remains /cmd_vel.
"""

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class TwoStagePoint3Dock(Node):
    """Run normal navigation to the staging pose and a bounded final crawl."""

    def __init__(self):
        super().__init__("dock_to_recorded_point3")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("staging_x", 1.754)
        self.declare_parameter("staging_y", 0.355)
        self.declare_parameter("staging_yaw", 3.140)
        self.declare_parameter("goal_x", 0.126)
        self.declare_parameter("goal_y", 0.394)
        self.declare_parameter("goal_yaw", 3.113)
        self.declare_parameter("position_tolerance", 0.10)
        self.declare_parameter("yaw_tolerance", 0.12)
        self.declare_parameter("crawl_speed", 0.14)
        self.declare_parameter("crawl_timeout", 45.0)
        self.declare_parameter("max_yaw_rate", 0.16)
        self.declare_parameter("alignment_tolerance", 0.04)
        self.declare_parameter("front_safety_enabled", True)
        self.declare_parameter("front_stop_distance", 0.45)
        self.declare_parameter("front_half_width", 0.16)
        self.declare_parameter("front_min_points", 4)
        self.declare_parameter(
            "precision_behavior_tree",
            "/home/nvidia/auto/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws/src/"
            "scout_navigation_bringup/behavior_trees/navigate_to_pose_outdoor_precision.xml",
        )

        self._navigator = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._front_blocked = False
        self._front_sub = self.create_subscription(
            PointCloud2, "/fastlio2/body_cloud", self._front_cloud_callback, 10
        )

    def _front_cloud_callback(self, cloud):
        """Keep a minimal physical stop zone in front of the chassis.

        This is intentionally independent from costmaps.  Side returns are
        ignored so a known side wall does not reject the parking creep.
        """
        if not bool(self.get_parameter("front_safety_enabled").value):
            self._front_blocked = False
            return
        stop_distance = float(self.get_parameter("front_stop_distance").value)
        half_width = float(self.get_parameter("front_half_width").value)
        minimum = int(self.get_parameter("front_min_points").value)
        count = 0
        try:
            for x, y, z in point_cloud2.read_points(
                cloud, field_names=("x", "y", "z"), skip_nans=True
            ):
                # The lidar is 0.30 m above ground.  Reject the ground and
                # points behind/beside the narrow forward emergency corridor.
                if 0.20 < x < stop_distance and abs(y) < half_width and -0.15 < z < 1.70:
                    count += 1
                    if count >= minimum:
                        self._front_blocked = True
                        return
        except Exception as error:
            self.get_logger().warn(f"Front safety cloud parse failed: {error}")
        self._front_blocked = False

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _pose(self, timeout=0.15):
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter("frame_id").value),
                str(self.get_parameter("base_frame").value),
                Time(),
                timeout=Duration(seconds=timeout),
            )
        except TransformException:
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return t.x, t.y, yaw

    def _wait_for_pose(self, timeout=20.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            pose = self._pose()
            if pose is not None:
                return pose
        return None

    def _navigate_to_staging(self):
        if not self._navigator.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("/navigate_to_pose is unavailable")
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = str(self.get_parameter("frame_id").value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.get_parameter("staging_x").value)
        goal.pose.pose.position.y = float(self.get_parameter("staging_y").value)
        yaw = float(self.get_parameter("staging_yaw").value)
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
        goal.behavior_tree = str(
            self.get_parameter("precision_behavior_tree").value
        )
        self.get_logger().info(
            "Navigating precisely to staging pose before the direct final approach"
        )
        future = self._navigator.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=25.0)
        if not future.done() or future.result() is None or not future.result().accepted:
            self.get_logger().error("Nav2 rejected the staging pose")
            return False
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        if (
            not result_future.done()
            or result_future.result() is None
            or result_future.result().status != GoalStatus.STATUS_SUCCEEDED
        ):
            self.get_logger().error("Staging navigation did not succeed; final crawl will not start")
            return False
        self._stop()
        return True

    def _crawl_to_goal(self):
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)
        goal_yaw = float(self.get_parameter("goal_yaw").value)
        position_tolerance = float(self.get_parameter("position_tolerance").value)
        yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        speed = float(self.get_parameter("crawl_speed").value)
        timeout = float(self.get_parameter("crawl_timeout").value)
        max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        alignment_tolerance = float(
            self.get_parameter("alignment_tolerance").value
        )
        deadline = time.monotonic() + timeout
        self.get_logger().info(
            "Starting direct final crawl: local costmap is ignored; "
            "narrow forward emergency stop remains enabled"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            pose = self._pose()
            if pose is None:
                self._stop()
                self.get_logger().error("Lost map-to-base transform; direct crawl stopped")
                return False
            x, y, yaw = pose
            dx, dy = goal_x - x, goal_y - y
            distance = math.hypot(dx, dy)
            yaw_error = normalize(goal_yaw - yaw)
            if distance <= position_tolerance and abs(yaw_error) <= yaw_tolerance:
                self._stop()
                self.get_logger().info("Reached recorded point 3 with precision tolerance")
                return True

            # Nav2's normal staging goal permits a relatively loose yaw error.
            # Do not creep while turning: first make the chassis parallel to
            # the verified parking direction, then move forward only.
            if abs(yaw_error) > alignment_tolerance:
                command = Twist()
                command.angular.z = max(
                    -max_yaw_rate, min(max_yaw_rate, 1.2 * yaw_error)
                )
                self._cmd_pub.publish(command)
                continue

            # Evaluate progress in the fixed parking direction, never from a
            # transient steering angle.  A negative value would require a
            # reverse maneuver, which this terminal mode intentionally forbids.
            forward_error = math.cos(goal_yaw) * dx + math.sin(goal_yaw) * dy
            if forward_error <= 0.0:
                self._stop()
                self.get_logger().error("Direct goal is no longer in front; stopping without reversing")
                return False
            if self._front_blocked:
                self._stop()
                self.get_logger().warning("Object detected in narrow forward emergency corridor; crawl paused")
                return False

            command = Twist()
            command.linear.x = min(speed, max(0.03, 0.6 * forward_error))
            command.angular.z = max(
                -max_yaw_rate,
                min(max_yaw_rate, 0.8 * yaw_error),
            )
            self._cmd_pub.publish(command)

        self._stop()
        self.get_logger().error("Direct crawl timed out; vehicle stopped")
        return False

    def run(self):
        if self._wait_for_pose() is None:
            self.get_logger().error("No valid map-to-base transform")
            return 2
        if not self._navigate_to_staging():
            return 3
        return 0 if self._crawl_to_goal() else 4


def main():
    rclpy.init()
    node = TwoStagePoint3Dock()
    try:
        result = node.run()
    except KeyboardInterrupt:
        node._stop()
        result = 130
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(result)


if __name__ == "__main__":
    main()
