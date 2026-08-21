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
from nav_msgs.msg import Odometry
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
        # The map pose is sampled again after localization settles. Do not
        # begin an odometry-locked final crawl if that correction moved the
        # robot away from the recorded staging/pre-stop pose.
        self.declare_parameter("staging_start_position_tolerance", 0.10)
        self.declare_parameter("staging_start_yaw_tolerance", 0.12)
        self.declare_parameter("crawl_speed", 0.14)
        self.declare_parameter("crawl_timeout", 45.0)
        self.declare_parameter("max_yaw_rate", 0.16)
        self.declare_parameter("alignment_tolerance", 0.04)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_stale_timeout", 0.5)
        self.declare_parameter("localization_settle_time", 1.5)
        # Zero means wait indefinitely while stopped. Route runners use this
        # so a transient relocalization correction cannot terminate a long
        # pickup/place mission after all previous stages have succeeded.
        self.declare_parameter("localization_wait_timeout", 30.0)
        self.declare_parameter("max_map_odom_drift_m", 0.08)
        self.declare_parameter("max_map_odom_drift_deg", 3.0)
        self.declare_parameter("front_safety_enabled", True)
        self.declare_parameter("front_stop_distance", 0.45)
        self.declare_parameter("front_half_width", 0.16)
        self.declare_parameter("front_min_points", 4)
        # Staging poses are intermediate: the position must be accurate (the
        # odometry-locked crawl starts from it) and the heading should be
        # roughly respected, but 0.12 rad precision stalls forever on lidar
        # localization noise. Use the staging tree (xy 0.10 m, yaw 0.30 rad);
        # the crawl finishes the final yaw with wheel odometry.
        self.declare_parameter(
            "staging_behavior_tree",
            "/home/nvidia/auto/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws/src/"
            "scout_navigation_bringup/behavior_trees/navigate_to_pose_staging.xml",
        )
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
        self._latest_odom = None
        self._odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            20,
        )
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

    def _odom_callback(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._latest_odom = (p.x, p.y, yaw, time.monotonic())

    def _odom_pose(self):
        sample = self._latest_odom
        if sample is None:
            return None
        if time.monotonic() - sample[3] > float(
            self.get_parameter("odom_stale_timeout").value
        ):
            return None
        return sample[:3]

    @staticmethod
    def _map_to_odom_signature(map_pose, odom_pose):
        """Return the planar map->odom transform implied by two poses."""
        map_x, map_y, map_yaw = map_pose
        odom_x, odom_y, odom_yaw = odom_pose
        yaw = normalize(map_yaw - odom_yaw)
        c = math.cos(yaw)
        s = math.sin(yaw)
        return (
            map_x - (c * odom_x - s * odom_y),
            map_y - (s * odom_x + c * odom_y),
            yaw,
        )

    def _stable_crawl_start(self):
        """Require map localization to settle, then lock an odometric start."""
        settle_s = max(
            0.2, float(self.get_parameter("localization_settle_time").value)
        )
        deadline = time.monotonic() + settle_s
        signatures = []
        latest = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            map_pose = self._pose()
            odom_pose = self._odom_pose()
            if map_pose is None or odom_pose is None:
                continue
            latest = (map_pose, odom_pose)
            signatures.append(self._map_to_odom_signature(map_pose, odom_pose))
        if latest is None or len(signatures) < 3:
            self.get_logger().error(
                "Cannot start direct crawl: map or odometry feedback is unavailable"
            )
            return None
        ref_x, ref_y, ref_yaw = signatures[0]
        translation_drift = max(
            math.hypot(x - ref_x, y - ref_y) for x, y, _yaw in signatures
        )
        yaw_drift_deg = max(
            abs(math.degrees(normalize(yaw - ref_yaw)))
            for _x, _y, yaw in signatures
        )
        max_translation = float(self.get_parameter("max_map_odom_drift_m").value)
        max_yaw = float(self.get_parameter("max_map_odom_drift_deg").value)
        # A single FAST-LIO correction can produce an implausible spike even
        # while the chassis is stationary. Ignore isolated spikes; require
        # three consecutive bad samples before blocking the crawl.
        bad_samples = [
            math.hypot(x - ref_x, y - ref_y) > max_translation
            or abs(math.degrees(normalize(yaw - ref_yaw))) > max_yaw
            for x, y, yaw in signatures
        ]
        consecutive_bad = 0
        persistent_bad = False
        for bad in bad_samples:
            consecutive_bad = consecutive_bad + 1 if bad else 0
            if consecutive_bad >= 3:
                persistent_bad = True
                break
        if persistent_bad:
            self.get_logger().error(
                "Cannot start direct crawl: map localization is still jumping "
                f"(map-odom drift={translation_drift:.3f}m/{yaw_drift_deg:.2f}deg, "
                f"limits={max_translation:.3f}m/{max_yaw:.2f}deg)"
            )
            return None
        if any(bad_samples):
            self.get_logger().warning(
                "忽略一次性 map→odom 定位跳变，继续执行精细前进"
            )
        return latest

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
            self.get_parameter("staging_behavior_tree").value
        )
        self.get_logger().info(
            "Navigating to staging pose (position precise, heading moderate)"
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
        wait_timeout = float(
            self.get_parameter("localization_wait_timeout").value
        )
        wait_deadline = (
            time.monotonic() + wait_timeout if wait_timeout > 0.0 else None
        )
        start = None
        while rclpy.ok() and start is None:
            self._stop()
            start = self._stable_crawl_start()
            if start is not None:
                break
            if wait_deadline is not None and time.monotonic() >= wait_deadline:
                self.get_logger().error(
                    "Localization did not stabilize before the direct-crawl "
                    "wait timeout; vehicle remains stopped"
                )
                return False
            self.get_logger().warning(
                "Holding at the staging pose until map localization is stable; "
                "the direct approach will resume automatically"
            )
        if start is None:
            self._stop()
            return False
        map_start, odom_start = start
        map_x, map_y, map_yaw = map_start
        odom_x, odom_y, odom_yaw = odom_start
        staging_x = float(self.get_parameter("staging_x").value)
        staging_y = float(self.get_parameter("staging_y").value)
        staging_yaw = float(self.get_parameter("staging_yaw").value)
        staging_position_tolerance = float(
            self.get_parameter("staging_start_position_tolerance").value
        )
        staging_yaw_tolerance = float(
            self.get_parameter("staging_start_yaw_tolerance").value
        )
        staging_position_error = math.hypot(map_x - staging_x, map_y - staging_y)
        staging_yaw_error = abs(normalize(map_yaw - staging_yaw))
        if (
            staging_position_error > staging_position_tolerance or
            staging_yaw_error > staging_yaw_tolerance
        ):
            self._stop()
            self.get_logger().warning(
                "Final crawl blocked: localization settled away from the "
                "recorded staging pose "
                f"(position={staging_position_error:.3f}m/"
                f"{staging_position_tolerance:.3f}m, yaw="
                f"{staging_yaw_error:.3f}rad/{staging_yaw_tolerance:.3f}rad); "
                "returning to the recorded staging pose before retry"
            )
            return False
        dx, dy = goal_x - map_x, goal_y - map_y
        target_travel = math.cos(goal_yaw) * dx + math.sin(goal_yaw) * dy
        initial_lateral_error = -math.sin(goal_yaw) * dx + math.cos(goal_yaw) * dy
        target_odom_yaw = normalize(odom_yaw + normalize(goal_yaw - map_yaw))
        if target_travel <= -position_tolerance:
            self._stop()
            self.get_logger().error(
                "Direct goal is already behind at crawl start; refusing to reverse "
                f"(forward={target_travel:.3f}m lateral={initial_lateral_error:.3f}m)"
            )
            return False

        deadline = time.monotonic() + timeout
        self.get_logger().info(
            "Starting odometry-locked direct final crawl: local costmap is ignored; "
            "map relocalization jumps are isolated; narrow forward emergency stop "
            f"remains enabled (travel={target_travel:.3f}m)"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            pose = self._odom_pose()
            if pose is None:
                self._stop()
                self.get_logger().error("Lost fresh odometry; direct crawl stopped")
                return False
            x, y, yaw = pose
            traveled = (
                math.cos(target_odom_yaw) * (x - odom_x)
                + math.sin(target_odom_yaw) * (y - odom_y)
            )
            forward_error = target_travel - traveled
            yaw_error = normalize(target_odom_yaw - yaw)
            if abs(forward_error) <= position_tolerance and abs(yaw_error) <= yaw_tolerance:
                self._stop()
                self.get_logger().info(
                    "Reached recorded point 3 using odometry-locked precision "
                    f"(traveled={traveled:.3f}m remaining={forward_error:.3f}m)"
                )
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

            # A small negative remainder is accepted by the position-tolerance
            # branch above. A larger odometric overshoot remains a hard stop;
            # this terminal mode never commands reverse motion.
            if forward_error < -position_tolerance:
                self._stop()
                self.get_logger().error(
                    "Odometry reports final-crawl overshoot; stopping without "
                    f"reversing (traveled={traveled:.3f}m target={target_travel:.3f}m)"
                )
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
