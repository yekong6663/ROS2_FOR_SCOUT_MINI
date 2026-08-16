#!/usr/bin/env python3
"""Run the outdoor_02 route with two complete grasp-and-place handoffs.

The route reaches points 1--8 once.  It then runs points 9--15 twice; each
round hands control to the arm at point 10 to acquire the target selected by
the photo card, and at point 15 to align and release it into its labelled box.
Navigation and arm services stay in one persistent ROS process throughout.
"""

import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import (
    ComputePathToPose,
    ComputePathThroughPoses,
    NavigateThroughPoses,
    NavigateToPose,
)
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# Approved outdoor_02 outbound leg. The small-road and pillar detour points
# (2--6) are deliberately excluded: after point 1 the route returns directly
# to the two recorded return points.
OUTBOUND_POINTS = [
    (0, 17.580, -2.213, -0.097, "目标点1前置点（2026-08-15 记录）"),
    (1, 47.405, -4.040, -0.093, "目标点1（2026-08-15 重录）"),
    (7, 36.237, -6.434, 3.132, "回原点1（2026-08-16 重录）"),
    (8, -0.544, -1.887, -3.096, "回原点2"),
]

WORK_CYCLE_POINTS = [
    (9, -18.638, 3.721, 3.108, "待抓取点"),
    (10, -20.470, 3.775, 3.092, "抓取点"),
    (11, -30.113, 3.611, 3.093, "抓取后行动一"),
    (12, -29.566, -2.249, -1.401, "抓取后行动二"),
    (13, -15.076, -3.375, -0.008, "抓取后行动三"),
    (14, -19.200, 3.750, 3.102, "放置预停点"),
    (15, -20.685, 3.813, 3.090, "放置点"),
]

SOURCE_ROOT = Path(
    "/home/nvidia/auto/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws/src/"
    "scout_navigation_bringup"
)
PRECISION_BEHAVIOR_TREE = str(
    SOURCE_ROOT / "behavior_trees/navigate_to_pose_outdoor_precision.xml"
)
CONTINUOUS_BEHAVIOR_TREE = str(
    SOURCE_ROOT / "behavior_trees/navigate_through_poses_outdoor2_continuous.xml"
)
PRECISION_POINT_NUMBERS = {9, 10, 14, 15}
RED_FLAG_GATE = "/home/nvidia/auto/Robot_arm/source/scripts/wait_for_red_flag_start.sh"
STARTUP_FORWARD_HELPER = SOURCE_ROOT / "scripts/safe_startup_forward.py"

class Outdoor2RecordedRoute(Node):
    """Visit each currently enabled outdoor_02 point in order."""

    def __init__(self):
        super().__init__("run_outdoor2_recorded_route")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("retry_delay", 2.0)
        self.declare_parameter("handoff_retry_delay", 5.0)
        self.declare_parameter("arm_handoff_timeout", 900.0)
        self.declare_parameter("precision_behavior_tree", PRECISION_BEHAVIOR_TREE)
        self.declare_parameter("continuous_behavior_tree", CONTINUOUS_BEHAVIOR_TREE)
        # Localization guard: pause navigation when localization inputs stall
        # or the map pose jumps, then resume automatically once stable.
        self.declare_parameter("localization_guard_enabled", True)
        self.declare_parameter("localization_settle_time", 1.5)
        # Zero means wait indefinitely while stopped, so a transient
        # relocalization correction cannot terminate a long mission.
        self.declare_parameter("localization_wait_timeout", 0.0)
        self.declare_parameter("max_map_odom_drift_m", 0.08)
        self.declare_parameter("max_map_odom_drift_deg", 3.0)
        # FAST-LIO2 on this Jetson takes ~2.7 s per frame and drops queued
        # lidar frames to keep up, so body_cloud arrives every 2-3 s (and can
        # stall much longer under load). 0.5 s caused the guard to cancel every
        # goal within 0.2 s of sending it. 3.0 s still catches a dead LIO node
        # while tolerating its normal slow cadence.
        self.declare_parameter("cloud_stale_timeout", 3.0)
        self.declare_parameter("odom_stale_timeout", 0.5)
        self.declare_parameter("monitor_period", 0.2)
        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._through_client = ActionClient(
            self, NavigateThroughPoses, "/navigate_through_poses"
        )
        # This client is deliberately separate from NavigateThroughPoses.  It
        # validates that the current pose is genuinely free before the route
        # action is allowed to publish any velocity command.
        self._through_plan_client = ActionClient(
            self, ComputePathThroughPoses, "/compute_path_through_poses"
        )
        self._straight_plan_client = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose"
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._goal_handle = None
        self._last_feedback_time = 0.0
        self._active_point = 0
        self._pipeline_probe = self.create_client(Trigger, "/grasp_pipeline/probe")
        self._pipeline_run = self.create_client(Trigger, "/grasp_pipeline/run")
        self._pipeline_set_parameters = self.create_client(
            SetParameters, "/grasp_pipeline/set_parameters"
        )
        self._placement_align = self.create_client(
            Trigger, "/grasp_pipeline/scan_and_align_placement_target"
        )
        self._placement_execute = self.create_client(
            Trigger, "/grasp_pipeline/execute_aligned_place"
        )
        self._pipeline_result_sequence = 0
        self._pipeline_result = None
        self._target_item_id = ""
        self._pipeline_result_subscription = self.create_subscription(
            String, "/grasp_pipeline/result_json", self._pipeline_result_callback, 10
        )
        # Localization-guard inputs. The guard only acts while a navigation
        # goal is active or while waiting to (re)send one, never during arm
        # handoffs.
        self._latest_odom = None
        self._last_odom_time = 0.0
        self._odom_subscription = self.create_subscription(
            Odometry, "/odom", self._odom_callback, 20
        )
        self._last_cloud_time = 0.0
        self._cloud_subscription = self.create_subscription(
            PointCloud2, "/fastlio2/body_cloud", self._cloud_callback, 10
        )
        # The guard may need to hold the vehicle while no Nav2 action is
        # running (between a cancel and the next goal). Nav2's own
        # publish_zero_velocity covers action completion; this explicit zero
        # covers the hold window.
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.get_logger().info("Outdoor_02 two-cycle navigation/arm bridge is ready")

    def _odom_callback(self, message):
        position = message.pose.pose.position
        quaternion = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )
        self._latest_odom = (position.x, position.y, yaw, time.monotonic())
        self._last_odom_time = self._latest_odom[3]

    def _cloud_callback(self, _message):
        self._last_cloud_time = time.monotonic()

    def _pipeline_result_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict):
            self._pipeline_result = payload
            self._pipeline_result_sequence += 1

    def _call_service(self, client, request, *, label, timeout):
        deadline = time.monotonic() + max(0.1, float(timeout))
        while rclpy.ok() and not client.service_is_ready():
            if time.monotonic() >= deadline:
                raise RuntimeError(f"{label} is unavailable")
            rclpy.spin_once(self, timeout_sec=0.1)
        future = client.call_async(request)
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError(f"{label} timed out after {timeout:.1f}s")
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"{label} returned no response")
        return future.result()

    def _set_pipeline_parameters(self, values):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name=name, value=value).to_parameter_msg()
            for name, value in values.items()
        ]
        response = self._call_service(
            self._pipeline_set_parameters,
            request,
            label="grasp pipeline parameter update",
            timeout=8.0,
        )
        failures = [
            result.reason or "rejected" for result in response.results if not result.successful
        ]
        if failures:
            raise RuntimeError("grasp pipeline parameter update failed: " + "; ".join(failures))

    def preflight_arm_pipeline(self):
        response = self._call_service(
            self._pipeline_probe, Trigger.Request(), label="grasp pipeline probe", timeout=20.0
        )
        if not response.success:
            raise RuntimeError(f"grasp pipeline probe failed: {response.message}")
        for client, label in (
            (self._pipeline_run, "/grasp_pipeline/run"),
            (self._pipeline_set_parameters, "/grasp_pipeline/set_parameters"),
            (self._placement_align, "/grasp_pipeline/scan_and_align_placement_target"),
            (self._placement_execute, "/grasp_pipeline/execute_aligned_place"),
        ):
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError(f"{label} is unavailable")
        self.get_logger().info("Arm pipeline preflight passed; persistent DDS clients are warm")

    def _wait_for_grasp_result(self, sequence_before):
        deadline = time.monotonic() + float(self.get_parameter("arm_handoff_timeout").value)
        terminal = {"ok", "completed", "failed", "no_candidate", "stopped", "cancelled", "rejected"}
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._pipeline_result_sequence <= sequence_before:
                continue
            payload = self._pipeline_result or {}
            if str(payload.get("run_id") or "").startswith("grasp-") and str(payload.get("status") or "") in terminal:
                return payload
        raise RuntimeError("grasp task timed out")

    @staticmethod
    def _right_side_observation_parameters():
        return {
            "observe_pose": [0.0, -35.5, 491.1, 180.0, 67.77, 89.97],
            "placement_observe_pose": [0.0, -35.5, 491.1, 180.0, 67.77, 89.97],
        }

    def grasp_handoff(self):
        parameters = {
            "auto_target_from_card": True,
            "target_item_id": "",
            "prompt": "",
            "execute": True,
            "speed": 25,
            "observation_speed": 25,
            "confirm": False,
            "place_after_grasp": False,
            "base_grasp_scan_enabled": True,
            "target_card_base_search_enabled": True,
            "move_to_placement_observation_after_grasp": True,
            "continuous_search_enabled": True,
            "continuous_search_stop_on_center": True,
        }
        parameters.update(self._right_side_observation_parameters())
        self._set_pipeline_parameters(parameters)
        sequence_before = self._pipeline_result_sequence
        response = self._call_service(
            self._pipeline_run, Trigger.Request(), label="grasp pipeline run", timeout=8.0
        )
        if not response.success:
            raise RuntimeError(f"grasp run was rejected: {response.message}")
        payload = self._wait_for_grasp_result(sequence_before)
        if str(payload.get("status") or "") not in {"ok", "completed"}:
            raise RuntimeError(f"grasp failed: {payload.get('summary') or payload.get('message') or payload.get('status')}")
        target_item_id = str(dict(payload.get("target_item") or {}).get("item_id") or "").strip()
        if target_item_id not in {"red_block", "yellow_block", "blue_block", "orange_bottle", "dark_bottle", "green_bottle"}:
            raise RuntimeError("grasp completed without a valid catalog target")
        self._target_item_id = target_item_id
        self.get_logger().info(f"Round target acquired: {target_item_id}")
        return True

    def place_handoff(self):
        if not self._target_item_id:
            raise RuntimeError("placement has no target from the completed grasp")
        parameters = {
            "target_item_id": self._target_item_id,
            "speed": 25,
            "observation_speed": 25,
            "base_target_alignment_enabled": True,
            "base_aligned_place_enabled": False,
            "base_target_center_tolerance_norm": 0.08,
            "continuous_search_enabled": True,
            "continuous_search_stop_on_center": True,
        }
        parameters.update(self._right_side_observation_parameters())
        self._set_pipeline_parameters(parameters)
        try:
            align = self._call_service(
                self._placement_align, Trigger.Request(), label="target-box scan and alignment",
                timeout=float(self.get_parameter("arm_handoff_timeout").value),
            )
        finally:
            self._set_pipeline_parameters({"base_target_alignment_enabled": False})
        if not align.success:
            raise RuntimeError(f"target-box alignment failed: {align.message}")
        self._set_pipeline_parameters({"base_aligned_place_enabled": True})
        try:
            placement = self._call_service(
                self._placement_execute, Trigger.Request(), label="calibrated placement",
                timeout=float(self.get_parameter("arm_handoff_timeout").value),
            )
        finally:
            self._set_pipeline_parameters({"base_aligned_place_enabled": False})
        if not placement.success:
            raise RuntimeError(f"calibrated placement failed: {placement.message}")
        self._target_item_id = ""
        return True

    def _stop(self):
        # Only the localization guard calls this, while no Nav2 action is
        # running. During handoffs the guard is idle, so this publisher can
        # never compete with Nav2 or the arm's scan controller.
        self._cmd_pub.publish(Twist())

    def _handoff_until_success(self, label, operation):
        while rclpy.ok():
            self.get_logger().info(label)
            try:
                if operation():
                    return True
            except Exception as error:
                self.get_logger().error(f"{label} failed: {error}; holding position and retrying")
            deadline = time.monotonic() + max(1.0, float(self.get_parameter("handoff_retry_delay").value))
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _wait_external_until_success(self, label, command):
        """Run a gate while retaining this node's DDS discovery and TF state."""
        while rclpy.ok():
            self.get_logger().info(label)
            try:
                process = subprocess.Popen(command)
            except OSError as error:
                self.get_logger().error(f"{label} failed to start: {error}")
                self._wait_before_retry()
                continue

            while rclpy.ok() and process.poll() is None:
                rclpy.spin_once(self, timeout_sec=0.1)

            if not rclpy.ok():
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=2.0)
                return False
            if process.returncode == 0:
                return True
            self.get_logger().error(
                f"{label} failed with status {process.returncode}; gate will retry"
            )
            self._wait_before_retry()
        return False

    def _dock_until_success(self, staging, goal, label):
        """Reuse the proven two-stage slow-parking node for a final approach.

        The helper first uses normal Nav2 navigation to reach ``staging``.  It
        then switches to its odometry-locked low-speed forward crawl for the
        short verified segment to ``goal``.  That crawl does not use the local
        costmap (which is intentionally the reason this helper exists), while
        its narrow forward PointCloud emergency-stop corridor remains active.
        A transient localization or point-cloud interruption retries the whole
        stage and never terminates the mission by itself.
        """
        point_number, sx, sy, syaw, staging_name = staging
        _goal_number, gx, gy, gyaw, goal_name = goal
        helper = SOURCE_ROOT / "scripts/dock_to_recorded_point3.py"
        attempt = 1
        while rclpy.ok():
            command = [
                sys.executable,
                str(helper),
                "--ros-args",
                "-p", f"staging_x:={sx}",
                "-p", f"staging_y:={sy}",
                "-p", f"staging_yaw:={syaw}",
                "-p", f"goal_x:={gx}",
                "-p", f"goal_y:={gy}",
                "-p", f"goal_yaw:={gyaw}",
                "-p", "position_tolerance:=0.10",
                "-p", "yaw_tolerance:=0.12",
                "-p", "crawl_speed:=0.15",
                "-p", "crawl_timeout:=45.0",
                "-p", "max_yaw_rate:=0.16",
                "-p", "alignment_tolerance:=0.04",
                "-p", "localization_wait_timeout:=0.0",
                "-p", "front_safety_enabled:=true",
                "-p", "front_stop_distance:=0.45",
                "-p", "front_half_width:=0.16",
                "-p", "front_min_points:=4",
                "-p", f"precision_behavior_tree:={self.get_parameter('precision_behavior_tree').value}",
            ]
            self.get_logger().info(
                f"{label}：复用慢速停车程序，第 {attempt} 次从点{point_number}（{staging_name}）"
                f"低速靠近（{goal_name}）"
            )
            try:
                process = subprocess.Popen(command)
            except OSError as error:
                self.get_logger().error(f"{label} 无法启动慢速停车程序: {error}")
                self._wait_before_retry()
                attempt += 1
                continue

            while rclpy.ok() and process.poll() is None:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not rclpy.ok():
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=2.0)
                return False
            if process.returncode == 0:
                self.get_logger().info(f"{label} 完成")
                return True
            self.get_logger().warning(
                f"{label} 本次未完成（状态 {process.returncode}）；等待后自动重试，不退出路线"
            )
            self._wait_before_retry()
            attempt += 1
        return False

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
        while rclpy.ok():
            if (
                self._client.wait_for_server(timeout_sec=1.0)
                and self._through_client.wait_for_server(timeout_sec=1.0)
                and self._through_plan_client.wait_for_server(timeout_sec=1.0)
                and self._straight_plan_client.wait_for_server(timeout_sec=1.0)
            ):
                return True
            self.get_logger().warning(
                "等待 Nav2 动作服务；任务保持运行，Nav2 恢复后自动继续"
            )
        return False

    def _validate_plan_until_safe(self, points, label):
        """Require a valid no-motion global plan before starting a route.

        In particular, never let the NavigateThroughPoses behavior tree begin
        FollowPath while the planner still considers the robot's start cell
        lethal.  That race previously allowed a transient turn toward the
        right wall before the planning failure was reported.
        """
        attempt = 1
        while rclpy.ok():
            goal = ComputePathThroughPoses.Goal()
            goal.goals = self._make_through_poses(points)
            goal.use_start = False
            goal.planner_id = "GridBased"
            self.get_logger().info(
                f"安全预检 {label}：仅计算路线（第 {attempt} 次），未向底盘发送运动命令"
            )
            future = self._through_plan_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None and future.result().accepted:
                result_future = future.result().get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                result = result_future.result()
                if (
                    result is not None
                    and result.status == GoalStatus.STATUS_SUCCEEDED
                    and len(result.result.path.poses) >= 2
                ):
                    self.get_logger().info(
                        f"安全预检 {label} 通过：已获得 {len(result.result.path.poses)} 个路径点"
                    )
                    return True
            self.get_logger().warning(
                f"安全预检 {label} 未通过（当前位置可能仍在致命代价区）；"
                "保持停车并等待重试"
            )
            attempt += 1
            self._wait_before_retry()
        return False

    def _map_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter("frame_id").value),
                "base_link",
                Time(),
                timeout=Duration(seconds=0.2),
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

    def _odom_pose(self):
        sample = self._latest_odom
        if sample is None:
            return None
        if time.monotonic() - sample[3] > float(
            self.get_parameter("odom_stale_timeout").value
        ):
            return None
        return sample[:3]

    def _localization_stale_reason(self):
        """Return a reason when a mid-leg localization input has stalled, or
        None while the vehicle may keep driving.

        This intentionally checks interruptions only (TF chain, lidar cloud,
        wheel odometry). ICP jump detection is unreliable while moving, so it
        is handled separately at the stopped gates via
        _localization_stable_while_stopped().
        """
        if self._map_pose() is None:
            return "map transform unavailable"
        now = time.monotonic()
        if now - self._last_cloud_time > float(
            self.get_parameter("cloud_stale_timeout").value
        ):
            return f"lidar cloud stale for {now - self._last_cloud_time:.1f}s"
        if now - self._last_odom_time > float(
            self.get_parameter("odom_stale_timeout").value
        ):
            return f"odometry stale for {now - self._last_odom_time:.1f}s"
        return None

    def _localization_stable_while_stopped(self):
        """Require the map localization to settle before a goal (re)send.

        Only meaningful while stopped: both the map pose and the wheel-odom
        pose are frozen, so any drift of the implied map->odom signature is a
        localization jump, not vehicle motion.
        """
        settle_s = max(
            0.2, float(self.get_parameter("localization_settle_time").value)
        )
        deadline = time.monotonic() + settle_s
        signatures = []
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            map_pose = self._map_pose()
            odom_pose = self._odom_pose()
            if map_pose is None or odom_pose is None:
                continue
            signatures.append(self._map_to_odom_signature(map_pose, odom_pose))
        if len(signatures) < 3:
            self.get_logger().warning(
                "localization guard: cannot judge stability "
                "(map or odometry feedback unavailable)"
            )
            return False
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
        if translation_drift > max_translation or yaw_drift_deg > max_yaw:
            self.get_logger().warning(
                "localization guard: map localization is still jumping "
                f"(map-odom drift={translation_drift:.3f}m/"
                f"{yaw_drift_deg:.2f}deg, limits={max_translation:.3f}m/"
                f"{max_yaw:.2f}deg)"
            )
            return False
        return True

    def _wait_until_localization_stable(self):
        """Hold the vehicle stopped until localization is fresh and stable.

        Returns True when navigation may (re)send a goal, False when the
        configured wait timeout expires or the node shuts down.
        """
        if not bool(self.get_parameter("localization_guard_enabled").value):
            return True
        wait_timeout = float(self.get_parameter("localization_wait_timeout").value)
        deadline = (
            time.monotonic() + wait_timeout if wait_timeout > 0.0 else None
        )
        hold_started = time.monotonic()
        next_log = 0.0
        while rclpy.ok():
            self._stop()
            reason = self._localization_stale_reason()
            if reason is None and self._localization_stable_while_stopped():
                self.get_logger().info(
                    "localization guard: localization stable again; "
                    "navigation may resume"
                )
                return True
            if deadline is not None and time.monotonic() >= deadline:
                self.get_logger().error(
                    "localization guard: did not stabilize within "
                    f"{wait_timeout:.0f}s; vehicle remains stopped"
                )
                return False
            now = time.monotonic()
            if now >= next_log:
                detail = f" ({reason})" if reason else " (position jumping)"
                self.get_logger().warning(
                    "localization guard: holding at the current pose until "
                    f"localization is stable{detail}; held for "
                    f"{now - hold_started:.1f}s"
                )
                next_log = now + 5.0
            rclpy.spin_once(self, timeout_sec=0.3)
        return False

    @staticmethod
    def _path_is_straight_ahead(path, start_x, start_y, start_yaw):
        if len(path) < 2:
            return False
        total = 0.0
        max_lateral = 0.0
        previous = path[0].pose.position
        for pose in path[1:]:
            current = pose.pose.position
            total += math.hypot(current.x - previous.x, current.y - previous.y)
            dx, dy = current.x - start_x, current.y - start_y
            max_lateral = max(
                max_lateral,
                abs(-math.sin(start_yaw) * dx + math.cos(start_yaw) * dy),
            )
            previous = current
        end = path[-1].pose.position
        forward = math.cos(start_yaw) * (end.x - start_x) + math.sin(start_yaw) * (end.y - start_y)
        return 1.90 <= total <= 2.25 and forward >= 1.85 and max_lateral <= 0.20

    def _mandatory_startup_forward_until_safe(self):
        """Run the mandatory 2 m crawl only on a proven-free straight chord."""
        attempt = 1
        while rclpy.ok():
            pose = self._map_pose()
            if pose is not None:
                x, y, yaw = pose
                goal = ComputePathToPose.Goal()
                goal.goal = self._make_pose(
                    x + 2.0 * math.cos(yaw),
                    y + 2.0 * math.sin(yaw),
                    yaw,
                )
                goal.use_start = False
                goal.planner_id = "GridBased"
                self.get_logger().info(
                    f"启动前进安全预检（第 {attempt} 次）：仅验证车头正前方 2 m"
                )
                future = self._straight_plan_client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is not None and future.result().accepted:
                    result_future = future.result().get_result_async()
                    rclpy.spin_until_future_complete(self, result_future)
                    result = result_future.result()
                    if (
                        result is not None
                        and result.status == GoalStatus.STATUS_SUCCEEDED
                        and self._path_is_straight_ahead(
                            result.result.path.poses, x, y, yaw
                        )
                    ):
                        return self._wait_external_until_success(
                            "Mandatory guarded startup forward",
                            [
                                sys.executable,
                                str(STARTUP_FORWARD_HELPER),
                                "--ros-args",
                                "-p", "distance:=2.0",
                                "-p", "speed:=0.5",
                            ],
                        )
            self.get_logger().warning(
                "启动前进安全预检未通过：前方 2 m 不是已验证直通区域；保持停车等待"
            )
            attempt += 1
            self._wait_before_retry()
        return False

    def _wait_before_retry(self):
        delay = max(0.1, float(self.get_parameter("retry_delay").value))
        deadline = time.monotonic() + delay
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_goal_with_guard(self, result_future, label):
        """Block until an action result, canceling and holding the vehicle
        when the localization guard trips.

        Returns:
          - GoalStatus on normal completion (caller compares to SUCCEEDED)
          - None when the action ended without a result
          - False when the guard canceled the goal (outer retry holds until
            localization is stable again)
        """
        guard_enabled = bool(
            self.get_parameter("localization_guard_enabled").value
        )
        monitor_period = max(
            0.1, float(self.get_parameter("monitor_period").value)
        )
        last_check = time.monotonic()
        cancelled_for_localization = False
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if not guard_enabled:
                continue
            now = time.monotonic()
            if now - last_check < monitor_period:
                continue
            last_check = now
            reason = self._localization_stale_reason()
            if reason is None:
                continue
            self.get_logger().warning(
                f"localization guard: cancelling goal ({label}) because {reason}"
            )
            self.cancel()
            self._stop()
            cancelled_for_localization = True
            break
        if cancelled_for_localization:
            # Drain the action result (ABORTED after cancel) so no goal handle
            # lingers; the outer retry loop will hold until stable.
            drain_deadline = time.monotonic() + 5.0
            while (
                rclpy.ok()
                and not result_future.done()
                and time.monotonic() < drain_deadline
            ):
                rclpy.spin_once(self, timeout_sec=0.1)
            self._goal_handle = None
            return False
        result = result_future.result()
        self._goal_handle = None
        if result is None:
            return None
        return result.status

    def _navigate(self, point_number, x, y, yaw, description, waypoint=False):
        self._active_point = point_number
        self._last_feedback_time = 0.0
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = str(self.get_parameter("frame_id").value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
        # Waypoints use the ordinary outdoor tree (relaxed goal checker), so
        # the next goal is sent almost immediately after the previous one is
        # reached, keeping pass-through legs continuous. Precise parking points
        # keep their precision tree and recorded yaw.
        if not waypoint:
            precision = point_number in PRECISION_POINT_NUMBERS
            if precision:
                goal.behavior_tree = str(
                    self.get_parameter("precision_behavior_tree").value
                )

        self.get_logger().info(
            f"前往 outdoor_02 目标点 {point_number}（{description}，"
            f"{'过路点' if waypoint else ('高精度' if precision else '普通精度')}）: "
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
        outcome = self._wait_goal_with_guard(
            result_future, f"目标点 {point_number}"
        )
        if outcome is False:
            return False
        if outcome is None:
            self.get_logger().error(f"目标点 {point_number} ended without a result")
            return False
        if outcome != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"目标点 {point_number} failed with action status {outcome}; preparing retry"
            )
            return False

        self.get_logger().info(f"已到达 outdoor_02 目标点 {point_number}")
        return True

    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = str(self.get_parameter("frame_id").value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _make_through_poses(self, points):
        """Create ordinary waypoint poses without enforcing recorded yaws.

        SmacPlannerLattice honours every pose orientation while planning.  For
        pass-through points, recorded parking headings caused large S-shaped
        detours just to arrive at an arbitrary old yaw.  Give each point the
        tangent toward its successor instead; precise parking points continue
        to use _make_pose() with their recorded yaw through NavigateToPose.
        """
        poses = []
        for index, (_number, x, y, _recorded_yaw, _description) in enumerate(points):
            if index + 1 < len(points):
                _next_number, next_x, next_y, _next_yaw, _next_description = points[index + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            elif index > 0:
                _previous_number, previous_x, previous_y, _previous_yaw, _previous_description = points[index - 1]
                yaw = math.atan2(y - previous_y, x - previous_x)
            else:
                # A one-point ordinary route does not impose a final heading.
                yaw = 0.0
            poses.append(self._make_pose(x, y, yaw))
        return poses

    def _navigate_through(self, points, label):
        """Pass ordinary waypoints in one Nav2 action without goal-stop gaps."""
        self._active_point = f"{points[0][0]}-{points[-1][0]}"
        self._last_feedback_time = 0.0
        goal = NavigateThroughPoses.Goal()
        goal.poses = self._make_through_poses(points)
        goal.behavior_tree = str(
            self.get_parameter("continuous_behavior_tree").value
        )
        self.get_logger().info(
            f"连续通过 outdoor_02 普通目标点 {self._active_point}（{label}）；"
            "中间点不会停车"
        )
        send_future = self._through_client.send_goal_async(
            goal, feedback_callback=self._feedback
        )
        rclpy.spin_until_future_complete(self, send_future)
        if send_future.result() is None or not send_future.result().accepted:
            self.get_logger().error(f"连续路线 {self._active_point} 被 Nav2 拒绝")
            return False

        self._goal_handle = send_future.result()
        result_future = self._goal_handle.get_result_async()
        guard_enabled = bool(
            self.get_parameter("localization_guard_enabled").value
        )
        monitor_period = max(
            0.1, float(self.get_parameter("monitor_period").value)
        )
        last_check = time.monotonic()
        cancelled_for_localization = False
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if not guard_enabled:
                continue
            now = time.monotonic()
            if now - last_check < monitor_period:
                continue
            last_check = now
            reason = self._localization_stale_reason()
            if reason is None:
                continue
            self.get_logger().warning(
                "localization guard: cancelling goal on leg "
                f"{self._active_point} because {reason}"
            )
            self.cancel()
            self._stop()
            cancelled_for_localization = True
            break
        if cancelled_for_localization:
            # Drain the action result (ABORTED after cancel) so no goal handle
            # lingers; the outer retry loop will hold until stable.
            drain_deadline = time.monotonic() + 5.0
            while (
                rclpy.ok()
                and not result_future.done()
                and time.monotonic() < drain_deadline
            ):
                rclpy.spin_once(self, timeout_sec=0.1)
            self._goal_handle = None
            return False
        if result_future.result() is None:
            self.get_logger().error(f"连续路线 {self._active_point} 没有返回结果")
            return False
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"连续路线 {self._active_point} 失败，状态 {status}；准备重试"
            )
            return False
        self.get_logger().info(f"已连续通过普通目标点 {self._active_point}")
        self._goal_handle = None
        return True

    def _navigate_until_success(self, point):
        attempt = 1
        while rclpy.ok():
            try:
                if self._navigate(*point):
                    return True
            except (KeyboardInterrupt, ExternalShutdownException):
                raise
            except Exception as error:
                self.get_logger().error(f"目标点 {point[0]} 本次导航异常: {error}")
            self._goal_handle = None
            self.get_logger().warning(
                f"目标点 {point[0]} 第 {attempt} 次未成功；等待后重试同一目标，路线不会退出"
            )
            attempt += 1
            self._wait_before_retry()
        return False

    def _validate_point_until_safe(self, x, y, yaw, description):
        """Require a valid no-motion plan to a single goal before sending it."""
        attempt = 1
        while rclpy.ok():
            goal = ComputePathToPose.Goal()
            goal.goal = self._make_pose(x, y, yaw)
            goal.use_start = False
            goal.planner_id = "GridBased"
            self.get_logger().info(
                f"安全预检 {description}：仅计算路线（第 {attempt} 次），未向底盘发送运动命令"
            )
            future = self._straight_plan_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None and future.result().accepted:
                result_future = future.result().get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                result = result_future.result()
                if (
                    result is not None
                    and result.status == GoalStatus.STATUS_SUCCEEDED
                    and len(result.result.path.poses) >= 2
                ):
                    self.get_logger().info(
                        f"安全预检 {description} 通过：已获得 {len(result.result.path.poses)} 个路径点"
                    )
                    return True
            self.get_logger().warning(
                f"安全预检 {description} 未通过（当前位置可能仍在致命代价区）；"
                "保持停车并等待重试"
            )
            attempt += 1
            self._wait_before_retry()
        return False

    def _navigate_sequence_until_success(self, points, label):
        """Visit ordinary waypoints one NavigateToPose goal at a time.

        Unlike NavigateThroughPoses (whose RemovePassedGoals judged pass-through
        by distance and re-issued every waypoint on retry, making the robot
        drive back to already-passed points), this keeps explicit progress:
        a mid-leg failure resumes from the failed point, never re-driving
        completed points. The relaxed waypoint goal checker (0.6 m / 1.2 rad)
        accepts each point as soon as the chassis is near it, so the next goal
        is sent almost immediately and pass-through legs stay continuous.
        """
        attempt = 1
        start_index = 0
        while rclpy.ok():
            # The vehicle is stopped here (after a failure/cancel or at leg
            # start), so the jump check is valid. Never (re)send a goal while
            # localization is stale or still jumping; hold and auto-resume.
            if not self._wait_until_localization_stable():
                return False
            resumed = False
            for index in range(start_index, len(points)):
                point_number, x, y, yaw, description = points[index]
                if not resumed:
                    if not self._validate_point_until_safe(x, y, yaw, description):
                        break
                    resumed = True
                try:
                    if self._navigate(
                        point_number, x, y, yaw, description, waypoint=True
                    ):
                        start_index = index + 1
                        continue
                except (KeyboardInterrupt, ExternalShutdownException):
                    raise
                except Exception as error:
                    self.get_logger().error(
                        f"目标点 {point_number} 本次导航异常: {error}"
                    )
                self._goal_handle = None
                self.get_logger().warning(
                    f"逐点路线 {label} 第 {attempt} 次在目标点 {point_number} 失败；"
                    f"从该点继续重试，已通过的 {start_index} 个点不会重走"
                )
                attempt += 1
                self._wait_before_retry()
                break
            else:
                self.get_logger().info(f"逐点路线 {label} 全部目标点通过")
                return True
        return False

    def _navigate_through_until_success(self, points, label):
        attempt = 1
        while rclpy.ok():
            # The vehicle is stopped here (after a failure/cancel or at leg
            # start), so the jump check is valid. Never (re)send a goal while
            # localization is stale or still jumping; hold and auto-resume.
            if not self._wait_until_localization_stable():
                return False
            if not self._validate_plan_until_safe(points, label):
                return False
            try:
                if self._navigate_through(points, label):
                    return True
            except (KeyboardInterrupt, ExternalShutdownException):
                raise
            except Exception as error:
                self.get_logger().error(f"连续路线 {label} 本次导航异常: {error}")
            self._goal_handle = None
            self.get_logger().warning(
                f"连续路线 {label} 第 {attempt} 次未成功；等待后重试，路线不会退出"
            )
            attempt += 1
            self._wait_before_retry()
        return False

    def run(self):
        # Arm and start-gate preparation must not depend on Nav2 discovery.
        # This lets the referee-facing observation move happen immediately
        # after the bridge is launched, while Nav2 is still warming up.
        arm_handoff_enabled = os.environ.get("ARM_HANDOFF_ENABLED", "1") == "1"
        if arm_handoff_enabled:
            while rclpy.ok():
                try:
                    self.preflight_arm_pipeline()
                    break
                except Exception as error:
                    self.get_logger().error(f"Arm pipeline preflight failed: {error}; retrying")
                    self._wait_before_retry()
            if not rclpy.ok():
                return 130
        else:
            self.get_logger().warning(
                "Arm handoff disabled by ARM_HANDOFF_ENABLED=0; running navigation only"
            )

        if os.environ.get("RED_FLAG_START_ENABLED", "1") == "1":
            if not self._wait_external_until_success(
                "Waiting for the red-flag start signal", [RED_FLAG_GATE]
            ):
                return 130
        else:
            self.get_logger().warning(
                "Red-flag start gate bypassed by RED_FLAG_START_ENABLED=0"
            )

        # Do not let a delayed Nav2 launch suppress the red-flag observation.
        # Once the start signal is accepted, retain the mission and wait here
        # until navigation is actually ready to receive the first goal.
        if not self._wait_for_server():
            return 130

        outbound_label = "前置点、目标点1至回原点2（跳过小路和大柱子）"
        if not self._navigate_sequence_until_success(OUTBOUND_POINTS, outbound_label):
            return 130

        for round_index in range(1, 3):
            self._target_item_id = ""
            self.get_logger().info(f"Starting outdoor_02 grasp/place round {round_index}/2")
            pickup_staging, pickup, transit_1, transit_2, transit_3, place_staging, place = WORK_CYCLE_POINTS
            if not self._dock_until_success(
                pickup_staging, pickup, f"第 {round_index} 轮待抓取点到抓取点"
            ):
                return 130
            if arm_handoff_enabled:
                if not self._handoff_until_success(
                    f"Round {round_index}: pickup reached; handing control to the arm pipeline",
                    self.grasp_handoff,
                ):
                    return 130
                self.get_logger().info(
                    f"Round {round_index}: grasp completed; resuming outdoor navigation"
                )
            if not self._navigate_sequence_until_success(
                [transit_1, transit_2, transit_3],
                f"第 {round_index} 轮抓取后行动一至三",
            ):
                return 130
            if not self._dock_until_success(
                place_staging, place, f"第 {round_index} 轮放置预停点到放置点"
            ):
                return 130
            if arm_handoff_enabled:
                if not self._handoff_until_success(
                    f"Round {round_index}: placement reached; handing control to the arm pipeline",
                    self.place_handoff,
                ):
                    return 130
                self.get_logger().info(
                    f"Round {round_index}: placement completed; resuming outdoor navigation"
                )

        if not rclpy.ok():
            return 130
        self.get_logger().info("outdoor_02 completed two grasp-and-place rounds; press Ctrl+C to exit")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
        return 130

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
