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
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import (
    ComputePathToPose,
    ComputePathThroughPoses,
    NavigateThroughPoses,
    NavigateToPose,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


# Approved outdoor_02 outbound leg. The small-road and pillar detour points
# (2--6) are deliberately excluded: after point 1 the route returns directly
# to the two recorded return points.
OUTBOUND_POINTS = [
    (0, 17.580, -2.213, -0.097, "目标点1前置点（2026-08-15 记录）"),
    (1, 47.405, -4.040, -0.093, "目标点1（2026-08-15 重录）"),
    (7, 30.649, -7.153, 3.135, "回原点1"),
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
        self.get_logger().info("Outdoor_02 two-cycle navigation/arm bridge is ready")

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
        # A zero Twist is published by the existing navigation stack whenever
        # an action is completed/cancelled.  This bridge has no extra cmd_vel
        # publisher, so it cannot compete with Nav2 during handoffs.
        return None

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
        precision = point_number in PRECISION_POINT_NUMBERS
        if precision:
            goal.behavior_tree = str(
                self.get_parameter("precision_behavior_tree").value
            )

        self.get_logger().info(
            f"前往 outdoor_02 目标点 {point_number}（{description}，"
            f"{'高精度' if precision else '普通精度'}）: "
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
        rclpy.spin_until_future_complete(self, result_future)
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

    def _navigate_through_until_success(self, points, label):
        attempt = 1
        while rclpy.ok():
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
        if not self._validate_plan_until_safe(OUTBOUND_POINTS, outbound_label):
            return 130
        if not self._navigate_through_until_success(OUTBOUND_POINTS, outbound_label):
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
            if not self._navigate_through_until_success(
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
