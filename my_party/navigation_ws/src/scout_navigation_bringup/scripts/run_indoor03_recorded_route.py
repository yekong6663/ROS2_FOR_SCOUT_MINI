#!/usr/bin/env python3
"""Run indoor_03 navigation and arm handoffs in one persistent ROS node."""

import math
import json
import os
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dock_to_recorded_point3 import TwoStagePoint3Dock


SOURCE_ROOT = (
    "/home/nvidia/auto/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws/src/"
    "scout_navigation_bringup"
)
NORMAL_BT = f"{SOURCE_ROOT}/behavior_trees/navigate_to_pose_lane_safe.xml"
PRECISION_BT = (
    f"{SOURCE_ROOT}/behavior_trees/navigate_to_pose_indoor03_precision.xml"
)
RED_FLAG_GATE = "/home/nvidia/auto/Robot_arm/source/scripts/wait_for_red_flag_start.sh"


class Indoor03Route(TwoStagePoint3Dock):
    """Keep DDS discovery, TF and the Nav2 action client alive for all legs."""

    def __init__(self):
        super().__init__()
        self.declare_parameter("route_retry_delay", 2.0)
        self.declare_parameter("handoff_retry_delay", 5.0)
        self.declare_parameter("arm_handoff_timeout", 900.0)
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
            String,
            "/grasp_pipeline/result_json",
            self._pipeline_result_callback,
            10,
        )
        self.get_logger().info("Indoor_03 continuous route node is ready")

    def _pipeline_result_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(payload, dict):
            return
        self._pipeline_result = payload
        self._pipeline_result_sequence += 1

    def _call_service(self, client, request, *, label, timeout):
        """Call through an already-created DDS client and keep this node warm."""
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
            result.reason or "rejected"
            for result in response.results
            if not result.successful
        ]
        if failures:
            raise RuntimeError(
                "grasp pipeline parameter update failed: " + "; ".join(failures)
            )

    def preflight_arm_pipeline(self):
        """Perform the expensive full health probe once, before flag start."""
        response = self._call_service(
            self._pipeline_probe,
            Trigger.Request(),
            label="grasp pipeline probe",
            timeout=20.0,
        )
        if not response.success:
            raise RuntimeError(f"grasp pipeline probe failed: {response.message}")
        for client, label in (
            (self._pipeline_run, "/grasp_pipeline/run"),
            (self._pipeline_set_parameters, "/grasp_pipeline/set_parameters"),
            (
                self._placement_align,
                "/grasp_pipeline/scan_and_align_placement_target",
            ),
            (self._placement_execute, "/grasp_pipeline/execute_aligned_place"),
        ):
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError(f"{label} is unavailable")
        self.get_logger().info(
            "Arm pipeline preflight passed; persistent DDS clients are warm"
        )

    def _wait_for_grasp_result(self, sequence_before):
        timeout = float(self.get_parameter("arm_handoff_timeout").value)
        deadline = time.monotonic() + timeout
        terminal = {"ok", "completed", "failed", "no_candidate", "stopped", "cancelled", "rejected"}
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._pipeline_result_sequence <= sequence_before:
                continue
            payload = self._pipeline_result or {}
            run_id = str(payload.get("run_id") or "")
            status = str(payload.get("status") or "")
            if run_id.startswith("grasp-") and status in terminal:
                return payload
        raise RuntimeError(f"grasp task timed out after {timeout:.1f}s")

    def grasp_handoff(self):
        """Start pickup with one parameter transaction and no ros2 CLI startup."""
        self._set_pipeline_parameters(
            {
                "auto_target_from_card": True,
                "target_item_id": "",
                "prompt": "",
                "execute": True,
                "confirm": False,
                "place_after_grasp": False,
                "base_grasp_scan_enabled": True,
                "move_to_placement_observation_after_grasp": True,
            }
        )
        sequence_before = self._pipeline_result_sequence
        response = self._call_service(
            self._pipeline_run,
            Trigger.Request(),
            label="grasp pipeline run",
            timeout=8.0,
        )
        if not response.success:
            raise RuntimeError(f"grasp run was rejected: {response.message}")
        self.get_logger().info(
            "Pickup accepted immediately through the persistent ROS service"
        )
        payload = self._wait_for_grasp_result(sequence_before)
        status = str(payload.get("status") or "")
        if status not in {"ok", "completed"}:
            raise RuntimeError(
                f"grasp failed: {payload.get('summary') or payload.get('message') or status}"
            )
        target = dict(payload.get("target_item") or {})
        target_item_id = str(target.get("item_id") or "").strip()
        if target_item_id not in {
            "red_block",
            "yellow_block",
            "blue_block",
            "orange_bottle",
            "dark_bottle",
            "green_bottle",
        }:
            raise RuntimeError("grasp completed without a valid catalog target")
        self._target_item_id = target_item_id
        return True

    def place_handoff(self):
        """Align and place through persistent service clients."""
        if not self._target_item_id:
            raise RuntimeError("placement has no target from the completed grasp")
        self._set_pipeline_parameters(
            {
                "target_item_id": self._target_item_id,
                "base_target_alignment_enabled": True,
                "base_aligned_place_enabled": False,
            }
        )
        try:
            align = self._call_service(
                self._placement_align,
                Trigger.Request(),
                label="target-box scan and alignment",
                timeout=float(self.get_parameter("arm_handoff_timeout").value),
            )
        finally:
            self._set_pipeline_parameters({"base_target_alignment_enabled": False})
        if not align.success:
            raise RuntimeError(f"target-box alignment failed: {align.message}")

        self._set_pipeline_parameters({"base_aligned_place_enabled": True})
        try:
            placement = self._call_service(
                self._placement_execute,
                Trigger.Request(),
                label="calibrated placement",
                timeout=float(self.get_parameter("arm_handoff_timeout").value),
            )
        finally:
            self._set_pipeline_parameters({"base_aligned_place_enabled": False})
        if not placement.success:
            raise RuntimeError(f"calibrated placement failed: {placement.message}")
        return True

    def run_handoff_until_success(self, label, operation):
        """Retry a required handoff without spawning a new shell process."""
        while rclpy.ok():
            self.get_logger().info(label)
            try:
                if operation():
                    return True
            except Exception as error:
                self.get_logger().error(
                    f"{label} failed: {error}; holding position and retrying"
                )
            self._stop()
            delay = max(
                1.0, float(self.get_parameter("handoff_retry_delay").value)
            )
            deadline = time.monotonic() + delay
            while rclpy.ok() and time.monotonic() < deadline:
                self._stop()
                rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _wait_stopped(self):
        """Pause safely between attempts while continuing to receive ROS data."""
        delay = max(0.2, float(self.get_parameter("route_retry_delay").value))
        deadline = time.monotonic() + delay
        while rclpy.ok() and time.monotonic() < deadline:
            self._stop()
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_external_until_success(self, label, command):
        """Run a gate while keeping TF/action discovery warm in this node."""
        while rclpy.ok():
            self.get_logger().info(label)
            try:
                process = subprocess.Popen(command)
            except OSError as error:
                self.get_logger().error(f"{label} failed to start: {error}")
                self._wait_stopped()
                continue

            while rclpy.ok() and process.poll() is None:
                self._stop()
                rclpy.spin_once(self, timeout_sec=0.1)

            if not rclpy.ok():
                if process.poll() is None:
                    process.terminate()
                    process.wait()
                return False
            if process.returncode == 0:
                return True
            self.get_logger().error(
                f"{label} failed with status {process.returncode}; mission "
                "remains active and the gate will retry"
            )
            self._wait_stopped()
        return False

    def _navigate_once(self, label, x, y, yaw, behavior_tree):
        if not self._navigator.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("/navigate_to_pose is unavailable")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
        goal.behavior_tree = behavior_tree
        self.get_logger().info(
            f"Navigating to {label}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )

        send_future = self._navigator.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=20.0)
        if not send_future.done() or send_future.result() is None:
            self.get_logger().error(f"Timed out sending goal: {label}")
            return False
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error(f"Nav2 rejected goal: {label}")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result() if result_future.done() else None
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = result.status if result is not None else "unknown"
            self.get_logger().error(f"Goal {label} failed with status {status}")
            return False

        self.get_logger().info(
            f"Reached {label}; dispatching the next navigation stage immediately"
        )
        return True

    def navigate(self, label, x, y, yaw, behavior_tree):
        """Retry one Nav2 leg forever; only ROS shutdown/user Ctrl+C ends it."""
        while rclpy.ok():
            while rclpy.ok() and self._wait_for_pose(timeout=5.0) is None:
                self._stop()
                self.get_logger().warning(
                    f"{label}: waiting for localization; mission remains active"
                )
            if not rclpy.ok():
                return False
            if self._navigate_once(label, x, y, yaw, behavior_tree):
                return True
            self._stop()
            self.get_logger().warning(
                f"{label} failed; holding position and retrying the same target"
            )
            self._wait_stopped()
        return False

    def dock(self, label, staging, goal):
        while rclpy.ok():
            if not self.navigate(
                f"{label}预停点", *staging, behavior_tree=PRECISION_BT
            ):
                return False

            values = {
                "goal_x": goal[0],
                "goal_y": goal[1],
                "goal_yaw": goal[2],
                "position_tolerance": 0.08,
                "yaw_tolerance": 0.08,
                "crawl_speed": 0.12,
                "crawl_timeout": 30.0,
                "max_yaw_rate": 0.14,
                "alignment_tolerance": 0.05,
                # Keep the mission alive at a staging pose until localization
                # is stable. The chassis remains stopped throughout this wait.
                "localization_wait_timeout": 0.0,
                "front_safety_enabled": True,
                "front_stop_distance": 0.35,
                "front_half_width": 0.16,
                "front_min_points": 4,
            }
            results = self.set_parameters(
                [Parameter(name=name, value=value) for name, value in values.items()]
            )
            if not all(result.successful for result in results):
                self.get_logger().error(
                    f"Failed to configure slow approach: {label}; retrying"
                )
                self._wait_stopped()
                continue
            self.get_logger().info(
                f"Starting {label} slow approach without restarting a ROS process"
            )
            if self._crawl_to_goal():
                return True
            self._stop()
            self.get_logger().warning(
                f"{label} slow approach failed; returning safely to its staging "
                "target and retrying instead of ending the mission"
            )
            self._wait_stopped()
        return False

    def run_route(self):
        while rclpy.ok() and self._wait_for_pose(timeout=5.0) is None:
            self.get_logger().warning(
                "Waiting for map-to-base transform; the red-flag trigger is "
                "retained and the route will start automatically when "
                "localization recovers"
            )
        if not rclpy.ok():
            return 2

        if not self.navigate("门口（红绿灯）", 3.711, -0.579, -0.068, NORMAL_BT):
            return 130
        if not self.dock(
            "取件",
            (10.455, -5.603, -0.092),
            (11.292, -5.682, -0.094),
        ):
            return 130

        self._stop()
        if not self.run_handoff_until_success(
            "Pickup reached; handing control to the arm pipeline",
            self.grasp_handoff,
        ):
            return 130
        self.get_logger().info("Arm handoff completed; resuming navigation immediately")

        if not self.navigate("另一侧1", 13.065, -12.413, 3.092, NORMAL_BT):
            return 130
        if not self.dock(
            "放置",
            (10.877, -5.582, -0.122),
            (11.472, -5.684, -0.115),
        ):
            return 130

        self._stop()
        if not self.run_handoff_until_success(
            "Placement reached; handing control to the arm pipeline",
            self.place_handoff,
        ):
            return 130
        self.get_logger().info("Placement handoff completed; resuming navigation immediately")

        if not self.navigate("另一侧2", 13.065, -12.413, 3.092, NORMAL_BT):
            return 130
        if not self.navigate(
            "终点（起点反方向）", 3.711, -0.579, 3.074, NORMAL_BT
        ):
            return 130
        self.get_logger().info("Indoor_03 continuous route completed")
        self.get_logger().info(
            "Mission remains active at the final pose; press Ctrl+C to exit"
        )
        while rclpy.ok():
            self._stop()
            rclpy.spin_once(self, timeout_sec=0.2)
        return 130


def main():
    rclpy.init()
    node = Indoor03Route()
    try:
        result = 130
        while rclpy.ok():
            try:
                node.preflight_arm_pipeline()
                break
            except Exception as error:
                node.get_logger().error(
                    f"Arm pipeline preflight failed: {error}; retrying"
                )
                node._wait_stopped()
        # Create and spin the route node before waiting for the flag. This lets
        # DDS discovery and the TF buffer warm up while the camera gate runs,
        # so a successful wave can dispatch navigation immediately.
        if os.environ.get("RED_FLAG_START_ENABLED", "1") == "1":
            if not node.wait_external_until_success(
                "Waiting for the red-flag start signal", [RED_FLAG_GATE]
            ):
                return 130
        else:
            node.get_logger().warning(
                "Red-flag start gate bypassed by RED_FLAG_START_ENABLED=0"
            )
        while rclpy.ok():
            try:
                result = node.run_route()
                break
            except Exception as error:
                node._stop()
                node.get_logger().error(
                    f"Unexpected mission error: {error}; holding position and "
                    "restarting the mission state machine instead of exiting"
                )
                node._wait_stopped()
    except KeyboardInterrupt:
        node._stop()
        result = 130
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
