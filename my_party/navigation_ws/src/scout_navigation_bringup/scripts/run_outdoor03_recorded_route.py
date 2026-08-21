#!/usr/bin/env python3
"""Run the outdoor_03 route with position-only, low-precision waypoints."""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
import yaml

import run_outdoor2_recorded_route as outdoor2

MAP_DIR = Path("/home/nvidia/auto/ROS2_FOR_SCOUT_MINI/maps/outdoor_03")
POSES_FILE = MAP_DIR / "recorded_poses.yaml"
ROUTE_IDS = (1, 2, 3, 5, 6, 7, 9, 10, 12, 14, 15, 16, 19, 20)
# The point immediately before every pre-stop is also used as a low-speed
# docking lead-in. This keeps the whole final approach precise without adding
# separately recorded guide poses.
STAGING_IDS = set()
FINAL_LEG_MAX_SPEED_MPS = 0.60
TRANSIT_SPEED_MPS = 1.00
FINAL_APPROACH_SPEED_MPS = 0.30
FINAL_OBSERVATION_HOLD_SEC = 3.0
# No startup delay during normal operation.  Set
# OUTDOOR03_STARTUP_DELAY_SEC explicitly when a full-map test needs a pause.
STARTUP_DELAY_SEC = 0.0
OUTDOOR_OLD_PRECISION_BT = str(
    Path("/home/nvidia/auto/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws/src/") /
    "scout_navigation_bringup/behavior_trees/navigate_to_pose_outdoor_precision.xml"
)


def _point(item: dict) -> tuple:
    number = int(item["id"])
    name = str(item["name"])
    try:
        return (number, float(item["x"]), float(item["y"]), float(item["yaw"]), name)
    except (KeyError, TypeError, ValueError) as error:
        raise SystemExit(f"outdoor_03 路点 {number}（{name}）坐标无效：{error}") from error


def load_route() -> dict[int, tuple]:
    if not POSES_FILE.is_file():
        raise SystemExit(f"缺少路点文件: {POSES_FILE}")
    payload = yaml.safe_load(POSES_FILE.read_text(encoding="utf-8")) or {}
    route = [_point(item) for item in payload.get("route") or []]
    points = {point[0]: point for point in route}
    if len(route) != len(points) or tuple(sorted(points)) != ROUTE_IDS:
        raise SystemExit(
            f"{POSES_FILE} 路点编号不正确；应为 {ROUTE_IDS}，当前为 {tuple(sorted(points))}"
        )
    return points


class Outdoor3RecordedRoute(outdoor2.Outdoor2RecordedRoute):
    """Full outdoor_03 route, retaining the common Nav2 safety/retry guards."""

    def __init__(self, points: dict[int, tuple]):
        super().__init__()
        # Use the proven outdoor_03_old tolerance for both the recorded
        # pre-stops and their final docking points. Wait longer before locking
        # odometry so a one-frame FAST-LIO correction cannot start a crawl
        # from a shifted map pose.
        results = self.set_parameters(
            [
                Parameter("dock_position_tolerance", value=0.10),
                Parameter("dock_yaw_tolerance", value=0.12),
                Parameter("dock_crawl_speed", value=0.12),
                Parameter("dock_staging_start_position_tolerance", value=0.15),
                # Allow normal outdoor LIO heading jitter at a pre-stop;
                # final grab/place docking keeps its tighter yaw tolerance.
                Parameter("dock_staging_start_yaw_tolerance", value=0.20),
                # outdoor_03_old used the ordinary outdoor precision tree for
                # pre-stops. Restore it rather than the newer simultaneous
                # staging checker, which chases normal outdoor LIO jitter.
                Parameter("staging_behavior_tree", value=OUTDOOR_OLD_PRECISION_BT),
                Parameter("localization_settle_time", value=4.0),
                # Outdoor FAST-LIO commonly settles with 8--12 cm of
                # map-to-odom variation. Permit that normal noise but still
                # reject the 30+ cm jumps seen during failed relocalization.
                # Ignore brief moderate map/odom corrections, but still stop
                # on a large persistent jump (the hard guard remains active).
                Parameter("max_map_odom_drift_m", value=0.30),
            ]
        )
        if not all(result.successful for result in results):
            raise RuntimeError("failed to apply outdoor_03 precision docking profile")
        self._points = points
        self._route_label = "run_outdoor03_recorded_route"

    def _ordinary_until_success(self, ids: tuple[int, ...], label: str) -> bool:
        # All outdoor_03 points are now ordinary position-only waypoints:
        # do not enforce recorded yaw or a high-precision parking checker.
        return self._navigate_sequence_until_success(
            [self._points[number] for number in ids], label
        )

    def _cone_until_success(self, ids: tuple[int, ...], label: str) -> bool:
        """Pass the cone slalom loosely: point-by-point position-only goals so
        the chassis drives straight through without stopping to re-align."""
        return self._navigate_sequence_until_success(
            [self._points[number] for number in ids], label
        )

    def _dock_until_complete(self, staging_id: int, goal_id: int, label: str) -> bool:
        return self._dock_until_success(
            self._points[staging_id], self._points[goal_id], label
        )

    def _arm_after_pickup(self, cycle: int, enabled: bool) -> bool:
        if not enabled:
            return False
        return self._handoff_limited(
            f"第 {cycle} 轮已到抓取点，交给机械臂抓取", self.grasp_handoff
        )

    def _arm_after_place(self, cycle: int, grasped: bool, enabled: bool) -> None:
        if enabled and grasped:
            placed = self._handoff_limited(
                f"第 {cycle} 轮已到放置点，交给机械臂放置",
                self.place_handoff,
                max_retries=0,
            )
            if not placed:
                self.discard_held_object_after_place_failure(cycle)
        elif enabled:
            self.get_logger().warning(f"第 {cycle} 轮未抓到物品，跳过机械臂放置交接")

    def run(self) -> int:
        arm_enabled = os.environ.get("ARM_HANDOFF_ENABLED", "1") == "1"
        skip_outbound = os.environ.get("SKIP_OUTBOUND", "0") == "1"
        skip_to_prepare = os.environ.get("SKIP_TO_PREPARE", "0") == "1"
        red_flag_enabled = os.environ.get("RED_FLAG_START_ENABLED", "1") == "1"
        self.get_logger().info(
            "outdoor_03 启动模式："
            f"SKIP_OUTBOUND={'1' if skip_outbound else '0'}，"
            f"SKIP_TO_PREPARE={'1' if skip_to_prepare else '0'}，"
            f"RED_FLAG_START_ENABLED={'1' if red_flag_enabled else '0'}，"
            f"ARM_HANDOFF_ENABLED={'1' if arm_enabled else '0'}"
        )

        if arm_enabled:
            while rclpy.ok():
                try:
                    self.preflight_arm_pipeline()
                    break
                except Exception as error:
                    self.get_logger().error(f"机械臂流程未就绪：{error}；继续等待")
                    self._wait_before_retry()
        else:
            self.get_logger().warning("ARM_HANDOFF_ENABLED=0：仅导航，不启动机械臂交接")
        if not rclpy.ok():
            return 130

        while rclpy.ok() and self._wait_for_pose(timeout=5.0) is None:
            self.get_logger().warning("等待 map→base_link 定位；路线会保留，不会退出")
            self._wait_before_retry()
        if not rclpy.ok():
            return 130

        if red_flag_enabled:
            if not self._wait_external_until_success(
                "等待红旗启动信号", [outdoor2.RED_FLAG_GATE]
            ):
                return 130
        else:
            self.get_logger().warning("RED_FLAG_START_ENABLED=0：跳过红旗，直接开始")

        if not self._wait_for_server():
            return 130

        # Apply the transit speed once before the ordinary route. Pre-stop
        # markers are ordinary pass-through points and must not trigger the
        # inherited staging approach slowdown.
        try:
            self.set_parameters([Parameter("cruise_speed", value=TRANSIT_SPEED_MPS)])
            self._set_nav_cruise_speed(TRANSIT_SPEED_MPS)
            self.get_logger().info("普通路线（含预停点）巡航速度限制为 1.00 m/s")
        except Exception as error:
            self.get_logger().warning(f"普通路线限速设置失败（{error}）；按当前控制器速度运行")

        # Delay only before publishing the first route goal. Navigation and
        # localization are already initialized; subsequent goals are sent
        # without this startup pause.
        startup_delay = max(
            0.0, float(os.environ.get("OUTDOOR03_STARTUP_DELAY_SEC", STARTUP_DELAY_SEC))
        )
        if startup_delay > 0.0:
            self.get_logger().info(
                f"初始化完成，首个目标点将在 {startup_delay:.0f} 秒后发布"
            )
            deadline = time.monotonic() + startup_delay
            last_reported = None
            while rclpy.ok() and time.monotonic() < deadline:
                remaining = max(0, int(deadline - time.monotonic() + 0.999))
                if remaining != last_reported:
                    self.get_logger().info(f"首个目标点倒计时：{remaining} 秒")
                    last_reported = remaining
                rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info("倒计时结束，发布首个目标点")

        if skip_to_prepare:
            self.get_logger().warning(
                "SKIP_TO_PREPARE=1：已用新准备避障点19作为初始位姿，跳过前置路线"
            )
        elif skip_outbound:
            # Point 2 is the relocalization pose selected by
            # skip_outbound_initial_pose.yaml. Do not send a second Nav2 goal
            # to the same pose: at a tight/partly occupied start pose Nav2 may
            # abort that zero-distance goal before the real mission begins.
            self.get_logger().warning(
                "SKIP_OUTBOUND=1：已用拐弯2作为初始位姿，跳过重复导航，直接前往抓取预停点"
            )
        elif not self._ordinary_until_success((1, 2), "拐弯点 1--2"):
            return 130

        if arm_enabled:
            self.get_logger().warning(
                "当前路线已删除抓取/放置点，忽略 ARM_HANDOFF_ENABLED，机械臂不参与路线"
            )

        # The four pre-stop markers remain as ordinary low-precision transit
        # points. The deleted grab/place poses are intentionally not visited.
        if not skip_to_prepare and not self._ordinary_until_success(
            (3, 5, 6, 7, 9, 10, 12, 14, 15, 16),
            "预停点与返程路线（普通精度）",
        ):
            return 130

        # The terminal leg is still ordinary navigation, but cap it at 0.6 m/s
        # whenever its next point is the final endpoint (point 19).
        try:
            self._set_nav_cruise_speed(FINAL_LEG_MAX_SPEED_MPS)
        except Exception as error:
            self.get_logger().warning(f"终点段限速失败（{error}）；按当前巡航速度前往")
        if not skip_to_prepare:
            if not self._ordinary_until_success(
                (19,), "前往新准备避障观察点19（最大 0.6 m/s）"
            ):
                return 130
        self._stop()
        self.get_logger().info(
            f"已到达准备避障点19，停车观察 {FINAL_OBSERVATION_HOLD_SEC:.1f}s 后重新规划"
        )
        deadline = time.monotonic() + FINAL_OBSERVATION_HOLD_SEC
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        self._refresh_costmaps_at_observation_point()
        try:
            self.set_parameters([Parameter("cruise_speed", value=FINAL_APPROACH_SPEED_MPS)])
            self._set_nav_cruise_speed(FINAL_APPROACH_SPEED_MPS)
            self.get_logger().info("19→20 最终避障段限速为 0.30 m/s")
        except Exception as error:
            self.get_logger().warning(f"最终避障段限速失败（{error}）；继续重新规划")
        if not self._ordinary_until_success((20,), "观察点19至终点20（重新规划，最大0.30 m/s）"):
            return 130
        self.get_logger().info("outdoor_03 全部启用路点完成；按 Ctrl+C 才退出")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
        return 130


def main() -> int:
    points = load_route()
    outdoor2.PRECISION_POINT_NUMBERS = set()
    outdoor2.STAGING_POINT_NUMBERS = STAGING_IDS
    rclpy.init()
    node = Outdoor3RecordedRoute(points)
    try:
        return node.run()
    except KeyboardInterrupt:
        node.get_logger().warning("收到 Ctrl+C，停止 outdoor_03 路线")
        node.cancel()
        return 130
    except Exception as error:
        node.get_logger().error(f"outdoor_03 路线异常：{error}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
