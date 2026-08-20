#!/usr/bin/env python3
"""Run the full recorded outdoor_03 route with precise pickup/place docks."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import rclpy
import yaml

import run_outdoor2_recorded_route as outdoor2

MAP_DIR = Path("/home/nvidia/auto/ROS2_FOR_SCOUT_MINI/maps/outdoor_03")
POSES_FILE = MAP_DIR / "recorded_poses.yaml"
ROUTE_IDS = tuple(range(1, 23))


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
            f"{POSES_FILE} 必须包含且只包含 1--22 各一个路点；当前为 {sorted(points)}"
        )
    return points


class Outdoor3RecordedRoute(outdoor2.Outdoor2RecordedRoute):
    """Full outdoor_03 route, retaining the common Nav2 safety/retry guards."""

    def __init__(self, points: dict[int, tuple]):
        super().__init__()
        self._points = points
        self._route_label = "run_outdoor03_recorded_route"

    def _ordinary_until_success(self, ids: tuple[int, ...], label: str) -> bool:
        return self._navigate_through_until_success([self._points[number] for number in ids], label)

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
            self._handoff_limited(
                f"第 {cycle} 轮已到放置点，交给机械臂放置", self.place_handoff
            )
        elif enabled:
            self.get_logger().warning(f"第 {cycle} 轮未抓到物品，跳过机械臂放置交接")

    def run(self) -> int:
        arm_enabled = os.environ.get("ARM_HANDOFF_ENABLED", "1") == "1"
        skip_outbound = os.environ.get("SKIP_OUTBOUND", "0") == "1"
        red_flag_enabled = os.environ.get("RED_FLAG_START_ENABLED", "1") == "1"
        self.get_logger().info(
            "outdoor_03 启动模式："
            f"SKIP_OUTBOUND={'1' if skip_outbound else '0'}，"
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

        if skip_outbound:
            # SKIP_OUTBOUND=1: start from 拐弯2 (point 4), then continue to the
            # grasp pre-stop (5).
            self.get_logger().warning("SKIP_OUTBOUND=1：从拐弯2（点4）开始")
            if not self._ordinary_until_success((4,), "拐弯2（SKIP_OUTBOUND 初始点）"):
                return 130
        elif not self._ordinary_until_success((3, 4), "拐弯点 3--4（已去除红绿灯 1--2）"):
            return 130

        if not self._dock_until_complete(5, 6, "抓取预停点 5 到抓取点 6"):
            return 130
        grasped = self._arm_after_pickup(1, arm_enabled)

        if not self._cone_until_success((7, 8), "越过锥桶 7--8（快速通过）"):
            return 130
        if not self._dock_until_complete(9, 10, "放置预停点 9 到放置点 10"):
            return 130
        self._arm_after_place(1, grasped, arm_enabled)

        if not self._ordinary_until_success((11, 12, 14), "返程 11、12、14（已删除返程3与返程4中的3）"):
            return 130
        if not self._dock_until_complete(15, 16, "抓取预停点 15 到抓取点 16"):
            return 130
        grasped = self._arm_after_pickup(2, arm_enabled)

        if not self._cone_until_success((17, 18), "越过锥桶 17--18（快速通过）"):
            return 130
        if not self._dock_until_complete(19, 20, "放置预停点 19 到放置点 20"):
            return 130
        self._arm_after_place(2, grasped, arm_enabled)

        # Final leg approaches the mission end: slow down like a pre-stop so
        # the chassis eases into the end pose instead of rushing it.
        try:
            self._set_nav_cruise_speed(self._nav_speed_for_staging_approach())
        except Exception as error:
            self.get_logger().warning(f"终点段降速失败（{error}）；按巡航速度前往")
        if not self._ordinary_until_success((21, 22), "避障前至终点 21--22（慢速）"):
            return 130
        self.get_logger().info("outdoor_03 全部 22 个路点完成；按 Ctrl+C 才退出")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
        return 130


def main() -> int:
    points = load_route()
    outdoor2.PRECISION_POINT_NUMBERS = set()
    outdoor2.STAGING_POINT_NUMBERS = {5, 9, 15, 19}
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
