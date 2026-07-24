#!/usr/bin/env python3

import sys
import time
from pathlib import Path

import rclpy
import yaml
from interface.srv import IsValid, Relocalize
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class RelocalizationGate(Node):
    def __init__(self):
        super().__init__("relocalization_gate")
        self.declare_parameter("pcd_path", "")
        self.declare_parameter("pose_file", "")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("pitch", 0.0)
        self.declare_parameter("roll", 0.0)
        self.declare_parameter("timeout", 120.0)
        self.declare_parameter("check_period", 1.0)

        self.relocalize_client = self.create_client(
            Relocalize, "/localizer/relocalize"
        )
        self.valid_client = self.create_client(
            IsValid, "/localizer/relocalize_check"
        )

    def _load_pose(self):
        pose = {
            key: float(self.get_parameter(key).value)
            for key in ("x", "y", "z", "yaw", "pitch", "roll")
        }
        pose_file_value = self.get_parameter("pose_file").value
        if not pose_file_value:
            return pose

        pose_file = Path(pose_file_value)
        if not pose_file.is_file():
            self.get_logger().warning(
                f"No initial_pose.yaml found at {pose_file}; using launch arguments"
            )
            return pose

        with pose_file.open("r", encoding="utf-8") as stream:
            content = yaml.safe_load(stream) or {}
        values = content.get("initial_pose", content)
        for key in pose:
            if key in values:
                pose[key] = float(values[key])
        self.get_logger().info(f"Loaded initial pose from {pose_file}")
        return pose

    def _wait_for_service(self, client, service_name, deadline):
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                return True
            self.get_logger().info(f"Waiting for {service_name}...")
        return False

    def run(self):
        pcd_path = Path(self.get_parameter("pcd_path").value)
        timeout = float(self.get_parameter("timeout").value)
        check_period = float(self.get_parameter("check_period").value)
        deadline = time.monotonic() + timeout

        if not pcd_path.is_file():
            self.get_logger().error(f"3D map does not exist: {pcd_path}")
            return 2

        if not self._wait_for_service(
            self.relocalize_client, "/localizer/relocalize", deadline
        ):
            self.get_logger().error("Timed out waiting for relocalize service")
            return 3
        if not self._wait_for_service(
            self.valid_client, "/localizer/relocalize_check", deadline
        ):
            self.get_logger().error("Timed out waiting for relocalize_check service")
            return 3

        pose = self._load_pose()
        request = Relocalize.Request()
        request.pcd_path = str(pcd_path)
        request.x = pose["x"]
        request.y = pose["y"]
        request.z = pose["z"]
        request.yaw = pose["yaw"]
        request.pitch = pose["pitch"]
        request.roll = pose["roll"]

        self.get_logger().info(
            "Requesting relocalization at "
            f"x={request.x:.3f}, y={request.y:.3f}, z={request.z:.3f}, "
            f"yaw={request.yaw:.3f}, pitch={request.pitch:.3f}, "
            f"roll={request.roll:.3f}"
        )
        future = self.relocalize_client.call_async(request)
        remaining = max(0.0, deadline - time.monotonic())
        rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)
        if not future.done() or future.result() is None:
            self.get_logger().error("Relocalize service call timed out")
            return 4
        if not future.result().success:
            self.get_logger().error(
                f"Relocalize request rejected: {future.result().message}"
            )
            return 4

        self.get_logger().info(
            "3D map loaded; waiting for FAST-LIO2 scan-to-map alignment"
        )
        while rclpy.ok() and time.monotonic() < deadline:
            check_request = IsValid.Request()
            check_request.code = 0
            check_future = self.valid_client.call_async(check_request)
            rclpy.spin_until_future_complete(
                self, check_future, timeout_sec=min(check_period, 2.0)
            )
            if (
                check_future.done()
                and check_future.result() is not None
                and check_future.result().valid
            ):
                self.get_logger().info(
                    "Relocalization is valid; Nav2 may now start"
                )
                return 0
            time.sleep(check_period)

        self.get_logger().error(
            "Relocalization did not become valid before the timeout"
        )
        return 5


def main():
    rclpy.init()
    node = RelocalizationGate()
    try:
        return_code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        return_code = 130
    except Exception as error:  # Keep launch failure visible and block Nav2 startup.
        node.get_logger().error(f"Relocalization gate failed: {error}")
        return_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return return_code


if __name__ == "__main__":
    sys.exit(main())
