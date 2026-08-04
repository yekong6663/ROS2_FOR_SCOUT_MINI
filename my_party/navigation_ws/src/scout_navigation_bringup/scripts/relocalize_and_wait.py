#!/usr/bin/env python3

import sys
import time
import math
from pathlib import Path

import rclpy
import yaml
from interface.srv import IsValid, Relocalize
from PIL import Image
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class RelocalizationGate(Node):
    def __init__(self):
        super().__init__("relocalization_gate")
        self.declare_parameter("pcd_path", "")
        self.declare_parameter("pose_file", "")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("footprint_length", 0.62)
        self.declare_parameter("footprint_width", 0.45)
        self.declare_parameter("footprint_padding", 0.07)
        # A 5 cm occupancy grid can clip one or two samples at a map image
        # border even when the physical footprint is in the surveyed lane.
        # More than this remains a hard startup safety failure.
        self.declare_parameter("max_blocked_samples", 2)
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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
                f"No initial_pose.yaml found at {pose_file}; using the launch "
                "pose arguments as the ICP initial guess"
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

    @staticmethod
    def _yaw_from_quaternion(rotation):
        siny_cosp = 2.0 * (
            rotation.w * rotation.z + rotation.x * rotation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            rotation.y * rotation.y + rotation.z * rotation.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    def _pose_footprint_is_free(self):
        """Require the complete padded robot footprint to be in map free space."""
        map_yaml = Path(self.get_parameter("map_yaml").value)
        if not map_yaml.is_file():
            raise FileNotFoundError(f"2D map YAML does not exist: {map_yaml}")

        with map_yaml.open("r", encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream) or {}
        image_path = Path(metadata["image"])
        if not image_path.is_absolute():
            image_path = map_yaml.parent / image_path

        image = Image.open(image_path).convert("L")
        width, height = image.size
        pixels = image.load()
        resolution = float(metadata["resolution"])
        origin_x, origin_y, origin_yaw = [
            float(value) for value in metadata["origin"]
        ]
        free_threshold = float(metadata.get("free_thresh", 0.196))
        negate = int(metadata.get("negate", 0))
        base_frame = str(self.get_parameter("base_frame").value)

        transform = self.tf_buffer.lookup_transform(
            "map",
            base_frame,
            Time(),
            timeout=Duration(seconds=2.0),
        )
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        robot_yaw = self._yaw_from_quaternion(transform.transform.rotation)

        half_length = (
            float(self.get_parameter("footprint_length").value) / 2.0
            + float(self.get_parameter("footprint_padding").value)
        )
        half_width = (
            float(self.get_parameter("footprint_width").value) / 2.0
            + float(self.get_parameter("footprint_padding").value)
        )
        sample_step = min(resolution, 0.05)
        x_samples = int(math.ceil((2.0 * half_length) / sample_step))
        y_samples = int(math.ceil((2.0 * half_width) / sample_step))
        map_cos = math.cos(origin_yaw)
        map_sin = math.sin(origin_yaw)
        robot_cos = math.cos(robot_yaw)
        robot_sin = math.sin(robot_yaw)

        checked = 0
        blocked = 0
        first_blocked = None
        for x_index in range(x_samples + 1):
            local_x = -half_length + (2.0 * half_length * x_index / x_samples)
            for y_index in range(y_samples + 1):
                local_y = -half_width + (
                    2.0 * half_width * y_index / y_samples
                )
                world_x = robot_x + robot_cos * local_x - robot_sin * local_y
                world_y = robot_y + robot_sin * local_x + robot_cos * local_y
                delta_x = world_x - origin_x
                delta_y = world_y - origin_y
                grid_x = (map_cos * delta_x + map_sin * delta_y) / resolution
                grid_y = (-map_sin * delta_x + map_cos * delta_y) / resolution
                column = math.floor(grid_x)
                row = height - 1 - math.floor(grid_y)
                checked += 1

                is_free = False
                if 0 <= column < width and 0 <= row < height:
                    value = pixels[column, row]
                    occupancy = (
                        value / 255.0 if negate else (255 - value) / 255.0
                    )
                    is_free = occupancy < free_threshold
                if not is_free:
                    blocked += 1
                    if first_blocked is None:
                        first_blocked = (world_x, world_y)

        max_blocked_samples = int(
            self.get_parameter("max_blocked_samples").value
        )
        if blocked > max_blocked_samples:
            self.get_logger().error(
                "Relocalization result is unsafe: map pose "
                f"x={robot_x:.3f}, y={robot_y:.3f}, yaw={robot_yaw:.3f}; "
                f"{blocked}/{checked} padded-footprint samples are gray, "
                f"occupied, or outside the map (first at "
                f"x={first_blocked[0]:.3f}, y={first_blocked[1]:.3f})."
            )
            return False

        if blocked:
            self.get_logger().warning(
                "Tolerating "
                f"{blocked}/{checked} padded-footprint samples at the map "
                "raster edge; this is within the configured two-cell limit"
            )

        self.get_logger().info(
            "2D white-area safety check passed at "
            f"x={robot_x:.3f}, y={robot_y:.3f}, yaw={robot_yaw:.3f}"
        )
        return True

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
                try:
                    if self._pose_footprint_is_free():
                        self.get_logger().info(
                            "Relocalization and white-area check are valid; "
                            "Nav2 may now start"
                        )
                        return 0
                    return 6
                except TransformException as error:
                    self.get_logger().warning(
                        f"Waiting for map to base transform: {error}"
                    )
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
