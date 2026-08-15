#!/usr/bin/env python3
"""Drive a measured straight startup distance without consulting Nav2 costmaps.

This is intended only for clearing harmless side-pointcloud/costmap clutter at
the beginning of a mission.  It never ignores a real object in the forward
travel corridor: motion pauses until that corridor is clear.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class SafeStartupForward(Node):
    def __init__(self):
        super().__init__("safe_startup_forward")
        self.declare_parameter("distance", 2.0)
        self.declare_parameter("speed", 0.5)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cloud_topic", "/fastlio2/body_cloud")
        self.declare_parameter("front_stop_distance", 0.80)
        self.declare_parameter("front_half_width", 0.35)
        self.declare_parameter("front_min_points", 4)
        self.declare_parameter("cloud_stale_timeout", 0.50)

        self._publisher = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self._pose = None
        self._cloud_time = None
        self._front_blocked = True
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._odom, 20
        )
        self.create_subscription(
            PointCloud2, str(self.get_parameter("cloud_topic").value), self._cloud, 10
        )

    def _odom(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._pose = (p.x, p.y, yaw, time.monotonic())

    def _cloud(self, cloud):
        count = 0
        stop_distance = float(self.get_parameter("front_stop_distance").value)
        half_width = float(self.get_parameter("front_half_width").value)
        minimum = int(self.get_parameter("front_min_points").value)
        try:
            for x, y, z in point_cloud2.read_points(
                cloud, field_names=("x", "y", "z"), skip_nans=True
            ):
                if 0.20 < x < stop_distance and abs(y) < half_width and -0.15 < z < 1.70:
                    count += 1
                    if count >= minimum:
                        self._front_blocked = True
                        self._cloud_time = time.monotonic()
                        return
        except Exception as error:
            self.get_logger().warn(f"Cannot parse startup safety cloud: {error}")
            self._front_blocked = True
            self._cloud_time = time.monotonic()
            return
        self._front_blocked = False
        self._cloud_time = time.monotonic()

    def _stop(self):
        self._publisher.publish(Twist())

    def _front_is_safe(self):
        if self._cloud_time is None:
            return False
        return (
            time.monotonic() - self._cloud_time
            <= float(self.get_parameter("cloud_stale_timeout").value)
            and not self._front_blocked
        )

    def run(self):
        distance = float(self.get_parameter("distance").value)
        speed = float(self.get_parameter("speed").value)
        if distance <= 0.0 or speed <= 0.0:
            self.get_logger().error("distance and speed must both be positive")
            return 2

        self.get_logger().info(
            f"Startup forward enabled: travel {distance:.2f} m at {speed:.2f} m/s; "
            "Nav2 local costmap is bypassed, forward emergency protection remains on"
        )
        start = None
        while rclpy.ok() and start is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._pose is not None and time.monotonic() - self._pose[3] < 0.5:
                start = self._pose
        if start is None:
            return 3

        x0, y0, yaw0, _ = start
        last_warning = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._pose is None:
                self._stop()
                continue
            x, y, _yaw, stamp = self._pose
            if time.monotonic() - stamp > 0.5:
                self._stop()
                continue
            traveled = math.cos(yaw0) * (x - x0) + math.sin(yaw0) * (y - y0)
            if traveled >= distance:
                self._stop()
                self.get_logger().info(f"Startup forward completed: {traveled:.2f} m")
                return 0
            if not self._front_is_safe():
                self._stop()
                if time.monotonic() - last_warning >= 2.0:
                    self.get_logger().warning(
                        "Startup forward paused: forward safety corridor is occupied or stale"
                    )
                    last_warning = time.monotonic()
                continue
            command = Twist()
            command.linear.x = speed
            self._publisher.publish(command)
        self._stop()
        return 130


def main():
    rclpy.init()
    node = SafeStartupForward()
    try:
        result = node.run()
    except KeyboardInterrupt:
        result = 130
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(result)


if __name__ == "__main__":
    main()
