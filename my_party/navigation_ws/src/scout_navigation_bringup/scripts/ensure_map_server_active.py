#!/usr/bin/env python3
"""Reliably configure and activate Nav2's map_server at startup."""

import sys
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node


class MapServerActivationGuard(Node):
    """Drive map_server to ACTIVE, tolerating ROS discovery delays."""

    def __init__(self):
        super().__init__("map_server_activation_guard")
        self.declare_parameter("map_server_name", "/map_server")
        self.declare_parameter("timeout", 45.0)
        server = str(self.get_parameter("map_server_name").value).rstrip("/")
        self._get_state = self.create_client(GetState, f"{server}/get_state")
        self._change_state = self.create_client(ChangeState, f"{server}/change_state")

    def _call(self, client, request, timeout=5.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def _state(self):
        response = self._call(self._get_state, GetState.Request())
        return response.current_state if response is not None else None

    def _transition(self, transition_id, label):
        request = ChangeState.Request()
        request.transition.id = transition_id
        response = self._call(self._change_state, request, timeout=10.0)
        if response is None:
            self.get_logger().warning(f"map_server {label} request timed out; retrying")
            return False
        if not response.success:
            self.get_logger().warning(f"map_server rejected {label}; rechecking state")
            return False
        self.get_logger().info(f"map_server {label} succeeded")
        return True

    def run(self):
        timeout = float(self.get_parameter("timeout").value)
        deadline = time.monotonic() + timeout
        self.get_logger().info("Waiting for map_server lifecycle services")

        while rclpy.ok() and time.monotonic() < deadline:
            if not self._get_state.wait_for_service(timeout_sec=0.5):
                continue
            if not self._change_state.wait_for_service(timeout_sec=0.5):
                continue

            state = self._state()
            if state is None:
                continue
            if state.id == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info("map_server is active; static /map is available")
                return 0
            if state.id == State.PRIMARY_STATE_UNCONFIGURED:
                self._transition(Transition.TRANSITION_CONFIGURE, "configure")
            elif state.id == State.PRIMARY_STATE_INACTIVE:
                self._transition(Transition.TRANSITION_ACTIVATE, "activate")
            else:
                self.get_logger().info(
                    f"map_server is transitioning ({state.label}); waiting"
                )
            time.sleep(0.2)

        self.get_logger().error(
            f"map_server did not become active within {timeout:.1f} seconds"
        )
        return 1


def main():
    rclpy.init()
    node = MapServerActivationGuard()
    try:
        result = node.run()
    except (KeyboardInterrupt, RCLError):
        result = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
