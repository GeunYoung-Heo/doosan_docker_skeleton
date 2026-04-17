#!/usr/bin/env python3
"""
ROS 2 driver node for OnRobot RG2-FT V2.

Service : /onrobot_rg2_ft/set_gripper  (onrobot_rg2_ft/srv/SetGripper)
Topic   : /onrobot_rg2_ft/state        (onrobot_rg2_ft/msg/GripperState) @ state_hz Hz

ROS parameters (settable via --ros-args -p <name>:=<value>):
  host        (str,   default '192.168.1.1')  Compute Box IP
  timeout_sec (float, default 3.0)            HTTP request timeout
  state_hz    (float, default 5.0)            State publish rate

Run:
  ros2 run onrobot_rg2_ft gripper_node
  ros2 run onrobot_rg2_ft gripper_node --ros-args -p host:=192.168.1.1
"""

import os
import sys

# compute_box_client.py is installed alongside this script in lib/onrobot_rg2_ft/
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from compute_box_client import ComputeBoxClient  # noqa: E402

import rclpy
from rclpy.node import Node

from onrobot_rg2_ft.srv import SetGripper
from onrobot_rg2_ft.msg import GripperState


class GripperNode(Node):
    def __init__(self) -> None:
        super().__init__("onrobot_rg2_ft_node")

        self.declare_parameter("host", "192.168.1.1")
        self.declare_parameter("timeout_sec", 3.0)
        self.declare_parameter("state_hz", 5.0)

        host = self.get_parameter("host").get_parameter_value().string_value
        timeout = self.get_parameter("timeout_sec").get_parameter_value().double_value
        state_hz = self.get_parameter("state_hz").get_parameter_value().double_value

        self._client = ComputeBoxClient(host=host, timeout=timeout)

        self.get_logger().info(f"Connecting to Compute Box at {host}:502 (Modbus TCP) ...")
        if not self._client.ping():
            self.get_logger().warn(
                "Compute Box Modbus port did not respond at startup. "
                "Ensure 192.168.1.x route is active on the host (ip addr add)."
            )
        else:
            self.get_logger().info("Compute Box reachable.")

        self._srv = self.create_service(
            SetGripper, "~/set_gripper", self._handle_set_gripper
        )
        self._pub = self.create_publisher(GripperState, "~/state", 10)
        self._timer = self.create_timer(1.0 / state_hz, self._publish_state)

        self.get_logger().info(
            "onrobot_rg2_ft_node ready. "
            "Service: ~/set_gripper  Topic: ~/state"
        )

    # ------------------------------------------------------------------
    def _handle_set_gripper(
        self, request: SetGripper.Request, response: SetGripper.Response
    ) -> SetGripper.Response:
        width = request.width
        force = request.force
        self.get_logger().info(f"set_gripper: width={width} mm, force={force} N")

        try:
            self._client.command(width_mm=width, force_n=force)
            response.success = True
            response.message = "OK"
        except ValueError as exc:
            response.success = False
            response.message = f"Invalid parameter: {exc}"
            self.get_logger().error(str(exc))
        except OSError as exc:
            response.success = False
            response.message = f"Compute Box unreachable: {exc}"
            self.get_logger().error(f"Compute Box unreachable: {exc}")
        except Exception as exc:
            response.success = False
            response.message = f"Unexpected error: {exc}"
            self.get_logger().error(f"Unexpected error: {exc}")

        return response

    # ------------------------------------------------------------------
    def _publish_state(self) -> None:
        try:
            state = self._client.get_state()
        except Exception as exc:
            self.get_logger().warn(f"State poll failed: {exc}", throttle_duration_sec=5.0)
            return

        msg = GripperState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.actual_width = state["actual_width"]
        msg.busy = state["busy"]
        msg.gripped = state["gripped"]
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._client.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
