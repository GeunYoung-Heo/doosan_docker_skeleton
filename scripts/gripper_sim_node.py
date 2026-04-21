#!/usr/bin/env python3
"""Sim-mode gripper node — same ROS interface as the real gripper_node.py.

Exposes:
    Service: /onrobot_rg2_ft_node/set_gripper (onrobot_rg2_ft/srv/SetGripper)
    Topic:   /onrobot_rg2_ft_node/state       (onrobot_rg2_ft/msg/GripperState) @ 5 Hz

Internally converts width(mm) → finger_joint(rad) and publishes the target
on /gripper_finger_target (std_msgs/Float64) for the Isaac Sim bridge.

Force is logged but not physically applied (sim limitation — no contact
force simulation without scene objects). gripped is determined by a simple
width threshold (< 5 mm).

Run (inside container, after onrobot_rg2_ft package is built):
    python3 /ros2_ws/src/gripper_sim_node.py
Or launched from m1013_sim_bringup.launch.py.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from onrobot_rg2_ft.srv import SetGripper
from onrobot_rg2_ft.msg import GripperState

# RG2-FT specs
MAX_WIDTH_MM = 110.0
MAX_FINGER_RAD = 1.18
GRIP_THRESHOLD_MM = 5.0


def width_to_rad(width_mm: float) -> float:
    """Convert gripper width (mm) to finger_joint position (rad).
    width=110 → rad=0 (open), width=0 → rad=1.18 (closed)."""
    w = max(0.0, min(MAX_WIDTH_MM, width_mm))
    return (MAX_WIDTH_MM - w) / MAX_WIDTH_MM * MAX_FINGER_RAD


def rad_to_width(rad: float) -> float:
    """Convert finger_joint position (rad) to gripper width (mm)."""
    r = max(0.0, min(MAX_FINGER_RAD, rad))
    return MAX_WIDTH_MM * (1.0 - r / MAX_FINGER_RAD)


class GripperSimNode(Node):
    def __init__(self):
        # Use same node name as real gripper for identical topic/service paths
        super().__init__("onrobot_rg2_ft_node")

        self.current_width_mm = MAX_WIDTH_MM  # start fully open
        self.current_rad = 0.0
        self.busy = False
        self.gripped = False
        self.last_force = 0.0

        # Service (same as real — ~/name uses node name as prefix)
        self.srv = self.create_service(
            SetGripper, "~/set_gripper", self.set_gripper_cb
        )

        # State publisher (same as real, 5 Hz)
        self.state_pub = self.create_publisher(GripperState, "~/state", 10)
        self.state_timer = self.create_timer(0.2, self.publish_state)

        # Finger target publisher (consumed by Isaac Sim bridge)
        self.finger_pub = self.create_publisher(
            Float64, "/gripper_finger_target", 10
        )

        self.get_logger().info(
            f"Sim gripper ready. width={self.current_width_mm:.0f} mm (open)"
        )

    def set_gripper_cb(self, request, response):
        target_width = max(0.0, min(MAX_WIDTH_MM, request.width))
        force = max(0.0, min(40.0, request.force))

        self.get_logger().info(
            f"set_gripper: width={target_width:.1f} mm, force={force:.1f} N"
        )

        self.busy = True
        self.current_width_mm = target_width
        self.current_rad = width_to_rad(target_width)
        self.gripped = target_width < GRIP_THRESHOLD_MM
        self.last_force = force

        # Publish finger target for Isaac Sim bridge
        msg = Float64()
        msg.data = self.current_rad
        self.finger_pub.publish(msg)

        self.busy = False

        response.success = True
        response.message = (
            f"Sim: width={target_width:.1f} mm "
            f"(rad={self.current_rad:.3f}, gripped={self.gripped})"
        )
        if force > 0:
            response.message += f" [force={force:.1f} N logged, not applied in sim]"

        return response

    def publish_state(self):
        msg = GripperState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.actual_width = self.current_width_mm
        msg.busy = self.busy
        msg.gripped = self.gripped
        self.state_pub.publish(msg)

        # Keep publishing finger target so bridge stays in sync
        ft = Float64()
        ft.data = self.current_rad
        self.finger_pub.publish(ft)


def main():
    rclpy.init()
    node = GripperSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
