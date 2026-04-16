#!/usr/bin/env python3
"""Minimal MoveGroup action smoke test — go to all-zeros joint target.

Run inside the doosan_isaac container (with ROS 2 + dsr_moveit_bringup up):
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    python3 /ros2_ws/src/moveit_backend_smoketest.py

Exits 0 on success, non-zero on plan/execute failure.
"""
import sys
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    WorkspaceParameters,
)


JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def build_joint_goal(positions):
    goal = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = "manipulator"
    req.num_planning_attempts = 10
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.3
    req.max_acceleration_scaling_factor = 0.3
    req.pipeline_id = "ompl"

    ws = WorkspaceParameters()
    ws.header.frame_id = "base_link"
    ws.min_corner.x, ws.min_corner.y, ws.min_corner.z = -1.5, -1.5, -0.5
    ws.max_corner.x, ws.max_corner.y, ws.max_corner.z = 1.5, 1.5, 1.5
    req.workspace_parameters = ws

    c = Constraints()
    c.name = "joint_goal"
    for name, pos in zip(JOINT_NAMES, positions):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = float(pos)
        jc.tolerance_above = 0.001
        jc.tolerance_below = 0.001
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    req.goal_constraints.append(c)

    goal.request = req
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = False
    return goal


def main():
    rclpy.init()
    node = rclpy.create_node("moveit_smoketest")

    action = "/move_action"
    client = ActionClient(node, MoveGroup, action)
    print(f"[smoketest] waiting for action server {action} ...", flush=True)
    if not client.wait_for_server(timeout_sec=15.0):
        print(f"[smoketest] FAIL: action server not available", flush=True)
        rclpy.shutdown()
        return 2

    print("[smoketest] sending joint-space goal -> all zeros", flush=True)
    send_fut = client.send_goal_async(build_joint_goal(HOME))
    rclpy.spin_until_future_complete(node, send_fut, timeout_sec=10.0)
    gh = send_fut.result()
    if gh is None or not gh.accepted:
        print(f"[smoketest] FAIL: goal rejected (gh={gh})", flush=True)
        rclpy.shutdown()
        return 3

    print("[smoketest] goal accepted, waiting for result ...", flush=True)
    result_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, result_fut, timeout_sec=30.0)
    result = result_fut.result()
    if result is None:
        print(f"[smoketest] FAIL: no result within timeout", flush=True)
        rclpy.shutdown()
        return 4

    code = result.result.error_code.val
    print(f"[smoketest] result error_code.val = {code}", flush=True)
    # SUCCESS = 1 (see moveit_msgs/msg/MoveItErrorCodes)
    if code == 1:
        print("[smoketest] SUCCESS — motion plan + execute completed", flush=True)
        rc = 0
    else:
        print(f"[smoketest] FAIL — error code {code}", flush=True)
        rc = 5

    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
