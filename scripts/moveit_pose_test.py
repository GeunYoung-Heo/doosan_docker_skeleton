#!/usr/bin/env python3
"""End-effector pose goal test via MoveIt.

Sends a Cartesian pose goal to MoveIt's /move_action server. MoveIt solves IK
and plans a collision-free trajectory. The trajectory is executed through
dsr_moveit_controller → mock_components → /joint_states → Isaac Sim viewport.

This is the same interface CaP will use when it wants to command the robot
to a particular end-effector pose.

Usage (inside the doosan_isaac container):
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    python3 /ros2_ws/src/moveit_pose_test.py                    # default pose
    python3 /ros2_ws/src/moveit_pose_test.py --xyz 0.5 0.0 0.6  # custom xyz
    python3 /ros2_ws/src/moveit_pose_test.py --xyz 0.4 0.2 0.5 --quat 0 1 0 0  # with orientation
    python3 /ros2_ws/src/moveit_pose_test.py --position-only    # skip orientation constraint

Defaults (M1013 reachable, tip slightly forward and up):
    xyz  = (0.45, 0.0, 0.55) in base_link frame
    quat = (0, 1, 0, 0) = 180° around Y (tip pointing down)
"""

import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    WorkspaceParameters,
)
from shape_msgs.msg import SolidPrimitive


DEFAULT_XYZ = (0.45, 0.0, 0.55)
DEFAULT_QUAT = (0.0, 1.0, 0.0, 0.0)  # 180° around Y: tip down
PLANNING_FRAME = "base_link"
TIP_LINK = "link_6"
GROUP = "manipulator"


def build_pose_goal(xyz, quat, position_only, position_tol, orient_tol):
    goal = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = GROUP
    req.num_planning_attempts = 10
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.3
    req.max_acceleration_scaling_factor = 0.3
    req.pipeline_id = "ompl"

    ws = WorkspaceParameters()
    ws.header.frame_id = PLANNING_FRAME
    ws.min_corner.x, ws.min_corner.y, ws.min_corner.z = -1.5, -1.5, -0.5
    ws.max_corner.x, ws.max_corner.y, ws.max_corner.z = 1.5, 1.5, 1.5
    req.workspace_parameters = ws

    c = Constraints()
    c.name = "pose_target"

    # Position constraint — tip_link must reach xyz ± position_tol
    pc = PositionConstraint()
    pc.header.frame_id = PLANNING_FRAME
    pc.link_name = TIP_LINK
    pc.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [float(position_tol)]
    pc.constraint_region.primitives.append(sphere)
    pose = Pose()
    pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    pc.constraint_region.primitive_poses.append(pose)
    pc.weight = 1.0
    c.position_constraints.append(pc)

    # Orientation constraint — optional
    if not position_only:
        oc = OrientationConstraint()
        oc.header.frame_id = PLANNING_FRAME
        oc.link_name = TIP_LINK
        oc.orientation = Quaternion(
            x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3])
        )
        oc.absolute_x_axis_tolerance = float(orient_tol)
        oc.absolute_y_axis_tolerance = float(orient_tol)
        oc.absolute_z_axis_tolerance = float(orient_tol)
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

    req.goal_constraints.append(c)

    goal.request = req
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = False
    return goal


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--xyz", type=float, nargs=3, default=list(DEFAULT_XYZ),
                        metavar=("X", "Y", "Z"),
                        help=f"Target position in {PLANNING_FRAME} frame [m]")
    parser.add_argument("--quat", type=float, nargs=4, default=list(DEFAULT_QUAT),
                        metavar=("QX", "QY", "QZ", "QW"),
                        help=f"Target orientation (xyzw) for {TIP_LINK}")
    parser.add_argument("--position-only", action="store_true",
                        help="Only constrain position; let MoveIt pick any orientation")
    parser.add_argument("--position-tol", type=float, default=0.01,
                        help="Position constraint tolerance radius [m]")
    parser.add_argument("--orient-tol", type=float, default=0.1,
                        help="Orientation constraint tolerance per axis [rad]")
    parser.add_argument("--action", default="/move_action",
                        help="MoveGroup action server name")
    args = parser.parse_args()

    print(f"[pose_test] target xyz={args.xyz}", flush=True)
    if args.position_only:
        print(f"[pose_test] orientation: FREE (position only)", flush=True)
    else:
        print(f"[pose_test] target quat={args.quat}", flush=True)
    print(f"[pose_test] frame={PLANNING_FRAME}, tip={TIP_LINK}, group={GROUP}", flush=True)

    rclpy.init()
    node = rclpy.create_node("moveit_pose_test")
    client = ActionClient(node, MoveGroup, args.action)

    print(f"[pose_test] waiting for {args.action} ...", flush=True)
    if not client.wait_for_server(timeout_sec=15.0):
        print(f"[pose_test] FAIL: action server {args.action} not available")
        rclpy.shutdown()
        return 2

    goal = build_pose_goal(
        args.xyz, args.quat, args.position_only,
        args.position_tol, args.orient_tol,
    )

    print("[pose_test] sending goal ...", flush=True)
    t0 = time.time()
    sf = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sf, timeout_sec=10.0)
    gh = sf.result()
    if gh is None or not gh.accepted:
        print(f"[pose_test] FAIL: goal rejected (gh={gh})")
        rclpy.shutdown()
        return 3

    print("[pose_test] goal accepted, planning + executing ...", flush=True)
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=60.0)
    res = rf.result()
    total = time.time() - t0

    if res is None:
        print(f"[pose_test] FAIL: no result within 60s (total={total:.2f}s)")
        rclpy.shutdown()
        return 4

    code = res.result.error_code.val
    # moveit_msgs.msg.MoveItErrorCodes.SUCCESS = 1
    if code == 1:
        print(f"[pose_test] SUCCESS — total time {total:.2f}s", flush=True)
        rc = 0
    else:
        code_names = {
            -1: "FAILURE",
            -2: "PLANNING_FAILED",
            -3: "INVALID_MOTION_PLAN",
            -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
            -5: "CONTROL_FAILED",
            -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
            -7: "TIMED_OUT",
            -10: "START_STATE_IN_COLLISION",
            -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
            -12: "GOAL_IN_COLLISION",
            -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
            -14: "GOAL_CONSTRAINTS_VIOLATED",
            -15: "INVALID_GROUP_NAME",
            -31: "NO_IK_SOLUTION",
        }
        name = code_names.get(code, f"code={code}")
        print(f"[pose_test] FAILED: {name} (total={total:.2f}s)")
        rc = 5

    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
