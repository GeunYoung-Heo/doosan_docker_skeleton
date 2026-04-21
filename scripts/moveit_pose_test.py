#!/usr/bin/env python3
"""MoveIt goal test — Cartesian pose, joint-space, or straight-line path.

Usage (inside the doosan_isaac container):
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash

    # Cartesian pose goal (OMPL — may take indirect path)
    python3 /ros2_ws/src/moveit_pose_test.py --xyz 0.5 0.0 0.6
    python3 /ros2_ws/src/moveit_pose_test.py --xyz 0.4 0.2 0.5 --quat 0 1 0 0
    python3 /ros2_ws/src/moveit_pose_test.py --position-only

    # Straight-line path (EE moves in a line — predictable, no big detours)
    python3 /ros2_ws/src/moveit_pose_test.py --xyz 0.45 0.0 0.35 --cartesian
    python3 /ros2_ws/src/moveit_pose_test.py --xyz 0.45 0.0 0.55 --cartesian

    # Joint-space goal (degrees — same convention as doosan move_joint service)
    python3 /ros2_ws/src/moveit_pose_test.py --joints 0 0 90 0 90 0
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
    WorkspaceParameters,
)
from moveit_msgs.srv import GetCartesianPath
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


DEFAULT_XYZ = (0.45, 0.0, 0.55)
DEFAULT_QUAT = (0.0, 1.0, 0.0, 0.0)  # 180° around Y: tip down
PLANNING_FRAME = "base_link"
TIP_LINK = "link_6"
GROUP = "manipulator"
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


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


def build_joint_goal(joint_degrees):
    """Build a MoveGroup.Goal for a joint-space target.

    joint_degrees: list of 6 floats in DEGREES (same convention as doosan
    move_joint service). Converted to radians internally.
    """
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
    c.name = "joint_goal"
    for name, deg in zip(JOINT_NAMES, joint_degrees):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = math.radians(float(deg))
        jc.tolerance_above = 0.001
        jc.tolerance_below = 0.001
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    req.goal_constraints.append(c)

    goal.request = req
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = False
    return goal


def execute_cartesian_path(node, xyz, quat, max_step, vel_scale):
    """Plan a straight-line Cartesian path to target and execute it.

    Uses /compute_cartesian_path service + /execute_trajectory action.
    The EE moves in a straight line — no big arm reconfigurations.
    """
    # 1) Get current joint state
    print("[moveit_test] waiting for /joint_states ...", flush=True)
    joint_state_msg = None
    def js_cb(msg):
        nonlocal joint_state_msg
        joint_state_msg = msg
    sub = node.create_subscription(JointState, "/joint_states", js_cb, 1)
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if joint_state_msg:
            break
    node.destroy_subscription(sub)
    if not joint_state_msg:
        print("[moveit_test] FAIL: no /joint_states received")
        return 6

    # 2) Build target pose
    target_pose = Pose()
    target_pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
    target_pose.orientation = Quaternion(
        x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3])
    )

    # 3) Call /compute_cartesian_path service
    cart_client = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    print("[moveit_test] waiting for /compute_cartesian_path ...", flush=True)
    if not cart_client.wait_for_service(timeout_sec=10.0):
        print("[moveit_test] FAIL: /compute_cartesian_path not available")
        return 7

    req = GetCartesianPath.Request()
    req.header.frame_id = PLANNING_FRAME
    req.group_name = GROUP
    req.link_name = TIP_LINK
    req.max_step = max_step
    req.jump_threshold = 0.0  # disabled
    req.avoid_collisions = True
    req.start_state = RobotState()
    req.start_state.joint_state = joint_state_msg
    req.waypoints = [target_pose]

    print("[moveit_test] computing Cartesian path ...", flush=True)
    # Fast DDS discovery workaround: small delay before service call
    time.sleep(0.5)
    t0 = time.time()

    # Retry up to 3x on DDS response timeout (Fast DDS "failed to send response")
    resp = None
    for attempt in range(3):
        future = cart_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        resp = future.result()
        if resp is not None:
            break
        print(f"[moveit_test] attempt {attempt+1}/3 timed out, retrying ...", flush=True)
        time.sleep(1.0)

    if resp is None:
        print("[moveit_test] FAIL: no response from compute_cartesian_path after 3 retries")
        return 8

    if resp.fraction < 0.99:
        print(f"[moveit_test] FAIL: only {resp.fraction*100:.1f}% of path achievable (need ~100%)")
        return 9

    n_pts = len(resp.solution.joint_trajectory.points)
    print(f"[moveit_test] path computed: {resp.fraction*100:.0f}% achievable, "
          f"{n_pts} waypoints", flush=True)

    # 3.5) Scale trajectory speed
    # vel_scale=1.0 → original speed, 0.3 → ~3.3x slower, 0.1 → 10x slower
    if vel_scale > 0 and vel_scale != 1.0:
        scale_factor = 1.0 / vel_scale
        for pt in resp.solution.joint_trajectory.points:
            t_sec = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            t_sec *= scale_factor
            pt.time_from_start.sec = int(t_sec)
            pt.time_from_start.nanosec = int((t_sec % 1) * 1e9)
            # Also scale velocities down
            pt.velocities = [v * vel_scale for v in pt.velocities]
            pt.accelerations = [a * vel_scale * vel_scale for a in pt.accelerations]
        print(f"[moveit_test] speed scaled: {vel_scale:.0%} of max", flush=True)

    # 4) Execute via /execute_trajectory action
    exec_client = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
    if not exec_client.wait_for_server(timeout_sec=10.0):
        print("[moveit_test] FAIL: /execute_trajectory action not available")
        return 10

    exec_goal = ExecuteTrajectory.Goal()
    exec_goal.trajectory = resp.solution

    print("[moveit_test] executing straight-line trajectory ...", flush=True)
    sf = exec_client.send_goal_async(exec_goal)
    rclpy.spin_until_future_complete(node, sf, timeout_sec=10.0)
    gh = sf.result()
    if not gh or not gh.accepted:
        print("[moveit_test] FAIL: execution rejected")
        return 11

    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=60.0)
    res = rf.result()
    total = time.time() - t0

    if res and res.result.error_code.val == 1:
        print(f"[moveit_test] SUCCESS (cartesian) — {total:.2f}s", flush=True)
        return 0
    else:
        code = res.result.error_code.val if res else "?"
        print(f"[moveit_test] FAIL: execution error {code} ({total:.2f}s)")
        return 12


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--joints", type=float, nargs=6, default=None,
                        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                        help="Joint-space target in DEGREES (e.g. --joints 0 0 90 0 90 0)")
    parser.add_argument("--xyz", type=float, nargs=3, default=list(DEFAULT_XYZ),
                        metavar=("X", "Y", "Z"),
                        help=f"Target position in {PLANNING_FRAME} frame [m]")
    parser.add_argument("--quat", type=float, nargs=4, default=list(DEFAULT_QUAT),
                        metavar=("QX", "QY", "QZ", "QW"),
                        help=f"Target orientation (xyzw) for {TIP_LINK}")
    parser.add_argument("--cartesian", action="store_true",
                        help="Use straight-line Cartesian path (EE moves in a line, no detours)")
    parser.add_argument("--position-only", action="store_true",
                        help="Only constrain position; let MoveIt pick any orientation")
    parser.add_argument("--max-step", type=float, default=0.005,
                        help="Cartesian path resolution in meters (default: 5mm)")
    parser.add_argument("--vel-scale", type=float, default=0.3,
                        help="Velocity scaling factor (default: 0.3)")
    parser.add_argument("--position-tol", type=float, default=0.01,
                        help="Position constraint tolerance radius [m]")
    parser.add_argument("--orient-tol", type=float, default=0.1,
                        help="Orientation constraint tolerance per axis [rad]")
    parser.add_argument("--action", default="/move_action",
                        help="MoveGroup action server name")
    args = parser.parse_args()

    is_joint_goal = args.joints is not None

    if is_joint_goal:
        print(f"[moveit_test] JOINT goal (deg): {args.joints}", flush=True)
    elif args.cartesian:
        print(f"[moveit_test] CARTESIAN straight-line to xyz={args.xyz}", flush=True)
        print(f"[moveit_test] quat={args.quat}, max_step={args.max_step}m", flush=True)
    else:
        print(f"[moveit_test] OMPL pose goal xyz={args.xyz}", flush=True)
        if args.position_only:
            print(f"[moveit_test] orientation: FREE", flush=True)
        else:
            print(f"[moveit_test] quat={args.quat}", flush=True)

    rclpy.init()
    node = rclpy.create_node("moveit_test")

    # --- Cartesian straight-line path ---
    if args.cartesian and not is_joint_goal:
        rc = execute_cartesian_path(
            node, args.xyz, args.quat, args.max_step, args.vel_scale,
        )
        rclpy.shutdown()
        return rc

    # --- OMPL joint or pose goal ---
    client = ActionClient(node, MoveGroup, args.action)
    print(f"[moveit_test] waiting for {args.action} ...", flush=True)
    if not client.wait_for_server(timeout_sec=15.0):
        print(f"[moveit_test] FAIL: action server {args.action} not available")
        rclpy.shutdown()
        return 2

    if is_joint_goal:
        goal = build_joint_goal(args.joints)
    else:
        goal = build_pose_goal(
            args.xyz, args.quat, args.position_only,
            args.position_tol, args.orient_tol,
        )

    print("[moveit_test] sending goal ...", flush=True)
    t0 = time.time()
    sf = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sf, timeout_sec=10.0)
    gh = sf.result()
    if gh is None or not gh.accepted:
        print(f"[moveit_test] FAIL: goal rejected (gh={gh})")
        rclpy.shutdown()
        return 3

    print("[moveit_test] goal accepted, planning + executing ...", flush=True)
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=60.0)
    res = rf.result()
    total = time.time() - t0

    if res is None:
        print(f"[moveit_test] FAIL: no result within 60s (total={total:.2f}s)")
        rclpy.shutdown()
        return 4

    code = res.result.error_code.val
    if code == 1:
        print(f"[moveit_test] SUCCESS — total time {total:.2f}s", flush=True)
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
        print(f"[moveit_test] FAILED: {name} (total={total:.2f}s)")
        rc = 5

    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
