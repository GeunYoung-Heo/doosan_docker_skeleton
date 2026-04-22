#!/usr/bin/env python3
"""Pick-and-place scenario: three-cube stacking with RG2-FT gripper.

One persistent rclpy node — avoids the Fast DDS discovery / service response
timeout issue that shows up when bash chains many `moveit_pose_test.py` calls.

Sequence:
  1. Go to ready pose (joints)
  2. Open gripper
  3. Cube1: approach → grasp → lift → move over cube2 → drop
  4. Cube3: approach → grasp → lift → move over stack → drop
  5. Return home

Usage (inside the doosan_isaac container):
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    python3 /ros2_ws/src/scenarios/cube_stacking.py
"""

import math
import os
import sys
import time

# Reuse planning helpers from moveit_pose_test.py
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from moveit_pose_test import (  # noqa: E402
    DEFAULT_QUAT,
    GROUP,
    JOINT_NAMES,
    PLANNING_FRAME,
    TIP_LINK,
    build_joint_goal,
)

import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402

from geometry_msgs.msg import Point, Pose, Quaternion  # noqa: E402
from moveit_msgs.action import ExecuteTrajectory, MoveGroup  # noqa: E402
from moveit_msgs.msg import RobotState  # noqa: E402
from moveit_msgs.srv import GetCartesianPath  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

from onrobot_rg2_ft.srv import SetGripper  # noqa: E402


READY_JOINTS_DEG = [0, 0, -90, 0, -90, 0]


class ScenarioRunner(Node):
    """Single long-lived node that runs the whole pick-and-place scenario."""

    def __init__(self):
        super().__init__("cube_stacking_scenario")

        # --- Cached state ---
        self._latest_joint_state = None
        self.create_subscription(
            JointState, "/joint_states", self._js_cb, 10
        )

        # --- Clients / action clients (created once, reused) ---
        self._move_action = ActionClient(self, MoveGroup, "/move_action")
        self._exec_action = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self._cart_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._gripper_client = self.create_client(
            SetGripper, "/onrobot_rg2_ft_node/set_gripper"
        )

        self._wait_for_all()

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------
    def _js_cb(self, msg):
        self._latest_joint_state = msg

    def _wait_for_all(self):
        self.get_logger().info("Waiting for MoveIt + gripper services ...")
        if not self._move_action.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("/move_action not available")
        if not self._exec_action.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("/execute_trajectory not available")
        if not self._cart_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("/compute_cartesian_path not available")
        if not self._gripper_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("/onrobot_rg2_ft_node/set_gripper not available")

        # Wait for first joint state
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_joint_state is not None:
                break
        if self._latest_joint_state is None:
            raise RuntimeError("No /joint_states received")

        self.get_logger().info("All endpoints ready.")

    # ------------------------------------------------------------------
    # Primitives
    # ------------------------------------------------------------------
    def move_joints(self, joints_deg, label=""):
        """Joint-space move via MoveGroup action (OMPL planner)."""
        self.get_logger().info(f"[joint] {label} → {joints_deg}°")
        goal = build_joint_goal(joints_deg)

        sf = self._move_action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sf, timeout_sec=10.0)
        gh = sf.result()
        if gh is None or not gh.accepted:
            raise RuntimeError(f"Joint goal rejected: {label}")

        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=60.0)
        res = rf.result()
        if res is None or res.result.error_code.val != 1:
            code = res.result.error_code.val if res else "no-result"
            raise RuntimeError(f"Joint move failed ({label}): code={code}")

    def move_cartesian(self, xyz, label="", vel_scale=0.3, max_step=0.005):
        """Straight-line Cartesian move using /compute_cartesian_path + /execute_trajectory."""
        self.get_logger().info(f"[cart] {label} → xyz={xyz}")

        target = Pose()
        target.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        target.orientation = Quaternion(
            x=DEFAULT_QUAT[0], y=DEFAULT_QUAT[1],
            z=DEFAULT_QUAT[2], w=DEFAULT_QUAT[3],
        )

        req = GetCartesianPath.Request()
        req.header.frame_id = PLANNING_FRAME
        req.group_name = GROUP
        req.link_name = TIP_LINK
        req.max_step = max_step
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.start_state = RobotState()
        req.start_state.joint_state = self._latest_joint_state
        req.waypoints = [target]

        # Retry compute_cartesian_path up to 3× (rare Fast DDS blip)
        resp = None
        for attempt in range(3):
            future = self._cart_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            resp = future.result()
            if resp is not None:
                break
            self.get_logger().warn(
                f"compute_cartesian_path attempt {attempt+1}/3 timed out, retrying"
            )
            time.sleep(0.5)
        if resp is None:
            raise RuntimeError(f"compute_cartesian_path no response ({label})")
        if resp.fraction < 0.99:
            raise RuntimeError(
                f"Cartesian path only {resp.fraction*100:.1f}% achievable ({label})"
            )

        # Scale velocity
        if vel_scale > 0 and vel_scale != 1.0:
            scale_factor = 1.0 / vel_scale
            for pt in resp.solution.joint_trajectory.points:
                t_sec = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                t_sec *= scale_factor
                pt.time_from_start.sec = int(t_sec)
                pt.time_from_start.nanosec = int((t_sec % 1) * 1e9)
                pt.velocities = [v * vel_scale for v in pt.velocities]
                pt.accelerations = [a * vel_scale * vel_scale for a in pt.accelerations]

        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = resp.solution

        sf = self._exec_action.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, sf, timeout_sec=10.0)
        gh = sf.result()
        if not gh or not gh.accepted:
            raise RuntimeError(f"Cartesian execute rejected ({label})")

        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=60.0)
        res = rf.result()
        if not res or res.result.error_code.val != 1:
            code = res.result.error_code.val if res else "no-result"
            raise RuntimeError(f"Cartesian execution failed ({label}): code={code}")

    def set_gripper(self, width_mm, force_n, settle_sec=1.5, label=""):
        """Send gripper command, wait for physical motion to settle."""
        self.get_logger().info(
            f"[grip] {label} → width={width_mm}mm force={force_n}N"
        )
        req = SetGripper.Request()
        req.width = float(width_mm)
        req.force = float(force_n)

        future = self._gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        if resp is None or not resp.success:
            msg = resp.message if resp else "no-response"
            raise RuntimeError(f"Gripper command failed ({label}): {msg}")

        # Modbus returns instantly; wait for physical fingers to reach target
        time.sleep(settle_sec)


# ----------------------------------------------------------------------
# Scenario
# ----------------------------------------------------------------------
def run_scenario(runner: ScenarioRunner):
    # --- Ready pose + open gripper ---
    runner.move_joints(READY_JOINTS_DEG, label="ready")
    runner.set_gripper(110.0, 5.0, settle_sec=1.0, label="open")

    # --- Pick cube1 ---
    runner.move_cartesian((-0.45, 0.0, 0.55), label="above cube1")
    runner.move_cartesian((-0.45, 0.0, 0.24), label="grab cube1")
    runner.set_gripper(40.0, 5.0, settle_sec=2.0, label="grasp cube1")

    # --- Place cube1 on cube2 ---
    runner.move_cartesian((-0.45,  0.0, 0.55), label="lift cube1")
    runner.move_cartesian((-0.45, -0.4, 0.55), label="above cube2")
    runner.move_cartesian((-0.45, -0.4, 0.292), label="drop cube1 on cube2")
    runner.set_gripper(110.0, 5.0, settle_sec=1.0, label="release cube1")
    runner.move_cartesian((-0.45, -0.4, 0.55), label="retreat after cube1")

    # --- Pick cube3 ---
    runner.move_cartesian((-0.65, -0.2, 0.55), label="above cube3")
    runner.move_cartesian((-0.65, -0.2, 0.24), label="grab cube3")
    runner.set_gripper(40.0, 5.0, settle_sec=2.0, label="grasp cube3")

    # --- Place cube3 on top of stack ---
    runner.move_cartesian((-0.65, -0.2, 0.55), label="lift cube3")
    runner.move_cartesian((-0.45, -0.4, 0.55), label="above stack")
    runner.move_cartesian((-0.45, -0.4, 0.342), label="drop cube3 on stack")
    runner.set_gripper(110.0, 5.0, settle_sec=1.0, label="release cube3")
    runner.move_cartesian((-0.45, -0.4, 0.55), label="retreat after cube3")

    # --- Home ---
    runner.move_joints(READY_JOINTS_DEG, label="home")


def main():
    rclpy.init()
    runner = ScenarioRunner()

    t0 = time.time()
    try:
        run_scenario(runner)
    except RuntimeError as exc:
        runner.get_logger().error(f"SCENARIO ABORTED: {exc}")
        rc = 1
    except KeyboardInterrupt:
        runner.get_logger().warn("Interrupted by user")
        rc = 2
    else:
        runner.get_logger().info(f"SCENARIO COMPLETE — total {time.time()-t0:.1f}s")
        rc = 0
    finally:
        runner.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
