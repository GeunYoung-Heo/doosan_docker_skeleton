#!/usr/bin/env python3
"""Multi pick-and-place scenario for pill collection.

4 pills from a row at x=-0.65, y∈{0.1, 0.0, -0.1, -0.2} are picked one by one
and placed at a common drop target (x=-0.75, y=0.0, z=0.30).

Approach height: z=0.33
Pick descent:    z=0.23
Drop descent:    z=0.30
Grasp:           width=0.0,  force=5.0 N
Release:         width=60.0, force=5.0 N

Usage:
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    python3 /ros2_ws/src/scenarios/multi_pick_place.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from cube_stacking import ScenarioRunner, READY_JOINTS_DEG  # noqa: E402

import rclpy  # noqa: E402


# Pick locations (x, y) — same z handling for all
PICK_X = -0.65
PICK_Y_LIST = [0.1, 0.0, -0.1, -0.2]
APPROACH_Z = 0.33
PICK_Z = 0.23

# Common drop target
DROP_XYZ = (-0.75, 0.0, 0.30)
DROP_APPROACH_XYZ = (-0.75, 0.0, 0.33)


def run_scenario(runner: ScenarioRunner):
    # --- Start: ready pose + open gripper ---
    runner.move_joints(READY_JOINTS_DEG, label="ready")
    runner.set_gripper(110.0, 5.0, settle_sec=1.0, label="open")

    for i, y in enumerate(PICK_Y_LIST, start=1):
        tag = f"pill{i}"
        approach_xyz = (PICK_X, y, APPROACH_Z)
        pick_xyz     = (PICK_X, y, PICK_Z)

        # --- Pick ---
        runner.move_cartesian(approach_xyz, label=f"above {tag}")
        runner.move_cartesian(pick_xyz,     label=f"descend {tag}")
        runner.set_gripper(0.0, 5.0, settle_sec=2.0, label=f"grasp {tag}")
        runner.move_cartesian(approach_xyz, label=f"lift {tag}")

        # --- Place at common drop ---
        runner.move_cartesian(DROP_APPROACH_XYZ, label=f"above drop ({tag})")
        runner.move_cartesian(DROP_XYZ,          label=f"descend drop ({tag})")
        runner.set_gripper(60.0, 5.0, settle_sec=1.0, label=f"release {tag}")
        runner.move_cartesian(DROP_APPROACH_XYZ, label=f"retreat drop ({tag})")

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
        runner.get_logger().info(
            f"SCENARIO COMPLETE — total {time.time()-t0:.1f}s"
        )
        rc = 0
    finally:
        runner.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
