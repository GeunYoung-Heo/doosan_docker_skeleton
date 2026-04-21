#!/usr/bin/env python3
"""
Doosan M1013 + OnRobot RG2-FT Isaac Sim bridge.

Loads the combined M1013+RG2-FT URDF as a single articulation.
- Arm (joint_1~6): driven by ROS 2 /joint_states via OmniGraph
- Gripper: driven by Python mimic logic each tick.
  A separate gripper_sim_node (ROS 2) will publish target finger position
  on /gripper_finger_target; this bridge subscribes and applies it.
  (Until gripper_sim_node is running, gripper stays at open position.)

Run:
    /isaac-sim/python.sh /workspace/isaac/m1013_ros2_bridge.py
"""

import argparse
import math
import sys

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--urdf",
        default="/workspace/isaac/m1013_rg2ft_combined.urdf",
        help="Path to the combined M1013+RG2-FT URDF",
    )
    parser.add_argument(
        "--topic",
        default="joint_states",
        help="ROS 2 topic for arm joint states (from MoveIt/mock)",
    )
    parser.add_argument(
        "--gripper-topic",
        default="gripper_finger_target",
        help="ROS 2 topic for gripper finger target (Float64, radians)",
    )
    return parser.parse_args()

ARGS = parse_args()

from isaacsim import SimulationApp  # noqa: E402
simulation_app = SimulationApp({"headless": ARGS.headless, "renderer": "RayTracedLighting"})

import os  # noqa: E402
import carb  # noqa: E402
import omni  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

# ---- Extensions ----
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")

for _ in range(10):
    simulation_app.update()

# ---- Stage ----
omni.usd.get_context().new_stage()
simulation_app.update()

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
set_camera_view(eye=[2.0, 2.0, 1.5], target=[0.0, 0.0, 0.5])
stage = omni.usd.get_context().get_stage()

# ============================================================
# 1. Import combined URDF (single articulation: arm + gripper)
# ============================================================
if not os.path.isfile(ARGS.urdf):
    carb.log_error(f"[bridge] URDF not found: {ARGS.urdf}")
    simulation_app.close()
    sys.exit(1)

print(f"[bridge] importing combined URDF: {ARGS.urdf}", flush=True)
from isaacsim.asset.importer.urdf import _urdf  # noqa: E402

cfg = _urdf.ImportConfig()
cfg.merge_fixed_joints = False
cfg.convex_decomp = False
cfg.import_inertia_tensor = True
cfg.fix_base = True
cfg.distance_scale = 1.0
cfg.density = 0.0
cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
cfg.default_drive_strength = 1e7
cfg.default_position_drive_damping = 1e5
cfg.self_collision = False
cfg.create_physics_scene = False
cfg.make_default_prim = False

status, robot_prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile", urdf_path=ARGS.urdf, import_config=cfg, dest_path="",
)
if not status or not robot_prim_path:
    carb.log_error(f"[bridge] URDF import failed")
    simulation_app.close()
    sys.exit(1)

print(f"[bridge] imported at: {robot_prim_path}", flush=True)
simulation_app.update()

# ============================================================
# 2. Find articulation root for OmniGraph ArticulationController
# ============================================================
# URDF importer places ArticulationRootAPI on the base_link (with fix_base=True).
# Find it by traversal.
robot_path = None
for prim in stage.TraverseAll():
    if prim.GetPath().HasPrefix(robot_prim_path) and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        robot_path = str(prim.GetPath())
        break

if not robot_path:
    # Fallback: use the imported root prim
    robot_path = robot_prim_path

print(f"[bridge] articulation root: {robot_path}", flush=True)

# ============================================================
# 3. Set up gripper joint drives (mimic via Python)
# ============================================================
GRIPPER_JOINTS = {
    "finger_joint":              1.0,
    "left_inner_knuckle_joint": -1.0,
    "left_outer_knuckle_joint": -1.0,
    "left_inner_finger_joint":  -1.0,
    "right_inner_finger_joint": -1.0,
    "right_inner_knuckle_joint": 1.0,
}

def find_joint(name):
    for p in stage.TraverseAll():
        if p.GetName() == name and "Joint" in p.GetTypeName():
            return p
    return None

gripper_joint_prims = {}
for n in GRIPPER_JOINTS:
    p = find_joint(n)
    if p:
        gripper_joint_prims[n] = p
        d = UsdPhysics.DriveAPI.Get(p, "angular")
        if not d:
            d = UsdPhysics.DriveAPI.Apply(p, "angular")
        d.CreateTypeAttr("force")
        d.CreateMaxForceAttr(1e10)
        d.CreateStiffnessAttr(1e7)
        d.CreateDampingAttr(1e4)

print(f"[bridge] gripper joints: {len(gripper_joint_prims)}/{len(GRIPPER_JOINTS)}", flush=True)

# Shared state for gripper target (will be updated by ROS subscriber or gripper_sim_node)
gripper_finger_target_rad = 0.0  # 0 = open, 1.18 = closed

# ============================================================
# 4. Build ROS 2 Action Graph (arm joint_states + clock)
# ============================================================
print("[bridge] building ROS 2 Action Graph", flush=True)

keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("OnTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("Context.outputs:context", "SubscribeJointState.inputs:context"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("Context.outputs:context", "PublishJointState.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
            ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
        ],
        keys.SET_VALUES: [
            ("SubscribeJointState.inputs:topicName", ARGS.topic),
            ("ArticulationController.inputs:robotPath", robot_path),
            ("PublishJointState.inputs:topicName", "isaac_joint_states"),
            ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot_path)]),
        ],
    },
)

print(f"[bridge] OmniGraph ready. arm topic: /{ARGS.topic}", flush=True)

# ============================================================
# 5. Set up ROS 2 subscriber for gripper finger target
# ============================================================
import rclpy  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402

rclpy.init()
ros_node = rclpy.create_node("isaac_gripper_bridge")

def gripper_cb(msg):
    global gripper_finger_target_rad
    gripper_finger_target_rad = max(0.0, min(1.18, msg.data))

ros_node.create_subscription(Float64, ARGS.gripper_topic, gripper_cb, 10)
print(f"[bridge] gripper subscriber: /{ARGS.gripper_topic} (Float64, rad)", flush=True)

# ============================================================
# 6. Main loop
# ============================================================
world.reset()

try:
    omni.timeline.get_timeline_interface().play()
    print("[bridge] timeline: play", flush=True)
except Exception as e:
    print(f"[bridge] timeline play failed: {e}", flush=True)

print("[bridge] entering main loop. Ctrl-C to quit.", flush=True)
try:
    while simulation_app.is_running():
        world.step(render=True)

        # Pump ROS 2 callbacks (gripper target updates)
        rclpy.spin_once(ros_node, timeout_sec=0.0)

        # Apply gripper mimic drives each tick
        for name, ratio in GRIPPER_JOINTS.items():
            p = gripper_joint_prims.get(name)
            if p:
                d = UsdPhysics.DriveAPI.Get(p, "angular")
                if d:
                    d.GetTargetPositionAttr().Set(
                        math.degrees(gripper_finger_target_rad * ratio)
                    )

except KeyboardInterrupt:
    print("[bridge] interrupted", flush=True)
finally:
    try:
        rclpy.shutdown()
    except Exception:
        pass
    simulation_app.close()
