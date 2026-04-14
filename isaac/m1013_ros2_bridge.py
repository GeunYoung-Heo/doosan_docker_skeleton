#!/usr/bin/env python3
"""
Doosan M1013 + Isaac Sim 4.5 + ROS 2 Humble bridge (standalone application).

Pipeline:
    DRCF Emulator (host:12345)
      -> doosan-robot2 dsr_hardware2 ROS 2 driver
      -> publishes sensor_msgs/JointState on /joint_states
      -> this script's Action Graph SubscribeJointState node
      -> ArticulationController applies to imported M1013 prim
      -> Isaac Sim viewport reflects DRCF state.

Run inside the unified container:
    /isaac-sim/python.sh /workspace/isaac/m1013_ros2_bridge.py
or with the host-mounted path:
    /isaac-sim/python.sh /ros2_ws/src/third_party/.../m1013_ros2_bridge.py

Headless variant (no GUI): pass --headless.
"""

import argparse
import sys


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim without GUI")
    parser.add_argument(
        "--usd",
        default="/ros2_ws/src/third_party/doosan-robot2/dsr_description2/usd/m1013.usd",
        help="Path to the official Doosan M1013 USD inside the container",
    )
    parser.add_argument(
        "--topic",
        default="joint_states",
        help="ROS 2 topic to subscribe for joint commands (default: joint_states)",
    )
    parser.add_argument(
        "--robot-prim",
        default="/World/m1013",
        help="Stage path under which the M1013 articulation will be placed",
    )
    return parser.parse_args()


ARGS = parse_args()

# ---- 1. SimulationApp must be the FIRST Isaac Sim import ----
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp(
    {
        "headless": ARGS.headless,
        "renderer": "RayTracedLighting",
    }
)

# Now we can import everything else
import os  # noqa: E402

import carb  # noqa: E402
import omni  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402

# ---- 2. Enable the ROS 2 bridge extension ----
# OmniGraph ros2 nodes are registered when this extension is enabled.
enable_extension("isaacsim.ros2.bridge")

# Pump frames so the extension registers its OmniGraph node types.
for _ in range(10):
    simulation_app.update()

# ---- 3. Fresh stage + ground plane ----
omni.usd.get_context().new_stage()
simulation_app.update()

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
set_camera_view(eye=[2.0, 2.0, 1.5], target=[0.0, 0.0, 0.5])

# ---- 4. Reference the official Doosan M1013 USD ----
if not os.path.isfile(ARGS.usd):
    carb.log_error(f"[bridge] USD not found: {ARGS.usd}")
    simulation_app.close()
    sys.exit(1)

print(f"[bridge] referencing USD: {ARGS.usd}", flush=True)
parent_path = ARGS.robot_prim
add_reference_to_stage(usd_path=ARGS.usd, prim_path=parent_path)
print(f"[bridge] placed reference at: {parent_path}", flush=True)

# Pump a frame so the reference fully composes onto the stage.
simulation_app.update()

# Auto-discover the articulation root inside the referenced USD.
# The USD may put ArticulationRootAPI on a child prim (e.g. /World/m1013/base_0)
# rather than on our reference Xform.
from pxr import UsdPhysics  # noqa: E402

stage = omni.usd.get_context().get_stage()
parent_prim = stage.GetPrimAtPath(parent_path)
if not parent_prim.IsValid():
    carb.log_error(f"[bridge] reference prim invalid: {parent_path}")
    simulation_app.close()
    sys.exit(1)

# Doosan's M1013 USD ships with ArticulationRootAPI applied to a fixed JOINT
# (`root_joint`), not to a rigid body. The IsaacArticulationController node +
# omni.physx.tensors plugin then fail with
#   "Pattern '<path>' did not match any rigid bodies"
# because PhysX wants the articulation root on a body Xform, not a joint.
#
# Fix: move ArticulationRootAPI from the joint onto the first rigid body link
# (`base_link`) so PhysX registers a valid articulation rooted there.
stage = omni.usd.get_context().get_stage()
joint_root = stage.GetPrimAtPath(f"{parent_path}/m1013/root_joint")
link_root = stage.GetPrimAtPath(f"{parent_path}/m1013/base_link")

if joint_root.IsValid() and joint_root.HasAPI(UsdPhysics.ArticulationRootAPI):
    joint_root.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    print(f"[bridge] removed ArticulationRootAPI from {joint_root.GetPath()}", flush=True)

if link_root.IsValid():
    if not link_root.HasAPI(UsdPhysics.ArticulationRootAPI):
        UsdPhysics.ArticulationRootAPI.Apply(link_root)
        print(f"[bridge] applied ArticulationRootAPI to {link_root.GetPath()}", flush=True)
    robot_path = str(link_root.GetPath())
else:
    carb.log_error(f"[bridge] base_link not found under {parent_path}/m1013")
    simulation_app.close()
    sys.exit(1)

simulation_app.update()
print(f"[bridge] articulation robotPath: {robot_path}", flush=True)

# ---- 5. Build the ROS 2 Action Graph (subscribe + republish + clock) ----
print("[bridge] building ROS 2 Action Graph", flush=True)

graph_path = "/ActionGraph"
keys = og.Controller.Keys

og.Controller.edit(
    {"graph_path": graph_path, "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            # Echo joint state back so other tools (rqt, rviz on host) see Isaac state too.
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

            # Drive the articulation from the subscriber outputs.
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
            ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
        ],
        keys.SET_VALUES: [
            ("SubscribeJointState.inputs:topicName", ARGS.topic),
            ("ArticulationController.inputs:robotPath", robot_path),
            # PublishJointState echoes state on a distinct topic to avoid loops.
            ("PublishJointState.inputs:topicName", "isaac_joint_states"),
            ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot_path)]),
        ],
    },
)

print(f"[bridge] graph ready. Subscribing to '/{ARGS.topic}', echoing on '/isaac_joint_states'", flush=True)

# ---- 6. Reset world + main loop ----
world.reset()

print("[bridge] entering main loop. Ctrl-C to quit.", flush=True)
try:
    while simulation_app.is_running():
        world.step(render=True)
except KeyboardInterrupt:
    print("[bridge] interrupted", flush=True)
finally:
    simulation_app.close()
