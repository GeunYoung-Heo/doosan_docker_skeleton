#!/usr/bin/env python3
"""
Doosan M1013 + OnRobot RG2-FT Isaac Sim bridge.

Decoupled-tick architecture:

  - Physics + arm control run at a fixed 60 Hz in this Python loop,
    regardless of render load. Arm joint targets come from /joint_states
    (MoveIt mock_components) via an rclpy subscription and are applied
    directly to the articulation each physics tick — the old OmniGraph
    SubscribeJointState → ArticulationController chain has been removed
    because it was tied to render-rate OnPlaybackTick ticks, which meant
    recording (which slows render) also starved the control loop and
    caused severe end-effector vibration.

  - Rendering (and the replicator frame capture that feeds ffmpeg) runs
    at a lower sub-multiple of the physics rate. Rendering can take tens
    of ms during recording without disturbing the control loop; the
    physics loop simply keeps stepping while the occasional render tick
    burns its own budget.

Run:
    /isaac-sim/python.sh /workspace/isaac/m1013_ros2_bridge.py
"""

import argparse
import math
import sys
import time

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
    parser.add_argument(
        "--spawn-cubes", action="store_true",
        help="Spawn three 5cm cubes for the cube-stacking scenario",
    )
    parser.add_argument(
        "--scene-usd",
        default=None,
        help="Load this USD stage instead of importing URDF + spawning cubes. "
             "The USD must already contain the robot and all scene objects "
             "(e.g. produced by the doosan-cap SceneBuilderAPI).",
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
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

# ---- Extensions ----
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")

for _ in range(10):
    simulation_app.update()

# ---- Physics / render rates ----
# PhysX physics dt is locked to mock_components / ros2_control's update rate
# (100 Hz). Running them at different rates (the default 60 Hz physics vs
# 100 Hz publish) causes a sampling alias: each 16.67 ms physics tick
# alternates between consuming 1 or 2 published commands, so the joint
# drive sees a non-uniform target velocity and produces low-frequency
# vibration even on smooth interpolated trajectories.
PHYSICS_HZ        = 100
PHYSICS_DT        = 1.0 / PHYSICS_HZ
# Render every N physics ticks. With physics at 100 Hz, N=3 gives ~33 Hz
# rendering — fast enough for smooth viewport playback but cheap enough
# during recording.
RENDER_EVERY_N    = int(os.environ.get("BRIDGE_RENDER_EVERY_N", "3"))
RENDERING_DT      = PHYSICS_DT * RENDER_EVERY_N

# ---- Stage ----
if ARGS.scene_usd is not None:
    if not os.path.isfile(ARGS.scene_usd):
        carb.log_error(f"[bridge] scene USD not found: {ARGS.scene_usd}")
        simulation_app.close()
        sys.exit(1)
    print(f"[bridge] opening scene USD: {ARGS.scene_usd}", flush=True)
    omni.usd.get_context().open_stage(ARGS.scene_usd)
    simulation_app.update()
    world = World(stage_units_in_meters=1.0,
                  physics_dt=PHYSICS_DT, rendering_dt=RENDERING_DT)
else:
    omni.usd.get_context().new_stage()
    simulation_app.update()
    world = World(stage_units_in_meters=1.0,
                  physics_dt=PHYSICS_DT, rendering_dt=RENDERING_DT)
    world.scene.add_default_ground_plane()

set_camera_view(eye=[2.0, 2.0, 1.5], target=[0.0, 0.0, 0.5])
stage = omni.usd.get_context().get_stage()

# ============================================================
# 1. Either use the prim already in the opened USD, or import URDF.
# ============================================================
if ARGS.scene_usd is not None:
    # Scene USD already contains the robot (from SceneBuilderAPI.add_doosan).
    # Physics config (armature, implicitSpringDamper, finger friction) was
    # applied during scene generation and persists in the USD.
    # Find the robot prim: look for an articulation root that's /World/*.
    robot_prim_path = None
    for prim in stage.TraverseAll():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            path_str = str(prim.GetPath())
            if path_str.startswith("/World") or path_str.startswith("/m1013"):
                robot_prim_path = path_str
                break
    if robot_prim_path is None:
        carb.log_error("[bridge] no articulation root found in scene USD")
        simulation_app.close()
        sys.exit(1)
    print(f"[bridge] robot prim in scene USD: {robot_prim_path}", flush=True)
else:
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
# 2. Find articulation root
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
# 2.5. Enable implicit spring damper on all joints (suppress vibration)
# ============================================================
from pxr import Sdf as _Sdf, PhysxSchema  # noqa: E402

_joint_count = 0
for prim in stage.TraverseAll():
    if "Joint" in prim.GetTypeName() and "Revolute" in prim.GetTypeName():
        # Implicit spring damper
        attr = prim.GetAttribute("physxJoint:implicitSpringDamper")
        if not attr:
            attr = prim.CreateAttribute("physxJoint:implicitSpringDamper", _Sdf.ValueTypeNames.Bool)
        attr.Set(True)
        # Joint armature
        physx_joint = PhysxSchema.PhysxJointAPI.Apply(prim)
        physx_joint.CreateArmatureAttr(0.1)
        _joint_count += 1

print(f"[bridge] implicitSpringDamper + armature(0.1) on {_joint_count} revolute joints", flush=True)

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

# Cap the angular velocity of every gripper joint (deg/s).
# 25 deg/s lets the gripper complete a full 0<->110 mm travel in ~3 s,
# so the API's default 2 s settle_sec covers closing-on-an-object distances.
# Contact with an object still halts the fingers immediately via the physics
# contact solver, so grasp force is unaffected by this cap.
GRIPPER_MAX_JOINT_VEL_DEG = 25.0

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
        d.CreateDampingAttr(1e5)
        # Slow the gripper's open/close animation to a natural speed.
        pj = PhysxSchema.PhysxJointAPI.Apply(p)
        pj.CreateMaxJointVelocityAttr(GRIPPER_MAX_JOINT_VEL_DEG)

print(f"[bridge] gripper joints: {len(gripper_joint_prims)}/{len(GRIPPER_JOINTS)} "
      f"(maxVel={GRIPPER_MAX_JOINT_VEL_DEG:.0f} deg/s)", flush=True)

# Apply high-friction physics material to gripper finger collision prims
from pxr import UsdGeom, Gf, UsdShade, Sdf  # noqa: E402

_grip_mtl = UsdShade.Material.Define(stage, "/World/high_friction_material")
_grip_phys = UsdPhysics.MaterialAPI.Apply(_grip_mtl.GetPrim())
_grip_phys.CreateStaticFrictionAttr(2.0)
_grip_phys.CreateDynamicFrictionAttr(2.0)
_grip_phys.CreateRestitutionAttr(0.0)

FINGER_COL_PATHS = [
    "/m1013/right_inner_finger/collisions",
    "/m1013/left_inner_finger/collisions",
]
for fp in FINGER_COL_PATHS:
    p = stage.GetPrimAtPath(fp)
    if p.IsValid():
        p.CreateRelationship("material:binding:physics").SetTargets(
            [_grip_mtl.GetPath()]
        )
        print(f"  [friction] bound to {fp}", flush=True)
print("[bridge] finger friction material applied", flush=True)

# Shared state for gripper target (updated by ROS subscriber)
gripper_finger_target_rad = 0.0  # 0 = open, 1.18 = closed


# ============================================================
# 3.5. (Optional) Spawn cubes for pick-and-place scenario
# ============================================================
if ARGS.spawn_cubes and ARGS.scene_usd is None:
    from pxr import UsdGeom, Gf, UsdShade, Sdf  # noqa: E402

    CUBE_SIZE = 0.05  # 5 cm
    CUBE_POSITIONS = {
        "cube1": Gf.Vec3d(-0.45,  0.0, CUBE_SIZE / 2),
        "cube2": Gf.Vec3d(-0.45, -0.4, CUBE_SIZE / 2),
        "cube3": Gf.Vec3d(-0.65, -0.2, CUBE_SIZE / 2),
    }
    CUBE_COLORS = {
        "cube1": Gf.Vec3f(0.9, 0.2, 0.2),  # red
        "cube2": Gf.Vec3f(0.2, 0.7, 0.2),  # green
        "cube3": Gf.Vec3f(0.2, 0.3, 0.9),  # blue
    }

    for name, pos in CUBE_POSITIONS.items():
        cube_path = f"/World/{name}"
        cube_prim = UsdGeom.Cube.Define(stage, cube_path)
        cube_prim.CreateSizeAttr(CUBE_SIZE)
        cube_prim.AddTranslateOp().Set(pos)

        UsdPhysics.RigidBodyAPI.Apply(cube_prim.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube_prim.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(cube_prim.GetPrim())
        mass_api.CreateMassAttr(0.1)  # 100g

        # Bind high-friction physics material
        cube_prim.GetPrim().CreateRelationship("material:binding:physics").SetTargets(
            [_grip_mtl.GetPath()]
        )

        # Visual material (color)
        mtl_path = f"/World/{name}_material"
        mtl = UsdShade.Material.Define(stage, mtl_path)
        shader = UsdShade.Shader.Define(stage, f"{mtl_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(CUBE_COLORS[name])
        mtl.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(cube_prim.GetPrim()).Bind(mtl)

    simulation_app.update()
    print(f"[bridge] spawned 3 cubes (5cm, 100g): {list(CUBE_POSITIONS.keys())}", flush=True)

# ============================================================
# 3.75. Register the robot as a scene object so world.reset() initializes it
# ============================================================
# SingleArticulation gives us a Python handle to the PhysX articulation so we
# can push joint-position targets directly from the main loop (bypassing the
# OmniGraph ArticulationController, which is tied to render-rate ticks).
articulation = SingleArticulation(prim_path=robot_path, name="m1013_full")
world.scene.add(articulation)

# ============================================================
# 4. Build ROS 2 Action Graph (clock + sim-state publish only)
# ============================================================
# We deliberately DO NOT put SubscribeJointState or ArticulationController on
# the graph any more. /joint_states is consumed directly in the rclpy node
# below and applied to the articulation each physics tick, which decouples
# arm control from the render tick (see file-level docstring).
print("[bridge] building ROS 2 Action Graph (clock + publish only)", flush=True)

keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("Context.outputs:context", "PublishJointState.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
        ],
        keys.SET_VALUES: [
            ("PublishJointState.inputs:topicName", "isaac_joint_states"),
            ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot_path)]),
        ],
    },
)

print("[bridge] OmniGraph ready (no arm control on graph)", flush=True)

# ============================================================
# 5. ROS 2 nodes: subscribers (background thread) + services (main thread)
# ============================================================
# We run subscribers on a dedicated background-thread executor so that
# /joint_states callbacks fire IMMEDIATELY when a message arrives, not
# whenever the main loop happens to call spin_once. With main-thread
# spin_once at 60 Hz vs mock_components publishing at ~100 Hz, the
# average latency between command publication and apply_action was
# ~8 ms with non-uniform jitter — enough for PhysX's stiff position
# drive to see step-jump targets and produce low-frequency vibration.
# Putting subscribers on their own thread brings that latency down to
# the rclpy callback dispatch overhead (sub-ms).
#
# Services stay on the main-thread executor because /doosan_bridge/
# get_object_poses traverses the USD stage, which is not thread-safe.
import threading  # noqa: E402

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402

rclpy.init()
ros_node = rclpy.create_node("isaac_gripper_bridge")        # services
ros_sub_node = rclpy.create_node("isaac_bridge_subs")        # subscribers

def gripper_cb(msg):
    global gripper_finger_target_rad
    gripper_finger_target_rad = max(0.0, min(1.18, msg.data))

ros_sub_node.create_subscription(Float64, ARGS.gripper_topic, gripper_cb, 10)
print(f"[bridge] gripper subscriber: /{ARGS.gripper_topic} (Float64, rad)", flush=True)

# Arm command subscriber — replaces OmniGraph SubscribeJointState. Cache
# the latest commanded arm joint positions for the main loop to apply.
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
_latest_arm_cmd = {"positions": None}  # list[float] in ARM_JOINT_NAMES order

_arm_cb_count = 0
_arm_cb_count_lock = threading.Lock()

def _arm_cmd_cb(msg: JointState) -> None:
    global _arm_cb_count
    # msg.name/msg.position may contain fingers too. Index arm joints by name.
    try:
        idx = [msg.name.index(n) for n in ARM_JOINT_NAMES]
    except ValueError:
        return  # arm joints not yet present in message
    # Single ref-assignment of a fresh list is GIL-atomic — safe to share
    # with the main thread without an explicit lock.
    _latest_arm_cmd["positions"] = [float(msg.position[i]) for i in idx]
    _latest_arm_cmd["t_recv"] = time.perf_counter()
    with _arm_cb_count_lock:
        _arm_cb_count += 1

# depth=1 keeps only the most recent message — combined with the dedicated
# spinner thread this gives us "always latest" semantics with negligible
# latency.
_arm_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)
ros_sub_node.create_subscription(JointState, ARGS.topic, _arm_cmd_cb, _arm_qos)
print(f"[bridge] arm-command subscriber: /{ARGS.topic} (JointState)", flush=True)

# Spin subscribers in a background daemon thread. The main loop touches
# the cached state but never calls into rclpy on this node.
_sub_executor = SingleThreadedExecutor()
_sub_executor.add_node(ros_sub_node)
_sub_thread = threading.Thread(
    target=_sub_executor.spin, name="ros-sub-spinner", daemon=True
)
_sub_thread.start()
print("[bridge] subscriber spinner thread started", flush=True)

# ============================================================
# 5.5. Services for recording + object pose queries
# ============================================================
# These services let an external control script (doosan-cap/run_control.py)
# ask the bridge to record the front/overhead cameras during a task and to
# read back the current pose of every non-robot rigid body in the scene.

try:
    from doosan_bridge_msgs.srv import (  # noqa: E402
        StartRecording, StopRecording, GetObjectPoses,
    )
    from geometry_msgs.msg import Pose as _Pose  # noqa: E402
    _HAS_DOOSAN_BRIDGE_MSGS = True
except ImportError as _e:
    print(f"[bridge] doosan_bridge_msgs not found ({_e}); "
          f"recording & object pose services disabled", flush=True)
    _HAS_DOOSAN_BRIDGE_MSGS = False

# Shared recording state. Service callbacks only mark intent; the actual
# replicator/render-product setup happens on the main loop thread because some
# kit APIs are main-thread-only.
_rec_state = {
    "request": None,        # dict: {"output_dir": str, "fps": int} pending start
    "stop_request": False,
    "active": False,
    "cameras": [],          # list of (annotator, writer_process, mp4_path)
    "ffmpeg_fps": 30,
}


def _start_recording_cb(req, resp):
    if _rec_state["active"]:
        resp.success = False
        resp.message = "recording already active"
        return resp
    if not req.output_dir:
        resp.success = False
        resp.message = "output_dir must be set"
        return resp
    _rec_state["request"] = {
        "output_dir": req.output_dir,
        "fps": int(req.fps) if req.fps > 0 else 30,
    }
    resp.success = True
    resp.message = f"queued start at {req.output_dir}"
    return resp


def _stop_recording_cb(_req, resp):
    if not _rec_state["active"] and _rec_state["request"] is None:
        resp.success = False
        resp.message = "no recording active"
        return resp
    _rec_state["stop_request"] = True
    resp.success = True
    resp.message = "queued stop"
    return resp


def _get_object_poses_cb(req, resp):
    names_filter = set(req.names) if req.names else None
    returned_names = []
    poses = []

    # Read robot base world pose to convert to base frame.
    base_prim_path = None
    for prim in stage.TraverseAll():
        if prim.GetName() == "base_link" and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            base_prim_path = str(prim.GetPath())
            break

    if base_prim_path is not None:
        base_xf = UsdGeom.Xformable(
            stage.GetPrimAtPath(base_prim_path)
        ).ComputeLocalToWorldTransform(0)
        base_pos = base_xf.ExtractTranslation()
    else:
        base_pos = Gf.Vec3d(0.0, 0.0, 0.0)

    for prim in stage.TraverseAll():
        path = str(prim.GetPath())
        # Skip robot body prims and ground plane
        if "m1013" in path.lower() or "doosan" in path.lower():
            continue
        if "groundplane" in path.lower() or "ground_plane" in path.lower():
            continue
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        # Keep only scene-level prims like /World/<Name>
        name = prim.GetName()
        if names_filter is not None and name not in names_filter and path not in names_filter:
            continue

        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        pos = xf.ExtractTranslation()
        q = xf.ExtractRotationQuat()
        img = q.GetImaginary()

        p = _Pose()
        p.position.x = float(pos[0] - base_pos[0])
        p.position.y = float(pos[1] - base_pos[1])
        p.position.z = float(pos[2] - base_pos[2])
        p.orientation.w = float(q.GetReal())
        p.orientation.x = float(img[0])
        p.orientation.y = float(img[1])
        p.orientation.z = float(img[2])

        returned_names.append(name)
        poses.append(p)

    resp.success = True
    resp.message = f"returned {len(poses)} poses"
    resp.returned_names = returned_names
    resp.poses = poses
    return resp


if _HAS_DOOSAN_BRIDGE_MSGS:
    ros_node.create_service(
        StartRecording, "/doosan_bridge/start_recording", _start_recording_cb
    )
    ros_node.create_service(
        StopRecording, "/doosan_bridge/stop_recording", _stop_recording_cb
    )
    ros_node.create_service(
        GetObjectPoses, "/doosan_bridge/get_object_poses", _get_object_poses_cb
    )
    print("[bridge] services: start_recording, stop_recording, get_object_poses",
          flush=True)


# ============================================================
# 5.75. Recording helpers (no world.step monkey-patching)
# ============================================================
# With the decoupled-tick loop below, the capture is driven by the main loop
# on render ticks only. The replicator render product / annotator setup is
# created once when recording starts and torn down when it stops.

REC_WIDTH  = 1280
REC_HEIGHT = 720
REC_FPS    = 30


def _activate_recording(output_dir: str, fps: int) -> None:
    import os as _os
    import subprocess as _sp
    try:
        import omni.replicator.core as rep  # noqa: E402
    except Exception as e:
        print(f"[bridge] replicator import failed: {e}", flush=True)
        return

    # fps from the service request is only advisory; ffmpeg fps is fixed at
    # REC_FPS so video playback timing matches capture cadence.
    _ = fps
    fps = REC_FPS

    _os.makedirs(output_dir, exist_ok=True)
    cams = []
    for cam_name, cam_prim in [
        ("front",    "/World/Camera_Front"),
        ("overhead", "/World/Camera_Overhead"),
    ]:
        if not stage.GetPrimAtPath(cam_prim).IsValid():
            print(f"[bridge] camera not found, skip: {cam_prim}", flush=True)
            continue
        rp = rep.create.render_product(cam_prim, (REC_WIDTH, REC_HEIGHT))
        annot = rep.AnnotatorRegistry.get_annotator("rgb")
        annot.attach([rp])
        mp4_path = _os.path.join(output_dir, f"recording_{cam_name}.mp4")
        writer = _sp.Popen(
            ["ffmpeg", "-y",
             "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", f"{REC_WIDTH}x{REC_HEIGHT}", "-r", str(fps),
             "-i", "-",
             "-c:v", "libx264", "-preset", "ultrafast",
             "-pix_fmt", "yuv420p", mp4_path],
            stdin=_sp.PIPE, stdout=_sp.DEVNULL, stderr=_sp.DEVNULL,
        )
        cams.append((annot, writer, mp4_path))

    if not cams:
        print("[bridge] no cameras available, recording aborted", flush=True)
        return

    _rec_state["cameras"] = cams
    _rec_state["ffmpeg_fps"] = fps
    _rec_state["active"] = True
    print(f"[bridge] recording: {REC_WIDTH}x{REC_HEIGHT} @ {fps} fps", flush=True)
    for _, _, path in cams:
        print(f"[bridge] recording -> {path}", flush=True)


def _capture_recording_frames() -> None:
    """Read one frame from each camera's annotator and push it to ffmpeg stdin.
    Call this AFTER a Kit render update (world.render()) so the annotator has
    fresh data."""
    if not _rec_state["active"]:
        return
    import numpy as _np
    for annot, writer, _ in _rec_state["cameras"]:
        try:
            data = annot.get_data()
        except Exception:
            continue
        if data is None:
            continue
        if isinstance(data, dict):
            data = data["data"]
        try:
            rgb = _np.ascontiguousarray(data[:, :, :3])
            writer.stdin.write(rgb.tobytes())
        except (BrokenPipeError, OSError):
            pass
        except Exception:
            pass


def _deactivate_recording() -> None:
    if not _rec_state["active"]:
        return
    for _, writer, mp4_path in _rec_state["cameras"]:
        try:
            writer.stdin.close()
        except Exception:
            pass
        try:
            writer.wait(timeout=10)
        except Exception:
            writer.kill()
        print(f"[bridge] recording finalised: {mp4_path}", flush=True)
    _rec_state["cameras"] = []
    _rec_state["active"] = False


# ============================================================
# 6. Initialise physics, grab articulation controller, start timeline
# ============================================================
world.reset()  # initialises PhysX view and the registered articulation

# The scene object has been initialised by world.reset(); pull the controller
# and resolve arm DOF indices now.
articulation.initialize()
art_ctrl = articulation.get_articulation_controller()

try:
    arm_dof_indices = [articulation.get_dof_index(n) for n in ARM_JOINT_NAMES]
except Exception as e:
    carb.log_error(f"[bridge] failed to resolve arm DOF indices: {e}")
    simulation_app.close()
    sys.exit(1)

print(f"[bridge] arm DOF indices {ARM_JOINT_NAMES} -> {arm_dof_indices} "
      f"(total DOF: {articulation.num_dof})", flush=True)

try:
    omni.timeline.get_timeline_interface().play()
    print("[bridge] timeline: play", flush=True)
except Exception as e:
    print(f"[bridge] timeline play failed: {e}", flush=True)


# ============================================================
# 7. Main loop — physics tick at 60 Hz, render at a sub-multiple
# ============================================================
import numpy as _np  # noqa: E402

# PHYSICS_HZ / PHYSICS_DT / RENDER_EVERY_N / RENDERING_DT are defined near
# the top of the file (before World() construction). The values are reused
# here for the loop's real-time throttle and diagnostics.

# Diagnostic: when BRIDGE_DEBUG_NO_APPLY=1, the main loop skips apply_action
# entirely. The arm therefore holds whatever PhysX initialised it to (the
# scene/URDF default pose). If the EE STILL visibly vibrates in this mode,
# the cause is purely physics-side (drive stiffness/damping/armature) and
# unrelated to the /joint_states command path.
DEBUG_NO_APPLY    = os.environ.get("BRIDGE_DEBUG_NO_APPLY", "") == "1"

# Diagnostic stats period (sec). Set to 0 to disable per-second logging.
DEBUG_STATS_PERIOD = float(os.environ.get("BRIDGE_DEBUG_STATS_PERIOD", "1.0"))

print(f"[bridge] main loop: physics={PHYSICS_HZ} Hz "
      f"(dt={PHYSICS_DT*1000:.2f}ms), "
      f"render every {RENDER_EVERY_N} physics ticks "
      f"({1.0/RENDERING_DT:.1f} Hz) "
      f"| debug_no_apply={DEBUG_NO_APPLY} stats_period={DEBUG_STATS_PERIOD}s",
      flush=True)

step_counter = 0
next_t = time.perf_counter()
_loop_period_samples = []
_loop_stats_t_last = time.perf_counter()

try:
    while simulation_app.is_running():
        loop_start_t = time.perf_counter()

        # ── 1. Service callbacks (subscribers run on their own thread) ──
        # Subscribers (gripper target, /joint_states) tick in the background
        # daemon thread. We only need to drain main-thread service callbacks
        # here (start/stop recording, get_object_poses).
        rclpy.spin_once(ros_node, timeout_sec=0.0)

        # ── 2. Apply arm joint-position targets to the articulation ──
        cmd = _latest_arm_cmd["positions"]
        if cmd is not None and not DEBUG_NO_APPLY:
            art_ctrl.apply_action(
                ArticulationAction(
                    joint_positions=_np.asarray(cmd, dtype=_np.float32),
                    joint_indices=_np.asarray(arm_dof_indices, dtype=_np.int32),
                )
            )

        # ── 3. Apply gripper mimic drive targets (USD joint drive) ───
        for name, ratio in GRIPPER_JOINTS.items():
            p = gripper_joint_prims.get(name)
            if p:
                d = UsdPhysics.DriveAPI.Get(p, "angular")
                if d:
                    d.GetTargetPositionAttr().Set(
                        math.degrees(gripper_finger_target_rad * ratio)
                    )

        # ── 4. Honour queued recording start/stop requests ───────────
        if _rec_state["request"] is not None and not _rec_state["active"]:
            req = _rec_state["request"]
            _rec_state["request"] = None
            try:
                _activate_recording(req["output_dir"], req["fps"])
            except Exception as _e:
                import traceback as _tb
                print(f"[bridge] _activate_recording failed: {_e}", flush=True)
                _tb.print_exc()
                _rec_state["active"] = False
                _rec_state["cameras"] = []
        if _rec_state["stop_request"]:
            _rec_state["stop_request"] = False
            try:
                _deactivate_recording()
            except Exception as _e:
                import traceback as _tb
                print(f"[bridge] _deactivate_recording failed: {_e}", flush=True)
                _tb.print_exc()
                _rec_state["active"] = False
                _rec_state["cameras"] = []

        # ── 5. Advance physics by one tick (no render) ───────────────
        world.step(render=False)

        # ── 6. Occasionally render + capture + OmniGraph publish ─────
        step_counter += 1
        if step_counter % RENDER_EVERY_N == 0:
            # world.render() triggers Kit update with playSimulations=False, so
            # rendering + OmniGraph ticks (PublishClock, PublishJointState)
            # happen but physics does NOT double-step.
            world.render()
            if _rec_state["active"]:
                _capture_recording_frames()

        # ── 7. Measure loop body duration (excludes sleep below) ─────
        body_dur = time.perf_counter() - loop_start_t
        if DEBUG_STATS_PERIOD > 0:
            _loop_period_samples.append(body_dur)

        # ── 8. Real-time throttle to PHYSICS_HZ ──────────────────────
        next_t += PHYSICS_DT
        sleep_s = next_t - time.perf_counter()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            # Overrun: catch-up would just chain-step and risk unbounded
            # drift; reset the clock so we stay wall-time-aligned.
            next_t = time.perf_counter()

        # ── 9. Diagnostic stats (once per DEBUG_STATS_PERIOD seconds) ──
        if DEBUG_STATS_PERIOD > 0:
            now = time.perf_counter()
            if now - _loop_stats_t_last >= DEBUG_STATS_PERIOD:
                with _arm_cb_count_lock:
                    cb_count = _arm_cb_count
                    _arm_cb_count = 0
                n = len(_loop_period_samples)
                if n > 0:
                    body_avg = sum(_loop_period_samples) / n
                    body_min = min(_loop_period_samples)
                    body_max = max(_loop_period_samples)
                    actual_hz = n / (now - _loop_stats_t_last)
                    t_recv = _latest_arm_cmd.get("t_recv", 0.0)
                    cmd_age_ms = ((now - t_recv) * 1000.0
                                  if t_recv > 0 else float("nan"))
                    print(
                        f"[stats] body avg={body_avg*1000:.2f}ms "
                        f"min={body_min*1000:.2f}ms max={body_max*1000:.2f}ms "
                        f"| loop_hz={actual_hz:.1f} "
                        f"| /joint_states cb/s={cb_count/(now-_loop_stats_t_last):.1f} "
                        f"| cmd_age={cmd_age_ms:.1f}ms "
                        f"| no_apply={DEBUG_NO_APPLY}",
                        flush=True,
                    )
                _loop_period_samples = []
                _loop_stats_t_last = now

except KeyboardInterrupt:
    print("[bridge] interrupted", flush=True)
finally:
    try:
        if _rec_state["active"]:
            _deactivate_recording()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass
    simulation_app.close()
