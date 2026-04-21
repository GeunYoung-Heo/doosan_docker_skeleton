#!/usr/bin/env python3
"""OnRobot RG2-FT gripper standalone test (PROVEN WORKING).

Drives finger_joint as primary, sets mimic joint targets via Python
each tick using the multiplier ratios from the original URDF.

This is the same approach that was validated earlier — restoring it
to re-confirm before integrating with M1013.

Run:
    /isaac-sim/python.sh /workspace/isaac/gripper_standalone_test.py
"""

import argparse, math, sys, time

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--headless", action="store_true")
    p.add_argument("--urdf", default="/tmp/onrobot_rg2ft_no_mimic.urdf")
    return p.parse_args()

ARGS = parse_args()

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": ARGS.headless, "renderer": "RayTracedLighting"})

import carb, omni, omni.kit.commands, omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.viewports import set_camera_view
from pxr import UsdPhysics, Gf

enable_extension("isaacsim.asset.importer.urdf")
for _ in range(10):
    simulation_app.update()

omni.usd.get_context().new_stage()
simulation_app.update()

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
set_camera_view(eye=[0.3, 0.3, 0.3], target=[0.0, 0.0, 0.1])

stage = omni.usd.get_context().get_stage()

# ---- Import URDF (no mimic tags, fix_base=True for stability) ----
print(f"[gripper] importing: {ARGS.urdf}", flush=True)
from isaacsim.asset.importer.urdf import _urdf

cfg = _urdf.ImportConfig()
cfg.merge_fixed_joints = False
cfg.convex_decomp = False
cfg.import_inertia_tensor = True
cfg.fix_base = True            # ← anchored to world = maximally stable
cfg.distance_scale = 1.0
cfg.density = 0.0
cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
cfg.default_drive_strength = 1e7
cfg.default_position_drive_damping = 1e4
cfg.self_collision = False
cfg.create_physics_scene = False
cfg.make_default_prim = False

status, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile", urdf_path=ARGS.urdf, import_config=cfg, dest_path="",
)
assert status and prim_path, "Import failed"
print(f"[gripper] at: {prim_path}", flush=True)
simulation_app.update()

# ---- Joint definitions (mimic ratios from original URDF) ----
JOINTS = {
    "finger_joint":              1.0,   # PRIMARY
    "left_inner_knuckle_joint": -1.0,   # mimic
    "left_outer_knuckle_joint": -1.0,   # mimic
    "left_inner_finger_joint":  -1.0,   # mimic
    "right_inner_finger_joint": -1.0,   # mimic
    "right_inner_knuckle_joint": 1.0,   # mimic
}

def find_joint(name):
    for p in stage.TraverseAll():
        if p.GetName() == name and "Joint" in p.GetTypeName():
            return p
    return None

joint_prims = {}
for n in JOINTS:
    p = find_joint(n)
    if p:
        joint_prims[n] = p
        # Ensure strong drive on ALL joints
        d = UsdPhysics.DriveAPI.Get(p, "angular")
        if not d:
            d = UsdPhysics.DriveAPI.Apply(p, "angular")
        d.CreateTypeAttr("force")
        d.CreateMaxForceAttr(1e10)
        d.CreateStiffnessAttr(1e7)
        d.CreateDampingAttr(1e4)

print(f"[gripper] joints: {len(joint_prims)}/{len(JOINTS)}", flush=True)

# ---- Main loop ----
world.reset()
try:
    omni.timeline.get_timeline_interface().play()
except Exception:
    pass

CLOSE_RAD = 1.18
PERIOD = 4.0

print("[gripper] cycling. Ctrl-C to quit.", flush=True)

t0 = time.time()
last_log = 0
try:
    while simulation_app.is_running():
        world.step(render=True)
        t = time.time() - t0
        phase = (math.sin(2 * math.pi * t / PERIOD - math.pi / 2) + 1) / 2
        finger_rad = phase * CLOSE_RAD

        # Drive ALL joints: target = finger_rad × ratio
        for name, ratio in JOINTS.items():
            p = joint_prims.get(name)
            if p:
                d = UsdPhysics.DriveAPI.Get(p, "angular")
                if d:
                    d.GetTargetPositionAttr().Set(math.degrees(finger_rad * ratio))

        if int(t) > last_log and int(t) % 2 == 0:
            last_log = int(t)
            print(f"  t={t:.0f}s finger={math.degrees(finger_rad):.1f}°", flush=True)

except KeyboardInterrupt:
    print("[gripper] done", flush=True)
finally:
    simulation_app.close()
