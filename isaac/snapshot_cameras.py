#!/usr/bin/env python3
"""
Snapshot cameras in a scene USD.

Renders one image per Camera prim in the loaded stage and saves them to
the output directory. The headless Isaac Sim runs only long enough to
warm up the renderer and grab one annotated frame per camera, then exits.

Useful for iterating on camera placement before committing to the values
in `scene_builder_api.py::_setup_cameras()`. Use the `--override` flag
to try a candidate pose without regenerating the scene USD.

The bridge (`m1013_ros2_bridge.py`) must NOT be running while this is
invoked — Isaac Sim is single-instance per GPU.

Usage:

  # Snapshot the cameras AS-IS in the scene USD
  /isaac-sim/python.sh /workspace/isaac/snapshot_cameras.py \\
      --scene-usd /workspace/doosan-cap/generated_by_LLM/sim_env.usd \\
      --output-dir /workspace/doosan-cap/generated_by_LLM/camera_snapshots

  # Override one or more camera poses (position in m, rotation in deg XYZ)
  /isaac-sim/python.sh /workspace/isaac/snapshot_cameras.py \\
      --scene-usd /workspace/doosan-cap/generated_by_LLM/sim_env.usd \\
      --output-dir /workspace/doosan-cap/generated_by_LLM/camera_snapshots \\
      --override /World/Camera_Front    -1.9  0.0 0.6  90 0 -90 \\
      --override /World/Camera_Overhead -0.55 0.0 1.5   0 0  90
"""

import argparse
import os
import sys


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--scene-usd", required=True)
    p.add_argument("--output-dir", required=True)
    p.add_argument("--width",  type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument(
        "--warmup-frames", type=int, default=30,
        help="App update count between attaching annotator and grabbing the frame. "
             "Raise if images look noisy/half-loaded; lower for faster turnaround.",
    )
    p.add_argument(
        "--override", action="append", default=[],
        nargs=7,
        metavar=("PRIM", "TX", "TY", "TZ", "RX", "RY", "RZ"),
        help="Override a camera's transform. Position in metres, rotation in degrees "
             "(XYZ Euler — matches scene_builder_api.py convention). Can be repeated.",
    )
    return p.parse_args()


ARGS = parse_args()

# Start Isaac Sim headless before any omni import
from isaacsim import SimulationApp  # noqa: E402
simulation_app = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})

import carb        # noqa: E402
import numpy as np  # noqa: E402
import omni.usd    # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from pxr import UsdGeom, Gf  # noqa: E402

# Optional PNG encoder
try:
    from PIL import Image
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False
    print("[warn] PIL unavailable; falling back to .ppm output.", flush=True)

if not os.path.isfile(ARGS.scene_usd):
    carb.log_error(f"scene USD not found: {ARGS.scene_usd}")
    simulation_app.close()
    sys.exit(1)

omni.usd.get_context().open_stage(ARGS.scene_usd)
for _ in range(10):
    simulation_app.update()
stage = omni.usd.get_context().get_stage()

# Apply per-camera overrides
for ov in ARGS.override:
    prim_path = ov[0]
    try:
        tx, ty, tz = (float(v) for v in ov[1:4])
        rx, ry, rz = (float(v) for v in ov[4:7])
    except ValueError as e:
        print(f"[error] cannot parse override for {prim_path}: {e}", file=sys.stderr)
        continue
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"[warn] prim not found, skipping override: {prim_path}", file=sys.stderr)
        continue
    xf = UsdGeom.Xformable(prim)
    existing = {op.GetOpType(): op for op in xf.GetOrderedXformOps()}
    if UsdGeom.XformOp.TypeTranslate in existing:
        existing[UsdGeom.XformOp.TypeTranslate].Set(Gf.Vec3d(tx, ty, tz))
    else:
        xf.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if UsdGeom.XformOp.TypeRotateXYZ in existing:
        existing[UsdGeom.XformOp.TypeRotateXYZ].Set(Gf.Vec3f(rx, ry, rz))
    else:
        xf.AddRotateXYZOp().Set(Gf.Vec3f(rx, ry, rz))
    print(f"[override] {prim_path}: pos=({tx},{ty},{tz}) rot=({rx},{ry},{rz})",
          flush=True)

simulation_app.update()

# Find camera prims under the stage
cameras = [str(p.GetPath()) for p in stage.TraverseAll()
           if p.GetTypeName() == "Camera"]
print(f"[snapshot] cameras found: {cameras}", flush=True)
if not cameras:
    carb.log_error("no Camera prims in stage")
    simulation_app.close()
    sys.exit(1)

os.makedirs(ARGS.output_dir, exist_ok=True)


def save_image(arr: np.ndarray, path_no_ext: str) -> str:
    if _HAS_PIL:
        out = path_no_ext + ".png"
        Image.fromarray(arr).save(out)
    else:
        out = path_no_ext + ".ppm"
        h, w = arr.shape[:2]
        with open(out, "wb") as f:
            f.write(f"P6\n{w} {h}\n255\n".encode())
            f.write(arr.tobytes())
    return out


for cam_path in cameras:
    safe = cam_path.strip("/").replace("/", "_")
    out_base = os.path.join(ARGS.output_dir, safe)

    rp = rep.create.render_product(cam_path, (ARGS.width, ARGS.height))
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach([rp])

    for _ in range(ARGS.warmup_frames):
        simulation_app.update()

    data = annot.get_data()
    if data is None:
        print(f"[warn] no data for {cam_path}, skipping", flush=True)
        try:
            annot.detach([rp])
        except Exception:
            pass
        continue
    if isinstance(data, dict):
        data = data["data"]
    rgb = np.ascontiguousarray(data[:, :, :3])
    out_path = save_image(rgb, out_base)
    print(f"[saved] {out_path}  ({rgb.shape[1]}x{rgb.shape[0]})", flush=True)

    try:
        annot.detach([rp])
    except Exception:
        pass

simulation_app.close()
