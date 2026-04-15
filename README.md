# Doosan M1013 + Isaac Sim 4.5 + ROS 2 Humble + MoveIt

Single-container sim environment for a Doosan M1013 manipulator. MoveIt plans
Cartesian / joint-space motions and Isaac Sim 4.5 renders the robot's PhysX
articulation as it executes. The DRCF emulator is **not** used for motion —
everything runs on `mock_components` + Isaac Sim, so there is no jerky amovej
behavior and no second Docker container to manage.

This repo is the sim environment only. The same `move_group` / ROS 2 control
layer will be reused on a lab PC against a real M1013, with a small launch
swap. See "Switching to real robot" below.

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                      doosan_isaac container                      │
│                                                                  │
│  (CaP / test scripts call MoveIt — e.g. /move_action)            │
│    │                                                             │
│    ▼                                                             │
│  move_group (MoveIt OMPL)                                        │
│    │ IK + planning + time parameterization                       │
│    │ FollowJointTrajectory action                                │
│    ▼                                                             │
│  dsr_moveit_controller (joint_trajectory_controller/JTC, 100 Hz) │
│    │ position command interface                                  │
│    ▼                                                             │
│  mock_components/GenericSystem  ← hardware interface             │
│    │ (echo command → state, instant tracking)                    │
│    │ position state interface                                    │
│    ▼                                                             │
│  joint_state_broadcaster                                         │
│    │                                                             │
│    ▼                                                             │
│  /joint_states (100 Hz)                                          │
│    │                                                             │
│    ├─► robot_state_publisher → /tf                               │
│    │                                                             │
│    └─► Isaac Sim OmniGraph                                       │
│            │ ROS2SubscribeJointState                             │
│            ▼                                                     │
│         IsaacArticulationController                              │
│            ▼                                                     │
│         M1013 PhysX articulation in Isaac Sim viewport           │
│         (+ gravity, collision, camera sensors — future work)     │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

Why this design:

1. **`move_group` produces the same time-parameterized trajectory whether
   the backend is sim or real** — it only looks at URDF + SRDF + joint_limits.
   The planned trajectory is identical in both modes.
2. **PhysX drives track commanded positions faithfully** — visual and
   kinematic behavior matches the real robot closely enough for CaP-style
   high-level validation (collision, reachability, sequence correctness).
3. **Collision detection happens in MoveIt's `PlanningScene`** — environment
   objects go there at plan time. Isaac Sim PhysX is the visual safety net.
4. **Swap to real robot = one xacro / hardware interface change**. Launch
   structure, MoveIt config, CaP code, controllers, action names — all stay
   the same.

Why DRCF emulator was dropped:

Earlier revisions used the `doosanrobot/dsr_emulator:3.0.1` container plus
`dsr_hardware2` as the sim backend. In virtual mode, `dsr_hw_interface2.cpp`
calls `Drfl.amovej(...)` — a point-to-point motion primitive — for every
`write()` tick. With `joint_trajectory_controller` feeding ~100 position
setpoints per second, each `amovej` call restarts an accel/decel profile and
interrupts the previous one. The result is visibly jerky motion that can be
reproduced with upstream `dsr_bringup2_moveit.launch.py` too. DRCF emulator
is a protocol / state machine harness, not a physics simulator.

## Repository layout

```
doosan_docker_skeleton/
├── docker/
│   ├── Dockerfile              # nvcr Isaac Sim 4.5 + ROS 2 Humble + MoveIt deps
│   ├── bootstrap_ws.sh         # first-run: clone doosan-robot2 (humble @ pinned SHA) + colcon build
│   ├── entrypoint.sh
│   ├── container.sh            # {start|enter|stop|clean} container manager
│   ├── docker_build.sh
│   └── run_emulator.sh         # kept for protocol-level testing; NOT used by sim
├── isaac/
│   └── m1013_ros2_bridge.py    # Isaac Sim standalone app: USD load + ROS 2 bridge graph
├── scripts/
│   ├── m1013_sim_bringup.launch.py      # MoveIt + mock_components, no DRCF
│   ├── dsr_moveit_controller_sim.yaml   # JTC command_interfaces override (position only)
│   ├── moveit_backend_smoketest.py      # joint-space goal test
│   └── moveit_pose_test.py              # Cartesian pose goal test (CaP-like API)
├── third_party/                 # runtime-only, git-ignored (bootstrap clones here)
└── README.md
```

## Prerequisites (host, one-time)

- Ubuntu 22.04
- NVIDIA GPU + recent driver
- Docker Engine + **NVIDIA Container Toolkit** (`docker info | grep -i runtime` must include `nvidia`)
- `xhost +local:docker` (each session, or persist)

If NVIDIA Container Toolkit is missing:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
    sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg && \
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null && \
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit && \
sudo nvidia-ctk runtime configure --runtime=docker && \
sudo systemctl restart docker
```

## Setup

```bash
git clone https://github.com/GeunYoung-Heo/doosan_docker_skeleton.git
cd doosan_docker_skeleton
chmod +x docker/*.sh
bash docker/container.sh start
```

`container.sh start` is idempotent and on first run does the following (~25 minutes total):

1. `docker build` → pulls `nvcr.io/nvidia/isaac-sim:4.5.0` (~15 GB), installs
   ROS 2 Humble + `ros-humble-ros-base` + moveit core + ros2_control + the
   Doosan runtime dependencies.
2. Creates the `doosan_isaac` container (`--gpus all --network host --ipc host
   --privileged`, X11 forwarding, Isaac Sim caches bind-mounted).
3. Runs `bootstrap_ws.sh` inside the container, which:
   - clones `doosan-robot2` (humble branch, pinned commit `ec9242546`) into
     `third_party/doosan-robot2/`;
   - `rosdep install` + `colcon build --symlink-install`;
   - leaves the container running as a background idle process.

Subsequent `container.sh start` calls just `docker start` the existing
container. Use `stop`, `enter`, and `clean` for the other verbs.

## Run order

Three terminals (all open to the **same** `doosan_isaac` container via
`docker exec -it`):

### Terminal 1 — sim bringup (MoveIt + mock_components)

```bash
docker exec -it doosan_isaac bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch /ros2_ws/src/m1013_sim_bringup.launch.py
```

Wait until you see `You can start planning now!` — typically ~30 seconds
after launch. At this point:

- `/controller_manager` is active with `joint_state_broadcaster` +
  `dsr_moveit_controller`;
- `/joint_states` publishes at 100 Hz;
- `/move_action` (MoveGroup action) is ready.

### Terminal 2 — Isaac Sim viewport

```bash
docker exec -it doosan_isaac bash
/isaac-sim/python.sh /workspace/isaac/m1013_ros2_bridge.py
```

The script:

1. Boots a standalone Isaac Sim 4.5 app with the RTX renderer.
2. Enables the `isaacsim.ros2.bridge` extension.
3. Loads the official `m1013.usd` from `dsr_description2`.
4. Moves the `ArticulationRootAPI` from the shipped `root_joint` fixed joint
   onto `base_link` so PhysX tensors can register a valid articulation.
5. Wires an OmniGraph `ROS2SubscribeJointState → IsaacArticulationController`
   chain that mirrors `/joint_states` into the M1013 PhysX articulation.
6. Auto-plays the timeline and enters the main loop.

When the window appears, the robot may start at a non-home pose — it will
converge to whatever `/joint_states` is currently reporting (mock_components
remembers the last commanded state across bridge restarts).

Headless alternative: add `--headless`.

### Terminal 3 — command / test

```bash
docker exec -it doosan_isaac bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

#### Joint-space test

```bash
python3 /ros2_ws/src/moveit_backend_smoketest.py
```

Sends a "go to all zeros" joint goal via `/move_action`.

#### Cartesian pose test (the CaP-style interface)

```bash
# Default — tip at [0.45, 0, 0.55] with tip pointing down
python3 /ros2_ws/src/moveit_pose_test.py

# Custom target
python3 /ros2_ws/src/moveit_pose_test.py --xyz 0.4 0.3 0.4

# Position only (MoveIt picks any orientation)
python3 /ros2_ws/src/moveit_pose_test.py --xyz 0.6 0.0 0.3 --position-only

# Unreachable — should return PLANNING_FAILED
python3 /ros2_ws/src/moveit_pose_test.py --xyz 1.5 0 0.5
```

Each successful call:

1. Sends a MoveGroup goal with PositionConstraint (+ optional OrientationConstraint);
2. MoveIt solves IK + plans + executes;
3. `/joint_states` transitions to the new configuration;
4. Isaac Sim mirrors the motion in the viewport smoothly (no DRCF amovej jerk).

This is the same interface CaP will call. A higher-level wrapper would
build `MoveGroup.Goal` objects via the helpers in `moveit_pose_test.py`'s
`build_pose_goal()`.

## Verification

After following the run order once, the following should all hold:

- [ ] `ros2 control list_controllers --controller-manager /controller_manager`
      shows `joint_state_broadcaster` + `dsr_moveit_controller` both `active`.
- [ ] `ros2 action list | grep move_action` returns `/move_action`.
- [ ] `ros2 topic hz /joint_states` shows ~100 Hz.
- [ ] `moveit_backend_smoketest.py` returns `SUCCESS` and `/joint_states`
      goes to `[0, 0, 0, 0, 0, 0]`.
- [ ] `moveit_pose_test.py` returns `SUCCESS` for the default pose.
- [ ] The Isaac Sim viewport shows the M1013 moving smoothly during each
      motion (no jerky stop-start behavior).

## Switching to a real M1013

Not implemented in this repo yet; outline only:

1. Write a second launch file (e.g. `m1013_real_bringup.launch.py`) that:
   - Builds `robot_description` from `dsr_description2`'s main xacro with
     `mode:=real host:=<ROBOT_IP> port:=12345` — this xacro uses
     `dsr_hardware2/DRHWInterface` as the ros2_control plugin.
   - Loads `dsr_controller2.yaml` the same way `m1013_sim_bringup.launch.py` does.
   - Still spawns `joint_state_broadcaster` + `dsr_moveit_controller`.
   - Still launches `move_group` via the same `MoveItConfigsBuilder` call.
2. Skip the Isaac Sim viewport (or run it in parallel as a digital twin if
   desired).
3. CaP scripts and `moveit_pose_test.py` work unchanged — the action name is
   still `/move_action`.

The key takeaway: everything above the hardware interface (MoveIt, controllers,
topics, actions, scripts) is identical in sim and real. Only the ros2_control
plugin + its URDF source changes.

## Troubleshooting

- **`docker build` fails on `libfreetype6-dev` unmet dependency** — a
  `ros-humble-*` package pulled rviz/ogre. This Dockerfile intentionally
  avoids `ros-humble-desktop` and the full `ros-humble-moveit` metapackage.
  Isaac Sim is the viewer; rviz's `libfreetype6-dev` transitively conflicts
  with the nvcr Isaac Sim base image.
- **Isaac Sim window never appears** — run `xhost +local:docker` and recreate
  the container with `bash docker/container.sh clean && bash docker/container.sh start`.
  Headless alternative: pass `--headless` to the bridge.
- **`omni.physx.tensors.plugin: did not match any rigid bodies`** — the
  `ArticulationRootAPI` patch in the bridge did not find `base_link` under
  the USD reference. Check the bridge's `[bridge] === stage dump ===`
  output (uncomment the walk) and adjust the path if the USD layout changes.
- **`dsr_moveit_controller` activates as `inactive` / "Failed to activate"** —
  the JTC is trying to claim both `position` and `velocity` command
  interfaces but `mock_components` only exposes `position`. Check that
  `scripts/dsr_moveit_controller_sim.yaml` is in the `control_node`'s
  `parameters=` list and is being loaded (you should see it in the param
  file arguments of `ros2_control_node`).
- **`move_action` goal returns `PLANNING_FAILED` / `NO_IK_SOLUTION`** — the
  requested pose is outside M1013's workspace or in collision. Try a closer
  xyz or pass `--position-only` to let MoveIt pick any orientation.
- **Isaac Sim starts at a non-home pose** — `mock_components` remembers the
  last commanded state across restarts of the Isaac Sim bridge (but not
  across restarts of `m1013_sim_bringup.launch.py`, which resets to the USD
  defaults). Send a home goal first:
  `python3 /ros2_ws/src/moveit_backend_smoketest.py`.

## Reference — what's NOT used any more (legacy)

This repo used to have a larger surface area exploring DRCF-backed sim and
an Isaac Sim drag-to-command UI. Those files were removed once we confirmed
that:

- DRCF emulator (virtual mode) gives jerky motion via `amovej`, reproduced
  with upstream Doosan launches too;
- the drag UI was a test-time convenience and CaP will drive the robot via
  API calls, not mouse drags.

`docker/run_emulator.sh` is kept as a convenience for someone who wants to
talk to a real DRCF binary for protocol-level testing. It is not wired into
any launch in this repo.
