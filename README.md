# Doosan M1013 + Isaac Sim 4.5 + ROS 2 Humble + DRCF Emulator

Single-container setup that lets a Doosan M1013 visualized in Isaac Sim 4.5
mirror joint state coming from the Doosan DRCF emulator (or a real M1013) via
ROS 2 Humble + the `doosan-robot2` driver stack.

The same Dockerfile is built identically on a personal PC (with the emulator,
for development) and on a lab PC (with the real robot). The only runtime
differences are which `mode:=` and `host:=` you pass to the launch file.

## Architecture in one picture

```
┌────────────────────────┐     ┌────────────────────────┐
│   DRCF emulator        │     │      Isaac Sim          │
│   (virtual mode)       │     │      (PhysX + RTX)      │
├────────────────────────┤     ├────────────────────────┤
│ ✓ Motion planning      │     │ ✓ Rendering             │
│ ✓ Trajectory (vel/acc) │     │ ✓ Environment physics   │
│ ✓ Kinematics           │     │   (gravity / collision) │
│ ✓ Robot state machine  │     │ ✓ Joint position drive  │
│                        │     │ ✓ Sensors / perception  │
└────────────┬───────────┘     └────────────▲───────────┘
             │                              │
             │ TCP 127.0.0.1:12345          │ Action Graph
             │                              │   subscribes /joint_states
             ▼                              │   drives articulation
      dsr_hardware2  ─ publishes ───────────┘
      (ROS 2 driver)   /dsr01/joint_states
```

- **DRCF emulator** is the official Doosan controller binary in a container.
  In virtual mode, it simulates the motion trajectory internally.
- **doosan-robot2** is Doosan's official ROS 2 driver. It talks to DRCF over
  TCP using the same binary protocol as a real robot, so sim → real swap is
  only a `mode:=` + `host:=` change.
- **Isaac Sim** loads the official `m1013.usd` shipped with `dsr_description2`
  and uses an OmniGraph `ROS2SubscribeJointState → IsaacArticulationController`
  chain to drive the articulation from the live ROS 2 stream.

## Repository layout

```
doosan_docker_skeleton/
├── docker/
│   ├── Dockerfile           # nvcr Isaac Sim 4.5 + ROS 2 Humble + apt deps
│   ├── bootstrap_ws.sh      # first-run: clones pinned doosan-robot2 + colcon build
│   ├── entrypoint.sh        # sources /opt/ros/humble + /ros2_ws/install
│   ├── container.sh         # {start|enter|stop|clean} container manager
│   ├── docker_build.sh      # standalone image build
│   └── run_emulator.sh      # host-side DRCF emulator launcher
├── isaac/
│   └── m1013_ros2_bridge.py # Isaac Sim standalone app: USD load + ROS 2 graph
├── scripts/
│   └── m1013_isaac_bringup.launch.py  # minimal doosan-robot2 launch, no rviz
├── third_party/             # runtime-only, git-ignored (see below)
│   └── doosan-robot2/       # ← cloned by bootstrap_ws.sh on first run
└── README.md
```

Everything under `third_party/` and `docker/cache/` is **git-ignored** and
regenerated on each fresh machine by `container.sh start`.

## Prerequisites (host side, one-time)

- Ubuntu 22.04
- NVIDIA GPU + recent driver (tested on RTX 4060 + driver 580.x)
- Docker Engine
- **NVIDIA Container Toolkit** with the `nvidia` runtime registered

Verify:

```bash
lsb_release -a                    # Ubuntu 22.04
nvidia-smi                        # GPU visible
docker info | grep -i runtime     # must include 'nvidia'
df -h ~                           # at least ~30 GB free (Isaac Sim image is ~15 GB)
```

If the `nvidia` runtime is missing, install it once:

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

Allow containers to use the X display (once per session or persist as you
like):

```bash
xhost +local:docker
```

## Setup (any machine, clean checkout)

```bash
git clone https://github.com/GeunYoung-Heo/doosan_docker_skeleton.git
cd doosan_docker_skeleton
chmod +x docker/*.sh
bash docker/container.sh start     # ~25–45 min on first run
```

`container.sh start` is idempotent and does the following on first run:

1. `docker build` → pulls `nvcr.io/nvidia/isaac-sim:4.5.0` (~15 GB), installs
   ROS 2 Humble + `ros-humble-ros-base`, moveit core, ros2_control, and the
   Doosan runtime dependencies.
2. Creates the `doosan_isaac` container (`--gpus all --network host --ipc host
   --privileged`, X11 forwarding, Isaac Sim caches bind-mounted to
   `docker/cache/isaac-*`).
3. Runs `bootstrap_ws.sh` inside the container, which:
   - clones `doosan-robot2` (humble branch, pinned commit) into
     `third_party/doosan-robot2/` on the host (via bind mount);
   - `rosdep install` + `colcon build --symlink-install` over the 22
     `dsr_*` packages;
   - leaves the container running in the background (`tail -f /dev/null`).

Subsequent `container.sh start` calls just `docker start` the existing
container. Pass `stop`, `enter`, or `clean` for the other verbs.

## Run order (five terminals)

All of the work beyond Terminal 1 happens inside the same `doosan_isaac`
container — Terminals 2–5 are just independent shells into it via
`docker exec -it doosan_isaac bash`.

### Terminal 1 — host: DRCF emulator

```bash
cd ~/doosan_docker_skeleton
bash docker/run_emulator.sh            # default: m1013, port 12345
ss -tlnp | grep 12345                  # LISTEN on 12345 = ok
```

The emulator is a separate Docker container (`doosanrobot/dsr_emulator:3.0.1`)
running in `--network host` mode. Stop it with `docker rm -f emulator`.

### Terminal 2 — container: Isaac Sim bridge

```bash
docker exec -it doosan_isaac bash
/isaac-sim/python.sh /workspace/isaac/m1013_ros2_bridge.py \
    --topic dsr01/joint_states
```

The script:

1. Boots a standalone Isaac Sim 4.5 app with the RTX renderer.
2. Enables the `isaacsim.ros2.bridge` extension.
3. Loads the official `m1013.usd` from
   `/ros2_ws/src/third_party/doosan-robot2/dsr_description2/usd/m1013.usd`.
4. Patches the stage in memory: moves `ArticulationRootAPI` from the
   `root_joint` fixed joint onto `base_link` so PhysX tensors can register a
   valid articulation (the shipped USD applies the API to the wrong prim
   type — see below).
5. Builds an OmniGraph that wires
   `OnPlaybackTick → ROS2SubscribeJointState → IsaacArticulationController`
   and also publishes `/isaac_joint_states`, `/clock`, and a mirror of joint
   state for debug.
6. Enters the main simulation loop.

First run compiles RTX shaders and can take several minutes. A successful run
prints:

```
[bridge] referencing USD: /ros2_ws/.../m1013.usd
[bridge] removed ArticulationRootAPI from /World/m1013/m1013/root_joint
[bridge] applied ArticulationRootAPI to /World/m1013/m1013/base_link
[bridge] articulation robotPath: /World/m1013/m1013/base_link
[bridge] graph ready. Subscribing to '/dsr01/joint_states', echoing on '/isaac_joint_states'
[bridge] entering main loop. Ctrl-C to quit.
```

### Terminal 3 — container: doosan-robot2 driver

```bash
docker exec -it doosan_isaac bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch /ros2_ws/src/m1013_isaac_bringup.launch.py \
    mode:=virtual host:=127.0.0.1 port:=12345 model:=m1013
```

This is a minimal launch file that intentionally **excludes** rviz (Isaac Sim
is our viewer) and runs only:

- `ros2_control_node` (loads the `dsr_hardware2` system interface)
- `robot_state_publisher`
- `joint_state_broadcaster` spawner
- `dsr_controller2` spawner

A successful bringup ends with `Configured and activated dsr_controller2`. The
`dsr_hardware2` log must say `mode : virtual` and `Emulator Mode` — if it says
`Real Robot Control Mode`, the emulator silently rejects every motion and
`get_current_posj` stays at zero.

### Terminal 4 — container: joint state monitor

```bash
docker exec -it doosan_isaac bash
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
ros2 topic echo /dsr01/joint_states --field position
```

Positions are in radians. Expected rate is ~100 Hz.

### Terminal 5 — container: send motion commands

```bash
docker exec -it doosan_isaac bash
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
ros2 service call /dsr01/motion/move_home dsr_msgs2/srv/MoveHome "{target: 0}"
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint \
    "{pos: [0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel: 30.0, acc: 30.0, time: 0.0, radius: 0.0, mode: 0, blend_type: 0, sync_type: 0}"
```

`move_joint` is blocking — it returns only when the trajectory has finished
(~3 s for this example). While it runs, Terminal 4 should show positions
interpolating from `[0, 0, 0, 0, 0, 0]` to `[0, 0, π/2, 0, π/2, 0]` (note the
ROS joint-name order inside the topic is `[joint_1, joint_2, joint_4, joint_5,
joint_3, joint_6]`, so entries 3 and 4 are the `joint_5` and `joint_3`
values). Isaac Sim's M1013 follows in real time.

## Switching to the real M1013 on the lab PC

The Docker image and all scripts are identical. Only the runtime arguments
change.

| Step | Personal PC (emulator) | Lab PC (real robot) |
| ---- | ---------------------- | ------------------- |
| Terminal 1 | `bash docker/run_emulator.sh` | *skip* — real robot is always on |
| Terminal 3 `mode` | `virtual` | `real` |
| Terminal 3 `host` | `127.0.0.1` | `<ROBOT_IP>` (ask the lab) |
| Safety | — | Check workspace, reduce `vel`/`acc`, know the E-stop |

Everything else — Terminals 2, 4, 5 and all scripts — is byte-for-byte
identical.

## How reproducibility works across machines

1. **Docker image**: base is the exact tag `nvcr.io/nvidia/isaac-sim:4.5.0`,
   and all subsequent `apt` + `rosdep` steps run against pinned Ubuntu 22.04
   and ROS 2 Humble package archives.
2. **`doosan-robot2` source**: [bootstrap_ws.sh](docker/bootstrap_ws.sh) pins
   commit `ec9242546ec6202835900dbcd8498e2daabfa6a6` on the `humble` branch
   and `checkout`s it after cloning. Update this constant on purpose; never
   implicitly.
3. **M1013 USD**: comes from the pinned `doosan-robot2` clone above, so every
   machine references the same geometry.
4. **Isaac Sim API fix-up**: the `ArticulationRootAPI` move is applied at
   runtime in [isaac/m1013_ros2_bridge.py](isaac/m1013_ros2_bridge.py), not
   saved into the USD, so the upstream asset is left untouched.

If all four of these are stable, both machines produce the same simulation
behavior.

## Troubleshooting

- **`docker build` fails with `libfreetype6-dev` unmet dependency**: you added
  `ros-humble-desktop` or full `ros-humble-moveit` back to the Dockerfile.
  Don't. Isaac Sim is the viewer; rviz is unnecessary and its `libfreetype6-dev`
  dependency conflicts with the `nvcr.io/nvidia/isaac-sim:4.5.0` base image.
- **Isaac Sim window never appears**: run `xhost +local:docker` on the host
  and recreate the container (`bash docker/container.sh clean && bash
  docker/container.sh start`). Headless alternative: pass `--headless` to the
  bridge script.
- **`omni.physx.tensors.plugin: did not match any rigid bodies`**: the bridge
  script's `ArticulationRootAPI` patch did not find `base_link` under the
  USD's reference root. Check the `[bridge] === stage dump ===` output to see
  the actual prim hierarchy and update the script's `base_link` path if the
  USD layout changed.
- **`/dsr01/joint_states` is stuck at all zeros and `move_*` returns
  `success=True`**: you launched with `mode:=real` while pointed at the
  emulator. Re-launch with `mode:=virtual`. In real mode DRCF expects an
  actual robot encoder and silently rejects motions as "Too Close Target".
- **Emulator container keeps printing `#TARGET MODE: Real Robot Control
  Mode`**: same symptom as above; use `mode:=virtual` on Terminal 3 or
  connect to a real robot.
- **`ros2 control list_controllers` reports "waiting for service"**: the
  controller manager is under the `dsr01` namespace — append
  `--controller-manager /dsr01/controller_manager`.
- **Isaac Sim viewport does not animate even though topics look fine**: check
  that the timeline is playing (▶ button in the Isaac Sim UI or Spacebar in
  the viewport). OmniGraph ROS 2 nodes only tick while the timeline is
  playing.

## What was verified

On the development machine the following end-to-end path has been exercised
successfully:

1. `docker build` + `bootstrap_ws.sh` + `colcon build` (22 `dsr_*` packages).
2. Isaac Sim 4.5 loads `m1013.usd`, OmniGraph wires subscribe + controller.
3. `move_joint` / `move_home` service calls are accepted by the DRCF emulator
   in `virtual` mode and the resulting trajectory is reflected in
   `/dsr01/joint_states` at 100 Hz.
4. Isaac Sim's M1013 mirrors the commanded trajectory in real time.

The lab PC replication via `git clone` + `container.sh start` is intended to
reproduce exactly this path with no file editing.
