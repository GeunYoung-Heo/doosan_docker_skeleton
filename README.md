# Doosan M1013 + Isaac Sim 4.5 + ROS 2 Humble + MoveIt

Doosan M1013 매니퓰레이터 + OnRobot RG2-FT V2 그리퍼를 위한 단일 컨테이너 환경.  
실물 로봇과 Isaac Sim 시뮬레이션을 동일한 MoveIt/ROS 2 인터페이스로 제어한다.

---

## 빠른 시작 — 실물 로봇 + 그리퍼

> 처음 세팅하는 경우라면 아래 "전체 설치 가이드"를 먼저 읽고 돌아오세요.

```bash
# 1. 컨테이너 시작 (이미 생성된 경우 docker start만 실행됨)
bash docker/container.sh start

# 2. 컨테이너 진입 후 launch
bash docker/container.sh enter
source /ros2_ws/install/setup.bash
ros2 launch /ros2_ws/src/real_bringup.launch.py \
    host:=192.168.137.100 gripper_host:=192.168.1.1
```

`You can start planning now!` 메시지가 나오면 팔 + 그리퍼 모두 준비된 상태.

```bash
# 새 터미널에서
bash docker/container.sh enter
source /ros2_ws/install/setup.bash

# 그리퍼 열기
ros2 service call /onrobot_rg2_ft_node/set_gripper \
    onrobot_rg2_ft/srv/SetGripper "{width: 110.0, force: 5.0}"

# 팔 이동 (안전한 workspace)
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.0 0.55 --cartesian
```

---

## 전체 설치 가이드 (최초 1회)

### 1단계 — 호스트 소프트웨어 요구사항

| 항목 | 요구사항 |
|------|----------|
| OS | Ubuntu 22.04 |
| GPU | NVIDIA GPU + 드라이버 |
| Docker | Docker Engine 설치 |
| NVIDIA Container Toolkit | `docker info \| grep -i runtime` 에 `nvidia` 포함 |

```bash
# 확인 명령어
lsb_release -a 2>/dev/null | grep Release          # 22.04
nvidia-smi | head -3                                # GPU 정보 출력
docker --version                                    # Docker 버전
docker info 2>/dev/null | grep -i runtimes          # nvidia 포함 여부
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi | head -3  # GPU 컨테이너 접근 확인
```

NVIDIA Container Toolkit이 없다면:

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

### 2단계 — 호스트 네트워크 설정 (최초 1회, 재부팅 후에도 유지)

컨테이너는 `--network host`로 실행되므로 호스트에 라우트를 설정하면 컨테이너 안에서도 자동으로 접근 가능하다.

```bash
# 현재 네트워크 인터페이스 확인
ip addr show

# enp5s0에 해당하는 ConnectionManager 연결 이름 확인
nmcli connection show
```

로봇과 Compute Box가 연결된 인터페이스(예: `enp5s0`, 연결 이름 `Wired connection 1`)에 보조 IP를 영구 등록:

```bash
# 로봇 컨트롤러 대역 (192.168.137.x) — 이미 설정되어 있다면 생략
nmcli connection modify "Wired connection 1" +ipv4.addresses 192.168.137.50/24

# OnRobot Compute Box 대역 (192.168.1.x)
nmcli connection modify "Wired connection 1" +ipv4.addresses 192.168.1.100/24

# 적용
nmcli connection up "Wired connection 1"

# 확인 — 두 IP 모두 noprefixroute + valid_lft forever 로 표시되면 성공
ip addr show enp5s0
```

> 이 설정은 재부팅해도 유지된다. 이후에는 별도의 네트워크 설정 없이 컨테이너를 시작하면 된다.

연결 확인:

```bash
ping -c 2 192.168.137.100   # 로봇 컨트롤러
ping -c 2 192.168.1.1       # OnRobot Compute Box
```

### 3단계 — 리포지토리 클론 및 컨테이너 최초 빌드

```bash
git clone https://github.com/GeunYoung-Heo/doosan_docker_skeleton.git
cd doosan_docker_skeleton
chmod +x docker/*.sh
bash docker/container.sh start
```

`container.sh start` 최초 실행 시 약 25분 소요:

1. `docker build` — `nvcr.io/nvidia/isaac-sim:4.5.0` (~15 GB) 다운로드 + ROS 2 Humble / MoveIt / ros2_control 설치
2. `doosan_isaac` 컨테이너 생성 (`--gpus all --network host --ipc host --privileged`)
3. `bootstrap_ws.sh` 자동 실행:
   - `doosan-robot2` (humble 브랜치, 고정 커밋 `ec9242546`) 클론
   - `rosdep install` + `colcon build --symlink-install`

이후 `container.sh start`는 기존 컨테이너를 `docker start`만 한다.

### 4단계 — 그리퍼 패키지 빌드 (최초 1회)

```bash
bash docker/container.sh enter

cd /ros2_ws
colcon build --symlink-install --packages-select onrobot_rg2_ft
source install/setup.bash
```

빌드 성공 확인:

```bash
ros2 pkg list | grep onrobot   # onrobot_rg2_ft 출력되면 성공
```

> `--symlink-install` 덕분에 `.py` 파일 수정은 재빌드 불필요.  
> `msg/`, `srv/`, `CMakeLists.txt` 변경 시에만 재빌드 필요.

---

## 실물 로봇 사용

### 매일 사용하는 절차

```
호스트                          컨테이너
─────────────────────────────────────────────────────
bash docker/container.sh start
bash docker/container.sh enter  →  Terminal 1: launch
                                →  Terminal 2: 제어 명령
```

**Terminal 1 — launch (팔 + 그리퍼 동시 기동)**

```bash
bash docker/container.sh enter
source /ros2_ws/install/setup.bash
ros2 launch /ros2_ws/src/real_bringup.launch.py \
    host:=192.168.137.100 gripper_host:=192.168.1.1
```

정상 기동 시 로그 순서:

```
[gripper_node] Connecting to Compute Box at 192.168.1.1:502 (Modbus TCP) ...
[gripper_node] Compute Box reachable.
[gripper_node] onrobot_rg2_ft_node ready.
...
[move_group] You can start planning now!
```

`You can start planning now!` 가 나오면 팔과 그리퍼 모두 준비 완료.

**Terminal 2 — 상태 확인**

```bash
bash docker/container.sh enter
source /ros2_ws/install/setup.bash

# 컨트롤러 확인
ros2 control list_controllers
# joint_state_broadcaster [active]
# dsr_moveit_controller   [active]

# 그리퍼 상태 모니터링 (5 Hz)
ros2 topic echo /onrobot_rg2_ft_node/state
# actual_width: 110.x, busy: False, gripped: False
```

### 그리퍼 제어

```bash
# 완전히 열기 (110 mm, 20 N)
ros2 service call /onrobot_rg2_ft_node/set_gripper \
    onrobot_rg2_ft/srv/SetGripper "{width: 110.0, force: 5.0}"

# 물체 파지 (그리퍼가 물체에 닿으면 자동 정지, gripped: True)
ros2 service call /onrobot_rg2_ft_node/set_gripper \
    onrobot_rg2_ft/srv/SetGripper "{width: 0.0, force: 5.0}"

# 중간 위치 (60 mm, 약한 힘)
ros2 service call /onrobot_rg2_ft_node/set_gripper \
    onrobot_rg2_ft/srv/SetGripper "{width: 60.0, force: 5.0}"
```

### MoveIt 웨이포인트 제어

#### 관절 공간 제어 (Joint-space)

6개 관절 각도를 **도(°)** 단위로 직접 지정한다. 스크립트가 라디안으로 변환해 MoveIt에 전달.

```bash
# 현재 관절 각도 확인 (라디안 → 수동으로 도 환산)
ros2 topic echo /joint_states --once

# 안전한 작업 시작 자세 (권장 홈)
python3 /ros2_ws/src/moveit_pose_test.py --joints 0 0 -90 0 -90 0

# 베이스 회전 (J1 = 30°)
python3 /ros2_ws/src/moveit_pose_test.py --joints 30 0 -90 0 -90 0

# 팁을 약간 들어올리기 (J2 조정)
python3 /ros2_ws/src/moveit_pose_test.py --joints 0 -20 -70 0 -90 0
```

> 실물 첫 동작 전에는 반드시 현재 관절 각도를 확인하고 목표를 설정할 것.  
> `0 0 -90 0 -90 0`이 안전한 기본 자세 — 팁이 아래로 향하고 팔이 접힌 상태.  
> 큰 각도 변화는 충돌 위험이 있다. 속도는 최대 30%로 제한되어 있다.

#### Cartesian 제어 (작업 공간, OMPL)

OMPL planner로 목표 pose까지 이동. 경로가 직선이 아닐 수 있음 (팔이 크게 자세를 바꿀 수 있음).

```bash
# 안전한 작업 영역 상단 (팁 아래 방향)
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.0 0.55

# 자세 없이 위치만 (MoveIt이 자세 자동 선택)
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.0 0.30 --position-only
```

> **안전한 workspace:** `x ≈ -0.45`, `y ∈ [-0.2, 0.2]`, `z ∈ [0.30, 0.55]`  
> 이 범위를 벗어난 좌표는 실물에서 먼저 시뮬로 검증 후 사용할 것.

#### 직선 경로 제어 (Cartesian straight-line) ⭐ 추천

`--cartesian` 옵션으로 EE가 **직선으로** 이동. OMPL과 달리 팔이 예측 가능하게 움직인다.  
Pick-and-place, 수직 접근/후퇴 등 대부분의 작업에 적합.

```bash
# 직선으로 이동 (작업 영역 상단 접근)
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.0 0.40 --cartesian

# 속도 조절 (기본: 0.3 = 최대 속도의 30%)
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.0 0.40 --cartesian --vel-scale 0.1   # 느리게 (10%)
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.0 0.40 --cartesian --vel-scale 0.5   # 빠르게 (50%)
```

**`--vel-scale` 참고:**

| 값 | 의미 | 용도 |
|---|---|---|
| `1.0` | 최대 속도 | — |
| `0.5` | 50% (2배 느림) | 일반 이동 |
| `0.3` | 30% (기본값, 3.3배 느림) | 안전한 기본 속도 |
| `0.1` | 10% (10배 느림) | 물체 근처 접근 |
| `0.05` | 5% (20배 느림) | 시연/디버깅 |

> `--cartesian`이 실패하면 ("only X% achievable") 현재 자세에서 목표까지 직선 경로가 불가능한 것 (singularity, joint limit). 이 경우 OMPL (`--cartesian` 없이)로 우회하거나 중간 waypoint를 추가.

### 팔 + 그리퍼 연계 예시 (pick-and-place)

```bash
# 1. 작업 시작 자세
python3 /ros2_ws/src/moveit_pose_test.py --joints 0 0 -90 0 -90 0

# 2. 그리퍼 열기
ros2 service call /onrobot_rg2_ft_node/set_gripper \
    onrobot_rg2_ft/srv/SetGripper "{width: 110.0, force: 5.0}"

# 3. 물체 위로 직선 접근
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.0 0.40 --cartesian

# 4. 아래로 내려가기 (천천히)
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.0 0.35 --cartesian --vel-scale 0.1

# 5. 파지
ros2 service call /onrobot_rg2_ft_node/set_gripper \
    onrobot_rg2_ft/srv/SetGripper "{width: 0.0, force: 5.0}"

# 6. 들어올리기
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.0 0.55 --cartesian

# 7. 옆으로 이동
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.2 0.55 --cartesian

# 8. 내려놓기 (천천히)
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.2 0.30 --cartesian --vel-scale 0.1

# 9. 놓기
ros2 service call /onrobot_rg2_ft_node/set_gripper \
    onrobot_rg2_ft/srv/SetGripper "{width: 110.0, force: 5.0}"

# 10. 위로 빠지기
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.2 0.55 --cartesian
```

> 위 명령은 시뮬과 실물에서 **동일하게** 동작한다.

### real_bringup.launch.py 전체 인수

| 인수 | 기본값 | 설명 |
|------|--------|------|
| `host` | `192.168.137.100` | 로봇 컨트롤러 IP |
| `port` | `12345` | 로봇 컨트롤러 포트 |
| `mode` | `real` | `real` 또는 `virtual` |
| `model` | `m1013` | 로봇 모델 |
| `rt_host` | `192.168.137.50` | RT 인터페이스 IP |
| `gripper_host` | `192.168.1.1` | OnRobot Compute Box IP |
| `gui` | `false` | RViz2 실행 여부 (이 컨테이너에서는 stub) |
| `name` | `` | 로봇 네임스페이스 |

---

## 시뮬레이션 사용 (Isaac Sim)

M1013 팔 + OnRobot RG2-FT 그리퍼가 하나의 combined URDF로 Isaac Sim에 로드된다.  
**실물과 완전히 동일한 ROS 2 명령**으로 팔과 그리퍼를 동시에 제어할 수 있다.

### Terminal 1 — MoveIt + mock_components + 그리퍼 sim 노드

```bash
bash docker/container.sh enter
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch /ros2_ws/src/m1013_sim_bringup.launch.py
```

정상 기동 시 두 줄을 확인:
- `Sim gripper ready. width=110 mm (open)` ← 그리퍼 sim 노드
- `You can start planning now!` ← MoveIt

### Terminal 2 — Isaac Sim 뷰포트

```bash
bash docker/container.sh enter
/isaac-sim/python.sh /workspace/isaac/m1013_ros2_bridge.py
```

창이 열리면 M1013 + RG2-FT 그리퍼가 하나의 아티큘레이션으로 렌더링된다.  
팔은 `/joint_states`를, 그리퍼는 `/gripper_finger_target`을 따라 움직인다.

### Terminal 3 — 동작 테스트 (실물과 동일한 명령)

```bash
bash docker/container.sh enter
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# 관절 공간 목표 (도 단위) — 안전한 작업 시작 자세
python3 /ros2_ws/src/moveit_pose_test.py --joints 0 0 -90 0 -90 0

# Cartesian 목표 (실물과 동일한 safe workspace)
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.0 0.55 --cartesian

# 그리퍼 닫기 (실물과 동일한 service call)
ros2 service call /onrobot_rg2_ft_node/set_gripper \
    onrobot_rg2_ft/srv/SetGripper "{width: 0.0, force: 5.0}"

# 그리퍼 열기
ros2 service call /onrobot_rg2_ft_node/set_gripper \
    onrobot_rg2_ft/srv/SetGripper "{width: 110.0, force: 5.0}"

# 그리퍼 상태 확인
ros2 topic echo /onrobot_rg2_ft_node/state --once
```

### 팔 + 그리퍼 연계 시퀀스 (pick-and-place 예시)

```bash
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.0 0.35 --cartesian   # 물체 위치로
ros2 service call /onrobot_rg2_ft_node/set_gripper \
    onrobot_rg2_ft/srv/SetGripper "{width: 0.0, force: 5.0}"                # 파지
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.0 0.55 --cartesian   # 들어올리기
ros2 service call /onrobot_rg2_ft_node/set_gripper \
    onrobot_rg2_ft/srv/SetGripper "{width: 110.0, force: 5.0}"              # 놓기
```

> 위 명령은 실물 로봇에서도 **그대로** 동작한다.

### 시뮬레이션 그리퍼 참고사항

- **width**: mm 단위, 0(완전 닫힘)~110(완전 열림). 내부에서 `finger_joint` 라디안으로 변환.
- **force**: 로그에만 기록됨. 시뮬에서는 물리적 접촉 힘을 적용하지 않음 (추후 scene 물체 추가 시 구현 가능).
- **gripped**: `width < 5 mm`이면 `True` 반환 (단순 threshold 판정).
- 시각화: Isaac Sim에서 그리퍼 손가락이 실시간으로 열리고 닫힘.

### combined URDF 재생성 (마운트 각도 수정이 필요한 경우)

```bash
# 컨테이너 안에서
bash /ros2_ws/src/generate_combined_urdf.sh
```

`generate_combined_urdf.sh` 내부의 `MOUNT_ORIGIN` 변수를 수정하면 tool0 ↔ gripper 간 회전/오프셋을 조정할 수 있다. 재생성 후 `isaac/m1013_rg2ft_combined.urdf`를 커밋하면 된다.

---

## ROS 2 인터페이스 정리

### 팔 (MoveIt)

| 타입 | 이름 | 설명 |
|------|------|------|
| Action | `/move_action` | MoveGroup 목표 전송 |
| Topic | `/joint_states` | 관절 상태 (100 Hz) |
| Service | `/controller_manager/...` | 컨트롤러 관리 |

### 그리퍼

| 타입 | 이름 | 설명 |
|------|------|------|
| Service | `/onrobot_rg2_ft_node/set_gripper` | width(mm) + force(N) 명령 |
| Topic | `/onrobot_rg2_ft_node/state` | 현재 상태 (5 Hz) |

**SetGripper 서비스 정의:**

```
float64 width    # mm [0.0 – 110.0]
float64 force    # N  [0.0 – 40.0]
---
bool    success
string  message
```

**GripperState 토픽 정의:**

```
builtin_interfaces/Time stamp
float64 actual_width    # 현재 열림 폭 (mm)
bool    busy            # 동작 중이면 True
bool    gripped         # 물체 감지 / 목표 힘 도달 시 True
```

---

## OnRobot RG2-FT V2 드라이버 상세

### 통신 구조

```
gripper_node.py
  │
  ├── command()  → Modbus TCP → 192.168.1.1:502 (Unit ID 65)
  │                FC16 write regs 2–4: [force_01N, width_01mm, control=1]
  │                  reg 2 = target force  [0.1 N  단위, 0–400  → 0–40 N  ]
  │                  reg 3 = target width  [0.1 mm 단위, 0–1100 → 0–110 mm]
  │                  reg 4 = control       [1 쓰면 실행]
  │
  └── get_state() → Socket.IO → 192.168.1.1/socket.io (EIO=4, HTTP long-polling)
                   event "message" → devices[0].variable.backpack
                     width (float) → actual_width mm
                     grip  (bool)  → gripped
                     ready (bool)  → not busy
```

**왜 Modbus TCP인가:**  
Compute Box는 HTTP REST(`/api/dc/rg2ft/set_width/{t}/{e}`)도 제공하지만 이는
웹 GUI 시퀀스 트리거로, 위치 제어가 아니다. 파라미터와 무관하게 비결정적인 위치로
이동한다. 정확한 위치 제어는 Modbus TCP(포트 502) 레지스터 쓰기가 올바른 인터페이스다.  
`pymodbus` 등 외부 라이브러리 불필요 — Python stdlib `socket` + `struct`만 사용.

### 노드 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `host` | `192.168.1.1` | Compute Box IP |
| `timeout_sec` | `3.0` | Modbus/HTTP 타임아웃 |
| `state_hz` | `5.0` | 상태 토픽 발행 주기 |

---

## 아키텍처

```
┌──────────────────────────────────────────────────────────────────┐
│                      doosan_isaac container                      │
│                                                                  │
│  MoveIt / test scripts  →  /move_action                          │
│    │                                                             │
│    ▼                                                             │
│  move_group (MoveIt OMPL)                                        │
│    │ IK + 경로계획 + 시간 파라미터화                              │
│    ▼                                                             │
│  dsr_moveit_controller (JTC, 100 Hz)                             │
│    │                                                             │
│    ▼  [실물]                    ▼  [시뮬]                        │
│  dsr_hardware2               mock_components/GenericSystem       │
│  (servoj_rt 스트리밍)         (명령 → 상태 즉시 반영)            │
│    │                                │                           │
│    └──────────┬─────────────────────┘                           │
│               ▼                                                  │
│         joint_state_broadcaster  →  /joint_states (100 Hz)      │
│               │                                                  │
│               ├─► robot_state_publisher → /tf                    │
│               └─► Isaac Sim OmniGraph (시뮬 전용)                │
│                       IsaacArticulationController                │
│                       M1013 PhysX 아티큘레이션                   │
│                                                                  │
│  onrobot_rg2_ft_node                                             │
│    ├── Modbus TCP → 192.168.1.1:502  (명령)                      │
│    ├── Socket.IO  → 192.168.1.1      (상태 5 Hz)                 │
│    ├── Service  ~/set_gripper                                    │
│    └── Topic    ~/state                                          │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 리포지토리 구조

```
doosan_docker_skeleton/
├── docker/
│   ├── Dockerfile              # Isaac Sim 4.5 + ROS 2 Humble + MoveIt
│   ├── bootstrap_ws.sh         # 최초 실행: doosan-robot2 클론 + colcon build
│   ├── entrypoint.sh
│   ├── container.sh            # {start|enter|stop|clean}
│   ├── docker_build.sh
│   └── run_emulator.sh         # 프로토콜 테스트용 (시뮬에서는 미사용)
├── isaac/
│   ├── m1013_ros2_bridge.py             # Isaac Sim 브리지: combined URDF 로드 + 팔/그리퍼 동시 제어
│   ├── m1013_rg2ft_combined.urdf        # pre-generated combined URDF (M1013 + RG2-FT)
│   ├── gripper_meshes/                  # RG2-FT STL 메시 (repo에 번들, 외부 clone 불필요)
│   │   ├── visual/    (9 STL)
│   │   └── collision/ (9 STL)
│   ├── gripper_standalone_test.py       # 그리퍼 단독 테스트 (Phase 1 검증용)
│   └── m1013_gripper_test.py            # 팔+그리퍼 통합 테스트 (Phase 2 검증용)
├── scripts/
│   ├── real_bringup.launch.py           # 실물: 팔(Doosan) + 그리퍼 동시 기동
│   ├── m1013_sim_bringup.launch.py      # 시뮬: MoveIt + mock_components + gripper sim
│   ├── generate_combined_urdf.sh        # combined URDF 재생성 스크립트
│   ├── gripper_sim_node.py              # 시뮬 그리퍼 mock 노드 (실물과 동일 인터페이스)
│   ├── dsr_moveit_controller_sim.yaml   # JTC 명령 인터페이스 오버라이드
│   ├── moveit_backend_smoketest.py      # 관절 공간 목표 테스트
│   ├── moveit_pose_test.py              # Cartesian / Joint 목표 테스트
│   ├── trajectory_recorder.py           # 궤적 기록
│   ├── trajectory_compare.py            # 시뮬-실물 궤적 비교
│   └── onrobot_rg2_ft/                  # ROS 2 그리퍼 드라이버 패키지
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── msg/GripperState.msg
│       ├── srv/SetGripper.srv
│       └── onrobot_rg2_ft/
│           ├── gripper_node.py          # ROS 2 노드 (서비스 + 토픽)
│           └── compute_box_client.py   # Modbus TCP + Socket.IO 클라이언트
├── third_party/                 # 런타임 전용, git-ignore (bootstrap이 클론)
└── README.md
```

---

## 시뮬-실물 궤적 비교

동일한 목표에 대해 시뮬과 실물의 궤적이 얼마나 일치하는지 정량 검증.

```bash
# 시뮬에서 기록
python3 /ros2_ws/src/trajectory_recorder.py -o sim_traj.csv
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.0 0.55 --cartesian

# 실물에서 기록 (real_bringup 실행 후)
python3 /ros2_ws/src/trajectory_recorder.py -o real_traj.csv
python3 /ros2_ws/src/moveit_pose_test.py --xyz -0.45 0.0 0.55 --cartesian

# 비교
python3 trajectory_compare.py sim_traj.csv real_traj.csv \
    --threshold-deg 2.0 --plot comparison.png
```

출력: 관절별 max/mean/RMSE 오차(도), PASS/FAIL 판정, 12패널 오버레이 플롯.

---

## 트러블슈팅

### 네트워크

| 증상 | 원인 | 해결 |
|------|------|------|
| `ping 192.168.137.100` 실패 | 호스트 라우트 없음 | 전체 설치 가이드 2단계 (nmcli) 재실행 |
| `ping 192.168.1.1` 실패 | Compute Box 라우트 없음 | `nmcli connection modify "Wired connection 1" +ipv4.addresses 192.168.1.100/24` |
| 재부팅 후 연결 끊김 | `ip addr add`로 임시 설정했던 경우 | nmcli로 영구 등록 필요 (전체 설치 가이드 2단계) |

### 그리퍼

| 증상 | 원인 | 해결 |
|------|------|------|
| `actual_width` 가 0.0 고정 | Socket.IO 연결 실패 | `curl http://192.168.1.1/` 로 Compute Box 응답 확인 |
| `set_gripper` → `Compute Box unreachable` | Modbus TCP 포트 502 차단 | `python3 -c "import socket; socket.create_connection(('192.168.1.1',502),3); print('OK')"` |
| 명령은 성공하지만 그리퍼가 엉뚱한 위치로 이동 | 구 버전 HTTP REST 코드 남아있음 | `rm -rf /ros2_ws/build/onrobot_rg2_ft && colcon build --symlink-install --packages-select onrobot_rg2_ft` |

### MoveIt / 팔

| 증상 | 원인 | 해결 |
|------|------|------|
| `PLANNING_FAILED` / `NO_IK_SOLUTION` | 목표 포즈 도달 불가 | `--position-only` 추가 또는 xyz 조정 |
| 컨트롤러 `inactive` | 아직 초기화 중 | `You can start planning now!` 대기 |
| `dsr_moveit_controller` Failed to activate | velocity 인터페이스 충돌 (시뮬) | `dsr_moveit_controller_sim.yaml` 이 launch에 포함되어 있는지 확인 |

### 빌드

| 증상 | 원인 | 해결 |
|------|------|------|
| `add_custom_target ... already exists` | `ament_cmake_python` + `rosidl` 충돌 | `CMakeLists.txt`에서 `ament_cmake_python` / `ament_python_install_package` 제거 |
| `ModuleNotFoundError: compute_box_client` | 모듈 경로 미등록 | `gripper_node.py` 상단에 `sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))` 확인 |

### Isaac Sim (시뮬 전용)

| 증상 | 원인 | 해결 |
|------|------|------|
| 창이 열리지 않음 | X11 포워딩 없음 | `xhost +local:docker` 실행 후 재시도 |
| `did not match any rigid bodies` | ArticulationRootAPI 경로 불일치 | 브리지 스크립트 stage dump 확인 후 경로 수정 |
| 비홈 포즈에서 시작 | mock_components가 이전 상태 유지 | `moveit_backend_smoketest.py` 로 홈 이동 후 시작 |

---

## 설계 결정 메모

**DRCF 에뮬레이터 제거 이유:**  
`dsr_hardware2` 가상 모드에서 `write()` 틱마다 `Drfl.amovej()` 호출 → JTC가 초당 100개의 위치 setpoint를 보낼 때 매번 가감속 프로파일이 재시작되어 눈에 띄는 jerky 움직임 발생. DRCF 에뮬레이터는 물리 시뮬레이터가 아닌 프로토콜/상태 머신 하네스.

**rviz2 stub 이유:**  
`ros-humble-rviz2`의 의존성 `libfreetype6-dev`가 Isaac Sim 베이스 이미지의 `libfreetype6`와 충돌. Doosan 공식 launch가 rviz2 노드를 무조건 spawn하므로 패키지가 없으면 move_group까지 크래시. No-op stub으로 해결.

**HTTP REST 대신 Modbus TCP:**  
Compute Box HTTP API(`/api/dc/rg2ft/set_width/`)는 웹 GUI 시퀀스 트리거로, 실제 위치 제어가 아님. Modbus TCP 레지스터 쓰기(포트 502, Unit ID 65)가 정확한 위치 + 힘 제어 인터페이스.
