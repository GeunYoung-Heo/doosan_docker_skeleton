#!/usr/bin/env bash
# Doosan + Isaac Sim 4.5 + ROS 2 Humble unified container manager.
# Usage: container.sh {start|enter|stop|clean}
#
# Environment overrides (optional):
#   DRCF_HOST       (default 127.0.0.1)  — DRCF emulator on host or real robot IP
#   DRCF_PORT       (default 12345)
#   ROS_DOMAIN_ID   (default 0)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

WS="/ros2_ws"
SRC="${WS}/src"

IMAGE="doosan-isaac:latest"
CONTAINER="doosan_isaac"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile"

SCRIPTS_HOST="${ROOT}/scripts"
THIRD_HOST="${ROOT}/third_party"
ISAAC_HOST="${ROOT}/isaac"
CAP_HOST="${DOOSAN_CAP_PATH:-${HOME}/doosan-cap}"

CACHE_HOST="${SCRIPT_DIR}/cache"
BUILD_HOST="${CACHE_HOST}/build"
INSTALL_HOST="${CACHE_HOST}/install"
LOG_HOST="${CACHE_HOST}/log"
ISAAC_KIT_CACHE="${CACHE_HOST}/isaac-kit"
ISAAC_OV_CACHE="${CACHE_HOST}/isaac-ov"

DRCF_HOST="${DRCF_HOST:-127.0.0.1}"
DRCF_PORT="${DRCF_PORT:-12345}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

usage() { echo "Usage: $0 {start|enter|stop|clean}"; }

ensure_dirs() {
  mkdir -p "${SCRIPTS_HOST}" "${THIRD_HOST}" "${ISAAC_HOST}" \
           "${BUILD_HOST}" "${INSTALL_HOST}" "${LOG_HOST}" \
           "${ISAAC_KIT_CACHE}" "${ISAAC_OV_CACHE}"
}

build_image() {
  echo "[container] building image: ${IMAGE}"
  docker build -t "${IMAGE}" -f "${DOCKERFILE}" "${SCRIPT_DIR}"
}

create_or_start_container() {
  if docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
    docker start "${CONTAINER}" >/dev/null || true
    return
  fi

  echo "[container] creating container: ${CONTAINER}"
  xhost +local:docker >/dev/null 2>&1 || true

  docker run -d \
    --name "${CONTAINER}" \
    --network host \
    --ipc host \
    --privileged \
    --gpus all \
    -e "DISPLAY=${DISPLAY:-:0}" \
    -e "QT_X11_NO_MITSHM=1" \
    -e "ACCEPT_EULA=Y" \
    -e "PRIVACY_CONSENT=Y" \
    -e "DRCF_HOST=${DRCF_HOST}" \
    -e "DRCF_PORT=${DRCF_PORT}" \
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device /dev/dri \
    -v "${SCRIPTS_HOST}:${SRC}:rw" \
    -v "${THIRD_HOST}:${SRC}/third_party:rw" \
    -v "${ISAAC_HOST}:/workspace/isaac:rw" \
    -v "${CAP_HOST}:/workspace/doosan-cap:rw" \
    -v "${BUILD_HOST}:${WS}/build:rw" \
    -v "${INSTALL_HOST}:${WS}/install:rw" \
    -v "${LOG_HOST}:${WS}/log:rw" \
    -v "${ISAAC_KIT_CACHE}:/isaac-sim/kit/cache:rw" \
    -v "${ISAAC_OV_CACHE}:/root/.cache/ov:rw" \
    "${IMAGE}" \
    tail -f /dev/null
}

bootstrap_once() {
  if ! docker exec "${CONTAINER}" test -x /usr/local/bin/bootstrap_ws.sh; then
    echo "[container] bootstrap_ws.sh missing in image -> rebuild + recreate"
    docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true
    build_image
    create_or_start_container
  fi

  if [ ! -f "${INSTALL_HOST}/.built_doosan_humble" ]; then
    echo "[container] bootstrap (first build, ~10–20 min)"
    docker exec -it "${CONTAINER}" bash -lc "/usr/local/bin/bootstrap_ws.sh"
  else
    echo "[container] bootstrap skipped (already built)"
  fi
}

enter_shell() {
  docker exec -it "${CONTAINER}" bash -lc \
    "source /opt/ros/humble/setup.bash; \
     [ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash; \
     exec bash"
}

stop_container() {
  docker stop "${CONTAINER}" >/dev/null 2>&1 || true
  echo "[container] stopped"
}

clean_all() {
  docker rm -f "${CONTAINER}" 2>/dev/null || true
  rm -rf "${CACHE_HOST}"
  echo "[container] removed container + cache (image kept)"
}

case "${1:-}" in
  start)
    ensure_dirs
    build_image
    create_or_start_container
    bootstrap_once
    echo "[OK] started: ${CONTAINER}"
    echo "[hint] enter with: $0 enter"
    ;;
  enter) enter_shell ;;
  stop)  stop_container ;;
  clean) clean_all ;;
  *)     usage; exit 1 ;;
esac
