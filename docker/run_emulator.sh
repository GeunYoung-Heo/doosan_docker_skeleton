#!/usr/bin/env bash
# Host-side helper that launches the Doosan DRCF emulator container.
# Mirrors what dsr_common2/bin/run_drcf.sh does, but stripped down for our flow.
#
# Usage:
#   bash docker/run_emulator.sh           # default: m1013, port 12345
#   bash docker/run_emulator.sh m0609 12346
#
# Stop with:  docker rm -f emulator
set -euo pipefail

MODEL="${1:-m1013}"
PORT="${2:-12345}"
NAME="emulator"
IMAGE="doosanrobot/dsr_emulator:3.0.1"

if ! command -v docker >/dev/null 2>&1; then
  echo "[emulator] docker not found in PATH" >&2
  exit 1
fi

if docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
  echo "[emulator] '${NAME}' is already running"
  echo "[hint] stop with:  docker rm -f ${NAME}"
  exit 0
fi

if docker ps -a --format '{{.Names}}' | grep -qx "${NAME}"; then
  echo "[emulator] removing stopped '${NAME}'"
  docker rm -f "${NAME}" >/dev/null
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[emulator] pulling ${IMAGE}"
  docker pull "${IMAGE}"
fi

echo "[emulator] starting ${NAME}: model=${MODEL^^} port=${PORT}"
docker run -dit --rm \
  --name "${NAME}" \
  --env "ROBOT_MODEL=${MODEL^^}" \
  -p "${PORT}:12345" \
  "${IMAGE}" >/dev/null

sleep 1
echo "[emulator] running. logs:  docker logs -f ${NAME}"
echo "[emulator] verify port:    ss -tlnp | grep ${PORT}"
