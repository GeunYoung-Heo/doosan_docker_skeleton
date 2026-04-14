#!/usr/bin/env bash
# Standalone image build. container.sh start does this automatically;
# this script exists for parity with the lab's docker_build.sh.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="doosan-isaac:latest"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile"

docker build -t "${IMAGE}" -f "${DOCKERFILE}" "${SCRIPT_DIR}"
echo "[OK] built: ${IMAGE}"
