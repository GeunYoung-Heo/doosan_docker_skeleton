#!/usr/bin/env bash
# First-run bootstrap: clone doosan-robot2 (humble) and colcon build the workspace.
# Re-runs are idempotent — guarded by the .built_doosan_${ROS_DISTRO} marker.
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
WS="${WS:-/ros2_ws}"
THIRD="${WS}/src/third_party"
DOOSAN_DIR="${THIRD}/doosan-robot2"
MARK="${WS}/install/.built_doosan_${ROS_DISTRO}"

# Pin to a known-good commit so the lab PC build matches the personal PC.
# Update this when you intentionally move to a newer upstream snapshot.
DOOSAN_COMMIT="ec9242546ec6202835900dbcd8498e2daabfa6a6"
DOOSAN_REMOTE="https://github.com/DoosanRobotics/doosan-robot2.git"

mkdir -p "${WS}/src" "${WS}/build" "${WS}/install" "${WS}/log" "${THIRD}"

if [ ! -d "${DOOSAN_DIR}" ]; then
  echo "[bootstrap] doosan-robot2 not found -> cloning humble branch"
  git clone -b humble "${DOOSAN_REMOTE}" "${DOOSAN_DIR}"
fi

git config --global --add safe.directory "${DOOSAN_DIR}" >/dev/null 2>&1 || true

# Checkout the pinned commit if the working tree is on a different one.
CURRENT_COMMIT="$(git -C "${DOOSAN_DIR}" rev-parse HEAD 2>/dev/null || echo unknown)"
if [ "${CURRENT_COMMIT}" != "${DOOSAN_COMMIT}" ]; then
  echo "[bootstrap] checking out pinned commit ${DOOSAN_COMMIT} (was ${CURRENT_COMMIT})"
  git -C "${DOOSAN_DIR}" fetch --depth 1 origin "${DOOSAN_COMMIT}" || true
  git -C "${DOOSAN_DIR}" checkout -q "${DOOSAN_COMMIT}"
fi

if [ -f "${MARK}" ] && [ -f "${WS}/install/setup.bash" ]; then
  echo "[bootstrap] already built -> skip"
  exit 0
fi

# ROS setup.bash trips set -u on AMENT_TRACE_SETUP_FILES
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

apt-get update || true
rosdep update
cd "${WS}"
rosdep install -r --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -y || true

colcon build --symlink-install --event-handlers console_direct+

touch "${MARK}"
echo "[bootstrap] build done."
