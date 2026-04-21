#!/usr/bin/env bash
# Generate combined M1013 + OnRobot RG2-FT URDF.
#
# This script merges the M1013 arm URDF with the RG2-FT gripper URDF
# into a single file so Isaac Sim can import them as one articulation.
#
# Modifications applied:
#   - M1013: package:// → file:// absolute paths (for Isaac Sim URDF importer)
#   - Gripper: package:// → file:// absolute paths
#   - Gripper: <mimic> tags removed (Isaac Sim handles mimic via Python)
#   - Gripper: <world> link removed (not needed when attached to arm)
#   - Gripper: quick_changer_joint parent changed from "world" → "link_6"
#   - Gripper: quick_changer_joint origin set to (0,0,0) (flush mount)
#   - Gripper: <gazebo> and <transmission> blocks removed
#
# Usage (inside the doosan_isaac container):
#   bash /ros2_ws/src/generate_combined_urdf.sh
#
# Output: /ros2_ws/src/m1013_rg2ft_combined.urdf
#   (also copied to isaac/ for the bridge script's default path)
#
# To customize (e.g. add yaw rotation at mount point):
#   Edit the quick_changer_joint <origin> line below.
set -euo pipefail

DESC_DIR=/ros2_ws/src/third_party/doosan-robot2/dsr_description2
GRIP_DIR=/ros2_ws/src/third_party/OnRobot-RG2FT-ROS/onrobot_rg2ft_description
GRIP_URDF_RAW=${GRIP_DIR}/urdf/onrobot_rg2ft.xacro

OUT=/ros2_ws/src/m1013_rg2ft_combined.urdf

echo "[generate] M1013 URDF: ${DESC_DIR}/urdf/m1013.urdf"
echo "[generate] Gripper xacro: ${GRIP_URDF_RAW}"

# ---- Step 1: Resolve gripper xacro → URDF (needs ament_index registration) ----
# Ensure onrobot_rg2ft_description is findable by xacro
if ! ros2 pkg prefix onrobot_rg2ft_description >/dev/null 2>&1; then
    echo "[generate] registering onrobot_rg2ft_description in ament_index..."
    mkdir -p /opt/ros/humble/share/ament_index/resource_index/packages
    touch /opt/ros/humble/share/ament_index/resource_index/packages/onrobot_rg2ft_description
    ln -sf ${GRIP_DIR} /opt/ros/humble/share/onrobot_rg2ft_description
fi

GRIP_URDF_RESOLVED=/tmp/_gripper_resolved.urdf
xacro ${GRIP_URDF_RAW} > ${GRIP_URDF_RESOLVED}
echo "[generate] gripper xacro resolved ($(wc -l < ${GRIP_URDF_RESOLVED}) lines)"

# ---- Step 2: Remove mimic tags from gripper URDF ----
GRIP_URDF_NO_MIMIC=/tmp/_gripper_no_mimic.urdf
sed -e '/<mimic /d' \
    -e '/<plugin.*mimic/,/<\/plugin>/d' \
    ${GRIP_URDF_RESOLVED} > ${GRIP_URDF_NO_MIMIC}

# ---- Step 3: Resolve mesh paths to absolute file:// ----
# Also fix gripper: package:// → file://
sed -i "s|package://onrobot_rg2ft_description/|file://${GRIP_DIR}/|g" ${GRIP_URDF_NO_MIMIC}

# ---- Step 4: M1013 URDF — resolve mesh paths, strip closing </robot> ----
sed "s|package://dsr_description2/|file://${DESC_DIR}/|g" \
    ${DESC_DIR}/urdf/m1013.urdf | head -n -1 > /tmp/_arm_part.urdf

# ---- Step 5: Gripper URDF — extract body content ----
# Remove: XML header, <robot> tags, <link name="world"/>, <gazebo>, <transmission>
# Change: quick_changer_joint parent from "world" to "link_6"
# Change: quick_changer_joint origin to (0, 0, 0) for flush mount
#   ↑ CUSTOMIZE HERE if you need rotation: e.g. rpy="0 0 1.5708" for 90° yaw
MOUNT_ORIGIN='<origin rpy="0 0 0" xyz="0 0 0"/>'

grep -v '<?xml' ${GRIP_URDF_NO_MIMIC} \
    | grep -v '<robot ' \
    | grep -v '</robot>' \
    | grep -v '<link name="world"/>' \
    | sed '/<gazebo>/,/<\/gazebo>/d' \
    | sed '/<transmission/,/<\/transmission>/d' \
    | sed "s|<parent link=\"world\"/>|<parent link=\"link_6\"/>|g" \
    | sed "s|<origin rpy=\"0 0 0\" xyz=\"0 0 0.1\"/>|${MOUNT_ORIGIN}|g" \
    > /tmp/_gripper_part.urdf

# ---- Step 6: Combine ----
{
    cat /tmp/_arm_part.urdf
    echo ""
    echo "  <!-- ====== OnRobot RG2-FT V2 Gripper (attached to link_6) ====== -->"
    cat /tmp/_gripper_part.urdf
    echo "</robot>"
} > ${OUT}

# Also copy to isaac/ for the bridge's default path
cp ${OUT} /workspace/isaac/m1013_rg2ft_combined.urdf 2>/dev/null || true

echo "[generate] output: ${OUT}"
echo "[generate] joints: $(grep -c 'joint name' ${OUT}), links: $(grep -c 'link name' ${OUT})"
echo "[generate] done."
