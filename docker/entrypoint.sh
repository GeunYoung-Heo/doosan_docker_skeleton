#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
exec "$@"
