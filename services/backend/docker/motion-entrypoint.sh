#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/ros_ws/install/setup.bash
export AMENT_PREFIX_PATH="/opt/ros_ws/install/stararm_102_motion_node:${AMENT_PREFIX_PATH}"
exec "$@"
