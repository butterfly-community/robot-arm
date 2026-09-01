#!/usr/bin/env bash
set -e
source /opt/ros/lyrical/setup.bash
source /opt/cv_bridge_ws/install/setup.bash
source /opt/realsense_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_sync:=true &
camera_pid=$!
trap 'kill "$camera_pid" 2>/dev/null || true' EXIT
exec /src/docker/dora-entrypoint.sh "$@"
