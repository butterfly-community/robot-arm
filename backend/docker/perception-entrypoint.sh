#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_sync:=true &
camera_pid=$!
trap 'kill "$camera_pid" 2>/dev/null || true' EXIT
exec "$@"
