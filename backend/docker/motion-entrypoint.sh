#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/ros_ws/install/setup.bash
export AMENT_PREFIX_PATH="/opt/ros_ws/install/stararm_102_motion_node:${AMENT_PREFIX_PATH}"
ros2 launch stararm_102_motion_node motion_stack.launch.py &
ros_pid=$!
"$@" &
daemon_pid=$!
trap 'kill -INT "$ros_pid" "$daemon_pid" 2>/dev/null || true' INT TERM EXIT
wait -n "$ros_pid" "$daemon_pid"
