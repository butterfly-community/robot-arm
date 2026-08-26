#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/stararm_ws/install/setup.bash
set -u
exec ros2 launch stararm102_teleop_moveit arm.launch.py \
  ipc_path:=/ipc/moveit-servo.sock
