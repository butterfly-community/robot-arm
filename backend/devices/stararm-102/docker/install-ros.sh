#!/usr/bin/env bash
set -e

device_root=/opt/devices/stararm-102
project_root="$device_root/project"
vendor_root="$device_root/vendor"
ros_workspace="$device_root/ros_ws"

cp "$project_root/ros2/motion-node/config/stararm102_description.srdf" \
  "$vendor_root/ROS2_HUMBLE/src/stararm102_moveit_config/config/"
mkdir -p "$ros_workspace/src"
ln -s "$vendor_root/ROS2_HUMBLE/src/stararm102_description" \
  "$ros_workspace/src/stararm102_description"
ln -s "$vendor_root/ROS2_HUMBLE/src/stararm102_moveit_config" \
  "$ros_workspace/src/stararm102_moveit_config"
ln -s "$project_root/ros2/mtc" "$ros_workspace/src/stararm_102_mtc"
ln -s "$project_root/ros2/motion-node" "$ros_workspace/src/stararm_102_motion_node"

source /opt/ros/lyrical/setup.bash
source /opt/moveit_ws/install/setup.bash
cd "$ros_workspace"
colcon build --packages-select \
  stararm102_description stararm102_moveit_config \
  stararm_102_mtc stararm_102_motion_node \
  --cmake-args -DBUILD_TESTING=OFF
