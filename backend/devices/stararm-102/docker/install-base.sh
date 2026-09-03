#!/usr/bin/env bash
set -e

device_root=/opt/devices/stararm-102
project_root="$device_root/project"
vendor_root="$device_root/vendor"
ros_workspace="$device_root/ros_ws"

git clone "$1" "$vendor_root"
git -C "$vendor_root" checkout "$2"
python3 "$project_root/tools/verify-model.py" "$vendor_root" vendor
for patch in model dynamics topic-io; do
  git -C "$vendor_root" apply --ignore-space-change --ignore-whitespace \
    "$project_root/patches/$patch.patch"
done
python3 "$project_root/tools/verify-model.py" "$vendor_root" patched

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

mkdir -p /grippers/x_grippers
/opt/compute-venv/bin/python /opt/tools/graspgenx/build-description.py \
  "$project_root/graspgenx/manifest.json" \
  "$vendor_root" /grippers/x_grippers \
  --graspgenx-root /opt/graspgenx

install -m 644 "$project_root/docker/rviz-index.html" /usr/share/novnc/index.html
