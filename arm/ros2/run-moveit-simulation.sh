#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(realpath "${script_dir}/../..")"
develop_root="$(realpath "${repo_root}/../../../..")"
vendor_root="${STAR_ARM_102_SOURCE:-${develop_root}/temp/Star-Arm-102}"
state_dir="${repo_root}/vr-xr/state"
image="moveit/moveit2:jazzy-release"

if [[ ! -d "${vendor_root}/.git" ]]; then
  echo "缺少厂家源码：${vendor_root}" >&2
  echo "请将 https://github.com/servodevelop/Star-Arm-102 克隆到 ~/Develop/temp/Star-Arm-102，或设置 STAR_ARM_102_SOURCE。" >&2
  exit 1
fi

mkdir -p "${state_dir}"

exec docker run --rm --network host \
  --name stararm102-moveit-simulation \
  --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  --env ROS_DOMAIN_ID=42 \
  --volume "${vendor_root}:/vendor:ro" \
  --volume "${repo_root}/arm/patches:/project-patches:ro" \
  --volume "${script_dir}/stararm102_teleop_moveit:/project-package:ro" \
  --volume "${state_dir}:/ipc" \
  "${image}" \
  bash -lc '
    set -eo pipefail
    mkdir -p /tmp/vendor/ROS2_HUMBLE/src /tmp/ws/src
    cp -a /vendor/ROS2_HUMBLE/src/stararm102_description /tmp/vendor/ROS2_HUMBLE/src/
    cp -a /vendor/ROS2_HUMBLE/src/stararm102_moveit_config /tmp/vendor/ROS2_HUMBLE/src/
    cd /tmp/vendor
    git apply --ignore-space-change --ignore-whitespace \
      /project-patches/star-arm-102-fl-moveit-model.patch \
      /project-patches/star-arm-102-fl-moveit-dynamics.patch
    cp -a /tmp/vendor/ROS2_HUMBLE/src/stararm102_description /tmp/ws/src/
    cp -a /tmp/vendor/ROS2_HUMBLE/src/stararm102_moveit_config /tmp/ws/src/
    cp -a /project-package /tmp/ws/src/stararm102_teleop_moveit
    source /opt/ros/jazzy/setup.bash
    cd /tmp/ws
    colcon build --packages-select \
      stararm102_description stararm102_moveit_config stararm102_teleop_moveit \
      --event-handlers console_direct+
    source /tmp/ws/install/setup.bash
    set -u
    exec ros2 launch stararm102_teleop_moveit simulation.launch.py \
      ipc_path:=/ipc/moveit-servo.sock
  '
