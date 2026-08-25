#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(realpath "${script_dir}/../..")"
develop_root="$(realpath "${repo_root}/../../../..")"
vendor_root="${STAR_ARM_102_SOURCE:-${develop_root}/temp/Star-Arm-102}"
state_dir="${repo_root}/vr-xr/state"
image="${STAR_ARM_102_HARDWARE_IMAGE:-stararm102-moveit-hardware:jazzy}"
requested_port="${STAR_ARM_102_PORT:-}"

if [[ -z "${requested_port}" ]]; then
  echo "必须设置 STAR_ARM_102_PORT，例如 /dev/ttyUSB0" >&2
  exit 1
fi
if [[ ! -e "${requested_port}" ]]; then
  echo "设备不存在：${requested_port}" >&2
  exit 1
fi
resolved_port="$(realpath "${requested_port}")"
if [[ ! -d "${vendor_root}/.git" ]]; then
  echo "缺少厂家源码：${vendor_root}" >&2
  exit 1
fi
if ! docker image inspect "${image}" >/dev/null 2>&1; then
  echo "缺少真机镜像 ${image}，先执行：" >&2
  echo "docker build -t ${image} -f arm/ros2/Dockerfile.hardware arm/ros2" >&2
  exit 1
fi

mkdir -p "${state_dir}"

exec docker run --rm --network host \
  --name stararm102-moveit-hardware \
  --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  --env ROS_DOMAIN_ID=43 \
  --device "${resolved_port}:/dev/stararm102" \
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
    # The hardware patch is intentionally based on the corrected FL model.
    # Apply each layer in order so Git evaluates every patch against the state
    # produced by the preceding layer.
    git apply --ignore-space-change --ignore-whitespace \
      /project-patches/star-arm-102-fl-moveit-model.patch
    git apply --ignore-space-change --ignore-whitespace \
      /project-patches/star-arm-102-fl-moveit-dynamics.patch
    git apply --ignore-space-change --ignore-whitespace \
      /project-patches/star-arm-102-fl-topic-hardware.patch
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
    exec ros2 launch stararm102_teleop_moveit hardware.launch.py \
      port:=/dev/stararm102 baudrate:=1000000 \
      ipc_path:=/ipc/moveit-servo-hardware.sock
  '
