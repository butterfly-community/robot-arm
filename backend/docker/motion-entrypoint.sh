#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/ros_ws/install/setup.bash
export AMENT_PREFIX_PATH="/opt/ros_ws/install/stararm_102_motion_node:${AMENT_PREFIX_PATH}"
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
Xkasmvnc :1 \
  -interface 0.0.0.0 -disableBasicAuth -SecurityTypes None \
  -publicIP 192.168.100.10 \
  -geometry 1600x1000 -depth 24 -AcceptSetDesktopSize 1 \
  -httpd /usr/share/kasmvnc/www -websocketPort 6080 -sslOnly 0 \
  -FreeKeyMappings -Log '*:stdout:30' &
display_pid=$!
while [[ ! -S /tmp/.X11-unix/X1 ]]; do
  kill -0 "$display_pid"
  sleep 0.05
done
openbox &
window_manager_pid=$!
ros2 launch stararm_102_motion_node motion_stack.launch.py &
ros_pid=$!
"$@" &
daemon_pid=$!
trap 'kill -INT "$display_pid" "$window_manager_pid" "$ros_pid" "$daemon_pid" 2>/dev/null || true' INT TERM EXIT
wait -n "$display_pid" "$window_manager_pid" "$ros_pid" "$daemon_pid"
