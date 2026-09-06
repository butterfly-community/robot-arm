#!/usr/bin/env bash
set -e
source /opt/ros/lyrical/setup.bash
source /opt/moveit_ws/install/setup.bash
source /opt/devices/stararm-102/ros_ws/install/setup.bash
export AMENT_PREFIX_PATH="/opt/devices/stararm-102/ros_ws/install/stararm_102_motion_node:${AMENT_PREFIX_PATH}"
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
Xtigervnc :1 -geometry 1600x1000 -depth 24 -SecurityTypes None \
  -localhost yes -rfbport 5901 -AcceptSetDesktopSize=1 &
display_pid=$!
while [[ ! -S /tmp/.X11-unix/X1 ]]; do
  kill -0 "$display_pid"
  sleep 0.05
done
websockify --web=/usr/share/novnc 0.0.0.0:6080 localhost:5901 &
web_pid=$!
openbox &
window_manager_pid=$!
ros2 launch stararm_102_motion_node motion_stack.launch.py &
ros_pid=$!
/usr/local/bin/dora-entrypoint.sh "$@" &
daemon_pid=$!
trap 'kill -INT "$display_pid" "$web_pid" "$window_manager_pid" "$ros_pid" "$daemon_pid" 2>/dev/null || true' INT TERM EXIT
wait -n "$display_pid" "$web_pid" "$window_manager_pid" "$ros_pid" "$daemon_pid"
