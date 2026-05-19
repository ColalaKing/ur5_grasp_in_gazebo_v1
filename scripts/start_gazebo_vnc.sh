#!/usr/bin/env bash
set -euo pipefail

DISPLAY_NUM="${GAZEBO_VNC_DISPLAY:-:99}"
GEOMETRY="${GAZEBO_VNC_GEOMETRY:-1280x800x24}"
VNC_PORT="${GAZEBO_VNC_PORT:-5900}"
VNC_LISTEN="${GAZEBO_VNC_LISTEN:-127.0.0.1}"
RANDOMIZE="${GAZEBO_RANDOMIZE:-true}"
SEED="${GAZEBO_SEED:-0}"
SPAWN_OBSTACLES="${GAZEBO_SPAWN_OBSTACLES:-false}"
PAUSED="${GAZEBO_PAUSED:-false}"

export DISPLAY="$DISPLAY_NUM"
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export GAZEBO_MODEL_PATH="/usr/share/gazebo-7/models:/usr/share/gazebo/models:/root/.gazebo/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-7:/usr/share/gazebo:${GAZEBO_RESOURCE_PATH:-}"

source /opt/ros/kinetic/setup.bash

cd /root/arm_ws
if [ ! -f /root/arm_ws/devel/setup.bash ]; then
  catkin_make
fi
source /root/arm_ws/devel/setup.bash

# The GPU image uses NVIDIA GLVND libraries by default.  VNC runs on Xvfb and
# must use Mesa/llvmpipe instead, otherwise gzclient exits with GLX fbconfig
# errors and the viewer only shows an empty desktop.
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/mesa:${LD_LIBRARY_PATH:-}"

if ! pgrep -f "Xvfb $DISPLAY_NUM" >/dev/null 2>&1; then
  Xvfb "$DISPLAY_NUM" -screen 0 "$GEOMETRY" +extension GLX +render -noreset &
  sleep 2
fi

if ! pgrep -x fluxbox >/dev/null 2>&1; then
  fluxbox >/tmp/fluxbox-gazebo.log 2>&1 &
fi

if ! pgrep -f "x11vnc.*$DISPLAY_NUM" >/dev/null 2>&1; then
  x11vnc -display "$DISPLAY_NUM" -forever -shared -nopw -listen "$VNC_LISTEN" -rfbport "$VNC_PORT" >/tmp/x11vnc-gazebo.log 2>&1 &
fi

echo "Gazebo VNC display is running on port $VNC_PORT."
echo "Connect from the host with: vncviewer localhost:$VNC_PORT"

roslaunch ur5_notebook assignment_env.launch \
  gui:=true \
  paused:="$PAUSED" \
  randomize:="$RANDOMIZE" \
  seed:="$SEED" \
  spawn_obstacles:="$SPAWN_OBSTACLES"
