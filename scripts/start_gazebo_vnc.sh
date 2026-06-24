#!/usr/bin/env bash
set -euo pipefail

if [ ! -f /.dockerenv ]; then
  cat >&2 <<'EOF'
This script must run inside the Docker container.

From the host, start and enter the container first:

  ./scripts/start_gazebo_docker.sh

Then run inside the container:

  /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh

EOF
  exit 1
fi

DISPLAY_NUM="${GAZEBO_VNC_DISPLAY:-:99}"
GEOMETRY="${GAZEBO_VNC_GEOMETRY:-1280x800x24}"
VNC_PORT="${GAZEBO_VNC_PORT:-5900}"
VNC_LISTEN="${GAZEBO_VNC_LISTEN:-0.0.0.0}"
RANDOMIZE="${GAZEBO_RANDOMIZE:-true}"
SEED="${GAZEBO_SEED:-0}"
SPAWN_OBSTACLES="${GAZEBO_SPAWN_OBSTACLES:-false}"
PAUSED="${GAZEBO_PAUSED:-false}"
GUI="${GAZEBO_GUI:-false}"
VIRTUAL_WRIST_CAMERA="${GAZEBO_VIRTUAL_WRIST_CAMERA:-true}"
ENABLE_GAZEBO_WRIST_CAMERA="${GAZEBO_ENABLE_GAZEBO_WRIST_CAMERA:-false}"
AUTO_IMAGE_VIEWER="${GAZEBO_AUTO_IMAGE_VIEWER:-true}"
ALLOW_UNSTABLE_REAL_CAMERA="${GAZEBO_ALLOW_UNSTABLE_REAL_CAMERA:-false}"

if [ "$ENABLE_GAZEBO_WRIST_CAMERA" = "true" ] && [ "$ALLOW_UNSTABLE_REAL_CAMERA" != "true" ]; then
  cat >&2 <<'EOF'
Gazebo's real wrist camera is not supported on the VNC/Xvfb route by default.
It often crashes Gazebo 7 because the camera sensor uses the rendering stack.

Use one of these instead:

  1. Stable VNC route with virtual wrist camera:
     GAZEBO_VIRTUAL_WRIST_CAMERA=true /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh

  2. Real Gazebo wrist camera via the host GPU/X11 route:
     ./scripts/start_gazebo_docker_gpu.sh
     roslaunch ur5_notebook assignment_env.launch virtual_wrist_camera:=false enable_gazebo_wrist_camera:=true gui:=true

If you still want to try the unstable VNC real-camera route, set:

  GAZEBO_ALLOW_UNSTABLE_REAL_CAMERA=true

EOF
  exit 1
fi

export DISPLAY="$DISPLAY_NUM"
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-2.1}"
export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-120}"
export OGRE_RTT_MODE="${OGRE_RTT_MODE:-Copy}"
export QT_X11_NO_MITSHM=1
export GAZEBO_MODEL_PATH="/usr/share/gazebo-7/models:/usr/share/gazebo/models:/root/.gazebo/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-7:/usr/share/gazebo:${GAZEBO_RESOURCE_PATH:-}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
unset ROS_HOSTNAME

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

if ! pgrep -x Xvfb >/dev/null 2>&1; then
  Xvfb "$DISPLAY_NUM" -screen 0 "$GEOMETRY" +extension GLX +render -noreset &
  sleep 2
fi

if ! pgrep -x fluxbox >/dev/null 2>&1; then
  fluxbox >/tmp/fluxbox-gazebo.log 2>&1 &
fi

if ! pgrep -x x11vnc >/dev/null 2>&1; then
  x11vnc -display "$DISPLAY_NUM" -forever -shared -nopw -noxdamage -repeat \
    -listen "$VNC_LISTEN" -rfbport "$VNC_PORT" > /tmp/x11vnc-gazebo.log 2>&1 &
fi

echo "Gazebo VNC display is running on container port $VNC_PORT."
echo "Docker Compose maps it to host port 5901 by default: vncviewer localhost:5901"
echo "Gazebo GUI: $GUI, virtual wrist camera: $VIRTUAL_WRIST_CAMERA, Gazebo wrist camera: $ENABLE_GAZEBO_WRIST_CAMERA"
echo "To view images in another container shell: export DISPLAY=$DISPLAY_NUM && rqt_image_view /ur5/wrist_camera/image_raw"

if [ "$AUTO_IMAGE_VIEWER" = "true" ] && ! pgrep -x rqt_image_view >/dev/null 2>&1; then
  (
    sleep 10
    rqt_image_view /ur5/wrist_camera/image_raw >/tmp/rqt-image-view.log 2>&1
  ) &
fi

roslaunch ur5_notebook assignment_env.launch \
  gui:="$GUI" \
  paused:="$PAUSED" \
  randomize:="$RANDOMIZE" \
  seed:="$SEED" \
  spawn_obstacles:="$SPAWN_OBSTACLES" \
  virtual_wrist_camera:="$VIRTUAL_WRIST_CAMERA" \
  enable_gazebo_wrist_camera:="$ENABLE_GAZEBO_WRIST_CAMERA"
