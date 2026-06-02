#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [ -f /.dockerenv ]; then
  echo "Run this script on the host, not inside the container." >&2
  exit 1
fi

if [ -z "${DISPLAY:-}" ]; then
  first_socket="$(find /tmp/.X11-unix -maxdepth 1 -type s -name 'X*' 2>/dev/null | sort | head -n 1 || true)"
  if [ -z "$first_socket" ]; then
    echo "DISPLAY is empty and no /tmp/.X11-unix/X* socket was found." >&2
    exit 1
  fi
  export DISPLAY=":${first_socket##*/X}"
fi

display_number="${DISPLAY#:}"
display_number="${display_number%%.*}"
socket="/tmp/.X11-unix/X${display_number}"
if [ ! -S "$socket" ]; then
  echo "Expected X11 socket '$socket' for DISPLAY='$DISPLAY', but it does not exist." >&2
  echo "Run this from the graphical desktop session, or set DISPLAY to the active X display." >&2
  exit 1
fi

if command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null
  xhost +local:docker >/dev/null
else
  echo "xhost was not found; X11 permissions may block the container." >&2
fi

docker compose \
  -f docker-compose.yml \
  -f docker-compose.gpu.yml \
  -f docker-compose.x11-gpu.yml \
  down

DISPLAY="$DISPLAY" docker compose \
  -f docker-compose.yml \
  -f docker-compose.gpu.yml \
  -f docker-compose.x11-gpu.yml \
  up -d --force-recreate

docker exec ur5_grasp_kinetic bash -lc '
  set -e
  echo "DISPLAY=$DISPLAY"
  ls -l /tmp/.X11-unix
  nvidia-smi
  unset LIBGL_ALWAYS_SOFTWARE
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  glxinfo -B
'
