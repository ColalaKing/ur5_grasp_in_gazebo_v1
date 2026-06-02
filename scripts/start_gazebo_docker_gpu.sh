#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

pick_docker() {
  local candidates=()
  if [ -n "${DOCKER_BIN:-}" ]; then
    candidates+=("$DOCKER_BIN")
  fi
  candidates+=("/usr/bin/docker" "/usr/local/bin/docker")
  if command -v docker >/dev/null 2>&1; then
    candidates+=("$(command -v docker)")
  fi

  local candidate version major
  for candidate in "${candidates[@]}"; do
    [ -x "$candidate" ] || continue
    version="$("$candidate" --version 2>/dev/null | sed -n 's/.*version \([0-9][0-9]*\).*/\1/p')"
    [ -n "$version" ] || continue
    major="$version"
    if [ "$major" -ge 25 ]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  return 1
}

DOCKER="$(pick_docker || true)"
if [ -z "$DOCKER" ]; then
  cat >&2 <<'EOF'
Docker CLI is too old for the running Docker daemon.
Please upgrade Docker client/compose, then run this script again:

  sudo apt-get update
  sudo apt-get install --only-upgrade docker-ce docker-ce-cli docker-compose-plugin

EOF
  exit 1
fi

if command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null || echo "Warning: xhost failed; check DISPLAY/X11 permissions if Gazebo GUI cannot open."
  xhost +local:docker >/dev/null || true
fi

if "$DOCKER" compose version >/dev/null 2>&1; then
  "$DOCKER" compose -f docker-compose.yml -f docker-compose.gpu.yml -f docker-compose.x11-gpu.yml up -d --build ur5_grasp
  "$DOCKER" compose -f docker-compose.yml -f docker-compose.gpu.yml -f docker-compose.x11-gpu.yml exec ur5_grasp bash
else
  docker-compose -f docker-compose.yml -f docker-compose.gpu.yml -f docker-compose.x11-gpu.yml up -d --build ur5_grasp
  docker-compose -f docker-compose.yml -f docker-compose.gpu.yml -f docker-compose.x11-gpu.yml exec ur5_grasp bash
fi
