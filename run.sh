#!/usr/bin/env bash
set -euo pipefail
scene_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$scene_root/scripts/venv.sh"
edog_source_ros "$@"
edog_use_venv "$scene_root"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$scene_root/runtime/ros_logs}"
mkdir -p -- "$ROS_LOG_DIR"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
exec "$EDOG_PYTHON" "$scene_root/viewer.py" "$@"
