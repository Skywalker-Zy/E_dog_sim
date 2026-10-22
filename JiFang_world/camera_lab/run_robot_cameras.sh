#!/usr/bin/env bash
set -euo pipefail
camera_lab_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$camera_lab_root/../scripts/venv.sh"
edog_source_ros "$@"
edog_use_venv "$camera_lab_root"
export PYTHONPATH="$camera_lab_root${PYTHONPATH:+:$PYTHONPATH}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$camera_lab_root/runtime/ros_logs}"
mkdir -p -- "$ROS_LOG_DIR"
exec "$EDOG_PYTHON" -m inspection_camera.robot_app "$@"
