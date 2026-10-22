#!/usr/bin/env bash
set -euo pipefail
experiment_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
world_root="$(dirname -- "$experiment_root")"
camera_lab_root="$world_root/shared/camera_lab"
source "$world_root/shared/scripts/venv.sh"
edog_source_ros "$@"
edog_use_venv "$experiment_root"
export PYTHONPATH="$camera_lab_root${PYTHONPATH:+:$PYTHONPATH}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$experiment_root/runtime/ros_logs}"
mkdir -p -- "$ROS_LOG_DIR"
exec "$EDOG_PYTHON" -m inspection_camera \
  --scene-dir "$world_root/01_single_cabinet" \
  --config "$experiment_root/config/default.json" \
  --output "$experiment_root/experiments" "$@"
