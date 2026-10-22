#!/usr/bin/env bash
set -euo pipefail
robot_camera_lab_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
export PYTHONPATH="$robot_camera_lab_root${PYTHONPATH:+:$PYTHONPATH}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$robot_camera_lab_root/runtime/ros_logs}"
mkdir -p -- "$ROS_LOG_DIR"
exec /usr/bin/python3 -m inspection_camera.robot_app "$@"
