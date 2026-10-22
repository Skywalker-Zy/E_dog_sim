#!/usr/bin/env bash
set -euo pipefail
camera_lab_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Source ROS before enabling nounset: ROS setup scripts may reference unset vars.
if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
export PYTHONPATH="$camera_lab_root${PYTHONPATH:+:$PYTHONPATH}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$camera_lab_root/runtime/ros_logs}"
mkdir -p -- "$ROS_LOG_DIR"
exec /usr/bin/python3 -m inspection_camera "$@"
