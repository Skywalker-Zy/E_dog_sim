#!/usr/bin/env bash
set -euo pipefail
robot_world_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$robot_world_root/camera_lab/run_robot_cameras.sh" --scene-dir "$robot_world_root" "$@"
