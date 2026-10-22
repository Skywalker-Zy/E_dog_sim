#!/usr/bin/env bash
set -euo pipefail
jifang_world_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$jifang_world_root/camera_lab/run_camera.sh" --scene-dir "$jifang_world_root" "$@"
