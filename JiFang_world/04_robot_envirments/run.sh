#!/usr/bin/env bash
set -euo pipefail
scene_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$scene_root/../shared/scripts/venv.sh"
edog_use_venv "$scene_root"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
exec "$EDOG_PYTHON" "$scene_root/viewer.py" "$@"
