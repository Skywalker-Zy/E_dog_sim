#!/usr/bin/env bash
set -euo pipefail
room_scene_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$room_scene_dir/venv.sh"
edog_use_venv "$room_scene_dir"
exec "$EDOG_PYTHON" "$room_scene_dir/room_viewer.py" "$@"
