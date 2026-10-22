#!/usr/bin/env bash
set -euo pipefail
room_scene_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Use system Qt/Pillow when available; MUJOCO_PYTHON_SITE can point to any
# separate MuJoCo installation. The scene itself has no parent dependencies.
room_python="${ROOM_PYTHON:-/usr/bin/python3}"
exec "$room_python" "$room_scene_dir/room_viewer.py" "$@"
