#!/usr/bin/env bash
set -euo pipefail
room_scene_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
room_simulator=""
if [[ -n "${MUJOCO_HOME:-}" && -x "$MUJOCO_HOME/bin/simulate" ]]; then
  room_simulator="$MUJOCO_HOME/bin/simulate"
elif command -v simulate >/dev/null 2>&1; then
  room_simulator="$(command -v simulate)"
elif [[ -x "$HOME/mujoco_work/mujoco-3.16.0/bin/simulate" ]]; then
  room_simulator="$HOME/mujoco_work/mujoco-3.16.0/bin/simulate"
fi
if [[ -z "$room_simulator" ]]; then
  echo '找不到 simulate，请设置 MUJOCO_HOME 或将 simulate 加入 PATH。' >&2
  exit 1
fi
exec "$room_simulator" "$room_scene_dir/scene.xml"
