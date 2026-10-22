#!/usr/bin/env bash
# Activate the project venv, then open the standalone MuJoCo simulator.
set -euo pipefail
scene_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$scene_dir/../shared/scripts/venv.sh"
edog_use_venv "$scene_dir"
scene_file="${1:-$scene_dir/scene.xml}"
if [[ ! -f "$scene_file" && -f "$scene_dir/$scene_file" ]]; then
  scene_file="$scene_dir/$scene_file"
fi
if [[ ! -f "$scene_file" ]]; then
  printf '找不到场景文件: %s\n' "$scene_file" >&2
  exit 1
fi
simulator=""
if [[ -n "${MUJOCO_HOME:-}" && -x "$MUJOCO_HOME/bin/simulate" ]]; then
  simulator="$MUJOCO_HOME/bin/simulate"
elif command -v simulate >/dev/null 2>&1; then
  simulator="$(command -v simulate)"
elif [[ -x "$HOME/mujuco_work/mujoco-3.16.0/bin/simulate" ]]; then
  simulator="$HOME/mujuco_work/mujoco-3.16.0/bin/simulate"
elif [[ -x "$HOME/mujoco_work/mujoco-3.16.0/bin/simulate" ]]; then
  simulator="$HOME/mujoco_work/mujoco-3.16.0/bin/simulate"
fi
if [[ -z "$simulator" ]]; then
  printf '未找到 simulate。请设置 MUJOCO_HOME，或将 simulate 加入 PATH。\n' >&2
  exit 1
fi
exec "$simulator" "$scene_file"
