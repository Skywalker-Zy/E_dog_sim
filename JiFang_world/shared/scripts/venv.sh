#!/usr/bin/env bash
# Shared setup for launchers; source this file, then call edog_use_venv DIR.
edog_use_venv() {
  local launch_dir="$1"
  local search_dir="$launch_dir"
  local selected="${EDOG_VENV:-${VIRTUAL_ENV:-}}"
  if [[ -z "$selected" ]]; then
    while [[ -n "$search_dir" ]]; do
      if [[ -f "$search_dir/.venv/pyvenv.cfg" && -x "$search_dir/.venv/bin/python" ]]; then
        selected="$search_dir/.venv"
        break
      fi
      [[ "$search_dir" == / ]] && break
      search_dir="$(dirname -- "$search_dir")"
    done
  fi
  if [[ -z "$selected" || ! -f "$selected/pyvenv.cfg" || ! -x "$selected/bin/python" ]]; then
    printf '未找到可用的虚拟环境。请在项目目录执行：\n  python3 -m venv .venv\n  .venv/bin/python -m pip install -r requirements.txt\n也可设置 EDOG_VENV=/虚拟环境目录。\n' >&2
    return 1
  fi
  selected="$(cd -- "$selected" && pwd)"
  export VIRTUAL_ENV="$selected"
  export PATH="$selected/bin:$PATH"
  export EDOG_PYTHON="$selected/bin/python"
}

edog_source_ros() {
  # --no-ros must also work on computers without ROS installed.
  local argument
  for argument in "$@"; do
    [[ "$argument" == --no-ros || "$argument" == --help || "$argument" == -h ]] && return 0
  done
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    return 0
  fi
  local ros_distribution=""
  if [[ -r /etc/os-release ]]; then
    local ID="" VERSION_ID=""
    source /etc/os-release
    case "$ID:$VERSION_ID" in
      ubuntu:24.04) ros_distribution=jazzy ;;
      ubuntu:22.04) ros_distribution=humble ;;
    esac
  fi
  if [[ -n "$ros_distribution" && -f "/opt/ros/$ros_distribution/setup.bash" ]]; then
    # ROS setup files may reference unset variables under `set -u`.
    set +u
    source "/opt/ros/$ros_distribution/setup.bash"
    set -u
  fi
}
