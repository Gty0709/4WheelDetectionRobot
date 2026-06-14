#!/bin/bash
# 加载本工作空间 overlay
# 用法: source scripts/source_ws.sh [humble|jazzy]
PROFILE="${1:-humble}"
_WS_SCRIPT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$_WS_SCRIPT/.." && pwd)"

if [ -z "${ROS_DISTRO:-}" ]; then
  if [ -f "/opt/ros/humble/setup.bash" ]; then
    # shellcheck source=/dev/null
    source "/opt/ros/humble/setup.bash"
  else
    echo "[source_ws] 错误: 未找到 /opt/ros/humble/setup.bash" >&2
    return 1 2>/dev/null || exit 1
  fi
fi

if [ ! -f "$WS_DIR/install/local_setup.bash" ]; then
  echo "[source_ws] 错误: 未编译本工作空间，请先执行: bash setup_humble.sh" >&2
  return 1 2>/dev/null || exit 1
fi

# shellcheck source=/dev/null
source "$WS_DIR/install/local_setup.bash"

if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi

if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix mickrobot_description >/dev/null 2>&1; then
  echo "[source_ws] OK  mickrobot_description -> $(ros2 pkg prefix mickrobot_description)"
else
  echo "[source_ws] 错误: 仍找不到 mickrobot_description" >&2
  return 1 2>/dev/null || exit 1
fi
