#!/bin/bash
# 按 profile（humble|jazzy）配置工作空间
set -e
PROFILE="${1:-humble}"
WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ENV_FILE="$WS_DIR/config/${PROFILE}.env"

if [ ! -f "$ENV_FILE" ]; then
  echo "[错误] 未知 profile: $PROFILE（缺少 $ENV_FILE）"
  exit 1
fi
# shellcheck source=/dev/null
source "$ENV_FILE"
export WS_DIR

echo "================================================"
echo " profile=$PROFILE  ROS=$ROS_DISTRO  Gazebo=$GAZEBO_STACK"
echo " 工作空间: $WS_DIR"
echo "================================================"

if [ "$PROFILE" = "humble" ]; then
  bash "$WS_DIR/scripts/install_deps_humble.sh"
elif [ "$PROFILE" = "jazzy" ]; then
  echo "[提示] Jazzy 环境请使用根目录 bash setup.sh"
  exit 0
else
  echo "[错误] 仅支持 humble 或 jazzy"
  exit 1
fi

if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  echo "[错误] 未找到 /opt/ros/$ROS_DISTRO/setup.bash"
  exit 1
fi
# shellcheck source=/dev/null
source "/opt/ros/$ROS_DISTRO/setup.bash"

cd "$WS_DIR"
colcon build --symlink-install
echo "[OK] colcon build 完成"
echo "请执行: source $WS_DIR/env_humble.bash"
