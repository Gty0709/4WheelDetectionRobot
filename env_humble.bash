#!/bin/bash
# 在仓库根目录一键加载环境（供复制粘贴）
# 用法: source env_humble.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"

# Gazebo Classic 系统资源（着色器/材质）；必须在 overlay 前 source
if [ -f /usr/share/gazebo/setup.sh ]; then
  # shellcheck source=/dev/null
  source /usr/share/gazebo/setup.sh
fi

# [Humble迁移] 默认 GPU 渲染（NVIDIA），勿改回 CPU 软渲染
# shellcheck source=/dev/null
source "$SCRIPT_DIR/scripts/gazebo_gpu_env.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/scripts/source_ws.sh" humble

# 多次启停仿真后避免 FastDDS 共享内存端口锁失败（RTPS_TRANSPORT_SHM Error）
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
