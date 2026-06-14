#!/bin/bash
# 启动 Gazebo 前清理僵尸进程，避免 gzserver exit 255 / gzclient 卡在 Preparing your world
set +e
MASTER_PORT=11345

echo "[gazebo_preflight] 清理旧 Gazebo 进程..."
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
sleep 1

if command -v ss >/dev/null 2>&1; then
  if ss -ltn 2>/dev/null | grep -q ":${MASTER_PORT} "; then
    echo "[gazebo_preflight] 警告: 端口 ${MASTER_PORT} 仍被占用，再次清理..."
    pkill -9 -f "gzserver|gzclient" 2>/dev/null
    sleep 1
  fi
fi

echo "[gazebo_preflight] 就绪"
exit 0
