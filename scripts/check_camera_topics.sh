#!/usr/bin/env bash
# 建图时确认 Gazebo 双目是否在发图（检测节点依赖 /camera/left|right/image_raw）
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "$ROOT/env_humble.bash"

echo "[check_camera] 等待 /camera/left/image_raw …"
if ! timeout 8 ros2 topic echo /camera/left/image_raw --once >/dev/null 2>&1; then
  echo "[check_camera] 失败：8s 内无左目图像。" >&2
  echo "  1) 确认终端1 humble_sim_slam 在跑且机器人已 spawn" >&2
  echo "  2) bash scripts/kill_sim.sh 后重启仿真" >&2
  echo "  3) ros2 topic list | grep camera" >&2
  exit 1
fi

echo "[check_camera] 左目 OK，测频率（3s）…"
timeout 3 ros2 topic hz /camera/left/image_raw 2>&1 | tail -3 || true
timeout 3 ros2 topic hz /camera/right/image_raw 2>&1 | tail -3 || true
echo "[check_camera] 通过。可启动 detection.launch.py"
