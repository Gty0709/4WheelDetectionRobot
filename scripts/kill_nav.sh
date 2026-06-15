#!/bin/bash
# 仅清理 Nav2 / AMCL / 巡逻残留（不杀 Gazebo）。重复启动终端 2/3 前必须执行，
# 否则僵尸 map_server(active) 会导致 lifecycle configure 失败、无 map TF、RViz 白模。
set -euo pipefail

_pkill_node() {
  local node="$1"
  pkill -9 -f "__node:=${node}" 2>/dev/null || true
}

for n in map_server playback_map_server amcl \
  controller_server smoother_server planner_server behavior_server \
  bt_navigator waypoint_follower; do
  _pkill_node "$n"
done

pkill -9 -f "nav2_lifecycle_manager/lifecycle_manager" 2>/dev/null || true
pkill -9 -f "robot_localization/ekf_node" 2>/dev/null || true
pkill -9 -f "waypoint_patrol.py" 2>/dev/null || true
pkill -9 -f "rviz2.*navigation.rviz|playback_rviz.launch|playback_mission.py" 2>/dev/null || true

# 勿在此删除 /dev/shm/fastrtps_*：Gazebo 仍在跑时会破坏 DDS，导致 lifecycle get_state 超时

echo "[kill_nav] Nav2 / AMCL / patrol / navigation RViz 已清理"
