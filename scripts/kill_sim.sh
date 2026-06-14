#!/bin/bash
# 一键停止本工作空间仿真相关进程（含四终端建图流程）
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
pkill -9 -f "humble_sim_slam|bringup_classic|spawn_entity.py" 2>/dev/null
pkill -9 -f "async_slam_toolbox_node|sync_slam_toolbox_node" 2>/dev/null
pkill -9 -f "slider_teleop|teleop_slider|cmd_vel_guard|rviz_slam|detection.launch|clip_detector" 2>/dev/null
pkill -9 -f "ros2 topic pub.*cmd_vel" 2>/dev/null
pkill -9 -f "ros2 bag record" 2>/dev/null
# 反复启停后 FastDDS 共享内存锁残留 → RTPS_TRANSPORT_SHM 刷屏
rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
echo "[kill_sim] 已清理 Gazebo / SLAM / Teleop / RViz / Detection / 录包进程"
