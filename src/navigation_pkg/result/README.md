# 航点巡逻任务结果

本地运行 `waypoint_patrol` 后，每次任务写入 `path_<时间戳>/`，`path_latest` 符号链接指向最近一次。

## 目录结构

```
result/
  path_<YYYYMMDD_HHMMSS>/
    session_meta.json
    waypoints_snapshot.yaml  # clip_detection，非 GT
    tsp_order.yaml
    trajectory.yaml          # odom_dead_reckon 积分轨迹
    mission_trajectory.png   # 地图叠加轨迹；紫色=局部代价地图新障碍物
    detected_obstacles.yaml  # 新障碍物 map 坐标
    mission.yaml             # 每 3s 刷新，回放入口
    mission_summary.yaml
    legs/
      leg_00_-1_to_5.yaml
      ...
  path_latest -> path_<ts>/
```

## 回放（一键 RViz + 地图 + 机器人动画）

```bash
source install/setup.bash
ros2 launch navigation_pkg playback_rviz.launch.py rate:=10.0
```

默认 `result/path_latest`。指定任务目录：

```bash
ros2 launch navigation_pkg playback_rviz.launch.py \
  mission:=src/navigation_pkg/result/path_20260615_084352 rate:=5.0
```

仅发布话题（不打开 RViz / 地图）：

```bash
ros2 run navigation_pkg playback_mission.py --rate 2.0
```

话题：

- `/navigation/driven_path` — 已行驶轨迹（巡逻实时 + 回放）
- `/navigation/playback_pose` — 回放动画位姿
- `/navigation/waypoint_markers` — 橙（未访问）/ 绿（已访问）**检测回形针**，非 GT

本目录内容默认 **不纳入 git**（见仓库根 `.gitignore`）。
