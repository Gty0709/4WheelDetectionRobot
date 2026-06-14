# 模块C：Nav2 航点巡逻导航

## 功能

- Gazebo 在 **mapping 先验位姿** spawn（`initial_pose.yaml` 的 x/y/yaw）
- AMCL 使用同一 `initial_pose.yaml` 作为 **mapping 先验**
- 曼哈顿 TSP 航点巡逻 + 结果落盘 `result/path_<ts>/`
- 重定位位姿作为 TSP 起点（不再先导航到最近航点）
- **Held-Karp DP** 在曼哈顿距离度量下求**全局最优**访问顺序

## 坐标系说明

| 坐标系 | 用途 |
|--------|------|
| `map` | 保存的 SLAM 地图、航点、AMCL 输出 |
| `odom` | Gazebo 轮式里程计（spawn 点为零点） |
| Gazebo `world` | 与 `odom` 起点一致 |

**流程**：Gazebo 与 AMCL 均从 mapping 先验出发 → 粒子收敛 → Nav2 规划导航。

## 前提

`map_latest/` 含 `slam_map.yaml`、`waypoints.yaml`、`initial_pose.yaml`（只读，供 AMCL 与 Gazebo spawn）。

**需要模块 A 仿真**。

## 三终端启动（推荐）

仓库根目录执行：

```bash
source env_humble.bash
colcon build --packages-select navigation_pkg mickrobot_description perception_pkg --symlink-install
python3 scripts/launch_navigation_terminals.py --kill-first
```

### 手动分三个终端（依次启动，间隔约 6s）

**终端 1 — Gazebo 仿真**

```bash
cd /home/gty/ros2ws/robothomework20260613
source env_humble.bash
ros2 launch mickrobot_description bringup_classic.launch.py \
  mapping_prior_file:=src/perception_pkg/maps/map_latest/initial_pose.yaml
```

**终端 2 — AMCL 定位 + Nav2**（等 Gazebo 就绪后）

```bash
cd /home/gty/ros2ws/robothomework20260613
source env_humble.bash
ros2 launch navigation_pkg navigation_humble.launch.py \
  start_patrol:=false rviz:=false \
  session_dir:=src/perception_pkg/maps/map_latest
```

**终端 3 — RViz + 航点巡逻**（等 Nav2 active 后）

```bash
cd /home/gty/ros2ws/robothomework20260613
source env_humble.bash
ros2 launch navigation_pkg patrol_rviz.launch.py \
  session_dir:=src/perception_pkg/maps/map_latest
```

| 终端 | 内容 |
|------|------|
| 1 | Gazebo（`spawn_mode:=prior`，默认按 mapping 先验出生） |
| 2 | AMCL + Nav2 |
| 3 | RViz + `waypoint_patrol` |

## RViz 显示

- `Map`：静态栅格 `/map`
- `Global Costmap` / `Local Costmap`：粉色膨胀层（需终端 2 Nav2 active）
- `RobotModel`：机器人模型
- `AMCL Particles`：重定位粒子
- `Detected Paperclips`：橙/绿球 = **CLIP 检测到的回形针位置**（`waypoints.yaml`），**不是 GT**
- `Detected Paperclips`：红（未访问）/ 绿（已访问）**检测回形针**，非 GT
- `Traveled Path`：绿色 = 里程计积分已走轨迹（`/navigation/driven_path`，无 AMCL 折线）
- `Global Plan`：红色全局规划路径

## 航点来源说明

| 数据 | 含义 |
|------|------|
| `waypoints.yaml` | 感知模块 CLIP 检测回形针投影到地图，**非真值** |
| `slam_map_waypoints.png`（感知） | 绿=检测、红叉=GT，仅供建图阶段对照 |
| RViz 橙球 | 与 `waypoints.yaml` 一致，巡逻导航目标 |
| PNG 青黄线 | 已行驶轨迹（仅 PNG；RViz 用绿色 `/navigation/driven_path`） |
| PNG 红球 | 未访问检测回形针 |
| PNG 绿球 | 已访问回形针 |
| PNG | 青黄轨迹 + 红/绿航点；不绘制局部障碍橙点（见 `detected_obstacles.yaml`） |

## 结果目录

启动巡逻后即创建 `result/path_<ts>/` 与 `path_latest`。**每 3 秒自动刷新** `progress.yaml`、`trajectory.yaml`、`mission.yaml`、`mission_trajectory.png`（含紫色新障碍物标记）；Ctrl+C 会立即落盘完整结果。

## TSP 说明

| 项目 | 说明 |
|------|------|
| 起点 | 重定位完成后的机器人位姿（虚拟 depot，非航点列表中的点） |
| 距离 | 航点间 **曼哈顿距离** \|Δx\|+\|Δy\| |
| 算法 | **Held-Karp** 动态规划，固定起点、开放路径（不返回起点） |
| 最优性 | 对上述度量 **全局最优**（穷举所有排列的最短解；20 个航点约 2²¹ 状态，可精确求解） |
| 注意 | Nav2 实际行驶路径受障碍物/栅格影响，可能与曼哈顿理论长度不同 |

`result/.../tsp_order.yaml` 含 `optimal_manhattan_cost_m` 与 `order_ids`。

## 规划器说明

| 组件 | 当前配置 |
|------|----------|
| 全局规划 | **SmacPlanner2D**（差速/四驱友好，路径更平滑） |
| 局部控制 | **DWB**（禁止倒车 `min_vel_x:0`；无 `PreferForward`，允许转向） |
| 航点坐标 | **不修改** `waypoints.yaml` 原始位置，导航目标与标记球一致 |
| 备用局部 | `FollowPath_rpp`（RPP，未启用） |
| 备用全局 | `GridBased_navfn`（NavFn，未启用） |

RViz：`Global Plan` 红、`Traveled Path` 绿（`/navigation/driven_path`）。

## 回放

```bash
source install/setup.bash
ros2 run navigation_pkg playback_mission.py --rate 2.0
```

默认回放 `result/path_latest`。

## 故障排查

| 现象 | 处理 |
|------|------|
| 代价地图 `No map received` | 确认终端 2 已启动；`ros2 lifecycle get /planner_server` 为 active；`ros2 topic echo /global_costmap/costmap --once` |
| 不重定位 | 观察粒子云是否收敛；默认约 5s 后开始判断，先验对齐后稳定 2s 即导航 |
| result 为空 | 巡逻启动后应有 `progress.yaml`（每 3s 刷新）；Ctrl+C / 异常退出会写入完整 `mission_summary.yaml` |
| 贴墙航点难到达 | inflation 半径缩小（local 0.28 / global 0.30）、`cost_scaling_factor: 4.5` 陡衰减；DWB 用 `BaseObstacle` 可穿行非致命膨胀 |
| 不转弯 / wz=0 | 已去掉 `PreferForward`（会惩罚原地转向）；`min_vel_x:0` 已禁止倒车 |
| 倒数航点 id=14/4 前后抖动 | DWB 禁止倒车；`Oscillation` 适中；巡逻节点卡死检测区分「转向中」 |
| id=6→5 撞红方块 | 恢复 `ObstacleFootprint`(0.12)+`BaseObstacle`；全局规划 `cost_travel_multiplier: 6` 绕障 |
| 航点在膨胀区内 | 导航失败但距航点 ≤ `--approach-tolerance`（默认 0.40 m）→ 软成功标记已访问 |
| TSP 末段未访问（如 id=4/8 红球） | 主循环失败**延后**到 sweep（默认 4 轮、`--sweep-approach-tolerance 0.55`、`--sweep-max-nav-retries 6`）；勿在旧版逻辑里把跳过点记入 `aborted` 后直接 SAVE |
| No valid trajectories | 已改 `ObstacleFootprint`→`BaseObstacle`；致命障碍仍禁止，膨胀代价仅扣分 |
| 窄通道过不去 | 可略增局部 inflation 到 0.40，或改 `controller_plugins: ["FollowPath_dwb"]` |
| 不动 / Failed to make progress | Nav2→`/cmd_vel_nav`→`cmd_vel_guard`→`/cmd_vel`；终端1日志应出现 `Nav2 cmd_vel_nav: vx=...`；**重启终端1+2** |
| TSP 后卡很久才导航 | Held-Karp 约 20s（20航点），已改后台线程，重定位后应立即显示 `computing optimal TSP` |
| Goal rejected 连环跳过 | 已修：换航点重置重试计数，取消后延迟 10s 再发目标 |
| 导航失败 | **先 backup 倒车** → 清空代价地图 → 等待 **10s** 后重新发目标；重试耗尽仍近航点则软成功 |
| 到点绕圈 | 目标 yaw 用当前朝向；已去掉 DWB `RotateToGoal`/`GoalAlign`；`xy_goal_tolerance: 0.25` |
| backup 崩溃 | 已修：`DurationMsg` 用于 BackUp `time_allowance`（勿与 `rclpy.duration.Duration` 混用） |

## Ubuntu 版本

| 系统 | Launch |
|------|--------|
| 22.04 Humble | `navigation_humble.launch.py` |
| 24.04 Jazzy | `navigation.launch.py`（保留） |

见 [`result/README.md`](result/README.md)。
