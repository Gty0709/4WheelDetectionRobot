# 模块 C 导航调参记录

> 配置文件：`src/navigation_pkg/config/nav2_params.yaml`  
> 巡逻编排：`src/navigation_pkg/scripts/waypoint_patrol.py`  
> 地图会话：`src/perception_pkg/maps/map_20260614_185137`（`map_latest` 指向该会话）  
> 任务落盘：`src/navigation_pkg/result/path_<时间戳>/`

本文记录历次 Nav2 / DWB / 代价地图 / 巡逻节点调参、对应任务结果，以及**当前生效的一版参数**（以 `path_20260615_131011` 实验 G **20/20** 验证为准；回退基线见 `path_20260615_084352`）。

---

## 0. 基线快照（2026-06-15，调参前冻结）

| 项目 | 路径 / 值 |
|------|-----------|
| Nav2 参数快照 | `src/navigation_pkg/config/nav2_params_baseline_20260615.yaml` |
| 代表任务 | `path_20260615_084352`（20/20，1141 s） |
| 回退命令 | `cp src/navigation_pkg/config/nav2_params_baseline_20260615.yaml src/navigation_pkg/config/nav2_params.yaml` |

**`waypoint_patrol.py` 基线默认参数（快照时）：**

| 参数 | 值 |
|------|-----|
| `--dwell-sec` | 2.0 |
| `--approach-tolerance` | 0.40 |
| `--retry-delay-sec` | 10.0 |
| `--stall-time-sec` | 12.0 |
| `--stall-min-travel-m` | 0.25 |
| `--max-nav-retries` | 3 |
| `--backup-dist` / `--backup-speed` | 0.40 m / 0.18 m/s |

**实验轮次 G（2026-06-15，快速恢复 + DWB 小步调参）：**

| 类别 | 改动 |
|------|------|
| BT | `navigate_to_pose_fast_recovery.xml`：去掉 Wait 5s，顶层重试 6→3 |
| progress | `movement_time_allowance` 25→11 s |
| DWB | `PathAlign/PathDist` 14→9；`ObstacleFootprint` 0.12→0.16；`BaseObstacle` 0.18→0.22；`max_vel_x` 0.30→0.26 |
| planner | `cost_travel_multiplier` 6→7.5 |
| patrol | 首次失败先 local clear + 0.8s 重试；之后定向 spin+backup；`dwell` 2→0.75 s；`retry-delay` 10→1 s |

---

## 1. 当前结论（2026-06-15）

| 指标 | 结果 |
|------|------|
| **代表任务（实验 G，当前）** | `path_20260615_131011`（`path_latest`） |
| 调参前基线 | `path_20260615_084352`（20/20，1141 s） |
| 访问航点（实验 G） | **20 / 20** |
| 任务时长（实验 G） | **1017.6 s**（约 17 min，较基线略短） |
| 难点点位 | id=6→5（675 s）；id=11（986 s）；id=4/8 末段 sweep 成功 |

轨迹图：`src/navigation_pkg/result/path_20260615_131011/mission_trajectory.png`  
README **图 5 / 图 6** 使用同会话截图与落盘轨迹。

---

## 2. 任务结果一览

| 任务目录 | 时长 (s) | 访问数 | 放弃 id | 说明 |
|----------|----------|--------|---------|------|
| `path_20260614_221324` | 978 | 17/20 | — | 早期：窄道 inflation 初调、提速 |
| `path_20260614_224227` | 382 | 17/20 | — | 缩短 DWB / 容差试验 |
| `path_20260614_225154` | 578 | 15/20 | — | 倒数 id=14→4 前后抖动、贴墙卡住 |
| `path_20260614_231327` | 66 | 0/20 | — | **中断/未进入 TSP**（仅重定位或早停） |
| `path_20260614_231825` | 508 | 17/20 | 4, 8 | 末段跳过逻辑，左下两点未访 |
| `path_20260614_233051` | 187 | 9/20 | — | 部分任务 / 早停 |
| `path_20260614_233705` | 160 | 3/20 | — | 部分任务 / 早停 |
| `path_20260614_234220` | 341 | 11/20 | — | 部分任务 / 早停 |
| `path_20260614_235730` | 486 | 17/20 | — | inflation + ObstacleFootprint 组合试验 |
| `path_20260615_000833` | 113 | 2/20 | — | 早停 / 调试 |
| `path_20260615_001157` | 86 | 1/20 | — | 早停 / 调试 |
| `path_20260615_001514` | 1261 | 18/20 | **4, 8** | sweep 未跑完即 SAVE；id=11 在 1259 s 才到 |
| `path_20260615_084352` | 1141 | 20/20 | — | 调参前基线：sweep + 双 critic + 禁倒车 |
| **`path_20260615_131011`** | **1018** | **20/20** | **—** | **实验 G：快速恢复 BT + DWB 小步 + 启动时序修复** |

> 仅 `mission_summary.yaml` 中 `visit_count` 与 `aborted_waypoint_ids` 为权威计数；早停任务时长偏短属正常现象。

---

## 3. 调参历程（按问题驱动）

### 3.1 窄通道 / 贴墙（id=14、id=4）

**现象**：`Failed to make progress`、`No valid trajectories`；全局路径有，局部 DWB 过不去。

| 轮次 | 主要改动 | 结果 |
|------|----------|------|
| A | footprint `0.14×0.12` → `0.12×0.10`；global/local `inflation_radius` 降至 0.22/0.20；`cost_scaling_factor: 8`；`max_vel_x: 0.34` | `path_20260614_221324` 17/20，通道略通 |
| B | 局部窗口 3→**4 m**；local `inflation_radius` 提到 0.38~0.50；`BaseObstacle.scale` 下调 | 粉带变宽，仍易贴墙振荡 |
| C | **恢复** `ObstacleFootprint` + `BaseObstacle` 双 critic；inflation 统一 **0.32**；`cost_scaling_factor: 5.0` | `path_20260614_235730` 17/20，id=14 可过 |
| D | 巡逻节点：高代价区 **backup 0.4 m**；Nav2 `behavior_server.backup` 同步 | 贴墙脱困，但配合倒车时出现来回抖 |

### 3.2 来回振荡 / 倒车（id=14→4）

**现象**：机器人在航点附近前后挪动；DWB 选倒车轨迹。

| 轮次 | 主要改动 | 结果 |
|------|----------|------|
| E | DWB `min_vel_x: -0.25`（允许倒车）+ `vx_samples: 28` | 脱困能力↑，振荡↑ |
| F | **`min_vel_x: 0`**（禁止倒车）；`Oscillation.scale: 12`；巡逻 stall 检测区分「原地转向」 | 抖动减轻；`path_20260614_225154` 仍 15/20 |

### 3.3 不转向（wz=0）

**现象**：应原地对齐时线速度被压到 0，角速度也为 0。

| 改动 | 说明 |
|------|------|
| 去掉 DWB `PreferForward` | 该 critic 惩罚低 `vx`，导致「只转不走」被判极差 |
| 去掉 `RotateToGoal` / `GoalAlign` | 到点不再绕圈对齐；目标 yaw 用**当前朝向** |
| `xy_goal_tolerance: 0.25` | 到点判定放宽 |

### 3.4 碰撞 / 膨胀区航点（id=6→5、红方块）

**现象**：6→5 段擦碰 Gazebo 红块；部分航点落在膨胀格内。

| 改动 | 说明 |
|------|------|
| `ObstacleFootprint.scale: 0.12` + `BaseObstacle.scale: 0.18` | 致命障碍仍硬禁，膨胀区可穿行但扣分 |
| Smac `cost_travel_multiplier: 6.0` | 全局路径更绕开高代价 |
| `collision_monitor.FootprintApproach.enabled: false` | 窄道误刹降低 |
| 巡逻 `--approach-tolerance 0.40` | 距航点 ≤0.4 m 导航失败也**软成功** |

### 3.5 恢复链路与节点崩溃

**现象**：导航失败时巡逻节点直接退出；backup 报 `Duration` 类型错误。

| 改动 | 说明 |
|------|------|
| `DurationMsg`（`builtin_interfaces`）用于 BackUp `time_allowance` | 修复 backup 崩溃 |
| 失败流程：软成功 → backup → **清空 costmap** → 10 s 重试 | `_clear_costmaps()` |
| `movement_time_allowance: 25.0` | 真卡住约 25 s 进入恢复 |

### 3.6 TSP 末段截断（id=4、id=8）

**现象**：`path_20260615_001514` 访问 18/20，`aborted: [4,8]`，主循环跳过后直接 SAVE。

| 改动 | 说明 |
|------|------|
| 主循环失败 → **defer 到 sweep**，不立即 `aborted` | 继续走完 TSP 其余点 |
| sweep：4 轮 × 每点 6 次重试，`approach_tolerance 0.55` | 专补左下贴墙点 |
| 冻结 `_tsp_targets` 坐标 | 目标不因重访逻辑漂移 |

**结果**：`path_20260615_084352` **20/20**，id=4 @1120 s，id=8 @1138 s。

### 3.7 回放（非 Nav2，但与演示相关）

| 改动 | 说明 |
|------|------|
| `playback_rviz.launch.py` | 一键 map + RViz + 动画 |
| 墙钟计时、`map→base_footprint` TF | 避免 `use_sim_time` 冻结 |
| 播完 `Shutdown` 全进程退出 | 避免重复 TF 导致「两位置跳变」 |

### 3.8 Nav2 lifecycle 启动失败（`planner_server` 超时）

**现象**：终端 2 仅一行 `lifecycle_manager_localization: Managed nodes are active`；`planner_server/get_state (timeout)`；终端 3 一直 `Still waiting for Nav2`。

| 改动 | 说明 |
|------|------|
| Nav2 节点**逐个**启动（间隔 1 s），`lifecycle_manager_navigation` **延后 ~19 s** | 避免 configure 期间 `get_state` 2 s 硬超时 |
| `bond_timeout: 15.0` | 慢硬件 / 仿真负载下 bond 更稳 |
| `kill_nav.sh` **不再**删除 `/dev/shm/fastrtps_*` | Gazebo 仍在跑时清 DDS 共享内存会破坏通信 |
| `kill_sim.sh` 全量清理时才清 DDS shm | 仅重启 Nav2 用 `kill_nav.sh` |
| `patrol_rviz` 巡逻节点延迟 25 s | 给 Nav2 留足 bringup 时间 |

**成功标志**：终端 2 出现**两次** `Managed nodes are active`（约启动后 25–30 s）。

### 3.9 实验 G（快速恢复 + DWB 小步）

**现象**：基线 1141 s；希望缩短卡死恢复与到点驻留时间。

| 改动 | 说明 |
|------|------|
| BT `navigate_to_pose_fast_recovery.xml` | 去掉 Wait 5 s，顶层重试 6→3 |
| `movement_time_allowance` 25→11 s | 更快进入恢复 |
| DWB `PathAlign/PathDist` 14→9；`max_vel_x` 0.30→0.26 | 减轻贴墙振荡 |
| patrol `dwell` 2→0.75 s；`retry-delay` 10→1 s | 缩短驻留与重试间隔 |
| 恢复链：local clear → spin+backup | 首次失败先清局部代价地图 |

**结果**：`path_20260615_131011` **20/20**，1017.6 s；README 图 5/6 已更新。

---

## 4. 当前 Nav2 参数（`nav2_params.yaml`）

### 4.1 代价地图

| 参数 | local | global |
|------|-------|--------|
| `width` × `height` | 4 × 4 m | — |
| `resolution` | 0.05 | 0.05 |
| `footprint` | `[[0.12,±0.10],[-0.12,±0.10]]` | 同左 |
| `inflation_radius` | **0.32** | **0.32** |
| `cost_scaling_factor` | **5.0** | **5.0** |

### 4.2 DWB `FollowPath`（当前启用）

| 参数 | 值 |
|------|-----|
| `controller_plugins` | `["FollowPath"]` |
| `min_vel_x` / `max_vel_x` | **0.0** / **0.26** |
| `max_vel_theta` | 1.0 |
| `vx_samples` / `vtheta_samples` | 28 / 28 |
| `sim_time` | 2.0 |
| `xy_goal_tolerance` | **0.25** |
| `critics` | `Oscillation`, `ObstacleFootprint`, `BaseObstacle`, `PathAlign`, `PathDist`, `GoalDist` |
| `ObstacleFootprint.scale` | **0.16** |
| `BaseObstacle.scale` | **0.22** |
| `PathAlign.scale` / `PathDist.scale` / `GoalDist.scale` | 9 / 9 / 18 |
| `Oscillation.scale` | 12.0 |
| `Oscillation.oscillation_reset_dist` | 0.10 |
| `Oscillation.oscillation_reset_angle` | 0.35 |

**未使用（保留在 yaml 内）：** `FollowPath_rpp`、`FollowPath_dwb_legacy`（含 `RotateToGoal`/`GoalAlign`）、`FollowPath_rpp_legacy`。

### 4.3 全局规划

| 参数 | 值 |
|------|-----|
| `planner_plugins` | `["GridBased"]` |
| plugin | `nav2_smac_planner/SmacPlanner2D` |
| `cost_travel_multiplier` | **7.5** |
| `tolerance` | 0.20 |
| `allow_unknown` | true |

### 4.4 进度 / 目标 / 速度

| 节点 | 参数 | 值 |
|------|------|-----|
| `progress_checker` | `movement_time_allowance` | **11.0** |
| `progress_checker` | `required_movement_radius` | 0.08 m |
| `general_goal_checker` | `xy_goal_tolerance` | 0.25 m |
| `general_goal_checker` | `yaw_goal_tolerance` | 1.57 rad |
| `velocity_smoother` | `max_velocity` | [0.30, 0, 1.0] |
| `behavior_server.backup` | `backup_dist` / `speed` | 0.40 m / 0.18 m/s |
| `collision_monitor` | `FootprintApproach.enabled` | **false** |

### 4.5 BT / 超时

| 参数 | 值 |
|------|-----|
| `default_nav_to_pose_bt_xml` | `navigate_to_pose_fast_recovery.xml`（本包） |
| `action_server_result_timeout` | 900.0 s |

---

## 5. 当前巡逻节点参数（`waypoint_patrol.py` 默认值）

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `--dwell-sec` | 0.75 |
| `--approach-tolerance` | 0.40 m |
| `--sweep-approach-tolerance` | 0.55 m |
| `--max-nav-retries` | 3 |
| `--sweep-max-nav-retries` | 6 |
| `--max-sweep-rounds` | 4 |
| `--quick-retry-delay-sec` | 0.8 |
| `--retry-delay-sec` | 1.0 |
| `--stall-spin-time-sec` | 8.0 |
| `--backup-dist` | 0.40 m | 巡逻触发 backup 距离 |
| `--backup-speed` | 0.18 m/s | backup 速度 |
| `--max-backup-attempts` | 3 | 单次 backup 链最多尝试 |
| `--high-cost-threshold` | 140 | footprint 代价超阈先 backup |
| `--stall-time-sec` | 12.0 | 振荡卡死判定窗口 |
| `--stall-min-travel-m` | 0.25 | 窗口内最小位移（区分原地转） |
| `--relocalize-min-sec` | 5.0 | AMCL 最短等待 |
| `--relocalize-settle-sec` | 2.0 | 位姿稳定时间 |

---

## 6. 难点点位坐标（检测航点，非 GT）

| id | x | y | 备注 |
|----|------|------|------|
| 14 | — | — | TSP 倒数第 4，贴墙窄道入口 |
| 4 | -2.9976 | -1.5102 | 左下，膨胀区内 |
| 8 | -4.2137 | -2.7961 | 左下最角 |
| 6 | 0.2078 | 2.9966 | 近中部 |
| 5 | -1.8041 | 3.2072 | 6→5 近 Gazebo 红块 (-3, 2.5) |

手动补点（Nav2 须在终端 2 运行）：

```bash
ros2 run navigation_pkg send_goal.py -2.9976 -1.5102 0   # id=4
ros2 run navigation_pkg send_goal.py -4.2137 -2.7961 0   # id=8
```

---

## 7. 调参后如何生效

| 变更类型 | 需重启 |
|----------|--------|
| `nav2_params.yaml` | **终端 2**（Nav2 launch） |
| `waypoint_patrol.py` | **终端 3**（`patrol_rviz.launch.py`） |
| Gazebo / `cmd_vel_guard` | **终端 1** |

```bash
source env_humble.bash
colcon build --packages-select navigation_pkg --symlink-install
python3 scripts/launch_navigation_terminals.py --kill-first
```

---

## 8. 相关文档

- 操作与 RViz 说明：[`src/navigation_pkg/开发文档_C.md`](../src/navigation_pkg/开发文档_C.md)
- 任务目录结构：[`src/navigation_pkg/result/README.md`](../src/navigation_pkg/result/README.md)
- 话题全图：[`docs/TOPIC_COMMUNICATION.md`](TOPIC_COMMUNICATION.md)
