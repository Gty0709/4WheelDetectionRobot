# SLAM 建图漂移 / RViz「飘移」排查与修复记录

针对 `humble_sim_slam.launch.py` + `slam_toolbox` + Gazebo Classic `small_house` 建图时，RViz 固定帧为 **`map`** 下小车前后震荡、地图抖动的问题。2026-06-14 经录包对比与运行时探针确认根因并完成修复。

---

## 常见误解：是否直接用了 Gazebo 位姿真值？

**不是。**

当前链路仍是**轮式里程计（encoder 模型）**，不是 Gazebo 世界坐标系下的 ground truth：

| 环节 | 话题 / TF | 数据来源 |
|------|-----------|----------|
| Gazebo `diff_drive` | `/wheel/odom` | 根据四轮关节角速度、轮径、轮距**积分**得到位姿（与实车编码器里程计同类模型） |
| `robot_localization` EKF | `/odom`、`odom→base_footprint` | **融合** `/wheel/odom` 的位姿与速度（非真值拷贝） |
| `slam_toolbox` | `map→odom` | 激光 scan matching **修正**里程计累积误差 |

Gazebo 插件若设置 `odometry_source=world` 才会用仿真真值；本仓库 **未启用**，`publish_odom_tf` 也为 `false`（由 EKF 独占 `odom→base` TF）。

修复的本质是：**让 EKF 信任并跟踪 diff_drive 的轮子里程计**，而不是以前那样「只拿 `vx` + IMU 航向自己积分」导致 `/odom` 与 `/wheel/odom` 严重背离。

---

## 现象（修复前）

- 滑条恒速 ~0.07 m/s 时，RViz（Fixed Frame = `map`）中小车**前后震荡**
- 轮子偶发白模、激光在 RViz/Gazebo 中不易看见（显示配置问题，见下文）
- 终端可见 `slam_toolbox` 激光 TF 丢包、`min_laser_range` 警告

---

## 根因（按影响排序）

### 1. EKF 里程计链路错误（主因）

**改前**（`ekf_sim.yaml`）：

- `odom0` 仅融合 `/wheel/odom` 的 **`vx`**
- 位姿由 EKF 用 **`/imu/data` 航向 + 积分`** 推算

**问题**：仿真 IMU 航向与 diff_drive 航向不一致时，`/odom` 轨迹偏离 `/wheel/odom`。  
RViz 在 `map` 帧下位姿 = `map→odom` × `odom→base`，错误的 `odom→base` 迫使 SLAM 频繁大幅修正 `map→odom`，表现为「飘」。

**录包证据**（`map_20260614_105109`）：

| 指标 | 数值 |
|------|------|
| 稳速段 `/odom.x` 方向翻转 | 31 次 |
| 末端 `ekf.x` vs `wheel.x` | **-0.63 vs +1.57**（完全背离） |
| `map→odom` 单步最大跳变 | **0.45 m** |

**改后**（当前 `ekf_sim.yaml`）：

- 融合 `/wheel/odom` 的 **x, y, yaw, vx, vyaw**
- **移除 IMU 输入**（仿真阶段轮子里程计已含航向，避免冲突）

**录包证据**（`map_20260614_111048`）：

| 指标 | 数值 |
|------|------|
| 稳速段 `wheel.x` 方向翻转 | **0 次** |
| `/odom.x` 与 `/wheel/odom.x` | 对齐（同区间） |
| `map→odom` 跳变 | **0** |

### 2. Gazebo 四驱速度环振荡（次因）

**改前**：`max_wheel_acceleration=0.25`、`max_wheel_torque=8`，恒速指令下 `/wheel/odom` 的 `vx` 仍在 **-0.17～+0.19 m/s** 间周期性翻转（指令恒 ~0.075）。

**改后**（`mickrobot_ugv_classic.urdf.xacro`，**真四驱** `num_wheel_pairs=2` 保留）：

- `max_wheel_acceleration=5.0`、`max_wheel_torque=30`
- 轮地接触：各向同性摩擦 `mu1=mu2=1.0`，适度 `kp/kd`，去掉易引发前后轴不一致的 `fdir1` 各向异性配置

### 3. SLAM / 激光配置（辅助）

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `min_laser_range` | **0.2** | 与 slam_toolbox 对激光能力的判断一致，减少无效近距点 |
| `minimum_travel_distance` | **0.2 m** | 降低 `map→odom` 更新频率，减轻小幅抖动 |
| `minimum_travel_heading` | **0.15 rad** | 同上 |
| `restamp_tf` | **true** | 仿真时间下减轻激光与 TF 不同步导致的 Message Filter 丢包 |

### 4. 控制与显示（非 SLAM 根因，但影响体验）

- **`cmd_vel_guard`**：唯一向 Gazebo 发布 `/cmd_vel`，转发 `/cmd_vel_teleop`，防止僵尸节点残留速度
- **RViz 黄色轨迹**：`slam_classic.rviz` / `detection.rviz` 启用 Odometry 显示（`Keep: 120`，订阅 `/odom`）
- **Gazebo 蓝色激光线**：URDF 中 `<visualize>true</visualize>`

---

## TF 与话题链（当前）

```
map  ──(slam_toolbox)──►  odom  ──(EKF)──►  base_footprint  ──►  laser_link
                              ▲
                         /wheel/odom
                    (Gazebo diff_drive 积分)
```

- `/wheel/odom`：Gazebo 发布，remap 自 diff_drive
- `/odom`：EKF 发布（融合轮子里程计）
- **不要**同时让 diff_drive `publish_odom_tf=true` 与 EKF 发布同一 TF

自检：

```bash
ros2 run tf2_tools view_frames
ros2 topic echo /wheel/odom --field pose.pose.position
ros2 topic echo /odom --field pose.pose.position
# 两者应接近，不应长期背离
```

---

## RViz Fixed Frame 说明

| 场景 | 推荐 Fixed Frame | 说明 |
|------|------------------|------|
| 建图观察 SLAM 修正 | `map` | 看地图与激光是否对齐；修复后 `map→odom` 应稳定 |
| 只看里程计轨迹 | `odom` | 橙色箭头直接反映 `/odom` |

若 Fixed Frame = `map` 且 `odom→base` 错误，会误以为「SLAM 在飘」——实际是**里程计 TF 在抖**。

---

## 验证步骤

```bash
bash scripts/kill_sim.sh
colcon build --packages-select mickrobot_description perception_pkg --symlink-install
source install/setup.bash
python3 scripts/launch_mapping_terminals.py --kill-first
```

1. Gazebo 日志应出现 **Wheel pair 1** 与 **Wheel pair 2**（双轴四驱）
2. 滑条恒速 ~0.07 m/s 保持 20–30 s：RViz 中车位应平稳前进，无前后震荡
3. 启动日志中 `min_laser_range` 警告应消失或减轻
4. 录包后对比 `/odom` 与 `/wheel/odom` 位姿应一致

---

## 小结

| 类型 | 结论 |
|------|------|
| 是否用 Gazebo 真值 | **否**；仍为轮速积分里程计 + EKF 融合 + SLAM 激光修正 |
| 主因 | EKF 仅积分 `vx`+IMU → `/odom` 与 `/wheel/odom` 背离 |
| 次因 | 四驱 `max_wheel_acceleration` 过低 → 轮速振荡 |
| 当前状态 | 录包证实稳速段无位姿翻转；仿真建图已关闭环（`slam_toolbox_params_sim.yaml`） |
| 显示 | Odometry 黄箭头、Gazebo 激光线已恢复 |

### 建图尾声双墙 / 突然飘移（2026-06-14 补充）

**现象**：接近结束或回到已扫区域时，RViz/`slam_map.pgm` 出现**重影双墙**（旧栅格未清 + `map→odom` 跳变）。

**主因**：`do_loop_closing: true` 时 slam_toolbox 误闭环或闭环优化，`map→odom` 突变，新激光帧与旧 occupancy 叠在一起。

**修复**（已合入）：

1. 仿真建图使用 [`slam_toolbox_params_sim.yaml`](../src/perception_pkg/config/slam_toolbox_params_sim.yaml)：`do_loop_closing: false`
2. `save_map_snapshot` 在**正式存图前**连发零速并静置 `save_settle_sec`（默认 2.5 s）
3. 存图时写入 `map_odom_offset.yaml`，供航点叠加图将 GT（odom 帧）变换到 map 帧

若仍飘：确认未混用旧 launch；录包对比末段 `map→odom` 是否单步大跳。
