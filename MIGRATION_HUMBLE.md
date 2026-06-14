# Ubuntu 22.04 + Humble 迁移说明

## 与 Jazzy（24.04）版的区别

| 项目 | Jazzy（`setup.sh`） | Humble 本机（`setup_humble.sh`） |
|------|---------------------|----------------------------------|
| 仿真 | Gazebo Sim + `ros_gz` | **Gazebo Classic 11** |
| 模块 A launch | `bringup.launch.py` | **`bringup_classic.launch.py`** |
| 模块 A URDF | `mickrobot_ugv.urdf.xacro` | **`mickrobot_ugv_classic.urdf.xacro`** |
| 模块 A 世界 | `small_house_gz.world` | **`small_house.world`** |
| 模块 B launch | `perception.launch.py` | **`perception_humble.launch.py`** |
| 一键建图 | — | **`humble_sim_slam.launch.py`** + **`rviz_slam.launch.py`** |
| SLAM 后端 | slam_toolbox | slam_toolbox（相同） |

原 gz-sim 相关 launch/urdf/world **保留在包内**（24.04 路径未删除，仅 Classic 使用独立文件）。

## 工作空间整合

原顶层 `A/`、`B/`、`C/` 与 `src/` 内容对比：

- **A/mickrobot_description** 与 **src/mickrobot_description**：5 个文件有差异（launch、urdf、world、rviz），以 `src/` 为准并合并 TaskA v5 模型
- **B/perception_pkg** 与 **src/perception_pkg**：launch/maps/package.xml 有差异，`src/` 含 `slam_toolbox_params.yaml` 等更新
- **C/navigation_pkg**：原先不在 `src/`，已移入 `src/navigation_pkg/`

整合后删除 `A/`、`B/`、`C/`，所有包位于 `src/`。

## TaskA v5 模型替换范围

仅替换仿真中的**机器人模型与传感器外参**（来自 `TaskA_交付物_v5.zip`）：

- `urdf/mickrobot_ugv.urdf.xacro` / `mickrobot_ugv_classic.urdf.xacro` 的 link/joint
- `urdf/mesh/` 全部 mesh（含 `board_camera_radar_assembly.stl`）
- `urdf/mickrobot_ugv.urdf`、`mickrobot_stl*.urdf` 同步更新

未改动：Nav2 参数、slam_toolbox 参数、模块间接口话题名。

## 一次性配置

```bash
cd /home/gty/ros2ws/robothomework20260613
bash setup_humble.sh
source env_humble.bash
```

## 依赖

清单：`config/humble/deps-apt.list`
