# sim_dataset 投影标注修复记录

本文档记录 `scripts/generate_sim_dataset.py` 在生成 YOLO 虚拟数据集时，**3D 回形针世界坐标 → 2D 图像 bbox** 投影长期错位、最终定位根因并修复的全过程。

## 背景

### 任务

从 rosbag（`slam_20260613_161043`）读取双目图像，结合 `small_house.world` 中 20 个回形针的已知世界坐标（`src/perception_pkg/config/paperclips_small_house.yaml`），自动生成 YOLO 格式标注，输出到 `sim_dataset/`。

### 症状

`sim_dataset/preview/` 中大量预览图出现：

- 绿色 bbox **浮在地平线/天空** 上，而真实回形针贴图在灰色地面
- bbox 与贴图 **水平大致接近、垂直系统性偏高**（约 10–60 px）
- 机器人 **转弯后** 错位更严重（早期误以为只是小偏差）
- 复杂场景（墙角、立柱、红/绿方块旁）出现 **框在空地或错误物体上**

典型错误帧（修复前）：

| 预览文件 | 表现 |
|----------|------|
| `sim_000229_left` | 4 个绿框挤在地平线，地面贴图无框 |
| `sim_000286_right` | 框在天空/地平线，贴图在地面 |
| `sim_000860_left` | 框在空地或方块上，贴图未覆盖 |

## 投影管线（修复后）

```
paperclip 世界坐标 (odom 帧)
    ↓
T_odom_base  ← /odom 插值（child: base_footprint，与 base_link 零位重合）
    ↓
T_base_camera_link  ← /tf_static: base_link → camera_{left,right}_link
    ↓
T_camera_link_optical  ← REP-103 固定旋转（右乘）
    ↓
cv2.projectPoints(K, dist=0) → 像素 bbox → YOLO 归一化
```

关键代码：`scripts/generate_sim_dataset.py` 中 `build_rvec_tvec()`。

## 排查时间线

### 阶段 1：数据集根本生成不出来

| 现象 | 原因 | 修复 |
|------|------|------|
| 采样 0 帧 | 用 bag **记录时间戳** 索引图像，而非 `Image.header.stamp` | 改为 `stamp_to_ns(message.header.stamp)` |
| 脚本报错退出 | `cv_bridge` 与 NumPy 2.x 不兼容 | 手写 `image_msg_to_bgr()`，直接解析 `rgb8` |

### 阶段 2：有框但位置全错

尝试过多种「Gazebo 相机」经验公式，均未根治：

1. **手工 pinhole + `-Z` 前向** — 与 Gazebo 实际成像不一致
2. **`Rotation.from_euler('zyx', [π/2, π/2, 0])` 左乘到 `T_odom_camera`** — 当前代码路径，误差最大
3. **后乘（post-multiply）光学修正** — 有改善，仍偏差约 30–45 px
4. **在错误光学矩阵上叠加 pitch ≈ -50° 网格搜索** — 某一帧可降到 ~13 px，但物理意义不明，无法泛化

同时加了 `above_horizon` 等启发式过滤（`center_v < 0.32 * height`），只能减少「框在天上」的样本，**不能修正几何**。

### 阶段 3：定量对比，锁定根因

对 `sim_000229_left` 做 **地面暗斑检测（blob）↔ 投影中心** 距离评估，对比多种光学旋转与乘序：

| 配置 | 乘序 | blob 平均距离 |
|------|------|----------------|
| `zyx(π/2, π/2, 0)`（旧） | pre（左乘） | ~45 px |
| `zyx(π/2, π/2, 0)`（旧） | post（右乘） | ~39 px |
| **`xyz(-π/2, 0, -π/2)`（REP-103）** | **post（右乘）** | **~3.9 px** |
| `xyz(-π/2, 0, -π/2)` | pre | ~27 px |
| 相机 link 帧 `-X` 深度投影 | — | ~3.9 px（与上式等价） |

结论：**两个问题同时存在**——

1. **欧拉角约定错误**：应用 REP-103 标准的 `xyz(-π/2, 0, -π/2)`，而非 `zyx(π/2, π/2, 0)`
2. **变换乘序错误**：光学修正应 **右乘** 在 `T_odom_camera_link` 上，即  
   `T_odom_optical = T_odom_camera_link @ T_link_optical`  
   而非 `T_optical @ T_odom_camera_link`

### 为何转弯后更糟？

左乘光学修正时，旋转会作用在 **已含机器人航向的** `T_odom_camera` 上，等价于在错误坐标系下施加固定旋转；航向角越大，图像上的偏差越大。右乘 REP-103 修正表示「在相机 link 坐标系内转到光学系」，与机器人朝向无关，故全轨迹稳定。

## 最终修复

```python
# REP-103: camera_link → camera_optical_frame
CAMERA_OPTICAL_FIX = Rotation.from_euler('xyz', [-np.pi / 2, 0, -np.pi / 2]).as_matrix()

def build_rvec_tvec(odom, camera_extrinsic):
    translation, rotation = camera_extrinsic
    transform_odom_camera = (
        make_transform(odom.rotation, odom.position)
        @ make_transform(rotation, translation)
    )
    # 右乘光学修正（修复前为左乘）
    transform_world_camera = transform_odom_camera @ make_transform(
        CAMERA_OPTICAL_FIX, np.zeros(3),
    )
    ...
```

### 已排除的误判

| 怀疑点 | 结论 |
|--------|------|
| `base_footprint` vs `base_link` 外参 | bag 中 `base_footprint → base_link` 为零变换，无影响 |
| 回形针坐标解析错误 | 与 `small_house.world` 中 `<pose>` 一致 |
| 应用 `map` 帧而非 `odom` | 贴图 GT 在 Gazebo 世界/odom 帧；bag 无 SLAM map 漂移问题 |
| 需额外 pitch -50° | 仅为错误光学矩阵上的补偿，非正解 |

## 验证

### 重新生成

```bash
source env_humble.bash
export PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"

python3 scripts/generate_sim_dataset.py \
  --bag src/perception_pkg/maps/bags/slam_20260613_161043 \
  --output sim_dataset \
  --sample-every 5 \
  --preview 80
```

### 指标对比（修复前 → 修复后）

| 指标 | 修复前 | 修复后 |
|------|--------|--------|
| 采样帧数 | ~1475 | 3161 |
| 标注框总数 | ~3063 | 9104 |
| `sim_000229_left` blob 距离 | ~45 px | ~6 px |
| `sim_000286_right` blob 距离 | ~61 px | ~7 px |
| 预览 median blob 距离（80 张） | — | ~14 px |

### 目视结果

修复后用户关心的帧：

- **`sim_000229_left`**：绿框落在地面贴图附近（仍受 `min_box_height` 抬高细框影响，略偏上）
- **`sim_000286_right`**：三框与三处贴图对齐良好
- **`sim_000860_left`**：5 个贴图均有对应绿框

## 相关文件

| 路径 | 说明 |
|------|------|
| `scripts/generate_sim_dataset.py` | 主生成脚本 |
| `src/perception_pkg/config/paperclips_small_house.yaml` | 20 个回形针 GT |
| `src/mickrobot_description/worlds/small_house.world` | 世界源文件 |
| `src/mickrobot_description/urdf/mickrobot_ugv_classic.urdf.xacro` | 相机外参与 Gazebo 插件 |
| `sim_dataset/data.yaml` | YOLO 训练配置 |
| `sim_dataset/preview/` | 带绿框的抽检图 |

## 经验与参考

1. **Gazebo Classic + `frame_name=camera_*_link`** 时，图像像素对应 **光学系 pinhole 模型**；投影必须用 REP-103 的 `camera_link → camera_optical` 变换，并与 OpenCV `projectPoints`（Z 前向、X 右、Y 下）一致。
2. **乘序即物理含义**：右乘 = 在子坐标系（相机 link）内再旋转；左乘 = 在世界/父系下旋转，与标定惯例不符。
3. **不要只靠目视**：用地面特征 blob 与投影中心距离做回归测试，可快速对比 `pre/post`、多种欧拉约定。
4. **启发式过滤不能代替几何**：`above_horizon` 等只能删坏样本，不能修投影。

参考：

- [REP-103](https://www.ros.org/reps/rep-0103.html) — `camera_optical_frame` 后缀与轴向
- [Robotics Stack Exchange: Gazebo camera frame vs OpenCV](https://robotics.stackexchange.com/questions/73459/gazebo-camera-frame-is-inconsistent-with-rviz-opencv-convention)
- [Projecting Points with ROS Pinhole Camera Model](https://xperroni.me/projecting-points-with-ros-pinhole-camera-model.html) — body 系到光学系轴交换

## 后续可选改进

- 在 URDF 中显式添加 `camera_*_optical` link，Gazebo 插件 `frame_name` 指向光学系，与 ROS 生态完全一致
- 对极扁地面框，按距离自适应 `min_box_height`，减少「框比贴图高」的视觉偏差
- 将 blob 对齐抽检集成进 `generate_sim_dataset.py --validate`，作为回归门禁
