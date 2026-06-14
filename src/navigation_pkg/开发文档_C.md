# 模块C：Nav2 导航开发文档

## 功能

模块 C 负责在已有模块 A/B 的基础上启动 Nav2 导航栈，并提供命令行发送终点的工具。

- 模块 A：提供 Gazebo 仿真、`/scan`、`/odom`、`/cmd_vel`
- 模块 B：提供地图加载、AMCL 定位、`map -> odom` TF
- 模块 C：提供全局规划、局部控制、行为树导航、速度平滑、碰撞监控

## 运行前提

必须先通过模块 B 保存过地图，默认地图文件为：

```bash
/home/admin/Desktop/mickrobot_AB/install/perception_pkg/share/perception_pkg/maps/slam_map.yaml
```

如果还没有地图，请先运行 SLAM 并保存地图。

## 启动顺序

终端 1：启动仿真。

```bash
cd /home/admin/Desktop/mickrobot_AB
source install/setup.bash
ros2 launch mickrobot_description bringup.launch.py use_sim:=true
```

终端 2：启动定位和导航。

```bash
source /home/admin/Desktop/mickrobot_AB/install/setup.bash
ros2 launch navigation_pkg navigation.launch.py use_sim:=true
```

终端 3：发送目标点。

```bash
source /home/admin/Desktop/mickrobot_AB/install/setup.bash
ros2 run navigation_pkg send_goal.py 1.0 0.0 0.0
```

第三个参数是 yaw，默认单位是弧度。也可以用角度：

```bash
ros2 run navigation_pkg send_goal.py 1.0 0.0 90 --degrees
```

## 常用参数

```bash
# 不自动启动模块 B 定位，适合你已经手动启动 perception localize 的情况
ros2 launch navigation_pkg navigation.launch.py start_localization:=false

# 打开导航 RViz
ros2 launch navigation_pkg navigation.launch.py rviz:=true

# 指定地图
ros2 launch navigation_pkg navigation.launch.py map_yaml_file:=/absolute/path/to/slam_map.yaml
```
