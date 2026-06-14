"""
perception.launch.py — 模块B 感知节点总入口
════════════════════════════════════════════════════════════════════
顶层参数：
  use_sim   true=仿真(Gazebo) / false=实体硬件
  mode      slam=slam_toolbox 2D建图  / localize=AMCL定位

【建图→定位完整工作流】
  # 终端1：启动模块A仿真
  ros2 launch mickrobot_description bringup.launch.py use_sim:=true

  # 终端2：启动感知 SLAM
  ros2 launch perception_pkg perception.launch.py use_sim:=true mode:=slam

  # 终端3：让小车自动探索建图
  ros2 run perception_pkg patrol_mapper.py
  # 或手动遥控：ros2 run teleop_twist_keyboard teleop_twist_keyboard

  # 终端4：建图满意后保存（SLAM 保持运行）
  ros2 launch perception_pkg save_map.launch.py use_sim:=true

  # 然后 Ctrl-C 终止终端2、3，改为 localize 模式：
  ros2 launch perception_pkg perception.launch.py use_sim:=true mode:=localize

【输出话题（供模块C/E使用）】
  /perception/map            OccupancyGrid  建好的栅格地图
  /perception/current_pose   PoseWithCovarianceStamped  实时位姿
  /perception/waypoints      PoseArray  航点（来自模块E）

【航点写入接口（供模块E）】
  发布到 /detection/waypoints (geometry_msgs/PoseArray)
  perception 节点自动透传至 /perception/waypoints
════════════════════════════════════════════════════════════════════
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare


def _map_snapshot_node(pkg_share: str, use_sim_bool: bool, auto_save: bool):
    if not auto_save:
        return None
    return Node(
        package='perception_pkg',
        executable='save_map_snapshot.py',
        name='map_snapshot_saver',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_bool,
            'maps_dir': os.path.join(pkg_share, 'maps'),
            'filename_prefix': 'slam_map',
            'update_latest_symlink': True,
            'save_on_shutdown': True,
        }],
    )


def launch_nodes(context, *args, **kwargs):
    pkg_share = FindPackageShare('perception_pkg').find('perception_pkg')

    use_sim  = context.perform_substitution(LaunchConfiguration('use_sim'))
    mode     = context.perform_substitution(LaunchConfiguration('mode'))
    map_yaml = context.perform_substitution(LaunchConfiguration('map_yaml_file'))
    auto_save_map = context.perform_substitution(LaunchConfiguration('auto_save_map'))

    use_sim_bool  = use_sim.lower() == 'true'
    localize_mode = mode.lower() == 'localize'
    auto_save_bool = auto_save_map.lower() == 'true'

    config_dir  = os.path.join(pkg_share, 'config')
    amcl_params = os.path.join(config_dir, 'amcl_params.yaml')
    ekf_params = os.path.join(config_dir, 'ekf_sim.yaml')
    nodes = []

    if use_sim_bool:
        nodes.append(TimerAction(
            period=1.0,
            actions=[Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_params, {'use_sim_time': use_sim_bool}],
                remappings=[('odometry/filtered', '/odom')],
            )],
        ))

    # ================================================================
    # LOCALIZE 模式：map_server + AMCL + lifecycle_manager（保持不变）
    # ================================================================
    if localize_mode:
        nodes += [
            LifecycleNode(
                package='nav2_map_server', executable='map_server',
                name='map_server', namespace='', output='screen',
                parameters=[{'use_sim_time': use_sim_bool, 'yaml_filename': map_yaml}],
            ),
            LifecycleNode(
                package='nav2_amcl', executable='amcl',
                name='amcl', namespace='', output='screen',
                parameters=[amcl_params, {'use_sim_time': use_sim_bool}],
                remappings=[('scan', '/scan')],
            ),
            Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager_localization', output='screen',
                parameters=[{
                    'use_sim_time': use_sim_bool,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl'],
                }],
            ),
            Node(
                package='topic_tools', executable='relay', name='map_relay',
                output='screen',
                arguments=['/map', '/perception/map',
                           '--transient-local-sub', '--transient-local-pub'],
                parameters=[{'use_sim_time': use_sim_bool}],
            ),
            Node(
                package='topic_tools', executable='relay', name='pose_relay',
                output='screen',
                arguments=['/amcl_pose', '/perception/current_pose'],
                parameters=[{'use_sim_time': use_sim_bool}],
            ),
            Node(
                package='topic_tools', executable='relay', name='waypoints_relay',
                output='screen',
                arguments=['/detection/waypoints', '/perception/waypoints'],
                parameters=[{'use_sim_time': use_sim_bool}],
            ),
        ]
        return nodes

    # ================================================================
    # SLAM 模式：slam_toolbox 2D 激光建图
    # 与真实车接口保持一致：/scan + /odom + TF(base_footprint)。
    # RTAB-Map 配置文件保留作实验参考，但默认建图使用 slam_toolbox。
    # ================================================================
    slam_toolbox_params = os.path.join(
        config_dir,
        'slam_toolbox_params_sim.yaml' if use_sim_bool else 'slam_toolbox_params.yaml',
    )

    nodes += [
        # ── slam_toolbox 2D SLAM 主节点 ───────────────────────────────
        # 延迟2秒启动：等待 /scan、/odom 和 odom->base_footprint TF 就绪
        TimerAction(
            period=2.0,
            actions=[Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_toolbox_params, {'use_sim_time': use_sim_bool}],
                remappings=[('scan', '/scan')],
            )],
        ),

        # ── /map → /perception/map（供模块C使用）─────────────────────
        Node(
            package='topic_tools', executable='relay', name='map_relay',
            output='screen',
            arguments=['/map', '/perception/map'],
            parameters=[{'use_sim_time': use_sim_bool}],
        ),

        # ── /detection/waypoints → /perception/waypoints（模块E航点）─
        Node(
            package='topic_tools', executable='relay', name='waypoints_relay',
            output='screen',
            arguments=['/detection/waypoints', '/perception/waypoints'],
            parameters=[{'use_sim_time': use_sim_bool}],
        ),
    ]
    snapshot = _map_snapshot_node(pkg_share, use_sim_bool, auto_save_bool)
    if snapshot is not None:
        nodes.append(snapshot)
    return nodes


def generate_launch_description():
    pkg_share = FindPackageShare('perception_pkg').find('perception_pkg')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument(
            'mode',
            default_value='slam',
            description='slam=slam_toolbox 2D激光建图, localize=AMCL纯定位',
        ),
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=os.path.join(pkg_share, 'maps', 'slam_map.yaml'),
            description='(localize模式) AMCL加载的地图yaml路径',
        ),
        DeclareLaunchArgument(
            'auto_save_map',
            default_value='true',
            description='slam 模式：Ctrl-C 退出时自动保存带时间戳地图（yaml+pgm+png）',
        ),
        OpaqueFunction(function=launch_nodes),
    ])
