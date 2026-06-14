"""
perception_humble.launch.py — 模块B（Ubuntu 22.04 + Humble + Gazebo Classic）
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare


def _resolve_maps_dir(pkg_share: str) -> str:
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    src_maps = os.path.join(ws_root, 'src', 'perception_pkg', 'maps')
    if os.path.isdir(src_maps):
        return src_maps
    return os.path.join(pkg_share, 'maps')


def _load_initial_pose(path: str) -> dict:
    if not path or not os.path.isfile(path):
        return {}
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    out = {'set_initial_pose': True}
    for key in ('initial_pose_x', 'initial_pose_y', 'initial_pose_yaw'):
        src = key.replace('initial_pose_', '')
        if src in data:
            out[key] = float(data[src])
    mapping = {'cov_xx': 'initial_cov_xx', 'cov_yy': 'initial_cov_yy', 'cov_aa': 'initial_cov_aa'}
    for src, dst in mapping.items():
        if src in data:
            out[dst] = float(data[src])
    return out


def _map_snapshot_node(maps_dir: str, session_dir: str, use_sim_bool: bool, auto_save: bool):
    if not auto_save:
        return None
    params = {
        'use_sim_time': use_sim_bool,
        'maps_dir': maps_dir,
        'filename_prefix': 'slam_map',
        'update_latest_symlink': True,
        'save_on_shutdown': True,
        'save_interval_sec': 90.0,
        'save_settle_sec': 2.5 if use_sim_bool else 1.0,
        'stop_robot_before_save': True,
    }
    if session_dir:
        params['session_dir'] = session_dir
    return Node(
        package='perception_pkg',
        executable='save_map_snapshot.py',
        name='map_snapshot_saver',
        output='screen',
        parameters=[params],
    )


def launch_nodes(context, *args, **kwargs):
    pkg_share = FindPackageShare('perception_pkg').find('perception_pkg')

    use_sim = context.perform_substitution(LaunchConfiguration('use_sim'))
    mode = context.perform_substitution(LaunchConfiguration('mode'))
    map_yaml = context.perform_substitution(LaunchConfiguration('map_yaml_file'))
    auto_save_map = context.perform_substitution(LaunchConfiguration('auto_save_map'))
    maps_dir_arg = context.perform_substitution(LaunchConfiguration('maps_dir')).strip()
    session_dir_arg = context.perform_substitution(LaunchConfiguration('session_dir')).strip()
    initial_pose_file = context.perform_substitution(
        LaunchConfiguration('initial_pose_file')).strip()

    maps_dir = maps_dir_arg if maps_dir_arg else _resolve_maps_dir(pkg_share)
    session_dir = session_dir_arg
    if not session_dir and initial_pose_file:
        session_dir = os.path.dirname(os.path.abspath(initial_pose_file))

    use_sim_bool = use_sim.lower() == 'true'
    localize_mode = mode.lower() == 'localize'
    auto_save_bool = auto_save_map.lower() == 'true'

    config_dir = os.path.join(pkg_share, 'config')
    amcl_params = os.path.join(config_dir, 'amcl_params.yaml')
    amcl_overrides = _load_initial_pose(initial_pose_file)
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
                parameters=[amcl_params, amcl_overrides, {'use_sim_time': use_sim_bool}],
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

    slam_toolbox_params = os.path.join(
        config_dir,
        'slam_toolbox_params_sim.yaml' if use_sim_bool else 'slam_toolbox_params.yaml',
    )

    nodes += [
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
        Node(
            package='topic_tools', executable='relay', name='map_relay',
            output='screen',
            arguments=['/map', '/perception/map'],
            parameters=[{'use_sim_time': use_sim_bool}],
        ),
        Node(
            package='topic_tools', executable='relay', name='waypoints_relay',
            output='screen',
            arguments=['/detection/waypoints', '/perception/waypoints'],
            parameters=[{'use_sim_time': use_sim_bool}],
        ),
    ]
    snapshot = _map_snapshot_node(maps_dir, session_dir, use_sim_bool, auto_save_bool)
    if snapshot is not None:
        nodes.append(snapshot)
    return nodes


def generate_launch_description():
    pkg_share = FindPackageShare('perception_pkg').find('perception_pkg')
    default_map = os.path.join(pkg_share, 'maps', 'map_latest', 'slam_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument(
            'mode',
            default_value='slam',
            description='slam=slam_toolbox 2D激光建图, localize=AMCL纯定位',
        ),
        DeclareLaunchArgument('map_yaml_file', default_value=default_map),
        DeclareLaunchArgument('auto_save_map', default_value='true'),
        DeclareLaunchArgument('maps_dir', default_value=''),
        DeclareLaunchArgument(
            'session_dir', default_value='',
            description='建图会话目录 maps/map_<时间戳>/',
        ),
        DeclareLaunchArgument(
            'initial_pose_file', default_value='',
            description='localize 模式：从 initial_pose.yaml 加载 AMCL 初值',
        ),
        OpaqueFunction(function=launch_nodes),
    ])
