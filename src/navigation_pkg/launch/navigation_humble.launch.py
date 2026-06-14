"""
navigation_humble.launch.py — 模块C（Ubuntu 22.04 + Humble + Gazebo Classic）

默认使用 src/perception_pkg/maps/map_latest 会话中的地图、初值与航点。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _resolve_ws_root() -> str:
    pkg_share = get_package_share_directory('navigation_pkg')
    return os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))


def _abs_path(path: str) -> str:
    p = path.strip()
    if not p:
        return p
    if not os.path.isabs(p):
        p = os.path.join(_resolve_ws_root(), p)
    return os.path.abspath(os.path.realpath(p))


def _resolve_session_dir(context) -> str:
    session_arg = context.perform_substitution(
        LaunchConfiguration('session_dir')).strip()
    if session_arg:
        p = session_arg
        if not os.path.isabs(p):
            p = os.path.join(_resolve_ws_root(), p)
        return os.path.abspath(p)
    return os.path.join(
        _resolve_ws_root(), 'src', 'perception_pkg', 'maps', 'map_latest')


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('navigation_pkg')
    perception_share = get_package_share_directory('perception_pkg')

    session_dir = _resolve_session_dir(context)
    map_yaml = context.perform_substitution(LaunchConfiguration('map_yaml_file')).strip()
    initial_pose = context.perform_substitution(
        LaunchConfiguration('initial_pose_file')).strip()
    waypoints_file = context.perform_substitution(
        LaunchConfiguration('waypoints_file')).strip()

    if not map_yaml:
        map_yaml = os.path.join(session_dir, 'slam_map.yaml')
    if not initial_pose:
        initial_pose = os.path.join(session_dir, 'initial_pose.yaml')
    if not waypoints_file:
        waypoints_file = os.path.join(session_dir, 'waypoints.yaml')

    map_yaml = _abs_path(map_yaml)
    initial_pose = _abs_path(initial_pose)
    waypoints_file = _abs_path(waypoints_file)
    session_dir = _abs_path(session_dir)

    use_sim_time = LaunchConfiguration('use_sim')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    start_localization = LaunchConfiguration('start_localization')
    rviz = LaunchConfiguration('rviz')
    log_level = LaunchConfiguration('log_level')
    start_patrol = LaunchConfiguration('start_patrol')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    nav_cmd_remappings = remappings + [('/cmd_vel', '/cmd_vel_nav')]
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    actions = []
    actions.append(LogInfo(
        msg=(
            f'[navigation_humble] session={session_dir} '
            f'map={map_yaml} initial_pose={initial_pose}'
        ),
    ))

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'perception_humble.launch.py')
        ),
        condition=IfCondition(start_localization),
        launch_arguments={
            'use_sim': use_sim_time,
            'mode': 'localize',
            'map_yaml_file': map_yaml,
            'session_dir': session_dir,
            'initial_pose_file': initial_pose,
            'auto_save_map': 'false',
        }.items(),
    )
    actions.append(localization_launch)

    for pkg, exe, name, node_remappings in (
        ('nav2_controller', 'controller_server', 'controller_server', nav_cmd_remappings),
        ('nav2_smoother', 'smoother_server', 'smoother_server', remappings),
        ('nav2_planner', 'planner_server', 'planner_server', remappings),
        ('nav2_behaviors', 'behavior_server', 'behavior_server', nav_cmd_remappings),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', remappings),
        ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', remappings),
    ):
        actions.append(Node(
            package=pkg,
            executable=exe,
            name=name,
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=node_remappings,
        ))

    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes},
        ],
    ))

    actions.append(Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    ))

    actions.append(Node(
        condition=IfCondition(start_patrol),
        package='navigation_pkg',
        executable='waypoint_patrol.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--session-dir', session_dir],
    ))

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('navigation_pkg')
    ws_root = _resolve_ws_root()
    default_session = os.path.join(ws_root, 'src', 'perception_pkg', 'maps', 'map_latest')
    default_map = os.path.join(default_session, 'slam_map.yaml')
    default_pose = os.path.join(default_session, 'initial_pose.yaml')
    default_wps = os.path.join(default_session, 'waypoints.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('start_localization', default_value='true'),
        DeclareLaunchArgument('start_patrol', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'session_dir',
            default_value=default_session,
            description='Map session directory (map_latest)',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument('map_yaml_file', default_value=default_map),
        DeclareLaunchArgument('initial_pose_file', default_value=default_pose),
        DeclareLaunchArgument('waypoints_file', default_value=default_wps),
        OpaqueFunction(function=_launch_setup),
    ])
