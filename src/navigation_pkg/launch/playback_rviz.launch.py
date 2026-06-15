"""RViz + map_server + robot model + mission trajectory playback."""

from __future__ import annotations

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

_CLEANUP_CMD = (
    'pkill -f "[p]layback_mission.py" 2>/dev/null || true; '
    'pkill -f "[r]viz2.*playback.rviz" 2>/dev/null || true; '
    'sleep 0.3'
)


def _resolve_ws_root() -> str:
    pkg_share = get_package_share_directory('navigation_pkg')
    return os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))


def _resolve_mission_dir(context) -> str:
    mission_arg = context.perform_substitution(LaunchConfiguration('mission')).strip()
    if mission_arg:
        path = mission_arg
        if not os.path.isabs(path):
            path = os.path.join(_resolve_ws_root(), path)
        return os.path.abspath(path)
    latest = os.path.join(
        _resolve_ws_root(), 'src', 'navigation_pkg', 'result', 'path_latest',
    )
    return os.path.realpath(latest)


def _resolve_map_yaml(mission_dir: str) -> str:
    meta_path = os.path.join(mission_dir, 'session_meta.json')
    if os.path.isfile(meta_path):
        with open(meta_path, encoding='utf-8') as f:
            meta = json.load(f) or {}
        map_dir = str(meta.get('map_session_dir', '')).strip()
        if map_dir:
            candidate = os.path.join(map_dir, 'slam_map.yaml')
            if os.path.isfile(candidate):
                return os.path.abspath(candidate)
    progress_path = os.path.join(mission_dir, 'progress.yaml')
    if os.path.isfile(progress_path):
        import yaml
        with open(progress_path, encoding='utf-8') as f:
            progress = yaml.safe_load(f) or {}
        map_dir = str(progress.get('map_session_dir', '')).strip()
        if map_dir:
            candidate = os.path.join(map_dir, 'slam_map.yaml')
            if os.path.isfile(candidate):
                return os.path.abspath(candidate)
    default = os.path.join(
        _resolve_ws_root(), 'src', 'perception_pkg', 'maps', 'map_latest', 'slam_map.yaml',
    )
    return os.path.abspath(default)


def _launch_setup(context, *args, **kwargs):
    nav_share = get_package_share_directory('navigation_pkg')
    robot_share = get_package_share_directory('mickrobot_description')
    mission_dir = _resolve_mission_dir(context)
    map_yaml = _resolve_map_yaml(mission_dir)
    rate = context.perform_substitution(LaunchConfiguration('rate'))
    loop = context.perform_substitution(LaunchConfiguration('loop')).lower() == 'true'
    urdf_path = os.path.join(robot_share, 'urdf', 'mickrobot_ugv_classic.urdf.xacro')

    playback_args = ['--mission', mission_dir, '--rate', rate]
    if loop:
        playback_args.append('--loop')

    playback_node = Node(
        package='navigation_pkg',
        executable='playback_mission.py',
        output='screen',
        arguments=playback_args,
    )

    cleanup = ExecuteProcess(cmd=['bash', '-c', _CLEANUP_CMD], output='log')

    session_nodes = [
        LogInfo(msg=f'[playback_rviz] mission={mission_dir} map={map_yaml} rate={rate}x'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='playback_map_server',
            output='screen',
            parameters=[{'use_sim_time': False, 'yaml_filename': map_yaml}],
        ),
        TimerAction(period=0.8, actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='playback_lifecycle_manager',
                output='screen',
                parameters=[
                    {'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['playback_map_server']},
                    {'attempt_respawn_reconnection': False},
                ],
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'robot_description': Command(['xacro ', urdf_path]),
                }],
            ),
            playback_node,
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(nav_share, 'rviz', 'playback.rviz')],
                parameters=[{'use_sim_time': False}],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=playback_node,
                    on_exit=[Shutdown(reason='Playback session ended')],
                ),
            ),
        ]),
    ]

    return [
        cleanup,
        RegisterEventHandler(
            OnProcessExit(
                target_action=cleanup,
                on_exit=[TimerAction(period=0.2, actions=session_nodes)],
            ),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mission', default_value='',
                              description='Mission dir; default path_latest'),
        DeclareLaunchArgument('rate', default_value='2.0'),
        DeclareLaunchArgument('loop', default_value='false',
                              description='Loop playback (session stays open)'),
        OpaqueFunction(function=_launch_setup),
    ])
