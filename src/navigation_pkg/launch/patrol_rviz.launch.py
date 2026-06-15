"""RViz + waypoint patrol only (Nav2 must already be running)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_ws_root() -> str:
    pkg_share = get_package_share_directory('navigation_pkg')
    return os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))


def _resolve_session_dir(context) -> str:
    session_arg = context.perform_substitution(LaunchConfiguration('session_dir')).strip()
    if session_arg:
        p = session_arg
        if not os.path.isabs(p):
            p = os.path.join(_resolve_ws_root(), p)
        return os.path.abspath(p)
    return os.path.join(_resolve_ws_root(), 'src', 'perception_pkg', 'maps', 'map_latest')


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('navigation_pkg')
    session_dir = _resolve_session_dir(context)
    use_sim = context.perform_substitution(LaunchConfiguration('use_sim')).lower() == 'true'

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'navigation.rviz')],
            parameters=[{'use_sim_time': use_sim}],
        ),
        TimerAction(
            period=25.0,
            actions=[Node(
                package='navigation_pkg',
                executable='waypoint_patrol.py',
                output='screen',
                parameters=[{'use_sim_time': use_sim}],
                arguments=['--session-dir', session_dir],
            )],
        ),
    ]


def generate_launch_description():
    ws_root = _resolve_ws_root()
    default_session = os.path.join(ws_root, 'src', 'perception_pkg', 'maps', 'map_latest')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('session_dir', default_value=default_session),
        OpaqueFunction(function=_launch_setup),
    ])
