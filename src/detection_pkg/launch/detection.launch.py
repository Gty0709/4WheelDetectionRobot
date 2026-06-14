"""Launch clip_detector_node for mapping session."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _default_session_dir() -> str:
    """Prefer workspace src/.../maps/map_latest (updated by humble_sim_slam)."""
    try:
        p_share = get_package_share_directory('perception_pkg')
        ws_root = os.path.abspath(os.path.join(p_share, '..', '..', '..', '..'))
        candidate = os.path.join(ws_root, 'src', 'perception_pkg', 'maps', 'map_latest')
        return candidate
    except Exception:
        return 'src/perception_pkg/maps/map_latest'


def generate_launch_description():
    pkg_share = get_package_share_directory('detection_pkg')
    params_file = os.path.join(pkg_share, 'config', 'detection_params.yaml')
    use_sim = LaunchConfiguration('use_sim_time')
    default_session = _default_session_dir()

    return LaunchDescription([
        DeclareLaunchArgument(
            'session_dir',
            default_value=default_session,
            description='建图会话目录；默认 map_latest（须先启动终端1 humble_sim_slam）',
        ),
        DeclareLaunchArgument('weights', default_value=''),
        DeclareLaunchArgument('conf_threshold', default_value='0.5'),
        DeclareLaunchArgument('max_fps', default_value='8.0'),
        DeclareLaunchArgument('ground_z', default_value='0.002'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真建图时必须为 true（与 Gazebo /clock 一致）',
        ),
        GroupAction([
            SetParameter(name='use_sim_time', value=use_sim),
            Node(
                package='detection_pkg',
                executable='clip_detector_wrapper.sh',
                name='clip_detector',
                output='screen',
                parameters=[
                    params_file,
                    {
                        'session_dir': LaunchConfiguration('session_dir'),
                        'weights': LaunchConfiguration('weights'),
                        'conf_threshold': LaunchConfiguration('conf_threshold'),
                        'max_fps': LaunchConfiguration('max_fps'),
                        'ground_z': LaunchConfiguration('ground_z'),
                        'use_sim_time': use_sim,
                    },
                ],
            ),
        ]),
    ])
