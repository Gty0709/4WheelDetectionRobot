"""
save_map.launch.py — 手动立即保存当前 /map（带时间戳 + PNG 预览）

SLAM 模式已默认在 Ctrl-C 退出时自动保存；本 launch 用于建图中途额外存盘。
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_maps_dir(pkg_share: str) -> str:
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    src_maps = os.path.join(ws_root, 'src', 'perception_pkg', 'maps')
    if os.path.isdir(src_maps):
        return src_maps
    return os.path.join(pkg_share, 'maps')


def _launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('perception_pkg').find('perception_pkg')
    maps_dir = _resolve_maps_dir(pkg_share)
    use_sim = context.perform_substitution(LaunchConfiguration('use_sim')).lower() == 'true'

    return [
        LogInfo(msg=(
            f'[save_map] 等待 /map 后保存到 {maps_dir}/'
            'slam_map_<timestamp>.{{yaml,pgm,png}}'
        )),
        Node(
            package='perception_pkg',
            executable='save_map_snapshot.py',
            name='map_snapshot_saver',
            output='screen',
            arguments=['--once'],
            parameters=[{
                'use_sim_time': use_sim,
                'maps_dir': maps_dir,
                'filename_prefix': 'slam_map',
                'update_latest_symlink': True,
                'save_on_shutdown': False,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='与 SLAM launch 保持一致',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
