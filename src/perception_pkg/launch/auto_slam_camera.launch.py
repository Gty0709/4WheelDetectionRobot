"""
auto_slam_camera.launch.py — 回放参考 bag 路径 + 双目相机录包

复现 maps/bags/slam_20260613_151154 的 /cmd_vel 轨迹，新包写入 maps/new_with_camera/。

用法：
  ros2 launch perception_pkg auto_slam_camera.launch.py
  bash scripts/auto_build_map_camera.sh
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _resolve_maps_dir(pkg_share: str, subdir: str = '') -> str:
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    src_maps = os.path.join(ws_root, 'src', 'perception_pkg', 'maps')
    if not os.path.isdir(src_maps):
        src_maps = os.path.join(pkg_share, 'maps')
    if subdir:
        return os.path.join(src_maps, subdir)
    return src_maps


def generate_launch_description():
    b_share = FindPackageShare('perception_pkg').find('perception_pkg')
    maps_subdir = LaunchConfiguration('maps_subdir')
    replay_bag = LaunchConfiguration('replay_cmd_vel_bag')
    default_replay = os.path.join(
        _resolve_maps_dir(b_share),
        'bags', 'slam_20260613_151154',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'maps_subdir', default_value='new_with_camera',
            description='maps 下子目录名',
        ),
        DeclareLaunchArgument(
            'replay_cmd_vel_bag', default_value=default_replay,
            description='回放 /cmd_vel 的参考 rosbag 目录',
        ),
        LogInfo(msg=(
            '\n'
            '════════════════════════════════════════════════════════\n'
            '  回放参考 bag 路径 + 双目录包 → maps/new_with_camera/\n'
            f'  参考: {default_replay}\n'
            '  录包含 /camera/left|right image_raw + camera_info\n'
            '════════════════════════════════════════════════════════\n'
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(b_share, 'launch', 'auto_slam.launch.py')
            ),
            launch_arguments={
                'maps_subdir': maps_subdir,
                'record_camera': 'true',
                'record_bag': 'true',
                'record_db': 'true',
                'replay_cmd_vel_bag': replay_bag,
                'save_on_patrol_done': 'false',
            }.items(),
        ),
    ])
