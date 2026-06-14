import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _launch_record(context, *args, **kwargs):
    use_sim  = context.perform_substitution(LaunchConfiguration('use_sim')).lower() == 'true'
    bag_path = context.perform_substitution(LaunchConfiguration('bag_path'))

    topics = [
        '/scan',
        '/odom',
        '/imu/data',
        '/camera/imu/data',
        '/camera/left/image_raw',
        '/camera/left/camera_info',
        '/camera/right/image_raw',
        '/camera/right/camera_info',
        '/tf',
        '/tf_static',
    ]
    if use_sim:
        topics.append('/clock')

    cmd = ['ros2', 'bag', 'record', '--storage', 'mcap', '-o', bag_path, '--topics'] + topics

    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    pkg_share   = FindPackageShare('perception_pkg').find('perception_pkg')
    timestamp   = datetime.now().strftime('%Y%m%d_%H%M%S')
    default_bag = os.path.join(pkg_share, 'maps', f'slam_bag_{timestamp}')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='true=仿真（额外录 /clock），false=实体机器人',
        ),
        DeclareLaunchArgument(
            'bag_path',
            default_value=default_bag,
            description='录包输出目录（默认含时间戳，避免覆盖历史包）',
        ),
        OpaqueFunction(function=_launch_record),
    ])
