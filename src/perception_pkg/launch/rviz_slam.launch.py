"""
rviz_slam.launch.py — 建图 RViz + 可选检测 RViz

slam_classic.rviz 不含 Image/MarkerArray（与部分 GPU 驱动组合会 segfault）。
检测可视化由独立 detection.rviz 窗口承担。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    p_share = FindPackageShare('perception_pkg').find('perception_pkg')
    d_share = FindPackageShare('detection_pkg').find('detection_pkg')
    slam_rviz = os.path.join(p_share, 'config', 'slam_classic.rviz')
    det_rviz = os.path.join(d_share, 'config', 'detection.rviz')

    use_sim = LaunchConfiguration('use_sim')
    with_detection = LaunchConfiguration('with_detection')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument(
            'with_detection', default_value='true',
            description='另开 detection.rviz（地图航点 + 检测图）',
        ),
        LogInfo(msg=(
            '[rviz_slam] 窗口1: slam_classic (map/scan/robot); '
            '窗口2: detection.rviz (检测图+航点，无 RobotModel/LaserScan); '
            'with_detection:=false 可关'
        )),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_slam',
            output='screen',
            arguments=['-d', slam_rviz],
            parameters=[{'use_sim_time': use_sim}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_detection',
            output='screen',
            arguments=['-d', det_rviz],
            parameters=[{'use_sim_time': use_sim}],
            condition=IfCondition(with_detection),
        ),
    ])
