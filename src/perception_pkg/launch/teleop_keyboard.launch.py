"""
teleop_keyboard.launch.py — 已弃用，请使用 teleop_slider.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg = FindPackageShare('perception_pkg').find('perception_pkg')
    return LaunchDescription([
        LogInfo(msg='[WARN] teleop_keyboard 已弃用，自动启动 teleop_slider'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'teleop_slider.launch.py')
            ),
        ),
    ])
