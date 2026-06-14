"""启动 cmd_vel_guard：/cmd_vel_nav + /cmd_vel_teleop → /cmd_vel（默认零速）"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_pkg',
            executable='cmd_vel_guard.py',
            name='cmd_vel_guard',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
