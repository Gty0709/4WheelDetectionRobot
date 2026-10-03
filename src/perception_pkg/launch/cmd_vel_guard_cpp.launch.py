"""cmd_vel_guard C++ launch -- identical to Python version but using C++ node."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_pkg',
            executable='cmd_vel_guard_cpp',
            name='cmd_vel_guard',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
