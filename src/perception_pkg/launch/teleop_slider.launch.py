"""
teleop_slider.launch.py — 滑条遥控 /cmd_vel（差速运动学解算轮速）

关闭窗口时可选调用 save_map 服务（需 SLAM 侧已启动 map_snapshot_saver）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    save_map_on_exit = LaunchConfiguration('save_map_on_exit')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel_teleop',
            description='滑条输出话题（由 cmd_vel_guard 转发到 /cmd_vel）',
        ),
        DeclareLaunchArgument(
            'save_map_on_exit', default_value='true',
            description='关闭遥控窗口时调用 /map_snapshot_saver/save_map',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真建图时应为 true',
        ),
        LogInfo(msg=(
            '[teleop_slider] 滑条窗口：调节 v / ω；'
            '退出时 save_map_on_exit 为 true 则保存地图'
        )),
        Node(
            package='perception_pkg',
            executable='slider_teleop.py',
            name='slider_teleop',
            output='screen',
            parameters=[{
                'save_map_on_exit': save_map_on_exit,
                'use_sim_time': use_sim_time,
            }],
            remappings=[('/cmd_vel_teleop', cmd_vel_topic)],
        ),
    ])
