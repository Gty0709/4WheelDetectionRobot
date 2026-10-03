"""teleop_slider_cpp.launch.py — 滑条遥控（tkinter GUI，与 Python 版一致）。

C++ 可执行文件 slider_teleop_cpp 为 ncurses 终端遥控，无图形窗口。
本 launch 默认启动 slider_teleop.py 滑条 GUI；SSH 无 DISPLAY 时用 interface:=terminal。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    save_map_on_exit = LaunchConfiguration('save_map_on_exit')
    use_sim_time = LaunchConfiguration('use_sim_time')
    interface = LaunchConfiguration('interface')

    gui_params = [{
        'save_map_on_exit': save_map_on_exit,
        'use_sim_time': use_sim_time,
    }]
    remappings = [('/cmd_vel_teleop', cmd_vel_topic)]

    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_teleop'),
        DeclareLaunchArgument('save_map_on_exit', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'interface',
            default_value='gui',
            description='gui=tkinter 滑条窗口; terminal=ncurses 键盘遥控 (slider_teleop_cpp)',
        ),
        LogInfo(
            condition=IfCondition(PythonExpression(["'", interface, "' == 'gui'"])),
            msg='[teleop_slider_cpp] 启动滑条 GUI（slider_teleop.py）',
        ),
        LogInfo(
            condition=IfCondition(PythonExpression(["'", interface, "' == 'terminal'"])),
            msg='[teleop_slider_cpp] 启动终端遥控（slider_teleop_cpp，w/a/s/d + q 退出）',
        ),
        Node(
            condition=IfCondition(PythonExpression(["'", interface, "' == 'gui'"])),
            package='perception_pkg',
            executable='slider_teleop.py',
            name='slider_teleop',
            output='screen',
            parameters=gui_params,
            remappings=remappings,
        ),
        Node(
            condition=IfCondition(PythonExpression(["'", interface, "' == 'terminal'"])),
            package='perception_pkg',
            executable='slider_teleop_cpp',
            name='slider_teleop',
            output='screen',
            parameters=gui_params,
            remappings=remappings,
        ),
    ])
