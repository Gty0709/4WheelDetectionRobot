"""
humble_sim_slam_cpp.launch.py — C++ variant of humble_sim_slam.launch.py

Uses perception_humble_cpp.launch.py and slider_teleop_cpp.
"""
import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_maps_dir(pkg_share: str) -> str:
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    src_maps = os.path.join(ws_root, 'src', 'perception_pkg', 'maps')
    if os.path.isdir(src_maps):
        return src_maps
    return os.path.join(pkg_share, 'maps')


_BAG_TOPICS = [
    '/scan', '/odom', '/wheel/odom', '/imu/data', '/camera/imu/data',
    '/camera/left/image_raw', '/camera/left/camera_info',
    '/camera/right/image_raw', '/camera/right/camera_info',
    '/map', '/tf', '/tf_static', '/cmd_vel', '/cmd_vel_teleop', '/clock',
]


def _setup(context, *args, **kwargs):
    a_share = FindPackageShare('mickrobot_description').find('mickrobot_description')
    b_share = FindPackageShare('perception_pkg').find('perception_pkg')
    maps_dir = _resolve_maps_dir(b_share)
    ws_root = os.path.abspath(os.path.join(b_share, '..', '..', '..', '..'))
    save_script = os.path.join(ws_root, 'src', 'perception_pkg', 'scripts', 'save_map_snapshot.py')
    rviz_config = os.path.join(b_share, 'config', 'slam_classic.rviz')

    use_sim = context.perform_substitution(LaunchConfiguration('use_sim')).lower() == 'true'
    world = context.perform_substitution(LaunchConfiguration('world'))
    gui = context.perform_substitution(LaunchConfiguration('gui'))
    slam_delay = float(context.perform_substitution(LaunchConfiguration('slam_delay')))
    rviz_delay = float(context.perform_substitution(LaunchConfiguration('rviz_delay')))
    teleop_delay = float(context.perform_substitution(LaunchConfiguration('teleop_delay')))
    launch_rviz = context.perform_substitution(LaunchConfiguration('launch_rviz')).lower() == 'true'
    launch_teleop = context.perform_substitution(LaunchConfiguration('launch_teleop')).lower() == 'true'
    record_bag = context.perform_substitution(LaunchConfiguration('record_bag')).lower() == 'true'
    record_db = context.perform_substitution(LaunchConfiguration('record_db')).lower() == 'true'
    do_record = record_bag or record_db

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    session_id = f'map_{stamp}'
    session_dir = os.path.join(maps_dir, session_id)
    os.makedirs(session_dir, exist_ok=True)
    map_latest = os.path.join(maps_dir, 'map_latest')
    if os.path.islink(map_latest) or os.path.exists(map_latest):
        os.unlink(map_latest)
    os.symlink(session_id, map_latest)
    bag_path = os.path.join(session_dir, 'bag')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(a_share, 'launch', 'bringup_classic.launch.py')
        ),
        launch_arguments={
            'use_sim': str(use_sim).lower(),
            'world': world,
            'gui': gui,
            'launch_rviz': 'false',
            # 新建图会话尚无 initial_pose.yaml，不能用 prior spawn
            'spawn_mode': 'fixed',
        }.items(),
    )

    perception_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(b_share, 'launch', 'perception_humble_cpp.launch.py')
        ),
        launch_arguments={
            'use_sim': str(use_sim).lower(),
            'mode': 'slam',
            'auto_save_map': 'true',
            'session_dir': session_dir,
            'maps_dir': maps_dir,
        }.items(),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(b_share, 'launch', 'teleop_slider_cpp.launch.py')
        ),
        launch_arguments={'save_map_on_exit': 'true'}.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim}],
    )

    shutdown_save = RegisterEventHandler(
        OnShutdown(on_shutdown=[
            LogInfo(msg=f'[humble_sim_slam_cpp] 退出兜底：从缓存保存到 {session_dir}'),
            ExecuteProcess(
                cmd=[
                    'python3', save_script,
                    '--from-cache', '--maps-dir', maps_dir,
                    '--session-dir', session_dir,
                ],
                output='screen',
            ),
        ]),
    )

    teleop_hint = (
        '  遥控: ros2 launch perception_pkg teleop_slider_cpp.launch.py\n'
        if not launch_teleop
        else '  遥控: 已随本 launch 自动启动滑条窗口 (C++)\n'
    )

    actions = [
        LogInfo(msg=(
            '\n'
            '════════════════════════════════════════════════════════\n'
            '  手动建图（仿真 + SLAM）— C++ 节点\n'
            f'  会话目录: {session_dir}\n'
            f'  录包: {"开 → " + bag_path if do_record else "关 (record_bag:=true 开启)"}\n'
            f'{teleop_hint}'
            '  RViz: ros2 launch perception_pkg rviz_slam_cpp.launch.py\n'
            '  终端4: ros2 launch detection_pkg detection_cpp.launch.py '
            f'session_dir:={map_latest}\n'
            '  Ctrl-C 退出时自动保存地图\n'
            '════════════════════════════════════════════════════════\n'
        )),
        shutdown_save,
        bringup,
        TimerAction(period=slam_delay, actions=[perception_slam]),
    ]

    if do_record:
        actions.append(
            TimerAction(
                period=slam_delay,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'bag', 'record', '-o', bag_path, *_BAG_TOPICS],
                        output='screen',
                        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
                    ),
                ],
            )
        )

    if launch_rviz:
        actions.append(TimerAction(period=rviz_delay, actions=[rviz]))

    if launch_teleop:
        actions.append(TimerAction(period=teleop_delay, actions=[teleop]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('world', default_value='small_house'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('slam_delay', default_value='8.0'),
        DeclareLaunchArgument('rviz_delay', default_value='12.0'),
        DeclareLaunchArgument('teleop_delay', default_value='14.0'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('launch_teleop', default_value='false'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('record_db', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
