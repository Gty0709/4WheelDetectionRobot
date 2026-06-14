"""
auto_slam.launch.py — 一键全自动建图

  Gazebo + SLAM + RViz + 贴墙巡逻 + 地图保存 + 可选录包

用法：
  ros2 launch perception_pkg auto_slam.launch.py
  ros2 launch perception_pkg auto_slam.launch.py record_bag:=true
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
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_maps_dir(pkg_share: str, subdir: str = '') -> str:
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    src_maps = os.path.join(ws_root, 'src', 'perception_pkg', 'maps')
    if not os.path.isdir(src_maps):
        src_maps = os.path.join(pkg_share, 'maps')
    if subdir:
        return os.path.join(src_maps, subdir)
    return src_maps


# 录包话题
_BAG_TOPICS_CORE = [
    '/scan', '/odom', '/wheel/odom', '/imu/data', '/camera/imu/data',
    '/camera/left/image_raw', '/camera/left/camera_info',
    '/camera/right/image_raw', '/camera/right/camera_info',
    '/map', '/tf', '/tf_static', '/cmd_vel', '/clock',
]
# 双目相机（Gazebo Classic namespace=camera + camera_name=left|right）
_BAG_TOPICS_CAMERA = [
    '/camera/left/image_raw',
    '/camera/left/camera_info',
    '/camera/right/image_raw',
    '/camera/right/camera_info',
]


def _setup(context, *args, **kwargs):
    a_share = FindPackageShare('mickrobot_description').find('mickrobot_description')
    b_share = FindPackageShare('perception_pkg').find('perception_pkg')

    use_sim = context.perform_substitution(LaunchConfiguration('use_sim')).lower() == 'true'
    world = context.perform_substitution(LaunchConfiguration('world'))
    gui = context.perform_substitution(LaunchConfiguration('gui'))
    slam_delay = float(context.perform_substitution(LaunchConfiguration('slam_delay')))
    rviz_delay = float(context.perform_substitution(LaunchConfiguration('rviz_delay')))
    patrol_delay = float(context.perform_substitution(LaunchConfiguration('patrol_delay')))
    save_on_patrol = (
        context.perform_substitution(LaunchConfiguration('save_on_patrol_done')).lower() == 'true'
    )
    record_bag = context.perform_substitution(LaunchConfiguration('record_bag')).lower() == 'true'
    record_db = context.perform_substitution(LaunchConfiguration('record_db')).lower() == 'true'
    record_camera = (
        context.perform_substitution(LaunchConfiguration('record_camera')).lower() == 'true'
    )
    maps_subdir = context.perform_substitution(LaunchConfiguration('maps_subdir'))
    replay_bag = context.perform_substitution(LaunchConfiguration('replay_cmd_vel_bag')).strip()
    do_record = record_bag or record_db
    use_replay = bool(replay_bag)

    maps_dir = _resolve_maps_dir(b_share, maps_subdir)
    ws_root = os.path.abspath(os.path.join(b_share, '..', '..', '..', '..'))
    save_script = os.path.join(ws_root, 'src', 'perception_pkg', 'scripts', 'save_map_snapshot.py')
    rviz_config = os.path.join(b_share, 'config', 'slam_classic.rviz')
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
        }.items(),
    )

    perception_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(b_share, 'launch', 'perception_humble.launch.py')
        ),
        launch_arguments={
            'use_sim': str(use_sim).lower(),
            'mode': 'slam',
            'auto_save_map': 'true',
            'maps_dir': maps_dir,
            'session_dir': session_dir,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim}],
    )

    patrol = Node(
        package='perception_pkg',
        executable='patrol_mapper.py',
        name='patrol_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'save_map_on_finish': save_on_patrol,
        }],
    )

    replay_env = {
        'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0'),
        'ROS_USE_SIM_TIME': '1' if use_sim else '0',
    }
    replay_drive = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            (
                f'ros2 bag play "{replay_bag}" --topics /cmd_vel --rate 1.0 && '
                'ros2 service call /map_snapshot_saver/save_map std_srvs/srv/Trigger "{}" '
                '&& echo "[auto_slam] cmd_vel 回放完成，地图已保存"'
            ),
        ],
        output='screen',
        additional_env=replay_env,
    )

    # Ctrl-C 兜底：从 _cache 提升地图（不依赖 ROS 图）
    shutdown_save = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(msg=f'[auto_slam] 退出兜底：从缓存保存到 {session_dir}'),
                ExecuteProcess(
                    cmd=[
                        'python3', save_script,
                        '--from-cache', '--maps-dir', maps_dir,
                        '--session-dir', session_dir,
                    ],
                    output='screen',
                ),
            ],
        )
    )

    drive_desc = (
        f'  驱动: 回放 cmd_vel ← {replay_bag}\n'
        if use_replay else
        '  巡逻: 贴墙路径，避开四柱\n'
    )
    actions = [
        LogInfo(msg=(
            '\n'
            '════════════════════════════════════════════════════════\n'
            '  全自动建图\n'
            f'  会话目录: {session_dir}\n'
            f'  录包: {"开 → " + bag_path if do_record else "关 (record_bag:=true 开启)"}\n'
            f'  相机: {"左/右目 /camera/left|right image_raw + camera_info" if record_camera else "未录"}\n'
            f'{drive_desc}'
            '════════════════════════════════════════════════════════\n'
        )),
        shutdown_save,
        bringup,
        TimerAction(period=slam_delay, actions=[perception_slam]),
    ]

    if do_record:
        bag_topics = list(_BAG_TOPICS_CORE)
        if record_camera:
            bag_topics.extend(_BAG_TOPICS_CAMERA)
        actions.append(
            TimerAction(
                period=slam_delay,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'bag', 'record', '-o', bag_path, *bag_topics],
                        output='screen',
                        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
                    ),
                ],
            )
        )

    drive_timer = TimerAction(
        period=patrol_delay,
        actions=[replay_drive if use_replay else patrol],
    )
    actions += [
        TimerAction(period=rviz_delay, actions=[rviz]),
        drive_timer,
    ]
    if use_replay:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=replay_drive,
                    on_exit=[LogInfo(msg='[auto_slam] 参考 bag cmd_vel 回放结束')],
                )
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('world', default_value='small_house'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('slam_delay', default_value='8.0'),
        DeclareLaunchArgument('rviz_delay', default_value='12.0'),
        DeclareLaunchArgument('patrol_delay', default_value='22.0'),
        DeclareLaunchArgument('save_on_patrol_done', default_value='true'),
        DeclareLaunchArgument(
            'record_bag', default_value='true',
            description='全自动建图时录制 rosbag 到 maps/map_<时间戳>/bag/',
        ),
        DeclareLaunchArgument(
            'record_db', default_value='true',
            description='record_bag 的别名，任一为 true 即录包',
        ),
        DeclareLaunchArgument(
            'maps_subdir', default_value='',
            description='maps 子目录，如 new_with_camera',
        ),
        DeclareLaunchArgument(
            'record_camera', default_value='false',
            description='录包时追加双目 /camera/left|right image_raw + camera_info',
        ),
        DeclareLaunchArgument(
            'replay_cmd_vel_bag',
            default_value='',
            description='非空则回放该 bag 的 /cmd_vel（复现相同路径），不启 patrol_mapper',
        ),
        OpaqueFunction(function=_setup),
    ])
