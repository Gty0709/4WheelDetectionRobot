import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('perception_pkg').find('perception_pkg')

    bag_path = LaunchConfiguration('bag_path')
    db_path  = LaunchConfiguration('db_path')
    map_path = LaunchConfiguration('map_path')

    # ================================================================
    # RTAB-Map：LiDAR + 轮式里程计模式（不依赖相机）
    # 输入：/scan（激光雷达）+ /odom（bag 里的 Gazebo 轮式里程计）
    # 优点：无相机同步问题，离线建图稳定可靠
    # ================================================================
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=['--delete_db_on_start'],
        parameters=[{
            'use_sim_time':             True,
            'frame_id':                 'base_footprint',
            'odom_frame_id':            'odom',
            'map_frame_id':             'map',
            'publish_tf':               True,
            # 不用相机，只用激光雷达
            'subscribe_stereo':         False,
            'subscribe_rgb':            False,
            'subscribe_depth':          False,
            'subscribe_scan':           True,
            'subscribe_odom_info':      False,
            'approx_sync':              True,
            'approx_sync_max_interval': 1.0,    # scan 约 1.5Hz，odom 约 4Hz，给宽松的同步窗口
            'database_path':            db_path,
            # 建图模式
            'Mem/IncrementalMemory':    'true',
            'Mem/InitWMWithAllNodes':   'false',
            'Mem/NotLinkedNodesKept':   'false',
            'Mem/STMSize':              '30',
            # 栅格地图（激光雷达）
            'Grid/Sensor':              '0',
            'Grid/RayTracing':          'true',
            'Grid/3D':                  'false',
            'Grid/CellSize':            '0.05',
            # 回环检测（基于扫描匹配）
            'RGBD/ProximityBySpace':    'true',
            'RGBD/LinearUpdate':        '0.1',
            'RGBD/AngularUpdate':       '0.1',
            'Rtabmap/DetectionRate':    '0',    # 每帧都检测（离线不限速）
        }],
        remappings=[
            ('odom', '/odom'),    # 直接用 bag 里的轮式里程计
            ('scan', '/scan'),
        ],
    )

    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock',
            '--delay', '3.0',   # 3 秒足够 rtabmap 启动（无需等 stereo_odometry）
            '--rate', '0.5',    # 无相机处理压力，0.5 倍速即可
        ],
        output='screen',
        name='bag_play',
    )

    # bag 播放结束后等 3 秒（RTAB-Map 完成最终优化），再保存地图
    save_map_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                                '-t', '/map',
                                '-f', map_path,
                                '--occ', '65',
                                '--free', '25',
                                '--fmt', 'pgm',
                            ],
                            output='screen',
                        ),
                    ],
                ),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value=os.path.join(pkg_share, 'maps', 'slam_bag'),
            description='bag 目录路径（包含 metadata.yaml）',
        ),
        DeclareLaunchArgument(
            'db_path',
            default_value=os.path.join(pkg_share, 'maps', 'rtabmap.db'),
            description='RTAB-Map 数据库输出路径',
        ),
        DeclareLaunchArgument(
            'map_path',
            default_value=os.path.join(pkg_share, 'maps', 'slam_map'),
            description='地图输出路径前缀（生成 slam_map.pgm + slam_map.yaml）',
        ),
        rtabmap_node,
        bag_play,
        save_map_on_exit,
    ])
