"""
bringup_classic.launch.py — Ubuntu 22.04 + Humble + Gazebo Classic 11

[Humble迁移] 与 bringup.launch.py（gz-sim / Jazzy 24.04）并行，不修改原文件。
默认世界：small_house（Classic 版 worlds/small_house.world）
TaskA v5 机器人模型：mickrobot_ugv_classic.urdf.xacro
"""
import os
import subprocess
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _gpu_env_actions():
    """[Humble迁移] 默认 NVIDIA GPU 渲染；勿设 LIBGL_ALWAYS_SOFTWARE=1"""
    return [
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '0'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__VK_LAYER_NV_optimus', 'NVIDIA_only'),
        SetEnvironmentVariable('CUDA_DEVICE_ORDER', 'PCI_BUS_ID'),
        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', os.environ.get('CUDA_VISIBLE_DEVICES', '0')),
    ]


def _run_gazebo_preflight():
    """清理占用 11345 端口的僵尸 gzserver，避免 exit 255"""
    ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    script = os.path.join(ws_root, 'scripts', 'gazebo_preflight.sh')
    if os.path.isfile(script):
        subprocess.run(['bash', script], check=False)


def _gazebo_env_actions(extra_resource_paths=None):
    """保留系统 Gazebo 媒体/着色器路径；此前覆盖 GAZEBO_RESOURCE_PATH 导致 RenderEngine 初始化失败"""
    gazebo_share = '/usr/share/gazebo-11'
    if not os.path.isdir(gazebo_share):
        gazebo_share = '/usr/share/gazebo'

    paths = [gazebo_share]
    if extra_resource_paths:
        for p in extra_resource_paths:
            if p and os.path.isdir(p):
                paths.append(p)

    existing = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    if existing:
        for part in existing.split(':'):
            if part and part not in paths:
                paths.append(part)

    resource_path = ':'.join(paths)

    return [
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', resource_path),
    ]


def launch_setup(context, *args, **kwargs):
    _run_gazebo_preflight()
    package_name = 'mickrobot_description'
    robot_name_in_model = 'mickrobot'
    urdf_classic_name = 'mickrobot_ugv_classic.urdf.xacro'

    try:
        pkg_share = FindPackageShare(package=package_name).find(package_name)
    except Exception:
        pkg_share = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_classic_name}')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_gazebo.rviz')

    use_sim_val = context.perform_substitution(LaunchConfiguration('use_sim'))
    use_sim_bool = use_sim_val.lower() == 'true'
    launch_rviz_val = context.perform_substitution(LaunchConfiguration('launch_rviz'))
    launch_rviz_bool = launch_rviz_val.lower() == 'true'
    gui_val = context.perform_substitution(LaunchConfiguration('gui'))

    actions = []
    for env_action in _gazebo_env_actions([pkg_share]):
        actions.append(env_action)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_bool,
            'robot_description': Command(['xacro ', urdf_model_path]),
        }],
    )
    actions.append(robot_state_publisher_node)

    if launch_rviz_bool and use_sim_bool:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz_config_path],
            parameters=[{'use_sim_time': use_sim_bool}],
        ))

    if not use_sim_bool:
        actions.append(ExecuteProcess(
            cmd=['echo', 'Hardware drivers (use_sim=false) — 请接入实体驱动'],
            output='screen',
        ))
        return actions

    world_name = context.perform_substitution(LaunchConfiguration('world'))
    world_path = os.path.join(pkg_share, 'worlds', f'{world_name}.world')
    spawn_poses = {
        'small_house': ('1.0', '-1.0', '0.0', '0.0'),
        'empty': ('0.0', '0.0', '0.0', '0.0'),
    }
    sx, sy, sz, syaw = spawn_poses.get(world_name, ('0.0', '0.0', '0.0', '0.0'))

    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            # verbose=true 便于排查 gzserver 崩溃（如端口占用 exit 255）
            'verbose': 'true',
            'gui': gui_val,
        }.items(),
    ))

    actions.append(TimerAction(
        period=3.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_name_in_model,
                '-topic', 'robot_description',
                '-x', sx, '-y', sy, '-z', sz, '-Y', syaw,
            ],
            output='screen',
        )],
    ))

    # Gazebo 相机话题兜底（旧 sensor 名 → detection 期望路径）
    _camera_relays = [
        ('/camera/left/camera_left_sensor/image_raw', '/camera/left/image_raw'),
        ('/camera/left/camera_left_sensor/camera_info', '/camera/left/camera_info'),
        ('/camera/right/camera_right_sensor/image_raw', '/camera/right/image_raw'),
        ('/camera/right/camera_right_sensor/camera_info', '/camera/right/camera_info'),
    ]
    actions.append(TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('perception_pkg'),
                        'launch', 'cmd_vel_guard.launch.py',
                    )
                ),
            ),
            *[
            Node(
                package='topic_tools',
                executable='relay',
                name=f'relay_{src.strip("/").replace("/", "_")}',
                output='screen',
                arguments=[src, dst],
                parameters=[{'use_sim_time': use_sim_bool}],
            )
            for src, dst in _camera_relays
            ],
        ],
    ))

    return actions


def generate_launch_description():
    declare_use_sim_cmd = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='true=Gazebo Classic 仿真; false=实体硬件占位',
    )
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='small_house',
        description='Classic 世界名（不含 .world）：small_house / empty',
    )
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Gazebo GUI；无显示器时设为 false',
    )
    declare_launch_rviz_cmd = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='是否在本 launch 中启动 RViz（双终端方案请 false）',
    )

    ld = LaunchDescription()
    for env_action in _gpu_env_actions():
        ld.add_action(env_action)
    ld.add_action(declare_use_sim_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_launch_rviz_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
