"""Launch clip_detector for mapping session (C++ if TensorRT engine exists, else Python YOLO)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _default_session_dir() -> str:
    """Prefer workspace src/.../maps/map_latest (updated by humble_sim_slam)."""
    try:
        p_share = get_package_share_directory('perception_pkg')
        ws_root = os.path.abspath(os.path.join(p_share, '..', '..', '..', '..'))
        candidate = os.path.join(ws_root, 'src', 'perception_pkg', 'maps', 'map_latest')
        return candidate
    except Exception:
        return 'src/perception_pkg/maps/map_latest'


def _engine_paths() -> list[str]:
    paths = []
    try:
        pkg_share = get_package_share_directory('detection_pkg')
        paths.append(os.path.join(pkg_share, 'simmodel', 'best.engine'))
        ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
        paths.append(os.path.join(ws_root, 'src', 'detection_pkg', 'simmodel', 'best.engine'))
    except Exception:
        pass
    return paths


def _has_trt_engine() -> bool:
    return any(os.path.isfile(p) for p in _engine_paths())


def _detector_node(context, *args, **kwargs):
    pkg_share = get_package_share_directory('detection_pkg')
    params_file = os.path.join(pkg_share, 'config', 'detection_params.yaml')
    use_sim = LaunchConfiguration('use_sim_time')
    force_cpp = (
        context.perform_substitution(LaunchConfiguration('force_cpp')).lower() == 'true'
    )
    use_cpp = force_cpp or _has_trt_engine()

    common_params = [
        params_file,
        {
            'session_dir': LaunchConfiguration('session_dir'),
            'weights': LaunchConfiguration('weights'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'max_fps': LaunchConfiguration('max_fps'),
            'ground_z': LaunchConfiguration('ground_z'),
            'use_sim_time': use_sim,
        },
    ]

    if use_cpp:
        return [
            LogInfo(msg='[detection_cpp] clip_detector_cpp (TensorRT best.engine)'),
            Node(
                package='detection_pkg',
                executable='clip_detector_cpp',
                name='clip_detector',
                output='screen',
                parameters=common_params,
            ),
        ]

    return [
        LogInfo(msg=(
            '[detection_cpp] 未找到 simmodel/best.engine，回退 Python YOLO '
            '(clip_detector_wrapper.sh)。生成 engine: bash scripts/build_all_trt_engines.sh'
        )),
        Node(
            package='detection_pkg',
            executable='clip_detector_wrapper.sh',
            name='clip_detector',
            output='screen',
            parameters=common_params,
        ),
    ]


def generate_launch_description():
    default_session = _default_session_dir()

    return LaunchDescription([
        DeclareLaunchArgument(
            'session_dir',
            default_value=default_session,
            description='建图会话目录；默认 map_latest（须先启动终端1 humble_sim_slam）',
        ),
        DeclareLaunchArgument('weights', default_value=''),
        DeclareLaunchArgument('conf_threshold', default_value='0.5'),
        DeclareLaunchArgument('max_fps', default_value='8.0'),
        DeclareLaunchArgument('ground_z', default_value='0.002'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真建图时必须为 true（与 Gazebo /clock 一致）',
        ),
        DeclareLaunchArgument(
            'force_cpp', default_value='false',
            description='强制 clip_detector_cpp（无 .engine 时 OpenCV DNN 可能无法加载 best.onnx）',
        ),
        GroupAction([
            SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
            OpaqueFunction(function=_detector_node),
        ]),
    ])
