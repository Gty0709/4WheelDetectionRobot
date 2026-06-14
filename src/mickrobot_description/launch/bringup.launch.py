import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'mickrobot_description'
    robot_name_in_model = 'mickrobot'
    urdf_gazebo_name = 'mickrobot_ugv.urdf.xacro'
    urdf_rviz_name = 'mickrobot_stl_rviz.urdf.xacro'

    # 通过标准 ROS 2 方式查找包路径；未 colcon build 时退回文件所在目录
    try:
        pkg_share = FindPackageShare(package=package_name).find(package_name)
    except Exception:
        pkg_share = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

    # 动态注入 Gazebo 资源路径，确保 Gazebo 能找到 mesh 模型
    workspace_path = os.path.abspath(os.path.join(pkg_share, '..'))
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if workspace_path not in gz_resource_path:
        os.environ['GZ_SIM_RESOURCE_PATH'] = workspace_path + ':' + gz_resource_path
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = workspace_path + ':' + gz_resource_path

    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_gazebo_name}')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_gazebo.rviz')

    # 顶层参数
    use_sim = LaunchConfiguration('use_sim')
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    declare_use_sim_cmd = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='是否使用仿真环境(Gazebo)或连接现实本体驱动(Real)'
    )
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='small_house',
        description='Gazebo世界文件名（不含.world后缀）：small_house / nav'
    )
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='是否启动 Gazebo 图形界面；SSH/headless 模式建议设为 false'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='是否启动 RViz2'
    )
    declare_render_engine_cmd = DeclareLaunchArgument(
        'render_engine',
        default_value='ogre',
        description='Gazebo 渲染引擎：Raspberry Pi 建议 ogre；有 OpenGL 3.3 的机器可用 ogre2'
    )

    # 节点: robot_state_publisher（发布 TF 并将 xacro 处理后的 URDF 发到 /robot_description）
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_model_path]),
                value_type=str,
            )
        }],
    )

    # RViz2
    start_rviz_cmd = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': use_sim}]
    )

    # 真实硬件占位（use_sim=false 时）
    real_robot_hardware_node = ExecuteProcess(
        condition=UnlessCondition(use_sim),
        cmd=['echo', 'Hardware drivers starting... (placeholder for use_sim=false)'],
        output='screen'
    )

    # -------------------------------------------------------
    # Gazebo 相关节点用 OpaqueFunction 延迟到运行时查找路径
    # 避免 ros_gz_sim 未安装时在文件加载阶段就抛出异常
    # -------------------------------------------------------
    def launch_gazebo_nodes(context, *args, **kwargs):
        use_sim_val = context.perform_substitution(LaunchConfiguration('use_sim'))
        if use_sim_val.lower() != 'true':
            return []

        world_name = context.perform_substitution(LaunchConfiguration('world'))
        gui_val = context.perform_substitution(LaunchConfiguration('gui')).lower()
        render_engine = context.perform_substitution(LaunchConfiguration('render_engine'))
        # [Humble迁移] Jazzy/gz-sim 世界文件带 _gz 后缀；Classic 见 bringup_classic.launch.py
        world_file = f'{world_name}_gz' if world_name == 'small_house' else world_name
        world_path = os.path.join(pkg_share, 'worlds', f'{world_file}.world')

        # 各地图的机器人初始位置（房间中央）
        spawn_poses = {
            'small_house': ('0.0',  '0.0',  '0.01', '0.0'),
            'nav':         ('6.32', '-12.56', '0.01', '0.0'),
        }
        sx, sy, sz, syaw = spawn_poses.get(world_name, ('0.0', '0.0', '0.1', '0.0'))

        try:
            from launch.actions import IncludeLaunchDescription
            from launch.launch_description_sources import PythonLaunchDescriptionSource
            gz_sim_share = FindPackageShare('ros_gz_sim').find('ros_gz_sim')
        except Exception:
            print(
                '\n[ERROR] ros_gz_sim 未安装，无法启动 Gazebo 仿真。\n'
                '请运行: sudo apt install ros-jazzy-ros-gz\n'
            )
            return []

        gz_args = f'-r --render-engine {render_engine} {world_path}'
        if gui_val != 'true':
            gz_args = f'-s --headless-rendering {gz_args}'

        start_gazebo_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')
            ]),
            launch_arguments={'gz_args': gz_args}.items(),
        )

        # spawn 从 /robot_description 话题读取（由 robot_state_publisher 发布的 xacro 结果）
        spawn_entity_cmd = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot_name_in_model,
                '-topic', '/robot_description',
                '-x', sx, '-y', sy, '-z', sz, '-Y', syaw
            ],
            output='screen'
        )

        bridge_params = [
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ]

        ros_gz_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=bridge_params,
            output='screen',
        )

        # /odom → odom→base_footprint TF（避免使用 /tf 桥接带入 Gazebo 全局位姿）
        odom_to_tf_node = Node(
            package='perception_pkg',
            executable='odom_to_tf.py',
            name='odom_to_tf',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )

        return [start_gazebo_cmd, spawn_entity_cmd, ros_gz_bridge_node, odom_to_tf_node]

    # 构建并返回 Launch 描述符
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_render_engine_cmd)
    ld.add_action(robot_state_publisher_node)
    # joint_state_publisher 不需要：仿真由Gazebo插件发布，实体由硬件驱动发布
    ld.add_action(OpaqueFunction(function=launch_gazebo_nodes))
    ld.add_action(real_robot_hardware_node)
    ld.add_action(start_rviz_cmd)

    return ld
