import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory('navigation_pkg')
    perception_share = get_package_share_directory('perception_pkg')

    use_sim_time = LaunchConfiguration('use_sim')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    start_localization = LaunchConfiguration('start_localization')
    rviz = LaunchConfiguration('rviz')
    log_level = LaunchConfiguration('log_level')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'perception.launch.py')
        ),
        condition=IfCondition(start_localization),
        launch_arguments={
            'use_sim': use_sim_time,
            'mode': 'localize',
            'map_yaml_file': map_yaml_file,
        }.items(),
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes},
        ],
    )

    rviz_node = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('start_localization', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=os.path.join(
                get_package_share_directory('perception_pkg'),
                'maps',
                'slam_map.yaml',
            ),
        ),
        localization_launch,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        rviz_node,
    ])
