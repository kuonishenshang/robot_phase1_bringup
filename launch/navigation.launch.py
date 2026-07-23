import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share_dir = get_package_share_directory('robot_phase1_bringup')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share_dir, 'launch', 'robot_base.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('start_hardware')),
        launch_arguments={
            'base_params_file': LaunchConfiguration('base_params_file'),
            'lidar_params_file': LaunchConfiguration('lidar_params_file'),
            'start_base': LaunchConfiguration('start_base'),
            'start_lidar': LaunchConfiguration('start_lidar'),
            'start_laser_tf': LaunchConfiguration('start_laser_tf'),
            'base_serial_port': LaunchConfiguration('base_serial_port'),
            'base_baudrate': LaunchConfiguration('base_baudrate'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'lidar_baudrate': LaunchConfiguration('lidar_baudrate'),
        }.items(),
    )

    nav2_common_params = [
        params_file,
        {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
    ]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=nav2_common_params + [{'yaml_filename': LaunchConfiguration('map')}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=nav2_common_params,
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=nav2_common_params,
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=nav2_common_params,
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=nav2_common_params,
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=nav2_common_params,
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=nav2_common_params,
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=nav2_common_params,
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            {'autostart': ParameterValue(autostart, value_type=bool)},
            {'node_names': ['map_server', 'amcl']},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            {'autostart': ParameterValue(autostart, value_type=bool)},
            {
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                ]
            },
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2_navigation',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_laser_tf', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('base_serial_port', default_value='/dev/robot_base'),
        DeclareLaunchArgument('base_baudrate', default_value='230400'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/robot_lidar'),
        DeclareLaunchArgument('lidar_baudrate', default_value='230400'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(share_dir, 'maps', 'map.yaml'),
            description='Full path to the saved occupancy-grid map YAML.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share_dir, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument(
            'base_params_file',
            default_value=os.path.join(share_dir, 'config', 'base.yaml'),
        ),
        DeclareLaunchArgument(
            'lidar_params_file',
            default_value=os.path.join(share_dir, 'config', 'lidar.yaml'),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share_dir, 'rviz', 'navigation.rviz'),
        ),
        hardware_launch,
        map_server,
        amcl,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager_localization,
        lifecycle_manager_navigation,
        rviz_node,
    ])
