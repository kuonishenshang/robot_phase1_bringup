import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share_dir = get_package_share_directory('robot_phase1_bringup')
    camera_share_dir = get_package_share_directory('astra_camera')
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

    camera_enabled = PythonExpression([
        "'", LaunchConfiguration('start_hardware'), "' == 'true' and '",
        LaunchConfiguration('start_camera'), "' == 'true'",
    ])
    camera_tf_enabled = PythonExpression([
        "'", LaunchConfiguration('start_hardware'), "' == 'true' and '",
        LaunchConfiguration('start_camera'), "' == 'true' and '",
        LaunchConfiguration('start_camera_tf'), "' == 'true'",
    ])

    camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(camera_share_dir, 'launch', 'astra_pro.launch.xml')
        ),
        condition=IfCondition(camera_enabled),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_ir': 'false',
            'enable_point_cloud': 'true',
            'enable_colored_point_cloud': 'false',
            'depth_registration': 'false',
            'color_width': LaunchConfiguration('camera_color_width'),
            'color_height': LaunchConfiguration('camera_color_height'),
            'color_fps': LaunchConfiguration('camera_color_fps'),
            'depth_width': LaunchConfiguration('camera_depth_width'),
            'depth_height': LaunchConfiguration('camera_depth_height'),
            'depth_fps': LaunchConfiguration('camera_depth_fps'),
            'color_qos': 'sensor_data',
            'depth_qos': 'sensor_data',
            'point_cloud_qos': 'sensor_data',
            'publish_tf': 'true',
            'tf_publish_rate': '0.0',
        }.items(),
    )

    camera_tf_node = Node(
        condition=IfCondition(camera_tf_enabled),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_link',
        arguments=[
            '--x', LaunchConfiguration('camera_x'),
            '--y', LaunchConfiguration('camera_y'),
            '--z', LaunchConfiguration('camera_z'),
            '--qx', LaunchConfiguration('camera_qx'),
            '--qy', LaunchConfiguration('camera_qy'),
            '--qz', LaunchConfiguration('camera_qz'),
            '--qw', LaunchConfiguration('camera_qw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
    )

    voxel_debug_node = Node(
        condition=IfCondition(LaunchConfiguration('publish_voxel_debug')),
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d_cloud',
        namespace='local_costmap',
        name='costmap_2d_cloud',
        output='screen',
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
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_camera_tf', default_value='true'),
        DeclareLaunchArgument('publish_voxel_debug', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('base_serial_port', default_value='/dev/robot_base'),
        DeclareLaunchArgument('base_baudrate', default_value='230400'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/robot_lidar'),
        DeclareLaunchArgument('lidar_baudrate', default_value='230400'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_color_width', default_value='640'),
        DeclareLaunchArgument('camera_color_height', default_value='480'),
        DeclareLaunchArgument('camera_color_fps', default_value='15'),
        DeclareLaunchArgument('camera_depth_width', default_value='640'),
        DeclareLaunchArgument('camera_depth_height', default_value='480'),
        DeclareLaunchArgument('camera_depth_fps', default_value='30'),
        DeclareLaunchArgument('camera_x', default_value='0.08'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.0'),
        DeclareLaunchArgument('camera_qx', default_value='0.0'),
        DeclareLaunchArgument('camera_qy', default_value='0.0'),
        DeclareLaunchArgument('camera_qz', default_value='0.0'),
        DeclareLaunchArgument('camera_qw', default_value='1.0'),
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
        camera_launch,
        camera_tf_node,
        voxel_debug_node,
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
