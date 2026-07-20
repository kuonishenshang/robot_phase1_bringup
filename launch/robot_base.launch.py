import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share_dir = get_package_share_directory('robot_phase1_bringup')

    base_params_file = LaunchConfiguration('base_params_file')
    lidar_params_file = LaunchConfiguration('lidar_params_file')
    start_base = LaunchConfiguration('start_base')
    start_lidar = LaunchConfiguration('start_lidar')
    start_laser_tf = LaunchConfiguration('start_laser_tf')

    base_node = Node(
        condition=IfCondition(start_base),
        package='serial_port_test',
        executable='serial_port_test_node',
        name='serial_port_test_node',
        output='screen',
        parameters=[
            base_params_file,
            {
                'serial_port_name': LaunchConfiguration('base_serial_port'),
                'serial_baud_rate': ParameterValue(
                    LaunchConfiguration('base_baudrate'), value_type=int
                ),
            },
        ],
    )

    lidar_node = Node(
        condition=IfCondition(start_lidar),
        package='cspc_lidar',
        executable='cspc_lidar',
        name='cspc_lidar',
        output='screen',
        emulate_tty=True,
        parameters=[
            lidar_params_file,
            {
                'port': LaunchConfiguration('lidar_port'),
                'baudrate': ParameterValue(
                    LaunchConfiguration('lidar_baudrate'), value_type=int
                ),
            },
        ],
    )

    laser_tf_node = Node(
        condition=IfCondition(start_laser_tf),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_link',
        arguments=[
            LaunchConfiguration('laser_x'),
            LaunchConfiguration('laser_y'),
            LaunchConfiguration('laser_z'),
            LaunchConfiguration('laser_roll'),
            LaunchConfiguration('laser_pitch'),
            LaunchConfiguration('laser_yaw'),
            'base_link',
            'laser_link',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'base_params_file',
            default_value=os.path.join(share_dir, 'config', 'base.yaml'),
            description='Parameters for the chassis serial node.',
        ),
        DeclareLaunchArgument(
            'lidar_params_file',
            default_value=os.path.join(share_dir, 'config', 'lidar.yaml'),
            description='Parameters for the CSPC lidar node.',
        ),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_laser_tf', default_value='true'),
        DeclareLaunchArgument('base_serial_port', default_value='/dev/robot_base'),
        DeclareLaunchArgument('base_baudrate', default_value='230400'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/robot_lidar'),
        DeclareLaunchArgument('lidar_baudrate', default_value='230400'),
        DeclareLaunchArgument('laser_x', default_value='-0.012'),
        DeclareLaunchArgument('laser_y', default_value='-0.007'),
        DeclareLaunchArgument('laser_z', default_value='0.1219'),
        DeclareLaunchArgument('laser_roll', default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
        base_node,
        lidar_node,
        laser_tf_node,
    ])
