import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('robot_phase1_bringup')

    # IncludeLaunch在一个总的launch文件中，嵌套调用其他的launch文件。
    # 这里我们嵌套调用 robot_base.launch.py 和 slam_toolbox 的 online_async_launch.py
    # 对相应的launch文件中的参数进行传递，使用LaunchConfiguration来获取参数值。
    hardware_launch = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource(
            os.path.join(share_dir, 'launch', 'robot_base.launch.py') #拼接路径，获取robot_base.launch.py的路径
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

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'), #获取slam_toolbox包的路径
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'use_lifecycle_manager': 'false',
        }.items(),
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_laser_tf', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('base_serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('base_baudrate', default_value='230400'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baudrate', default_value='230400'),
        DeclareLaunchArgument(
            'base_params_file',
            default_value=os.path.join(share_dir, 'config', 'base.yaml'),
        ),
        DeclareLaunchArgument(
            'lidar_params_file',
            default_value=os.path.join(share_dir, 'config', 'lidar.yaml'),
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(share_dir, 'config', 'slam_toolbox.yaml'),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share_dir, 'rviz', 'mapping.rviz'),
        ),
        hardware_launch,
        slam_launch,
        rviz_node,
    ])
