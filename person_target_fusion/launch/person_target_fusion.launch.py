from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('person_target_fusion'), 'config', 'person_target_fusion.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('vision_topic', default_value='/yolo/detections'),
        DeclareLaunchArgument('radar_topic', default_value='/radar/tracks'),
        Node(
            package='person_target_fusion',
            executable='person_target_fusion_node',
            name='person_target_fusion_node',
            output='screen',
            parameters=[params, {
                'vision_topic': LaunchConfiguration('vision_topic'),
                'radar_topic': LaunchConfiguration('radar_topic'),
            }],
        ),
    ])
