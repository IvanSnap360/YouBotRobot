import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='omnibot_platform_control',
            executable='omnibot_platform_translate_to_controllers',
            name='omnibot_platform_translate_to_controllers_node',
            output='screen',
            parameters=[
                {"config_file_path": os.path.join(get_package_share_directory('omnibot_platform_control'), 'config', 'translation_config.yaml')}
            ]
            ),
    ])
