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
            package="omnibot_platform_mecanum_drv",
            executable="omnibot_platform_mecanum_drv",
            name="omnibot_platform_mecanum_drv_node",
            output="screen",
            parameters=[
                {"config_file_path": "/home/ivan/Projects/OmniBot/src/omnibot_platform/omnibot_platform_mecanum_drv/config/omnibot_platform_mecanum_drv_lib_cfg.yaml"}
            ]
        )
    ])
