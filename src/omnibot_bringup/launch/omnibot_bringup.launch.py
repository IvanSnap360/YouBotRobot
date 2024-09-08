import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    DeclareLaunchArgument("sim", default_value="False", choices=["True", "False"])

    package_path = get_package_share_directory("omnibot_bringup")
    omnibot_description_package_path = get_package_share_directory(
        "omnibot_description"
    )

    omnibot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                omnibot_description_package_path,
                "launch",
                "omnibot_description.launch.py",
            )
        ),
        launch_arguments={
            "rviz": "False",
            "stand_alone": "True",
            "gui_launch": "False"
        }.items(),
    )

    loader = LaunchDescription()
    loader.add_action(omnibot_description_launch)
    return loader
