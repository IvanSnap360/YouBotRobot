import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    enable_rviz = LaunchConfiguration("rviz_gui")
    enable_gazebo = LaunchConfiguration("gazebo_gui")

    DeclareLaunchArgument("sim", default_value="False", choices=["True", "False"])
    DeclareLaunchArgument("rviz_gui", default_value="False", choices=["True", "False"])
    DeclareLaunchArgument("gazebo_gui", default_value="False", choices=["True", "False"])

    package_path = get_package_share_directory("omnibot_bringup")
    omnibot_gazebo_package_path = get_package_share_directory("omnibot_gazebo")
    omnibot_description_package_path = get_package_share_directory(
        "omnibot_description"
    )
    omnibot_platform_control_package_path = get_package_share_directory(
        "omnibot_platform_control"
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
            "standalone": "True",
            "gui": "False",
        }.items(),
    )
    
    
    omnibot_platrform_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                omnibot_platform_control_package_path, "launch", "omnibot_platform_control.launch.py"
            )
        ),
        # launch_arguments={"sim": sim}.items(),
    )
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                omnibot_gazebo_package_path, "launch", "omnibot_gazebo.launch.py"
            )
        ),
        launch_arguments={"gui_rviz": enable_rviz, "gui_gazebo": enable_gazebo}.items(),
        condition=IfCondition(sim),
    )

    loader = LaunchDescription()
    loader.add_action(omnibot_description_launch)
    loader.add_action(gazebo_launch)
    loader.add_action(omnibot_platrform_control_launch)
    return loader
