import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import TimerAction


def generate_launch_description():

    omnibot_gazebo_package_path = get_package_share_directory("omnibot_gazebo")
    omnibot_description_package_path = get_package_share_directory(
        "omnibot_description"
    )

    gui_gazebo = LaunchConfiguration("gui_gazebo")
    gui_rviz = LaunchConfiguration("gui_rviz")

    DeclareLaunchArgument("gui_rviz", default_value="False", choices=["True", "False"])
    DeclareLaunchArgument(
        "gui_gazebo", default_value="False", choices=["True", "False"]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
        launch_arguments={"verbose": "true", "gui": gui_gazebo}.items(),
    )

    rviz_node = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d"
            + os.path.join(omnibot_description_package_path, "config", "rviz_cfg.rviz")
        ],
        condition=IfCondition(gui_rviz),
    )

    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "omnibot_robot",
            "-topic",
            "/robot_description",
            "-x",
            "0.00",
            "-y",
            "0.00",
            "-z",
            "0.05",
            "-Y",
            "0.00",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            rviz_node,
            gazebo,
            spawn_entity_cmd,
        ]
    )
