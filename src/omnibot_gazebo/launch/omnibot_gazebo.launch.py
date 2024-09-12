import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import TimerAction



def generate_launch_description():
    
    omnibot_gazebo_package_path = get_package_share_directory("omnibot_gazebo")

    
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={"verbose": "true"}.items(), # "extra_gazebo_args": "-s libgazebo_ros_p3d.so"
    )
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        ),
        launch_arguments={"verbose": "true"}.items(),
    )
    spawn_entity_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', "omnibot_robot", 
                '-topic', '/robot_description',
                    '-x', "0.00",
                    '-y', "0.00",
                    '-z', "0.05",
                    '-Y', "0.00"],
                    output='screen')
    
    
    return LaunchDescription([
        gzclient,
        gzserver,
        spawn_entity_cmd
    ])
