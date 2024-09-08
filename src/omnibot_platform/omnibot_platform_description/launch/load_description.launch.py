import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch
import launch_ros.actions
import xacro


def generate_launch_description():
    package_path = get_package_share_directory('omnibot_platform_description')
    path = os.path.join(package_path, 'urdf/', "omnibot_platform.urdf.xacro")
    assert os.path.exists(path), f"Path: {path} not exists!!!"
    robot_description = xacro.process_file(path, mappings={'robot_namespace' : '/'}).toxml()
    
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description}
            ],
            output="screen"
            ),
        Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen"
            ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            output="screen",
            arguments=['-d' + os.path.join(package_path, 'config', 'rviz_cfg.rviz')]
            ),
        

    ])
