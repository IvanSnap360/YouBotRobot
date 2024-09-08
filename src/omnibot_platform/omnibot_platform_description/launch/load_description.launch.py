import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,NotSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
import launch
import launch_ros.actions
import xacro




def generate_launch_description():
    package_path = get_package_share_directory('omnibot_platform_description')
    path = os.path.join(package_path, 'urdf/', "omnibot_platform.urdf.xacro")
    assert os.path.exists(path), f"Path: {path} not exists!!!"

    robot_description = xacro.process_file(path, mappings={'robot_namespace' : '/'}).toxml()
    
    stand_alone_launch = LaunchConfiguration('stand_alone')
    gui_launch = LaunchConfiguration("gui_launch")
    
    DeclareLaunchArgument("stand_alone", default_value="True", choices=["True", "False"])
    DeclareLaunchArgument("gui_launch", default_value="False", choices=["True", "False"])

    

    robot_state_pub = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description}
                ],
                output="screen"
                )

    joint_state_pub = Node(
                    package="joint_state_publisher",
                    executable="joint_state_publisher",
                    name="joint_state_publisher",
                    output="screen",
                    condition = IfCondition(NotSubstitution(gui_launch))
                )
    
    joint_state_gui_pub = Node(
                    package="joint_state_publisher_gui",
                    executable="joint_state_publisher_gui",
                    name="joint_state_publisher_gui",
                    output="screen",
                    condition = IfCondition(gui_launch)
                )

    rviz_node = Node(
                package='rviz2',
                namespace='',
                executable='rviz2',
                name='rviz2',
                output="screen",
                arguments=['-d' + os.path.join(package_path, 'config', 'rviz_cfg.rviz')],
                condition = IfCondition(stand_alone_launch)
                )
    
    
    loader = LaunchDescription()
    loader.add_action(robot_state_pub)
    loader.add_action(joint_state_pub)
    loader.add_action(joint_state_gui_pub)
    loader.add_action(rviz_node)
    return loader


