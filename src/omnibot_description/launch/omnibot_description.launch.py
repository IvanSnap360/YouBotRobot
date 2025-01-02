import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,NotSubstitution,Command,PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node
import xacro



def generate_launch_description():
    package_path = get_package_share_directory('omnibot_description')
    urdf_path = os.path.join(package_path, 'urdf/', "omnibot_description.urdf.xacro")
    assert os.path.exists(urdf_path), f"Path: {urdf_path} not exists!!!"
    
    robot_ns = LaunchConfiguration('robot_ns', default='omnibot_robot')
    add_manipulator = LaunchConfiguration('add_manipulator', default='false')
      
    DeclareLaunchArgument("robot_ns", default_value="omnibot_robot")
    DeclareLaunchArgument("add_manipulator", default_value="False")

    description = xacro.process_file(urdf_path, 
        mappings={
            'robot_namespace' : robot_ns,
            'use_manipulator' : add_manipulator
            }).toxml()
    
    
   
    robot_state_publisher_node = Node(  
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': description
            }],
    )
    
    
    return LaunchDescription([
        robot_state_publisher_node,

    ])
