import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,NotSubstitution,Command,PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    sim = LaunchConfiguration("sim")
    DeclareLaunchArgument("sim", default_value="False", choices=["True", "False"])

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('omnibot_platform_control'),
            'config',
            'controller_manager.yaml',
        ]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="screen",
    )
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        control_node,
        load_joint_state_broadcaster,
        load_joint_velocity_controller
    ])
