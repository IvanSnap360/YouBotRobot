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

def generate_launch_description():
    package_path = get_package_share_directory('omnibot_description')
    urdf_path = os.path.join(package_path, 'urdf/', "omnibot_description.urdf.xacro")
    assert os.path.exists(urdf_path), f"Path: {urdf_path} not exists!!!"
    
    stand_alone_launch = LaunchConfiguration('standalone')
    gui_launch = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")

    
    DeclareLaunchArgument("rviz", default_value="True", choices=["True", "False"])
    DeclareLaunchArgument("standalone", default_value="True", choices=["True", "False"])
    DeclareLaunchArgument("gui", default_value="False", choices=["True", "False"])
    
    robot_description = xacro.process_file(urdf_path, mappings={'robot_namespace' : 'omnibot_robot',
        'use_manipulator' : 'False'}).toxml()
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('omnibot_description'),
            'config',
            'controller_manager.yaml',
        ]
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

    robot_state_pub = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{'robot_description': robot_description}],
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
                condition = IfCondition(rviz)
                )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="screen",
    )
    
    loader =  LaunchDescription()
    loader.add_action(control_node)
    loader.add_action(robot_state_pub)
    loader.add_action(joint_state_pub)
    loader.add_action(joint_state_gui_pub)
    loader.add_action(rviz_node)
    loader.add_action(load_joint_state_broadcaster)
    loader.add_action(load_joint_velocity_controller)
  
    return loader

