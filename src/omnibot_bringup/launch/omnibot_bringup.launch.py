import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription,ExecuteProcess, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,NotSubstitution,Command,PathJoinSubstitution,AndSubstitution,FindExecutable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import yaml



def generate_launch_description():
    
    #omnibot description
    omnibot_description_package_path = get_package_share_directory("omnibot_description")
    omnibot_description_file = os.path.join(omnibot_description_package_path,"launch","omnibot_description.launch.py")
    #omnibot_platform_control
    omnibot_platform_control_package_path = get_package_share_directory("omnibot_platform_control")
    omnibot_platform_control_file = os.path.join(omnibot_platform_control_package_path,"launch","omnibot_platform_contollers.launch.py")
    omnibot_platform_translate_to_controllers_file = os.path.join(omnibot_platform_control_package_path,"launch","joint_control_to_contoller_translate.launch.py")
    
    omnibot_platform_mecanum_drv_pkg = get_package_share_directory("omnibot_platform_mecanum_drv")
    omnibot_platform_mecanum_drv_file = os.path.join(omnibot_platform_mecanum_drv_pkg,"launch","omnibot_platform_mecanum_drv.launch.py")
    #omnibot_navigation
    #omnibot_manipulator
    #omnibot_state_machine
    #omnibot sim
    omnibot_sim_package_path = get_package_share_directory("omnibot_sim")
    omnibot_sim_file = os.path.join(omnibot_sim_package_path,"launch","omnibot_sim.launch.py")
    
    rviz = LaunchConfiguration('rviz', default='false')
    control_gui = LaunchConfiguration('control_gui', default='false')
    gazebo = LaunchConfiguration('gazebo', default='false')
    gazebo_gui = LaunchConfiguration('gazebo_gui', default='false')
    
    DeclareLaunchArgument("rviz", default_value="True", choices=["True", "False"])
    DeclareLaunchArgument("gazebo", default_value="True", choices=["True", "False"])
    DeclareLaunchArgument("gazebo_gui", default_value="True", choices=["True", "False"])
    DeclareLaunchArgument("control_gui", default_value="True", choices=["True", "False"])
    
    
    omnibot_description_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(omnibot_description_file),
        launch_arguments={
            "robot_ns": "omnibot_robot",
            "add_manipulator": "False",
        }.items(),
    )
    
    gazebo_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(omnibot_sim_file),
        launch_arguments={
            "gazebo_gui": gazebo_gui,
        }.items(),
        condition=IfCondition(gazebo)
    )

    

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('omnibot_description'), 'config', 'rviz.rviz')],
        condition = IfCondition(rviz)
    )
    
    joint_state_pub = Node(
                    package="joint_state_publisher",
                    executable="joint_state_publisher",
                    name="joint_state_publisher",
                    output="screen",
                    condition = IfCondition(AndSubstitution(NotSubstitution(gazebo) ,NotSubstitution(control_gui)))
                )
    
    joint_state_gui_pub = Node(
                    package="joint_state_publisher_gui",
                    executable="joint_state_publisher_gui",
                    name="joint_state_publisher_gui",
                    output="screen",
                    condition = IfCondition(AndSubstitution(NotSubstitution(gazebo) ,control_gui))
                )
    
    omnibot_controllers = IncludeLaunchDescription(PythonLaunchDescriptionSource(omnibot_platform_control_file))
    
    omnibot_mecanum_drv = IncludeLaunchDescription(PythonLaunchDescriptionSource(omnibot_platform_mecanum_drv_file))
    
    omnibot_platform_translate_to_controllers = IncludeLaunchDescription(PythonLaunchDescriptionSource(omnibot_platform_translate_to_controllers_file))
    
    omnibot_bringup = Node(
        package='omnibot_bringup',
        executable='omnibot_config',
        name='omnibot_config',
        output='screen',
        parameters=[{"config_file_path": os.path.join(get_package_share_directory('omnibot_bringup'), 'config', 'main_config.yaml')}]
    )
    
    return LaunchDescription([
        rviz_node,
        gazebo_launch,
        joint_state_pub,
        joint_state_gui_pub,
        
        
        omnibot_bringup,
        omnibot_controllers,
        omnibot_mecanum_drv,
        omnibot_description_launch,
        omnibot_platform_translate_to_controllers
    ])
