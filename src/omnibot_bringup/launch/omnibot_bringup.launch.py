import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,NotSubstitution,Command,PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    
    #omnibot description
    omnibot_description_package_path = get_package_share_directory("omnibot_description")
    omnibot_description_file = os.path.join(omnibot_description_package_path,"launch","omnibot_description.launch.py")
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
            "control_gui": control_gui,
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
    
    return LaunchDescription([
        omnibot_description_launch,
        rviz_node,
        gazebo_launch
    ])
