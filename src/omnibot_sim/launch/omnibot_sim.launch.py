import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution,IfElseSubstitution,NotSubstitution
from launch_ros.actions import Node
from ros_gz_sim.actions import GzSpawnModel
from launch.conditions import IfCondition


def generate_launch_description():
    omnibot_sim_pkg = get_package_share_directory('omnibot_sim')
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    gz_sim_launch_path = os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
    gz_sim_gui = LaunchConfiguration('gazebo_gui', default='true')
    DeclareLaunchArgument("gazebo_gui", default_value="True", choices=["True", "False"])

    
    
    world_path = os.path.join(omnibot_sim_pkg, 'worlds', 'my_empty_world.sdf')
    
    
    
    gz_sim_launch_NO_GUI = IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={
            "gz_args": [f"--headless-rendering -s -r {world_path}"],
            'on_exit_shutdown': 'True'
            }.items(),
        condition=IfCondition(NotSubstitution(gz_sim_gui))
    )
    
    gz_sim_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={
            "gz_args": [f"-r {world_path}"],
            'on_exit_shutdown': 'True'
            }.items(),
        condition=IfCondition(gz_sim_gui)
    )
    
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', "omnibot_robot",
            '-allow_renaming', 'false',
            '-x', "0.0",
            '-y', "0.0",
            '-z', "0.0",
            '-R', "0.0",
            '-P', "0.0",
            '-Y', "0.0"
        ])
    
    return LaunchDescription([
        gz_sim_launch,
        gz_sim_launch_NO_GUI,
        start_gazebo_ros_spawner_cmd
    ])
