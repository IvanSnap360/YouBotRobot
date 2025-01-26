import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='omnibot_platform_teleop',
        #     executable='omnibot_platform_teleop',
        #     name='omnibot_platform_teleop',
        #     output='screen',
        #     emulate_tty=False,
        #     parameters=[
        #         {"config_file_path": os.path.join(get_package_share_directory('omnibot_platform_teleop'), 'config', 'teleop_config.yaml'),
        #          "keys_mode": "numpad_keys"
        #         }]
        #     ),
        ExecuteProcess(
        cmd=[[
            
            FindExecutable(name='ros2'),
            ' run ',
            'omnibot_platform_teleop ',
            'omnibot_platform_teleop ',
            '--ros-args ',
            '-p ',
            'config_file_path:="',
            os.path.join(get_package_share_directory('omnibot_platform_teleop'), 'config', 'teleop_config.yaml'),
            '" ',
            '-p ',
            'keys_mode:=',
            '"numpad_keys" '
        ]],
        shell=False,
        output='screen'
    )
    ])
