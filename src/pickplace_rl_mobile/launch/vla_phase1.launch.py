import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('pickplace_rl_mobile')
    moveit_config_path = get_package_share_directory('moveit_config')

    return LaunchDescription([
        # 1. Launch Robot State Publisher and RViz (combined urdf)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_path, 'launch', 'demo.launch.py')
            ),
            launch_arguments={'use_rviz': 'true'}.items()
        ),

        # 2. VLA Action Node
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='pickplace_rl_mobile',
                    executable='vla_action_node',
                    name='vla_action_node',
                    output='screen'
                )
            ]
        )
    ])
