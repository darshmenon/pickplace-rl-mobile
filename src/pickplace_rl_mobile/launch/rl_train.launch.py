from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rl_node = Node(
        package='pickplace_rl_mobile',
        executable='train_rl',  # Matches setup.py entry point
        name='rl_env_node',
        output='screen'
    )
    return LaunchDescription([rl_node])