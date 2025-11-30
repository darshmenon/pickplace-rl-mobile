from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Use gz sim (latest Gazebo Sim) instead of old gazebo
    gz_command = ExecuteProcess(
        cmd=['gz', 'sim', '--verbose', 'src/pickplace_rl_mobile/gazebo/world/pickplace_world.world'],
        output='screen'
    )

    return LaunchDescription([gz_command])
