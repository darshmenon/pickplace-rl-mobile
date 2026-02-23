import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    from launch.substitutions import Command
    from launch_ros.parameter_descriptions import ParameterValue
    
    pkg_path = get_package_share_directory('pickplace_rl_mobile')
    urdf_file = os.path.join(pkg_path, 'urdf', 'mobile_ur3.urdf')
    rviz_config = os.path.join(pkg_path, 'rviz', 'urdf.rviz')

    robot_desc = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # arguments=['-d', rviz_config] # Skipping config for now to let it load default
        )
    ])
