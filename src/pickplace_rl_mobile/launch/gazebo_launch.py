#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('pickplace_rl_mobile')
    
    # Get paths
    urdf_file = os.path.join(pkg_dir, 'urdf', 'pickplace_mobile_arm.urdf')
    world_file = os.path.join(pkg_dir, 'worlds', 'pickplace_world.world')
    
    # Read URDF content
    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()
    
    robot_description = {'robot_description': robot_description_content}
    
    # Gazebo Sim launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'pickplace_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint command bridges
            '/shoulder_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/shoulder_pitch_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/elbow_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/wrist_roll_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/wrist_pitch_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/left_finger_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/right_finger_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double'
        ],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
