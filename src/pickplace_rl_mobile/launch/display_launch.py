#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Get the package directory
    pkg_dir = get_package_share_directory('pickplace_rl_mobile')
    
    # Get file paths
    urdf_file = os.path.join(pkg_dir, 'urdf', 'pickplace_mobile_arm.urdf')
    world_path = os.path.join(pkg_dir, 'gazebo', 'world', 'pickplace_world.world')
    
    # Debug: Print file paths
    print(f"URDF file: {urdf_file}")
    print(f"World file: {world_path}")
    print(f"URDF exists: {os.path.exists(urdf_file)}")
    print(f"World exists: {os.path.exists(world_path)}")
    
    # Robot description - using file:// URI for better compatibility
    robot_description_content = Command(['cat ', urdf_file])
    robot_description = {'robot_description': robot_description_content}

    # Launch Gazebo Garden/Harmonic - try without world first
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            '--verbose',
            '--render-engine', 'ogre2',
            # world_path  # Comment this out temporarily to start with empty world
        ],
        output='screen'
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Test robot description publishing first
    test_robot_description = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'echo', '/robot_description', '--once'],
                output='screen'
            )
        ]
    )

    # Spawn robot in Gazebo (with more delay and better error handling)
    spawn_robot_node = TimerAction(
        period=5.0,  # Increased delay
        actions=[
            LogInfo(msg="Attempting to spawn robot..."),
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                arguments=[
                    '-entity', 'pickplace_mobile_arm',
                    '-topic', '/robot_description',  # Add leading slash
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.5',  # Spawn higher to avoid ground collision
                    '--ros-args', '--log-level', 'debug'  # Add debug logging
                ],
                output='screen'
            )
        ]
    )

    # Alternative spawn method using string instead of topic
    spawn_robot_direct = TimerAction(
        period=7.0,  # Try this if topic method fails
        actions=[
            LogInfo(msg="Trying direct robot spawn..."),
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot_direct',
                arguments=[
                    '-entity', 'pickplace_mobile_arm_direct',
                    '-string', robot_description_content,  # Pass URDF directly
                    '-x', '2.0',  # Different position
                    '-y', '0.0',
                    '-z', '0.5',
                ],
                output='screen'
            )
        ]
    )

    # Simple bridge for basic functionality
    gz_ros_bridge = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_ros_bridge',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                ],
                output='screen'
            )
        ]
    )

    # Debug: List Gazebo entities
    list_entities = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'service', '-s', '/world/default/scene/info', '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gz_sim,
        robot_state_publisher_node,
        test_robot_description,  # Debug step
        spawn_robot_node,
        spawn_robot_direct,  # Backup spawn method
        gz_ros_bridge,
        list_entities,  # Debug step
    ])