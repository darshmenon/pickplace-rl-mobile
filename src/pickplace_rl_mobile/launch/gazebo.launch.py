#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('pickplace_rl_mobile')
    
    # World file path
    world_path = os.path.join(pkg_dir, 'gazebo', 'world', 'pickplace_world.world')
    
    # Alternative: Use SDF world file instead of .world if you have one
    # world_path = os.path.join(pkg_dir, 'gazebo', 'world', 'pickplace_world.sdf')
    
    # Launch Gazebo Garden/Harmonic with the world
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            '--verbose',
            '--render-engine', 'ogre2',  # Use ogre2 renderer (recommended)
            world_path
        ],
        output='screen'
    )
    
    # Alternative method using IncludeLaunchDescription if you prefer
    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('ros_gz_sim'),
    #             'launch',
    #             'gz_sim.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'gz_args': f'--verbose --render-engine ogre2 {world_path}'
    #     }.items()
    # )

    return LaunchDescription([
        gz_sim
    ])