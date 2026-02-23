#!/usr/bin/env python3
"""
Full VLA Pipeline Launch – Phase 1-4 combined.
Starts: robot_state_publisher, vla_vision_node, vla_language_node,
        vla_action_node, vla_coordinator_node
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    return LaunchDescription([
        # Phase 1 – Action execution node
        Node(
            package='pickplace_rl_mobile',
            executable='vla_action_node',
            name='vla_action_node',
            output='screen',
        ),

        # Phase 2 – Vision perception node (delayed so camera is ready)
        TimerAction(period=2.0, actions=[
            Node(
                package='pickplace_rl_mobile',
                executable='vla_vision_node',
                name='vla_vision_node',
                output='screen',
            ),
        ]),

        # Phase 3 – Language parser node
        TimerAction(period=2.5, actions=[
            Node(
                package='pickplace_rl_mobile',
                executable='vla_language_node',
                name='vla_language_node',
                output='screen',
            ),
        ]),

        # Phase 4 – Coordinator (start last so all services are up)
        TimerAction(period=4.0, actions=[
            Node(
                package='pickplace_rl_mobile',
                executable='vla_coordinator_node',
                name='vla_coordinator_node',
                output='screen',
            ),
        ]),
    ])
