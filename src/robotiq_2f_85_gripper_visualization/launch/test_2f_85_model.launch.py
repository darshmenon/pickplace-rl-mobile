import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory("robotiq_2f_85_gripper_visualization"),
        "urdf",
        "robotiq_arg2f_85_model.xacro",
    )

    robot_description = Command(["xacro", " ", urdf_file]) 

    rviz_config_file = os.path.join(
        get_package_share_directory("robotiq_2f_85_gripper_visualization"),
        "visualize.rviz",
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="True", description="Use GUI for visualization"),
        DeclareLaunchArgument("rviz_config_file", default_value=rviz_config_file, description="Path to RViz config file"),

        # Publish robot description
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),

        # Joint state publisher
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{"use_gui": LaunchConfiguration("gui")}],
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", LaunchConfiguration("rviz_config_file")],
            output="screen",
        ),
    ])
