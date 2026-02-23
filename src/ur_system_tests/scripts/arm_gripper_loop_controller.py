#!/usr/bin/env python3
"""
Control robot arm and gripper to perform repetitive movements between positions.

Action Clients:
    /arm_controller/follow_joint_trajectory (control_msgs/FollowJointTrajectory)
    /gripper_controller/gripper_cmd (control_msgs/GripperCommand)

Author: Darsh Menon
Date: April 6, 2025
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ArmGripperLoopController(Node):
    def __init__(self):
        super().__init__('arm_gripper_loop_controller')

        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        self.get_logger().info('Waiting for action servers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('Action servers connected!')

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        self.target_pos = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
        self.home_pos = [0.0] * 6

        self.loop()

    def send_arm_command(self, positions):
        goal_msg = FollowJointTrajectory.Goal()
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=2)
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [point]

        self.arm_client.send_goal_async(goal_msg)
        time.sleep(2.5)  # Wait for execution

    def send_gripper_command(self, position):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 40.0
        self.gripper_client.send_goal_async(goal_msg)
        time.sleep(1.0)  # Wait for execution

    def loop(self):
        while rclpy.ok():
            self.get_logger().info('Moving to target position')
            self.send_arm_command(self.target_pos)

            self.get_logger().info('Closing gripper')
            self.send_gripper_command(0.8)

            self.get_logger().info('Moving to home position')
            self.send_arm_command(self.home_pos)

            self.get_logger().info('Opening gripper')
            self.send_gripper_command(0.0)

            self.get_logger().info('Cycle complete. Waiting 2 seconds...')
            time.sleep(2.0)


def main(args=None):
    rclpy.init(args=args)
    controller = ArmGripperLoopController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
