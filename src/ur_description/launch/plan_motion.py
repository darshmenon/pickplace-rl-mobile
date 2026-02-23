#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
import moveit_commander
from geometry_msgs.msg import Pose

class MoveIt2Planner(Node):
    def __init__(self):
        super().__init__("moveit2_planner")

        # Initialize MoveIt 2 commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Create node
        self.node = rclpy.create_node("moveit2_planner")
        
        # Create MoveGroupCommander for your planning group
        self.move_group = moveit_commander.MoveGroupCommander("arm")  # Change "arm" to your group name

        # Set planning parameters
        self.move_group.set_planning_time(5.0)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_goal_tolerance(0.01)

    def plan_and_execute(self):
        # Get the current state
        start_state = self.move_group.get_current_pose().pose
        self.get_logger().info(f"Start State: {start_state}")

        # Define the goal state (modify coordinates as needed)
        goal_pose = Pose()
        goal_pose.position.x = 0.4
        goal_pose.position.y = 0.0
        goal_pose.position.z = 0.6
        goal_pose.orientation.w = 1.0

        self.move_group.set_pose_target(goal_pose)

        # Plan the motion
        plan = self.move_group.plan()
        
        # Execute if planning is successful
        if plan[0]:  # plan[0] is True if planning succeeds
            self.get_logger().info("Executing Plan...")
            self.move_group.execute(plan[1], wait=True)  # plan[1] is the trajectory
        else:
            self.get_logger().error("Planning failed!")

def main():
    rclpy.init()
    planner = MoveIt2Planner()
    planner.plan_and_execute()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
