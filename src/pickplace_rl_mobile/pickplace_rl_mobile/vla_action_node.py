#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
import time
from std_srvs.srv import Trigger

class VLAActionNode(Node):
    """
    Simulated VLA wrapper for testing MoveIt actions without MoveGroup Python API complexities 
    in Humble which often encounters build/version differences. 
    We will publish Cartesian targets and trigger execution sequences.
    """
    def __init__(self):
        super().__init__('vla_action_node')
        
        # In a real pipeline, we would connect directly to MoveGroupCommander.
        # But ROS2 MoveIt Python API support is tricky in some distributions without
        # compiling `moveit_ros_planning_interface`.
        # For this milestone, we set up the ROS 2 skeleton and a service interface.
        self.get_logger().info("VLA Action Node Initialized. Awaiting VLA commands.")
        
        self.srv = self.create_service(Trigger, 'execute_vla_sequence', self.execute_callback)
        
    def execute_callback(self, request, response):
        self.get_logger().info("Received VLA action sequence request: [Pick Blue Cube]")
        # Simulated execution delay
        time.sleep(1.0)
        self.get_logger().info("Executing pre-grasp approach...")
        time.sleep(1.0)
        self.get_logger().info("Executing Cartesian grasp...")
        time.sleep(1.0)
        self.get_logger().info("Executing placement...")
        
        response.success = True
        response.message = "VLA Task completed."
        return response

def main(args=None):
    rclpy.init(args=args)
    vla_action_node = VLAActionNode()
    rclpy.spin(vla_action_node)
    vla_action_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
