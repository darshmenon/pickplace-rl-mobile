#!/usr/bin/env python3
"""
VLA Phase 4 - Coordinator Node
Orchestrates the full VLA pipeline:
  1. Listens to /vla_instruction (raw text)
  2. Calls the language node for a structured command
  3. Looks up the target object pose from the vision node
  4. Calls the action node to execute the pick-and-place
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped


class VLACoordinatorNode(Node):
    def __init__(self):
        super().__init__('vla_coordinator_node')

        # Track latest info from sub-nodes
        self.world_state: dict = {}
        self.latest_cmd: dict  = {}
        self.latest_object_pose: PoseStamped | None = None

        # Subscriptions
        self.create_subscription(String, '/vla/world_state',        self.world_state_cb,   10)
        self.create_subscription(String, '/vla/structured_command', self.structured_cmd_cb, 10)
        self.create_subscription(
            PoseStamped, '/vla/detected_object_pose', self.pose_cb, 10
        )

        # Service clients
        self.action_client   = self.create_client(Trigger, '/execute_vla_sequence')
        self.language_client = self.create_client(Trigger, '/vla/parse_instruction')

        self.get_logger().info("VLA Coordinator ready. Orchestrating vision→language→action pipeline.")
        self.get_logger().info("Send a command: ros2 topic pub /vla_instruction std_msgs/String \"data: 'pick blue cube'\"")

    def world_state_cb(self, msg: String):
        try:
            self.world_state = json.loads(msg.data)
        except Exception:
            pass

    def pose_cb(self, msg: PoseStamped):
        self.latest_object_pose = msg

    def structured_cmd_cb(self, msg: String):
        """Called whenever the language node emits a parsed command. Trigger execution."""
        try:
            self.latest_cmd = json.loads(msg.data)
        except Exception:
            return

        action = self.latest_cmd.get('action', 'unknown')
        color  = self.latest_cmd.get('color', 'unknown')
        dest   = self.latest_cmd.get('destination', 'tray')

        self.get_logger().info(
            f"[Coordinator] Action={action} | Target={color} object | Destination={dest}"
        )

        # Check if world state has the target
        if color and color in self.world_state:
            pos = self.world_state[color]
            self.get_logger().info(
                f"[Coordinator] Object found at x={pos['x']:.3f} y={pos['y']:.3f} z={pos['z']:.3f}"
            )
        elif self.latest_object_pose:
            p = self.latest_object_pose.pose.position
            self.get_logger().info(
                f"[Coordinator] Using tracked pose: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
            )
        else:
            self.get_logger().warn("[Coordinator] Object not yet detected. Waiting for vision node...")
            return

        # Execute via action node
        if not self.action_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("[Coordinator] vla_action_node service unavailable.")
            return

        req = Trigger.Request()
        future = self.action_client.call_async(req)
        future.add_done_callback(self.action_result_cb)

    def action_result_cb(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f"[Coordinator] Action succeeded: {result.message}")
            else:
                self.get_logger().warn(f"[Coordinator] Action failed: {result.message}")
        except Exception as e:
            self.get_logger().error(f"[Coordinator] Service call exception: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = VLACoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
