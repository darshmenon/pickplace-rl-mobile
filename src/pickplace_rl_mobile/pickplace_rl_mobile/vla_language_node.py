#!/usr/bin/env python3
"""
VLA Phase 3 - Language Parser Node
Converts natural language instructions into structured JSON action commands.
"""

import re
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


# Supported action keywords
ACTIONS = ['pick', 'grab', 'grasp', 'place', 'put', 'move', 'transfer']
COLORS  = ['red', 'blue', 'green', 'yellow', 'orange', 'white', 'black']
OBJECTS = ['cube', 'box', 'ball', 'cylinder', 'block', 'object', 'item']
PLACES  = ['tray', 'bin', 'basket', 'table', 'shelf', 'box', 'container', 'drop zone']


def parse_instruction(text: str) -> dict:
    """Rule-based NLP parser that extracts intent from an instruction string."""
    text_lower = text.lower()

    action = next((a for a in ACTIONS if a in text_lower), None)
    color  = next((c for c in COLORS  if c in text_lower), None)
    obj    = next((o for o in OBJECTS  if o in text_lower), None)
    place  = next((p for p in PLACES   if p in text_lower), None)

    # Attempt regex extraction for "pick X and place in Y"
    transfer_match = re.search(
        r'(pick|grab|grasp)\s+(?:the\s+)?(\w+)\s+(?:and\s+)?(?:place|put|transfer)\s+(?:it\s+)?(?:in|into|on|onto)\s+(?:the\s+)?(\w+)',
        text_lower
    )
    if transfer_match:
        action = 'pick_and_place'
        target_desc = transfer_match.group(2)
        destination = transfer_match.group(3)
        color = color or (target_desc if target_desc in COLORS else None)
        place = destination

    return {
        'action': action or 'unknown',
        'color': color,
        'object': obj or 'cube',
        'destination': place or 'tray',
        'raw': text,
    }


class VLALanguageNode(Node):
    def __init__(self):
        super().__init__('vla_language_node')

        # Service to parse a text instruction
        self.srv = self.create_service(Trigger, '/vla/parse_instruction', self.parse_cb)

        # Topic-based interface
        self.text_sub = self.create_subscription(String, '/vla_instruction', self.instruction_cb, 10)
        self.cmd_pub  = self.create_publisher(String, '/vla/structured_command', 10)

        self.last_text = ''
        self.get_logger().info("VLA Language Node ready. Publish to /vla_instruction to start.")

    def instruction_cb(self, msg: String):
        self.last_text = msg.data
        result = parse_instruction(msg.data)
        self.get_logger().info(f"Parsed: {result}")
        out = String()
        out.data = json.dumps(result)
        self.cmd_pub.publish(out)

    def parse_cb(self, request, response):
        if self.last_text:
            result = parse_instruction(self.last_text)
            response.success = True
            response.message = json.dumps(result)
        else:
            response.success = False
            response.message = '{"error": "No instruction received yet"}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VLALanguageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
