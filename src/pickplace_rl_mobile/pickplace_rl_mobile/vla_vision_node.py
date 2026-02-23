#!/usr/bin/env python3
"""
VLA Phase 2 - Vision Node
Subscribes to camera topics, performs HSV color segmentation,
and publishes detected object poses for the VLA pipeline.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
import json

try:
    import cv2
    from cv_bridge import CvBridge
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


# HSV color ranges for object detection
COLOR_RANGES = {
    'red':    ([0, 120, 70],   [10, 255, 255]),
    'blue':   ([100, 150, 50], [140, 255, 255]),
    'green':  ([40, 50, 50],   [80, 255, 255]),
    'yellow': ([20, 100, 100], [30, 255, 255]),
}


class VLAVisionNode(Node):
    def __init__(self):
        super().__init__('vla_vision_node')

        self.bridge = CvBridge() if CV2_AVAILABLE else None
        self.depth_image = None
        self.camera_info = None

        # Subscriptions
        if CV2_AVAILABLE:
            self.color_sub = self.create_subscription(Image, '/camera/image_raw', self.color_cb, 10)
            self.depth_sub = self.create_subscription(Image, '/camera/depth', self.depth_cb, 10)
        self.cmd_sub = self.create_subscription(String, '/vla_track_object', self.cmd_cb, 10)

        # Publisher - detected object pose
        self.pose_pub = self.create_publisher(PoseStamped, '/vla/detected_object_pose', 10)
        # Publisher - world state as JSON string
        self.world_pub = self.create_publisher(String, '/vla/world_state', 10)

        self.tracked_color = 'blue'
        self.detected_objects = {}

        self.timer = self.create_timer(0.5, self.publish_world_state)
        self.get_logger().info("VLA Vision Node ready. Tracking: " + self.tracked_color)

    def cmd_cb(self, msg):
        color = msg.data.strip().lower()
        if color in COLOR_RANGES:
            self.tracked_color = color
            self.get_logger().info(f"Now tracking color: {color}")

    def depth_cb(self, msg):
        if CV2_AVAILABLE:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def color_cb(self, msg):
        if not CV2_AVAILABLE:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        for color, (lower, upper) in COLOR_RANGES.items():
            lo = np.array(lower, dtype=np.uint8)
            hi = np.array(upper, dtype=np.uint8)
            mask = cv2.inRange(hsv, lo, hi)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 300:
                    M = cv2.moments(largest)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])

                        # Get depth at centroid
                        depth_m = 0.5  # fallback
                        if self.depth_image is not None:
                            h, w = self.depth_image.shape[:2]
                            if 0 <= cy < h and 0 <= cx < w:
                                raw_depth = float(self.depth_image[cy, cx])
                                if raw_depth > 0:
                                    depth_m = raw_depth

                        # Simple pin-hole back-projection (assume 60deg FOV, 640x480)
                        fx = fy = 554.0
                        x = (cx - 320) * depth_m / fx
                        y = (cy - 240) * depth_m / fy
                        z = depth_m

                        self.detected_objects[color] = (x, y, z)

                        if color == self.tracked_color:
                            pose = PoseStamped()
                            pose.header.stamp = self.get_clock().now().to_msg()
                            pose.header.frame_id = 'camera_link'
                            pose.pose.position.x = x
                            pose.pose.position.y = y
                            pose.pose.position.z = z
                            pose.pose.orientation.w = 1.0
                            self.pose_pub.publish(pose)

    def publish_world_state(self):
        state = {
            color: {'x': round(pos[0], 3), 'y': round(pos[1], 3), 'z': round(pos[2], 3)}
            for color, pos in self.detected_objects.items()
        }
        msg = String()
        msg.data = json.dumps(state)
        self.world_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VLAVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
