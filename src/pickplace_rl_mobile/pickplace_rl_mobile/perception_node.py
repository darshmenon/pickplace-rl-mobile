#!/usr/bin/env python3
"""
Perception Node for the Pick-and-Place Mobile Manipulator.

Subscribes to RGB and depth camera data, performs HSV color segmentation
to detect the red pickup object, and projects the detection into 3D space
using depth. Publishes the detected object pose, debug image, and RViz markers.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from builtin_interfaces.msg import Duration
import struct


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # --- Parameters ---
        self.declare_parameter('target_color_lower_h', 0)
        self.declare_parameter('target_color_upper_h', 10)
        self.declare_parameter('target_color_lower_s', 100)
        self.declare_parameter('target_color_upper_s', 255)
        self.declare_parameter('target_color_lower_v', 100)
        self.declare_parameter('target_color_upper_v', 255)
        self.declare_parameter('min_contour_area', 100)
        self.declare_parameter('publish_rate', 10.0)

        # HSV range for red object detection
        self.lower_hsv1 = np.array([
            self.get_parameter('target_color_lower_h').value,
            self.get_parameter('target_color_lower_s').value,
            self.get_parameter('target_color_lower_v').value
        ])
        self.upper_hsv1 = np.array([
            self.get_parameter('target_color_upper_h').value,
            self.get_parameter('target_color_upper_s').value,
            self.get_parameter('target_color_upper_v').value
        ])
        # Red wraps around hue=180, so we need a second range
        self.lower_hsv2 = np.array([170, 100, 100])
        self.upper_hsv2 = np.array([180, 255, 255])

        self.min_contour_area = self.get_parameter('min_contour_area').value

        # Camera intrinsics (will be updated from CameraInfo if available)
        self.fx = 554.26  # Focal length x (default for 640x480 @ 60 deg FOV)
        self.fy = 554.26
        self.cx = 320.0
        self.cy = 240.0
        self.img_width = 640
        self.img_height = 480

        # Latest sensor data
        self.latest_rgb = None
        self.latest_depth = None
        self.detected_pose = None

        # --- Subscribers ---
        self.rgb_sub = self.create_subscription(
            Image, '/camera/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        # --- Publishers ---
        self.pose_pub = self.create_publisher(
            PoseStamped, '/perception/detected_object', 10)
        self.debug_pub = self.create_publisher(
            Image, '/perception/debug_image', 10)
        self.marker_pub = self.create_publisher(
            Marker, '/perception/markers', 10)

        # Processing timer
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.process_frame)

        self.get_logger().info('Perception node initialized - detecting red objects via HSV segmentation')

    def camera_info_callback(self, msg):
        """Update camera intrinsics from CameraInfo message."""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.img_width = msg.width
        self.img_height = msg.height

    def rgb_callback(self, msg):
        """Store latest RGB image."""
        self.latest_rgb = msg

    def depth_callback(self, msg):
        """Store latest depth image."""
        self.latest_depth = msg

    def image_msg_to_numpy(self, msg):
        """Convert sensor_msgs/Image to numpy array (RGB)."""
        if msg.encoding == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape((msg.height, msg.width, 3))
            return img
        elif msg.encoding == 'bgr8':
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape((msg.height, msg.width, 3))
            return img[:, :, ::-1]  # BGR -> RGB
        elif msg.encoding == '8UC3':
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape((msg.height, msg.width, 3))
            return img
        else:
            self.get_logger().warn(f'Unsupported RGB encoding: {msg.encoding}')
            return None

    def depth_msg_to_numpy(self, msg):
        """Convert depth Image message to numpy float array (meters)."""
        if msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, dtype=np.float32)
            depth = depth.reshape((msg.height, msg.width))
            return depth
        elif msg.encoding == '16UC1':
            depth = np.frombuffer(msg.data, dtype=np.uint16)
            depth = depth.reshape((msg.height, msg.width))
            return depth.astype(np.float32) / 1000.0  # mm -> meters
        else:
            self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}')
            return None

    def rgb_to_hsv(self, rgb_img):
        """Convert RGB image to HSV using pure numpy (no OpenCV dependency)."""
        img = rgb_img.astype(np.float32) / 255.0
        r, g, b = img[:, :, 0], img[:, :, 1], img[:, :, 2]

        cmax = np.maximum(np.maximum(r, g), b)
        cmin = np.minimum(np.minimum(r, g), b)
        diff = cmax - cmin

        # Hue
        h = np.zeros_like(cmax)
        mask_r = (cmax == r) & (diff > 0)
        mask_g = (cmax == g) & (diff > 0)
        mask_b = (cmax == b) & (diff > 0)

        h[mask_r] = (60 * ((g[mask_r] - b[mask_r]) / diff[mask_r]) + 360) % 360
        h[mask_g] = (60 * ((b[mask_g] - r[mask_g]) / diff[mask_g]) + 120) % 360
        h[mask_b] = (60 * ((r[mask_b] - g[mask_b]) / diff[mask_b]) + 240) % 360

        # Convert to OpenCV range [0, 180]
        h = h / 2.0

        # Saturation
        s = np.zeros_like(cmax)
        s[cmax > 0] = (diff[cmax > 0] / cmax[cmax > 0]) * 255.0

        # Value
        v = cmax * 255.0

        hsv = np.stack([h, s, v], axis=-1).astype(np.uint8)
        return hsv

    def detect_red_object(self, rgb_img):
        """
        Detect red object using HSV color segmentation.
        Returns (center_x, center_y, area) in pixel coords, or None if not found.
        """
        hsv = self.rgb_to_hsv(rgb_img)

        # Create mask for red color (two ranges since red wraps around hue)
        mask1 = ((hsv[:, :, 0] >= self.lower_hsv1[0]) &
                 (hsv[:, :, 0] <= self.upper_hsv1[0]) &
                 (hsv[:, :, 1] >= self.lower_hsv1[1]) &
                 (hsv[:, :, 1] <= self.upper_hsv1[1]) &
                 (hsv[:, :, 2] >= self.lower_hsv1[2]) &
                 (hsv[:, :, 2] <= self.upper_hsv1[2]))

        mask2 = ((hsv[:, :, 0] >= self.lower_hsv2[0]) &
                 (hsv[:, :, 0] <= self.upper_hsv2[0]) &
                 (hsv[:, :, 1] >= self.lower_hsv2[1]) &
                 (hsv[:, :, 1] <= self.upper_hsv2[1]) &
                 (hsv[:, :, 2] >= self.lower_hsv2[2]) &
                 (hsv[:, :, 2] <= self.upper_hsv2[2]))

        mask = mask1 | mask2

        # Find connected components (simple blob detection without OpenCV)
        # Use labeled region approach with numpy
        labeled_pixels = np.argwhere(mask)

        if len(labeled_pixels) < self.min_contour_area:
            return None, mask

        # Compute centroid and area of the largest blob
        cy_px = int(np.mean(labeled_pixels[:, 0]))
        cx_px = int(np.mean(labeled_pixels[:, 1]))
        area = len(labeled_pixels)

        return (cx_px, cy_px, area), mask

    def pixel_to_3d(self, cx_px, cy_px, depth_img):
        """
        Project a pixel coordinate + depth into 3D camera frame coordinates.
        Returns (x, y, z) in meters in the camera optical frame.
        """
        # Sample depth in a small window around the detection center
        half_w = 5
        y_min = max(0, cy_px - half_w)
        y_max = min(depth_img.shape[0], cy_px + half_w)
        x_min = max(0, cx_px - half_w)
        x_max = min(depth_img.shape[1], cx_px + half_w)

        depth_patch = depth_img[y_min:y_max, x_min:x_max]
        valid_depths = depth_patch[(depth_patch > 0.05) & (depth_patch < 3.0)]

        if len(valid_depths) == 0:
            return None

        z = float(np.median(valid_depths))

        # Pinhole camera model: back-project
        x = (cx_px - self.cx) * z / self.fx
        y = (cy_px - self.cy) * z / self.fy

        return (x, y, z)

    def create_debug_image(self, rgb_img, mask, detection):
        """Create a debug image with detection overlay."""
        # Create the debug image (draw a crosshair on the detection)
        debug = rgb_img.copy()

        if detection is not None:
            cx, cy, area = detection
            # Draw simple crosshair (red square)
            half = 10
            y_min = max(0, cy - half)
            y_max = min(debug.shape[0], cy + half)
            x_min = max(0, cx - half)
            x_max = min(debug.shape[1], cx + half)

            # Green crosshair
            debug[y_min:y_max, cx - 1:cx + 1, :] = [0, 255, 0]
            debug[cy - 1:cy + 1, x_min:x_max, :] = [0, 255, 0]

        # Overlay mask as red tint
        debug[mask, 0] = np.minimum(debug[mask, 0].astype(np.uint16) + 80, 255).astype(np.uint8)

        # Convert back to Image message
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        msg.height = debug.shape[0]
        msg.width = debug.shape[1]
        msg.encoding = 'rgb8'
        msg.step = debug.shape[1] * 3
        msg.data = debug.tobytes()
        return msg

    def publish_marker(self, x, y, z, frame_id='camera_optical_frame'):
        """Publish RViz marker for detected object."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = frame_id
        marker.ns = 'detected_object'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.06
        marker.scale.y = 0.06
        marker.scale.z = 0.06
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        marker.lifetime = Duration(sec=0, nanosec=500000000)  # 0.5s
        self.marker_pub.publish(marker)

        # Target zone marker
        target_marker = Marker()
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.header.frame_id = 'base_link'
        target_marker.ns = 'target_zone'
        target_marker.id = 1
        target_marker.type = Marker.CYLINDER
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = 0.6
        target_marker.pose.position.y = 0.5
        target_marker.pose.position.z = 0.05
        target_marker.pose.orientation.w = 1.0
        target_marker.scale.x = 0.3
        target_marker.scale.y = 0.3
        target_marker.scale.z = 0.01
        target_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
        target_marker.lifetime = Duration(sec=1, nanosec=0)
        self.marker_pub.publish(target_marker)

    def process_frame(self):
        """Main processing loop - detect object and publish pose."""
        if self.latest_rgb is None:
            return

        # Convert RGB
        rgb_img = self.image_msg_to_numpy(self.latest_rgb)
        if rgb_img is None:
            return

        # Detect red object
        detection, mask = self.detect_red_object(rgb_img)

        # Publish debug image
        debug_msg = self.create_debug_image(rgb_img, mask, detection)
        self.debug_pub.publish(debug_msg)

        if detection is None:
            return

        cx_px, cy_px, area = detection

        # If depth available, project to 3D
        if self.latest_depth is not None:
            depth_img = self.depth_msg_to_numpy(self.latest_depth)
            if depth_img is not None:
                point_3d = self.pixel_to_3d(cx_px, cy_px, depth_img)
                if point_3d is not None:
                    x, y, z = point_3d

                    # Publish PoseStamped
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = 'camera_optical_frame'
                    pose_msg.pose.position.x = x
                    pose_msg.pose.position.y = y
                    pose_msg.pose.position.z = z
                    pose_msg.pose.orientation.w = 1.0
                    self.pose_pub.publish(pose_msg)

                    # Publish RViz marker
                    self.publish_marker(x, y, z)

                    self.detected_pose = (x, y, z)
                    self.get_logger().debug(
                        f'Detected object at ({x:.3f}, {y:.3f}, {z:.3f}) | area={area}px')
        else:
            # No depth — publish 2D detection info only (z=0)
            self.get_logger().debug(
                f'Detected object at pixel ({cx_px}, {cy_px}) area={area}px (no depth)')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
