#!/usr/bin/env python3
"""
Safety Guard Node for the Pick-and-Place Mobile Manipulator.

Monitors robot state and sensor data to enforce safety constraints:
- Joint position limits
- End-effector workspace boundaries
- LiDAR-based obstacle proximity
- Emergency stop capability
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json


class SafetyGuard(Node):
    def __init__(self):
        super().__init__('safety_guard')

        # --- Parameters ---
        self.declare_parameter('joint_limit_margin', 0.1)  # radians from limit
        self.declare_parameter('min_obstacle_distance', 0.25)  # meters
        self.declare_parameter('ee_min_height', 0.02)  # meters
        self.declare_parameter('ee_max_reach', 0.75)  # meters from base
        self.declare_parameter('workspace_radius', 3.0)  # meters from origin
        self.declare_parameter('monitor_rate', 20.0)  # Hz

        self.joint_limit_margin = self.get_parameter('joint_limit_margin').value
        self.min_obstacle_dist = self.get_parameter('min_obstacle_distance').value
        self.ee_min_height = self.get_parameter('ee_min_height').value
        self.ee_max_reach = self.get_parameter('ee_max_reach').value
        self.workspace_radius = self.get_parameter('workspace_radius').value

        # Joint limits from URDF (name -> [lower, upper])
        self.joint_limits = {
            'shoulder_joint': [-3.14, 3.14],
            'shoulder_pitch_joint': [-1.57, 1.57],
            'elbow_joint': [-2.35, 2.35],
            'wrist_roll_joint': [-3.14, 3.14],
            'wrist_pitch_joint': [-1.57, 1.57],
            'left_finger_joint': [-0.01, 0.03],
            'right_finger_joint': [-0.01, 0.03],
        }

        # Robot kinematics parameters (matching pickplace_env.py)
        self.base_height = 0.2
        self.link1_length = 0.24
        self.link2_length = 0.24
        self.link3_length = 0.20

        # State
        self.joint_positions = {}
        self.joint_names = []
        self.base_pose = np.zeros(3)
        self.min_lidar_distance = float('inf')
        self.e_stop_active = False
        self.violations = []

        # --- Subscribers ---
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # --- Publishers ---
        self.status_pub = self.create_publisher(String, '/safety/status', 10)
        self.estop_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Monitor timer
        rate = self.get_parameter('monitor_rate').value
        self.timer = self.create_timer(1.0 / rate, self.monitor_safety)

        self.get_logger().info('Safety Guard initialized — monitoring joint limits, workspace, obstacles')

    def joint_callback(self, msg):
        """Update joint positions."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
        self.joint_names = list(msg.name)

    def odom_callback(self, msg):
        """Update base pose."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = np.arctan2(siny_cosp, cosy_cosp)
        self.base_pose = np.array([x, y, theta])

    def scan_callback(self, msg):
        """Update minimum LiDAR distance."""
        ranges = np.array(msg.ranges)
        valid = ranges[(ranges > msg.range_min) & (ranges < msg.range_max)]
        if len(valid) > 0:
            self.min_lidar_distance = float(np.min(valid))
        else:
            self.min_lidar_distance = float('inf')

    def compute_ee_position(self):
        """Compute approximate end-effector position from joint states."""
        shoulder = self.joint_positions.get('shoulder_joint', 0.0)
        shoulder_pitch = self.joint_positions.get('shoulder_pitch_joint', 0.0)
        elbow = self.joint_positions.get('elbow_joint', 0.0)

        z = self.base_height + self.link1_length
        x_local = (self.link2_length * np.cos(shoulder_pitch) +
                   self.link3_length * np.cos(shoulder_pitch + elbow))
        z_local = (self.link2_length * np.sin(shoulder_pitch) +
                   self.link3_length * np.sin(shoulder_pitch + elbow))

        x = 0.1 + x_local * np.cos(shoulder)
        y = x_local * np.sin(shoulder)
        z = z + z_local

        return np.array([x, y, z])

    def check_joint_limits(self):
        """Check if any joints are near their limits."""
        violations = []
        for name, limits in self.joint_limits.items():
            pos = self.joint_positions.get(name, 0.0)
            lower, upper = limits
            margin = self.joint_limit_margin

            if pos <= lower + margin:
                violations.append(f'{name} near lower limit ({pos:.3f} <= {lower + margin:.3f})')
            elif pos >= upper - margin:
                violations.append(f'{name} near upper limit ({pos:.3f} >= {upper - margin:.3f})')
        return violations

    def check_workspace_bounds(self):
        """Check end-effector and base workspace limits."""
        violations = []

        # Check base position is within workspace
        base_dist = np.sqrt(self.base_pose[0]**2 + self.base_pose[1]**2)
        if base_dist > self.workspace_radius:
            violations.append(
                f'Base outside workspace (dist={base_dist:.2f}m > {self.workspace_radius}m)')

        # Check EE position
        ee_pos = self.compute_ee_position()
        if ee_pos[2] < self.ee_min_height:
            violations.append(
                f'EE too low (z={ee_pos[2]:.3f}m < {self.ee_min_height}m)')

        ee_reach = np.sqrt(ee_pos[0]**2 + ee_pos[1]**2)
        if ee_reach > self.ee_max_reach:
            violations.append(
                f'EE overextended (reach={ee_reach:.2f}m > {self.ee_max_reach}m)')

        return violations

    def check_obstacle_proximity(self):
        """Check LiDAR for nearby obstacles."""
        violations = []
        if self.min_lidar_distance < self.min_obstacle_dist:
            violations.append(
                f'Obstacle too close (dist={self.min_lidar_distance:.2f}m < {self.min_obstacle_dist}m)')
        return violations

    def emergency_stop(self):
        """Send zero velocity on all axes."""
        if not self.e_stop_active:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            self.e_stop_active = True

        stop_msg = Twist()
        self.estop_cmd_pub.publish(stop_msg)

    def monitor_safety(self):
        """Main safety monitoring loop."""
        if not self.joint_positions:
            return  # No data yet

        self.violations = []

        # Run all checks
        self.violations.extend(self.check_joint_limits())
        self.violations.extend(self.check_workspace_bounds())
        self.violations.extend(self.check_obstacle_proximity())

        # Determine safety status
        if self.violations:
            severity = 'WARNING'
            # Critical violations trigger e-stop
            critical_keywords = ['Obstacle too close', 'EE too low']
            is_critical = any(
                kw in v for v in self.violations for kw in critical_keywords)

            if is_critical:
                severity = 'CRITICAL'
                self.emergency_stop()
            else:
                self.e_stop_active = False
        else:
            severity = 'OK'
            self.e_stop_active = False

        # Publish status
        status = {
            'severity': severity,
            'violations': self.violations,
            'ee_position': self.compute_ee_position().tolist(),
            'min_obstacle_dist': round(self.min_lidar_distance, 3),
            'e_stop': self.e_stop_active
        }

        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)

        if self.violations:
            self.get_logger().warn(
                f'Safety [{severity}]: {"; ".join(self.violations)}')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
