#!/usr/bin/env python3

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Float64
import time

class PickPlaceEnv(gym.Env):
    """
    Gymnasium environment for pick-and-place RL training.
    
    Observation: [joint_positions(5), end_effector_pos(3), object_pos(3), object_grasped(1)]
    Action: [joint_velocities(5), gripper_control(1)]
    """
    
    def __init__(self):
        super().__init__()
        
        # Initialize ROS 2
        if not rclpy.ok():
            rclpy.init()
        
        self.node = Node('pickplace_env_node')
        
        # Action space: 5 joint velocities + gripper control
        self.action_space = spaces.Box(
            low=-1.0, 
            high=1.0, 
            shape=(6,), 
            dtype=np.float32
        )
        
        # Observation space: 5 joint positions + 3 ee pos + 3 obj pos + 1 grasped flag
        self.observation_space = spaces.Box(
            low=-np.inf, 
            high=np.inf, 
            shape=(12,), 
            dtype=np.float32
        )
        
        # Publishers
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Joint velocity publishers (individual topics for Gazebo Harmonic)
        self.shoulder_pub = self.node.create_publisher(Float64, '/shoulder_joint/cmd_vel', 10)
        self.shoulder_pitch_pub = self.node.create_publisher(Float64, '/shoulder_pitch_joint/cmd_vel', 10)
        self.elbow_pub = self.node.create_publisher(Float64, '/elbow_joint/cmd_vel', 10)
        self.wrist_roll_pub = self.node.create_publisher(Float64, '/wrist_roll_joint/cmd_vel', 10)
        self.wrist_pitch_pub = self.node.create_publisher(Float64, '/wrist_pitch_joint/cmd_vel', 10)
        self.left_finger_pub = self.node.create_publisher(Float64, '/left_finger_joint/cmd_vel', 10)
        self.right_finger_pub = self.node.create_publisher(Float64, '/right_finger_joint/cmd_vel', 10)
        
        # Subscribers
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # State variables
        self.joint_positions = np.zeros(7)  # 5 arm joints + 2 gripper
        self.joint_velocities = np.zeros(7)
        self.episode_steps = 0
        self.max_episode_steps = 500
        
        # Target locations
        self.object_start_pos = np.array([0.6, 0.0, 0.055])
        self.target_pos = np.array([0.6, 0.5, 0.1])
        self.object_pos = self.object_start_pos.copy()
        self.object_grasped = False
        
        # Previous distance for reward shaping
        self.prev_distance_to_object = None
        self.prev_distance_to_target = None
        
    def joint_state_callback(self, msg):
        """Callback to update joint states from ROS."""
        # Map joint names to indices
        # Expected joints: shoulder_joint, shoulder_pitch_joint, elbow_joint, 
        #                  wrist_roll_joint, wrist_pitch_joint, left_finger_joint, right_finger_joint
        if len(msg.position) >= 7:
            self.joint_positions = np.array(msg.position[:7])
            self.joint_velocities = np.array(msg.velocity[:7]) if len(msg.velocity) >= 7 else np.zeros(7)
    
    def get_end_effector_pos(self):
        """Compute end effector position using forward kinematics (simplified)."""
        # Simplified FK - in practice, use proper kinematics library
        # For now, use approximate positions based on joint angles
        base_height = 0.1 + 0.1  # base_link + arm_base
        
        shoulder_angle = self.joint_positions[0]
        shoulder_pitch = self.joint_positions[1]
        elbow = self.joint_positions[2]
        
        # Simplified 2D FK in x-z plane, rotated by shoulder angle
        link1_length = 0.2  # arm_link1 height
        link2_length = 0.2  # arm_link2 length
        link3_length = 0.16  # arm_link3 length
        
        # Vertical reach
        z = base_height + link1_length
        
        # Horizontal reach in local frame
        x_local = link2_length * np.cos(shoulder_pitch) + link3_length * np.cos(shoulder_pitch + elbow)
        z_local = link2_length * np.sin(shoulder_pitch) + link3_length * np.sin(shoulder_pitch + elbow)
        
        # Rotate by shoulder angle
        x = 0.1 + x_local * np.cos(shoulder_angle)  # 0.1 is base offset
        y = x_local * np.sin(shoulder_angle)
        z = z + z_local
        
        return np.array([x, y, z])
    
    def get_observation(self):
        """Get current observation."""
        ee_pos = self.get_end_effector_pos()
        
        # Observation: [joint_pos(5), ee_pos(3), obj_pos(3), grasped(1)]
        obs = np.concatenate([
            self.joint_positions[:5],  # 5 arm joints
            ee_pos,
            self.object_pos,
            [float(self.object_grasped)]
        ])
        
        return obs.astype(np.float32)
    
    def compute_reward(self, obs):
        """Compute reward based on current state."""
        ee_pos = obs[5:8]
        obj_pos = obs[8:11]
        
        # Distance to object
        distance_to_object = np.linalg.norm(ee_pos - obj_pos)
        
        # Distance from object to target
        distance_to_target = np.linalg.norm(obj_pos - self.target_pos)
        
        reward = 0.0
        
        # Phase 1: Approaching object
        if not self.object_grasped:
            # Reward for getting closer to object
            if self.prev_distance_to_object is not None:
                reward += (self.prev_distance_to_object - distance_to_object) * 10.0
            
            # Bonus for being very close
            if distance_to_object < 0.05:
                reward += 5.0
                
            # Grasp detection (simplified - if gripper closed and close to object)
            gripper_pos = np.mean(self.joint_positions[5:7])
            if distance_to_object < 0.05 and gripper_pos > 0.02:
                self.object_grasped = True
                reward += 50.0  # Big bonus for grasping
                
        # Phase 2: Moving to target with object
        else:
            # Reward for moving object to target
            if self.prev_distance_to_target is not None:
                reward += (self.prev_distance_to_target - distance_to_target) * 20.0
            
            # Big bonus for successful placement
            if distance_to_target < 0.1 and ee_pos[2] < 0.15:  # At target and lowered
                reward += 100.0
                
        # Penalty for excessive joint velocities (smoothness)
        velocity_penalty = -0.01 * np.sum(np.abs(self.joint_velocities[:5]))
        reward += velocity_penalty
        
        # Update previous distances
        self.prev_distance_to_object = distance_to_object
        self.prev_distance_to_target = distance_to_target
        
        return reward
    
    def step(self, action):
        """Execute one step in the environment."""
        # Spin ROS node to process callbacks
        rclpy.spin_once(self.node, timeout_sec=0.01)
        
        # Apply actions (scale to appropriate ranges)
        # Action: [shoulder, shoulder_pitch, elbow, wrist_roll, wrist_pitch, gripper]
        joint_velocities = action[:5] * 0.5  # Scale down for safety
        gripper_command = action[5]
        
        # Publish joint velocities to individual controllers
        self.shoulder_pub.publish(Float64(data=float(joint_velocities[0])))
        self.shoulder_pitch_pub.publish(Float64(data=float(joint_velocities[1])))
        self.elbow_pub.publish(Float64(data=float(joint_velocities[2])))
        self.wrist_roll_pub.publish(Float64(data=float(joint_velocities[3])))
        self.wrist_pitch_pub.publish(Float64(data=float(joint_velocities[4])))
        
        # Gripper control (simplified: open/close based on sign)
        gripper_vel = 0.1 if gripper_command > 0 else -0.1
        self.left_finger_pub.publish(Float64(data=gripper_vel))
        self.right_finger_pub.publish(Float64(data=gripper_vel))
        
        # Update object position if grasped (simplified physics)
        if self.object_grasped:
            ee_pos = self.get_end_effector_pos()
            self.object_pos = ee_pos.copy()
            self.object_pos[2] -= 0.05  # Object hangs below gripper
        
        # Small delay to simulate real-time
        time.sleep(0.01)
        
        # Get observation
        obs = self.get_observation()
        
        # Compute reward
        reward = self.compute_reward(obs)
        
        # Check termination
        self.episode_steps += 1
        terminated = False
        truncated = self.episode_steps >= self.max_episode_steps
        
        # Check success
        distance_to_target = np.linalg.norm(self.object_pos - self.target_pos)
        if self.object_grasped and distance_to_target < 0.1:
            terminated = True
            reward += 100.0  # Success bonus
        
        return obs, reward, terminated, truncated, {}
    
    def reset(self, seed=None, options=None):
        """Reset the environment."""
        super().reset(seed=seed)
        
        # Reset counters
        self.episode_steps = 0
        self.object_grasped = False
        
        # Reset object position
        self.object_pos = self.object_start_pos.copy()
        
        # Reset distance tracking
        self.prev_distance_to_object = None
        self.prev_distance_to_target = None
        
        # Wait for joint states to be updated
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.01)
        
        obs = self.get_observation()
        
        return obs, {}
    
    def close(self):
        """Cleanup."""
        if rclpy.ok():
            self.node.destroy_node()
            rclpy.shutdown()
