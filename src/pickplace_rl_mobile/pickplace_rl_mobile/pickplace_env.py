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
        
        # Observation space: 5 joint positions + 3 ee pos + 3 obj pos + 1 grasped flag + 1 phase integer
        self.observation_space = spaces.Box(
            low=-np.inf, 
            high=np.inf, 
            shape=(13,), 
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
        self.max_episode_steps = 800
        
        # Target locations
        self.object_start_pos = np.array([0.6, 0.0, 0.055])
        self.target_pos = np.array([0.6, 0.5, 0.1])
        self.object_pos = self.object_start_pos.copy()
        self.object_grasped = False
        
        # State Machine Phase Tracking
        # 0: APPROACH, 1: LOWER, 2: GRASP, 3: LIFT, 4: MOVE, 5: RELEASE
        self.current_phase = 0
        self.prev_distance = None
        
    def joint_state_callback(self, msg):
        """Callback to update joint states from ROS."""
        if len(msg.position) >= 7:
            self.joint_positions = np.array(msg.position[:7])
            self.joint_velocities = np.array(msg.velocity[:7]) if len(msg.velocity) >= 7 else np.zeros(7)
    
    def get_end_effector_pos(self):
        """Compute end effector position using forward kinematics (simplified)."""
        base_height = 0.2  # base_link + arm_base
        
        shoulder_angle = self.joint_positions[0]
        shoulder_pitch = self.joint_positions[1]
        elbow = self.joint_positions[2]
        
        link1_length = 0.24  # arm_link1 height
        link2_length = 0.24  # arm_link2 length
        link3_length = 0.20  # arm_link3 length
        
        z = base_height + link1_length
        
        x_local = link2_length * np.cos(shoulder_pitch) + link3_length * np.cos(shoulder_pitch + elbow)
        z_local = link2_length * np.sin(shoulder_pitch) + link3_length * np.sin(shoulder_pitch + elbow)
        
        x = 0.1 + x_local * np.cos(shoulder_angle)  # 0.1 is base offset
        y = x_local * np.sin(shoulder_angle)
        z = z + z_local
        
        return np.array([x, y, z])
    
    def get_observation(self):
        """Get current observation."""
        ee_pos = self.get_end_effector_pos()
        
        obs = np.concatenate([
            self.joint_positions[:5],  # 5 arm joints
            ee_pos,
            self.object_pos,
            [float(self.object_grasped)],
            [float(self.current_phase)]
        ])
        
        return obs.astype(np.float32)
    
    def compute_reward(self, ee_pos):
        """Compute state-machine based reward."""
        reward = 0.0
        terminated = False
        
        gripper_pos = np.mean(self.joint_positions[5:7])
        
        # Collision Check: Reaching too low while NOT lowering/grasping/releasing
        if ee_pos[2] < 0.10 and self.current_phase not in [1, 2, 5]:
            return -100.0, True # Crash penalty

        # State Machine Progress Tracking
        if self.current_phase == 0:  # APPROACH
            # Goal: Get ee_pos (x,y) above object_pos (x,y), keep z safely up
            target_xy = self.object_pos[:2]
            ee_xy = ee_pos[:2]
            dist_xy = np.linalg.norm(target_xy - ee_xy)
            
            if self.prev_distance is not None:
                reward += (self.prev_distance - dist_xy) * 10.0
                
            self.prev_distance = dist_xy
            
            # Transition to LOWER
            if dist_xy < 0.05 and ee_pos[2] > 0.15:
                self.current_phase = 1
                self.prev_distance = None
                reward += 20.0
                
        elif self.current_phase == 1:  # LOWER
            # Goal: Lower z to object z
            dist_z = abs(ee_pos[2] - 0.07) # slightly above object center
            
            if self.prev_distance is not None:
                reward += (self.prev_distance - dist_z) * 10.0
                
            self.prev_distance = dist_z
            
            # Transition to GRASP
            if dist_z < 0.02:
                self.current_phase = 2
                self.prev_distance = None
                reward += 20.0
                
        elif self.current_phase == 2:  # GRASP
            # Goal: Close Gripper
            if gripper_pos < 0.02: # Gripper closed
                self.object_grasped = True
                self.current_phase = 3
                reward += 50.0 # Substantial bonus
            else:
                reward -= 0.1 # Small penalty for dawdling
                
        elif self.current_phase == 3:  # LIFT
            # Goal: Raise object Z safely
            dist_z = abs(ee_pos[2] - 0.25)
            
            if self.prev_distance is not None:
                reward += (self.prev_distance - dist_z) * 10.0
                
            self.prev_distance = dist_z
            
            # Transition
            if dist_z < 0.05:
                self.current_phase = 4
                self.prev_distance = None
                reward += 20.0
                
        elif self.current_phase == 4:  # MOVE_TO_TARGET
            # Goal: Move XY to target XY
            target_xy = self.target_pos[:2]
            ee_xy = ee_pos[:2]
            dist_xy = np.linalg.norm(target_xy - ee_xy)
            
            if self.prev_distance is not None:
                reward += (self.prev_distance - dist_xy) * 10.0
                
            self.prev_distance = dist_xy
            
            if dist_xy < 0.05:
                self.current_phase = 5
                self.prev_distance = None
                reward += 50.0

        elif self.current_phase == 5:  # RELEASE (and implicitly lower to target)
            dist = np.linalg.norm(ee_pos - self.target_pos)
            
            if self.prev_distance is not None:
                reward += (self.prev_distance - dist) * 10.0
                
            self.prev_distance = dist
            
            if dist < 0.08 and gripper_pos > 0.02: # Lowered AND opened
                self.object_grasped = False
                reward += 100.0 # Final Success
                terminated = True
                
        # Smoothing penalty
        reward -= 0.01 * np.sum(np.abs(self.joint_velocities[:5]))
        
        return reward, terminated

    def step(self, action):
        """Execute one step in the environment."""
        rclpy.spin_once(self.node, timeout_sec=0.01)
        
        joint_velocities = action[:5] * 0.5
        gripper_command = action[5]
        
        self.shoulder_pub.publish(Float64(data=float(joint_velocities[0])))
        self.shoulder_pitch_pub.publish(Float64(data=float(joint_velocities[1])))
        self.elbow_pub.publish(Float64(data=float(joint_velocities[2])))
        self.wrist_roll_pub.publish(Float64(data=float(joint_velocities[3])))
        self.wrist_pitch_pub.publish(Float64(data=float(joint_velocities[4])))
        
        gripper_vel = 0.5 if gripper_command > 0 else -0.5
        self.left_finger_pub.publish(Float64(data=gripper_vel))
        self.right_finger_pub.publish(Float64(data=gripper_vel))
        
        if self.object_grasped:
            ee_pos = self.get_end_effector_pos()
            self.object_pos = ee_pos.copy()
            self.object_pos[2] -= 0.05
        
        time.sleep(0.01)
        
        ee_pos = self.get_end_effector_pos()
        obs = self.get_observation()
        
        reward, terminated = self.compute_reward(ee_pos)
        
        self.episode_steps += 1
        truncated = self.episode_steps >= self.max_episode_steps
        
        return obs, reward, terminated, truncated, {}
    
    def reset(self, seed=None, options=None):
        """Reset the environment."""
        super().reset(seed=seed)
        
        self.episode_steps = 0
        self.object_grasped = False
        self.current_phase = 0
        self.prev_distance = None
        
        # Randomize Object Location within reach
        random_x = 0.5 + np.random.rand() * 0.2 # 0.5 to 0.7
        random_y = -0.2 + np.random.rand() * 0.4 # -0.2 to 0.2
        self.object_start_pos = np.array([random_x, random_y, 0.055])
        self.object_pos = self.object_start_pos.copy()
        
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
