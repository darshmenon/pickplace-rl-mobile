# Pick-and-Place RL Mobile Robot

## Overview
The **Pick-and-Place RL Mobile Robot** is a ROS 2 Jazzy-based project that integrates a mobile robotic base and a 4–6 DOF robotic arm. The system uses **Reinforcement Learning (RL)** to autonomously pick objects from a bin and place them at a target location.  

This project combines multiple robotics concepts:
- Mobile navigation
- Robotic arm manipulation
- Grasping and placement
- RL-based policy learning
- ROS 2-based modular architecture
- URDF robot modeling and RViz visualization

The goal is to create an end-to-end autonomous mobile manipulator capable of performing pick-and-place tasks in cluttered environments.

---

## Features
1. **Mobile Manipulator**
   - Differential drive base with encoders and optional IMU.
   - 4–6 DOF robotic arm mounted on top.
   - Parallel gripper for grasping objects.

2. **Reinforcement Learning**
   - RL agent controls the arm (and optionally the base) to pick objects.
   - Uses continuous action space for Cartesian velocities and gripper control.
   - Supports algorithms like SAC and PPO.
   - Reward shaping for approach, grasp, lift, place, smoothness, and collision penalties.

3. **Perception**
   - RGB-D camera or wrist camera for object detection.
   - AprilTag or ArUco markers for bin and goal localization.
   - Optional CNN-based object detection for arbitrary objects.

4. **Simulation and Real Robot**
   - Can be trained in Gazebo, Isaac Sim, or Unity simulation.
   - Supports domain randomization for sim-to-real transfer.
   - ROS 2 integration for running nodes on real hardware.

5. **URDF & Launch**
   - Robot description in `urdf/` folder.
   - Launch files for RViz visualization and `robot_state_publisher` in `launch/`.
   - **URDF Preview:**

   ![URDF Screenshot](images/Screenshot%202025-08-31%20at%203.49.03%E2%80%AFPM.png)

6. **Safety & Monitoring**
   - Workspace and velocity limits for safe arm operation.
   - Collision detection and emergency stop mechanisms.
   - Smoothness and action clamping to prevent jerky movements.

---
