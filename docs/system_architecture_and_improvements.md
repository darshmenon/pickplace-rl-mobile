# System Architecture & Future Improvements

This document outlines the software system architecture of the ARES Pick-and-Place RL Mobile Robot and details potential areas for future development and scaling.

## 1. System Architecture

The software stack follows a highly modular **Sense-Plan-Act** architecture built entirely on ROS 2 Jazzy and Python 3.10+. 

```
┌─────────────────────────────────────────────────────────────────┐
│                        Gazebo Harmonic                          │
│  ┌──────────────┐  ┌──────────┐  ┌──────────┐  ┌───────────┐  │
│  │ Diff Drive    │  │ RGB Cam  │  │ Depth Cam│  │ 2D LiDAR  │  │
│  │ /cmd_vel      │  │ /camera/ │  │ /camera/ │  │ /scan     │  │
│  │ /odom         │  │ image_raw│  │ depth    │  │           │  │
│  └──────┬───────┘  └────┬─────┘  └────┬─────┘  └─────┬─────┘  │
│         │               │             │               │        │
│  ┌──────┴───────────────┴─────────────┴───────────────┴─────┐  │
│  │                    ros_gz_bridge                          │  │
│  └──────┬───────────────┬─────────────┬───────────────┬─────┘  │
└─────────┼───────────────┼─────────────┼───────────────┼────────┘
          │               │             │               │
    ┌─────┴─────┐  ┌──────┴─────────────┴───────┐ ┌────┴────────┐
    │  ManipRL  │  │     Perception Node        │ │ Safety Guard│
    │  Node     │←─│  HSV Segmentation          │ │ Joint Limits│
    │  (SAC)    │  │  Depth 3D Projection       │ │ Obstacle    │
    │           │  │  /perception/detected_obj  │ │ E-Stop      │
    └─────┬─────┘  └────────────────────────────┘ └─────────────┘
          │
    ┌─────┴─────┐
    │   Nav2    │
    │  (opt.)   │
    │  AMCL +   │
    │  DWB      │
    └───────────┘
```

### 1.1 Perception Node (`perception_node.py`)
Decouples vision from the RL environment. It subscribes to `/camera/image_raw` and `/camera/depth`.
- Converts RGB to HSV (using pure NumPy for speed) to segment object colors.
- Computes the 2D pixel centroid of the detected object mask.
- Samples the 10x10 depth neighborhood to find the median depth.
- Uses camera intrinsics to back-project this depth into a 3D point in the camera frame, publishing it to `/perception/detected_object`.

### 1.2 Safety Guard Node (`safety_guard.py`)
A continuous 20Hz monitoring system to protect both simulated and real-world hardware.
- Subscribes to `/joint_states`, `/odom`, and `/scan`.
- Checks joint limits (giving warnings on proximity).
- Checks bounding areas (preventing the EE from hitting the ground or over-extending, and stopping the base from leaving the operational radius).
- Checks LiDAR minimum ranges, sending emergency `/cmd_vel` stop commands if an obstacle is within 25cm.

### 1.3 Reinforcement Learning Node (`manip_rl_node.py` & `pickplace_env.py`)
- **Environment**: A custom Gymnasium environment that defines a 16-D observation space (joints, EE xyz, object xyz, grasp state, phase, base pose) and an 8-D continuous action space (joint velocities, gripper binary, base linear/angular).
- **Curriculum State Machine**: Uses strict phases (APPROACH, LOWER, GRASP, LIFT, MOVE_TO_TARGET, RELEASE) with dense reward shaping to guide the SAC algorithm to convergence.
- **Inference Node**: Loads the trained Stable-Baselines3 `.zip` model and continuously publishes actions at 20Hz based on real-time sensor feedback.

### 1.4 Domain Randomizer (`domain_randomizer.py`)
Crucial for bridging the **sim-to-real gap**. Injects Gaussian noise into sensor readings, randomizes friction and mass, scales the object size visually, and randomizes spawn locations in Gazebo.

---

## 2. Future Improvements & Scope

The current implementation is a robust baseline. To scale the system into a production-level open-vocabulary mobile manipulator, several features can be added:

### 2.1 Integrating Vision-Language-Action (VLA) Models
Instead of a simple HSV block detector, the perception node can be swapped for a large visual-language model (like OpenVLA or RT-2). 
- **Open-Vocabulary Picking**: The robot could accept commands like "Pick up the yellow banana" rather than just looking for a hardcoded red cube.
- **Language Commanding**: Introducing edge LLMs and Whisper transcription for voice commanding.

### 2.2 Dynamic Obstacle Avoidance for the Arm
Currently, the mobile base avoids obstacles via Nav2, and the safety guard prevents collisions, but the RL policy assumes an empty workspace for the arm.
- **Improvement**: Pass LiDAR or depth voxel grid data into the RL observation space, or use **MoveIt2** for safe, obstacle-aware motion planning of the UR3 arm.

### 2.3 Sim-to-Real Deployment
- Deploy the system to physical hardware (e.g., a Clearpath Jackal or custom differential drive base + a real UR3e arm).
- Replace simulated cameras with an Intel RealSense D435i.
- Replace the Gazebo plugins with `ros2_control` hardware interfaces.
- Validate the domain randomization parameters and fine-tune the SAC agent on real-world data (Sim-to-Real Transfer).

### 2.4 Multi-Object Sorting Task
- Update the RL reward function and state machine to recognize multiple objects with different classes.
- Train a policy to pick sequentially and place them into corresponding target bins based on color or shape classification.
