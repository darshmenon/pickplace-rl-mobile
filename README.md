# Pick-and-Place RL Mobile Manipulator

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-green)](https://python.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A complete **autonomous mobile manipulator** system that combines a differential-drive mobile base with a 4-DOF robotic arm to perform pick-and-place tasks. The robot uses **Reinforcement Learning (SAC algorithm)** to learn how to navigate towards objects, pick them up, and place them at a target location — all trained end-to-end in simulation.

The project features a full **RGB-D perception pipeline** for camera-based object detection, a **2D LiDAR** for Nav2 obstacle-aware navigation, a **real-time safety guard** with emergency stop, and **domain randomization** for sim-to-real transfer — making it a comprehensive showcase of modern mobile manipulation in ROS 2.

![Gazebo Simulation](./images/gazebo_simulation.png)
![Robot Model](./images/gazebo_robot.png)

## Project Goal

The goal of this project is to build an **end-to-end autonomous mobile manipulator** that can:

1. **Navigate** to an object using its mobile base (differential drive + Nav2)
2. **Detect** the object using onboard RGB-D camera perception (HSV color segmentation + depth projection)
3. **Pick up** the object using a 4-DOF arm controlled by a trained RL policy
4. **Place** the object at a designated target zone
5. Do all of this **autonomously** with real-time safety monitoring

This bridges the gap between navigation and manipulation, training a single SAC policy that controls both the mobile base and the arm simultaneously.

---

## Key Features

| Feature | Description |
|---------|-------------|
| **Mobile Manipulator** | Diff-drive base + 4-DOF arm + parallel gripper in a single URDF |
| **RGB-D Perception** | HSV color segmentation + depth projection for 3D object detection |
| **RL Training (SAC)** | 8-dim continuous actions, 16-dim observations, 6-phase state machine |
| **Nav2 Navigation** | LiDAR-based AMCL + DWB planner for autonomous navigation |
| **Safety Guard** | Joint limits, workspace bounds, obstacle e-stop |
| **Domain Randomization** | Object/target position, color, physics noise for sim-to-real |
| **TensorBoard** | Full training metrics and evaluation logging |

---

## Quick Start

```bash
# Build
colcon build --packages-select pickplace_rl_mobile
source install/setup.bash

# Launch full system (Gazebo + Perception + Safety)
ros2 launch pickplace_rl_mobile full_system.launch.py

# Or with Nav2 navigation
ros2 launch pickplace_rl_mobile full_system.launch.py use_nav2:=true

# Or with trained RL policy
ros2 launch pickplace_rl_mobile full_system.launch.py use_rl:=true model_path:=./rl_models/pickplace_final_model.zip
```

---

## Full Documentation

See **[readme.md](./readme.md)** for complete documentation including:
- Detailed system architecture deep-dive
- Full directory structure
- Installation prerequisites
- Training guide with hyperparameters
- Topic reference table
- Configuration guide
- Troubleshooting

---

## Maintainer

**Darsh Menon**
- Email: darshmenon02@gmail.com
- GitHub: [@darshmenon](https://github.com/darshmenon)
