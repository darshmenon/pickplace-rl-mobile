#!/usr/bin/env python3

import argparse
import time
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from stable_baselines3 import SAC
from pickplace_rl_mobile.pickplace_env import PickPlaceEnv
import threading

class ImageRecorder(Node):
    def __init__(self, save_dir='images'):
        super().__init__('image_recorder')
        self.bridge = CvBridge()
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)
        self.latest_image = None
        self.lock = threading.Lock()
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            
    def save_snapshot(self, filename):
        with self.lock:
            if self.latest_image is not None:
                path = os.path.join(self.save_dir, filename)
                cv2.imwrite(path, self.latest_image)
                return True
        return False

def test_policy(model_path, num_episodes=5):
    """
    Test a trained policy and record images.
    """
    # Initialize ROS for image recording
    if not rclpy.ok():
        rclpy.init()
    
    recorder = ImageRecorder()
    
    # Spin recorder in separate thread
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(recorder)
    spinner_thread = threading.Thread(target=executor.spin, daemon=True)
    spinner_thread.start()
    
    # Load model
    print(f"Loading model from {model_path}...")
    try:
        model = SAC.load(model_path)
    except Exception as e:
        print(f"Error loading model: {e}")
        return

    # Create environment
    print("Creating environment...")
    env = PickPlaceEnv()
    
    # Run episodes
    episode_rewards = []
    success_count = 0
    
    for episode in range(num_episodes):
        print(f"\n=== Episode {episode + 1}/{num_episodes} ===")
        obs, _ = env.reset()
        episode_reward = 0
        done = False
        step = 0
        
        # Save start image
        time.sleep(1.0) # Wait for camera
        recorder.save_snapshot(f"episode_{episode+1}_start.png")
        
        while not done:
            # Get action from policy
            action, _ = model.predict(obs, deterministic=True)
            
            # Take step
            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward
            done = terminated or truncated
            step += 1
            
            # Print progress
            if step % 50 == 0:
                ee_pos = obs[5:8]
                print(f"  Step {step}: Reward = {reward:.2f}")
        
        # Episode summary
        episode_rewards.append(episode_reward)
        
        # Check success
        obj_pos = obs[8:11]
        target_pos = np.array([0.6, 0.5, 0.1])
        distance_to_target = np.linalg.norm(obj_pos - target_pos)
        
        if distance_to_target < 0.15:
            success_count += 1
            print(f"✓ Episode {episode + 1} SUCCESS!")
            recorder.save_snapshot(f"episode_{episode+1}_success.png")
        else:
            print(f"✗ Episode {episode + 1} failed.")
            recorder.save_snapshot(f"episode_{episode+1}_fail.png")
            
    # Final statistics
    print("\n" + "="*50)
    print(f"Success rate: {success_count}/{num_episodes} ({100*success_count/num_episodes:.1f}%)")
    print(f"Average reward: {np.mean(episode_rewards):.2f}")
    print("="*50)
    
    # Cleanup
    env.close()
    recorder.destroy_node()
    rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser(description='Test trained RL policy')
    parser.add_argument('--model', type=str, required=True,
                        help='Path to trained model')
    parser.add_argument('--episodes', type=int, default=5,
                        help='Number of test episodes')
    
    args = parser.parse_args()
    
    test_policy(args.model, args.episodes)

if __name__ == '__main__':
    main()
