#!/usr/bin/env python3
import rclpy
from pickplace_rl_mobile.pickplace_env import PickPlaceEnv

def main():
    rclpy.init()
    print("Testing PickPlaceEnv")
    try:
        env = PickPlaceEnv()
        print("Environment created successfully")
        
        print("Resetting environment...")
        obs, info = env.reset()
        print(f"Observation shape: {obs.shape}")
        
        print("Stepping environment...")
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"Step successful. Reward: {reward}")
        
    except Exception as e:
        print(f"Error testing environment: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'env' in locals():
            env.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
