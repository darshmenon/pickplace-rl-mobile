#!/usr/bin/env python3

import argparse
import time
import numpy as np
from stable_baselines3 import SAC
from pickplace_rl_mobile.pickplace_env import PickPlaceEnv

def test_policy(model_path, num_episodes=5):
    """
    Test a trained policy.
    
    Args:
        model_path: Path to saved model
        num_episodes: Number of test episodes
    """
    # Load model
    print(f"Loading model from {model_path}...")
    model = SAC.load(model_path)
    
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
                obj_pos = obs[8:11]
                print(f"  Step {step}: EE pos = [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}], "
                      f"Obj pos = [{obj_pos[0]:.3f}, {obj_pos[1]:.3f}, {obj_pos[2]:.3f}], "
                      f"Reward = {reward:.2f}")
        
        # Episode summary
        episode_rewards.append(episode_reward)
        
        # Check if successful (object near target)
        obj_pos = obs[8:11]
        target_pos = np.array([0.6, 0.5, 0.1])
        distance_to_target = np.linalg.norm(obj_pos - target_pos)
        
        if distance_to_target < 0.15:
            success_count += 1
            print(f"✓ Episode {episode + 1} SUCCESS! Total reward: {episode_reward:.2f}")
        else:
            print(f"✗ Episode {episode + 1} failed. Total reward: {episode_reward:.2f}")
    
    # Final statistics
    print("\n" + "="*50)
    print("Test Results:")
    print(f"  Episodes: {num_episodes}")
    print(f"  Success rate: {success_count}/{num_episodes} ({100*success_count/num_episodes:.1f}%)")
    print(f"  Average reward: {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f}")
    print("="*50)
    
    # Cleanup
    env.close()

def main():
    parser = argparse.ArgumentParser(description='Test trained RL policy')
    parser.add_argument('--model', type=str, required=True,
                        help='Path to trained model (e.g., ./rl_models/pickplace_final_model.zip)')
    parser.add_argument('--episodes', type=int, default=5,
                        help='Number of test episodes (default: 5)')
    
    args = parser.parse_args()
    
    test_policy(args.model, args.episodes)

if __name__ == '__main__':
    main()
