#!/usr/bin/env python3
"""Train right arm servos using MuJoCo simulation with continuous rotation.
Uses timing model to simulate real servo behavior.
"""

import numpy as np
import time
import json
from right_arm_model import ArmServoModel

try:
    import mujoco
    import mujoco_viewer
except ImportError:
    print("MuJoCo not installed. Install with:")
    print("pip install mujoco mujoco-python")
    exit(1)

class RightArmTrainer:
    def __init__(self):
        self.model = ArmServoModel()
        self.servo_ids = [1, 2, 3]
        
        # Load MuJoCo model
        self.mujoco_model = mujoco.MjModel.from_xml_path("right_arm_robot.xml")
        self.data = mujoco.MjData(self.mujoco_model)
        
        # Training parameters
        self.episodes = 1000
        self.max_steps = 200
        self.learning_rate = 0.01
        
        # Initialize Q-table
        self.q_table = {}
        self.state_space = 3  # -1, 0, 1 (negative, center, positive)
        self.action_space = 3  # -1, 0, 1 (rotate negative, stop, rotate positive)
        
    def get_state(self, servo_id):
        """Get current state for servo"""
        if servo_id == 1:  # Shoulder
            return int(np.clip(self.data.qpos[0] * 3, -1, 1))
        elif servo_id == 2:  # Upper arm
            return int(np.clip(self.data.qpos[1] * 3, -1, 1))
        elif servo_id == 3:  # Forearm
            return int(np.clip(self.data.qpos[2] * 3, -1, 1))
        return 0
    
    def get_reward(self, servo_id, target_angle, current_angle):
        """Calculate reward for reaching target"""
        error = abs(target_angle - current_angle)
        reward = -error  # Negative error as reward
        
        # Bonus for reaching target
        if error < 0.1:
            reward += 10
        
        return reward
    
    def choose_action(self, state, servo_id, epsilon=0.1):
        """Epsilon-greedy action selection"""
        key = f"{servo_id}_{state}"
        if np.random.random() < epsilon or key not in self.q_table:
            return np.random.choice([-1, 0, 1])
        
        return np.argmax(self.q_table[key]) - 1
    
    def update_q_table(self, state, action, reward, next_state, servo_id):
        """Update Q-table using Q-learning"""
        key = f"{servo_id}_{state}"
        next_key = f"{servo_id}_{next_state}"
        
        if key not in self.q_table:
            self.q_table[key] = [0, 0, 0]
        if next_key not in self.q_table:
            self.q_table[next_key] = [0, 0, 0]
        
        old_value = self.q_table[key][action + 1]
        next_max = max(self.q_table[next_key])
        
        new_value = old_value + self.learning_rate * (reward + 0.95 * next_max - old_value)
        self.q_table[key][action + 1] = new_value
    
    def run_episode(self, servo_id, target_angle):
        """Run one training episode"""
        self.data.qpos[:] = 0  # Reset to center
        mujoco.mj_step(self.mujoco_model, self.data)
        
        total_reward = 0
        state = self.get_state(servo_id)
        
        for step in range(self.max_steps):
            action = self.choose_action(state, servo_id)
            
            # Apply action to servo
            if action != 0:
                self.model.rotate_servo(servo_id, action * 10, 0.1)
            
            # Get current angle
            if servo_id == 1:
                current_angle = self.data.qpos[0]
            elif servo_id == 2:
                current_angle = self.data.qpos[1]
            elif servo_id == 3:
                current_angle = self.data.qpos[2]
            
            # Calculate reward
            reward = self.get_reward(servo_id, target_angle, current_angle)
            total_reward += reward
            
            # Update state
            next_state = self.get_state(servo_id)
            self.update_q_table(state, action, reward, next_state, servo_id)
            
            state = next_state
            
            # Check if reached target
            if abs(target_angle - current_angle) < 0.1:
                break
        
        return total_reward
    
    def train(self):
        """Train all servos"""
        print("Starting right arm training...")
        
        for servo_id in self.servo_ids:
            print(f"\nTraining Servo {servo_id}")
            
            for episode in range(self.episodes):
                # Random target angle
                target_angle = np.random.uniform(-np.pi/4, np.pi/4)
                
                reward = self.run_episode(servo_id, target_angle)
                
                if episode % 100 == 0:
                    print(f"  Episode {episode}: Reward = {reward:.2f}")
        
        print("\nTraining complete!")
    
    def test(self):
        """Test trained model"""
        print("\nTesting trained model...")
        
        for servo_id in self.servo_ids:
            print(f"\nTesting Servo {servo_id}")
            
            # Test different angles
            test_angles = [-30, -15, 0, 15, 30]
            
            for angle in test_angles:
                print(f"  Moving to {angle}°")
                self.model.rotate_servo(servo_id, angle, 1.0)
                time.sleep(0.5)

if __name__ == "__main__":
    trainer = RightArmTrainer()
    
    import sys
    if "--headless" in sys.argv:
        trainer.train()
    else:
        # Create viewer
        viewer = mujoco_viewer.MujocoViewer(trainer.mujoco_model, trainer.data)
        
        try:
            trainer.train()
        except KeyboardInterrupt:
            print("Training interrupted")
        finally:
            viewer.close()