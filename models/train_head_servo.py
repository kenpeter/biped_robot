"""
Train Head Servo in Isaac Sim
Goal: Train head (channel 0) to move in 10° increments

Run: ./run_isaac.sh train_head_servo.py
"""

import os
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import random
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import create_prim

ROBOT_USD = os.path.join(os.getcwd(), "simple_robot.usda")


class HeadServoEnv:
    """Train head servo only"""

    def __init__(self):
        self.world = World()
        self.world.scene.add_default_ground_plane()

        # Load robot - spawn high so all parts visible
        create_prim("/World/Humanoid", "Xform", usd_path=ROBOT_USD,
                   position=np.array([0.0, 0.0, 0.5]))
        self.world.reset()

        self.robot = self.world.scene.add(
            Articulation("/World/Humanoid", name="robot"))

        self.num_dof = self.robot.num_dof
        self.target_angle = 0.0
        self.step_count = 0
        self.max_steps = 200

        print(f"Loaded {self.num_dof} DOF robot")

    def reset(self):
        """Reset to neutral pose, pick random target"""
        self.robot.set_joint_positions(np.zeros(self.num_dof))
        self.robot.set_joint_velocities(np.zeros(self.num_dof))

        # Random target in 10° increments (-50° to +50°)
        self.target_angle = random.randint(-5, 5) * np.radians(10)
        self.step_count = 0

        for _ in range(10):  # Let robot settle
            self.world.step(render=True)

        return self._get_obs()

    def _get_obs(self):
        """Return [head_angle, head_velocity, target_angle]"""
        pos = self.robot.get_joint_positions()[0]
        vel = self.robot.get_joint_velocities()[0]
        return np.array([pos, vel, self.target_angle], dtype=np.float32)

    def step(self, action):
        """Apply action (head torque), return obs, reward, done"""
        # Apply action to head joint (index 0)
        efforts = np.zeros(self.num_dof)
        efforts[0] = action * 10.0

        # Hold other joints stable with PD control
        if self.num_dof > 1:
            pos = self.robot.get_joint_positions()
            vel = self.robot.get_joint_velocities()
            for i in range(1, self.num_dof):
                efforts[i] = -100.0 * pos[i] - 10.0 * vel[i]

        self.robot.set_joint_efforts(efforts)
        self.world.step(render=True)

        # Get observation
        obs = self._get_obs()
        error = abs(obs[0] - self.target_angle)

        # Reward: negative error + bonus for being close
        reward = -error * 10.0
        if error < np.radians(5):
            reward += 5.0

        # Check done
        self.step_count += 1
        root_z = self.robot.get_world_pose()[0][2]
        done = (self.step_count >= self.max_steps) or (error < np.radians(2)) or (root_z < 0.15)

        return obs, reward, done, {"error_deg": np.degrees(error), "height": root_z}

    def close(self):
        """Clean up"""
        simulation_app.close()


class Policy(nn.Module):
    """Simple 2-layer network"""
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(3, 32), nn.ReLU(),
            nn.Linear(32, 32), nn.ReLU(),
            nn.Linear(32, 1), nn.Tanh())

    def forward(self, x):
        return self.net(x)


def train(env, episodes=50):
    """Simple training with random exploration"""
    policy = Policy()
    optimizer = optim.Adam(policy.parameters(), lr=0.001)
    buffer = []

    print(f"\nTraining {episodes} episodes...")

    for ep in range(episodes):
        obs = env.reset()
        ep_reward = 0

        for step in range(env.max_steps):
            # Action: random early on, then use policy
            if random.random() < max(0.1, 1.0 - ep/30):
                action = random.uniform(-1, 1)
            else:
                with torch.no_grad():
                    action = policy(torch.FloatTensor(obs)).item()

            next_obs, reward, done, info = env.step(action)
            buffer.append((obs, action, reward))
            ep_reward += reward
            obs = next_obs

            if done:
                break

        # Train every episode on good experiences
        if len(buffer) > 32:
            samples = random.sample(buffer, 32)
            obs_t = torch.FloatTensor([s[0] for s in samples])
            act_t = torch.FloatTensor([s[1] for s in samples]).unsqueeze(1)
            rew_t = torch.FloatTensor([s[2] for s in samples])

            # Learn from good actions (positive reward)
            pred = policy(obs_t)
            loss = torch.mean((pred[rew_t > 0] - act_t[rew_t > 0]) ** 2)

            if loss.numel() > 0:
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

        if ep % 10 == 0:
            err = info["error_deg"]
            h = info["height"]
            print(f"Ep {ep:3d}: Reward={ep_reward:6.1f}, Error={err:.1f}°, Height={h:.2f}m")

    # Save model
    torch.save(policy.state_dict(), "head_servo_policy.pth")
    print("Saved head_servo_policy.pth")
    return policy


def test(env, policy, episodes=3):
    """Test trained policy"""
    print("\nTesting...")
    for ep in range(episodes):
        obs = env.reset()
        print(f"Target: {np.degrees(obs[2]):.0f}°")

        for step in range(env.max_steps):
            with torch.no_grad():
                action = policy(torch.FloatTensor(obs)).item()
            obs, reward, done, info = env.step(action)
            if done:
                print(f"  Done in {step} steps, error={info['error_deg']:.1f}°")
                break


if __name__ == "__main__":
    env = HeadServoEnv()

    try:
        policy = train(env, episodes=50)
        test(env, policy, episodes=3)

        print("\nDone! Ctrl+C to exit.")
        while simulation_app.is_running():
            env.world.step(render=True)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        simulation_app.close()
