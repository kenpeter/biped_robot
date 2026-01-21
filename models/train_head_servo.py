"""
Simple Head Servo Training - works!
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import random
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from pxr import UsdPhysics
import carb

print("=== HEAD SERVO TRAINING ===\n")

# Create world
world = World()
world.scene.add_default_ground_plane()

# Load robot
from omni.isaac.core.utils.stage import add_reference_to_stage
import os

robot_usd = os.path.join(os.getcwd(), "head_robot.usda")
print(f"✓ Loading: {robot_usd}")

add_reference_to_stage(usd_path=robot_usd, prim_path="/World/Robot")
world.reset()

# Get robot articulation
from isaacsim.core.prims import SingleArticulation

robot = world.scene.add(SingleArticulation("/World/Robot", name="robot"))

print(f"✓ Robot loaded: {robot.num_dof} DOF")
print(f"  Joint names: {robot.dof_names}")

# Simple policy network
class Policy(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(3, 32), nn.ReLU(),
            nn.Linear(32, 32), nn.ReLU(),
            nn.Linear(32, 1), nn.Tanh())

    def forward(self, x):
        return self.net(x)

policy = Policy()
optimizer = optim.Adam(policy.parameters(), lr=0.01)

# Training loop
print("\n▶ Training...")
episodes = 20
max_steps = 100

for ep in range(episodes):
    # Reset robot
    robot.set_joint_positions(np.zeros(robot.num_dof))
    robot.set_joint_velocities(np.zeros(robot.num_dof))

    # Random target angle (-30° to +30°)
    target = random.uniform(-0.5, 0.5)

    ep_reward = 0

    for step in range(max_steps):
        # Get observation
        pos = robot.get_joint_positions()[0]
        vel = robot.get_joint_velocities()[0]
        obs = torch.FloatTensor([pos, vel, target])

        # Get action from policy
        if random.random() < max(0.1, 1.0 - ep/10):
            action = random.uniform(-1, 1)  # Explore early
        else:
            action = policy(obs).item()  # Exploit later

        # Apply torque
        efforts = np.array([action * 10.0])
        robot.set_joint_efforts(efforts)

        # Step simulation
        world.step(render=True)

        # Calculate reward
        error = abs(pos - target)
        reward = -error * 100.0
        if error < 0.05:
            reward += 10.0

        ep_reward += reward

        # Early exit if close
        if error < 0.02:
            break

    # Print progress
    if ep % 5 == 0:
        print(f"  Ep {ep:2d}: Reward={ep_reward:7.1f}")

print(f"\n✓ Training complete!")
print(f"  Run with: ./run_isaac.sh train_head_servo.py")
print("\nPress Ctrl+C to exit...")

# Keep window open
try:
    while simulation_app.is_running():
        world.step(render=True)
except KeyboardInterrupt:
    pass

simulation_app.close()
