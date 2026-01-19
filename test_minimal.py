#!/usr/bin/env python3
"""
Minimal Robot Test
Just load robot and apply simple torque
"""

import os
import numpy as np
from isaacsim import SimulationApp

print("=" * 60)
print("MINIMAL ROBOT TEST")
print("=" * 60)

# Launch Isaac Sim in headless mode
print("Launching Isaac Sim (headless)...")
simulation_app = SimulationApp({"headless": True})

import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core import World

# Paths
PROJECT_ROOT = os.getcwd()
ROBOT_USD = os.path.join(PROJECT_ROOT, "models/humanoid_articulated.usda")

print(f"Robot USD path: {ROBOT_USD}")

# Create world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Load robot
print("Loading robot...")
create_prim(
    prim_path="/World/Humanoid",
    prim_type="Xform",
    usd_path=ROBOT_USD,
    position=np.array([0.0, 0.0, 0.2]),
    orientation=np.array([0.7071, 0.7071, 0.0, 0.0])
)

# Initialize world
world.reset()

# Get articulation view
robot = ArticulationView("/World/Humanoid")
print(f"Robot DOFs: {robot.num_dof}")

# Test simple torque control
print("Testing torque control...")
for i in range(50):
    # Apply small torque to first joint
    actions = np.zeros(robot.num_dof)
    actions[0] = 2.0  # 2 N⋅m to first joint
    
    robot.set_joint_efforts(actions)
    world.step(render=False)
    
    # Get velocity
    velocities = robot.get_joint_velocities()
    if i % 10 == 0:
        print(f"Step {i}: Joint 0 velocity = {velocities[0,0]:.4f}")

print("Test complete!")
simulation_app.close()