#!/usr/bin/env python3
"""
Debug robot joint control - test if joints respond to commands
"""

import os
import numpy as np
from isaacsim import SimulationApp

print("=" * 60)
print("ROBOT JOINT CONTROL TEST")
print("=" * 60)

simulation_app = SimulationApp({"headless": False})

import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core import World

PROJECT_ROOT = "/home/kenpeter/work/biped_robot"
ROBOT_USD = os.path.join(PROJECT_ROOT, "models/humanoid_articulated.usda")

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

create_prim(
    prim_path="/World/Humanoid",
    prim_type="Xform",
    usd_path=ROBOT_USD,
    position=np.array([0.0, 0.0, 0.2]),
    orientation=np.array([0.7071, 0.7071, 0.0, 0.0])
)

world.reset()

robot = ArticulationView("/World/Humanoid")
print(f"Found robot with {robot.num_dof} DOFs")

print("\nJoint names:")
for i in range(min(robot.num_dof, 5)):  # Show first 5
    try:
        name = robot.dof_names[i]
        print(f"  [{i}] {name}")
    except:
        print(f"  [{i}] (name unknown)")

print("\nInitial joint positions:")
pos = robot.get_joint_positions()
print(f"  Shape: {pos.shape}")
print(f"  Values: {pos.flatten()[:5]}...")  # First 5 values

print("\nSetting joint positions to 0.5 radians for first 3 joints...")
for i in range(3):
    target = np.zeros(robot.num_dof)
    target[i] = 0.5
    robot.set_joint_positions(target)
    world.step(render=True)

print("After setting positions:")
new_pos = robot.get_joint_positions()
print(f"  Values: {new_pos.flatten()[:5]}...")

print("\nDirect position setting test passed!")

print("\nTest complete - window will stay open.")
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
