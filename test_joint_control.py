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

print(f"Loading robot from: {ROBOT_USD}")
create_prim(
    prim_path="/World/Humanoid",
    prim_type="Xform",
    usd_path=ROBOT_USD,
    position=np.array([0.0, 0.0, 0.2]),
    orientation=np.array([0.7071, 0.7071, 0.0, 0.0])
)

world.reset()

print("\nStage prims:")
for prim in world.stage.Traverse():
    print(f"  {prim.GetPath()}")

print("\nLooking for articulation...")
robot = ArticulationView("/World/Humanoid")
print(f"Found robot with {robot.num_dof} DOFs")

print("\nJoint names:")
for i in range(robot.num_dof):
    try:
        name = robot.dof_names[i]
        print(f"  [{i}] {name}")
    except:
        print(f"  [{i}] (name unknown)")

print("\nInitial joint positions:")
pos = robot.get_joint_positions()
print(f"  {pos}")

print("\nApplying position commands...")
for step in range(100):
    target = np.zeros(robot.num_dof)
    target[0] = 0.5  # Move first joint to 0.5 radians
    
    kp = 100.0
    kd = 10.0
    current = robot.get_joint_positions()
    vel = robot.get_joint_velocities()
    
    effort = kp * (target - current) - kd * vel
    
    robot.set_joint_efforts(effort)
    world.step(render=True)
    
    if step % 20 == 0:
        new_pos = robot.get_joint_positions()
        print(f"  Step {step}: joint[0]={new_pos[0]:.4f}")

print("\nDone.")
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
