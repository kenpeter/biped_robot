#!/usr/bin/env python3
"""
Simple robot joint test - should show joints moving
"""

import os
import numpy as np
from isaacsim import SimulationApp

print("Starting...")

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core import World

PROJECT_ROOT = "/home/kenpeter/work/biped_robot"
ROBOT_USD = os.path.join(PROJECT_ROOT, "models/humanoid_articulated.usda")

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

print(f"Loading: {ROBOT_USD}")
create_prim(
    prim_path="/World/Humanoid",
    prim_type="Xform",
    usd_path=ROBOT_USD,
    position=np.array([0.0, 0.0, 0.15]),
    orientation=np.array([0.7071, 0.7071, 0.0, 0.0])
)

world.reset()

robot = ArticulationView("/World/Humanoid")
print(f"Robot loaded: {robot.num_dof} DOFs")

print("Moving joints...")
for step in range(50):
    target = np.zeros(robot.num_dof)
    target[0] = 0.5  # head
    target[1] = 0.3  # l_shoulder_pitch
    target[4] = -0.3  # r_shoulder_pitch
    
    robot.set_joint_positions(target)
    world.step(render=True)
    
    if step % 10 == 0:
        pos = robot.get_joint_positions()
        print(f"Step {step}: pos[0]={pos[0,0]:.3f}, pos[1]={pos[0,1]:.3f}")

print("Done. Window stays open.")
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
