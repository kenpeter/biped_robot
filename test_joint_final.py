#!/usr/bin/env python3
"""
Test basic joint control with humanoid robot
"""

import os
import numpy as np
from isaacsim import SimulationApp

print("=" * 60)
print("HUMANOID JOINT CONTROL TEST")
print("=" * 60)

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core import World

PROJECT_ROOT = "/home/kenpeter/work/biped_robot"
ROBOT_USD = os.path.join(PROJECT_ROOT, "models/humanoid_articulated.usda")

print(f"\nLoading robot from: {ROBOT_USD}")

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

create_prim(
    prim_path="/World/Humanoid",
    prim_type="Xform",
    usd_path=ROBOT_USD,
    position=np.array([0.0, 0.0, 0.15]),
    orientation=np.array([0.7071, 0.7071, 0.0, 0.0])
)

print("Resetting world...")
world.reset()

print("Looking for articulation...")
robot = ArticulationView("/World/Humanoid")

# Wait for initialization
import time
timeout = 10
start = time.time()
while robot.num_dof is None and (time.time() - start) < timeout:
    print(f"  Waiting... num_dof = {robot.num_dof}")
    world.step(render=True)
    time.sleep(0.1)

print(f"Found: {robot.num_dof} DOFs")

if robot.num_dof and robot.num_dof > 0:
    print("\nJoint names:")
    for i in range(min(robot.num_dof, 5)):
        name = robot.dof_names[i] if hasattr(robot, 'dof_names') and robot.dof_names is not None else f"joint_{i}"
        print(f"  [{i}] {name}")
    
    print("\nInitial position:", robot.get_joint_positions())
    
    print("\nSetting joint positions to move robot...")
    for step in range(50):
        target = np.zeros((1, robot.num_dof))
        target[0, 0] = 0.5  # Head joint
        target[0, 1] = 0.3  # Left shoulder pitch
        target[0, 4] = -0.3  # Right shoulder pitch
        
        robot.set_joint_positions(target)
        world.step(render=True)
        
        if step % 10 == 0:
            pos = robot.get_joint_positions()
            print(f"  Step {step}: head={pos[0,0]:.3f}, l_shoulder={pos[0,1]:.3f}, r_shoulder={pos[0,4]:.3f}")
    
    print("\nJoint control test complete!")
else:
    print("ERROR: Could not find articulation DOFs!")

print("\nWindow stays open. Press Ctrl+C to exit.")
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
