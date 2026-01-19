#!/usr/bin/env python3
"""Quick robot test"""

import os
import numpy as np
from isaacsim import SimulationApp

print("Starting Isaac Sim (headless)...")
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import create_prim

ROBOT_USD = os.path.join(os.getcwd(), "models/humanoid_articulated.usda")
print(f"Loading robot from: {ROBOT_USD}")

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

print(f"Robot DOFs: {robot.num_dof}")
print("Initial joint positions:", robot.get_joint_positions()[0])

# Apply torque and check movement
print("\nApplying torque...")
for i in range(10):
    actions = np.zeros(robot.num_dof)
    actions[0] = 5.0
    robot.set_joint_efforts(actions)
    world.step(render=False)
    
    vel = robot.get_joint_velocities()
    pos = robot.get_joint_positions()
    print(f"Step {i}: Joint 0 vel={vel[0,0]:.4f}, pos={pos[0,0]:.4f}")

print("\nFinal joint positions:", robot.get_joint_positions()[0])
print("Test complete!")

simulation_app.close()
