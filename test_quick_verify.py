#!/usr/bin/env python3
"""
Quick test to verify robot structure
"""

import os
import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

robot_usd_path = "/home/kenpeter/work/biped_robot/models/humanoid_articulated.usda"
print(f"Loading: {robot_usd_path}")

prim_path = "/World/Robot"
add_reference_to_stage(usd_path=robot_usd_path, prim_path=prim_path)

robot = world.scene.add(
    Articulation(
        prim_path=prim_path,
        name="humanoid"
    )
)

print("Resetting world...")
world.reset()

print(f"\n{'='*60}")
print(f"ROBOT STATUS:")
print(f"{'='*60}")
print(f"  num_dof: {robot.num_dof}")
print(f"  dof_names: {robot.dof_names}")
print(f"{'='*60}")

print("\nWave motion starting... Watch the robot in Isaac Sim window!")

step = 0
try:
    while simulation_app.is_running() and step < 300:
        time = step / 60.0
        wave = np.radians(30) * np.sin(2 * np.pi * 0.5 * time)
        positions = np.ones(robot.num_dof) * wave
        robot.set_joint_positions(positions)
        world.step(render=True)
        step += 1
except Exception as e:
    print(f"Error: {e}")
finally:
    simulation_app.close()
