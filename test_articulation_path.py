#!/usr/bin/env python3
"""
Simple robot loading test - fix the articulation path issue
"""

import os
import numpy as np
from isaacsim import SimulationApp

print("=" * 60)
print("SIMPLE ROBOT LOAD TEST")
print("=" * 60)

simulation_app = SimulationApp({"headless": False})

import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core import World

PROJECT_ROOT = "/home/kenpeter/work/biped_robot"
ROBOT_USD = os.path.join(PROJECT_ROOT, "models/humanoid_articulated.usda")

print(f"Robot USD: {ROBOT_USD}")
print(f"USD exists: {os.path.exists(ROBOT_USD)}")

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

print("Loading robot from USD...")
create_prim(
    prim_path="/World/Humanoid",
    prim_type="Xform",
    usd_path=ROBOT_USD,
    position=np.array([0.0, 0.0, 0.2]),
    orientation=np.array([0.7071, 0.7071, 0.0, 0.0])
)

world.reset()

print("Stage contents after reset:")
for prim in world.stage.Traverse():
    print(f"  {prim.GetPath()}")

print("\nLooking for articulations...")
try:
    robot = ArticulationView("/World/Humanoid")
    print(f"SUCCESS: Found articulation with {robot.num_dof} DOFs")
except Exception as e:
    print(f"ERROR: {e}")

    print("\nTrying alternate paths...")
    for path in ["/Humanoid", "/Humanoid/base_link"]:
        try:
            robot = ArticulationView(path)
            print(f"  SUCCESS at {path}: {robot.num_dof} DOFs")
        except Exception as e2:
            print(f"  FAILED at {path}: {e2}")

print("\nKeeping window open. Press Ctrl+C to exit.")
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
print("Done.")
