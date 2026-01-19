"""
Very simple Isaac Sim test with ArticulationController
"""

import os
import numpy as np
from isaacsim import SimulationApp

print("="*80)
print("SIMPLE ROBOT MOVEMENT TEST")
print("="*80)

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.controllers import ArticulationController
import omni.isaac.core.utils.stage as stage_utils

# Paths
PROJECT_ROOT = os.getcwd()
ROBOT_USD = os.path.join(PROJECT_ROOT, "models/humanoid_articulated.usda")

# Create world
print("\nCreating world...")
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Add robot
print("Loading robot...")
robot_prim_path = "/World/Humanoid"
create_prim(
    prim_path=robot_prim_path,
    prim_type="Xform",
    usd_path=ROBOT_USD,
    position=np.array([0.0, 0.0, 0.45]),
    orientation=np.array([0.7071, 0.7071, 0.0, 0.0])
)

# Initialize
print("Initializing...")
world.reset()

# Import articulation
from omni.isaac.core.articulations import Articulation
robot = world.scene.add(Articulation(prim_path=robot_prim_path, name="robot"))
print(f"Robot has {robot.num_dof} DOF")

# Create controller
print("Creating articulation controller...")
controller = ArticulationController(name="controller", articulation=robot)

print("\n" + "="*80)
print("DEMO: Slowly moving ONE joint back and forth")
print("Watch joint 0 (head) move slowly from -90° to +90°")
print("="*80 + "\n")

# VERY SIMPLE TEST: Move only joint 0 slowly
try:
    for step in range(600):  # 10 seconds
        # Simple sine wave for ONLY joint 0
        time = step / 60.0
        joint0_target = np.radians(90) * np.sin(2 * np.pi * 0.1 * time)  # 0.1 Hz, ±90°

        # Create position targets (all zeros except joint 0)
        position_targets = np.zeros(robot.num_dof)
        position_targets[0] = joint0_target

        # Apply position command
        robot.set_joint_positions(position_targets)

        # Step simulation
        world.step(render=True)

        if step % 30 == 0:
            current = robot.get_joint_positions()
            print(f"Step {step:3d}: Target j0={np.degrees(joint0_target):+6.1f}° | Actual j0={np.degrees(current[0]):+6.1f}°")

    print("\n" + "="*80)
    print("DEMO COMPLETE - Robot should have moved joint 0")
    print("Close the window or press Ctrl+C")
    print("="*80)

    # Keep window open
    while simulation_app.is_running():
        world.step(render=True)

except KeyboardInterrupt:
    print("\nShutting down...")
finally:
    simulation_app.close()
    print("Closed.")
