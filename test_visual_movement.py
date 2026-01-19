"""
DEFINITIVE movement test - sets positions directly (no physics)
This WILL show movement if rendering works at all
"""

import os
import numpy as np
from isaacsim import SimulationApp

print("="*80)
print("VISUAL MOVEMENT TEST - NO PHYSICS")
print("This test sets joint positions directly")
print("If you can't see movement with this test, there's a display issue")
print("="*80)

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.articulations import Articulation

# Paths
PROJECT_ROOT = os.getcwd()
ROBOT_USD = os.path.join(PROJECT_ROOT, "models/humanoid_articulated.usda")

# Create world WITHOUT ground plane (keep it simple)
print("\nCreating world...")
world = World(stage_units_in_meters=1.0)

# Add robot
print("Loading robot...")
robot_prim_path = "/World/Humanoid"
create_prim(
    prim_path=robot_prim_path,
    prim_type="Xform",
    usd_path=ROBOT_USD,
    position=np.array([0.0, 0.0, 0.5]),
    orientation=np.array([0.7071, 0.7071, 0.0, 0.0])
)

# Initialize
print("Initializing...")
world.reset()

# Add articulation
robot = world.scene.add(Articulation(prim_path=robot_prim_path, name="robot"))
print(f"Robot has {robot.num_dof} DOF\n")

print("="*80)
print("WAVING ALL JOINTS TOGETHER - DIRECT POSITION SETTING")
print("Frequency: 0.2 Hz (5 second period)")
print("Amplitude: ±60 degrees")
print("This will be VERY OBVIOUS if rendering works!")
print("="*80 + "\n")

# Wave all joints
step = 0
try:
    while simulation_app.is_running():
        # Calculate wave
        time = step / 60.0
        wave_radians = np.radians(60) * np.sin(2 * np.pi * 0.2 * time)

        # Set ALL joints to same wave position
        joint_positions = np.ones(robot.num_dof) * wave_radians

        # DIRECTLY SET (bypasses physics entirely)
        robot.set_joint_positions(joint_positions)

        # Render
        world.step(render=True)

        # Print every half second
        if step % 30 == 0:
            wave_degrees = np.degrees(wave_radians)
            actual = robot.get_joint_positions()[0]
            print(f"Step {step:4d} | Target: {wave_degrees:+7.1f}° | Actual j[0]: {np.degrees(actual):+7.1f}° | {'█' * int(abs(wave_degrees)/3)}")

        step += 1

except KeyboardInterrupt:
    print("\n" + "="*80)
    print("STOPPED")
    print("="*80)
finally:
    simulation_app.close()
    print("Closed.")
