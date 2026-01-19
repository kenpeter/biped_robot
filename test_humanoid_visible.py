"""
Test humanoid robot with embedded visible geometry
Should see COLORED CUBES representing robot parts
"""

import os
import numpy as np
from isaacsim import SimulationApp

print("="*80)
print("HUMANOID ROBOT VISIBILITY TEST")
print("You should see colored cubes: RED torso, YELLOW head, GREEN/BLUE limbs")
print("="*80)

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation

# Create world
print("\nCreating world...")
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Load robot USD
robot_usd_path = os.path.join(os.getcwd(), "models/humanoid_articulated.usda")
print(f"Loading robot from: {robot_usd_path}")

# Add robot to scene
robot = world.scene.add(
    Articulation(
        prim_path="/Humanoid",
        name="humanoid",
        usd_path=robot_usd_path
    )
)

print("Initializing simulation...")
world.reset()

print(f"\nRobot loaded!")
print(f"  DOF: {robot.num_dof}")
print(f"  Joint names: {robot.dof_names}")

print("\n" + "="*80)
print("WAVING ALL JOINTS - WATCH FOR COLORED CUBES MOVING")
print("="*80 + "\n")

# Wave all joints together
step = 0
try:
    while simulation_app.is_running() and step < 300:  # Run for 5 seconds
        # Create wave motion
        time = step / 60.0
        wave = np.radians(30) * np.sin(2 * np.pi * 0.5 * time)

        # Set all joints to wave
        positions = np.ones(robot.num_dof) * wave
        robot.set_joint_positions(positions)

        world.step(render=True)

        if step % 30 == 0:
            print(f"Step {step:3d} | Wave: {np.degrees(wave):+6.1f}°")

        step += 1

except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    simulation_app.close()
    print("Test complete!")
