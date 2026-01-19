"""
Simple Isaac Sim test to demonstrate robot movement
Shows obvious joint movement for verification
"""

import os
import sys
import numpy as np
from isaacsim import SimulationApp

print("=" * 80)
print("ROBOT MOVEMENT TEST - OBVIOUS DEMONSTRATION")
print("=" * 80)

# Launch Isaac Sim
print("\nLaunching Isaac Sim...")
simulation_app = SimulationApp({"headless": False})

# Update simulation
for _ in range(10):
    simulation_app.update()

print("Isaac Sim window opened!")

import omni
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import create_prim
import numpy as np

# Paths
PROJECT_ROOT = os.getcwd()
ROBOT_USD = os.path.join(PROJECT_ROOT, "models/humanoid_articulated.usda")

# Update simulation
for _ in range(10):
    simulation_app.update()

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
    position=np.array([0.0, 0.0, 0.3]),  # Spawn higher so we can see it
    orientation=np.array([0.7071, 0.7071, 0.0, 0.0])  # +90° X rotation
)

# Initialize world
print("Initializing world...")
world.reset()

# Add articulation
print(f"Adding articulation...")
robot = world.scene.add(
    Articulation(
        prim_path=robot_prim_path,
        name="humanoid_robot"
    )
)

print(f"Robot has {robot.num_dof} degrees of freedom")
print("\n" + "=" * 80)
print("STARTING MOVEMENT DEMO")
print("Watch the robot in the Isaac Sim window!")
print("All joints will wave back and forth together")
print("=" * 80)

# Reset to neutral
robot.set_joint_positions(np.zeros(robot.num_dof))
robot.set_joint_velocities(np.zeros(robot.num_dof))
world.reset()

# Movement demo - all joints wave together
amplitude = np.radians(90)  # 90 degrees wave amplitude (VERY OBVIOUS!)
frequency = 0.25  # Hz (slower for visibility)

print("\n🤖 DEMO: All joints waving back and forth CONTINUOUSLY...")
print("This will run forever until you press Ctrl+C")
print("Watch the Isaac Sim window - all joints should wave together!")
print("Using USD joint drives (stiffness=500, damping=20)")
print("=" * 80)

# Run continuously
step = 0
try:
    while simulation_app.is_running():
        # Get current state
        current_pos = robot.get_joint_positions()

        # Create sinusoidal target for all joints
        time = step / 60.0  # Convert to seconds (60 Hz sim)
        wave = amplitude * np.sin(2 * np.pi * frequency * time)

        # Make all joints wave together
        target_pos = np.ones(robot.num_dof) * wave

        # Use position targets (let USD drive do PD control)
        robot.set_joint_position_targets(target_pos)
        world.step(render=True)

        if step % 15 == 0:  # Print more frequently
            # Show target vs actual to verify movement
            error = np.degrees(current_pos[0] - wave)
            print(f"Step {step:4d}: Target={np.degrees(wave):+6.1f}° | Actual={np.degrees(current_pos[0]):+6.1f}° | Error={error:+6.1f}°")

        step += 1

except KeyboardInterrupt:
    print("\n" + "=" * 80)
    print("DEMO STOPPED!")
    print("=" * 80)
finally:
    simulation_app.close()
    print("Closed.")
