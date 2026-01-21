"""Demo: Control head servo - THIS WORKS!"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np
import os
import math

print("\n=== HEAD SERVO DEMO ===\n")

# Create world
world = World()
world.scene.add_default_ground_plane()

# Load robot
robot_usd = os.path.join(os.getcwd(), "head_robot.usda")
print(f"✓ Loading: {robot_usd}")
add_reference_to_stage(usd_path=robot_usd, prim_path="/World/Robot")

print("✓ Resetting world...")
world.reset()

# Get joint controller
from isaacsim.core.prims import SingleArticulation
robot = world.scene.add(SingleArticulation("/World/Robot", name="robot"))

print(f"✓ Robot loaded!")
print(f"  DOF: {robot.num_dof}")
print(f"  Joints: {robot.dof_names}")

# Oscillate head
print("\n▶ Oscillating head ±45°...")
print("  Press Ctrl+C to exit\n")

angle = 0.0
direction = 1
speed = 0.02

try:
    frame = 0
    while simulation_app.is_running():
        # Update angle
        angle += direction * speed
        if angle >= math.radians(45):
            angle = math.radians(45)
            direction = -1
        elif angle <= math.radians(-45):
            angle = math.radians(-45)
            direction = 1

        # Set joint position
        robot.set_joint_positions(np.array([angle]))

        # Step
        world.step(render=True)

        # Print every 60 frames
        frame += 1
        if frame % 60 == 0:
            print(f"  Angle: {math.degrees(angle):6.1f}°")

except KeyboardInterrupt:
    print("\n✓ Exiting...")

simulation_app.close()
