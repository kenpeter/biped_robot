"""Visual Test: Watch head rotate in Isaac Sim"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
import numpy as np
import os
import math
import time

print("\n" + "="*60)
print("  VISUAL ROTATION TEST - WATCH THE HEAD ROTATE!")
print("="*60)
print("\n✓ Opening Isaac Sim viewport...")
print("✓ Look for the blue cube (head) with black camera")
print("✓ It should rotate left and right slowly\n")

# Create world
world = World()
world.scene.add_default_ground_plane()

# Load robot
robot_usd = os.path.join(os.getcwd(), "head_robot.usda")
add_reference_to_stage(usd_path=robot_usd, prim_path="/World/Robot")
world.reset()

# Get robot
robot = world.scene.add(SingleArticulation("/World/Robot", name="robot"))

print(f"Robot has {robot.num_dof} joint: {robot.dof_names[0]}")
print("\n" + "="*60)
print("  STARTING ROTATION TEST")
print("="*60)

# Test positions with pauses
test_positions = [
    (0, "CENTER (0°)"),
    (0.5, "RIGHT (+28°)"),
    (0, "CENTER (0°)"),
    (-0.5, "LEFT (-28°)"),
    (0, "CENTER (0°)")
]

for angle_rad, description in test_positions:
    angle_deg = math.degrees(angle_rad)

    print(f"\n→ Moving to: {description}")
    print(f"  Target: {angle_deg:+6.1f}°")

    # Set target
    robot.set_joint_positions(np.array([angle_rad]))

    # Hold position for 3 seconds
    for i in range(180):  # 3 seconds at 60fps
        world.step(render=True)

        # Print actual position every second
        if i % 60 == 0:
            actual = robot.get_joint_positions()[0]
            print(f"  Actual: {math.degrees(actual):+6.1f}° (frame {i})")

print("\n" + "="*60)
print("  TEST COMPLETE!")
print("="*60)
print("\n✅ If you saw the head rotate → WORKING!")
print("❌ If head didn't move → Check USD file")
print("\nPress Ctrl+C to exit...")

try:
    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.1)
except KeyboardInterrupt:
    pass

simulation_app.close()
