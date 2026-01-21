"""Test: Verify head servo actually moves"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
import numpy as np
import os
import math

print("\n=== HEAD MOVEMENT TEST ===\n")

# Create world
world = World()
world.scene.add_default_ground_plane()

# Load robot
robot_usd = os.path.join(os.getcwd(), "head_robot.usda")
print(f"Loading: {robot_usd}")
add_reference_to_stage(usd_path=robot_usd, prim_path="/World/Robot")

world.reset()

# Get robot
robot = world.scene.add(SingleArticulation("/World/Robot", name="robot"))

print(f"\n✓ Robot Info:")
print(f"  DOF: {robot.num_dof}")
print(f"  Joint names: {robot.dof_names}")

# Test movement
print(f"\n=== MOVEMENT TEST ===\n")

test_angles = [0, 0.5, -0.5, 0]  # radians: 0°, 28°, -28°, 0°
wait_frames = 120  # 2 seconds per position

for i, target_angle in enumerate(test_angles):
    print(f"\n{i+1}. Setting angle to {math.degrees(target_angle):6.1f}°")

    # Set target position
    robot.set_joint_positions(np.array([target_angle]))

    # Wait and check actual position
    for frame in range(wait_frames):
        world.step(render=True)

        # Print position every 30 frames
        if frame % 30 == 0:
            actual_pos = robot.get_joint_positions()[0]
            actual_vel = robot.get_joint_velocities()[0]
            print(f"   Frame {frame:3d}: actual={math.degrees(actual_pos):6.1f}°, vel={actual_vel:6.3f} rad/s")

print(f"\n✓ Test complete!")
print(f"  If angles changed: Joint is WORKING ✓")
print(f"  If angles stayed 0: Joint is BROKEN ✗")

simulation_app.close()
