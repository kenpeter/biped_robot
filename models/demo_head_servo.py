"""Demo: Head servo oscillation"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
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

# Get robot
robot = world.scene.add(SingleArticulation("/World/Robot", name="robot"))

print(f"✓ Robot loaded!")
print(f"  DOF: {robot.num_dof}")
print(f"  Joints: {robot.dof_names}")

# Oscillate using POSITION control with step pattern
print("\n▶ Starting oscillation...")
print("  Pattern: Center → Left 20° → Center → Right 20° → Center")
print("  Watch the viewport!\n")

# Step pattern with direct commanded angles (Fixed mass ratio issue)
test_angles = [0, -20, 0, 20, 0]  # degrees
test_names = ["CENTER", "LEFT 20°", "CENTER", "RIGHT 20°", "CENTER"]

try:
    for cmd_deg, name in zip(test_angles, test_names):
        cmd_rad = math.radians(cmd_deg)

        print(f"\n→ {name} (commanding {cmd_deg:+3d}°)")
        robot._articulation_view.set_joint_position_targets(
            positions=np.array([cmd_rad]),
            joint_indices=np.array([0])
        )

        # Hold for 3 seconds
        for frame in range(180):
            world.step(render=True)

            if frame % 60 == 0:
                actual_rad = robot.get_joint_positions()[0]
                actual_deg = math.degrees(actual_rad)
                print(f"   Frame {frame:3d}: Actual = {actual_deg:+6.1f}°")

    print("\n▶ Looping oscillation - Press Ctrl+C to exit\n")

    # Continuous oscillation
    idx = 0
    frame_count = 0
    hold_frames = 120  # 2 seconds per position

    while simulation_app.is_running():
        if frame_count == 0:
            cmd_rad = math.radians(test_angles[idx])
            robot._articulation_view.set_joint_position_targets(
            positions=np.array([cmd_rad]),
            joint_indices=np.array([0])
        )

        world.step(render=True)
        frame_count += 1

        if frame_count >= hold_frames:
            frame_count = 0
            idx = (idx + 1) % len(test_angles)

except KeyboardInterrupt:
    print("\n✓ Exiting...")

simulation_app.close()
