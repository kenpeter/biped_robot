"""Test: Verify base never moves during head rotation"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
import numpy as np
import os

# Create world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Add lighting
from pxr import UsdLux

stage = world.stage
distant_light = UsdLux.DistantLight.Define(stage, "/World/defaultLight")
distant_light.CreateIntensityAttr(3000)

# Load robot
usd_path = os.path.join(os.path.dirname(__file__), "head_robot.usda")
add_reference_to_stage(usd_path=usd_path, prim_path="/World/Robot")

# Get articulation
robot = world.scene.add(SingleArticulation(prim_path="/World/Robot", name="robot"))
world.reset()

# Get initial base position
base_prim = world.stage.GetPrimAtPath("/World/Robot/base")
initial_base_pos = np.array(base_prim.GetAttribute("xformOp:translate").Get())

print(f"\n✓ Initial base position: {initial_base_pos}")
print(f"✓ Target: Base should NEVER move from {initial_base_pos}\n")

# Rotate head 360° slowly
num_steps = 360
angle_per_step = 1.0  # 1 degree per step
max_base_error = 0.0

for step in range(num_steps):
    target_angle = np.radians(step * angle_per_step)
    robot._articulation_view.set_joint_position_targets(
        positions=np.array([target_angle]),
        joint_indices=np.array([0])
    )
    world.step(render=True)

    # Check base position every 90 degrees
    if step % 90 == 0:
        current_base_pos = np.array(base_prim.GetAttribute("xformOp:translate").Get())
        error = np.linalg.norm(current_base_pos - initial_base_pos)
        max_base_error = max(max_base_error, error)

        print(f"Head: {step}° | Base error: {error:.6f}m | {'✓ FIXED' if error < 1e-6 else '✗ MOVED!'}")

print(f"\n{'✓ SUCCESS' if max_base_error < 1e-6 else '✗ FAILED'}: Max base movement = {max_base_error:.6f}m")
print("\n▶ Continuous rotation - Press Ctrl+C to exit\n")

# Keep rotating continuously
step = 0
try:
    while simulation_app.is_running():
        target_angle = np.radians(step)
        robot._articulation_view.set_joint_position_targets(
            positions=np.array([target_angle]),
            joint_indices=np.array([0])
        )
        world.step(render=True)
        step = (step + 1) % 360
except KeyboardInterrupt:
    print("\n✓ Exiting...")

simulation_app.close()
