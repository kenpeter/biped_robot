"""Train head servo to track left/right targets (±30°)"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
from pxr import UsdLux
import numpy as np
import os

# Setup world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
UsdLux.DistantLight.Define(world.stage, "/World/Light").CreateIntensityAttr(3000)

# Load robot
usd_path = os.path.join(os.path.dirname(__file__), "head_robot.usda")
add_reference_to_stage(usd_path=usd_path, prim_path="/World/Robot")
robot = world.scene.add(SingleArticulation(prim_path="/World/Robot", name="robot"))
world.reset()

print("\n=== TRAINING HEAD SERVO ===")
print("Target: Oscillate ±30° (left/right)\n")

# Training loop
targets = [-30, 30]  # degrees
current_idx = 0
hold_frames = 120
frame = 0

try:
    while simulation_app.is_running():
        if frame == 0:
            target_deg = targets[current_idx]
            target_rad = np.radians(target_deg)
            robot._articulation_view.set_joint_position_targets(
                positions=np.array([target_rad]),
                joint_indices=np.array([0])
            )
            print(f"→ Target: {target_deg:+3d}°")

        world.step(render=True)
        frame += 1

        if frame >= hold_frames:
            actual_rad = robot.get_joint_positions()[0]
            actual_deg = np.degrees(actual_rad)
            error = abs(actual_deg - targets[current_idx])
            print(f"  Actual: {actual_deg:+6.1f}° (error: {error:.1f}°)\n")

            frame = 0
            current_idx = (current_idx + 1) % len(targets)

except KeyboardInterrupt:
    print("\n✓ Training stopped")

simulation_app.close()
