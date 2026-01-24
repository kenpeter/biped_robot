"""Test head servo in Isaac Sim to verify timing calibration.
Measures actual rotation speed for LEFT and RIGHT separately to capture asymmetry.
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
from pxr import UsdLux
import numpy as np
import os

from head_model import HeadServoModel

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
UsdLux.DistantLight.Define(world.stage, "/World/Light").CreateIntensityAttr(3000)

usd_path = os.path.join(os.path.dirname(__file__), "head_robot.usda")
add_reference_to_stage(usd_path=usd_path, prim_path="/World/Robot")
robot = world.scene.add(SingleArticulation(prim_path="/World/Robot", name="robot"))
world.reset()

# Default calibration (from real hardware testing)
SECONDS_PER_360 = 6.0
SECONDS_PER_DEGREE = SECONDS_PER_360 / 360.0
SIM_DT = 1.0 / 60.0  # Isaac Sim timestep

print("\n" + "="*50)
print("HEAD SERVO TIMING TEST (Asymmetric)")
print("="*50)
print(f"Default: {SECONDS_PER_360}s per 360° ({SECONDS_PER_DEGREE*1000:.2f}ms per degree)")


def rotate_by_time(current_deg, target_deg):
    """Rotate and measure actual result. Returns (actual_deg, time_used, angle_error)."""
    delta_deg = target_deg - current_deg
    if abs(delta_deg) < 0.5:
        return current_deg, 0.0, 0.0

    VELOCITY_DEG_PER_SEC = 60.0
    time_needed = abs(delta_deg) / VELOCITY_DEG_PER_SEC
    velocity_rad = np.radians(VELOCITY_DEG_PER_SEC) * np.sign(delta_deg)

    max_steps = int((time_needed + 0.5) / SIM_DT)
    for i in range(max_steps):
        current_rad = robot.get_joint_positions()[0]
        current_deg_now = np.degrees(current_rad)
        error_deg = target_deg - current_deg_now

        if abs(error_deg) < 1.0:
            break

        if abs(error_deg) < 5.0:
            scale = abs(error_deg) / 5.0
            current_velocity = velocity_rad * max(scale, 0.2)
        else:
            current_velocity = velocity_rad

        robot._articulation_view.set_joint_velocity_targets(
            velocities=np.array([current_velocity]),
            joint_indices=np.array([0])
        )
        world.step(render=True)

    robot._articulation_view.set_joint_velocity_targets(
        velocities=np.array([0.0]),
        joint_indices=np.array([0])
    )
    for _ in range(20):
        world.step(render=True)

    actual_deg = np.degrees(robot.get_joint_positions()[0])
    angle_error = actual_deg - target_deg
    return actual_deg, time_needed, angle_error


print("\nTesting ±30° movements (tracking left/right separately)...")
print("Sequence: left 30° → center → right 30° → center\n")

current_pos = 0.0
left_errors = []
right_errors = []

# Test 5 cycles
for cycle_num in range(5):
    print(f"--- Cycle {cycle_num + 1} ---")

    for target_deg in [-30.0, 0.0, 30.0, 0.0]:
        actual_deg, time_used, angle_error = rotate_by_time(current_pos, target_deg)

        delta = target_deg - current_pos

        # Track errors by direction
        if delta > 0:
            left_errors.append(angle_error)
        elif delta < 0:
            right_errors.append(angle_error)

        print(f"  {current_pos:+6.1f}° → {target_deg:+6.1f}° (Δ{delta:+6.1f}°): "
              f"actual={actual_deg:+6.1f}°, error={angle_error:+5.2f}°")

        current_pos = actual_deg

        for _ in range(30):
            world.step(render=True)

# Calculate asymmetric timing adjustments
avg_left_error = np.mean(left_errors) if left_errors else 0.0
avg_right_error = np.mean(right_errors) if right_errors else 0.0

print(f"\n--- Results ---")
print(f"Left movements:  avg error = {avg_left_error:+.2f}°")
print(f"Right movements: avg error = {avg_right_error:+.2f}°")

# Adjust timing based on errors
# If we overshoot (positive error), we need less time (reduce timing)
# If we undershoot (negative error), we need more time (increase timing)
left_adjustment = 1.0 - (avg_left_error / 30.0)  # error per 30° movement
right_adjustment = 1.0 - (avg_right_error / 30.0)

model = HeadServoModel()
model.seconds_per_degree_left = SECONDS_PER_DEGREE * left_adjustment
model.seconds_per_degree_right = SECONDS_PER_DEGREE * right_adjustment

print(f"\nCalibrated timing:")
print(f"  Left:  {model.seconds_per_degree_left*1000:.3f}ms per degree (adj: {left_adjustment:.3f})")
print(f"  Right: {model.seconds_per_degree_right*1000:.3f}ms per degree (adj: {right_adjustment:.3f})")

model.save("head_model_weights.json")

simulation_app.close()
print("\n[OK] Saved to head_model_weights.json")
print("[OK] Ready for Jetson deployment")
