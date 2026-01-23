"""Train head servo model in Isaac Sim for continuous rotation servo.
Simulates velocity-controlled rotation and learns timing corrections.
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

# Continuous rotation servo parameters (from real hardware testing)
SECONDS_PER_360 = 6.0
SECONDS_PER_DEGREE = SECONDS_PER_360 / 360.0
SIM_DT = 1.0 / 60.0  # Isaac Sim timestep

print("\n" + "="*50)
print("TRAINING CONTINUOUS ROTATION HEAD SERVO")
print("="*50)
print(f"Timing: {SECONDS_PER_360}s per 360° ({SECONDS_PER_DEGREE*1000:.2f}ms per degree)")


def rotate_by_velocity(target_delta_deg, velocity_deg_per_sec=60.0):
    """Simulate continuous rotation by applying velocity for calculated time.

    Returns actual angle achieved.
    """
    if abs(target_delta_deg) < 0.1:
        return 0.0

    # Calculate expected duration
    duration = abs(target_delta_deg) / velocity_deg_per_sec
    steps = int(duration / SIM_DT)

    # Get starting position
    start_rad = robot.get_joint_positions()[0]

    # Apply velocity (direction based on sign)
    velocity_rad = np.radians(velocity_deg_per_sec) * np.sign(target_delta_deg)

    for _ in range(steps):
        robot._articulation_view.set_joint_velocity_targets(
            velocities=np.array([velocity_rad]),
            joint_indices=np.array([0])
        )
        world.step(render=True)

    # Stop
    robot._articulation_view.set_joint_velocity_targets(
        velocities=np.array([0.0]),
        joint_indices=np.array([0])
    )
    for _ in range(10):
        world.step(render=True)

    # Measure actual movement
    end_rad = robot.get_joint_positions()[0]
    actual_delta_deg = np.degrees(end_rad - start_rad)

    return actual_delta_deg


print("\n[1/3] Collecting training data...")
X_train = []  # Target angle deltas
y_train = []  # Timing corrections needed

# Test various rotation amounts
target_deltas = [-120, -90, -60, -30, -15, 15, 30, 60, 90, 120]

for target_deg in target_deltas:
    # Reset to center first
    robot._articulation_view.set_joint_position_targets(
        positions=np.array([0.0]),
        joint_indices=np.array([0])
    )
    for _ in range(60):
        world.step(render=True)

    # Rotate by velocity
    actual_deg = rotate_by_velocity(target_deg)

    # Calculate timing error
    expected_time = abs(target_deg) * SECONDS_PER_DEGREE
    actual_time = abs(actual_deg) * SECONDS_PER_DEGREE
    timing_error = actual_time - expected_time

    X_train.append([target_deg])
    y_train.append([timing_error])
    print(f"  Target: {target_deg:+4d}° -> Actual: {actual_deg:+7.1f}° (error: {timing_error*1000:+.1f}ms)")

X_train = np.array(X_train)
y_train = np.array(y_train)

print("\n[2/3] Training neural network...")
model = HeadServoModel()
model.seconds_per_degree = SECONDS_PER_DEGREE
model.train(X_train, y_train, epochs=2000, lr=0.01)

print("\n[3/3] Saving model...")
model.save("head_model_weights.json")

print("\nTest predictions (angle -> rotation time):")
for angle in [-90, -45, 0, 45, 90]:
    t = model.predict(angle)
    direction = "left" if t > 0 else "right" if t < 0 else "none"
    print(f"  {angle:+4d}° -> {abs(t):.3f}s {direction}")

simulation_app.close()
print("\n[OK] Saved to head_model_weights.json")
print("[OK] Ready for Jetson deployment")
