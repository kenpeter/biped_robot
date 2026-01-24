"""Train head servo timing calibration using MuJoCo.
Measures actual rotation speed for LEFT and RIGHT separately to capture asymmetry.

Simulates realistic servo behavior:
- Asymmetric left/right speeds (real motors have slight differences)
- Position control with speed limiting (like real servos)
- Measurement noise

Usage: python3 train_head_mujoco.py [--headless] [--fast]
"""
import mujoco
import numpy as np
import os
import argparse
import time

from head_model import HeadServoModel

# Parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('--headless', action='store_true', help='Run without visualization')
parser.add_argument('--fast', action='store_true', help='Run as fast as possible (no real-time sync)')
args = parser.parse_args()

# Load model
model_path = os.path.join(os.path.dirname(__file__), "head_robot.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Setup viewer if not headless
viewer = None
if not args.headless:
    try:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)
    except Exception as e:
        print(f"[WARN] Could not start viewer: {e}")
        print("[INFO] Running in headless mode")

# Simulation parameters
SIM_DT = model.opt.timestep
CONTROL_DT = 0.01  # 100Hz control loop (10ms)
STEPS_PER_CONTROL = int(CONTROL_DT / SIM_DT)

# Realistic servo characteristics
# Real servo: 6 seconds for 360° = 60°/s
# Add asymmetry: real servos have 5-15% speed difference between directions
BASE_SPEED_DEG_S = 60.0
LEFT_SPEED_DEG_S = BASE_SPEED_DEG_S * 0.93   # Left slightly slower
RIGHT_SPEED_DEG_S = BASE_SPEED_DEG_S * 1.07  # Right slightly faster

# Measurement noise
NOISE_DEG = 0.3

# Expected timing
SECONDS_PER_360 = 6.0
SECONDS_PER_DEGREE = SECONDS_PER_360 / 360.0

print("\n" + "="*50)
print("HEAD SERVO TIMING TEST (MuJoCo)")
print("="*50)
print(f"Timestep: {SIM_DT*1000:.1f}ms, Control: {CONTROL_DT*1000:.0f}ms")
print(f"Real-time: {'OFF (fast mode)' if args.fast else 'ON'}")
print(f"Servo speed: L={LEFT_SPEED_DEG_S:.1f}°/s, R={RIGHT_SPEED_DEG_S:.1f}°/s")
print(f"Expected: {SECONDS_PER_360}s per 360° ({SECONDS_PER_DEGREE*1000:.2f}ms per degree)")


def get_angle_deg(with_noise=True):
    """Get current head angle in degrees."""
    angle = np.degrees(data.qpos[0])
    if with_noise:
        angle += np.random.normal(0, NOISE_DEG)
    return angle


def set_target_deg(angle_deg):
    """Set servo target position in degrees."""
    data.ctrl[0] = np.radians(angle_deg)


def step(render=True):
    """Run one control step."""
    for _ in range(STEPS_PER_CONTROL):
        mujoco.mj_step(model, data)
    if viewer is not None and render:
        viewer.sync()
        if not args.fast:
            time.sleep(CONTROL_DT)


def move_to(target_deg, direction_sign):
    """Move to target at realistic servo speed.

    Args:
        target_deg: Target angle in degrees
        direction_sign: -1 for left, +1 for right

    Returns:
        (final_angle, time_taken, error)
    """
    start_angle = get_angle_deg(with_noise=False)
    start_time = data.time

    # Select speed based on direction
    if direction_sign < 0:
        speed = LEFT_SPEED_DEG_S
    else:
        speed = RIGHT_SPEED_DEG_S

    delta = target_deg - start_angle
    expected_time = abs(delta) / speed

    current_target = start_angle

    while True:
        # Move target at servo speed
        step_size = speed * CONTROL_DT * direction_sign
        current_target += step_size

        # Check if we've reached or passed target
        if direction_sign < 0:  # Moving left (negative)
            if current_target <= target_deg:
                current_target = target_deg
        else:  # Moving right (positive)
            if current_target >= target_deg:
                current_target = target_deg

        set_target_deg(current_target)
        step(render=True)

        # Check if done
        actual = get_angle_deg(with_noise=False)
        if abs(actual - target_deg) < 0.5:
            break

        # Timeout
        if data.time - start_time > expected_time * 2:
            break

    # Settle
    set_target_deg(target_deg)
    for _ in range(10):
        step(render=True)

    final = get_angle_deg(with_noise=True)
    time_taken = data.time - start_time
    error = final - target_deg

    return final, time_taken, error


print("\nTesting ±30° movements (tracking left/right separately)...")
print("Sequence: left 30° → center → right 30° → center\n")

# Initialize
mujoco.mj_resetData(model, data)
set_target_deg(0)
for _ in range(50):
    step(render=False)

current_pos = 0.0
left_errors = []
right_errors = []
left_times = []
right_times = []

# Test 5 cycles
for cycle_num in range(5):
    print(f"--- Cycle {cycle_num + 1} ---")

    for target_deg in [-30.0, 0.0, 30.0, 0.0]:
        delta = target_deg - current_pos

        if abs(delta) < 0.5:
            continue

        direction = -1 if delta < 0 else 1
        actual_deg, time_used, angle_error = move_to(target_deg, direction)

        # Track by direction
        if direction < 0:
            left_errors.append(angle_error)
            left_times.append(time_used)
        else:
            right_errors.append(angle_error)
            right_times.append(time_used)

        print(f"  {current_pos:+6.1f}° → {target_deg:+6.1f}° (Δ{delta:+6.1f}°): "
              f"actual={actual_deg:+6.1f}°, err={angle_error:+5.2f}°, t={time_used:.3f}s")

        current_pos = get_angle_deg(with_noise=False)

# Results
avg_left_error = np.mean(left_errors) if left_errors else 0.0
avg_right_error = np.mean(right_errors) if right_errors else 0.0
avg_left_time = np.mean(left_times) if left_times else 0.5
avg_right_time = np.mean(right_times) if right_times else 0.5

print(f"\n--- Results ---")
print(f"Left movements:  avg error = {avg_left_error:+.2f}°, avg time = {avg_left_time:.3f}s")
print(f"Right movements: avg error = {avg_right_error:+.2f}°, avg time = {avg_right_time:.3f}s")

# Calibrated timing
left_spd = avg_left_time / 30.0
right_spd = avg_right_time / 30.0

servo_model = HeadServoModel()
servo_model.seconds_per_degree_left = left_spd
servo_model.seconds_per_degree_right = right_spd

print(f"\nCalibrated timing:")
print(f"  Left:  {servo_model.seconds_per_degree_left*1000:.3f}ms per degree")
print(f"  Right: {servo_model.seconds_per_degree_right*1000:.3f}ms per degree")
print(f"  Asymmetry: {abs(left_spd - right_spd) / ((left_spd + right_spd) / 2) * 100:.1f}%")

output_path = os.path.join(os.path.dirname(__file__), "head_model_weights.json")
servo_model.save(output_path)

if viewer is not None:
    viewer.close()

print(f"\n[OK] Saved to {output_path}")
print("[OK] Ready for Jetson deployment")
