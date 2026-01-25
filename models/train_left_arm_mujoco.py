"""Train left arm servo timing using MuJoCo.

Servos:
  12 - Shoulder (forward/backward)
  13 - Upper arm (inward/outward)
  14 - Forearm (inward/outward)

Usage: python3 train_arm_mujoco.py [--headless] [--fast]
"""
import mujoco
import numpy as np
import os
import argparse
import time

from left_arm_model import ArmServoModel

# Parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('--headless', action='store_true', help='Run without visualization')
parser.add_argument('--fast', action='store_true', help='Run as fast as possible (no real-time sync)')
args = parser.parse_args()

# Load model
model_path = os.path.join(os.path.dirname(__file__), "left_arm_robot.xml")
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
CONTROL_DT = 0.01  # 100Hz control loop
STEPS_PER_CONTROL = int(CONTROL_DT / SIM_DT)

# Servo mapping: index in ctrl array
SERVO_MAP = {
    12: 0,  # shoulder_pitch
    13: 1,  # shoulder_roll
    14: 2,  # forearm_roll
}

SERVO_NAMES = {
    12: "shoulder (fwd/back)",
    13: "upper_arm (in/out)",
    14: "forearm (in/out)",
}

# Realistic servo characteristics with asymmetry
BASE_SPEED_DEG_S = 60.0  # 6 sec for 360°
SERVO_SPEEDS = {
    12: (BASE_SPEED_DEG_S * 0.95, BASE_SPEED_DEG_S * 1.05),  # (pos, neg)
    13: (BASE_SPEED_DEG_S * 0.92, BASE_SPEED_DEG_S * 1.08),
    14: (BASE_SPEED_DEG_S * 0.97, BASE_SPEED_DEG_S * 1.03),
}

NOISE_DEG = 0.3

print("\n" + "="*60)
print("LEFT ARM SERVO TIMING (MuJoCo)")
print("="*60)
print(f"Timestep: {SIM_DT*1000:.1f}ms, Control: {CONTROL_DT*1000:.0f}ms")
print(f"Real-time: {'OFF (fast mode)' if args.fast else 'ON'}")
print()
for sid, name in SERVO_NAMES.items():
    pos_spd, neg_spd = SERVO_SPEEDS[sid]
    print(f"Servo {sid} ({name}): +{pos_spd:.1f}°/s, -{neg_spd:.1f}°/s")


def get_angle_deg(servo_id, with_noise=True):
    """Get current servo angle in degrees."""
    idx = SERVO_MAP[servo_id]
    angle = np.degrees(data.qpos[idx])
    if with_noise:
        angle += np.random.normal(0, NOISE_DEG)
    return angle


def set_target_deg(servo_id, angle_deg):
    """Set servo target position in degrees."""
    idx = SERVO_MAP[servo_id]
    data.ctrl[idx] = np.radians(angle_deg)


def step(render=True):
    """Run one control step."""
    for _ in range(STEPS_PER_CONTROL):
        mujoco.mj_step(model, data)
    if viewer is not None and render:
        viewer.sync()
        if not args.fast:
            time.sleep(CONTROL_DT)


def move_servo(servo_id, target_deg, direction_sign):
    """Move servo to target at realistic speed.

    Returns:
        (final_angle, time_taken, error)
    """
    start_angle = get_angle_deg(servo_id, with_noise=False)
    start_time = data.time

    pos_spd, neg_spd = SERVO_SPEEDS[servo_id]
    speed = pos_spd if direction_sign > 0 else neg_spd

    delta = target_deg - start_angle
    expected_time = abs(delta) / speed

    current_target = start_angle

    while True:
        step_size = speed * CONTROL_DT * direction_sign
        current_target += step_size

        # Check if reached target
        if direction_sign < 0:
            if current_target <= target_deg:
                current_target = target_deg
        else:
            if current_target >= target_deg:
                current_target = target_deg

        set_target_deg(servo_id, current_target)
        step(render=True)

        actual = get_angle_deg(servo_id, with_noise=False)
        if abs(actual - target_deg) < 0.5:
            break

        if data.time - start_time > expected_time * 2:
            break

    # Settle
    set_target_deg(servo_id, target_deg)
    for _ in range(10):
        step(render=True)

    final = get_angle_deg(servo_id, with_noise=True)
    time_taken = data.time - start_time
    error = final - target_deg

    return final, time_taken, error


def test_servo(servo_id):
    """Test single servo with ±30° movements."""
    print(f"\n--- Testing Servo {servo_id}: {SERVO_NAMES[servo_id]} ---")

    pos_times = []
    neg_times = []
    pos_errors = []
    neg_errors = []

    current_pos = 0.0

    for cycle in range(3):
        print(f"  Cycle {cycle + 1}:")

        # Move to +30
        target = 30.0
        delta = target - current_pos
        direction = 1 if delta > 0 else -1
        actual, t, err = move_servo(servo_id, target, direction)

        if direction > 0:
            pos_times.append(t)
            pos_errors.append(err)
        else:
            neg_times.append(t)
            neg_errors.append(err)

        print(f"    {current_pos:+6.1f}° → {target:+6.1f}°: t={t:.3f}s, err={err:+.2f}°")
        current_pos = get_angle_deg(servo_id, with_noise=False)

        # Move to -30
        target = -30.0
        delta = target - current_pos
        direction = 1 if delta > 0 else -1
        actual, t, err = move_servo(servo_id, target, direction)

        if direction > 0:
            pos_times.append(t)
            pos_errors.append(err)
        else:
            neg_times.append(t)
            neg_errors.append(err)

        print(f"    {current_pos:+6.1f}° → {target:+6.1f}°: t={t:.3f}s, err={err:+.2f}°")
        current_pos = get_angle_deg(servo_id, with_noise=False)

        # Return to center
        target = 0.0
        delta = target - current_pos
        direction = 1 if delta > 0 else -1
        actual, t, err = move_servo(servo_id, target, direction)

        if direction > 0:
            pos_times.append(t)
            pos_errors.append(err)
        else:
            neg_times.append(t)
            neg_errors.append(err)

        print(f"    {current_pos:+6.1f}° → {target:+6.1f}°: t={t:.3f}s, err={err:+.2f}°")
        current_pos = get_angle_deg(servo_id, with_noise=False)

    # Calculate calibrated speeds
    avg_pos_time = np.mean(pos_times) if pos_times else 0.5
    avg_neg_time = np.mean(neg_times) if neg_times else 0.5

    # 30° movements, so spd = time / 30
    spd_pos = avg_pos_time / 30.0
    spd_neg = avg_neg_time / 30.0

    print(f"  Results: +{spd_pos*1000:.3f}ms/deg, -{spd_neg*1000:.3f}ms/deg")

    return spd_pos, spd_neg


# Initialize
mujoco.mj_resetData(model, data)
for sid in SERVO_MAP:
    set_target_deg(sid, 0)
for _ in range(50):
    step(render=False)

# Test each servo
servo_model = ArmServoModel()

for servo_id in [12, 13, 14]:
    spd_pos, spd_neg = test_servo(servo_id)
    servo_model.set_speeds(servo_id, spd_pos, spd_neg)

    # Reset to center before next servo
    mujoco.mj_resetData(model, data)
    for sid in SERVO_MAP:
        set_target_deg(sid, 0)
    for _ in range(50):
        step(render=False)

# Save results
print("\n" + "="*60)
print("CALIBRATION RESULTS")
print("="*60)
for sid in [12, 13, 14]:
    pos, neg = servo_model.get_speeds(sid)
    asym = abs(pos - neg) / ((pos + neg) / 2) * 100
    print(f"Servo {sid}: +{pos*1000:.3f}ms/deg, -{neg*1000:.3f}ms/deg (asym: {asym:.1f}%)")

output_path = os.path.join(os.path.dirname(__file__), "arm_model_weights.json")
servo_model.save(output_path)

if viewer is not None:
    viewer.close()

print(f"\n[OK] Saved to {output_path}")
print("[OK] Ready for Jetson deployment")
