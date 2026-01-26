#!/usr/bin/env python3
"""Deploy trained full robot walking policy to Jetson with real servos.

Uses the trained PPO model to control all 15 servos for bipedal walking.

Hardware Requirements:
- Hiwonder LSC-24 servo board on /dev/ttyUSB1 @ 9600 baud
- All 15 servos configured with continuous rotation

Usage:
    python3 deploy_full_robot_jetson.py           # Run walking policy
    python3 deploy_full_robot_jetson.py --demo    # Demo mode with manual control
"""

import os
import sys
import time
import serial
import struct
import numpy as np

# Servo configuration
SERVO_PORT = '/dev/ttyUSB1'
SERVO_BAUD = 9600

# Servo channels (15 total)
SERVOS = {
    'head': 0,
    'r_shoulder': 1, 'r_upper_arm': 2, 'r_forearm': 3,
    'r_hip_roll': 4, 'r_hip_pitch': 5, 'r_knee': 6, 'r_ankle': 7,
    'l_shoulder': 12, 'l_upper_arm': 13, 'l_forearm': 14,
    'l_hip_roll': 15, 'l_hip_pitch': 16, 'l_knee': 17, 'l_ankle': 18
}

# Continuous rotation servo speeds
SPEED_STOP = 1500
SPEED_RANGE = 150  # ±150 from stop

# Timing calibration (from training)
SECONDS_PER_DEGREE = 0.01667  # ~6 seconds per 360°


class ServoController:
    """Control Hiwonder LSC-24 servo board."""

    def __init__(self, port=SERVO_PORT, baud=SERVO_BAUD):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(0.1)
        print(f"[SERVO] Connected to {port} @ {baud} baud")

    def send_speed(self, servo_id, speed):
        """Send speed command to continuous rotation servo."""
        speed = int(np.clip(speed, SPEED_STOP - SPEED_RANGE, SPEED_STOP + SPEED_RANGE))
        speed_lo = speed & 0xFF
        speed_hi = (speed >> 8) & 0xFF

        cmd = bytes([0x55, 0x55, 0x08, 0x03, 0x01, 0, 0, servo_id, speed_lo, speed_hi])
        self.ser.write(cmd)

    def stop_all(self):
        """Stop all servos."""
        for servo_id in SERVOS.values():
            self.send_speed(servo_id, SPEED_STOP)

    def close(self):
        """Close serial connection."""
        self.stop_all()
        self.ser.close()


class PolicyRunner:
    """Run trained PPO policy on real hardware."""

    def __init__(self, policy_path):
        self.controller = ServoController()
        self.policy_path = policy_path

        # Load PPO model
        print(f"[POLICY] Loading {policy_path}...")
        try:
            from stable_baselines3 import PPO
            self.model = PPO.load(policy_path)
            print("[POLICY] Model loaded successfully!")
        except Exception as e:
            print(f"[ERROR] Failed to load model: {e}")
            print("Make sure stable-baselines3 is installed:")
            print("  pip install stable-baselines3")
            sys.exit(1)

        # State tracking
        self.joint_positions = np.zeros(15)
        self.joint_velocities = np.zeros(15)
        self.gait_phase = 0.0
        self.last_time = time.time()

    def get_observation(self):
        """Get current observation for policy.

        In simulation we had sensors. On real hardware, we estimate based on
        commanded positions (open-loop control).
        """
        # Torso orientation (assume upright for now - could add IMU later)
        torso_quat = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z

        # Torso velocity (assume zero for now - could add IMU later)
        torso_vel = np.zeros(6)

        # Foot contacts (estimate from leg positions)
        # Simple heuristic: if knee is bent, foot is likely on ground
        right_foot_contact = 1.0 if self.joint_positions[6] > 0.2 else 0.0
        left_foot_contact = 1.0 if self.joint_positions[13] > 0.2 else 0.0
        foot_contacts = np.array([right_foot_contact, left_foot_contact])

        # Gait phase clock
        gait_clock = np.array([np.sin(self.gait_phase), np.cos(self.gait_phase)])

        # Combine observation (must match training: 44 dimensions)
        obs = np.concatenate([
            self.joint_positions,      # 15
            self.joint_velocities,     # 15
            torso_quat,                # 4
            torso_vel,                 # 6
            foot_contacts,             # 2
            gait_clock                 # 2
        ])

        return obs.astype(np.float32)

    def action_to_servo_speeds(self, action):
        """Convert policy action [-1, 1] to servo speeds."""
        # Action is in range [-1, 1], map to servo speeds
        speeds = SPEED_STOP + (action * SPEED_RANGE)
        return speeds

    def update_state_estimate(self, action, dt):
        """Update internal state estimate based on action."""
        # Simple integration: position += velocity * dt
        # velocity is proportional to action (servo speed)

        # Convert action to angular velocities (rad/s)
        # action=1 → full speed → 360° in 6 sec → 60°/sec → 1.047 rad/s
        MAX_VEL = np.radians(60.0)
        velocities = action * MAX_VEL

        # Update positions
        self.joint_positions += velocities * dt
        self.joint_positions = np.clip(self.joint_positions, -np.pi, np.pi)

        # Update velocities (with damping)
        self.joint_velocities = velocities * 0.8 + self.joint_velocities * 0.2

        # Update gait phase (2 Hz walking cycle)
        self.gait_phase += 2.0 * np.pi * 2.0 * dt
        self.gait_phase = self.gait_phase % (2.0 * np.pi)

    def run(self, duration=60.0):
        """Run policy for specified duration."""
        print("\n" + "="*60)
        print("RUNNING WALKING POLICY")
        print("="*60)
        print(f"Duration: {duration}s")
        print("Press Ctrl+C to stop\n")

        try:
            start_time = time.time()
            step_count = 0

            while (time.time() - start_time) < duration:
                # Get current time
                current_time = time.time()
                dt = current_time - self.last_time
                self.last_time = current_time

                # Get observation
                obs = self.get_observation()

                # Get action from policy
                action, _ = self.model.predict(obs, deterministic=True)
                action = np.clip(action, -1, 1)

                # Convert to servo speeds
                speeds = self.action_to_servo_speeds(action)

                # Send to servos
                for i, servo_id in enumerate(SERVOS.values()):
                    self.controller.send_speed(servo_id, int(speeds[i]))

                # Update state estimate
                self.update_state_estimate(action, dt)

                # Log periodically
                step_count += 1
                if step_count % 50 == 0:
                    elapsed = time.time() - start_time
                    print(f"[{elapsed:6.1f}s] Step {step_count:4d} | "
                          f"Gait phase: {np.degrees(self.gait_phase):6.1f}°")

                # Control loop at ~50Hz
                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\n[INTERRUPTED]")

        finally:
            print("[STOPPING] All servos...")
            self.controller.stop_all()
            self.controller.close()
            print("[DONE]")


def demo_mode():
    """Demo mode for manual testing."""
    controller = ServoController()

    print("\n" + "="*60)
    print("DEMO MODE - Manual Servo Control")
    print("="*60)
    print("\nCommands:")
    print("  walk      - Simulate simple walking pattern")
    print("  stop      - Stop all servos")
    print("  quit      - Exit")
    print()

    try:
        while True:
            cmd = input("Command: ").strip().lower()

            if cmd == 'quit':
                break
            elif cmd == 'stop':
                controller.stop_all()
                print("[OK] All servos stopped")
            elif cmd == 'walk':
                print("[WALK] Running simple walking pattern for 5 seconds...")
                # Simple alternating leg pattern
                for cycle in range(25):  # 5 seconds at 5Hz
                    phase = (cycle % 10) / 10.0 * 2 * np.pi

                    # Right leg
                    controller.send_speed(SERVOS['r_hip_pitch'],
                                        int(SPEED_STOP + 50 * np.sin(phase)))
                    controller.send_speed(SERVOS['r_knee'],
                                        int(SPEED_STOP + 80 * abs(np.sin(phase))))

                    # Left leg (opposite phase)
                    controller.send_speed(SERVOS['l_hip_pitch'],
                                        int(SPEED_STOP + 50 * np.sin(phase + np.pi)))
                    controller.send_speed(SERVOS['l_knee'],
                                        int(SPEED_STOP + 80 * abs(np.sin(phase + np.pi))))

                    time.sleep(0.2)

                controller.stop_all()
                print("[OK] Walking pattern complete")
            else:
                print(f"Unknown command: {cmd}")

    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")

    finally:
        controller.stop_all()
        controller.close()


def main():
    if '--demo' in sys.argv:
        demo_mode()
    else:
        # Use path relative to this script's location
        script_dir = os.path.dirname(os.path.abspath(__file__))
        policy_path = os.path.join(script_dir, 'full_robot_ppo')  # No .zip - SB3 adds it
        runner = PolicyRunner(policy_path)
        runner.run(duration=60.0)


if __name__ == "__main__":
    main()
