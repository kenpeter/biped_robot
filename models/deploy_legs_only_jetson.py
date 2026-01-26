#!/usr/bin/env python3
"""Deploy trained walking policy with LEGS ONLY - upper body frozen.

Controls only 8 leg servos for walking, keeps upper body (head + arms) neutral.
This prevents twisting and makes the robot easier to control.

Hardware:
- 8 active servos: 2 legs × 4 joints each (hip roll, hip pitch, knee, ankle)
- 7 frozen servos: head (1) + arms (6) set to neutral position

Usage:
    python3 deploy_legs_only_jetson.py           # Run walking policy (legs only)
    python3 deploy_legs_only_jetson.py --demo    # Demo mode for testing
"""

import sys
import time
import serial
import numpy as np

# Servo configuration
SERVO_PORT = '/dev/ttyUSB1'
SERVO_BAUD = 9600
SPEED_STOP = 1500
SPEED_RANGE = 150

# ONLY leg servos (8 servos)
LEG_SERVOS = {
    'r_hip_roll': 4,
    'r_hip_pitch': 5,
    'r_knee': 6,
    'r_ankle': 7,
    'l_hip_roll': 15,
    'l_hip_pitch': 16,
    'l_knee': 17,
    'l_ankle': 18
}

# Upper body servos to FREEZE (7 servos)
FROZEN_SERVOS = {
    'head': 0,
    'r_shoulder': 1,
    'r_upper_arm': 2,
    'r_forearm': 3,
    'l_shoulder': 12,
    'l_upper_arm': 13,
    'l_forearm': 14
}


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

    def freeze_upper_body(self):
        """Set all upper body servos to neutral/stop position."""
        print("[FREEZE] Upper body (head + arms)...")
        for servo_id in FROZEN_SERVOS.values():
            self.send_speed(servo_id, SPEED_STOP)

    def stop_all(self):
        """Stop ALL servos (legs + upper body)."""
        all_servos = list(LEG_SERVOS.values()) + list(FROZEN_SERVOS.values())
        for servo_id in all_servos:
            self.send_speed(servo_id, SPEED_STOP)

    def close(self):
        """Close serial connection."""
        self.stop_all()
        self.ser.close()


class LegsOnlyPolicyRunner:
    """Run trained PPO policy with LEGS ONLY."""

    def __init__(self, policy_path):
        self.controller = ServoController()

        # Freeze upper body immediately
        self.controller.freeze_upper_body()
        time.sleep(0.5)

        # Load PPO model
        print(f"[POLICY] Loading {policy_path}...")
        try:
            from stable_baselines3 import PPO
            self.model = PPO.load(policy_path)
            print("[POLICY] Model loaded successfully!")
            print("[INFO] Using LEGS ONLY (8 servos)")
            print("[INFO] Upper body FROZEN (7 servos)")
        except Exception as e:
            print(f"[ERROR] Failed to load model: {e}")
            print("Install: pip install stable-baselines3")
            sys.exit(1)

        # State tracking (LEGS ONLY - 8 joints)
        self.leg_positions = np.zeros(8)
        self.leg_velocities = np.zeros(8)
        self.gait_phase = 0.0
        self.last_time = time.time()

    def get_observation(self):
        """Get observation for policy.

        Model expects 44 dims, but we only control 8 leg servos.
        Fill rest with neutral/zero values.
        """
        # Leg joint positions (8)
        leg_pos = self.leg_positions.copy()

        # Leg joint velocities (8)
        leg_vel = self.leg_velocities.copy()

        # Upper body joints (7) - FROZEN at neutral
        upper_pos = np.zeros(7)
        upper_vel = np.zeros(7)

        # Combine all joint states (15 total)
        # Order: head(1), r_arm(3), r_leg(4), l_arm(3), l_leg(4)
        joint_pos = np.concatenate([
            upper_pos[0:1],     # head
            upper_pos[1:4],     # r_arm
            leg_pos[0:4],       # r_leg
            upper_pos[4:7],     # l_arm
            leg_pos[4:8]        # l_leg
        ])

        joint_vel = np.concatenate([
            upper_vel[0:1],     # head
            upper_vel[1:4],     # r_arm
            leg_vel[0:4],       # r_leg
            upper_vel[4:7],     # l_arm
            leg_vel[4:8]        # l_leg
        ])

        # Torso orientation (assume upright)
        torso_quat = np.array([1.0, 0.0, 0.0, 0.0])

        # Torso velocity (assume stationary)
        torso_vel = np.zeros(6)

        # Foot contacts (estimate from knee position)
        right_foot_contact = 1.0 if self.leg_positions[2] > 0.2 else 0.0
        left_foot_contact = 1.0 if self.leg_positions[6] > 0.2 else 0.0
        foot_contacts = np.array([right_foot_contact, left_foot_contact])

        # Gait phase clock
        gait_clock = np.array([np.sin(self.gait_phase), np.cos(self.gait_phase)])

        # Combine observation (44 dimensions)
        obs = np.concatenate([
            joint_pos,      # 15
            joint_vel,      # 15
            torso_quat,     # 4
            torso_vel,      # 6
            foot_contacts,  # 2
            gait_clock      # 2
        ])

        return obs.astype(np.float32)

    def extract_leg_actions(self, full_action):
        """Extract only leg actions from full 15-joint action.

        Action order: head(1), r_arm(3), r_leg(4), l_arm(3), l_leg(4)
        We only want: r_leg(4) + l_leg(4) = 8 actions
        """
        # Skip head(1) + r_arm(3) = indices 0-3
        # Take r_leg = indices 4-7
        r_leg = full_action[4:8]

        # Skip l_arm(3) = indices 8-10
        # Take l_leg = indices 11-14
        l_leg = full_action[11:15]

        return np.concatenate([r_leg, l_leg])  # 8 actions

    def action_to_servo_speeds(self, leg_actions):
        """Convert leg actions [-1, 1] to servo speeds."""
        speeds = SPEED_STOP + (leg_actions * SPEED_RANGE)
        return speeds

    def update_state_estimate(self, leg_actions, dt):
        """Update internal state estimate for legs."""
        MAX_VEL = np.radians(60.0)
        velocities = leg_actions * MAX_VEL

        # Update leg positions
        self.leg_positions += velocities * dt
        self.leg_positions = np.clip(self.leg_positions, -np.pi, np.pi)

        # Update leg velocities (with damping)
        self.leg_velocities = velocities * 0.8 + self.leg_velocities * 0.2

        # Update gait phase (2 Hz walking cycle)
        self.gait_phase += 2.0 * np.pi * 2.0 * dt
        self.gait_phase = self.gait_phase % (2.0 * np.pi)

    def run(self, duration=60.0):
        """Run policy for specified duration."""
        print("\n" + "="*60)
        print("WALKING WITH LEGS ONLY (Upper Body Frozen)")
        print("="*60)
        print(f"Duration: {duration}s")
        print(f"Active servos: {len(LEG_SERVOS)} (legs)")
        print(f"Frozen servos: {len(FROZEN_SERVOS)} (head + arms)")
        print("\nPress Ctrl+C to stop\n")

        try:
            start_time = time.time()
            step_count = 0

            while (time.time() - start_time) < duration:
                current_time = time.time()
                dt = current_time - self.last_time
                self.last_time = current_time

                # Get observation (with frozen upper body)
                obs = self.get_observation()

                # Get action from policy (15 actions for all joints)
                full_action, _ = self.model.predict(obs, deterministic=True)
                full_action = np.clip(full_action, -1, 1)

                # Extract ONLY leg actions (8 actions)
                leg_actions = self.extract_leg_actions(full_action)

                # Convert to servo speeds
                speeds = self.action_to_servo_speeds(leg_actions)

                # Send to leg servos only
                for i, servo_id in enumerate(LEG_SERVOS.values()):
                    self.controller.send_speed(servo_id, int(speeds[i]))

                # Keep upper body frozen
                if step_count % 20 == 0:  # Re-freeze every 20 steps
                    self.controller.freeze_upper_body()

                # Update state estimate
                self.update_state_estimate(leg_actions, dt)

                # Log
                step_count += 1
                if step_count % 50 == 0:
                    elapsed = time.time() - start_time
                    print(f"[{elapsed:6.1f}s] Step {step_count:4d} | "
                          f"Gait: {np.degrees(self.gait_phase):6.1f}° | "
                          f"Legs active, upper body frozen")

                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\n[INTERRUPTED]")

        finally:
            print("[STOPPING] All servos...")
            self.controller.stop_all()
            self.controller.close()
            print("[DONE]")


def demo_mode():
    """Demo mode - simple leg-only walking pattern."""
    controller = ServoController()
    controller.freeze_upper_body()

    print("\n" + "="*60)
    print("DEMO MODE - Legs Only Simple Walking")
    print("="*60)
    print("\nCommands:")
    print("  walk      - Simple leg walking pattern (5 sec)")
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
                print("[WALK] Simple leg pattern for 5 seconds...")
                for cycle in range(25):
                    phase = (cycle % 10) / 10.0 * 2 * np.pi

                    # Right leg
                    controller.send_speed(LEG_SERVOS['r_hip_pitch'],
                                        int(SPEED_STOP + 50 * np.sin(phase)))
                    controller.send_speed(LEG_SERVOS['r_knee'],
                                        int(SPEED_STOP + 80 * abs(np.sin(phase))))

                    # Left leg (opposite phase)
                    controller.send_speed(LEG_SERVOS['l_hip_pitch'],
                                        int(SPEED_STOP + 50 * np.sin(phase + np.pi)))
                    controller.send_speed(LEG_SERVOS['l_knee'],
                                        int(SPEED_STOP + 80 * abs(np.sin(phase + np.pi))))

                    time.sleep(0.2)

                controller.stop_all()
                print("[OK] Walking complete")
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
        policy_path = '/home/kenpeter/work/biped_robot/models/full_robot_ppo.zip'
        runner = LegsOnlyPolicyRunner(policy_path)
        runner.run(duration=60.0)


if __name__ == "__main__":
    main()
