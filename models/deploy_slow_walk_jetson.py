#!/usr/bin/env python3
"""Slow, human-like walking for real robot - like a baby learning to walk.

Implements deliberate, slow gait pattern:
1. Lift right leg (bend knee)
2. Swing right leg forward (hip pitch)
3. Put right leg down (straighten knee)
4. Shift weight
5. Repeat with left leg

Usage:
    python3 deploy_slow_walk_jetson.py           # Slow walking
    python3 deploy_slow_walk_jetson.py --demo    # Demo mode
"""

import sys
import time
import serial
import numpy as np

SERVO_PORT = '/dev/ttyUSB1'
SERVO_BAUD = 9600
SPEED_STOP = 1500

# Leg servos only
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

# Upper body - frozen
FROZEN_SERVOS = [0, 1, 2, 3, 12, 13, 14]

# SLOW movement speeds (much slower than default)
SPEED_SLOW = 30      # Very slow movements
SPEED_MEDIUM = 50    # Medium speed
SPEED_FAST = 80      # Still relatively slow

# Movement durations (in seconds) - SLOW like baby
LIFT_DURATION = 1.5      # 1.5 seconds to lift leg
SWING_DURATION = 2.0     # 2 seconds to swing forward
DOWN_DURATION = 1.5      # 1.5 seconds to put leg down
SHIFT_DURATION = 1.0     # 1 second to shift weight
PAUSE_DURATION = 0.5     # 0.5 second pause between steps


class ServoController:
    """Control servos with slow, deliberate movements."""

    def __init__(self):
        self.ser = serial.Serial(SERVO_PORT, SERVO_BAUD, timeout=0.1)
        time.sleep(0.1)
        print(f"[SERVO] Connected to {SERVO_PORT}")

    def send_speed(self, servo_id, speed):
        """Send speed command."""
        speed = int(np.clip(speed, SPEED_STOP - 200, SPEED_STOP + 200))
        speed_lo = speed & 0xFF
        speed_hi = (speed >> 8) & 0xFF
        cmd = bytes([0x55, 0x55, 0x08, 0x03, 0x01, 0, 0, servo_id, speed_lo, speed_hi])
        self.ser.write(cmd)

    def move_servo_slow(self, servo_id, direction, speed, duration):
        """Move servo slowly for duration, then stop.

        direction: 1 for forward/up, -1 for backward/down
        """
        target_speed = SPEED_STOP + (direction * speed)
        self.send_speed(servo_id, target_speed)
        time.sleep(duration)
        self.send_speed(servo_id, SPEED_STOP)

    def freeze_upper_body(self):
        """Keep upper body still."""
        for servo_id in FROZEN_SERVOS:
            self.send_speed(servo_id, SPEED_STOP)

    def stop_all(self):
        """Stop all servos."""
        for servo_id in list(LEG_SERVOS.values()) + FROZEN_SERVOS:
            self.send_speed(servo_id, SPEED_STOP)

    def close(self):
        self.stop_all()
        self.ser.close()


class SlowWalker:
    """Human-like slow walking controller."""

    def __init__(self):
        self.controller = ServoController()
        self.controller.freeze_upper_body()
        time.sleep(0.5)
        print("[WALKER] Slow walking mode initialized")
        print("[INFO] Upper body frozen, legs moving slowly")

    def step_right_leg(self):
        """Take one step with right leg - SLOW like human."""
        print("  [RIGHT LEG] Lifting...")

        # 1. Lift right leg (bend knee)
        self.controller.move_servo_slow(
            LEG_SERVOS['r_knee'],
            1,                    # bend (positive)
            SPEED_SLOW,
            LIFT_DURATION
        )

        print("  [RIGHT LEG] Swinging forward...")

        # 2. Swing hip forward (while knee is bent)
        self.controller.move_servo_slow(
            LEG_SERVOS['r_hip_pitch'],
            1,                    # forward
            SPEED_SLOW,
            SWING_DURATION
        )

        print("  [RIGHT LEG] Putting down...")

        # 3. Put leg down (straighten knee)
        self.controller.move_servo_slow(
            LEG_SERVOS['r_knee'],
            -1,                   # straighten
            SPEED_SLOW,
            DOWN_DURATION
        )

        print("  [RIGHT LEG] Step complete")
        time.sleep(PAUSE_DURATION)

    def step_left_leg(self):
        """Take one step with left leg - SLOW like human."""
        print("  [LEFT LEG] Lifting...")

        # 1. Lift left leg (bend knee)
        self.controller.move_servo_slow(
            LEG_SERVOS['l_knee'],
            1,                    # bend
            SPEED_SLOW,
            LIFT_DURATION
        )

        print("  [LEFT LEG] Swinging forward...")

        # 2. Swing hip forward
        self.controller.move_servo_slow(
            LEG_SERVOS['l_hip_pitch'],
            1,                    # forward
            SPEED_SLOW,
            SWING_DURATION
        )

        print("  [LEFT LEG] Putting down...")

        # 3. Put leg down
        self.controller.move_servo_slow(
            LEG_SERVOS['l_knee'],
            -1,                   # straighten
            SPEED_SLOW,
            DOWN_DURATION
        )

        print("  [LEFT LEG] Step complete")
        time.sleep(PAUSE_DURATION)

    def walk(self, num_steps=10):
        """Walk forward with alternating legs - SLOW like baby."""
        print("\n" + "="*60)
        print("SLOW WALKING - Human-like Gait")
        print("="*60)
        print(f"Steps: {num_steps}")
        print("Speed: SLOW (baby learning to walk)")
        print("\nPress Ctrl+C to stop\n")

        try:
            for step_num in range(num_steps):
                print(f"\n=== Step {step_num + 1}/{num_steps} ===")

                # Alternate legs
                if step_num % 2 == 0:
                    self.step_right_leg()
                else:
                    self.step_left_leg()

                # Keep upper body frozen
                self.controller.freeze_upper_body()

            print("\n[COMPLETE] Walking finished")

        except KeyboardInterrupt:
            print("\n[INTERRUPTED]")

        finally:
            print("[STOPPING] All servos...")
            self.controller.stop_all()
            self.controller.close()
            print("[DONE]")


def demo_mode():
    """Demo individual movements."""
    controller = ServoController()
    controller.freeze_upper_body()

    print("\n" + "="*60)
    print("DEMO MODE - Test Individual Movements")
    print("="*60)
    print("\nCommands:")
    print("  r_lift     - Lift right leg (bend knee)")
    print("  r_down     - Put right leg down")
    print("  l_lift     - Lift left leg")
    print("  l_down     - Put left leg down")
    print("  r_forward  - Swing right hip forward")
    print("  l_forward  - Swing left hip forward")
    print("  stop       - Stop all")
    print("  quit       - Exit")
    print()

    try:
        while True:
            cmd = input("Command: ").strip().lower()

            if cmd == 'quit':
                break
            elif cmd == 'stop':
                controller.stop_all()
                print("[OK] All stopped")

            # Right leg
            elif cmd == 'r_lift':
                print("[RIGHT] Lifting leg (bending knee)...")
                controller.move_servo_slow(LEG_SERVOS['r_knee'], 1, SPEED_SLOW, LIFT_DURATION)
                print("[OK]")
            elif cmd == 'r_down':
                print("[RIGHT] Putting leg down...")
                controller.move_servo_slow(LEG_SERVOS['r_knee'], -1, SPEED_SLOW, DOWN_DURATION)
                print("[OK]")
            elif cmd == 'r_forward':
                print("[RIGHT] Swinging hip forward...")
                controller.move_servo_slow(LEG_SERVOS['r_hip_pitch'], 1, SPEED_SLOW, SWING_DURATION)
                print("[OK]")

            # Left leg
            elif cmd == 'l_lift':
                print("[LEFT] Lifting leg...")
                controller.move_servo_slow(LEG_SERVOS['l_knee'], 1, SPEED_SLOW, LIFT_DURATION)
                print("[OK]")
            elif cmd == 'l_down':
                print("[LEFT] Putting leg down...")
                controller.move_servo_slow(LEG_SERVOS['l_knee'], -1, SPEED_SLOW, DOWN_DURATION)
                print("[OK]")
            elif cmd == 'l_forward':
                print("[LEFT] Swinging hip forward...")
                controller.move_servo_slow(LEG_SERVOS['l_hip_pitch'], 1, SPEED_SLOW, SWING_DURATION)
                print("[OK]")

            else:
                print(f"Unknown: {cmd}")

    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")

    finally:
        controller.stop_all()
        controller.close()


def main():
    if '--demo' in sys.argv:
        demo_mode()
    else:
        walker = SlowWalker()
        walker.walk(num_steps=20)  # 20 slow steps


if __name__ == "__main__":
    main()
