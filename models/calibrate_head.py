#!/usr/bin/env python3
"""Calibrate continuous rotation head servo to find center (forward facing).

Since continuous rotation servos have no position feedback, we need a way
to establish a known "center" position. This script helps with that.

Usage:
  python3 calibrate_head.py          # Interactive calibration
  python3 calibrate_head.py reset    # Reset to saved center
  python3 calibrate_head.py show     # Show saved calibration
"""
import serial
import time
import sys
import json
import os

SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 9600
SERVO_CHANNEL = 0
CALIB_FILE = '/home/jetson/work/biped_robot/models/head_calibration.json'

# Servo parameters
STOP_VALUE = 1500
SPEED_LEFT = 1630  # reduced to compensate for overshoot
SPEED_RIGHT = 1350
SECONDS_PER_DEGREE = 6.0 / 360.0


def load_calibration():
    """Load saved calibration."""
    if os.path.exists(CALIB_FILE):
        with open(CALIB_FILE) as f:
            return json.load(f)
    return {'offset_from_center': 0.0}


def save_calibration(calib):
    """Save calibration to file."""
    with open(CALIB_FILE, 'w') as f:
        json.dump(calib, f, indent=2)


def send_speed(ser, speed):
    """Send speed to servo."""
    speed = int(max(500, min(2500, speed)))
    cmd = bytes([0x55, 0x55, 0x08, 0x03, 0x01, 0, 0,
                 SERVO_CHANNEL, speed & 0xFF, (speed >> 8) & 0xFF])
    ser.write(cmd)


def stop_servo(ser):
    """Stop servo."""
    send_speed(ser, STOP_VALUE)


def rotate(ser, degrees):
    """Rotate by degrees (positive=left, negative=right)."""
    if abs(degrees) < 0.1:
        return
    duration = abs(degrees) * SECONDS_PER_DEGREE
    speed = SPEED_LEFT if degrees > 0 else SPEED_RIGHT
    send_speed(ser, speed)
    time.sleep(duration)
    stop_servo(ser)


def main():
    calib = load_calibration()

    if len(sys.argv) > 1:
        cmd = sys.argv[1]

        if cmd == 'reset':
            offset = calib.get('offset_from_center', 0.0)
            if abs(offset) < 0.1:
                print("Already at center (no offset)")
                return

            print(f"Current offset: {offset:+.1f}°")
            print(f"Rotating {-offset:+.1f}° to return to center...")

            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            rotate(ser, -offset)
            ser.close()

            calib['offset_from_center'] = 0.0
            save_calibration(calib)
            print("[OK] Head reset to center (forward facing)")
            return

        elif cmd == 'show':
            offset = calib.get('offset_from_center', 0.0)
            print(f"Saved offset from center: {offset:+.1f}°")
            if abs(offset) < 0.1:
                print("Head is at center")
            else:
                print(f"Run 'calibrate_head.py reset' to return to center")
            return

    # Interactive calibration
    print("="*50)
    print("HEAD SERVO CALIBRATION (Continuous Rotation)")
    print("="*50)
    print("\nPosition head facing FORWARD, then fine-tune:\n")
    print("  l      - nudge left 5°")
    print("  r      - nudge right 5°")
    print("  ll     - nudge left 1°")
    print("  rr     - nudge right 1°")
    print("  l 20   - rotate left 20°")
    print("  r 30   - rotate right 30°")
    print("  save   - save current as center")
    print("  reset  - go back to saved center")
    print("  quit   - exit")
    print()

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    stop_servo(ser)

    # Track how far we've moved from where we started
    offset = calib.get('offset_from_center', 0.0)

    try:
        while True:
            prompt = f"[{offset:+.1f}° from center] > "
            cmd = input(prompt).strip().lower()

            if not cmd:
                continue

            elif cmd in ['q', 'quit', 'exit']:
                # Save current offset before exiting
                calib['offset_from_center'] = offset
                save_calibration(calib)
                print(f"Saved offset: {offset:+.1f}°")
                break

            elif cmd in ['s', 'save']:
                # Current position becomes the new center
                calib['offset_from_center'] = 0.0
                save_calibration(calib)
                offset = 0.0
                print("[OK] Current position saved as CENTER")

            elif cmd == 'reset':
                if abs(offset) < 0.1:
                    print("Already at center")
                else:
                    print(f"Returning to center...")
                    rotate(ser, -offset)
                    offset = 0.0
                    calib['offset_from_center'] = 0.0
                    save_calibration(calib)
                    print("[OK] At center")

            elif cmd == 'l' or cmd == 'left':
                rotate(ser, 5)
                offset += 5
                print(f"Nudged left 5°")

            elif cmd == 'r' or cmd == 'right':
                rotate(ser, -5)
                offset -= 5
                print(f"Nudged right 5°")

            elif cmd == 'll':
                rotate(ser, 1)
                offset += 1
                print(f"Nudged left 1°")

            elif cmd == 'rr':
                rotate(ser, -1)
                offset -= 1
                print(f"Nudged right 1°")

            elif cmd.startswith('l '):
                try:
                    deg = float(cmd.split()[1])
                    rotate(ser, deg)
                    offset += deg
                    print(f"Rotated left {deg}°")
                except (IndexError, ValueError):
                    print("Usage: l <degrees>")

            elif cmd.startswith('r '):
                try:
                    deg = float(cmd.split()[1])
                    rotate(ser, -deg)
                    offset -= deg
                    print(f"Rotated right {deg}°")
                except (IndexError, ValueError):
                    print("Usage: r <degrees>")

            else:
                print("Unknown command")

    except KeyboardInterrupt:
        calib['offset_from_center'] = offset
        save_calibration(calib)
        print(f"\nSaved offset: {offset:+.1f}°")

    stop_servo(ser)
    ser.close()
    print("[OK] Done")


if __name__ == "__main__":
    main()
