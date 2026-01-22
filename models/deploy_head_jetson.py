#!/usr/bin/env python3
"""Deploy trained head servo model to Jetson.
Uses head_model.py for inference, controls servo on /dev/ttyUSB1.
"""
import serial
import time
import json

SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 9600
SERVO_CHANNEL = 0
MODEL_PATH = '/home/jetson/work/biped_robot/models/head_model_weights.json'

from head_model import HeadServoModel


def move_servo(ser, servo_id, position, duration_ms=1000):
    position = int(position)
    time_lo = duration_ms & 0xFF
    time_hi = (duration_ms >> 8) & 0xFF
    pos_lo = position & 0xFF
    pos_hi = (position >> 8) & 0xFF
    cmd = bytes([0x55, 0x55, 0x08, 0x03, 0x01,
                 time_lo, time_hi, servo_id, pos_lo, pos_hi])
    ser.write(cmd)
    return cmd


def main():
    print("="*50)
    print("HEAD SERVO MODEL DEPLOYMENT")
    print("="*50)

    print(f"\nConnecting to {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(0.1)
    print("✓ Connected")

    print(f"\nLoading model from {MODEL_PATH}...")
    model = HeadServoModel()
    model.load(MODEL_PATH)

    print("\nTest predictions:")
    for angle in [-30, -15, 0, 15, 30]:
        pos = model.predict(angle)
        print(f"  {angle:+3d}° -> {pos:.0f}")

    print("\n" + "-"*50)
    print("Starting oscillation ±30° (Ctrl+C to stop)\n")

    target_angles = [-30, 30]
    idx = 0

    try:
        while True:
            angle = target_angles[idx]
            pos = model.predict(angle)
            cmd = move_servo(ser, SERVO_CHANNEL, pos, duration_ms=1000)
            print(f"→ {angle:+3d}° (pos: {pos:.0f})")
            time.sleep(2)
            idx = (idx + 1) % len(target_angles)

    except KeyboardInterrupt:
        print("\n\nStopping...")
        center = 1500
        move_servo(ser, SERVO_CHANNEL, center, duration_ms=500)
        print(f"✓ Center ({center})")

    finally:
        ser.close()
        print("✓ Done")


if __name__ == "__main__":
    main()
