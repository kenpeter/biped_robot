#!/usr/bin/env python3
"""
Test the CORRECT channels: 4-11 and 12-18
"""

import serial
import time

port = '/dev/ttyUSB0'
baud = 115200

print("\n" + "="*70)
print("TESTING CORRECT SERVO CHANNELS")
print("="*70)
print("\nAccording to documentation:")
print("  - Servos are on channels 4-11 and 12-18")
print("  - NOT on channel 0!\n")
print("Testing channels 4, 12, and 17 with LARGE movement\n")
print("="*70 + "\n")

ser = serial.Serial(port, baud, timeout=0.5)
ser.dtr = False
ser.rts = False
time.sleep(0.1)

print("✓ Connected to servo board\n")

# Test channels that should have servos connected
test_channels = [4, 5, 12, 17, 18]

for channel in test_channels:
    print(f"\n{'='*60}")
    print(f"Testing Channel {channel} (SHOULD be connected)")
    print(f"{'='*60}")

    positions = [500, 1500, 2500, 1500]
    position_names = ["LEFT (500)", "CENTER (1500)", "RIGHT (2500)", "CENTER (1500)"]

    for pwm, name in zip(positions, position_names):
        cmd = f"{{#{channel:03d}P{pwm:04d}T0800!}}"
        print(f"  {name}... ", end='', flush=True)
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(1.0)
        print("sent")

    print(f"\n  ⚠ Did channel {channel} move?")
    time.sleep(0.5)

print(f"\n{'='*70}")
print("TEST COMPLETE")
print(f"{'='*70}")
print("\nIf you saw movement:")
print("  ✓ Servos ARE working!")
print("  ✓ We just needed the correct channel numbers!")
print("\nIf still no movement:")
print("  - Check which channels actually have servos plugged in")
print("  - Use PC software to verify")
print(f"{'='*70}\n")

ser.close()
