#!/usr/bin/env python3
"""
SIMPLE TEST - One servo, full range, very slow
"""

import serial
import time

port = '/dev/ttyUSB0'
baud = 115200

print("\n" + "="*70)
print("SIMPLE SERVO TEST - Servo 0 full range movement")
print("="*70)
print("\nThis will move servo 0 from position 500 to 2500 SLOWLY")
print("You CANNOT miss this movement if servo 0 is connected!\n")

ser = serial.Serial(port, baud, timeout=0.5)
ser.dtr = False
ser.rts = False
time.sleep(0.1)

print("Connected to servo board\n")

# Very obvious full range sweep
print("Moving servo 0 through full range:")
print("500 → 1000 → 1500 → 2000 → 2500 → 2000 → 1500 → 1000 → 500\n")

for pwm in [500, 1000, 1500, 2000, 2500, 2000, 1500, 1000, 500]:
    cmd = f"{{#000P{pwm:04d}T1000!}}"
    print(f"Position {pwm}... ", end='', flush=True)
    ser.write(cmd.encode('ascii'))
    ser.flush()
    time.sleep(1.5)
    print("sent")

print("\n" + "="*70)
print("Did you see servo 0 move?")
print("="*70)
print("\nIf NO movement:")
print("  1. Servo 0 might not be connected to the board")
print("  2. Try a different servo number")
print("\nWhich servo number works in the PC software?")
print("We can test that specific servo next.")
print("="*70 + "\n")

ser.close()
