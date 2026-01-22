#!/usr/bin/env python3
"""Calibrate head servo to find real center position."""
import serial
import time

SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 9600
SERVO_CHANNEL = 0


def move_servo(ser, servo_id, position, duration_ms=500):
    position = int(position)
    time_lo = duration_ms & 0xFF
    time_hi = (duration_ms >> 8) & 0xFF
    pos_lo = position & 0xFF
    pos_hi = (position >> 8) & 0xFF
    cmd = bytes([0x55, 0x55, 0x08, 0x03, 0x01,
                 time_lo, time_hi, servo_id, pos_lo, pos_hi])
    ser.write(cmd)


def main():
    print("HEAD SERVO CALIBRATION")
    print("=" * 40)
    print(f"Connecting to {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(0.1)
    print("Connected!\n")

    current_pos = 1500
    print("Commands:")
    print("  +/- : move by 50")
    print("  ++/-- : move by 200")
    print("  [number] : go to position")
    print("  c : mark current as CENTER")
    print("  q : quit\n")

    move_servo(ser, SERVO_CHANNEL, current_pos)
    print(f"Starting at position: {current_pos}")

    center_pos = None

    while True:
        try:
            cmd = input(f"\nPos={current_pos} > ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd == 'q':
            break
        elif cmd == '+':
            current_pos = min(2500, current_pos + 50)
        elif cmd == '-':
            current_pos = max(500, current_pos - 50)
        elif cmd == '++':
            current_pos = min(2500, current_pos + 200)
        elif cmd == '--':
            current_pos = max(500, current_pos - 200)
        elif cmd == 'c':
            center_pos = current_pos
            print(f"\n*** CENTER marked at {center_pos} ***")
            continue
        elif cmd.isdigit():
            current_pos = max(500, min(2500, int(cmd)))
        else:
            print("Unknown command")
            continue

        move_servo(ser, SERVO_CHANNEL, current_pos)
        print(f"Moved to {current_pos}")

    ser.close()

    if center_pos:
        print(f"\n{'='*40}")
        print(f"CALIBRATION RESULT:")
        print(f"  Center position: {center_pos}")
        print(f"\nUpdate MODEL_CENTER in deploy script to: {center_pos}")
    else:
        print("\nNo center marked. Run again to calibrate.")


if __name__ == "__main__":
    main()
