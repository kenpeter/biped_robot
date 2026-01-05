#!/usr/bin/env python3
"""
Test servo control with various initialization sequences
"""

import serial
import time

port = '/dev/ttyUSB0'
baud = 115200

def test_init_sequence(ser, servo_id=0):
    """Try different initialization sequences"""

    init_sequences = [
        ("No init", []),
        ("Reset command", [b"$RST!\r\n"]),
        ("Query version", [b"$DGS:0!\r\n"]),
        ("Baud rate set", [b"$UBBS:1,115200!\r\n"]),
        ("Enable servos", [b"$SEN:1!\r\n"]),
        ("Power on", [b"$PON!\r\n"]),
        ("Init mode", [b"$INIT!\r\n"]),
        ("Start command", [b"$START!\r\n"]),
        ("Multiple inits", [b"$RST!\r\n", b"$SEN:1!\r\n"]),
    ]

    for name, commands in init_sequences:
        print(f"\n{'='*60}")
        print(f"Testing: {name}")
        print('='*60)

        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Send initialization commands
        for cmd in commands:
            print(f"  Init: {cmd}")
            ser.write(cmd)
            ser.flush()
            time.sleep(0.2)

            # Check response
            if ser.in_waiting > 0:
                resp = ser.read(ser.in_waiting)
                print(f"  Response: {resp}")

        # Now try servo command
        servo_cmd = f"#000P1500T1000!"
        print(f"  Servo command: {servo_cmd}")
        ser.write(servo_cmd.encode('ascii'))
        ser.flush()
        time.sleep(0.5)

        # Try with braces
        servo_cmd = f"{{#000P1500T1000!}}"
        print(f"  Servo command: {servo_cmd}")
        ser.write(servo_cmd.encode('ascii'))
        ser.flush()
        time.sleep(0.5)

        # Move to extreme position
        servo_cmd = f"{{#000P2500T1000!}}"
        print(f"  Servo command: {servo_cmd}")
        ser.write(servo_cmd.encode('ascii'))
        ser.flush()
        time.sleep(1.0)

        print("\nDid servo move? (watching...)")
        time.sleep(2)

print("\n" + "="*70)
print("INITIALIZATION SEQUENCE TEST")
print("="*70)
print("\nThis will try different initialization commands that PC software")
print("might be sending before servo commands.")
print("\nWatch your robot carefully for ANY movement!")
print("\nPress Enter to start...")
input()

try:
    ser = serial.Serial(port, baud, timeout=0.5)
    ser.dtr = False
    ser.rts = False
    time.sleep(0.1)

    print(f"\n✓ Connected to {port} at {baud} baud\n")

    test_init_sequence(ser)

    print("\n" + "="*70)
    print("Test complete!")
    print("="*70)
    print("\nIf nothing moved, the issue might be:")
    print("  1. Wrong servo channel (servo 0 not connected)")
    print("  2. Different protocol needed")
    print("  3. Need to spy on PC software to see exact commands")
    print("\n" + "="*70 + "\n")

    ser.close()

except Exception as e:
    print(f"\n✗ Error: {e}\n")
