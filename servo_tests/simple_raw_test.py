#!/usr/bin/env python3
"""
Simplest possible test - just send raw commands and see what happens
"""

import serial
import time
import sys

def test_port(port, baudrate):
    print("="*70)
    print(f"Testing: {port} at {baudrate} baud")
    print("="*70)

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print("✓ Port opened successfully")
        time.sleep(0.5)

        # Test 1: Board reset
        print("\nTest 1: Sending reset command: $RSTI")
        ser.write(b"$RSTI")
        time.sleep(0.5)
        if ser.in_waiting:
            print(f"  Response: {ser.read(ser.in_waiting)}")
        else:
            print("  No response")

        # Test 2: Simple servo command
        cmd = b"#000P1500T1000!"
        print(f"\nTest 2: Sending servo command: {cmd.decode()}")
        ser.write(cmd)
        time.sleep(0.5)
        if ser.in_waiting:
            print(f"  Response: {ser.read(ser.in_waiting)}")
        else:
            print("  No response")

        # Test 3: Try extreme position
        cmd = b"#000P2500T0500!"
        print(f"\nTest 3: Sending servo command: {cmd.decode()}")
        ser.write(cmd)
        time.sleep(1)
        if ser.in_waiting:
            print(f"  Response: {ser.read(ser.in_waiting)}")
        else:
            print("  No response")

        # Test 4: Back to center
        cmd = b"#000P1500T0500!"
        print(f"\nTest 4: Back to center: {cmd.decode()}")
        ser.write(cmd)
        time.sleep(1)
        if ser.in_waiting:
            print(f"  Response: {ser.read(ser.in_waiting)}")
        else:
            print("  No response")

        ser.close()
        print("\n✓ Test complete")

    except Exception as e:
        print(f"✗ Error: {e}")

    print("="*70 + "\n")

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB1'

    print("\nSIMPLE RAW SERVO TEST")
    print("Testing both USB1 and USB0 at multiple baud rates\n")

    # Test USB1 first (as user requested)
    for baud in [9600, 115200]:
        test_port('/dev/ttyUSB1', baud)
        time.sleep(1)

    # Also test USB0 (the CH340 servo board)
    print("\n" + "="*70)
    print("Now testing USB0 (CH340 - known servo board)")
    print("="*70 + "\n")
    for baud in [9600, 115200]:
        test_port('/dev/ttyUSB0', baud)
        time.sleep(1)

    print("\nDid ANY servos move during these tests?")
