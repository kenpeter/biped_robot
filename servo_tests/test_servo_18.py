#!/usr/bin/env python3
"""
Test servo channel 18 specifically (and nearby channels)
"""

import serial
import time

def test_servo_channel(port, baud, channel):
    """Test a specific servo channel"""
    print(f"\n{'='*70}")
    print(f"Testing Servo Channel {channel} at {baud} baud")
    print(f"{'='*70}")

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"✓ Connected to {port}")
        time.sleep(0.5)

        positions = [1500, 2000, 1000, 1500]  # center, right, left, center
        position_names = ["CENTER (1500μs)", "RIGHT (2000μs)", "LEFT (1000μs)", "CENTER (1500μs)"]

        for pos, name in zip(positions, position_names):
            cmd = f"#{channel:03d}P{pos:04d}T1000!"
            print(f"\n  Sending: {cmd} ({name})")
            ser.write(cmd.encode('ascii'))
            time.sleep(1.5)

            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                print(f"  Response: {len(response)} bytes")
            else:
                print(f"  Response: (none)")

        ser.close()
        print(f"\n✓ Test complete for channel {channel}")

    except Exception as e:
        print(f"✗ Error: {e}")

def main():
    port = '/dev/ttyUSB0'

    print("="*70)
    print("SERVO CHANNEL 18 TEST")
    print("="*70)
    print("\nTesting servo channel 18 and nearby channels")
    print("(Since you said servo 18 moves during boot)")
    print()

    # Test at 9600 baud first
    print("\n" + "="*70)
    print("TESTING AT 9600 BAUD")
    print("="*70)

    # Test channel 18 specifically
    test_servo_channel(port, 9600, 18)
    time.sleep(1)

    # Also test nearby channels (in case it's 17, 19, etc.)
    for channel in [17, 19, 16, 20]:
        test_servo_channel(port, 9600, channel)
        time.sleep(1)

    # Test at 115200 baud
    print("\n" + "="*70)
    print("TESTING AT 115200 BAUD")
    print("="*70)

    test_servo_channel(port, 115200, 18)
    time.sleep(1)

    # Test all 15 servos (if connected to sequential channels)
    print("\n" + "="*70)
    print("TESTING ALL 15 CHANNELS (0-14) AT 9600 BAUD")
    print("="*70)
    print("Quick test of channels 0-14...")

    try:
        ser = serial.Serial(port, 9600, timeout=1)
        time.sleep(0.5)

        for channel in range(15):
            cmd = f"#{channel:03d}P1500T0500!"
            print(f"  Ch {channel:2d}: {cmd}")
            ser.write(cmd.encode('ascii'))
            time.sleep(0.3)

        time.sleep(1)

        # Now sweep them
        print("\nSweeping all channels...")
        for pos in [2000, 1000, 1500]:
            for channel in range(15):
                cmd = f"#{channel:03d}P{pos:04d}T0300!"
                ser.write(cmd.encode('ascii'))
            time.sleep(0.5)

        ser.close()

    except Exception as e:
        print(f"Error: {e}")

    print("\n" + "="*70)
    print("TEST COMPLETE")
    print("="*70)
    print("\nDid you see ANY servo movement?")
    print("If yes, which test made it move?")
    print("="*70)

if __name__ == "__main__":
    main()
