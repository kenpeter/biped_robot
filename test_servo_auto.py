#!/usr/bin/env python3
"""
Automatic servo test with small movements - no user input required.
Tests servos with minimal movement to verify basic communication.
"""

import serial
import time
import sys

def test_servos_auto(port, baud, channels_to_test):
    """Test servos automatically with very small movements"""
    print(f"\n{'='*70}")
    print(f"AUTOMATIC SERVO TEST at {baud} baud")
    print(f"{'='*70}")
    print("Testing with SMALL movements to avoid wire tangling")
    print(f"Channels to test: {channels_to_test}")
    print(f"Watch the servos carefully!")
    print(f"{'='*70}\n")

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"✓ Connected to {port} at {baud} baud\n")
        time.sleep(0.5)

        for channel in channels_to_test:
            print(f"\n{'='*60}")
            print(f"Testing Channel {channel}")
            print(f"{'='*60}")

            # First, send center position
            cmd = f"#{channel:03d}P1500T0500!"
            print(f"  1. Center (90°): {cmd}")
            ser.write(cmd.encode('ascii'))
            time.sleep(1.0)

            # Small movement right (+10 degrees = ~111 PWM units)
            cmd = f"#{channel:03d}P1600T0300!"
            print(f"  2. Small right (+10°): {cmd}")
            ser.write(cmd.encode('ascii'))
            time.sleep(0.8)

            # Back to center
            cmd = f"#{channel:03d}P1500T0300!"
            print(f"  3. Back to center: {cmd}")
            ser.write(cmd.encode('ascii'))
            time.sleep(0.8)

            # Small movement left (-10 degrees)
            cmd = f"#{channel:03d}P1400T0300!"
            print(f"  4. Small left (-10°): {cmd}")
            ser.write(cmd.encode('ascii'))
            time.sleep(0.8)

            # Back to center
            cmd = f"#{channel:03d}P1500T0300!"
            print(f"  5. Back to center: {cmd}")
            ser.write(cmd.encode('ascii'))
            time.sleep(1.5)

            print(f"  → Watch servo {channel} - it should have made small movements")

        print(f"\n{'='*70}")
        print("TEST COMPLETE")
        print(f"{'='*70}")
        print("\nIf you saw servos move:")
        print("  ✓ Hardware communication is working!")
        print(f"  ✓ Baud rate {baud} is correct")
        print("  ✓ Protocol format is correct")
        print("\nIf NO servos moved:")
        print("  1. Check servo power supply")
        print("  2. Try different baud rate (9600 or 115200)")
        print("  3. Check servo connections to the board")
        print(f"{'='*70}\n")

        ser.close()

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

def main():
    port = '/dev/ttyUSB0'

    print("\n" + "="*70)
    print("AUTOMATIC SERVO TEST - No user input required")
    print("="*70)
    print("\nThis will test servos with small movements (±10 degrees)")
    print("to avoid wire tangling.")
    print("\nOptions:")
    print("  1. Test servos 17-18 at 115200 baud (RECOMMENDED)")
    print("  2. Test servos 17-18 at 9600 baud")
    print("  3. Test all enabled servos (0-7, 12-18) at 115200 baud")
    print("  4. Test all enabled servos (0-7, 12-18) at 9600 baud")
    print("="*70 + "\n")

    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        print("Usage: python3 test_servo_auto.py [1-4]")
        print("Example: python3 test_servo_auto.py 1")
        return

    if choice == '1':
        test_servos_auto(port, 115200, [17, 18])
    elif choice == '2':
        test_servos_auto(port, 9600, [17, 18])
    elif choice == '3':
        channels = list(range(0, 8)) + list(range(12, 19))  # 0-7, 12-18
        test_servos_auto(port, 115200, channels)
    elif choice == '4':
        channels = list(range(0, 8)) + list(range(12, 19))
        test_servos_auto(port, 9600, channels)
    else:
        print("Invalid choice! Use 1-4")

if __name__ == '__main__':
    main()
