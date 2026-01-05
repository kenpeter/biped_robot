#!/usr/bin/env python3
"""
Gentle servo test with small movements to avoid wire tangling.
Tests servos with minimal movement to verify basic communication.
"""

import serial
import time
import sys

def test_gentle_movement(port, baud, channels_to_test):
    """Test servos with very small movements"""
    print(f"\n{'='*70}")
    print(f"GENTLE SERVO TEST at {baud} baud")
    print(f"{'='*70}")
    print("Testing with SMALL movements to avoid wire tangling")
    print(f"Channels to test: {channels_to_test}")
    print(f"{'='*70}\n")

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"✓ Connected to {port} at {baud} baud\n")
        time.sleep(0.5)

        for channel in channels_to_test:
            print(f"\n--- Testing Channel {channel} ---")

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
            time.sleep(0.8)

            response = input(f"  → Did servo {channel} move? (y/n/skip all): ").lower()
            if response == 'skip all':
                break
            elif response == 'y':
                print(f"  ✓ Servo {channel} is working!")
            else:
                print(f"  ✗ No movement detected on channel {channel}")

        print(f"\n{'='*70}")
        print("TEST COMPLETE")
        print(f"{'='*70}\n")
        ser.close()

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

def main():
    port = '/dev/ttyUSB0'

    print("\n" + "="*70)
    print("GENTLE SERVO TEST - Small movements to avoid wire tangling")
    print("="*70)
    print("\nBased on your screenshot, the PC software controls servos.")
    print("We'll test with VERY SMALL movements first (±10 degrees).")
    print("\nWhich channels should we test?")
    print("  - From memo: servo 17/18 were mentioned")
    print("  - From config: channels 4-11, 12-18 are configured")
    print("\nOptions:")
    print("  1. Test channels 17-18 only (from memo) at 115200 baud")
    print("  2. Test channels 17-18 only (from memo) at 9600 baud")
    print("  3. Test all configured channels (4-18) at 115200 baud")
    print("  4. Test all configured channels (4-18) at 9600 baud")
    print("  5. Custom channel list at 115200 baud")
    print("  6. Custom channel list at 9600 baud")
    print("="*70 + "\n")

    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        choice = input("Enter choice (1-6): ").strip()

    if choice == '1':
        test_gentle_movement(port, 115200, [17, 18])
    elif choice == '2':
        test_gentle_movement(port, 9600, [17, 18])
    elif choice == '3':
        channels = list(range(4, 19))  # 4-18 inclusive
        test_gentle_movement(port, 115200, channels)
    elif choice == '4':
        channels = list(range(4, 19))
        test_gentle_movement(port, 9600, channels)
    elif choice == '5':
        channels_str = input("Enter channel numbers (comma-separated, e.g., '17,18'): ")
        channels = [int(x.strip()) for x in channels_str.split(',')]
        test_gentle_movement(port, 115200, channels)
    elif choice == '6':
        channels_str = input("Enter channel numbers (comma-separated, e.g., '17,18'): ")
        channels = [int(x.strip()) for x in channels_str.split(',')]
        test_gentle_movement(port, 9600, channels)
    else:
        print("Invalid choice!")

if __name__ == '__main__':
    main()
