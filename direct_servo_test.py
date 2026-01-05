#!/usr/bin/env python3
"""
Direct serial test - bypasses ROS to verify servo protocol works at hardware level.
This will help identify which servo channels are actually connected.
"""

import serial
import time

def test_all_channels(port, baud):
    """Test all servo channels from 0-23 to find which ones have servos"""
    print(f"\n{'='*70}")
    print(f"SCANNING ALL SERVO CHANNELS (0-23) at {baud} baud")
    print(f"{'='*70}")
    print("Watch your robot carefully - servos should move one at a time")
    print("Note which servos move and their channel numbers!")
    print(f"{'='*70}\n")

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"✓ Connected to {port} at {baud} baud\n")
        time.sleep(0.5)

        # Test each channel from 0 to 23
        for channel in range(24):
            print(f"Testing Channel {channel:02d}...", end='', flush=True)

            # Send center position command
            cmd = f"#{channel:03d}P1500T0500!"
            ser.write(cmd.encode('ascii'))
            print(f"  Sent: {cmd}", end='', flush=True)

            time.sleep(0.8)

            # Small movement to make it visible
            cmd = f"#{channel:03d}P1800T0300!"
            ser.write(cmd.encode('ascii'))
            time.sleep(0.5)

            cmd = f"#{channel:03d}P1200T0300!"
            ser.write(cmd.encode('ascii'))
            time.sleep(0.5)

            # Back to center
            cmd = f"#{channel:03d}P1500T0300!"
            ser.write(cmd.encode('ascii'))
            print("  ← Did a servo move? Note it!")
            time.sleep(0.8)

        print(f"\n{'='*70}")
        print("SCAN COMPLETE")
        print(f"{'='*70}\n")

        ser.close()

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

def test_single_servo(port, baud, channel):
    """Test a single servo channel with full range of motion"""
    print(f"\n{'='*70}")
    print(f"Testing Single Servo - Channel {channel} at {baud} baud")
    print(f"{'='*70}\n")

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"✓ Connected to {port}\n")
        time.sleep(0.5)

        positions = [
            (1500, "CENTER"),
            (500, "MIN (0°)"),
            (2500, "MAX (180°)"),
            (1500, "CENTER"),
        ]

        for pwm, name in positions:
            cmd = f"#{channel:03d}P{pwm:04d}T1000!"
            print(f"  {name:12s} → {cmd}")
            ser.write(cmd.encode('ascii'))
            time.sleep(1.5)

        print("\n✓ Test complete\n")
        ser.close()

    except Exception as e:
        print(f"\n✗ Error: {e}")

def main():
    import sys

    port = '/dev/ttyUSB0'

    print("\n" + "="*70)
    print("DIRECT SERVO HARDWARE TEST")
    print("="*70)
    print("\nThis bypasses ROS and sends commands directly to the servo board.")
    print("Protocol: #<index>P<position>T<time>!")
    print("\nOptions:")
    print("  1. Scan all channels (0-23) at 9600 baud")
    print("  2. Scan all channels (0-23) at 115200 baud")
    print("  3. Test single channel at 9600 baud")
    print("  4. Test single channel at 115200 baud")
    print("="*70 + "\n")

    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        choice = input("Enter choice (1-4): ").strip()

    if choice == '1':
        test_all_channels(port, 9600)
    elif choice == '2':
        test_all_channels(port, 115200)
    elif choice == '3':
        channel = int(input("Enter channel number (0-23): "))
        test_single_servo(port, 9600, channel)
    elif choice == '4':
        channel = int(input("Enter channel number (0-23): "))
        test_single_servo(port, 115200, channel)
    else:
        print("Invalid choice!")

if __name__ == '__main__':
    main()
