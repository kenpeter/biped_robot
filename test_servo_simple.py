#!/usr/bin/env python3
"""
Simplest possible servo test - try different variations
"""
import serial
import time

def test_variations(port, baud):
    """Test different command variations to see what works"""

    print(f"\n{'='*70}")
    print(f"TESTING COMMAND VARIATIONS at {baud} baud")
    print(f"{'='*70}\n")

    ser = serial.Serial(port, baud, timeout=1)
    print(f"✓ Connected to {port}\n")
    time.sleep(1)

    # Test channel 10 (middle of your range)
    channel = 10

    variations = [
        (f"#{channel:03d}P1500T1000!", "Standard format"),
        (f"#{channel:03d}P1500T1000!\r\n", "With CRLF"),
        (f"#{channel:03d}P1500T1000!\n", "With LF"),
        (f"#{channel:03d}P1500T1000!\r", "With CR"),
        (f"{{#{channel:03d}P1500T1000!}}", "Wrapped in braces"),
        (f"#010P1500T1000!", "Channel 10 no padding"),
    ]

    for i, (cmd, desc) in enumerate(variations):
        print(f"\nTest {i+1}: {desc}")
        print(f"  Sending: {repr(cmd)}")
        ser.write(cmd.encode('ascii'))
        print("  Waiting 2 seconds... Watch for movement!")
        time.sleep(2)

    print(f"\n{'='*70}")
    print("Now testing multiple channels in sequence...")
    print(f"{'='*70}\n")

    # Test your reported channels: 4-11, 12-18
    test_channels = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]

    for ch in test_channels:
        cmd = f"#{ch:03d}P1500T0500!"
        print(f"Channel {ch:02d}: {cmd}", end='', flush=True)
        ser.write(cmd.encode('ascii'))
        time.sleep(0.5)

        # Move it
        cmd = f"#{ch:03d}P2000T0300!"
        ser.write(cmd.encode('ascii'))
        time.sleep(0.5)

        cmd = f"#{ch:03d}P1000T0300!"
        ser.write(cmd.encode('ascii'))
        time.sleep(0.5)

        cmd = f"#{ch:03d}P1500T0300!"
        ser.write(cmd.encode('ascii'))
        print("  ✓")
        time.sleep(0.3)

    ser.close()
    print(f"\n{'='*70}")
    print("Test complete!")
    print(f"{'='*70}\n")

if __name__ == '__main__':
    import sys

    port = '/dev/ttyUSB0'

    print("\nChoose baud rate:")
    print("  1. 9600")
    print("  2. 115200")

    choice = input("\nEnter choice (1 or 2): ").strip() if len(sys.argv) < 2 else sys.argv[1]

    baud = 9600 if choice == '1' else 115200

    try:
        test_variations(port, baud)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
