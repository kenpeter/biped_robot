#!/usr/bin/env python3
"""
Test using exact format from PC software (Zide Config)
"""

import serial
import time

def test_pc_format(port, baud):
    """Test with exact PC software format from ZideConfig.ini"""
    print(f"\n{'='*70}")
    print(f"TESTING PC SOFTWARE FORMAT at {baud} baud")
    print(f"{'='*70}\n")

    try:
        ser = serial.Serial(port, baud, timeout=0.5)
        print(f"✓ Connected to {port} at {baud} baud\n")

        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.5)

        # Test 1: From config - Single servo in group format
        print(f"{'─'*60}")
        print("Test 1: Single servo - PC group format")
        cmd = "{G0000#017P1500T0500!}"
        print(f"Command: {cmd}")
        print(f"{'─'*60}")
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(1.0)

        # Move it slightly
        cmd = "{G0000#017P1600T0300!}"
        print(f"Moving right: {cmd}")
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(0.8)

        # Back to center
        cmd = "{G0000#017P1500T0300!}"
        print(f"Back to center: {cmd}")
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(1.0)
        print("  → Did servo 17 move?\n")

        # Test 2: Multiple servos like PC does
        print(f"{'─'*60}")
        print("Test 2: Multiple servos 17-18 - PC format")
        cmd = "{#017P1500T0500!#018P1500T0500!}"
        print(f"Command: {cmd}")
        print(f"{'─'*60}")
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(1.0)

        # Move both
        cmd = "{#017P1600T0300!#018P1600T0300!}"
        print(f"Moving both right: {cmd}")
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(0.8)

        # Back
        cmd = "{#017P1500T0300!#018P1500T0300!}"
        print(f"Back to center: {cmd}")
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(1.0)
        print("  → Did servos 17 or 18 move?\n")

        # Test 3: Try servo 0 (often the first connected)
        print(f"{'─'*60}")
        print("Test 3: Servo 0 - PC format")
        cmd = "{#000P1500T0500!}"
        print(f"Command: {cmd}")
        print(f"{'─'*60}")
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(1.0)

        cmd = "{#000P1600T0300!}"
        print(f"Moving: {cmd}")
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(0.8)

        cmd = "{#000P1400T0300!}"
        print(f"Moving: {cmd}")
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(0.8)

        cmd = "{#000P1500T0300!}"
        print(f"Center: {cmd}")
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(1.0)
        print("  → Did servo 0 move?\n")

        # Test 4: All enabled servos to center (from config: 0-7, 12-18)
        print(f"{'─'*60}")
        print("Test 4: ALL enabled servos to center")
        servos = list(range(0, 8)) + list(range(12, 19))
        cmd_parts = [f"#{ s:03d}P1500T1000!" for s in servos]
        cmd = "{" + "".join(cmd_parts) + "}"
        print(f"Command: {cmd[:80]}...")  # Truncate for display
        print(f"{'─'*60}")
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(2.0)
        print("  → Did ANY servos move to center?\n")

        print(f"{'='*70}")
        print("TEST COMPLETE")
        print(f"{'='*70}\n")

        ser.close()

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

def main():
    import sys
    port = '/dev/ttyUSB0'

    print("\n" + "="*70)
    print("PC SOFTWARE FORMAT TEST")
    print("="*70)
    print("\nTesting with exact format from ZideConfig.ini")
    print("\n1. Test at 115200 baud")
    print("2. Test at 9600 baud")
    print("="*70 + "\n")

    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        print("Usage: python3 test_pc_format.py [1-2]")
        return

    if choice == '1':
        test_pc_format(port, 115200)
    elif choice == '2':
        test_pc_format(port, 9600)
    else:
        print("Invalid choice!")

if __name__ == '__main__':
    main()
