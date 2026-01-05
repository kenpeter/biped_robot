#!/usr/bin/env python3
"""
Test servo communication and listen for responses from the board.
Try different protocol variations based on PC software config.
"""

import serial
import time
import sys

def test_with_response(port, baud):
    """Send commands and listen for any response"""
    print(f"\n{'='*70}")
    print(f"SERVO TEST WITH RESPONSE LISTENING at {baud} baud")
    print(f"{'='*70}\n")

    try:
        ser = serial.Serial(port, baud, timeout=0.5)
        print(f"✓ Connected to {port} at {baud} baud\n")

        # Clear any existing data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.5)

        # Test different protocol variations
        test_commands = [
            # Format 1: Single servo (from memo)
            ("#017P1500T0500!", "Single servo - standard format"),

            # Format 2: With braces (from config file)
            ("{#017P1500T0500!}", "Single servo - with braces"),

            # Format 3: Multiple servos in braces
            ("{#017P1500T0500!#018P1500T0500!}", "Multiple servos - with braces"),

            # Format 4: Group format (from config)
            ("{G0000#017P1500T0500!}", "Group format"),

            # Format 5: Try servo 0 instead (might be more commonly connected)
            ("#000P1500T0500!", "Servo 0 - standard format"),

            # Format 6: Servo 0 with braces
            ("{#000P1500T0500!}", "Servo 0 - with braces"),
        ]

        for cmd, description in test_commands:
            print(f"\n{'─'*60}")
            print(f"Test: {description}")
            print(f"Command: {cmd}")
            print(f"{'─'*60}")

            # Send command
            sent_bytes = ser.write(cmd.encode('ascii'))
            ser.flush()
            print(f"  → Sent {sent_bytes} bytes")

            # Wait and listen for response
            time.sleep(0.3)

            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                print(f"  ← Response ({len(response)} bytes): {response}")
                try:
                    print(f"  ← Decoded: {response.decode('ascii', errors='replace')}")
                except:
                    print(f"  ← Hex: {response.hex()}")
            else:
                print(f"  ← No response from board")

            # Small movement to make it visible
            if "017" in cmd or "000" in cmd:
                time.sleep(0.5)
                move_cmd = cmd.replace("P1500", "P1600")
                ser.write(move_cmd.encode('ascii'))
                ser.flush()
                time.sleep(0.3)

                # Back to center
                ser.write(cmd.encode('ascii'))
                ser.flush()
                time.sleep(0.5)

            print(f"  ⚠ Did you see any servo move?")

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
    print("SERVO PROTOCOL DIAGNOSTIC TEST")
    print("="*70)
    print("\nThis will try different protocol formats and listen for responses")
    print("\nOptions:")
    print("  1. Test at 115200 baud")
    print("  2. Test at 9600 baud")
    print("="*70 + "\n")

    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        print("Usage: python3 test_servo_listen.py [1-2]")
        print("Example: python3 test_servo_listen.py 1")
        return

    if choice == '1':
        test_with_response(port, 115200)
    elif choice == '2':
        test_with_response(port, 9600)
    else:
        print("Invalid choice!")

if __name__ == '__main__':
    main()
