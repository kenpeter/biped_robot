#!/usr/bin/env python3
"""
Comprehensive servo protocol test - tries different command formats
"""
import serial
import time

def test_command_format(ser, servo='A', angle=90, format_name="", command_template=""):
    """Send a servo command and wait for response"""
    command = command_template.format(servo=servo, angle=angle)
    print(f"  Format: {format_name:20s} Command: {repr(command):20s}", end='')
    ser.write(command.encode('utf-8'))
    ser.flush()
    time.sleep(0.3)
    print(" sent")

def main():
    # Determine which port to use
    print("Which port do you want to test?")
    print("1. /dev/ttyUSB0 (CH340)")
    print("2. /dev/ttyUSB1 (FTDI)")
    choice = input("Enter 1 or 2: ").strip()

    port = '/dev/ttyUSB0' if choice == '1' else '/dev/ttyUSB1'
    baud = 9600

    print(f"\nTesting on {port} at {baud} baud")
    print("Watch servo A (head pan) carefully!\n")

    # Different command formats to try
    formats = [
        ("Doc format + \\n",       "${servo}{angle:03d}#\n"),
        ("Doc format + \\r\\n",    "${servo}{angle:03d}#\r\n"),
        ("Doc format only",        "${servo}{angle:03d}#"),
        ("No $ prefix + \\n",      "{servo}{angle:03d}#\n"),
        ("No # suffix + \\n",      "${servo}{angle:03d}\n"),
        ("Decimal + \\n",          "{servo}{angle}#\n"),
        ("Hex angle + \\n",        "${servo}{angle:02X}#\n"),
        ("Space separated + \\n",  "{servo} {angle}\\n"),
    ]

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
        print(f"✓ Opened {port}\n")
        time.sleep(1.0)

        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Test each format with a simple movement sequence
        test_angles = [90, 60, 120, 90]  # Center, left, right, center

        for format_name, cmd_template in formats:
            print(f"\n{'='*60}")
            print(f"Testing: {format_name}")
            print(f"{'='*60}")

            for angle in test_angles:
                test_command_format(ser, 'A', angle, format_name, cmd_template)
                time.sleep(1.0)  # Wait between commands

            response = input("\nDid servo move? (y/n): ").strip().lower()
            if response == 'y':
                print(f"\n{'*'*60}")
                print(f"SUCCESS! Working format found:")
                print(f"  Port: {port}")
                print(f"  Baud: {baud}")
                print(f"  Format: {format_name}")
                print(f"  Template: {repr(cmd_template)}")
                print(f"{'*'*60}\n")
                ser.close()
                return

        print("\n" + "="*60)
        print("Trying with 115200 baud rate...")
        print("="*60)
        ser.close()

        # Try again with higher baud rate
        ser = serial.Serial(port, 115200, timeout=1.0)
        time.sleep(1.0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Test most common format at higher baud
        cmd_template = "${servo}{angle:03d}#\n"
        print(f"\nTesting standard format at 115200 baud")
        for angle in [90, 45, 135, 90]:
            test_command_format(ser, 'A', angle, "Standard", cmd_template)
            time.sleep(1.0)

        response = input("\nDid servo move at 115200? (y/n): ").strip().lower()
        if response == 'y':
            print(f"\n{'*'*60}")
            print(f"SUCCESS! Port: {port}, Baud: 115200")
            print(f"{'*'*60}\n")

        ser.close()

    except Exception as e:
        print(f"✗ Error: {e}")

    print("\nTest complete.")

if __name__ == '__main__':
    main()
