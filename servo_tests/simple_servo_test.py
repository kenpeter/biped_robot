#!/usr/bin/env python3
"""
Simple servo test - tests the most likely configurations quickly
"""
import serial
import time

def test_config(port, baud, use_newline):
    """Test servo on specific port/baud"""
    print(f"\n{'='*60}")
    print(f"Testing: {port} @ {baud} baud, newline={use_newline}")
    print(f"{'='*60}")

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
        print("✓ Port opened")
        time.sleep(1.0)

        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Test servos A through E with clear movements
        servos_to_test = ['A', 'B', 'C', 'D', 'E']

        for servo in servos_to_test:
            print(f"\nTesting servo {servo}:")
            angles = [90, 0, 180, 90]  # Center, min, max, center

            for angle in angles:
                if use_newline:
                    cmd = f'${servo}{angle:03d}#\n'
                else:
                    cmd = f'${servo}{angle:03d}#'

                print(f"  Sending: {repr(cmd)} ({angle}°)")
                ser.write(cmd.encode('utf-8'))
                ser.flush()
                time.sleep(1.0)  # Wait to see movement

        ser.close()
        print("\n✓ Test complete")
        return True

    except Exception as e:
        print(f"\n✗ Error: {e}")
        return False

def main():
    print("="*60)
    print("QUICK SERVO TEST")
    print("="*60)
    print("\nThis will test the most likely configurations:")
    print("1. USB0 @ 9600 baud (no newline)")
    print("2. USB0 @ 9600 baud (with newline)")
    print("3. USB1 @ 9600 baud (no newline)")
    print("4. USB1 @ 9600 baud (with newline)")
    print("\nWatch servos A, B, C, D, E carefully!")
    print("="*60)

    input("\nPress Enter to start...")

    # Test most likely configurations first
    configs = [
        ('/dev/ttyUSB0', 9600, False),  # Documented config
        ('/dev/ttyUSB0', 9600, True),   # With newline
        ('/dev/ttyUSB1', 9600, False),  # Memo mentioned USB1
        ('/dev/ttyUSB1', 9600, True),   # USB1 with newline
    ]

    for port, baud, newline in configs:
        test_config(port, baud, newline)

        response = input("\nDid ANY servo move? (y/n): ").strip().lower()
        if response == 'y':
            print(f"\n{'*'*60}")
            print("SUCCESS! Found working configuration:")
            print(f"  Port: {port}")
            print(f"  Baud: {baud}")
            print(f"  Newline: {newline}")
            print(f"{'*'*60}\n")

            # Save it
            with open('/home/jetson/WORKING_SERVO_CONFIG.txt', 'w') as f:
                f.write(f"Port: {port}\n")
                f.write(f"Baud: {baud}\n")
                f.write(f"Newline: {newline}\n")

            print("Saved to: /home/jetson/WORKING_SERVO_CONFIG.txt\n")
            return

    print("\n" + "="*60)
    print("None of the quick tests worked.")
    print("Run: python3 /home/jetson/auto_servo_test.py")
    print("for exhaustive testing.")
    print("="*60)

if __name__ == '__main__':
    main()
