#!/usr/bin/env python3
import serial
import time

def test_port(port, baud=9600):
    """Test a servo command on the specified port"""
    try:
        print(f"\n{'='*50}")
        print(f"Testing {port} at {baud} baud")
        print(f"{'='*50}")

        ser = serial.Serial(port, baud, timeout=1.0)
        print(f"✓ Port opened successfully")
        time.sleep(0.5)

        # Test servo A at different angles
        for angle in [90, 45, 135, 90]:
            command = f'$A{angle:03d}#\n'
            print(f"Sending: {repr(command)}")
            ser.write(command.encode('utf-8'))
            ser.flush()
            time.sleep(1.5)  # Wait to observe movement

        ser.close()
        print(f"✓ Test complete\n")

        response = input(f"Did servo move on {port}? (y/n): ").strip().lower()
        return response == 'y'

    except Exception as e:
        print(f"✗ Error: {e}\n")
        return False

if __name__ == '__main__':
    print("Testing servo board on different ports...")
    print("Watch the servos carefully!\n")

    ports = ['/dev/ttyUSB0', '/dev/ttyUSB1']

    for port in ports:
        if test_port(port):
            print(f"\n{'*'*50}")
            print(f"FOUND IT! Servo board is on {port}")
            print(f"{'*'*50}\n")
            break
    else:
        print("\nServo did not respond on either port.")
        print("Trying 115200 baud rate on both ports...\n")

        for port in ports:
            if test_port(port, baud=115200):
                print(f"\n{'*'*50}")
                print(f"FOUND IT! Servo board is on {port} at 115200 baud")
                print(f"{'*'*50}\n")
                break
