#!/usr/bin/env python3
"""
Manual servo test - sends direct serial commands to servos.
Run this after reconnecting USB Type-C cable.
"""
import serial
import time

def test_servos():
    print("Opening serial connection to /dev/ttyUSB0...")
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    time.sleep(1)

    print("\n=== TESTING SERVO A (HEAD) ===")
    print("Moving to 45 degrees...")
    ser.write(b'$A045#')
    ser.flush()
    time.sleep(1)

    print("Moving to 135 degrees...")
    ser.write(b'$A135#')
    ser.flush()
    time.sleep(1)

    print("Moving to center (90 degrees)...")
    ser.write(b'$A090#')
    ser.flush()
    time.sleep(1)

    print("\n=== TESTING SERVO B (LEFT SHOULDER) ===")
    print("Moving to 45 degrees...")
    ser.write(b'$B045#')
    ser.flush()
    time.sleep(1)

    print("Moving to 135 degrees...")
    ser.write(b'$B135#')
    ser.flush()
    time.sleep(1)

    print("Moving to center (90 degrees)...")
    ser.write(b'$B090#')
    ser.flush()
    time.sleep(1)

    print("\n=== TESTING ALL SERVOS - WAVE PATTERN ===")
    servos = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O']

    for servo in servos:
        print(f"Servo {servo}: 60 deg -> 90 deg -> 120 deg")
        ser.write(f'${servo}060#'.encode())
        ser.flush()
        time.sleep(0.3)

        ser.write(f'${servo}090#'.encode())
        ser.flush()
        time.sleep(0.3)

        ser.write(f'${servo}120#'.encode())
        ser.flush()
        time.sleep(0.3)

        ser.write(f'${servo}090#'.encode())
        ser.flush()
        time.sleep(0.2)

    print("\n✓ Test complete!")
    ser.close()

if __name__ == '__main__':
    try:
        test_servos()
    except KeyboardInterrupt:
        print("\nTest interrupted")
    except Exception as e:
        print(f"\nError: {e}")
