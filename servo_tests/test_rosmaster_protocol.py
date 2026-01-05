#!/usr/bin/env python3
"""
Test the Rosmaster protocol extracted from Rosmaster_Lib
This is the EXACT protocol used by the working Rosmaster system
"""

import serial
import time
import struct

class RosmasterProtocolTest:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        """Initialize with Rosmaster protocol constants"""
        self.ser = serial.Serial(port, baudrate, timeout=0.5)

        # Rosmaster protocol constants
        self.HEAD = 0xFF
        self.DEVICE_ID = 0xFC
        self.COMPLEMENT = 257 - self.DEVICE_ID  # = 1

        # Function codes
        self.FUNC_PWM_SERVO = 0x03
        self.FUNC_PWM_SERVO_ALL = 0x04
        self.FUNC_BEEP = 0x02

        print("="*70)
        print("ROSMASTER PROTOCOL TEST")
        print("="*70)
        print(f"Port: {port}")
        print(f"Baud: {baudrate}")
        print(f"Protocol: Rosmaster_Lib v3.3.9")
        print("="*70)

    def send_command(self, cmd, description):
        """Send command and show details"""
        checksum = sum(cmd, self.COMPLEMENT) & 0xff
        cmd.append(checksum)

        print(f"\n{description}")
        print(f"  Command: {' '.join(f'{b:02X}' for b in cmd)}")
        print(f"  Sending...")

        self.ser.write(bytes(cmd))
        time.sleep(0.1)

        # Check for response
        if self.ser.in_waiting > 0:
            response = self.ser.read(self.ser.in_waiting)
            print(f"  Response: {' '.join(f'{b:02X}' for b in response)}")
        else:
            print(f"  Response: (none)")

    def test_beep(self, duration_ms=100):
        """Test beep function - confirms communication works"""
        on_time = bytearray(struct.pack('h', int(duration_ms)))
        cmd = [self.HEAD, self.DEVICE_ID, 0x05, self.FUNC_BEEP, on_time[0], on_time[1]]
        self.send_command(cmd, f"Beep test ({duration_ms}ms)")

    def test_pwm_servo(self, servo_id, angle):
        """Test single PWM servo control"""
        # Clamp angle
        if angle > 180:
            angle = 180
        elif angle < 0:
            angle = 0

        cmd = [self.HEAD, self.DEVICE_ID, 0x00, self.FUNC_PWM_SERVO, int(servo_id), int(angle)]
        cmd[2] = len(cmd) - 1  # Update length field
        self.send_command(cmd, f"PWM Servo {servo_id} to {angle}°")

    def test_pwm_servo_all(self, angle1=90, angle2=90, angle3=90, angle4=90):
        """Test all 4 PWM servos at once"""
        cmd = [self.HEAD, self.DEVICE_ID, 0x00, self.FUNC_PWM_SERVO_ALL,
               int(angle1), int(angle2), int(angle3), int(angle4)]
        cmd[2] = len(cmd) - 1
        self.send_command(cmd, f"All PWM Servos to [{angle1}, {angle2}, {angle3}, {angle4}]°")

    def run_test_sequence(self):
        """Run a comprehensive test sequence"""
        print("\n\n" + "="*70)
        print("STARTING TEST SEQUENCE")
        print("="*70)

        # Test 1: Beep (confirms board communication)
        print("\n--- Test 1: Communication Check (Beep) ---")
        self.test_beep(200)
        time.sleep(1)

        # Test 2: Single servo tests
        print("\n--- Test 2: Individual PWM Servo Control ---")

        for servo_id in range(1, 5):  # Test servos 1-4
            print(f"\nTesting Servo {servo_id}...")
            self.test_pwm_servo(servo_id, 90)  # Center
            time.sleep(1)
            self.test_pwm_servo(servo_id, 0)   # Min
            time.sleep(1)
            self.test_pwm_servo(servo_id, 180) # Max
            time.sleep(1)
            self.test_pwm_servo(servo_id, 90)  # Back to center
            time.sleep(1)

        # Test 3: All servos at once
        print("\n--- Test 3: All PWM Servos Simultaneously ---")
        self.test_pwm_servo_all(0, 0, 0, 0)
        time.sleep(2)
        self.test_pwm_servo_all(180, 180, 180, 180)
        time.sleep(2)
        self.test_pwm_servo_all(90, 90, 90, 90)
        time.sleep(1)

        print("\n\n" + "="*70)
        print("TEST SEQUENCE COMPLETE")
        print("="*70)
        print("\nDid you observe:")
        print("  - Beep sound from the board?")
        print("  - Any servo movement?")
        print("  - Which servos moved and to which commands?")
        print("="*70)

    def close(self):
        """Close serial connection"""
        if self.ser.is_open:
            self.ser.close()
            print("\nSerial port closed")

def main():
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print("\nIMPORTANT: This uses the Rosmaster protocol at 115200 baud!")
    print("This is the EXACT protocol from the working Rosmaster_Lib.\n")

    try:
        tester = RosmasterProtocolTest(port, baudrate)
        tester.run_test_sequence()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'tester' in locals():
            tester.close()

if __name__ == "__main__":
    main()
