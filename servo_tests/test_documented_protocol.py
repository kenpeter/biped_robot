#!/usr/bin/env python3
"""
Test servo board using the ACTUAL documented protocol from board manual
Protocol: #<index>P<position>T<time>!
"""

import serial
import time

class ServoController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        """Initialize with documented protocol"""
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5
        )

        print("="*70)
        print("SERVO BOARD - DOCUMENTED PROTOCOL TEST")
        print("="*70)
        print(f"Port: {port}")
        print(f"Baud: {baudrate}")
        print("Protocol: #<index>P<position>T<time>!")
        print("  index: 0-254 (servo ID)")
        print("  position: 500-2500 (pulse width μs)")
        print("  time: 0-9999 (movement time ms)")
        print("="*70)

        # Small delay after opening port
        time.sleep(0.1)

    def send_command(self, cmd_str, description):
        """Send ASCII command and show details"""
        print(f"\n{description}")
        print(f"  Command: {cmd_str}")
        print(f"  Hex: {' '.join(f'{ord(c):02X}' for c in cmd_str)}")

        self.ser.write(cmd_str.encode('ascii'))
        time.sleep(0.1)

        # Check for response
        if self.ser.in_waiting > 0:
            response = self.ser.read(self.ser.in_waiting)
            print(f"  Response: {response}")
            print(f"  Response (hex): {' '.join(f'{b:02X}' for b in response)}")
        else:
            print(f"  Response: (none)")

    def move_servo(self, servo_id, position, move_time=1000):
        """
        Move single servo
        servo_id: 0-254
        position: 500-2500 (microseconds)
        move_time: 0-9999 (milliseconds)
        """
        # Clamp values
        servo_id = max(0, min(254, servo_id))
        position = max(500, min(2500, position))
        move_time = max(0, min(9999, move_time))

        cmd = f"#{servo_id:03d}P{position:04d}T{move_time:04d}!"
        self.send_command(cmd, f"Move servo {servo_id} to {position}μs in {move_time}ms")

    def move_multiple_servos(self, servo_commands):
        """
        Move multiple servos at once
        servo_commands: list of (id, position, time) tuples
        """
        cmd_parts = []
        for servo_id, position, move_time in servo_commands:
            servo_id = max(0, min(254, servo_id))
            position = max(500, min(2500, position))
            move_time = max(0, min(9999, move_time))
            cmd_parts.append(f"#{servo_id:03d}P{position:04d}T{move_time:04d}")

        cmd = "{" + "".join(cmd_parts) + "}"
        self.send_command(cmd, f"Move {len(servo_commands)} servos simultaneously")

    def reset_board(self):
        """Software reset the board"""
        self.send_command("$RSTI", "Software reset board")

    def run_test_sequence(self):
        """Run comprehensive test sequence"""
        print("\n\n" + "="*70)
        print("STARTING TEST SEQUENCE")
        print("="*70)

        # Test 1: Reset board
        print("\n--- Test 1: Board Reset ---")
        self.reset_board()
        time.sleep(2)

        # Test 2: Individual servo tests (servos 0-3)
        print("\n--- Test 2: Individual Servo Tests ---")

        for servo_id in range(4):
            print(f"\n>>> Testing Servo {servo_id} <<<")

            # Center position
            self.move_servo(servo_id, 1500, 1000)
            time.sleep(1.5)

            # Min position
            self.move_servo(servo_id, 500, 1000)
            time.sleep(1.5)

            # Max position
            self.move_servo(servo_id, 2500, 1000)
            time.sleep(1.5)

            # Back to center
            self.move_servo(servo_id, 1500, 1000)
            time.sleep(1.5)

        # Test 3: Multiple servos at once
        print("\n--- Test 3: Multiple Servos Simultaneously ---")

        print("\nAll servos to center...")
        self.move_multiple_servos([
            (0, 1500, 1000),
            (1, 1500, 1000),
            (2, 1500, 1000),
            (3, 1500, 1000)
        ])
        time.sleep(2)

        print("\nAll servos to min...")
        self.move_multiple_servos([
            (0, 500, 1000),
            (1, 500, 1000),
            (2, 500, 1000),
            (3, 500, 1000)
        ])
        time.sleep(2)

        print("\nAll servos to max...")
        self.move_multiple_servos([
            (0, 2500, 1000),
            (1, 2500, 1000),
            (2, 2500, 1000),
            (3, 2500, 1000)
        ])
        time.sleep(2)

        print("\nAll servos back to center...")
        self.move_multiple_servos([
            (0, 1500, 1000),
            (1, 1500, 1000),
            (2, 1500, 1000),
            (3, 1500, 1000)
        ])
        time.sleep(2)

        # Test 4: Fast movement
        print("\n--- Test 4: Fast Movement Test ---")
        print("\nRapid sweep on servo 0...")
        for pos in [500, 1000, 1500, 2000, 2500, 1500]:
            self.move_servo(0, pos, 300)
            time.sleep(0.5)

        print("\n\n" + "="*70)
        print("TEST SEQUENCE COMPLETE")
        print("="*70)
        print("\nDid you observe:")
        print("  - Any servo movement?")
        print("  - Which servos moved?")
        print("  - Smooth movement or jerky?")
        print("  - Any responses from the board?")
        print("="*70)

    def close(self):
        """Close serial connection"""
        if self.ser.is_open:
            self.ser.close()
            print("\nSerial port closed")

def main():
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 9600

    print("\n" + "="*70)
    print("SERVO BOARD CONTROL - DOCUMENTED PROTOCOL")
    print("="*70)
    print("\nThis uses the ACTUAL protocol from the board documentation:")
    print("  Format: #<index>P<position>T<time>!")
    print("  Example: #000P1500T1000! (servo 0 to center in 1 second)")
    print("\nDefault baud rate: 9600")
    print("(If no movement, try: python3 script.py /dev/ttyUSB0 115200)")
    print("="*70 + "\n")

    try:
        controller = ServoController(port, baudrate)
        controller.run_test_sequence()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'controller' in locals():
            controller.close()

if __name__ == "__main__":
    main()
