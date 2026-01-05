#!/usr/bin/env python3
"""
Test Hiwonder LSC-style binary protocol
Protocol: 0x55 0x55 Length CMD Parameters
"""
import serial
import time

def move_servo_lsc(ser, servo_id, position, duration):
    """
    LSC protocol: Move single servo
    CMD_SERVO_MOVE = 0x03
    Format: 0x55 0x55 Length CMD Time_LSB Time_MSB ID Pos_LSB Pos_H SB
    """
    time_lsb = duration & 0xFF
    time_msb = (duration >> 8) & 0xFF
    pos_lsb = position & 0xFF
    pos_msb = (position >> 8) & 0xFF

    packet = bytearray([
        0x55, 0x55,  # Header
        0x08,        # Length (N+2, where N=6)
        0x03,        # CMD_SERVO_MOVE
        time_lsb, time_msb,
        servo_id,
        pos_lsb, pos_msb
    ])

    print(f"Sending LSC command: servo={servo_id}, pos={position}, time={duration}ms")
    print(f"  Packet: {' '.join(f'{b:02X}' for b in packet)}")
    ser.write(packet)
    ser.flush()

def main():
    print("="*60)
    print("Testing Hiwonder LSC Binary Protocol")
    print("="*60)

    port = '/dev/ttyUSB0'
    baud = 9600

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Opened {port} at {baud} baud\n")
        time.sleep(1)

        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Test servo 1 (first channel)
        # Servo positions: 500-2500 (pulse width in microseconds)
        print("Testing Servo ID 1 with LSC protocol:")
        positions = [1500, 500, 2500, 1500]  # Center, Min, Max, Center
        duration = 1000  # 1 second movement time

        for pos in positions:
            move_servo_lsc(ser, servo_id=1, position=pos, duration=duration)
            time.sleep(1.5)

            # Check for response
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                print(f"  Response: {' '.join(f'{b:02X}' for b in response)}\n")

        print("\nTest complete. Did servo 1 move?")

        # Try a few more servos
        print("\nTesting servos 2, 3, 4...")
        for servo_id in [2, 3, 4]:
            print(f"\nServo {servo_id}:")
            for pos in [1500, 1000, 2000, 1500]:
                move_servo_lsc(ser, servo_id, pos, 1000)
                time.sleep(1.0)

        ser.close()
        print("\nAll tests complete!")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()
