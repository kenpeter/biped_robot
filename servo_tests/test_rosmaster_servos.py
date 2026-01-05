#!/usr/bin/env python3
"""
Test using Rosmaster protocol on servo channels
Based on the Rosmaster_Lib we found on the system
"""

import serial
import time
import struct

def test_uart_servo_rosmaster(port='/dev/ttyUSB0', baudrate=115200):
    """Test UART servo control using Rosmaster protocol"""

    HEAD = 0xFF
    DEVICE_ID = 0xFC
    COMPLEMENT = 257 - DEVICE_ID  # = 1
    FUNC_UART_SERVO = 0x20

    print("="*70)
    print("ROSMASTER UART SERVO TEST")
    print("="*70)
    print(f"Port: {port}")
    print(f"Baud: {baudrate}")
    print("Protocol: Rosmaster UART Servo (0xFF 0xFC ...)")
    print("="*70)

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print("✓ Port opened")
        time.sleep(0.5)

        # Test servo IDs that might be connected (based on 15 servos)
        test_servos = [1, 2, 3, 4, 5, 6, 18]

        for servo_id in test_servos:
            print(f"\n--- Testing Servo ID {servo_id} ---")

            positions = [1500, 2000, 1000, 1500]
            for pos in positions:
                # Build command: [HEAD, DEVICE_ID, length, FUNC, servo_id, pos_low, pos_high, time_low, time_high]
                value = struct.pack('H', pos)  # Convert to 2 bytes (little-endian)
                run_time = struct.pack('H', 1000)  # 1000ms

                cmd = [HEAD, DEVICE_ID, 0x00, FUNC_UART_SERVO,
                       servo_id, value[0], value[1], run_time[0], run_time[1]]
                cmd[2] = len(cmd) - 1  # Update length

                # Calculate checksum
                checksum = (sum(cmd) + COMPLEMENT) & 0xFF
                cmd.append(checksum)

                print(f"  Pos {pos:4d}μs: {' '.join(f'{b:02X}' for b in cmd)}")
                ser.write(bytes(cmd))
                time.sleep(0.8)

                # Check response
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting)
                    print(f"    Response: {len(response)} bytes")

        ser.close()
        print("\n✓ Test complete")

    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("\nThis test uses the Rosmaster UART servo protocol")
    print("(the protocol used by the Rosmaster_Lib installed on your system)\n")

    test_uart_servo_rosmaster()

    print("\n" + "="*70)
    print("Did any servos move?")
    print("="*70)
