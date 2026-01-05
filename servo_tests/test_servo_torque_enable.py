#!/usr/bin/env python3
"""
Enable servo torque and then test movement
Based on Rosmaster_Lib set_uart_servo_torque function
"""

import serial
import time
import struct

def send_rosmaster_cmd(ser, cmd):
    """Send command and show details"""
    print(f"  TX: {' '.join(f'{b:02X}' for b in cmd)}")
    ser.write(bytes(cmd))
    time.sleep(0.2)

    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"  RX: {len(response)} bytes")
        return response
    return None

def enable_servo_torque(ser):
    """Enable servo torque using Rosmaster protocol"""
    HEAD = 0xFF
    DEVICE_ID = 0xFC
    COMPLEMENT = 1
    FUNC_UART_SERVO_TORQUE = 0x22

    print("\n>>> Enabling Servo Torque <<<")
    cmd = [HEAD, DEVICE_ID, 0x04, FUNC_UART_SERVO_TORQUE, 1]  # 1 = enable
    checksum = (sum(cmd) + COMPLEMENT) & 0xFF
    cmd.append(checksum)

    send_rosmaster_cmd(ser, cmd)

def disable_servo_torque(ser):
    """Disable servo torque"""
    HEAD = 0xFF
    DEVICE_ID = 0xFC
    COMPLEMENT = 1
    FUNC_UART_SERVO_TORQUE = 0x22

    print("\n>>> Disabling Servo Torque <<<")
    cmd = [HEAD, DEVICE_ID, 0x04, FUNC_UART_SERVO_TORQUE, 0]  # 0 = disable
    checksum = (sum(cmd) + COMPLEMENT) & 0xFF
    cmd.append(checksum)

    send_rosmaster_cmd(ser, cmd)

def move_servo(ser, servo_id, position, move_time=1000):
    """Move a servo"""
    HEAD = 0xFF
    DEVICE_ID = 0xFC
    COMPLEMENT = 1
    FUNC_UART_SERVO = 0x20

    value = struct.pack('H', position)
    run_time = struct.pack('H', move_time)

    cmd = [HEAD, DEVICE_ID, 0x08, FUNC_UART_SERVO,
           servo_id, value[0], value[1], run_time[0], run_time[1]]
    checksum = (sum(cmd) + COMPLEMENT) & 0xFF
    cmd.append(checksum)

    print(f"\n  Moving servo {servo_id} to {position}μs")
    send_rosmaster_cmd(ser, cmd)

def test_with_torque_enable():
    """Test servos with torque enable"""
    port = '/dev/ttyUSB0'
    baudrate = 115200

    print("="*70)
    print("SERVO TORQUE ENABLE TEST")
    print("="*70)
    print("This will:")
    print("1. Enable servo torque (unlock servos)")
    print("2. Try to move servos")
    print("3. Disable torque at the end")
    print("="*70)

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"\n✓ Connected to {port} at {baudrate} baud")
        time.sleep(0.5)

        # Step 1: Enable torque
        enable_servo_torque(ser)
        time.sleep(1)

        # Step 2: Try moving servos
        print("\n" + "="*70)
        print("TESTING SERVO MOVEMENT (with torque enabled)")
        print("="*70)

        test_servos = [1, 2, 3, 4, 5, 6]

        for servo_id in test_servos:
            print(f"\n--- Servo {servo_id} ---")
            move_servo(ser, servo_id, 1500, 500)  # Center
            time.sleep(0.8)
            move_servo(ser, servo_id, 2000, 500)  # Right
            time.sleep(0.8)
            move_servo(ser, servo_id, 1000, 500)  # Left
            time.sleep(0.8)
            move_servo(ser, servo_id, 1500, 500)  # Center
            time.sleep(0.8)

        # Step 3: Disable torque
        print("\n" + "="*70)
        disable_servo_torque(ser)

        ser.close()
        print("\n✓ Test complete")

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("\nThis test enables servo torque before trying to move servos")
    print("(Many bus servos need torque enabled to move)\n")

    test_with_torque_enable()

    print("\n" + "="*70)
    print("DID ANY SERVOS MOVE?")
    print("="*70)
    print("\nIf YES: Great! We found the issue - servos need torque enabled!")
    print("If NO: The servos might not be bus servos, or need different init")
    print("="*70)
