#!/usr/bin/env python3
"""
Test using the protocol discovered from the bsp_uart_servo.c file.
"""
import serial
import time
import struct

def calculate_checksum(data):
    return (~sum(data)) & 0xFF

def test_servo_protocol(port, baudrate):
    """Test the servo protocol on a given port and baudrate."""
    print(f"--- Testing {port} at {baudrate} baud ---")
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            # 1. Enable Torque
            print("Enabling torque...")
            s_id = 0xFE  # Broadcast ID
            len_byte = 0x04
            cmd = 0x03
            addr = 0x28
            on_off = 0x01
            checksum = calculate_checksum([s_id, len_byte, cmd, addr, on_off])
            torque_on_packet = bytes([0xFF, 0xFF, s_id, len_byte, cmd, addr, on_off, checksum])
            print(f"Sending torque enable: {torque_on_packet.hex()}")
            ser.write(torque_on_packet)
            time.sleep(0.1)
            response = ser.read(100)
            if response:
                print(f"Response after torque enable: {response.hex()}")

            time.sleep(0.5) # Wait for servos to be ready

            # 2. Move Servo 18
            servo_id = 18
            position = 1500  # Center position
            move_time = 1000  # 1 second
            print(f"Moving servo {servo_id} to position {position}")

            s_id = servo_id
            len_byte = 0x07
            cmd = 0x03
            addr = 0x2A
            pos_H = (position >> 8) & 0xFF
            pos_L = position & 0xFF
            time_H = (move_time >> 8) & 0xFF
            time_L = move_time & 0xFF
            checksum_data = [s_id, len_byte, cmd, addr, pos_H, pos_L, time_H, time_L]
            checksum = calculate_checksum(checksum_data)
            
            move_packet = bytes([0xFF, 0xFF, s_id, len_byte, cmd, addr, pos_H, pos_L, time_H, time_L, checksum])
            print(f"Sending move command: {move_packet.hex()}")
            ser.write(move_packet)
            time.sleep(0.1)
            response = ser.read(100)
            if response:
                print(f"Response after move: {response.hex()}")
            
            time.sleep(2) # Wait for move to complete

            # 3. Move Servo 18 back
            position = 2500
            print(f"Moving servo {servo_id} to position {position}")
            pos_H = (position >> 8) & 0xFF
            pos_L = position & 0xFF
            checksum_data = [s_id, len_byte, cmd, addr, pos_H, pos_L, time_H, time_L]
            checksum = calculate_checksum(checksum_data)

            move_packet = bytes([0xFF, 0xFF, s_id, len_byte, cmd, addr, pos_H, pos_L, time_H, time_L, checksum])
            print(f"Sending move command: {move_packet.hex()}")
            ser.write(move_packet)
            time.sleep(0.1)
            response = ser.read(100)
            if response:
                print(f"Response after move back: {response.hex()}")


    except serial.SerialException as e:
        print(f"Error: {e}")
    print("-" * 20)

if __name__ == "__main__":
    print("======================================================================")
    print("C PROTOCOL SERVO TEST")
    print("This test uses the protocol discovered from the C source files.")
    print("======================================================================")
    
    test_servo_protocol("/dev/ttyUSB0", 115200)
    test_servo_protocol("/dev/ttyUSB0", 9600)

