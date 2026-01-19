#!/usr/bin/env python3
"""
Test individual servo on Hiwonder LSC-24 board.
Usage: python3 test_hiwonder_servo.py <servo_id>
Example: python3 test_hiwonder_servo.py 0  # Tests servo ID 0 (head)
"""

import sys
import serial
import time

PORT = '/dev/ttyUSB1'
BAUD = 9600
TIMEOUT = 1.0

def move_servo(ser, servo_id, position, time_ms=500):
    """Move servo to position (500-2500, where 1500 = center)."""
    position = max(500, min(2500, position))
    
    cmd = bytes([
        0x55, 0x55, 0x08, 0x03, 0x01,
        time_ms & 0xFF, (time_ms >> 8) & 0xFF,
        servo_id,
        position & 0xFF, (position >> 8) & 0xFF
    ])
    
    print(f"  Sending: servo_id={servo_id}, pos={position} (0x{position:04X})")
    ser.write(cmd)
    return cmd

def get_battery(ser):
    """Read battery voltage from LSC-24."""
    cmd = bytes([0x55, 0x55, 0x02, 0x0F])
    ser.reset_input_buffer()
    ser.write(cmd)
    response = ser.read(6)
    
    if len(response) >= 5 and response[0] == 0x55:
        voltage_mv = response[4] + (response[5] << 8)
        return voltage_mv
    return None

def test_servo(servo_id):
    """Test a single servo with wiggle motion."""
    print(f"\n{'='*60}")
    print(f"TESTING SERVO ID {servo_id}")
    print(f"{'='*60}")
    
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        print(f"Opened {PORT} at {BAUD} baud")
    except serial.SerialException as e:
        print(f"ERROR: Could not open {PORT}")
        print(f"  {e}")
        print("\nTroubleshooting:")
        print("  1. Check USB connection: ls /dev/ttyUSB*")
        print("  2. Check servo board power is ON")
        print("  3. Check baud rate is 9600")
        return False
    
    # Check battery
    voltage = get_battery(ser)
    if voltage:
        print(f"Battery: {voltage/1000.0:.2f}V", end="")
        if voltage < 6000:
            print(" (LOW - charge needed!)")
        else:
            print(" (OK)")
    else:
        print("Warning: No battery response")
    
    # Wiggle test
    print(f"\nWiggle test - watch servo #{servo_id}")
    print("  Center (1500) -> Left (1300) -> Right (1700) -> Center (1500)")
    
    try:
        # Center
        move_servo(ser, servo_id, 1500, time_ms=500)
        time.sleep(0.8)
        
        # Left
        move_servo(ser, servo_id, 1300, time_ms=500)
        time.sleep(0.8)
        
        # Right
        move_servo(ser, servo_id, 1700, time_ms=500)
        time.sleep(0.8)
        
        # Center
        move_servo(ser, servo_id, 1500, time_ms=500)
        time.sleep(0.8)
        
        print(f"\nTest complete for servo #{servo_id}")
        
    except Exception as e:
        print(f"Error during test: {e}")
    
    ser.close()
    return True

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        print("\nUsage: python3 test_hiwonder_servo.py <servo_id>")
        print("\nServo mapping:")
        print("  0  = Head (pan left/right)")
        print("  1-7  = Left body servos")
        print("  12-19 = Right body servos")
        sys.exit(1)
    
    try:
        servo_id = int(sys.argv[1])
        if servo_id < 0 or servo_id > 23:
            print("Error: Servo ID must be 0-23")
            sys.exit(1)
    except ValueError:
        print("Error: Servo ID must be an integer")
        sys.exit(1)
    
    test_servo(servo_id)
