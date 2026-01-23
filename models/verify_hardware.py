#!/usr/bin/env python3
"""
Master Hardware Verification Script for Biped Robot
1. Checks Battery Voltage (Hiwonder LSC-24).
2. Resets all servos to Neutral (1500).
3. Performs a small 'wiggle' test on all ports (0-23).
"""

import serial
import time
import sys

# Configuration
PORT = '/dev/ttyUSB1'
BAUD = 9600
TIMEOUT = 1.0

def get_battery(ser):
    """Reads battery voltage from LSC-24."""
    cmd = bytes([0x55, 0x55, 0x02, 0x0F]) # CMD_BATTERY_VOLTAGE
    ser.reset_input_buffer()
    ser.write(cmd)
    response = ser.read(6)
    
    if len(response) == 6 and response[0] == 0x55:
        voltage_mv = response[4] + (response[5] << 8)
        return voltage_mv
    return None

def move_servo(ser, port_id, position, time_ms=500):
    """Moves a single servo to a position (500-2500)."""
    cmd = bytes([
        0x55, 0x55, 0x08, 0x03, 0x01,
        time_ms & 0xFF, (time_ms >> 8) & 0xFF,
        port_id,
        position & 0xFF, (position >> 8) & 0xFF
    ])
    ser.write(cmd)

def main():
    print(f"Opening connection to {PORT}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"ERROR: Could not open port {PORT}. Is the USB plugged in?")
        print(f"Details: {e}")
        return

    # 1. Check Battery
    print("\n[1] Checking Battery...")
    voltage = get_battery(ser)
    if voltage:
        print(f"    Voltage: {voltage} mV ({voltage/1000.0:.2f} V)")
        if voltage < 6000:
            print("    WARNING: Battery is LOW! Please charge.")
    else:
        print("    ERROR: No response from board. Check connection & power switch.")
        ser.close()
        return

    # 2. Reset to Center
    print("\n[2] Resetting all 24 servos to Center (1500)...")
    for i in range(24):
        move_servo(ser, i, 1500, time_ms=1000)
        time.sleep(0.05) 
    time.sleep(1.5)

    # 3. Wiggle Test
    print("\n[3] Wiggle Test (Small Move Left -> Center)...")
    print("    Watch for movement on all connected servos.")
    
    # Wiggle Head (0) and Body (1-19)
    active_ports = list(range(20)) # Assuming 0-19 are used
    
    for port in active_ports:
        print(f"    Testing Port {port}...", end='\r')
        move_servo(ser, port, 1400, time_ms=300) # Small Left
        time.sleep(0.3)
        move_servo(ser, port, 1500, time_ms=300) # Center
        time.sleep(0.3)
    
    print("\n\n[SUCCESS] Hardware verification complete.")
    ser.close()

if __name__ == '__main__':
    main()
