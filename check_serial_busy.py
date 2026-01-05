#!/usr/bin/env python3
"""
Check if serial port is busy (blocked by PC software)
"""

import serial
import time

def check_port_access(port, baud_rates):
    """Try to open serial port at different baud rates"""
    print(f"\n{'='*70}")
    print(f"SERIAL PORT DIAGNOSTIC")
    print(f"{'='*70}\n")

    print(f"Checking if {port} is accessible...\n")

    for baud in baud_rates:
        print(f"Trying {baud} baud...", end=' ')
        try:
            ser = serial.Serial(port, baud, timeout=0.5)
            print(f"✓ Port opened successfully!")

            # Try to send a test command
            test_cmd = "{#000P1500T0500!}"
            ser.write(test_cmd.encode('ascii'))
            ser.flush()
            print(f"  → Sent test command: {test_cmd}")

            # Wait and check for any response
            time.sleep(0.3)
            if ser.in_waiting > 0:
                response = ser.read(min(ser.in_waiting, 100))
                print(f"  ← Board responded ({len(response)} bytes)")
            else:
                print(f"  ← No response from board")

            ser.close()
            print(f"  ✓ Closed port\n")
            return baud  # Return working baud rate

        except serial.SerialException as e:
            print(f"✗ FAILED: {e}")
            if "Permission denied" in str(e):
                print(f"  → Port is likely open by another program (PC software?)")
            elif "Device or resource busy" in str(e):
                print(f"  → Port is BUSY - close PC software first!")
            print()

    print(f"\n{'='*70}")
    print("DIAGNOSIS:")
    print("  ✗ Cannot access serial port at any baud rate")
    print("  → Close the PC software (Zide) if it's running")
    print("  → Then run this script again")
    print(f"{'='*70}\n")
    return None

if __name__ == '__main__':
    port = '/dev/ttyUSB0'
    baud_rates = [115200, 9600, 57600, 38400, 19200]

    working_baud = check_port_access(port, baud_rates)

    if working_baud:
        print(f"\n{'='*70}")
        print(f"SUCCESS! Port is accessible at {working_baud} baud")
        print(f"{'='*70}\n")
