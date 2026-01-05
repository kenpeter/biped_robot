#!/usr/bin/env python3
"""
Decode the binary response from the servo board
to see what it's actually saying
"""

import serial
import time

def test_and_decode():
    port = '/dev/ttyUSB0'
    baudrate = 115200

    print("="*70)
    print("DECODE BOARD RESPONSE")
    print("="*70)

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(0.5)

        # Send torque enable command
        cmd = bytes([0xFF, 0xFC, 0x04, 0x22, 0x01, 0x23])
        print(f"\nSending torque enable: {' '.join(f'{b:02X}' for b in cmd)}")
        ser.write(cmd)
        time.sleep(0.3)

        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"\nReceived {len(response)} bytes")
            print("\nFirst 200 bytes in different formats:")
            print("-"*70)

            # Show first 200 bytes
            sample = response[:200]

            # Hex
            print("\nHEX:")
            for i in range(0, len(sample), 16):
                hex_str = ' '.join(f'{b:02X}' for b in sample[i:i+16])
                print(f"  {i:04d}: {hex_str}")

            # ASCII
            print("\nASCII (. = non-printable):")
            ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in sample)
            for i in range(0, len(ascii_str), 64):
                print(f"  {ascii_str[i:i+64]}")

            # Look for patterns
            print("\nLooking for Rosmaster protocol patterns (FF FC ...):")
            i = 0
            count = 0
            while i < len(response) - 5 and count < 10:
                if response[i] == 0xFF and response[i+1] == 0xFC:
                    # Found potential Rosmaster message
                    length = response[i+2] if i+2 < len(response) else 0
                    func = response[i+3] if i+3 < len(response) else 0
                    print(f"  [{i:04d}] FF FC {length:02X} {func:02X} ... (func={func:02X}, len={length})")
                    count += 1
                    i += length + 2 if length > 0 else 1
                else:
                    i += 1

        ser.close()

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_and_decode()
