#!/usr/bin/env python3
"""
Monitor serial data when CONNECTING to already-powered board
Some boards send init messages when serial connection is established
"""

import serial
import time
import sys
from datetime import datetime

def format_hex(data):
    """Format bytes as hex string"""
    return ' '.join(f'{b:02X}' for b in data)

def format_ascii(data):
    """Format bytes as ASCII, replacing non-printable with dots"""
    return ''.join(chr(b) if 32 <= b < 127 else '.' for b in data)

def test_baud(port, baudrate, duration=3):
    """Test a specific baud rate"""
    data_log = []

    try:
        print(f"  Opening at {baudrate} baud... ", end='', flush=True)

        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )

        # Immediately start capturing
        start_time = time.time()
        while (time.time() - start_time) < duration:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                timestamp = time.time() - start_time
                data_log.append((timestamp, data))
            time.sleep(0.01)

        ser.close()

        if data_log:
            print(f"✓ GOT DATA!")
            return data_log
        else:
            print("(no data)")
            return None

    except Exception as e:
        print(f"Error: {e}")
        return None

def monitor_on_connect(port='/dev/ttyUSB0'):
    """Monitor multiple baud rates when connecting to powered board"""

    baud_rates = [9600, 115200, 57600, 38400, 19200, 4800]

    print("="*70)
    print("SERVO BOARD CONNECTION MONITOR")
    print("="*70)
    print(f"Port: {port}")
    print()
    print("INSTRUCTIONS:")
    print("1. Make sure servo board power is ALREADY ON")
    print("2. Make sure USB is connected to Jetson")
    print("3. Press ENTER to test each baud rate")
    print("   (Will open/close serial connection at each baud rate)")
    print("="*70)

    input("Press ENTER when servo board is powered on and ready...")

    print("\nTesting baud rates...\n")

    results = {}
    for baud in baud_rates:
        data = test_baud(port, baud, duration=3)
        if data:
            results[baud] = data
        time.sleep(0.5)  # Small delay between tests

    # Display results
    print("\n" + "="*70)
    print("RESULTS")
    print("="*70 + "\n")

    if results:
        for baud, data_log in results.items():
            print(f"{'='*70}")
            print(f"BAUD RATE: {baud} - RECEIVED {len(data_log)} MESSAGE(S)")
            print(f"{'='*70}")

            for i, (timestamp, data) in enumerate(data_log, 1):
                print(f"\nMessage {i} (at t+{timestamp:.3f}s after connection):")
                print(f"  Length: {len(data)} bytes")
                print(f"  HEX:    {format_hex(data)}")
                print(f"  ASCII:  {format_ascii(data)}")
                print(f"  RAW:    {data}")

            print()

    else:
        print("✗ No data received at any baud rate")
        print("\nThis means the board does NOT send data when serial connects.")
        print("It might send data only on power-up, or not send anything at all.")

    print("="*70)

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    monitor_on_connect(port)
