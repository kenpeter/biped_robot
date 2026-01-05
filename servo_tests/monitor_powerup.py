#!/usr/bin/env python3
"""
Monitor serial port during servo board power-up
Captures any initialization messages or protocol hints
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

def monitor_serial(port='/dev/ttyUSB0', baudrate=9600):
    """Monitor serial port and capture all data"""

    print("="*70)
    print("SERVO BOARD POWER-UP MONITOR")
    print("="*70)
    print(f"Port: {port}")
    print(f"Baud rate: {baudrate}")
    print()
    print("INSTRUCTIONS:")
    print("1. Make sure servo board power is OFF")
    print("2. Press ENTER to start monitoring")
    print("3. Turn ON the servo board power")
    print("4. Watch for any data from the board")
    print("5. Press Ctrl+C to stop")
    print("="*70)

    input("Press ENTER when ready to start monitoring...")

    try:
        # Open serial port
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )

        print(f"\n[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Serial port opened")
        print("Monitoring... Turn ON servo board power NOW!")
        print()

        data_received = False
        last_data_time = time.time()

        while True:
            if ser.in_waiting > 0:
                # Read available data
                data = ser.read(ser.in_waiting)
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

                if not data_received:
                    print("\n" + "!"*70)
                    print("!!! DATA RECEIVED FROM BOARD !!!")
                    print("!"*70 + "\n")
                    data_received = True

                # Display data in multiple formats
                print(f"[{timestamp}] Received {len(data)} bytes:")
                print(f"  HEX:   {format_hex(data)}")
                print(f"  ASCII: {format_ascii(data)}")
                print(f"  RAW:   {data}")
                print()

                last_data_time = time.time()

            else:
                # Check if we should print a status update
                if data_received and (time.time() - last_data_time) > 5:
                    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] No data for 5 seconds, still monitoring...")
                    last_data_time = time.time()

                # Small delay to prevent CPU spinning
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print("Monitoring stopped by user")
        print("="*70)
        if data_received:
            print("✓ Data was received during monitoring")
        else:
            print("✗ No data was received from the board")
            print("\nPossible reasons:")
            print("  1. Board doesn't send startup messages")
            print("  2. Wrong baud rate (try 115200, 57600, 38400, 19200)")
            print("  3. Wrong serial port")
            print("  4. Board needs USB connection first, then power")
        print("="*70)

    except Exception as e:
        print(f"\nError: {e}")

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    # Allow command line arguments
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 9600

    monitor_serial(port, baudrate)
