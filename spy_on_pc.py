#!/usr/bin/env python3
"""
SPY ON PC SOFTWARE - See what commands it sends!
Run this BEFORE opening PC software, then open PC and click connect.
"""

import serial
import time
import sys
from datetime import datetime

def spy_on_serial(port='/dev/ttyUSB0', baud=115200):
    """Monitor all data on serial port"""
    print(f"\n{'='*70}")
    print(f"SERIAL PORT SPY - Monitoring {port} at {baud} baud")
    print(f"{'='*70}\n")
    print("This will show EVERYTHING sent/received on the serial port.")
    print("\nINSTRUCTIONS:")
    print("  1. This script is now listening...")
    print("  2. Open the PC software (Zide)")
    print("  3. Click the green 'Connect' button")
    print("  4. Move a servo using the PC software")
    print("  5. Watch this window - it will show what commands are sent!")
    print("  6. Press Ctrl+C when done\n")
    print(f"{'='*70}\n")

    try:
        # Open in non-exclusive mode if possible
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.1,
            exclusive=False  # Allow PC software to also open it
        )
        print(f"[{datetime.now().strftime('%H:%M:%S')}] ✓ Monitoring started\n")

        last_data = None
        data_count = 0

        while True:
            # Check for incoming data (from board)
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

                # Check if it's repeating sensor data
                if data == last_data:
                    data_count += 1
                    if data_count == 1:
                        print(f"[{timestamp}] ← (sensor data repeating...)", end='\r')
                else:
                    if data_count > 0:
                        print(f"\n[{timestamp}] ← Sensor data repeated {data_count} times")
                        data_count = 0

                    # Show new data
                    print(f"\n[{timestamp}] ← RECEIVED {len(data)} bytes:")
                    try:
                        ascii_str = data.decode('ascii', errors='replace')
                        if ascii_str.isprintable() or '\r' in ascii_str or '\n' in ascii_str:
                            print(f"    ASCII: {repr(ascii_str)}")
                        else:
                            print(f"    HEX: {data[:50].hex()}")
                    except:
                        print(f"    HEX: {data[:50].hex()}")

                last_data = data

            time.sleep(0.01)

    except serial.SerialException as e:
        print(f"\n✗ Error: {e}")
        if "Permission denied" in str(e) or "busy" in str(e):
            print("\n⚠ Port is busy. The PC software might already be connected.")
            print("Close PC software first, then run this script.")
    except KeyboardInterrupt:
        print(f"\n\n[{datetime.now().strftime('%H:%M:%S')}] Monitoring stopped by user")
        print(f"{'='*70}\n")
    finally:
        try:
            ser.close()
        except:
            pass

if __name__ == '__main__':
    # Try different baud rates
    baud_rates = [115200, 9600]

    print("\nWhich baud rate should I monitor?")
    print("1. 115200 (recommended)")
    print("2. 9600")
    print("3. Try both\n")

    choice = input("Choice [1]: ").strip() or "1"

    if choice == "1":
        spy_on_serial(baud=115200)
    elif choice == "2":
        spy_on_serial(baud=9600)
    elif choice == "3":
        for baud in baud_rates:
            print(f"\nTrying {baud} baud...\n")
            try:
                spy_on_serial(baud=baud)
                break
            except:
                continue
