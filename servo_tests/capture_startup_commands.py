#!/usr/bin/env python3
"""
Capture what gets sent TO the servo board during system startup
This will help us understand the initialization sequence
"""

import serial
import time
from datetime import datetime

def format_hex(data):
    """Format bytes as hex string"""
    return ' '.join(f'{b:02X}' for b in data)

def format_ascii(data):
    """Format bytes as ASCII"""
    return ''.join(chr(b) if 32 <= b < 127 else '.' for b in data)

def monitor_bidirectional(port='/dev/ttyUSB0', baudrate=9600, duration=30):
    """
    Monitor BOTH directions on serial port
    - Data FROM board (what we just captured)
    - Data TO board (what we need to find)
    """

    print("="*70)
    print("BIDIRECTIONAL SERIAL MONITOR")
    print("="*70)
    print(f"Port: {port}")
    print(f"Baud: {baudrate}")
    print(f"Duration: {duration} seconds")
    print()
    print("This script will monitor what the board sends AND receives.")
    print("Leave this running during a FULL SYSTEM REBOOT to capture")
    print("the initialization sequence that makes servos move.")
    print("="*70)

    input("Press ENTER to start monitoring...")

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )

        print(f"\n[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Monitoring started")
        print("Waiting for data...\n")

        start_time = time.time()
        received_count = 0

        while (time.time() - start_time) < duration:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                received_count += 1

                print(f"[{timestamp}] <<< RECEIVED from board ({len(data)} bytes):")
                print(f"  HEX:   {format_hex(data)}")
                print(f"  ASCII: {format_ascii(data)}")
                print()

            time.sleep(0.01)

        print("\n" + "="*70)
        print(f"Monitoring complete. Received {received_count} messages from board.")
        print("="*70)

        if received_count == 0:
            print("\nNo data received. This is normal if:")
            print("  - Board only sends data during power-on (not continuously)")
            print("  - No ROS nodes or other programs are sending commands")

    except KeyboardInterrupt:
        print("\n\nMonitoring stopped by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 9600
    duration = int(sys.argv[3]) if len(sys.argv) > 3 else 30

    monitor_bidirectional(port, baudrate, duration)
