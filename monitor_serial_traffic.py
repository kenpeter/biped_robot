#!/usr/bin/env python3
"""
Monitor what the ROS servo driver is actually sending on the serial port.
This helps verify the protocol implementation is correct.
"""

import serial
import time
import threading

def monitor_serial(port, baud):
    """Monitor serial port and display all traffic"""
    print(f"\n{'='*70}")
    print(f"SERIAL PORT MONITOR - {port} at {baud} baud")
    print(f"{'='*70}")
    print("Listening for data on the serial port...")
    print("Press Ctrl+C to stop")
    print(f"{'='*70}\n")

    try:
        ser = serial.Serial(port, baud, timeout=0.1)

        tx_count = 0
        rx_count = 0

        while True:
            # Check for incoming data
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                rx_count += len(data)
                print(f"RX [{len(data):3d} bytes]: {data}")
                try:
                    print(f"         ASCII: {data.decode('ascii', errors='replace')}")
                except:
                    pass
                print(f"           Hex: {' '.join(f'{b:02X}' for b in data)}")
                print()

            time.sleep(0.05)

    except KeyboardInterrupt:
        print(f"\n\nMonitoring stopped.")
        print(f"  Total RX: {rx_count} bytes")
        ser.close()
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

def main():
    import sys

    port = '/dev/ttyUSB0'
    baud = int(sys.argv[1]) if len(sys.argv) > 1 else 9600

    print("\n" + "="*70)
    print("SERIAL TRAFFIC MONITOR")
    print("="*70)
    print("\nThis monitors ALL data sent/received on the servo serial port.")
    print("\nTo use:")
    print("  1. Run this script in one terminal")
    print("  2. Run your ROS servo driver in another terminal")
    print("  3. Publish joint commands")
    print("  4. Watch this terminal to see what's actually being sent")
    print("="*70 + "\n")

    monitor_serial(port, baud)

if __name__ == '__main__':
    main()
