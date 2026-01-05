#!/usr/bin/env python3
"""
Monitor serial port during servo board power-up at MULTIPLE baud rates
This helps if the board uses a different baud rate for startup messages
"""

import serial
import time
import sys
from datetime import datetime
import threading

def format_hex(data):
    """Format bytes as hex string"""
    return ' '.join(f'{b:02X}' for b in data)

def format_ascii(data):
    """Format bytes as ASCII, replacing non-printable with dots"""
    return ''.join(chr(b) if 32 <= b < 127 else '.' for b in data)

class SerialMonitor:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.data_received = []
        self.running = False

    def monitor(self, duration=10):
        """Monitor for specified duration"""
        try:
            ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )

            self.running = True
            start_time = time.time()

            while self.running and (time.time() - start_time) < duration:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    timestamp = time.time() - start_time
                    self.data_received.append((timestamp, data))

                time.sleep(0.01)

            ser.close()

        except Exception as e:
            print(f"[{self.baudrate}] Error: {e}")

    def stop(self):
        self.running = False

def monitor_multiple_bauds(port='/dev/ttyUSB0', duration=10):
    """Monitor at multiple baud rates simultaneously"""

    baud_rates = [9600, 115200, 57600, 38400, 19200, 4800]

    print("="*70)
    print("SERVO BOARD POWER-UP MONITOR - MULTI-BAUD")
    print("="*70)
    print(f"Port: {port}")
    print(f"Baud rates: {', '.join(map(str, baud_rates))}")
    print(f"Monitor duration: {duration} seconds")
    print()
    print("INSTRUCTIONS:")
    print("1. Make sure servo board power is OFF")
    print("2. Make sure USB is connected to Jetson")
    print("3. Press ENTER to start monitoring")
    print("4. Turn ON the servo board power IMMEDIATELY")
    print("5. Wait for monitoring to complete")
    print("="*70)

    input("Press ENTER when ready to start monitoring...")

    # Create monitors for each baud rate
    monitors = [SerialMonitor(port, baud) for baud in baud_rates]

    # Start all monitors in threads
    threads = []
    for monitor in monitors:
        thread = threading.Thread(target=monitor.monitor, args=(duration,))
        thread.start()
        threads.append(thread)

    print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Monitoring started!")
    print("Turn ON servo board power NOW!")
    print(f"Monitoring for {duration} seconds...\n")

    # Wait for all threads to complete
    for thread in threads:
        thread.join()

    print("\n" + "="*70)
    print("MONITORING COMPLETE - RESULTS")
    print("="*70 + "\n")

    # Display results
    found_data = False
    for monitor in monitors:
        if monitor.data_received:
            found_data = True
            print(f"{'='*70}")
            print(f"BAUD RATE: {monitor.baudrate} - RECEIVED {len(monitor.data_received)} MESSAGE(S)")
            print(f"{'='*70}")

            for i, (timestamp, data) in enumerate(monitor.data_received, 1):
                print(f"\nMessage {i} (at t+{timestamp:.3f}s):")
                print(f"  Length: {len(data)} bytes")
                print(f"  HEX:    {format_hex(data)}")
                print(f"  ASCII:  {format_ascii(data)}")
                print(f"  RAW:    {data}")

            print()

    if not found_data:
        print("✗ No data received at any baud rate")
        print("\nPossible reasons:")
        print("  1. Board doesn't send startup messages")
        print("  2. Need to power on board BEFORE opening serial port")
        print("  3. Wrong serial port")
        print("  4. Board power not turned on")
        print("\nTry running the reverse test: power on first, then monitor")

    print("="*70)

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 10

    monitor_multiple_bauds(port, duration)
