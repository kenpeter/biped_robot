#!/usr/bin/env python3
"""
FINAL TEST - Try both baud rates with all enabled servos
"""

import serial
import time

def test_servo(port, baud, channel):
    """Test one servo at specified baud rate"""
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
        ser.dtr = False
        ser.rts = False
        time.sleep(0.2)

        # Full range movement
        for pwm in [1500, 2000, 1000, 1500]:
            cmd = f"{{#{channel:03d}P{pwm:04d}T0500!}}"
            ser.write(cmd.encode('ascii'))
            ser.flush()
            time.sleep(0.7)

        ser.close()
        return True
    except Exception as e:
        print(f"Error: {e}")
        return False

def main():
    port = '/dev/ttyUSB0'

    print("\n" + "="*70)
    print("FINAL COMPREHENSIVE TEST")
    print("="*70)
    print("\nTesting enabled servos at BOTH baud rates")
    print("Watch for ANY movement!\n")
    print("="*70 + "\n")

    # Enabled servos from config
    test_servos = [0, 4, 5, 12, 17, 18]

    for baud in [9600, 115200]:
        print(f"\n{'='*60}")
        print(f"Testing at {baud} baud")
        print(f"{'='*60}\n")

        for channel in test_servos:
            print(f"  Channel {channel:02d}... ", end='', flush=True)
            test_servo(port, baud, channel)
            print("done")
            time.sleep(0.3)

        print(f"\n⚠ Did ANY servo move at {baud} baud?")
        time.sleep(1.0)

    print(f"\n{'='*70}")
    print("If STILL no movement, please:")
    print("  1. Open PC software")
    print("  2. Click Connect")
    print("  3. Move ONE servo slider")
    print("  4. Tell me WHICH servo box (S00, S04, S17, etc.) moved")
    print("="*70 + "\n")

if __name__ == '__main__':
    main()
