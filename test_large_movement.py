#!/usr/bin/env python3
"""
Test with LARGE movements so they're impossible to miss
Try different line endings
"""

import serial
import time

def test_obvious_movement(port='/dev/ttyUSB0', baud=115200):
    """Send commands with LARGE movements that are impossible to miss"""
    print(f"\n{'='*70}")
    print(f"LARGE MOVEMENT TEST - You CANNOT miss this!")
    print(f"{'='*70}\n")

    try:
        ser = serial.Serial(port, baud, timeout=0.5)
        print(f"✓ Connected to {port} at {baud} baud\n")

        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.5)

        # Test different line ending combinations
        line_endings = [
            ("",      "No line ending"),
            ("\r",    "CR only"),
            ("\n",    "LF only"),
            ("\r\n",  "CR+LF"),
        ]

        for ending, desc in line_endings:
            print(f"\n{'='*60}")
            print(f"Testing with: {desc}")
            print(f"{'='*60}\n")

            # Servo 0 - Full range sweep (500 to 2500)
            print("Servo 0 - FULL RANGE SWEEP (500→2500→500)")
            positions = [500, 1000, 1500, 2000, 2500, 2000, 1500, 1000, 500]
            for pwm in positions:
                cmd = f"{{#000P{pwm:04d}T0200!}}{ending}"
                ser.write(cmd.encode('ascii'))
                ser.flush()
                print(f"  {pwm} ", end='', flush=True)
                time.sleep(0.3)
            print("\n  → Did servo 0 move through full range?\n")

            time.sleep(1.0)

            # Servo 17 - Full range
            print("Servo 17 - FULL RANGE SWEEP (500→2500→500)")
            for pwm in positions:
                cmd = f"{{#017P{pwm:04d}T0200!}}{ending}"
                ser.write(cmd.encode('ascii'))
                ser.flush()
                print(f"  {pwm} ", end='', flush=True)
                time.sleep(0.3)
            print("\n  → Did servo 17 move through full range?\n")

            time.sleep(1.0)

            # Servo 18 - Full range
            print("Servo 18 - FULL RANGE SWEEP (500→2500→500)")
            for pwm in positions:
                cmd = f"{{#018P{pwm:04d}T0200!}}{ending}"
                ser.write(cmd.encode('ascii'))
                ser.flush()
                print(f"  {pwm} ", end='', flush=True)
                time.sleep(0.3)
            print("\n  → Did servo 18 move through full range?\n")

            time.sleep(1.0)

        print(f"\n{'='*70}")
        print("If NO servos moved with ANY of those tests:")
        print("  1. Servo board POWER supply might be OFF")
        print("  2. Servos might not be physically connected")
        print("  3. Board might need a hardware enable (switch/jumper)")
        print("  4. PC software might send a magic initialization command")
        print(f"{'='*70}\n")

        ser.close()

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    print("\n⚠ WARNING: Servos will move through FULL RANGE")
    print("Make sure robot won't hit anything!\n")
    input("Press ENTER to start test...")
    test_obvious_movement()
