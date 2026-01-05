#!/usr/bin/env python3
"""
Complete servo board diagnostic tool
"""
import serial
import time
import sys

def test_serial_communication(port, baud):
    """Test if board responds to commands"""
    print(f"\n{'='*70}")
    print(f"Testing serial communication at {baud} baud")
    print(f"{'='*70}\n")

    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(0.5)

        # Flush any existing data
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Test 1: Send query command
        print("1. Sending firmware query: $DGS:0!")
        ser.write(b'$DGS:0!')
        time.sleep(0.3)
        response = ser.read(100)
        if response:
            print(f"   ✓ Board responded with {len(response)} bytes")
            print(f"   Raw: {response[:50]}...")
        else:
            print("   ✗ No response")

        # Test 2: Send reset
        print("\n2. Sending reset: $RST!")
        ser.write(b'$RST!')
        time.sleep(1)
        response = ser.read(100)
        if response:
            print(f"   ✓ Board responded")
        else:
            print("   ✗ No response")

        # Test 3: Stop all servos
        print("\n3. Sending stop all: $DS:T!")
        ser.write(b'$DS:T!')
        time.sleep(0.3)

        ser.close()
        return True

    except Exception as e:
        print(f"   ✗ Error: {e}")
        return False

def test_servo_movement(port, baud, channels):
    """Test servo movement on specific channels"""
    print(f"\n{'='*70}")
    print(f"Testing servo movement at {baud} baud")
    print(f"WATCH YOUR ROBOT CAREFULLY!")
    print(f"{'='*70}\n")

    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(0.5)

        ser.reset_input_buffer()
        ser.reset_output_buffer()

        for channel in channels:
            print(f"\nChannel {channel:02d}:")
            print(f"  Press ENTER to test this channel (or 's' to skip):", end=' ')
            response = input().strip().lower()

            if response == 's':
                print(f"  Skipped")
                continue

            positions = [
                (1500, "CENTER", 1.0),
                (500, "FULL LEFT (0°)", 1.5),
                (2500, "FULL RIGHT (180°)", 1.5),
                (1500, "CENTER", 1.0),
            ]

            for pwm, desc, wait_time in positions:
                cmd = f"#{channel:03d}P{pwm:04d}T0500!"
                print(f"  → {desc:20s} ({cmd})")
                ser.write(cmd.encode('ascii'))
                time.sleep(wait_time)

            print(f"  Did this servo move? (y/n):", end=' ')
            moved = input().strip().lower()

            if moved == 'y':
                print(f"  ✓ Channel {channel} is working!")
                print(f"\n  What body part is this? (e.g. 'left shoulder'):", end=' ')
                body_part = input().strip()
                print(f"  Noted: Channel {channel} = {body_part}")

        ser.close()

    except Exception as e:
        print(f"\n✗ Error: {e}")

def main():
    port = '/dev/ttyUSB0'

    print("\n" + "="*70)
    print("SERVO BOARD DIAGNOSTIC TOOL")
    print("="*70)

    # Step 1: Test communication at both baud rates
    print("\n[STEP 1] Testing serial communication...")

    baud_9600_works = test_serial_communication(port, 9600)
    baud_115200_works = test_serial_communication(port, 115200)

    # Step 2: Determine which baud rate to use
    print("\n" + "="*70)
    print("[STEP 2] Baud rate selection")
    print("="*70)

    if baud_115200_works:
        print("\nBoth baud rates got responses. Recommend testing at 115200 first.")
        baud = 115200
    else:
        print("\nUsing 9600 baud")
        baud = 9600

    # Step 3: Interactive servo testing
    print("\n" + "="*70)
    print("[STEP 3] Servo movement test")
    print("="*70)
    print("\nWe will test servos on channels 4-18 (your reported range)")
    print("For each channel, you can:")
    print("  - Press ENTER to test it")
    print("  - Type 's' to skip it")
    print("  - After testing, tell us if it moved")

    channels = list(range(4, 19))  # 4-18

    test_servo_movement(port, baud, channels)

    print("\n" + "="*70)
    print("DIAGNOSTIC COMPLETE")
    print("="*70)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
