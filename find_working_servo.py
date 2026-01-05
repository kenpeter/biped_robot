#!/usr/bin/env python3
"""
Test ALL servo channels to find which ones are physically connected
"""

import serial
import time

def test_all_servos():
    """Test servos 0-23 one by one with obvious movement"""
    port = '/dev/ttyUSB0'
    baud = 115200

    print("\n" + "="*70)
    print("FINDING WHICH SERVOS ARE CONNECTED")
    print("="*70)
    print("\nThis will test each servo channel 0-23 with obvious movement")
    print("Watch your robot and note which servos actually move!\n")
    print("Press Ctrl+C to stop at any time\n")
    print("="*70 + "\n")

    input("Press ENTER to start testing...")

    ser = serial.Serial(port, baud, timeout=0.5)
    ser.dtr = False
    ser.rts = False
    time.sleep(0.1)

    print("\n✓ Connected to servo board\n")

    # Test channels 0-23
    channels_to_test = list(range(0, 24))

    for channel in channels_to_test:
        print(f"\n{'='*60}")
        print(f"Testing Channel {channel:02d}")
        print(f"{'='*60}")

        # Move to center
        cmd = f"{{#{channel:03d}P1500T0500!}}"
        ser.write(cmd.encode('ascii'))
        ser.flush()
        print(f"  Center (1500)... ", end='', flush=True)
        time.sleep(0.8)
        print("sent")

        # Move to 2000 (obvious movement)
        cmd = f"{{#{channel:03d}P2000T0500!}}"
        ser.write(cmd.encode('ascii'))
        ser.flush()
        print(f"  Right (2000)... ", end='', flush=True)
        time.sleep(0.8)
        print("sent")

        # Move to 1000 (opposite direction)
        cmd = f"{{#{channel:03d}P1000T0500!}}"
        ser.write(cmd.encode('ascii'))
        ser.flush()
        print(f"  Left (1000)... ", end='', flush=True)
        time.sleep(0.8)
        print("sent")

        # Back to center
        cmd = f"{{#{channel:03d}P1500T0500!}}"
        ser.write(cmd.encode('ascii'))
        ser.flush()
        print(f"  Center (1500)... ", end='', flush=True)
        time.sleep(0.8)
        print("sent")

        print(f"\n  ⚠ Did you see Channel {channel:02d} move? (Note it down!)")
        time.sleep(0.5)

    print(f"\n{'='*70}")
    print("TESTING COMPLETE")
    print(f"{'='*70}")
    print("\nWhich channels moved? Write them down!")
    print("Then tell me the channel numbers and I'll configure the driver.")
    print(f"{'='*70}\n")

    ser.close()

if __name__ == '__main__':
    try:
        test_all_servos()
    except KeyboardInterrupt:
        print("\n\nTest stopped by user\n")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
