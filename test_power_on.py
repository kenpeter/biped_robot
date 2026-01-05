#!/usr/bin/env python3
"""
Test sending commands immediately after power-on
"""
import serial
import time

def test_immediate_commands(baud):
    print(f"\n{'='*70}")
    print(f"POWER-ON TEST at {baud} baud")
    print(f"{'='*70}")
    print("\nINSTRUCTIONS:")
    print("1. Turn OFF the servo board power")
    print("2. Press ENTER when ready to start test")
    print("3. When prompted, turn ON the servo board")
    print("4. Script will immediately send commands")
    print(f"{'='*70}\n")

    input("Press ENTER when servo board power is OFF...")

    # Open serial port BEFORE powering on
    print("\nOpening serial port...")
    ser = serial.Serial('/dev/ttyUSB0', baud, timeout=1)
    print("Serial port ready!")

    print("\n*** NOW TURN ON SERVO BOARD POWER ***")
    print("Waiting for servo 17 to move (initialization)...")
    time.sleep(2)  # Wait for board to initialize

    print("\nSending commands to servo 17 RIGHT NOW:")

    # Try multiple servos quickly
    for channel in [17, 4, 5, 10, 12, 15, 18]:
        cmd = f"#{channel:03d}P1500T1000!"
        print(f"  → Channel {channel}: {cmd}")
        ser.write(cmd.encode('ascii'))
        time.sleep(0.3)

    print("\nNow sending OBVIOUS movements:")
    time.sleep(1)

    # Servo 17 - big sweep
    print("  → Servo 17: FULL SWEEP")
    ser.write(b'#017P0500T0500!')
    time.sleep(1.5)
    ser.write(b'#017P2500T0500!')
    time.sleep(1.5)
    ser.write(b'#017P1500T0500!')
    time.sleep(1.5)

    # Try other servos
    for channel in [4, 10, 12, 18]:
        print(f"  → Servo {channel}: FULL SWEEP")
        ser.write(f"#{channel:03d}P0500T0500!".encode('ascii'))
        time.sleep(1.5)
        ser.write(f"#{channel:03d}P2500T0500!".encode('ascii'))
        time.sleep(1.5)
        ser.write(f"#{channel:03d}P1500T0500!".encode('ascii'))
        time.sleep(1.5)

    ser.close()
    print("\nTest complete!")
    print("Did ANY servos move besides the initial servo 17 startup movement?")

if __name__ == '__main__':
    import sys

    print("\nChoose baud rate:")
    print("  1. 9600")
    print("  2. 115200")

    choice = input("Enter choice (1 or 2): ").strip() if len(sys.argv) < 2 else sys.argv[1]
    baud = 9600 if choice == '1' else 115200

    try:
        test_immediate_commands(baud)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
