#!/usr/bin/env python3
"""
Send servo commands IMMEDIATELY after board power-up
Based on observation that servos move during full system startup,
the board might be in a special receptive state right after power-on
"""

import serial
import time
from datetime import datetime

def format_hex(data):
    """Format bytes as hex string"""
    return ' '.join(f'{b:02X}' for b in data)

def send_and_listen(ser, data, description, listen_time=0.5):
    """Send data and listen for response"""
    print(f"  Sending: {description}")
    print(f"    HEX: {format_hex(data)}")

    ser.write(data)
    time.sleep(listen_time)

    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"    Response: {format_hex(response)}")
        return response
    else:
        print(f"    Response: (none)")
        return None

def test_immediate_commands(port='/dev/ttyUSB0', baudrate=9600):
    """Test sending commands immediately after power-up"""

    print("="*70)
    print("POWER-UP COMMAND TEST")
    print("="*70)
    print(f"Port: {port}")
    print(f"Baud: {baudrate}")
    print()
    print("INSTRUCTIONS:")
    print("1. Turn OFF servo board power")
    print("2. Keep USB connected")
    print("3. Press ENTER to open serial port")
    print("4. Turn ON servo board power IMMEDIATELY")
    print("5. Script will send test commands in first few seconds")
    print("="*70)

    input("Press ENTER when servo board power is OFF and ready...")

    try:
        # Open serial port BEFORE power-on
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )

        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Serial port opened")
        print(">>> TURN ON SERVO BOARD POWER NOW! <<<")
        print()

        # Wait for power-up messages
        print("Waiting 2 seconds for power-up messages...")
        time.sleep(2)

        # Clear any startup data
        if ser.in_waiting > 0:
            startup_data = ser.read(ser.in_waiting)
            print(f"Received startup data: {format_hex(startup_data[:32])}...\n")

        print("Now sending test commands immediately after power-up:\n")

        # Test 1: Binary LSC protocol (what the data looked like)
        print("Test 1: LSC binary protocol - Move servo 1 to 1500μs (center)")
        # Format: 55 55 <len> 03 <servo_id> <pos_low> <pos_high> <time_low> <time_high>
        cmd = bytes([0x55, 0x55, 0x08, 0x03, 0x01, 0xDC, 0x05, 0xE8, 0x03])
        response = send_and_listen(ser, cmd, "LSC servo 1 center", 1.0)

        # Test 2: Try servo 2
        print("\nTest 2: LSC binary protocol - Move servo 2 to 1500μs")
        cmd = bytes([0x55, 0x55, 0x08, 0x03, 0x02, 0xDC, 0x05, 0xE8, 0x03])
        response = send_and_listen(ser, cmd, "LSC servo 2 center", 1.0)

        # Test 3: ASCII protocol
        print("\nTest 3: ASCII protocol - Servo A to 90°")
        cmd = b"$A090#\n"
        response = send_and_listen(ser, cmd, "ASCII servo A 90°", 1.0)

        # Test 4: Try different servo position (visible movement)
        print("\nTest 4: LSC binary protocol - Move servo 1 to 2000μs (should move)")
        cmd = bytes([0x55, 0x55, 0x08, 0x03, 0x01, 0xD0, 0x07, 0xE8, 0x03])
        response = send_and_listen(ser, cmd, "LSC servo 1 to 2000μs", 1.0)

        # Test 5: Move back
        print("\nTest 5: LSC binary protocol - Move servo 1 to 1000μs (opposite)")
        cmd = bytes([0x55, 0x55, 0x08, 0x03, 0x01, 0xE8, 0x03, 0xE8, 0x03])
        response = send_and_listen(ser, cmd, "LSC servo 1 to 1000μs", 1.0)

        # Test 6: Query position (some boards need this first)
        print("\nTest 6: Query servo 1 position")
        # LSC query format: 55 55 <len> 15 <servo_id>
        cmd = bytes([0x55, 0x55, 0x03, 0x15, 0x01])
        response = send_and_listen(ser, cmd, "Query servo 1 position", 1.0)

        # Test 7: After query, try move again
        if response:
            print("\nTest 7: After query, try move command again")
            cmd = bytes([0x55, 0x55, 0x08, 0x03, 0x01, 0xDC, 0x05, 0xE8, 0x03])
            response = send_and_listen(ser, cmd, "LSC servo 1 center (after query)", 1.0)

        print("\n" + "="*70)
        print("Test complete!")
        print("="*70)
        print("\nDid any servos move?")
        print("If YES: Note which test number worked!")
        print("If NO: The board might need a different initialization sequence.")

    except KeyboardInterrupt:
        print("\n\nTest stopped by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("\nSerial port closed")

if __name__ == "__main__":
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 9600

    test_immediate_commands(port, baudrate)
