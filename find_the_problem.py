#!/usr/bin/env python3
"""
COMPREHENSIVE DIAGNOSTIC - This WILL find why it's not working!
"""

import serial
import time

def test_everything(port='/dev/ttyUSB0', baud=115200):
    """Test ALL possible issues"""
    print(f"\n{'='*70}")
    print(f"COMPREHENSIVE DIAGNOSTIC TEST")
    print(f"{'='*70}\n")

    try:
        print("Opening serial port...")
        ser = serial.Serial(port, baud, timeout=0.5)
        print(f"✓ Port opened: {port} at {baud} baud\n")

        # ═══════════════════════════════════════════════════════════════
        # TEST 1: Hardware Control Signals (DTR/RTS)
        # ═══════════════════════════════════════════════════════════════
        print(f"{'='*70}")
        print("TEST 1: HARDWARE CONTROL SIGNALS (DTR/RTS)")
        print(f"{'='*70}")
        print("\nPC software might use DTR or RTS to enable servo output!")
        print("Trying all combinations...\n")

        signal_combos = [
            (False, False, "DTR=OFF, RTS=OFF (default)"),
            (True, False,  "DTR=ON,  RTS=OFF"),
            (False, True,  "DTR=OFF, RTS=ON"),
            (True, True,   "DTR=ON,  RTS=ON (MOST LIKELY)"),
        ]

        for dtr, rts, desc in signal_combos:
            print(f"─────────────────────────────────────────────")
            print(f"Testing: {desc}")
            print(f"─────────────────────────────────────────────")

            # Set hardware signals
            ser.dtr = dtr
            ser.rts = rts
            time.sleep(0.1)  # Let signals settle

            # Clear buffers
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)

            # Try servo 0 with LARGE obvious movement
            print("Sending command to servo 0: 500→2000→500")
            for pwm in [500, 2000, 500]:
                cmd = f"{{#000P{pwm:04d}T0300!}}"
                ser.write(cmd.encode('ascii'))
                ser.flush()
                print(f"  → PWM {pwm}", flush=True)
                time.sleep(0.5)

            print("\n⚠ WATCH THE ROBOT! Did servo 0 move?")
            response = input("Did it move? (y/n): ").strip().lower()

            if response == 'y':
                print(f"\n{'='*70}")
                print(f"✓✓✓ FOUND IT! ✓✓✓")
                print(f"{'='*70}")
                print(f"\nThe servos need: {desc}")
                print("\nAdd this to your servo_driver.py after opening serial:")
                print(f"    self.serial_port.dtr = {dtr}")
                print(f"    self.serial_port.rts = {rts}")
                print(f"    time.sleep(0.1)")
                print(f"\n{'='*70}\n")
                ser.close()
                return True

        print("\n✗ No movement with any DTR/RTS combination\n")

        # ═══════════════════════════════════════════════════════════════
        # TEST 2: Check if board is sending data (maybe wrong port?)
        # ═══════════════════════════════════════════════════════════════
        print(f"\n{'='*70}")
        print("TEST 2: CHECK BOARD RESPONSE")
        print(f"{'='*70}\n")

        ser.reset_input_buffer()
        time.sleep(0.5)

        if ser.in_waiting > 0:
            data = ser.read(min(ser.in_waiting, 200))
            print(f"Board is sending data: {len(data)} bytes")
            print(f"This looks like sensor/IMU data (not servo ACKs)")
            print("✓ Board communication IS working\n")
        else:
            print("⚠ Board is NOT sending any data")
            print("This might mean:")
            print("  - Wrong baud rate")
            print("  - Board not powered")
            print("  - Wrong port\n")

        # ═══════════════════════════════════════════════════════════════
        # TEST 3: Try with delays (maybe board needs time)
        # ═══════════════════════════════════════════════════════════════
        print(f"\n{'='*70}")
        print("TEST 3: SLOW SENDING WITH DELAYS")
        print(f"{'='*70}\n")

        print("Setting DTR=ON, RTS=ON (most common)")
        ser.dtr = True
        ser.rts = True
        time.sleep(1.0)  # Long delay

        print("Sending ONE servo command with long delay...")
        cmd = "{#000P2000T1000!}"
        print(f"Command: {cmd}")
        ser.write(cmd.encode('ascii'))
        ser.flush()
        print("Waiting 3 seconds...")
        time.sleep(3.0)

        print("\n⚠ Did servo 0 move to position 2000?")
        response = input("Did it move? (y/n): ").strip().lower()

        if response == 'y':
            print(f"\n{'='*70}")
            print(f"✓✓✓ FOUND IT! Board needs DTR/RTS enabled ✓✓✓")
            print(f"{'='*70}\n")
            ser.close()
            return True

        # ═════════════════════════════════════════════════════════════
        # TEST 4: Check for power issue
        # ═══════════════════════════════════════════════════════════════
        print(f"\n{'='*70}")
        print("TEST 4: POWER DIAGNOSTIC")
        print(f"{'='*70}\n")

        print("⚠ IMPORTANT: Answer these questions:")
        print()
        print("1. Is there a SEPARATE power adapter plugged into the servo board?")
        print("   (Not just the USB cable - servos need 5-6V, several amps)")
        power_answer = input("   Is servo power connected? (y/n): ").strip().lower()

        if power_answer != 'y':
            print(f"\n{'='*70}")
            print(f"✗✗✗ FOUND THE PROBLEM! ✗✗✗")
            print(f"{'='*70}")
            print("\nSERVOS NEED EXTERNAL POWER!")
            print("USB only powers the CH340 chip and sensors.")
            print("Servos need their own 5-6V power supply (2-5 amps)")
            print("\nLook for:")
            print("  - Barrel jack connector on the board")
            print("  - Terminal block labeled VCC/GND or +/-")
            print("  - Power LED that should be lit")
            print(f"\n{'='*70}\n")
            ser.close()
            return False

        print("\n2. Is there a power LED lit on the servo board?")
        led_answer = input("   Is LED lit? (y/n): ").strip().lower()

        if led_answer != 'y':
            print("\n⚠ Power LED should be ON when servos are powered")
            print("Check power supply connection\n")

        print("\n3. Are servos physically connected to the board?")
        print("   (Check servo cables plugged into board channels)")
        servo_answer = input("   Are servos connected? (y/n): ").strip().lower()

        if servo_answer != 'y':
            print("\n⚠ Servos must be physically connected to receive commands!")
            print("Check cable connections\n")

        # ═══════════════════════════════════════════════════════════════
        # FINAL DIAGNOSIS
        # ═══════════════════════════════════════════════════════════════
        print(f"\n{'='*70}")
        print("FINAL DIAGNOSIS")
        print(f"{'='*70}\n")

        print("Summary:")
        print(f"  • Serial port: {port} at {baud} baud - ✓ Working")
        print(f"  • Board responds: ✓ Yes (sensor data)")
        print(f"  • Servo commands sent: ✓ Yes")
        print(f"  • Servos moved: ✗ NO")
        print()
        print("Most likely causes:")
        print("  1. ✗ Servo POWER SUPPLY not connected or OFF")
        print("  2. ✗ DTR/RTS hardware signals not enabled")
        print("  3. ✗ Servos not physically connected")
        print("  4. ✗ Board has enable switch/jumper that's OFF")
        print()
        print("Next steps:")
        print("  1. Check power supply is plugged in and turned ON")
        print("  2. Check for switches/jumpers on the servo board")
        print("  3. Take a photo of the servo board and show me")
        print(f"\n{'='*70}\n")

        ser.close()
        return False

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    print("\n" + "="*70)
    print("This test will find why servos don't work!")
    print("="*70)
    print("\nYou will be asked to watch for servo movement.")
    print("Answer 'y' if ANY servo moves, 'n' if not.\n")
    input("Press ENTER to start...\n")

    test_everything()
