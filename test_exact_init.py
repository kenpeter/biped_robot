#!/usr/bin/env python3
"""
Send EXACT initialization from ZideConfig.ini - what PC software sends
"""

import serial
import time

def send_exact_pc_command(port='/dev/ttyUSB0', baud=115200):
    """Send the exact command from PC software config file"""
    print(f"\n{'='*70}")
    print(f"SENDING EXACT PC SOFTWARE INITIALIZATION")
    print(f"{'='*70}\n")

    try:
        ser = serial.Serial(port, baud, timeout=0.5)
        print(f"✓ Connected to {port} at {baud} baud\n")

        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.5)

        # From ZideConfig.ini line 8: G0000 initialization command
        # This is what the PC software sends - ALL servos at once!
        init_cmd = "{#000P1500T0000!#001P1500T0000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1620T1000!#006P1500T0000!#007P1500T0000!#008P0000T1000!#009P0000T1000!#010P0000T1000!#011P0000T1000!#012P1500T0000!#013P1500T0000!#014P1500T1000!#015P1500T1000!#016P1500T1000!#017P1500T0000!#018P1500T0000!#019P0000T1000!#020P0000T1000!#021P0000T1000!#022P0000T1000!#023P0000T1000!}"

        print(f"{'─'*60}")
        print("Step 1: Sending initialization (all servos to default)")
        print(f"Command length: {len(init_cmd)} bytes")
        print(f"{'─'*60}")
        ser.write(init_cmd.encode('ascii'))
        ser.flush()
        print("  ✓ Sent initialization command")
        time.sleep(2.0)
        print("  → Did servos move to their default positions?\n")

        # Now try moving just servo 17 and 18
        print(f"{'─'*60}")
        print("Step 2: Moving servo 17 to 1600 (small movement)")
        print(f"{'─'*60}")
        cmd = "{#017P1600T0500!}"
        print(f"Command: {cmd}")
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(1.0)
        print("  → Did servo 17 move?\n")

        # Move servo 18
        print(f"{'─'*60}")
        print("Step 3: Moving servo 18 to 1600 (small movement)")
        print(f"{'─'*60}")
        cmd = "{#018P1600T0500!}"
        print(f"Command: {cmd}")
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(1.0)
        print("  → Did servo 18 move?\n")

        # Try servo 0 (might be easier to see)
        print(f"{'─'*60}")
        print("Step 4: Moving servo 0 - wiggle test")
        print(f"{'─'*60}")
        for pwm in [1500, 1700, 1300, 1500]:
            cmd = f"{{#000P{pwm:04d}T0300!}}"
            print(f"  {cmd}")
            ser.write(cmd.encode('ascii'))
            ser.flush()
            time.sleep(0.5)
        print("  → Did servo 0 wiggle?\n")

        # Try ALL enabled servos moving together
        print(f"{'─'*60}")
        print("Step 5: Move ALL enabled servos slightly right")
        print(f"{'─'*60}")
        # Servos 0-7 and 12-18 to 1600
        cmd_parts = []
        for s in list(range(0, 8)) + list(range(12, 19)):
            cmd_parts.append(f"#{s:03d}P1600T1000!")
        cmd = "{" + "".join(cmd_parts) + "}"
        print(f"Command length: {len(cmd)} bytes")
        ser.write(cmd.encode('ascii'))
        ser.flush()
        time.sleep(2.0)
        print("  → Did ANY servos move?\n")

        print(f"{'='*70}")
        print("TEST COMPLETE")
        print(f"{'='*70}\n")

        ser.close()

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    send_exact_pc_command()
