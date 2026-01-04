#!/usr/bin/env python3
"""
Script to control servos on the STM32 servo control board via serial port.
Uses the protocol: $ + servo number + angle (3 digits) + #
Example: $A090# moves servo A to 90 degrees.
"""
import serial
import time
import sys

SERIAL_PORT = "/dev/ttyUSB0"  # Identified CH340 serial port
BAUD_RATE = 9600

def send_servo_command(ser, servo_char, angle):
    """
    Sends a command to control a single servo.
    :param ser: Serial port object.
    :param servo_char: Character representing the servo (e.g., 'A' for channel 1).
    :param angle: Angle for the servo (0-180).
    """
    if not isinstance(angle, int) or not (0 <= angle <= 180):
        print(f"Error: Invalid angle {angle}. Angle must be an integer between 0 and 180.")
        return

    # Format the angle to 3 digits with leading zeros
    angle_str = f"{angle:03d}"
    command = f"${servo_char}{angle_str}#"
    
    print(f"Sending command: {command}")
    try:
        ser.write(command.encode('utf-8'))
        time.sleep(0.05) # Small delay after sending command
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

def main():
    print("==============================================================")
    print("STM32 Servo Controller: Servo Control Script")
    print(f"Attempting to open serial port: {SERIAL_PORT} at {BAUD_RATE} baud.")
    print("==============================================================")

    ser = None
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1  # Read timeout
        )
        print(f"Successfully opened serial port {SERIAL_PORT}")
        time.sleep(2)  # Give the serial connection time to establish

        # Test sequence for servo A (channel 1)
        servo_to_test = 'A'
        print(f"\nTesting servo '{servo_to_test}' (channel 1)...")

        print("Moving to 0 degrees...")
        send_servo_command(ser, servo_to_test, 0)
        time.sleep(1)

        print("Moving to 180 degrees...")
        send_servo_command(ser, servo_to_test, 180)
        time.sleep(1)

        print("Moving to 90 degrees (neutral)...")
        send_servo_command(ser, servo_to_test, 90)
        time.sleep(1)

        print(f"\nServo '{servo_to_test}' test complete.")

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}. {e}")
        print("Please ensure the device is connected and you have appropriate permissions.")
        print("You may need to add your user to the 'dialout' group: 'sudo usermod -a -G dialout $USER'")
    except KeyboardInterrupt:
        print("\nExiting servo control script.")
    finally:
        if ser and ser.is_open:
            ser.close()
            print(f"Serial port {SERIAL_PORT} closed.")

if __name__ == "__main__":
    main()
