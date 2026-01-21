"""Deploy trained head servo behavior to Jetson"""
import serial
import struct
import time
import math

# Hiwonder servo protocol
def move_servo(ser, servo_id, position, duration_ms=1000):
    """Move servo to position (500-2500) over duration"""
    time_lo = duration_ms & 0xFF
    time_hi = (duration_ms >> 8) & 0xFF
    pos_lo = position & 0xFF
    pos_hi = (position >> 8) & 0xFF

    cmd = bytes([0x55, 0x55, 0x08, 0x03, 0x01,
                 time_lo, time_hi, servo_id, pos_lo, pos_hi])
    ser.write(cmd)

def angle_to_position(angle_deg):
    """Convert angle (-90 to +90) to servo position (500-2500)"""
    # Center: 1500 (0°)
    # Range: 500-2500 (180°) = ±90°
    position = 1500 + int((angle_deg / 90.0) * 1000)
    return max(500, min(2500, position))

def main():
    # Connect to servo board
    port = '/dev/ttyUSB1'
    baud = 9600

    print(f"Connecting to {port} @ {baud} baud...")
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(0.1)

    print("\n=== HEAD SERVO DEPLOYMENT ===")
    print("Oscillating ±30° (Press Ctrl+C to exit)\n")

    # Trained behavior: oscillate ±30°
    angles = [-30, 30]
    idx = 0

    try:
        while True:
            angle = angles[idx]
            position = angle_to_position(angle)

            print(f"→ Moving to {angle:+3d}° (pos: {position})")
            move_servo(ser, servo_id=0, position=position, duration_ms=1000)

            time.sleep(2)  # Hold for 2 seconds
            idx = (idx + 1) % len(angles)

    except KeyboardInterrupt:
        print("\n✓ Stopped")
        # Return to center
        move_servo(ser, servo_id=0, position=1500, duration_ms=500)
        time.sleep(0.5)

    ser.close()

if __name__ == "__main__":
    main()
