#!/usr/bin/env python3
"""Move continuous rotation head servo.

Usage:
  python3 move_head.py              # show status
  python3 move_head.py stop         # stop rotation
  python3 move_head.py left 90      # rotate left 90 degrees
  python3 move_head.py right 45     # rotate right 45 degrees
  python3 move_head.py left         # rotate left continuously
  python3 move_head.py right        # rotate right continuously
  python3 move_head.py speed 1600   # set raw speed (1440-1558=stop)
  python3 move_head.py off          # release servo (torque off)
"""
import serial
import time
import sys

SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 9600
SERVO_CHANNEL = 0

# Continuous rotation servo calibration
DEADBAND_LOW = 1440   # below this = rotate left
DEADBAND_HIGH = 1558  # above this = rotate right
STOP_VALUE = 1500     # center of deadband

# Speed settings (robot's perspective)
SPEED_LEFT = 1630     # rotate left (reduced to compensate for overshoot)
SPEED_RIGHT = 1350    # rotate right (robot looks to its right)

# Timing: 6 seconds for 360° at speed 1600/1400
SECONDS_PER_DEGREE = 6.0 / 360.0  # ~0.0167s per degree

def send_speed(speed):
    """Send speed value to servo (1440-1558=stop, <1440=left, >1558=right)."""
    speed = int(speed)
    speed = max(500, min(2500, speed))
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time_ms = 0  # immediate
    cmd = bytes([0x55, 0x55, 0x08, 0x03, 0x01, time_ms & 0xFF, (time_ms >> 8) & 0xFF,
                 SERVO_CHANNEL, speed & 0xFF, (speed >> 8) & 0xFF])
    ser.write(cmd)
    ser.close()
    return speed

def stop_servo():
    """Stop servo rotation."""
    send_speed(STOP_VALUE)

def rotate_degrees(direction, degrees):
    """Rotate left or right by specified degrees."""
    duration = degrees * SECONDS_PER_DEGREE
    speed = SPEED_LEFT if direction == 'left' else SPEED_RIGHT

    print(f"Rotating {direction} {degrees}° ({duration:.2f}s)...")
    send_speed(speed)
    time.sleep(duration)
    stop_servo()
    print("Done")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Continuous rotation head servo")
        print(f"  Deadband: {DEADBAND_LOW}-{DEADBAND_HIGH} (stop)")
        print(f"  Left speed: {SPEED_LEFT}, Right speed: {SPEED_RIGHT}")
        print(f"  Timing: {SECONDS_PER_DEGREE*1000:.1f}ms per degree")
        print()
        print("Commands:")
        print("  stop         - stop rotation")
        print("  left [deg]   - rotate left (continuous or by degrees)")
        print("  right [deg]  - rotate right (continuous or by degrees)")
        print("  speed <val>  - send raw speed value")
        print("  off          - release servo")
        sys.exit(0)

    cmd = sys.argv[1]

    if cmd == 'stop':
        stop_servo()
        print("Stopped")

    elif cmd == 'off':
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        off_cmd = bytes([0x55, 0x55, 0x04, 0x14, 0x01, SERVO_CHANNEL])
        ser.write(off_cmd)
        ser.close()
        print("Servo released")

    elif cmd == 'left':
        if len(sys.argv) > 2:
            degrees = float(sys.argv[2])
            rotate_degrees('left', degrees)
        else:
            send_speed(SPEED_LEFT)
            print(f"Rotating left (speed {SPEED_LEFT}) - use 'stop' to stop")

    elif cmd == 'right':
        if len(sys.argv) > 2:
            degrees = float(sys.argv[2])
            rotate_degrees('right', degrees)
        else:
            send_speed(SPEED_RIGHT)
            print(f"Rotating right (speed {SPEED_RIGHT}) - use 'stop' to stop")

    elif cmd == 'speed':
        if len(sys.argv) > 2:
            speed = int(sys.argv[2])
            send_speed(speed)
            if DEADBAND_LOW <= speed <= DEADBAND_HIGH:
                print(f"Speed {speed} (in deadband - stopped)")
            elif speed < DEADBAND_LOW:
                print(f"Speed {speed} (rotating left)")
            else:
                print(f"Speed {speed} (rotating right)")
        else:
            print("Usage: move_head.py speed <value>")

    else:
        print(f"Unknown command: {cmd}")
        print("Use: stop, left, right, speed, off")
