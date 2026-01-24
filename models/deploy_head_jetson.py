#!/usr/bin/env python3
"""Deploy trained head servo model to Jetson for continuous rotation servo.
Uses timed rotation instead of position commands.
"""
import serial
import time
import sys

SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 9600
SERVO_CHANNEL = 0
MODEL_PATH = '/home/jetson/work/biped_robot/models/head_model_weights.json'

# Continuous rotation servo calibration (from testing)
DEADBAND_LOW = 1440
DEADBAND_HIGH = 1558
STOP_VALUE = 1500
SPEED_LEFT = 1630   # robot looks left (reduced to compensate for overshoot)
SPEED_RIGHT = 1350  # robot looks right

from head_model import HeadServoModel


def send_speed(ser, speed):
    """Send speed command to continuous rotation servo."""
    speed = int(max(500, min(2500, speed)))
    cmd = bytes([0x55, 0x55, 0x08, 0x03, 0x01, 0, 0,
                 SERVO_CHANNEL, speed & 0xFF, (speed >> 8) & 0xFF])
    ser.write(cmd)


def stop_servo(ser):
    """Stop servo rotation."""
    send_speed(ser, STOP_VALUE)


def rotate(ser, angle_deg, model):
    """Rotate head by specified degrees using trained model.

    Args:
        ser: Serial connection
        angle_deg: Degrees to rotate (positive=left, negative=right)
        model: Trained HeadServoModel
    """
    if abs(angle_deg) < 0.5:
        return

    # Get rotation time from model
    rotation_time = model.predict(angle_deg)

    # Reduce by 2% to compensate for overshoot
    rotation_time *= 0.98

    # Direction based on sign
    if rotation_time > 0:
        speed = SPEED_LEFT
    else:
        speed = SPEED_RIGHT
        rotation_time = abs(rotation_time)

    # Rotate
    send_speed(ser, speed)
    time.sleep(rotation_time)
    stop_servo(ser)
    time.sleep(0.05)  # 50ms settling time for servo momentum


class HeadController:
    """Control head with periodic drift compensation (industry standard)."""

    def __init__(self, ser, model):
        self.ser = ser
        self.model = model
        self.current_angle = 0.0  # Estimated position
        self.movement_count = 0
        self.RECALIBRATE_EVERY = 20  # Industry standard: recalibrate every 20-30 movements

    def rotate_by(self, delta_deg):
        """Rotate by relative amount."""
        rotate(self.ser, delta_deg, self.model)
        self.current_angle += delta_deg
        self.movement_count += 1

        # Industry standard: periodic recalibration to prevent drift accumulation
        if self.movement_count >= self.RECALIBRATE_EVERY:
            print(f"\n[DRIFT COMPENSATION] {self.movement_count} movements completed")
            print(f"[DRIFT COMPENSATION] Estimated drift: ±{self.movement_count * 0.8:.1f}°")
            self.recalibrate_home()

    def recalibrate_home(self):
        """Recalibrate to home pose (industry standard for drift compensation).

        For head servo: User manually centers, OR use thermal camera reference.
        For other servos: Use gravity (IMU), mechanical stops, or foot ground contact.
        """
        print("[RECALIBRATE] Please manually center the head (forward facing)")
        print("[RECALIBRATE] Or press 's' to skip recalibration")
        response = input("[RECALIBRATE] Press Enter when centered (or 's' to skip): ").strip().lower()

        if response != 's':
            self.current_angle = 0.0
            self.movement_count = 0
            print("[RECALIBRATE] ✓ Home pose restored, drift reset\n")
        else:
            print("[RECALIBRATE] Skipped, drift will continue accumulating\n")

    def go_to(self, target_deg):
        """Rotate to absolute angle (0 = forward)."""
        delta = target_deg - self.current_angle
        if abs(delta) < 0.5:
            return
        self.rotate_by(delta)

    def center(self):
        """Return to center (forward facing)."""
        self.go_to(0)

    def stop(self):
        """Stop any rotation."""
        stop_servo(self.ser)


def main():
    print("="*50)
    print("CONTINUOUS ROTATION HEAD SERVO DEPLOYMENT")
    print("="*50)

    print(f"\nConnecting to {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(0.1)
    print("[OK] Connected")

    print(f"\nLoading model from {MODEL_PATH}...")
    model = HeadServoModel()
    try:
        model.load(MODEL_PATH)
        print("[OK] Model loaded")
    except FileNotFoundError:
        print("[WARN] No trained model found, using default timing")

    print(f"\nTiming: {model.seconds_per_degree*1000:.2f}ms per degree")

    head = HeadController(ser, model)

    print("\n" + "-"*50)
    print("Commands: left/right <deg>, center, stop, quit")
    print("Example: left 45, right 90, center")
    print("-"*50 + "\n")

    # Interactive mode or demo mode
    if len(sys.argv) > 1 and sys.argv[1] == '--demo':
        print("Demo mode: oscillating ±30° (Ctrl+C to stop)\n")
        print("INDUSTRY STANDARD: Periodic home pose recalibration every 20 movements\n")
        try:
            while True:
                # Pattern: left 30 → center → right 30 → center
                print("-> left 30°")
                head.go_to(30)
                time.sleep(1)

                print("-> center")
                head.center()
                time.sleep(1)

                print("-> right 30°")
                head.go_to(-30)
                time.sleep(1)

                print("-> center")
                head.center()
                time.sleep(1)

                # Auto-recalibration happens in rotate_by() after 20 movements

        except KeyboardInterrupt:
            print("\nStopping...")
            head.stop()
    else:
        # Interactive mode
        try:
            while True:
                cmd = input(f"[angle: {head.current_angle:+.0f}°] > ").strip().lower()

                if not cmd:
                    continue
                elif cmd == 'quit' or cmd == 'q':
                    break
                elif cmd == 'stop' or cmd == 's':
                    head.stop()
                    print("Stopped")
                elif cmd == 'center' or cmd == 'c':
                    head.center()
                    print(f"Centered (angle: {head.current_angle:+.0f}°)")
                elif cmd.startswith('left') or cmd.startswith('l '):
                    parts = cmd.split()
                    deg = float(parts[1]) if len(parts) > 1 else 45
                    head.rotate_by(deg)
                    print(f"Rotated left {deg}° (angle: {head.current_angle:+.0f}°)")
                elif cmd.startswith('right') or cmd.startswith('r '):
                    parts = cmd.split()
                    deg = float(parts[1]) if len(parts) > 1 else 45
                    head.rotate_by(-deg)
                    print(f"Rotated right {deg}° (angle: {head.current_angle:+.0f}°)")
                elif cmd.startswith('go '):
                    target = float(cmd.split()[1])
                    head.go_to(target)
                    print(f"Went to {target}° (angle: {head.current_angle:+.0f}°)")
                else:
                    print("Unknown command. Use: left/right <deg>, center, stop, quit")

        except KeyboardInterrupt:
            print("\n")

    print("Centering head...")
    head.center()
    ser.close()
    print("[OK] Done")


if __name__ == "__main__":
    main()
