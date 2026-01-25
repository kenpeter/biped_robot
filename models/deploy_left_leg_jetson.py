#!/usr/bin/env python3
"""Deploy left leg servos to Jetson - continuous rotation servos.

All servos are continuous rotation (like head).
Uses timed rotation with speed commands.

Usage:
  python3 deploy_left_leg_jetson.py              # interactive mode
  python3 deploy_left_leg_jetson.py --calibrate  # calibrate all
  python3 deploy_left_leg_jetson.py --calibrate 15  # calibrate servo 15
  python3 deploy_left_leg_jetson.py --demo       # demo movements
"""
import serial
import time
import sys

SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 9600
MODEL_PATH = '/home/jetson/work/biped_robot/models/left_leg_model_weights.json'

# Servo channels
SERVO_15 = 15  # hip roll (out/in)
SERVO_16 = 16  # hip pitch (fwd/back)
SERVO_17 = 17  # knee (fwd/back)
SERVO_18 = 18  # ankle roll (out/in)

SERVO_NAMES = {
    15: "hip_roll (out/in)",
    16: "hip_pitch (fwd/back)",
    17: "knee (fwd/back)",
    18: "ankle_roll (out/in)",
}

# Continuous rotation: 1500 = stop, >1500 = one way, <1500 = other way
STOP_VALUE = 1500

# Speed values for each servo (matching right leg pattern)
# Format: (positive_speed, negative_speed)
# Servo 15, 18: (out, in)
# Servo 16, 17: (fwd, back)
SERVO_SPEEDS = {
    15: (1630, 1350),  # hip roll: out, in (swapped)
    16: (1350, 1630),  # hip pitch: fwd, back (swapped)
    17: (1630, 1350),  # knee: fwd, back (swapped)
    18: (1350, 1630),  # ankle roll: out, in (swapped)
}

from left_leg_model import LegServoModel


def send_speed(ser, servo_id, speed):
    """Send speed to continuous rotation servo."""
    speed = int(max(500, min(2500, speed)))
    cmd = bytes([0x55, 0x55, 0x08, 0x03, 0x01, 0, 0,
                 servo_id, speed & 0xFF, (speed >> 8) & 0xFF])
    ser.write(cmd)


def stop_servo(ser, servo_id):
    """Stop servo rotation."""
    send_speed(ser, servo_id, STOP_VALUE)


def stop_all(ser):
    """Stop all servos."""
    for sid in [15, 16, 17, 18]:
        stop_servo(ser, sid)


def rotate(ser, servo_id, angle_deg, model):
    """Rotate servo by degrees using timed rotation.

    Args:
        servo_id: 15, 16, 17, or 18
        angle_deg: Degrees to rotate (positive=fwd/out, negative=back/in)
        model: Trained model for timing
    """
    if abs(angle_deg) < 0.5:
        return

    # Get rotation time from model
    rotation_time = model.predict(servo_id, angle_deg)

    # Reduce by 2% for overshoot
    rotation_time *= 0.98

    # Get speeds for this servo
    fwd_speed, back_speed = SERVO_SPEEDS[servo_id]

    # Direction
    if angle_deg > 0:
        speed = fwd_speed
    else:
        speed = back_speed
        rotation_time = abs(rotation_time)

    # Rotate
    send_speed(ser, servo_id, speed)
    time.sleep(rotation_time)
    stop_servo(ser, servo_id)
    time.sleep(0.05)


class LegController:
    """Control left leg servos."""

    def __init__(self, ser, model):
        self.ser = ser
        self.model = model
        self.angles = {15: 0.0, 16: 0.0, 17: 0.0, 18: 0.0}

    def rotate_by(self, servo_id, delta_deg):
        """Rotate servo by relative amount."""
        rotate(self.ser, servo_id, delta_deg, self.model)
        self.angles[servo_id] += delta_deg

    def go_to(self, servo_id, target_deg):
        """Go to absolute angle."""
        delta = target_deg - self.angles[servo_id]
        if abs(delta) >= 0.5:
            self.rotate_by(servo_id, delta)

    def center(self, servo_id):
        """Return servo to center (0°)."""
        self.go_to(servo_id, 0)

    def center_all(self):
        """Center all servos."""
        for sid in [15, 16, 17, 18]:
            self.center(sid)

    def stop(self, servo_id):
        """Stop servo."""
        stop_servo(self.ser, servo_id)

    def stop_all(self):
        """Stop all servos."""
        stop_all(self.ser)


def calibrate_servo(ser, model, servo_id):
    """Calibrate single servo timing."""
    print(f"\n--- Calibrating Servo {servo_id}: {SERVO_NAMES[servo_id]} ---")
    print("This will test movements and adjust timing to fix drift.\n")

    input("1. Manually set servo to center position, then press Enter...")
    print("[OK] Starting from center\n")

    CYCLES = 3
    # Determine movement type based on servo
    if servo_id == 15:  # hip roll - limited range
        POS_ANGLE = 15  # OUT
        NEG_ANGLE = 5   # IN
        pos_name, neg_name = "OUT", "IN"
    elif servo_id == 18:  # ankle roll
        POS_ANGLE = 30  # OUT
        NEG_ANGLE = 30  # IN
        pos_name, neg_name = "OUT", "IN"
    else:  # fwd/back servos (16, 17)
        POS_ANGLE = 30  # FWD
        NEG_ANGLE = 30  # BACK
        pos_name, neg_name = "FWD", "BACK"

    fwd_speed, back_speed = SERVO_SPEEDS[servo_id]

    for attempt in range(10):
        print(f"--- Calibration run {attempt + 1} ---")
        print(f"Running {CYCLES} cycles of +{POS_ANGLE}°/-{NEG_ANGLE}° movements...\n")

        for cycle in range(CYCLES):
            print(f"  Cycle {cycle+1}: ", end="", flush=True)

            # Forward/out
            print(f"{pos_name}..", end="", flush=True)
            rotation_time = model.predict(servo_id, POS_ANGLE)
            send_speed(ser, servo_id, fwd_speed)
            time.sleep(abs(rotation_time) * 0.98)
            stop_servo(ser, servo_id)
            time.sleep(0.5)

            # Back to center
            print("CTR..", end="", flush=True)
            rotation_time = model.predict(servo_id, -POS_ANGLE)
            send_speed(ser, servo_id, back_speed)
            time.sleep(abs(rotation_time) * 0.98)
            stop_servo(ser, servo_id)
            time.sleep(0.5)

            # Backward/in
            print(f"{neg_name}..", end="", flush=True)
            rotation_time = model.predict(servo_id, -NEG_ANGLE)
            send_speed(ser, servo_id, back_speed)
            time.sleep(abs(rotation_time) * 0.98)
            stop_servo(ser, servo_id)
            time.sleep(0.5)

            # Back to center
            print("CTR")
            rotation_time = model.predict(servo_id, NEG_ANGLE)
            send_speed(ser, servo_id, fwd_speed)
            time.sleep(abs(rotation_time) * 0.98)
            stop_servo(ser, servo_id)
            time.sleep(0.5)

        pos_time, neg_time = model.get_speeds(servo_id)
        print(f"\nCurrent timing:")
        print(f"  {pos_name}: {pos_time*1000:.3f} ms/deg")
        print(f"  {neg_name}: {neg_time*1000:.3f} ms/deg")

        print(f"\nAfter test, where did servo end up?")
        print(f"  f  = ended {pos_name} of center (reduce {pos_name} 5%)")
        print(f"  ff = ended FAR {pos_name} (reduce {pos_name} 15%)")
        print(f"  b  = ended {neg_name} of center (reduce {neg_name} 5%)")
        print(f"  bb = ended FAR {neg_name} (reduce {neg_name} 15%)")
        print(f"  c  = centered (done!)")
        print(f"  m  = manual entry")
        print(f"  q  = quit")

        response = input("\nResult [f/ff/b/bb/c/m/q]: ").strip().lower()

        if response == 'c':
            print(f"[OK] Servo {servo_id} calibrated")
            return True
        elif response == 'f':
            model.set_speeds(servo_id, pos_time * 0.95, neg_time)
            print(f"[ADJ] Reduced {pos_name} by 5% -> {pos_time*0.95*1000:.2f} ms/deg")
            input(f"\nRe-center servo {servo_id}, press Enter...")
        elif response == 'ff':
            model.set_speeds(servo_id, pos_time * 0.85, neg_time)
            print(f"[ADJ] Reduced {pos_name} by 15% -> {pos_time*0.85*1000:.2f} ms/deg")
            input(f"\nRe-center servo {servo_id}, press Enter...")
        elif response == 'b':
            model.set_speeds(servo_id, pos_time, neg_time * 0.95)
            print(f"[ADJ] Reduced {neg_name} by 5% -> {neg_time*0.95*1000:.2f} ms/deg")
            input(f"\nRe-center servo {servo_id}, press Enter...")
        elif response == 'bb':
            model.set_speeds(servo_id, pos_time, neg_time * 0.85)
            print(f"[ADJ] Reduced {neg_name} by 15% -> {neg_time*0.85*1000:.2f} ms/deg")
            input(f"\nRe-center servo {servo_id}, press Enter...")
        elif response == 'm':
            try:
                new_fwd = float(input(f"  Enter {pos_name} ms/deg [{pos_time*1000:.2f}]: ") or pos_time*1000) / 1000
                new_back = float(input(f"  Enter {neg_name} ms/deg [{neg_time*1000:.2f}]: ") or neg_time*1000) / 1000
                model.set_speeds(servo_id, new_fwd, new_back)
                print(f"[SET] {pos_name}={new_fwd*1000:.2f}, {neg_name}={new_back*1000:.2f}")
            except ValueError:
                print("[ERROR] Invalid number")
            input(f"\nRe-center servo {servo_id}, press Enter...")
        elif response == 'q':
            return False

    print("[WARN] Max attempts reached")
    return True


def calibrate(ser, model, servos=None):
    """Calibrate servos."""
    if servos is None:
        servos = [15, 16, 17, 18]

    print("\n" + "="*50)
    print("LEFT LEG CALIBRATION (Continuous Rotation)")
    print("="*50)
    print(f"\nCalibrating servo(s): {servos}")
    print("(Ctrl+C to save and exit)\n")

    try:
        for servo_id in servos:
            if not calibrate_servo(ser, model, servo_id):
                print("[CANCELLED]")
                model.save(MODEL_PATH)
                print(f"[SAVED] Progress saved to {MODEL_PATH}")
                return
    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")
        model.save(MODEL_PATH)
        print(f"[SAVED] Progress saved to {MODEL_PATH}")
        return

    model.save(MODEL_PATH)
    print(f"\n[OK] Calibration saved to {MODEL_PATH}")


def main():
    print("="*50)
    print("LEFT LEG SERVOS (Continuous Rotation)")
    print("="*50)

    print(f"\nConnecting to {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(0.1)
    print("[OK] Connected")

    print(f"\nLoading model from {MODEL_PATH}...")
    model = LegServoModel()
    try:
        model.load(MODEL_PATH)
        print("[OK] Model loaded")
    except FileNotFoundError:
        print("[WARN] No model found, using defaults")

    print("\nServo timing:")
    for sid in [15, 16, 17, 18]:
        pos, neg = model.get_speeds(sid)
        print(f"  Servo {sid}: +{pos*1000:.2f}ms/deg, -{neg*1000:.2f}ms/deg")

    # Calibration mode
    if len(sys.argv) > 1 and sys.argv[1] == '--calibrate':
        if len(sys.argv) > 2:
            servo_id = int(sys.argv[2])
            if servo_id in [15, 16, 17, 18]:
                calibrate(ser, model, [servo_id])
            else:
                print(f"[ERROR] Invalid servo: {servo_id}")
        else:
            calibrate(ser, model)
        ser.close()
        return

    leg = LegController(ser, model)

    print("\n" + "-"*50)
    print("Commands:")
    print("  15 out/in <deg>    - hip roll")
    print("  16 fwd/back <deg>  - hip pitch")
    print("  17 fwd/back <deg>  - knee")
    print("  18 out/in <deg>    - ankle roll")
    print("  stop               - stop all")
    print("  quit               - exit")
    print("-"*50 + "\n")

    # Demo mode
    if len(sys.argv) > 1 and sys.argv[1] == '--demo':
        print("Demo mode (Ctrl+C to stop)\n")
        try:
            while True:
                for sid in [15, 16, 17, 18]:
                    print(f"-> Servo {sid} +15°")
                    leg.rotate_by(sid, 15)
                    time.sleep(0.5)

                    print(f"-> Servo {sid} center")
                    leg.center(sid)
                    time.sleep(0.5)

                    print(f"-> Servo {sid} -15°")
                    leg.rotate_by(sid, -15)
                    time.sleep(0.5)

                    print(f"-> Servo {sid} center")
                    leg.center(sid)
                    time.sleep(1)

        except KeyboardInterrupt:
            print("\nStopping...")
            leg.stop_all()
    else:
        # Interactive
        try:
            while True:
                status = f"[15:{leg.angles[15]:+.0f}° 16:{leg.angles[16]:+.0f}° 17:{leg.angles[17]:+.0f}° 18:{leg.angles[18]:+.0f}°]"
                cmd = input(f"{status} > ").strip().lower()

                if not cmd:
                    continue
                elif cmd in ['quit', 'q']:
                    break
                elif cmd in ['stop', 's']:
                    leg.stop_all()
                    print("Stopped")
                elif cmd.startswith('15 '):
                    parts = cmd.split()
                    if len(parts) >= 3:
                        direction = parts[1]
                        deg = float(parts[2])
                        if direction in ['out', 'o']:
                            leg.rotate_by(15, deg)
                        elif direction in ['in', 'i']:
                            leg.rotate_by(15, -deg)
                        print(f"Servo 15: {leg.angles[15]:+.0f}°")
                elif cmd.startswith('16 '):
                    parts = cmd.split()
                    if len(parts) >= 3:
                        direction = parts[1]
                        deg = float(parts[2])
                        if direction in ['fwd', 'f']:
                            leg.rotate_by(16, deg)
                        elif direction in ['back', 'b']:
                            leg.rotate_by(16, -deg)
                        print(f"Servo 16: {leg.angles[16]:+.0f}°")
                elif cmd.startswith('17 '):
                    parts = cmd.split()
                    if len(parts) >= 3:
                        direction = parts[1]
                        deg = float(parts[2])
                        if direction in ['fwd', 'f']:
                            leg.rotate_by(17, deg)
                        elif direction in ['back', 'b']:
                            leg.rotate_by(17, -deg)
                        print(f"Servo 17: {leg.angles[17]:+.0f}°")
                elif cmd.startswith('18 '):
                    parts = cmd.split()
                    if len(parts) >= 3:
                        direction = parts[1]
                        deg = float(parts[2])
                        if direction in ['out', 'o']:
                            leg.rotate_by(18, deg)
                        elif direction in ['in', 'i']:
                            leg.rotate_by(18, -deg)
                        print(f"Servo 18: {leg.angles[18]:+.0f}°")
                else:
                    print("Commands: 15 out/in <deg>, 16 fwd/back <deg>, 17 fwd/back <deg>, 18 out/in <deg>, stop, quit")

        except KeyboardInterrupt:
            print("\n")

    leg.stop_all()
    ser.close()
    print("[OK] Done")


if __name__ == "__main__":
    main()
