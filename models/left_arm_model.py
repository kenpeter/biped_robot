"""Timing model for left arm servos (12, 13, 14).

Servo 12: Shoulder (forward/backward)
Servo 13: Upper arm (inward/outward)
Servo 14: Forearm (inward/outward)

Each servo has asymmetric timing for positive/negative directions.
"""
import json


class ArmServoModel:
    def __init__(self):
        # Default calibration: 6 sec for 360° = 0.0167s per degree
        default_spd = 6.0 / 360.0

        # Servo 12: shoulder pitch (forward/backward)
        self.servo_12_spd_pos = default_spd  # forward
        self.servo_12_spd_neg = default_spd  # backward

        # Servo 13: shoulder roll (inward/outward)
        self.servo_13_spd_pos = default_spd  # outward
        self.servo_13_spd_neg = default_spd  # inward

        # Servo 14: forearm roll (inward/outward)
        self.servo_14_spd_pos = default_spd  # outward
        self.servo_14_spd_neg = default_spd  # inward

    def predict(self, servo_id, angle_deg):
        """Calculate rotation time for given angle.

        Args:
            servo_id: 12, 13, or 14
            angle_deg: Target angle change in degrees

        Returns:
            rotation_time: Seconds to rotate
        """
        if servo_id == 12:
            spd = self.servo_12_spd_pos if angle_deg >= 0 else self.servo_12_spd_neg
        elif servo_id == 13:
            spd = self.servo_13_spd_pos if angle_deg >= 0 else self.servo_13_spd_neg
        elif servo_id == 14:
            spd = self.servo_14_spd_pos if angle_deg >= 0 else self.servo_14_spd_neg
        else:
            raise ValueError(f"Unknown servo: {servo_id}")

        return abs(angle_deg) * spd

    def get_speeds(self, servo_id):
        """Get (positive_speed, negative_speed) for servo."""
        if servo_id == 12:
            return self.servo_12_spd_pos, self.servo_12_spd_neg
        elif servo_id == 13:
            return self.servo_13_spd_pos, self.servo_13_spd_neg
        elif servo_id == 14:
            return self.servo_14_spd_pos, self.servo_14_spd_neg
        raise ValueError(f"Unknown servo: {servo_id}")

    def set_speeds(self, servo_id, pos_spd, neg_spd):
        """Set speeds for servo."""
        if servo_id == 12:
            self.servo_12_spd_pos = pos_spd
            self.servo_12_spd_neg = neg_spd
        elif servo_id == 13:
            self.servo_13_spd_pos = pos_spd
            self.servo_13_spd_neg = neg_spd
        elif servo_id == 14:
            self.servo_14_spd_pos = pos_spd
            self.servo_14_spd_neg = neg_spd
        else:
            raise ValueError(f"Unknown servo: {servo_id}")

    def save(self, path):
        data = {
            'servo_12': {
                'spd_pos': self.servo_12_spd_pos,
                'spd_neg': self.servo_12_spd_neg
            },
            'servo_13': {
                'spd_pos': self.servo_13_spd_pos,
                'spd_neg': self.servo_13_spd_neg
            },
            'servo_14': {
                'spd_pos': self.servo_14_spd_pos,
                'spd_neg': self.servo_14_spd_neg
            }
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)

    def load(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
        self.servo_12_spd_pos = data['servo_12']['spd_pos']
        self.servo_12_spd_neg = data['servo_12']['spd_neg']
        self.servo_13_spd_pos = data['servo_13']['spd_pos']
        self.servo_13_spd_neg = data['servo_13']['spd_neg']
        self.servo_14_spd_pos = data['servo_14']['spd_pos']
        self.servo_14_spd_neg = data['servo_14']['spd_neg']


if __name__ == "__main__":
    model = ArmServoModel()
    print("Left arm servo timing model (servos 12, 13, 14)")
    print()
    for sid in [12, 13, 14]:
        pos, neg = model.get_speeds(sid)
        print(f"Servo {sid}: +{pos*1000:.2f}ms/deg, -{neg*1000:.2f}ms/deg")
    print()
    print("Test predictions (30° movement):")
    for sid in [12, 13, 14]:
        t = model.predict(sid, 30)
        print(f"  Servo {sid}: 30° -> {t:.3f}s")
