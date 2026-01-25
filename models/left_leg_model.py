"""Timing model for left leg servos (15, 16, 17, 18).

Servo 15: Hip roll (outward/inward)
Servo 16: Hip pitch (forward/backward)
Servo 17: Knee (forward/backward)
Servo 18: Ankle roll (outward/inward)

Each servo has asymmetric timing for positive/negative directions.
"""
import json


class LegServoModel:
    def __init__(self):
        # Default calibration: 6 sec for 360° = 0.0167s per degree
        default_spd = 6.0 / 360.0

        # Servo 15: hip roll (outward/inward)
        self.servo_15_spd_pos = default_spd  # outward
        self.servo_15_spd_neg = default_spd  # inward

        # Servo 16: hip pitch (forward/backward)
        self.servo_16_spd_pos = default_spd  # forward
        self.servo_16_spd_neg = default_spd  # backward

        # Servo 17: knee (forward/backward)
        self.servo_17_spd_pos = default_spd  # forward
        self.servo_17_spd_neg = default_spd  # backward

        # Servo 18: ankle roll (outward/inward)
        self.servo_18_spd_pos = default_spd  # outward
        self.servo_18_spd_neg = default_spd  # inward

    def predict(self, servo_id, angle_deg):
        """Calculate rotation time for given angle.

        Args:
            servo_id: 15, 16, 17, or 18
            angle_deg: Target angle change in degrees

        Returns:
            rotation_time: Seconds to rotate
        """
        if servo_id == 15:
            spd = self.servo_15_spd_pos if angle_deg >= 0 else self.servo_15_spd_neg
        elif servo_id == 16:
            spd = self.servo_16_spd_pos if angle_deg >= 0 else self.servo_16_spd_neg
        elif servo_id == 17:
            spd = self.servo_17_spd_pos if angle_deg >= 0 else self.servo_17_spd_neg
        elif servo_id == 18:
            spd = self.servo_18_spd_pos if angle_deg >= 0 else self.servo_18_spd_neg
        else:
            raise ValueError(f"Unknown servo: {servo_id}")

        return abs(angle_deg) * spd

    def get_speeds(self, servo_id):
        """Get (positive_speed, negative_speed) for servo."""
        if servo_id == 15:
            return self.servo_15_spd_pos, self.servo_15_spd_neg
        elif servo_id == 16:
            return self.servo_16_spd_pos, self.servo_16_spd_neg
        elif servo_id == 17:
            return self.servo_17_spd_pos, self.servo_17_spd_neg
        elif servo_id == 18:
            return self.servo_18_spd_pos, self.servo_18_spd_neg
        raise ValueError(f"Unknown servo: {servo_id}")

    def set_speeds(self, servo_id, pos_spd, neg_spd):
        """Set speeds for servo."""
        if servo_id == 15:
            self.servo_15_spd_pos = pos_spd
            self.servo_15_spd_neg = neg_spd
        elif servo_id == 16:
            self.servo_16_spd_pos = pos_spd
            self.servo_16_spd_neg = neg_spd
        elif servo_id == 17:
            self.servo_17_spd_pos = pos_spd
            self.servo_17_spd_neg = neg_spd
        elif servo_id == 18:
            self.servo_18_spd_pos = pos_spd
            self.servo_18_spd_neg = neg_spd
        else:
            raise ValueError(f"Unknown servo: {servo_id}")

    def save(self, path):
        data = {
            'servo_15': {
                'spd_pos': self.servo_15_spd_pos,
                'spd_neg': self.servo_15_spd_neg
            },
            'servo_16': {
                'spd_pos': self.servo_16_spd_pos,
                'spd_neg': self.servo_16_spd_neg
            },
            'servo_17': {
                'spd_pos': self.servo_17_spd_pos,
                'spd_neg': self.servo_17_spd_neg
            },
            'servo_18': {
                'spd_pos': self.servo_18_spd_pos,
                'spd_neg': self.servo_18_spd_neg
            }
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)

    def load(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
        self.servo_15_spd_pos = data['servo_15']['spd_pos']
        self.servo_15_spd_neg = data['servo_15']['spd_neg']
        self.servo_16_spd_pos = data['servo_16']['spd_pos']
        self.servo_16_spd_neg = data['servo_16']['spd_neg']
        self.servo_17_spd_pos = data['servo_17']['spd_pos']
        self.servo_17_spd_neg = data['servo_17']['spd_neg']
        self.servo_18_spd_pos = data['servo_18']['spd_pos']
        self.servo_18_spd_neg = data['servo_18']['spd_neg']


if __name__ == "__main__":
    model = LegServoModel()
    print("Left leg servo timing model (servos 15, 16, 17, 18)")
    print()
    for sid in [15, 16, 17, 18]:
        pos, neg = model.get_speeds(sid)
        print(f"Servo {sid}: +{pos*1000:.2f}ms/deg, -{neg*1000:.2f}ms/deg")
    print()
    print("Test predictions (30° movement):")
    for sid in [15, 16, 17, 18]:
        t = model.predict(sid, 30)
        print(f"  Servo {sid}: 30° -> {t:.3f}s")
