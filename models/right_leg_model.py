"""Timing model for right leg servos (4, 5, 6, 7).

Servo 4: Hip roll (outward/inward)
Servo 5: Hip pitch (forward/backward)
Servo 6: Knee (forward/backward)
Servo 7: Ankle roll (outward/inward)

Each servo has asymmetric timing for positive/negative directions.
"""
import json


class LegServoModel:
    def __init__(self):
        # Default calibration: 6 sec for 360° = 0.0167s per degree
        default_spd = 6.0 / 360.0

        # Servo 4: hip roll (outward/inward)
        self.servo_4_spd_pos = default_spd  # outward
        self.servo_4_spd_neg = default_spd  # inward

        # Servo 5: hip pitch (forward/backward)
        self.servo_5_spd_pos = default_spd  # forward
        self.servo_5_spd_neg = default_spd  # backward

        # Servo 6: knee (forward/backward)
        self.servo_6_spd_pos = default_spd  # forward
        self.servo_6_spd_neg = default_spd  # backward

        # Servo 7: ankle roll (outward/inward)
        self.servo_7_spd_pos = default_spd  # outward
        self.servo_7_spd_neg = default_spd  # inward

    def predict(self, servo_id, angle_deg):
        """Calculate rotation time for given angle.

        Args:
            servo_id: 4, 5, 6, or 7
            angle_deg: Target angle change in degrees

        Returns:
            rotation_time: Seconds to rotate
        """
        if servo_id == 4:
            spd = self.servo_4_spd_pos if angle_deg >= 0 else self.servo_4_spd_neg
        elif servo_id == 5:
            spd = self.servo_5_spd_pos if angle_deg >= 0 else self.servo_5_spd_neg
        elif servo_id == 6:
            spd = self.servo_6_spd_pos if angle_deg >= 0 else self.servo_6_spd_neg
        elif servo_id == 7:
            spd = self.servo_7_spd_pos if angle_deg >= 0 else self.servo_7_spd_neg
        else:
            raise ValueError(f"Unknown servo: {servo_id}")

        return abs(angle_deg) * spd

    def get_speeds(self, servo_id):
        """Get (positive_speed, negative_speed) for servo."""
        if servo_id == 4:
            return self.servo_4_spd_pos, self.servo_4_spd_neg
        elif servo_id == 5:
            return self.servo_5_spd_pos, self.servo_5_spd_neg
        elif servo_id == 6:
            return self.servo_6_spd_pos, self.servo_6_spd_neg
        elif servo_id == 7:
            return self.servo_7_spd_pos, self.servo_7_spd_neg
        raise ValueError(f"Unknown servo: {servo_id}")

    def set_speeds(self, servo_id, pos_spd, neg_spd):
        """Set speeds for servo."""
        if servo_id == 4:
            self.servo_4_spd_pos = pos_spd
            self.servo_4_spd_neg = neg_spd
        elif servo_id == 5:
            self.servo_5_spd_pos = pos_spd
            self.servo_5_spd_neg = neg_spd
        elif servo_id == 6:
            self.servo_6_spd_pos = pos_spd
            self.servo_6_spd_neg = neg_spd
        elif servo_id == 7:
            self.servo_7_spd_pos = pos_spd
            self.servo_7_spd_neg = neg_spd
        else:
            raise ValueError(f"Unknown servo: {servo_id}")

    def save(self, path):
        data = {
            'servo_4': {
                'spd_pos': self.servo_4_spd_pos,
                'spd_neg': self.servo_4_spd_neg
            },
            'servo_5': {
                'spd_pos': self.servo_5_spd_pos,
                'spd_neg': self.servo_5_spd_neg
            },
            'servo_6': {
                'spd_pos': self.servo_6_spd_pos,
                'spd_neg': self.servo_6_spd_neg
            },
            'servo_7': {
                'spd_pos': self.servo_7_spd_pos,
                'spd_neg': self.servo_7_spd_neg
            }
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)

    def load(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
        self.servo_4_spd_pos = data['servo_4']['spd_pos']
        self.servo_4_spd_neg = data['servo_4']['spd_neg']
        self.servo_5_spd_pos = data['servo_5']['spd_pos']
        self.servo_5_spd_neg = data['servo_5']['spd_neg']
        self.servo_6_spd_pos = data['servo_6']['spd_pos']
        self.servo_6_spd_neg = data['servo_6']['spd_neg']
        self.servo_7_spd_pos = data['servo_7']['spd_pos']
        self.servo_7_spd_neg = data['servo_7']['spd_neg']


if __name__ == "__main__":
    model = LegServoModel()
    print("Right leg servo timing model (servos 4, 5, 6, 7)")
    print()
    for sid in [4, 5, 6, 7]:
        pos, neg = model.get_speeds(sid)
        print(f"Servo {sid}: +{pos*1000:.2f}ms/deg, -{neg*1000:.2f}ms/deg")
    print()
    print("Test predictions (30° movement):")
    for sid in [4, 5, 6, 7]:
        t = model.predict(sid, 30)
        print(f"  Servo {sid}: 30° -> {t:.3f}s")
