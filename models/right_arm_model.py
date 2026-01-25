"""Timing model for right arm servos (1, 2, 3).

Servo 1: Shoulder (forward/backward)
Servo 2: Upper arm (inward/outward)
Servo 3: Forearm (inward/outward)

Each servo has asymmetric timing for positive/negative directions.
"""
import json


class ArmServoModel:
    def __init__(self):
        # Default calibration: 6 sec for 360° = 0.0167s per degree
        default_spd = 6.0 / 360.0

        # Servo 1: shoulder pitch (forward/backward)
        self.servo_1_spd_pos = default_spd  # forward
        self.servo_1_spd_neg = default_spd  # backward

        # Servo 2: shoulder roll (inward/outward)
        self.servo_2_spd_pos = default_spd  # outward
        self.servo_2_spd_neg = default_spd  # inward

        # Servo 3: forearm roll (inward/outward)
        self.servo_3_spd_pos = default_spd  # outward
        self.servo_3_spd_neg = default_spd  # inward

    def predict(self, servo_id, angle_deg):
        """Calculate rotation time for given angle.

        Args:
            servo_id: 1, 2, or 3
            angle_deg: Target angle change in degrees

        Returns:
            rotation_time: Seconds to rotate
        """
        if servo_id == 1:
            spd = self.servo_1_spd_pos if angle_deg >= 0 else self.servo_1_spd_neg
        elif servo_id == 2:
            spd = self.servo_2_spd_pos if angle_deg >= 0 else self.servo_2_spd_neg
        elif servo_id == 3:
            spd = self.servo_3_spd_pos if angle_deg >= 0 else self.servo_3_spd_neg
        else:
            raise ValueError(f"Unknown servo: {servo_id}")

        return abs(angle_deg) * spd

    def get_speeds(self, servo_id):
        """Get (positive_speed, negative_speed) for servo."""
        if servo_id == 1:
            return self.servo_1_spd_pos, self.servo_1_spd_neg
        elif servo_id == 2:
            return self.servo_2_spd_pos, self.servo_2_spd_neg
        elif servo_id == 3:
            return self.servo_3_spd_pos, self.servo_3_spd_neg
        raise ValueError(f"Unknown servo: {servo_id}")

    def set_speeds(self, servo_id, pos_spd, neg_spd):
        """Set speeds for servo."""
        if servo_id == 1:
            self.servo_1_spd_pos = pos_spd
            self.servo_1_spd_neg = neg_spd
        elif servo_id == 2:
            self.servo_2_spd_pos = pos_spd
            self.servo_2_spd_neg = neg_spd
        elif servo_id == 3:
            self.servo_3_spd_pos = pos_spd
            self.servo_3_spd_neg = neg_spd
        else:
            raise ValueError(f"Unknown servo: {servo_id}")

    def save(self, path):
        data = {
            'servo_1': {
                'spd_pos': self.servo_1_spd_pos,
                'spd_neg': self.servo_1_spd_neg
            },
            'servo_2': {
                'spd_pos': self.servo_2_spd_pos,
                'spd_neg': self.servo_2_spd_neg
            },
            'servo_3': {
                'spd_pos': self.servo_3_spd_pos,
                'spd_neg': self.servo_3_spd_neg
            }
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)

    def load(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
        self.servo_1_spd_pos = data['servo_1']['spd_pos']
        self.servo_1_spd_neg = data['servo_1']['spd_neg']
        self.servo_2_spd_pos = data['servo_2']['spd_pos']
        self.servo_2_spd_neg = data['servo_2']['spd_neg']
        self.servo_3_spd_pos = data['servo_3']['spd_pos']
        self.servo_3_spd_neg = data['servo_3']['spd_neg']


if __name__ == "__main__":
    model = ArmServoModel()
    print("Right arm servo timing model (servos 1, 2, 3)")
    print()
    for sid in [1, 2, 3]:
        pos, neg = model.get_speeds(sid)
        print(f"Servo {sid}: +{pos*1000:.2f}ms/deg, -{neg*1000:.2f}ms/deg")
    print()
    print("Test predictions (30° movement):")
    for sid in [1, 2, 3]:
        t = model.predict(sid, 30)
        print(f"  Servo {sid}: 30° -> {t:.3f}s")
