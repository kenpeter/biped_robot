"""Simple timing model for continuous rotation head servo.
Input: target_angle (-180° to +180°)
Output: rotation_time (seconds) - positive=left, negative=right
"""
import json


class HeadServoModel:
    def __init__(self):
        # Calibration from real servo testing: 6 sec for 360°
        self.seconds_per_degree = 6.0 / 360.0

    def predict(self, angle_deg):
        """Calculate rotation time for given angle.

        Args:
            angle_deg: Target angle change in degrees (positive=left, negative=right)

        Returns:
            rotation_time: Seconds to rotate (positive=left, negative=right)
        """
        return angle_deg * self.seconds_per_degree

    def predict_for_target(self, current_deg, target_deg):
        """Calculate rotation time to go from current to target angle."""
        delta = target_deg - current_deg
        return self.predict(delta)

    def save(self, path):
        with open(path, 'w') as f:
            json.dump({'seconds_per_degree': self.seconds_per_degree}, f, indent=2)

    def load(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
        if 'seconds_per_degree' in data:
            self.seconds_per_degree = data['seconds_per_degree']


if __name__ == "__main__":
    model = HeadServoModel()
    print("Simple head servo timing model")
    print(f"Timing: {model.seconds_per_degree*1000:.2f}ms per degree\n")
    print("Test predictions (angle -> time):")
    for angle in [-90, -45, 0, 45, 90]:
        t = model.predict(angle)
        direction = "left" if t > 0 else "right" if t < 0 else "none"
        print(f"  {angle:+4d}° -> {abs(t):.3f}s {direction}")
