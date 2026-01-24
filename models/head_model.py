"""Simple timing model for continuous rotation head servo.
Input: target_angle (-180° to +180°)
Output: rotation_time (seconds) - positive=left, negative=right

Supports asymmetric left/right timing to compensate for motor differences.
"""
import json


class HeadServoModel:
    def __init__(self):
        # Default calibration from real servo testing: 6 sec for 360°
        default_spd = 6.0 / 360.0
        self.seconds_per_degree_left = default_spd
        self.seconds_per_degree_right = default_spd

    @property
    def seconds_per_degree(self):
        """Average for backwards compatibility."""
        return (self.seconds_per_degree_left + self.seconds_per_degree_right) / 2

    def predict(self, angle_deg):
        """Calculate rotation time for given angle.

        Args:
            angle_deg: Target angle change in degrees (positive=left, negative=right)

        Returns:
            rotation_time: Seconds to rotate (positive=left, negative=right)
        """
        if angle_deg >= 0:
            # Left rotation
            return angle_deg * self.seconds_per_degree_left
        else:
            # Right rotation (returns negative time)
            return angle_deg * self.seconds_per_degree_right

    def predict_for_target(self, current_deg, target_deg):
        """Calculate rotation time to go from current to target angle."""
        delta = target_deg - current_deg
        return self.predict(delta)

    def save(self, path):
        with open(path, 'w') as f:
            json.dump({
                'seconds_per_degree_left': self.seconds_per_degree_left,
                'seconds_per_degree_right': self.seconds_per_degree_right
            }, f, indent=2)

    def load(self, path):
        with open(path, 'r') as f:
            data = json.load(f)
        # Support both old and new format
        if 'seconds_per_degree_left' in data:
            self.seconds_per_degree_left = data['seconds_per_degree_left']
            self.seconds_per_degree_right = data['seconds_per_degree_right']
        elif 'seconds_per_degree' in data:
            # Old format - use same value for both
            self.seconds_per_degree_left = data['seconds_per_degree']
            self.seconds_per_degree_right = data['seconds_per_degree']


if __name__ == "__main__":
    model = HeadServoModel()
    print("Asymmetric head servo timing model")
    print(f"Left timing:  {model.seconds_per_degree_left*1000:.2f}ms per degree")
    print(f"Right timing: {model.seconds_per_degree_right*1000:.2f}ms per degree")
    print("\nTest predictions (angle -> time):")
    for angle in [-90, -45, 0, 45, 90]:
        t = model.predict(angle)
        direction = "left" if t > 0 else "right" if t < 0 else "none"
        print(f"  {angle:+4d}° -> {abs(t):.3f}s {direction}")
