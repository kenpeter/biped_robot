"""Neural network for continuous rotation head servo control.
Input: target_angle (-180° to +180°)
Output: rotation_time (seconds) - positive=left, negative=right
"""
import numpy as np
import json


class HeadServoModel:
    def __init__(self):
        self.input_size = 1
        self.hidden_size = 8
        self.output_size = 1

        self.W1 = np.random.randn(self.input_size, self.hidden_size) * 0.1
        self.b1 = np.zeros(self.hidden_size)
        self.W2 = np.random.randn(self.hidden_size, self.output_size) * 0.1
        self.b2 = np.zeros(self.output_size)

        # Calibration from real servo testing
        self.seconds_per_degree = 6.0 / 360.0  # 6 sec for 360°

    def relu(self, x):
        return np.maximum(0, x)

    def relu_derivative(self, x):
        return (x > 0).astype(float)

    def forward(self, x):
        self.x = x
        self.h = self.relu(np.dot(x, self.W1) + self.b1)
        return np.dot(self.h, self.W2) + self.b2

    def predict(self, angle_deg):
        """Predict rotation time to reach angle from current position.

        Args:
            angle_deg: Target angle change in degrees (positive=left, negative=right)

        Returns:
            rotation_time: Seconds to rotate (positive=left, negative=right)
        """
        x = np.array([[angle_deg / 180.0]])  # Normalize input
        angle_correction = self.forward(x)[0, 0] * 180.0  # Denormalize output
        # Base time from calibration, adjusted for learned angle error
        corrected_angle = angle_deg - angle_correction
        base_time = corrected_angle * self.seconds_per_degree
        return base_time

    def predict_for_target(self, current_deg, target_deg):
        """Predict rotation time to go from current angle to target angle."""
        delta = target_deg - current_deg
        return self.predict(delta)

    def get_weights(self):
        return {
            'W1': self.W1.tolist(),
            'b1': self.b1.tolist(),
            'W2': self.W2.tolist(),
            'b2': self.b2.tolist(),
            'seconds_per_degree': self.seconds_per_degree
        }

    def set_weights(self, weights):
        self.W1 = np.array(weights['W1'])
        self.b1 = np.array(weights['b1'])
        self.W2 = np.array(weights['W2'])
        self.b2 = np.array(weights['b2'])
        if 'seconds_per_degree' in weights:
            self.seconds_per_degree = weights['seconds_per_degree']

    def save(self, path):
        weights = self.get_weights()
        with open(path, 'w') as f:
            json.dump(weights, f, indent=2)

    def load(self, path):
        with open(path, 'r') as f:
            weights = json.load(f)
        self.set_weights(weights)

    def train(self, X_train, y_train, epochs=2000, lr=0.001):
        """Train to predict angle corrections.

        X_train: target angle deltas (will be normalized internally)
        y_train: angle errors (actual - target)
        """
        # Normalize inputs and outputs
        X_norm = X_train / 180.0  # Scale to [-1, 1] range
        y_norm = y_train / 180.0

        for epoch in range(epochs):
            total_loss = 0
            for x, y_true in zip(X_norm, y_norm):
                x = x.reshape(1, -1)
                y_true = y_true.reshape(1, -1)
                y_pred = self.forward(x)
                loss = np.mean((y_pred - y_true) ** 2)
                total_loss += loss

                # Backpropagation with gradient clipping
                delta2 = 2 * (y_pred - y_true) / len(X_norm)
                dW2 = np.dot(self.h.T, delta2)
                db2 = np.sum(delta2, axis=0)
                delta1 = np.dot(delta2, self.W2.T) * self.relu_derivative(self.h)
                dW1 = np.dot(self.x.T, delta1)
                db1 = np.sum(delta1, axis=0)

                # Gradient clipping to prevent explosion
                max_grad = 10.0
                dW2 = np.clip(dW2, -max_grad, max_grad)
                db2 = np.clip(db2, -max_grad, max_grad)
                dW1 = np.clip(dW1, -max_grad, max_grad)
                db1 = np.clip(db1, -max_grad, max_grad)

                self.W2 -= lr * dW2
                self.b2 -= lr * db2
                self.W1 -= lr * dW1
                self.b1 -= lr * db1

            if (epoch + 1) % 500 == 0:
                avg_loss = total_loss / len(X_norm)
                print(f"Epoch {epoch+1}/{epochs}, Loss: {avg_loss:.6f}")


if __name__ == "__main__":
    model = HeadServoModel()
    # Simple test: no correction needed for ideal servo
    X_train = np.array([[-90], [-45], [0], [45], [90]])
    y_train = np.array([[0], [0], [0], [0], [0]])  # zero correction
    print("Training head servo model...")
    model.train(X_train, y_train, epochs=1000, lr=0.01)
    model.save("head_model_weights.json")
    print("\nTest predictions (angle -> time):")
    for angle in [-90, -45, 0, 45, 90]:
        t = model.predict(angle)
        print(f"  {angle:+4d}° -> {t:+.3f}s")
