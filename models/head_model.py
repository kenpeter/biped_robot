"""Simple neural network for head servo control.
Input: target_angle (-30° to +30°)
Output: servo_position (500-2500)
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

    def relu(self, x):
        return np.maximum(0, x)

    def relu_derivative(self, x):
        return (x > 0).astype(float)

    def forward(self, x):
        self.x = x
        self.h = self.relu(np.dot(x, self.W1) + self.b1)
        return np.dot(self.h, self.W2) + self.b2

    def predict(self, angle_deg):
        angle_deg = np.clip(angle_deg, -30, 30)
        x = np.array([[angle_deg]])
        output = self.forward(x)
        position = 1500 + output[0, 0] * 1000
        return np.clip(position, 500, 2500)

    def get_weights(self):
        return {
            'W1': self.W1.tolist(),
            'b1': self.b1.tolist(),
            'W2': self.W2.tolist(),
            'b2': self.b2.tolist()
        }

    def set_weights(self, weights):
        self.W1 = np.array(weights['W1'])
        self.b1 = np.array(weights['b1'])
        self.W2 = np.array(weights['W2'])
        self.b2 = np.array(weights['b2'])

    def save(self, path):
        weights = self.get_weights()
        with open(path, 'w') as f:
            json.dump(weights, f, indent=2)

    def load(self, path):
        with open(path, 'r') as f:
            weights = json.load(f)
        self.set_weights(weights)

    def train(self, X_train, y_train, epochs=2000, lr=0.01):
        for epoch in range(epochs):
            total_loss = 0
            for x, y_true in zip(X_train, y_train):
                x = x.reshape(1, -1)
                y_true = y_true.reshape(1, -1)
                y_pred = self.forward(x)
                loss = np.mean((y_pred - y_true) ** 2)
                total_loss += loss
                delta2 = 2 * (y_pred - y_true) / 1
                dW2 = np.dot(self.h.T, delta2)
                db2 = np.sum(delta2, axis=0)
                delta1 = np.dot(delta2, self.W2.T) * self.relu_derivative(self.h)
                dW1 = np.dot(self.x.T, delta1)
                db1 = np.sum(delta1, axis=0)
                self.W2 -= lr * dW2
                self.b2 -= lr * db2
                self.W1 -= lr * dW1
                self.b1 -= lr * db1
            if (epoch + 1) % 500 == 0:
                print(f"Epoch {epoch+1}/{epochs}, Loss: {total_loss/len(X_train):.6f}")


if __name__ == "__main__":
    model = HeadServoModel()
    X_train = np.array([[-30], [0], [30]])
    y_train = np.array([[-1000], [0], [1000]])
    print("Training head servo model...")
    model.train(X_train, y_train, epochs=1000, lr=0.1)
    model.save("head_model_weights.json")
    print("\nTest predictions:")
    for angle in [-30, -15, 0, 15, 30]:
        pos = model.predict(angle)
        print(f"  {angle:+3d}° -> {pos:.0f}")
