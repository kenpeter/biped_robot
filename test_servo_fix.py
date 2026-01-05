#!/usr/bin/env python3
"""
Test script to verify the servo driver sends correct protocol commands.
This publishes joint states to test the servo driver.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class ServoTestPublisher(Node):
    def __init__(self):
        super().__init__('servo_test_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info('Servo test publisher initialized')

    def publish_test_position(self, joint_name, angle_degrees):
        """Publish a joint state for testing"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [joint_name]

        # Convert degrees to radians (centered at 0)
        # -90° = -π/2, 0° = 0, +90° = π/2
        angle_rad = (angle_degrees - 90) * 3.14159 / 180.0
        msg.position = [angle_rad]

        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {joint_name} = {angle_degrees}° ({angle_rad:.3f} rad)')

def main():
    rclpy.init()
    node = ServoTestPublisher()

    print("\n" + "="*70)
    print("SERVO DRIVER TEST - PROTOCOL VERIFICATION")
    print("="*70)
    print("\nThis will publish joint states to test servo commands.")
    print("Expected servo board commands should be:")
    print("  Format: #<index>P<position>T<time>!")
    print("  Example: #000P1500T0500!")
    print("\nMake sure the servo_driver node is running in another terminal!")
    print("="*70 + "\n")

    time.sleep(2)

    # Test sequence
    test_joints = [
        ('head_pan_joint', 0),          # Servo 0
        ('left_shoulder_pan_joint', 1), # Servo 1
        ('left_hip_pan_joint', 7),      # Servo 7
        ('right_ankle_pitch_joint', 14) # Servo 14
    ]

    print("\n--- Test 1: Move servos to center (90°) ---")
    for joint_name, servo_idx in test_joints:
        node.publish_test_position(joint_name, 90)
        print(f"  Expected: #{ servo_idx:03d}P1500T0500!")
        time.sleep(1)

    print("\n--- Test 2: Move servos to 0° ---")
    for joint_name, servo_idx in test_joints:
        node.publish_test_position(joint_name, 0)
        print(f"  Expected: #{servo_idx:03d}P0500T0500!")
        time.sleep(1)

    print("\n--- Test 3: Move servos to 180° ---")
    for joint_name, servo_idx in test_joints:
        node.publish_test_position(joint_name, 180)
        print(f"  Expected: #{servo_idx:03d}P2500T0500!")
        time.sleep(1)

    print("\n--- Test 4: Move servos back to center ---")
    for joint_name, servo_idx in test_joints:
        node.publish_test_position(joint_name, 90)
        print(f"  Expected: #{servo_idx:03d}P1500T0500!")
        time.sleep(1)

    print("\n" + "="*70)
    print("TEST COMPLETE")
    print("="*70)
    print("\nDid the servos move?")
    print("If not, check:")
    print("  1. Servo board power is ON")
    print("  2. USB cable is connected")
    print("  3. Baud rate is correct (9600 or 115200)")
    print("  4. Servos are physically connected to the board")
    print("="*70 + "\n")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
