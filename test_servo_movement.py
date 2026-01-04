#!/usr/bin/env python3
"""
Test script to verify servo movement through ROS2.
Moves each servo sequentially to test hardware connectivity.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class ServoTester(Node):
    def __init__(self):
        super().__init__('servo_tester')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # All 15 joints in order
        self.joints = [
            'head_pan_joint',
            'left_shoulder_pan_joint',
            'left_shoulder_pitch_joint',
            'left_elbow_pitch_joint',
            'right_shoulder_pan_joint',
            'right_shoulder_pitch_joint',
            'right_elbow_pitch_joint',
            'left_hip_pan_joint',
            'left_hip_pitch_joint',
            'left_knee_pitch_joint',
            'left_ankle_pitch_joint',
            'right_hip_pan_joint',
            'right_hip_pitch_joint',
            'right_knee_pitch_joint',
            'right_ankle_pitch_joint',
        ]

    def test_all_servos(self):
        """Test each servo by moving it through a sequence."""
        self.get_logger().info('Starting servo test sequence...')
        self.get_logger().info('Moving all servos to center position (0 rad = 90 degrees)')

        # First, center all servos
        msg = JointState()
        msg.name = self.joints
        msg.position = [0.0] * len(self.joints)
        self.publisher.publish(msg)
        time.sleep(2)

        # Test each servo individually
        for i, joint_name in enumerate(self.joints):
            self.get_logger().info(f'Testing {joint_name} (Servo {chr(65+i)})')

            # Move to +45 degrees (0.785 rad)
            msg = JointState()
            msg.name = self.joints
            msg.position = [0.0] * len(self.joints)
            msg.position[i] = 0.785  # +45 degrees
            self.publisher.publish(msg)
            time.sleep(0.8)

            # Move to -45 degrees (-0.785 rad)
            msg.position[i] = -0.785  # -45 degrees
            self.publisher.publish(msg)
            time.sleep(0.8)

            # Return to center
            msg.position[i] = 0.0
            self.publisher.publish(msg)
            time.sleep(0.5)

        self.get_logger().info('Servo test complete!')
        self.get_logger().info('All servos returned to center position')

def main():
    rclpy.init()
    tester = ServoTester()

    try:
        tester.test_all_servos()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
