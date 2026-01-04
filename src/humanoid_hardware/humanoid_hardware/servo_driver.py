#!/usr/bin/env python3
"""
ROS 2 Hardware Driver Node for Humanoid Robot Servo Control
Subscribes to /joint_states and sends commands to STM32 servo board via serial.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import math
import sys

class ServoDriverNode(Node):
    def __init__(self):
        super().__init__('servo_driver')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('timeout', 1.0)

        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value

        # Joint to servo channel mapping (15 DOF)
        # Maps joint names to servo channels (A-O for channels 1-15)
        self.joint_to_servo = {
            'head_pan_joint': 'A',           # Channel 1
            'left_shoulder_pan_joint': 'B',  # Channel 2
            'left_shoulder_pitch_joint': 'C',# Channel 3
            'left_elbow_pitch_joint': 'D',   # Channel 4
            'right_shoulder_pan_joint': 'E', # Channel 5
            'right_shoulder_pitch_joint': 'F',# Channel 6
            'right_elbow_pitch_joint': 'G',  # Channel 7
            'left_hip_pan_joint': 'H',       # Channel 8
            'left_hip_pitch_joint': 'I',     # Channel 9
            'left_knee_pitch_joint': 'J',    # Channel 10
            'left_ankle_pitch_joint': 'K',   # Channel 11
            'right_hip_pan_joint': 'L',      # Channel 12
            'right_hip_pitch_joint': 'M',    # Channel 13
            'right_knee_pitch_joint': 'N',   # Channel 14
            'right_ankle_pitch_joint': 'O',  # Channel 15
        }

        # Store last known positions to avoid unnecessary commands
        self.last_positions = {}

        # Initialize serial connection
        self.serial_port = None
        try:
            self.serial_port = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=timeout
            )
            self.get_logger().info(f'Successfully opened serial port: {serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {serial_port}: {e}')
            self.get_logger().error('Please check device connection and permissions')
            sys.exit(1)

        # Subscribe to joint_states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('Servo driver node initialized and ready')
        self.get_logger().info(f'Listening to /joint_states topic')
        self.get_logger().info(f'Controlling {len(self.joint_to_servo)} servos (A-O)')

    def radians_to_servo_angle(self, radians):
        """
        Convert radians to servo angle (0-180 degrees).
        Maps -π to π range to 0-180 degrees, with 90 as center.
        """
        # Convert radians to degrees
        degrees = math.degrees(radians)

        # Map to 0-180 range with 90 as center
        # -90 degrees -> 0, 0 degrees -> 90, +90 degrees -> 180
        servo_angle = degrees + 90

        # Clamp to valid servo range
        servo_angle = max(0, min(180, servo_angle))

        return int(servo_angle)

    def send_servo_command(self, servo_char, angle):
        """
        Send command to servo board.
        Format: $<servo_char><angle:03d>#
        Example: $A090# moves servo A to 90 degrees
        """
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().warn('Serial port not open, cannot send command')
            return

        # Ensure angle is in valid range
        angle = max(0, min(180, int(angle)))

        # Format command
        command = f'${servo_char}{angle:03d}#'

        try:
            self.serial_port.write(command.encode('utf-8'))
            # self.get_logger().debug(f'Sent: {command}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def joint_state_callback(self, msg):
        """
        Callback for /joint_states topic.
        Processes joint positions and sends commands to servos.
        """
        # Process each joint in the message
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_to_servo:
                # Get position in radians
                if i < len(msg.position):
                    position_rad = msg.position[i]

                    # Check if position has changed significantly (>1 degree)
                    if joint_name in self.last_positions:
                        last_pos = self.last_positions[joint_name]
                        if abs(position_rad - last_pos) < 0.017:  # ~1 degree in radians
                            continue

                    # Convert to servo angle
                    servo_angle = self.radians_to_servo_angle(position_rad)

                    # Get servo channel
                    servo_char = self.joint_to_servo[joint_name]

                    # Send command
                    self.send_servo_command(servo_char, servo_angle)

                    # Update last position
                    self.last_positions[joint_name] = position_rad

                    self.get_logger().debug(
                        f'{joint_name} -> Servo {servo_char}: {position_rad:.3f} rad = {servo_angle}°'
                    )

    def destroy_node(self):
        """Clean up serial connection on shutdown."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ServoDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
