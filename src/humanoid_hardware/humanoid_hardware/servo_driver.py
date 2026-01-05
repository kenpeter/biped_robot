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
import time
import configparser

class ServoDriverNode(Node):
    def __init__(self):
        super().__init__('servo_driver')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)  # Changed from 9600 to 115200
        self.declare_parameter('timeout', 1.0)

        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value

        # Load servo configuration
        self.config = configparser.ConfigParser()
        # TODO: Make the config file path a ROS parameter
        config_path = '/home/jetson/biped_ws/ZideConfig.ini'
        self.config.read(config_path)
        self.get_logger().info(f'Loaded servo configuration from: {config_path}')


        self.joint_to_servo = {}
        self.servo_metadata = {}
        for section in self.config.sections():
            if section.startswith('steer'):
                try:
                    if self.config.getboolean(section, 'enable'):
                        servo_id = self.config.getint(section, 'id')
                        joint_name = self.config.get(section, 'title')
                        self.joint_to_servo[joint_name] = servo_id
                        self.servo_metadata[servo_id] = {
                            'pmin': self.config.getint(section, 'pmin'),
                            'pmax': self.config.getint(section, 'pmax'),
                            'bias': self.config.getint(section, 'bias')
                        }
                except (configparser.NoOptionError, ValueError) as e:
                    self.get_logger().warn(f'Skipping section {section} due to missing or invalid data: {e}')


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

            # Set hardware control signals (CRITICAL for servo control!)
            self.serial_port.dtr = False
            self.serial_port.rts = False
            time.sleep(0.1)  # Let signals settle

            self.get_logger().info(f'Successfully opened serial port: {serial_port}')
            self.get_logger().info(f'Baud rate: {baud_rate}, DTR=OFF, RTS=OFF')
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
        self.get_logger().info(f'Controlling {len(self.joint_to_servo)} servos (channels 4-11, 12-18)')

    def radians_to_servo_angle(self, radians, servo_id):
        """
        Convert radians to servo angle (0-180 degrees).
        Maps -π to π range to 0-180 degrees, with 90 as center.
        Applies a bias correction for the specific servo.
        """
        # Convert radians to degrees
        degrees = math.degrees(radians)

        # Map to 0-180 range with 90 as center
        # -90 degrees -> 0, 0 degrees -> 90, +90 degrees -> 180
        servo_angle = degrees + 90

        # Apply bias
        if servo_id in self.servo_metadata:
            bias = self.servo_metadata[servo_id].get('bias', 0)
            servo_angle += bias

        # Clamp to valid servo range
        servo_angle = max(0, min(180, servo_angle))

        return int(servo_angle)

    def send_servo_command(self, servo_index, angle, move_time=500):
        """
        Send command to servo board using documented protocol.
        Format: #<index>P<position>T<time>!
        Example: #000P1500T1000! moves servo 0 to 1500μs (90°) in 1000ms

        Args:
            servo_index: Servo ID (0-254)
            angle: Servo angle in degrees (0-180)
            move_time: Movement time in milliseconds (default 500ms)
        """
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().warn('Serial port not open, cannot send command')
            return

        # Ensure angle is in valid range
        angle = max(0, min(180, int(angle)))

        # Convert angle (0-180 degrees) to PWM pulse width using per-servo pmin/pmax
        if servo_index in self.servo_metadata:
            pmin = self.servo_metadata[servo_index].get('pmin', 500)
            pmax = self.servo_metadata[servo_index].get('pmax', 2500)
            pwm = int(pmin + (angle / 180.0) * (pmax - pmin))
        else:
            # Fallback to default if metadata not found
            pwm = int(500 + (angle / 180.0) * 2000)

        pwm = max(500, min(2500, pwm))  # Clamp to valid range

        # Ensure servo index and time are valid
        servo_index = max(0, min(254, int(servo_index)))
        move_time = max(0, min(9999, int(move_time)))

        # Format command with braces (PC software format): {#<index>P<position>T<time>!}
        command = f'{{#{servo_index:03d}P{pwm:04d}T{move_time:04d}!}}'

        try:
            self.serial_port.write(command.encode('ascii'))
            self.serial_port.flush()  # Ensure data is sent immediately
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

                    # Get servo index (MUST be before using it!)
                    servo_index = self.joint_to_servo[joint_name]

                    # Convert to servo angle
                    servo_angle = self.radians_to_servo_angle(position_rad, servo_index)

                    # Send command
                    self.send_servo_command(servo_index, servo_angle)

                    # Update last position
                    self.last_positions[joint_name] = position_rad

                    self.get_logger().debug(
                        f'{joint_name} -> Servo {servo_index}: {position_rad:.3f} rad = {servo_angle}°'
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
    except (KeyboardInterrupt, SystemExit):
        # This allows the program to exit cleanly on Ctrl+C or from sys.exit()
        pass
    except Exception as e:
        # Log any other unexpected exceptions
        # We check if a logger exists, in case the error was before node init
        try:
            # Attempt to get a logger if one is available
            logger = rclpy.logging.get_logger("servo_driver_main")
            logger.fatal(f"Unhandled exception in servo_driver: {e}")
        except Exception:
            print(f"Unhandled exception in servo_driver main: {e}", file=sys.stderr)
    finally:
        # Ensure rclpy is shut down
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
