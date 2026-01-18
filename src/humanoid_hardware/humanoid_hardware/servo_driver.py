#!/usr/bin/env python3
"""ROS 2 driver for Hiwonder LSC-24 servo board."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import math
import configparser

class ServoDriverNode(Node):
    def __init__(self):
        super().__init__('servo_driver')

        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 9600)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        # Load servo config
        self.servos = {}
        config = configparser.ConfigParser()
        config.read('/home/jetson/work/biped_ws/ZideConfig.ini')
        for section in config.sections():
            if section.startswith('steer') and config.getboolean(section, 'enable', fallback=False):
                sid = config.getint(section, 'id')
                self.servos[config.get(section, 'title')] = {
                    'id': sid,
                    'bias': config.getint(section, 'bias', fallback=0)
                }

        # Serial connection
        self.ser = serial.Serial(port, baud, timeout=1.0)
        self.last_pos = {}

        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.get_logger().info(f'Hiwonder LSC-24 ready on {port}, {len(self.servos)} servos')

    def send_servo(self, servo_id, angle, time_ms=500):
        """Send servo command using Hiwonder binary protocol."""
        position = int(500 + (angle / 180.0) * 2000)
        position = max(500, min(2500, position))

        cmd = bytes([
            0x55, 0x55, 0x08, 0x03, 0x01,
            time_ms & 0xFF, (time_ms >> 8) & 0xFF,
            servo_id,
            position & 0xFF, (position >> 8) & 0xFF
        ])
        self.ser.write(cmd)

    def on_joint_state(self, msg):
        for i, name in enumerate(msg.name):
            if name not in self.servos or i >= len(msg.position):
                continue

            pos = msg.position[i]
            if name in self.last_pos and abs(pos - self.last_pos[name]) < 0.017:
                continue

            servo = self.servos[name]
            angle = max(0, min(180, math.degrees(pos) + 90 + servo['bias']))
            self.send_servo(servo['id'], angle)
            self.last_pos[name] = pos

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
