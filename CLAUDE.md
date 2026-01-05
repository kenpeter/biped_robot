# Project Guidelines for Biped Robot Development

## Hardware Configuration

- /dev/ttyUSB0: 24-channel Servo Controller (ZL-IS2, CH340 serial)
- /dev/ttyUSB1: SIPEED Meta Sense Lite Camera
- /dev/ttyUSB2: SIPEED Meta Sense Lite Camera
- Servo board model: ZL-IS2 (24路舵机控制板) with CH340 USB-serial converter

## Servo Control Protocol

- Single servo command format: `#<index>P<position>T<time>!`
- Multiple servo command format: `{#<index>P<position>T<time>#<index>P<position>T<time>...}`
- Use zero-padding for all parameters: index (000-254), position (0500-2500), time (0000-9999)
- PWM range: 500μs (0°) to 2500μs (180°), center position at 1500μs (90°)
- PWM to angle conversion: `PWM = 500 + (angle / 180.0) * 2000`
- Baud rates supported: 9600 (default) or 115200

## Physical Servo Configuration

- Servos are physically connected on channels 4-11 and 12-18 (15 servos total)
- Do NOT send commands to channels 0-14; use actual physical channels 4-11, 12-18
- Configuration metadata stored in ZideConfig.ini (pmin, pmax, bias values)

## Project Conventions

- Isolate all hardware communication logic in separate modules for independent testing
- Hardware protocol implementation should be in dedicated files (e.g., protocol.py) not mixed with ROS callbacks
- Use configparser to load servo metadata from ZideConfig.ini for dynamic configuration

## Common Errors to Avoid

- Wrong channel mapping: ROS driver must target channels 4-11, 12-18, not 0-14
- Missing zero-padding in protocol strings (must use #004P0500T1000! not #4P500T1000!)
- CH340 device is serial (/dev/ttyUSB0), not HID, despite PC software showing "HID Connected"
- After power cycle, servo controller firmware may have state confusion requiring reset
- Servo board responds with IMU sensor data even when servo commands fail

## Useful Commands

- `$RST!` - Software reset of servo controller
- `$DGS:0!` - Query firmware version
- `$DS:T!` - Stop all servos immediately
- `$DS:T×!` - Stop specific servo × (replace × with channel number)
- `$UBBS:1,9600!` or `$UBBS:1,115200!` - Set baud rate (takes effect after power cycle)
- `#005PSCK+010!` - Adjust servo 5 bias by +10 (-500 to +500 range)

## Environment Setup

- ROS 2 Humble workspace at /home/jetson/biped_ws
- Source setup: `source /opt/ros/humble/setup.bash && source install/setup.bash`
- Launch command: `ros2 launch humanoid_hardware humanoid_hardware_node.py`
- Rebuild package: `colcon build --packages-select humanoid_hardware`

## Auto-Captured Learnings


