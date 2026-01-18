# Biped Robot

## Hardware
- Servo board: Hiwonder LSC-24 on /dev/ttyUSB1 @ 9600 baud
- Head: channel 0
- Left body: channels 1-7
- Right body: channels 12-19

## Hiwonder Protocol
Binary format: `[0x55, 0x55, 0x08, 0x03, 0x01, time_lo, time_hi, servo_id, pos_lo, pos_hi]`
Position range: 500-2500 (0-180 degrees)

## Commands
```bash
# Test servo
python3 test_hiwonder_servo.py 0

# Run ROS driver
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch humanoid_hardware humanoid_hardware_node.py

# Rebuild
colcon build --packages-select humanoid_hardware
```
