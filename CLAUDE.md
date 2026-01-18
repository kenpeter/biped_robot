# Biped Robot

## Hardware
- Servo board: Hiwonder LSC-24 on /dev/ttyUSB1 @ 9600 baud
- Head: channel 0
- Left body: channels 1-7
- Right body: channels 12-19

## Robot Model (15 Servos)
USD file: `src/humanoid_description/usd/humanoid.usda`

```
HEAD (1):
  head_joint            → left/right (Z)

LEFT ARM (3):
  l_shoulder_pitch      → forward/backward (Y)
  l_shoulder_roll       → close/away body (X)
  l_forearm_roll        → close/away body (X)

RIGHT ARM (3):
  r_shoulder_pitch      → forward/backward (Y)
  r_shoulder_roll       → close/away body (X)
  r_forearm_roll        → close/away body (X)

LEFT LEG (4):
  l_hip_roll            → close/away body (X)
  l_knee_pitch          → forward/backward (Y)
  l_ankle_pitch         → forward/backward (Y)
  l_foot_roll           → close/away body (X)

RIGHT LEG (4):
  r_hip_roll            → close/away body (X)
  r_knee_pitch          → forward/backward (Y)
  r_ankle_pitch         → forward/backward (Y)
  r_foot_roll           → close/away body (X)
```

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
