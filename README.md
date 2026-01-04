# Humanoid Biped Robot - ROS2 Workspace

## Quick Start

### 1. Launch Robot Control System
```bash
cd /home/jetson/biped_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch humanoid_hardware robot_control.launch.py use_rviz:=false
```

This starts:
- Robot state publisher (URDF/transforms)
- Joint state publisher GUI (interactive control)
- Servo driver (hardware interface)

### 2. Test All Servos
```bash
# In a new terminal
cd /home/jetson/biped_ws
source /opt/ros/humble/setup.bash
python3 test_servo_movement.py
```

### 3. Manual Joint Control
```bash
# Move head left/right
ros2 topic pub --once /joint_states sensor_msgs/msg/JointState \
  "{name: ['head_pan_joint'], position: [0.785]}"  # +45 degrees

# Move left arm
ros2 topic pub --once /joint_states sensor_msgs/msg/JointState \
  "{name: ['left_shoulder_pitch_joint'], position: [-0.5]}"
```

## Hardware Configuration

- **Servo Board:** STM32 via CH340 USB-serial converter
- **Serial Port:** /dev/ttyUSB0 (verify with `ls /dev/ttyUSB*`)
- **Baud Rate:** 9600
- **DOF:** 15 (Head: 1, Arms: 6, Legs: 8)
- **ToF Camera:** Connected to Jetson, mounted on head

### Tethered Support Cart
- **Platform:** 25cm × 25cm red cart with electronics
- **Wheels:** 16 total (4 corners × 4 wheels each)
- **Connection:** Elastic string (10cm relaxed length) from robot waist to cart
- **Electronics on Cart:**
  - Jetson Nano (ROS2 brain)
  - STM32 servo control board
  - 2× Batteries
  - USB hub
- **Behavior:** Cart follows robot as it walks, can slide closer/farther on elastic tether

## Servo Mapping

| Servo | Joint | Body Part |
|-------|-------|-----------|
| A | head_pan_joint | Head pan |
| B | left_shoulder_pan_joint | Left shoulder |
| C | left_shoulder_pitch_joint | Left shoulder |
| D | left_elbow_pitch_joint | Left elbow |
| E | right_shoulder_pan_joint | Right shoulder |
| F | right_shoulder_pitch_joint | Right shoulder |
| G | right_elbow_pitch_joint | Right elbow |
| H | left_hip_pan_joint | Left hip |
| I | left_hip_pitch_joint | Left hip |
| J | left_knee_pitch_joint | Left knee |
| K | left_ankle_pitch_joint | Left ankle |
| L | right_hip_pan_joint | Right hip |
| M | right_hip_pitch_joint | Right hip |
| N | right_knee_pitch_joint | Right knee |
| O | right_ankle_pitch_joint | Right ankle |

## Building

```bash
cd /home/jetson/biped_ws
source /opt/ros/humble/setup.bash
colcon build
```

## Packages

1. **humanoid_description** - Robot URDF model and visualization
2. **humanoid_hardware** - Servo control driver and launch files

## Troubleshooting

### Serial port permission denied
```bash
sudo usermod -a -G dialout $USER
# Then logout and login
```

### Wrong serial port
```bash
# Check available ports
ls /dev/ttyUSB*

# Launch with custom port
ros2 launch humanoid_hardware robot_control.launch.py serial_port:=/dev/ttyUSB1
```

### Servos not responding
1. Check serial connection: `lsusb` (look for CH340 device)
2. Test direct serial: `python3 servo_controller.py`
3. Verify ROS nodes: `ros2 node list`
4. Check topics: `ros2 topic list`

## Files

- `memo.md` - Detailed development log and documentation
- `servo_controller.py` - Direct serial test script
- `test_servo_movement.py` - ROS2 servo test sequence
- `src/humanoid_description/` - Robot model package
- `src/humanoid_hardware/` - Hardware driver package

## Status

✓ Hardware control fully functional
✓ All 15 servos responding correctly
✓ ROS2 integration complete
✓ URDF updated with tethered cart system (Isaac Sim compatible)
✓ Ready for algorithm development and simulation

See `memo.md` for complete development history and technical details.
