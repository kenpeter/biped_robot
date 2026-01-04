# Biped Workspace Progress Memo

## Date: 2026-01-03

## Summary
Successfully built and tested the `humanoid_description` ROS 2 package.

## Issues Fixed

### 1. CMakeLists.txt Build Errors
**Problem:** Build failed due to incorrect dependencies in CMakeLists.txt
- `joint_state_publisher_gui` was listed as a `find_package()` dependency
- This package is a Python tool and doesn't provide CMake config files

**Solution:** Removed line 10 from `src/humanoid_description/CMakeLists.txt`:
```cmake
find_package(joint_state_publisher_gui REQUIRED)  # REMOVED
```

### 2. Missing Build Dependencies
**Problem:** Missing `ament_lint_auto` package for testing/linting

**Solution:** Installed required packages:
```bash
sudo apt install -y ros-humble-ament-lint-auto ros-humble-ament-lint-common
```

### 3. Missing Visualization Tool
**Problem:** RViz2 not installed

**Solution:** Installed RViz2:
```bash
sudo apt install -y ros-humble-rviz2
```

## Build Status
- Package: `humanoid_description`
- Status: **Built Successfully**
- Build Time: ~3.5 seconds

## Package Structure

### Humanoid Robot Configuration
- **Total Joints:** 15
  - Head: 1 joint (pan)
  - Arms: 6 joints (left/right shoulder pan/pitch, elbow pitch)
  - Legs: 8 joints (left/right hip pan/pitch, knee pitch, ankle pitch)

### Files
- `launch/display.launch.py` - Visualization launch file
- `urdf/humanoid.urdf` - Robot model description
- `rviz/display.rviz` - RViz configuration

## Testing Results

### Test Command
```bash
source /home/jetson/biped_ws/install/setup.bash
ros2 launch humanoid_description display.launch.py
```

### Running Nodes (Verified)
- `/robot_state_publisher` - Publishes robot transforms
- `/joint_state_publisher_gui` - GUI for controlling joints
- `/rviz2` - 3D visualization

### Active Topics (Verified)
- `/joint_states` - 15 joint positions
- `/tf` & `/tf_static` - Transform data
- `/robot_description` - URDF model

### Joint States Published
All 15 joints publishing correctly:
- head_pan_joint
- left_shoulder_pan_joint, left_shoulder_pitch_joint, left_elbow_pitch_joint
- right_shoulder_pan_joint, right_shoulder_pitch_joint, right_elbow_pitch_joint
- left_hip_pan_joint, left_hip_pitch_joint, left_knee_pitch_joint, left_ankle_pitch_joint
- right_hip_pan_joint, right_hip_pitch_joint, right_knee_pitch_joint, right_ankle_pitch_joint

## Usage

### Build Command
```bash
cd /home/jetson/biped_ws
colcon build
```

### Launch Visualization
```bash
source /home/jetson/biped_ws/install/setup.bash
ros2 launch humanoid_description display.launch.py
```

This will open:
1. RViz2 window showing the 3D robot model
2. Joint State Publisher GUI with sliders to control each joint

## Hardware Integration Status

### STM32 Servo Control Board (24 DOF)
**Connection Status:** Connected via Type-C USB to Jetson

**Issue Identified:**
The STM32 board is connecting as a **HID device** instead of a serial port:
```
Device: STMicroelectronics STM32 Custm HID
USB ID: 0483:5750
Interface: /dev/hidraw5
Type: USB HID (Human Interface Device)
```

**Problem:**
- Board appears as `/dev/hidraw5` (HID device) NOT `/dev/ttyUSB0` or `/dev/ttyACM0` (serial port)
- This prevents standard serial communication with the servo board
- ROS serial communication libraries expect `/dev/ttyUSB*` or `/dev/ttyACM*`

**Possible Solutions:**
1. **Use Hardware UART (Recommended)**
   - Connect via GPIO pins: STM32 TX/RX ↔ Jetson GPIO pins
   - Use `/dev/ttyTHS1` or `/dev/ttyS0` for communication
   - Requires physical GPIO wire connections

2. **Reflash STM32 Firmware**
   - Change USB mode from HID to CDC-ACM (Virtual COM Port)
   - Board would then appear as `/dev/ttyACM0`

3. **Use HID Protocol**
   - Communicate through `/dev/hidraw5` using HID protocol
   - More complex, requires custom HID communication code

**Detection Log:**
```bash
# From dmesg when connected:
usb 1-2.3: new full-speed USB device number 9
input: STMicroelectronics STM32 Custm HID Keyboard
hid-generic 0003:0483:5750.0006: input,hidraw5: USB HID v1.10
```

**Question to Resolve:**
- Are there GPIO wires connected between Jetson and STM32 board (in addition to USB)?
- What is the board model/part number?
- Does the board have multiple USB ports or mode switches?

### System Architecture (Planned)
```
┌─────────────────────────────────────────────────────────────┐
│                    Jetson (ROS Board)                       │
│  ┌──────────────────┐         ┌──────────────────┐         │
│  │ ROS 2 Controller │ ─────> │ Hardware Driver  │         │
│  │   (To Create)    │         │   Node (TODO)    │         │
│  └──────────────────┘         └─────────┬────────┘         │
└────────────────────────────────────────┼──────────────────┘
                                         │
                              USB Type-C or GPIO UART
                                         │
                          ┌──────────────▼────────────────┐
                          │  STM32 Servo Control Board    │
                          │     (24 DOF, 0483:5750)       │
                          │  - Currently: HID mode        │
                          │  - Needed: Serial mode        │
                          └──────────────┬────────────────┘
                                         │
                          ┌──────────────▼────────────────┐
                          │    24x Physical Servos        │
                          │  (Head, Arms, Legs joints)    │
                          └───────────────────────────────┘
```

**Note:** Currently only URDF visualization works. Hardware control requires resolving the serial communication issue above.

## Next Steps
1. **Resolve STM32 Connection** (PRIORITY)
   - Determine if GPIO UART pins are connected
   - Check board documentation for serial mode configuration
   - Test communication method (HID vs Serial vs UART)

2. **Create Hardware Driver Package**
   - ROS 2 node to communicate with STM32 board
   - Subscribe to `/joint_states` topic
   - Convert joint angles to servo commands
   - Send commands via serial/UART/HID

3. **Add Control Algorithms**
   - Bipedal locomotion controllers
   - Kinematics/dynamics
   - Balance control

4. **Simulation Integration**
   - Gazebo/Isaac Sim integration
   - Test control algorithms safely

## Servo Control Protocol (Discovered from Documentation)
- **Baud rate:** 9600
- **Format:** `$` + `servo number` + `angle` + `#`
- **Servo numbers:** A-P (A=channel 1, B=channel 2, etc., up to P=channel 16)
- **Angle:** 0-180 degrees (3 digits, e.g., 010 for 10 degrees)
- **Example:** `$A010#` moves servo A to 10 degrees
## Notes
- ROS 2 Distribution: Humble
- Platform: Jetson (ARM64)
- Display required for visualization tools
- Robot Specs: 24 DOF (URDF currently shows 15 DOF - needs update)
- Current workspace only has URDF description, no hardware control yet

---

## Update: 2026-01-04 - Hardware Control Implementation COMPLETE

### Hardware Connection Status
**RESOLVED:** Servo board now connected via USB hub
- Serial port: `/dev/ttyUSB0` (CH340 USB-to-serial converter)
- Connection verified and working
- All 15 servos (channels A-O) responding correctly

### Packages Created

#### 1. humanoid_hardware Package
ROS 2 hardware driver package for servo control
- **Location:** `/home/jetson/biped_ws/src/humanoid_hardware/`
- **Node:** `servo_driver`
- **Dependencies:** rclpy, sensor_msgs, std_msgs, pyserial

**Features:**
- Subscribes to `/joint_states` topic
- Maps 15 URDF joints to servo channels A-O
- Converts radians to servo angles (0-180 degrees)
- Sends serial commands to STM32 board
- Configurable parameters: serial_port, baud_rate, timeout

**Joint to Servo Mapping:**
| Joint Name | Servo Channel | Body Part |
|------------|--------------|-----------|
| head_pan_joint | A (1) | Head |
| left_shoulder_pan_joint | B (2) | Left Arm |
| left_shoulder_pitch_joint | C (3) | Left Arm |
| left_elbow_pitch_joint | D (4) | Left Arm |
| right_shoulder_pan_joint | E (5) | Right Arm |
| right_shoulder_pitch_joint | F (6) | Right Arm |
| right_elbow_pitch_joint | G (7) | Right Arm |
| left_hip_pan_joint | H (8) | Left Leg |
| left_hip_pitch_joint | I (9) | Left Leg |
| left_knee_pitch_joint | J (10) | Left Leg |
| left_ankle_pitch_joint | K (11) | Left Leg |
| right_hip_pan_joint | L (12) | Right Leg |
| right_hip_pitch_joint | M (13) | Right Leg |
| right_knee_pitch_joint | N (14) | Right Leg |
| right_ankle_pitch_joint | O (15) | Right Leg |

### Launch Files

#### robot_control.launch.py
Complete system launch file that starts:
1. `robot_state_publisher` - URDF/transforms
2. `joint_state_publisher_gui` - Interactive joint control
3. `servo_driver` - Hardware interface
4. `rviz2` (optional) - 3D visualization

**Usage:**
```bash
# Terminal 1: Launch the control system
cd /home/jetson/biped_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch humanoid_hardware robot_control.launch.py

# With RViz (requires display)
ros2 launch humanoid_hardware robot_control.launch.py use_rviz:=true

# With custom serial port
ros2 launch humanoid_hardware robot_control.launch.py serial_port:=/dev/ttyUSB1
```

### Testing Results

#### Test 1: Serial Communication
- **Status:** ✓ PASSED
- **Tool:** `/home/jetson/biped_ws/servo_controller.py`
- **Result:** Successfully sent commands to servo A, confirmed serial communication

#### Test 2: ROS2 Integration
- **Status:** ✓ PASSED
- **Tool:** `test_servo_movement.py`
- **Result:** All 15 servos tested in sequence
- Each servo moved through: center → +45° → -45° → center
- Commands successfully transmitted via ROS2 topics to hardware

#### Test 3: Full System Launch
- **Status:** ✓ PASSED
- All nodes running correctly:
  - `/robot_state_publisher` - Publishing transforms
  - `/joint_state_publisher_gui` - Publishing joint states
  - `/servo_driver` - Receiving and processing commands
- Topics active: `/joint_states`, `/robot_description`, `/tf`, `/tf_static`

### System Architecture (Implemented)

```
┌─────────────────────────────────────────────────────────────┐
│                   Jetson Nano (ROS2 Humble)                 │
│                                                             │
│  ┌──────────────────┐    ┌────────────────────────┐        │
│  │ Joint State      │───>│ /joint_states Topic    │        │
│  │ Publisher GUI    │    └──────────┬─────────────┘        │
│  └──────────────────┘               │                      │
│                                     │                      │
│  ┌──────────────────┐              │                      │
│  │ Robot State      │              │                      │
│  │ Publisher (URDF) │              │                      │
│  └──────────────────┘              │                      │
│                                     ▼                      │
│                          ┌──────────────────────┐          │
│                          │  Servo Driver Node   │          │
│                          │  - Subscribes to     │          │
│                          │    /joint_states     │          │
│                          │  - Converts rad→deg  │          │
│                          │  - Sends serial cmds │          │
│                          └──────────┬───────────┘          │
└─────────────────────────────────────┼──────────────────────┘
                                      │
                           USB (/dev/ttyUSB0)
                           via USB Hub
                                      │
                          ┌───────────▼────────────┐
                          │  STM32 Servo Board     │
                          │  CH340 Serial (9600)   │
                          │  Channels: A-O (15)    │
                          └───────────┬────────────┘
                                      │
                          ┌───────────▼────────────┐
                          │  15 Physical Servos    │
                          │  (Head, Arms, Legs)    │
                          └────────────────────────┘
```

### Additional Hardware
- **Time of Flight Camera:** Connected to Jetson, mounted on head servo
  - Note: ToF camera driver integration pending

### Current System Status
✓ **Hardware Control:** FULLY FUNCTIONAL
- All 15 servos responding to ROS2 commands
- Serial communication stable
- Joint control through GUI working
- Ready for locomotion algorithm development

### Quick Start Commands

```bash
# Build workspace
cd /home/jetson/biped_ws
source /opt/ros/humble/setup.bash
colcon build

# Launch robot control (no display needed for basic operation)
source install/setup.bash
ros2 launch humanoid_hardware robot_control.launch.py use_rviz:=false

# In another terminal: Test all servos
source /opt/ros/humble/setup.bash
python3 /home/jetson/biped_ws/test_servo_movement.py

# Manually control a joint
ros2 topic pub --once /joint_states sensor_msgs/msg/JointState \
  "{name: ['head_pan_joint'], position: [0.5]}"
```

### Next Development Steps
1. **ToF Camera Integration**
   - Create ROS2 node for ToF sensor
   - Publish depth/distance data

2. **Locomotion Controllers**
   - Walking gait patterns
   - Balance control algorithms
   - Inverse kinematics

3. **Sensor Integration**
   - IMU for balance feedback
   - Current sensing for servo health

4. **High-Level Control**
   - Behavior state machine
   - Autonomous navigation
   - Remote control interface

---

## Update: 2026-01-04 - URDF Cart System Added

### Tethered Support Cart Implementation
**Status:** ✓ Complete, Isaac Sim ready

**Added to URDF:**
- 25cm × 25cm red cart platform (2.5kg total mass)
- 16 wheels (4 corners × 4 wheels) with continuous rotation
- Elastic tether: Prismatic joint, 10cm rest, ±20cm range, damping=5.0
- Electronics platform: Jetson, servo board, 2× batteries, USB hub

**Key Features:**
- Collision/inertia on all components
- Flexible connection (cart slides back/forth on elastic string)
- Attached at robot waist, trails behind during walking
- Physics-ready for Isaac Sim simulation

**Validation:**
```bash
check_urdf humanoid.urdf  # ✓ 33 links, 32 joints
colcon build              # ✓ 0.23s
```

**Documentation:** See `CART_SYSTEM.md` for complete technical details.

---
