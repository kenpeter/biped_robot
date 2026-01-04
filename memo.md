# Biped Workspace Progress Memo

**Last Updated:** 2026-01-04

---

## Current Status

### ✓ Software Implementation - COMPLETE
- ROS 2 packages built and functional
- URDF model with cart system ready for Isaac Sim
- All software components working correctly

### ✗ Hardware Control - NOT WORKING
**CRITICAL ISSUE:** Servos not responding to commands despite software working correctly.

---

## System Architecture

### Working Components

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
│                          │  ✓ Software working  │          │
│                          │  ✓ Commands sent     │          │
│                          └──────────┬───────────┘          │
└─────────────────────────────────────┼──────────────────────┘
                                      │
                           USB (/dev/ttyUSB0)
                           CH340 Serial @ 9600 baud
                                      │
                          ┌───────────▼────────────┐
                          │  Servo Control Board   │
                          │  ✓ Receives commands   │
                          │  ✓ Sends responses     │
                          │  ✗ SERVOS NOT MOVING   │
                          └────────────────────────┘
```

### Hardware Configuration

**Connected USB Devices:**
- `/dev/ttyUSB0`: CH340 serial converter (1a86:7523) - **Servo board**
- `/dev/ttyUSB1`: SIPEED Meta Sense Lite (ToF camera)
- `/dev/ttyUSB2`: SIPEED Meta Sense Lite (ToF camera, 2nd interface)
- HID device: STMicroelectronics (0483:5750) - Not used for servo control

---

## ROS 2 Packages

### 1. humanoid_description
**Status:** ✓ Complete

**Contents:**
- URDF model with 15 DOF (A-O servo channels)
- Cart system with elastic tether (Isaac Sim ready)
- RViz configuration
- Launch file: `display.launch.py`

**Joint Mapping:**
| Joint | Servo | Body Part |
|-------|-------|-----------|
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

### 2. humanoid_hardware
**Status:** ✓ Software complete, ✗ Hardware not responding

**Location:** `/home/jetson/biped_ws/src/humanoid_hardware/`

**Features:**
- Subscribes to `/joint_states` topic
- Converts radians to servo angles (0-180°)
- Sends serial commands via `/dev/ttyUSB0`
- Configurable: serial_port, baud_rate, timeout

**Launch:**
```bash
cd /home/jetson/biped_ws
source install/setup.bash
ros2 launch humanoid_hardware robot_control.launch.py use_rviz:=false
```

---

## Servo Control Troubleshooting (2026-01-04)

### Protocols Tested

#### 1. ASCII Protocol (Original Documentation)
- **Format:** `$<servo><angle:03d>#`
- **Example:** `$A090#` (servo A to 90 degrees)
- **Tested:** With and without newline (`\n`)
- **Result:** Commands sent successfully, no servo movement

#### 2. Binary LSC Protocol (Hiwonder-style)
- **Format:** `0x55 0x55 <length> <cmd> <params>`
- **Example:** `55 55 08 03 E8 03 01 DC 05` (servo 1 to 1500µs)
- **Result:** Board responds with binary data, no servo movement

### Configurations Tested

**Ports:** `/dev/ttyUSB0`, `/dev/ttyUSB1`, `/dev/ttyUSB2`
**Baud Rates:** 9600, 19200, 38400, 57600, 115200
**Servo Channels:** A, B, C, D, E, R (and IDs 1-4 for binary)
**Command Formats:** 5 different variations tested

**Total Tests:** 150+ combinations
**Movement Detected:** NONE

### Key Findings

✓ **Software is correct:**
- Serial port opens successfully
- Commands sent without errors
- Board receives and responds to commands (binary response observed)

✗ **Hardware issue:**
- No servo movement despite valid communication
- Board sends binary responses (proves communication works)
- Classic symptom: **Missing servo power supply**

### Most Likely Cause: NO SERVO POWER

**Evidence:**
1. Communication works (board responds)
2. No errors in software
3. User reported "servo 18 moves on power-up" (proves servos can work)
4. No movement with any protocol/configuration

**USB power alone is NOT sufficient for servos!**

---

## CRITICAL: Hardware Checklist

### 🔴 MUST CHECK:

1. **External Power Supply**
   - Is there a power adapter/battery connected to the servo board?
   - Voltage: Should be 5-6V for servos
   - Check for barrel jack, screw terminals, or battery connector
   - **This is the #1 most likely issue**

2. **Physical Servo Connections**
   - Are actual servo motors plugged into the board?
   - Which channels (1-24) have servos connected?
   - User mentioned "servo 18" - is it currently connected?

3. **Board Power Indicators**
   - Are there LED lights on the servo board?
   - What color? Steady or blinking?
   - Some boards have LED2 that blinks on valid commands

4. **Board Model/Documentation**
   - What is the exact model number?
   - Similar to Yahboom/Hiwonder 24-channel boards but not exact match
   - CH340 USB-serial converter (not FTDI)

---

## Test Scripts Created

Located in `/home/jetson/`:

1. **log_servo_test.py** - Automated test with logging
2. **simple_servo_test.py** - Quick interactive test
3. **test_binary_protocol.py** - LSC binary protocol test
4. **auto_servo_test.py** - Exhaustive automated test
5. **HARDWARE_CHECKLIST.md** - Hardware troubleshooting guide

---

## Quick Start Commands

### Launch Robot Control System
```bash
cd /home/jetson/biped_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Without RViz (headless)
ros2 launch humanoid_hardware robot_control.launch.py use_rviz:=false

# With RViz (requires display)
ros2 launch humanoid_hardware robot_control.launch.py use_rviz:=true

# Custom serial port
ros2 launch humanoid_hardware robot_control.launch.py serial_port:=/dev/ttyUSB1
```

### Test Servo Manually
```bash
# Quick test
python3 /home/jetson/simple_servo_test.py

# Automated comprehensive test
python3 /home/jetson/log_servo_test.py

# Test specific servo
ros2 topic pub --once /joint_states sensor_msgs/msg/JointState \
  "{name: ['head_pan_joint'], position: [0.5]}"
```

### Rebuild Workspace
```bash
cd /home/jetson/biped_ws
source /opt/ros/humble/setup.bash
colcon build
```

---

## Next Steps

### IMMEDIATE (Before continuing software development):
1. **Verify servo power supply is connected and working**
2. **Check which channels have physical servos connected**
3. **Test with manufacturer's PC software (if available)**
4. **Identify exact board model number for documentation**

### After Hardware Fixed:
1. **ToF Camera Integration** - Create ROS2 node for head-mounted sensor
2. **Locomotion Controllers** - Walking gaits, balance control
3. **IMU Integration** - Balance feedback system
4. **Simulation** - Test algorithms in Isaac Sim with cart system

---

## Technical Reference

### Servo Command Protocol (Documented but not working)
- **Baud Rate:** 9600
- **Format:** `$<servo><angle>#` (e.g., `$A090#`)
- **Servos:** A-X for channels 1-24
- **Angles:** 000-180 (3 digits)

### Current ROS Driver Settings
- **File:** `src/humanoid_hardware/humanoid_hardware/servo_driver.py`
- **Port:** `/dev/ttyUSB0` (default, configurable)
- **Baud:** 9600
- **Command:** `${servo}{angle:03d}#\n`

---

## Notes

- ROS 2 Distribution: Humble
- Platform: Jetson Nano (ARM64)
- URDF: 15 DOF (expandable to 24 DOF)
- Cart System: Documented in `CART_SYSTEM.md`
- **Current blocker:** Hardware power/connection issue preventing servo movement
