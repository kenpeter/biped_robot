# Biped Robot - 15 DOF Humanoid

## Project Goal

**Train a walking model in Isaac Sim/Lab, then deploy to Jetson Nano for real robot locomotion.**

1. **Train** walking behavior in Isaac Sim physics simulation
2. **Export** trained model
3. **Deploy** to Jetson Nano hardware for real-world walking

---

## Hardware Setup

**Physical Robot:**
- **15 DOF humanoid** with servo-driven joints
- **LEGO cart with 4 wheels** containing all electronics
- **2 strings** connecting robot waist to LEGO cart (tether for power/control)

**Electronics (on LEGO cart):**
- Jetson Nano (main computer)
- Hiwonder LSC-24 servo board (servo controller)
- ROS master board
- USB hub
- Power supply

**Servos:**
- Hiwonder servos on /dev/ttyUSB1 @ 9600 baud
- Position range: 500-2500 (0-180 degrees)
- Binary protocol: `[0x55, 0x55, 0x08, 0x03, 0x01, time_lo, time_hi, servo_id, pos_lo, pos_hi]`

**Sensors:**
- FLIR thermal camera (mounted on head servo), USB to Jetson

---

## Robot Model

**Location:** `src/humanoid_description/usd/humanoid.usda`

**Mesh:** `models/humanoid.glb` (35 geometries from Blender scan)

**Structure (15 DOF):**
```
/World/robot
  - mesh (GLB reference)
  - head (1 DOF) - Y axis, ±60°
  - l_shoulder (2 DOF) - Y axis, ±90° + X axis, ±180°
  - r_shoulder (2 DOF) - Y axis, ±90° + X axis, ±180°
  - l_hip (4 DOF) - X axis, ±45° + Y axis, -120°~0° + Y axis, ±45° + X axis, ±30°
  - r_hip (4 DOF) - X axis, ±45° + Y axis, -120°~0° + Y axis, ±45° + X axis, ±30°
```

---

## Quick Start

### View Robot in Isaac Sim

```bash
cd /home/kenpeter/work/biped_robot/models
./run_isaac.sh load_humanoid.py
```

**Expected:** Isaac Sim opens showing humanoid robot with GLB mesh.

---

## Project Structure

```
biped_robot/
├── models/
│   ├── 15dof.png                      # Robot diagram (15 DOF)
│   ├── humanoid.glb                   # 3D mesh visual (from scanner)
│   ├── head_robot.usda                # Head servo test robot (1 DOF)
│   ├── load_humanoid.py               # Launch Isaac Sim with full robot
│   ├── demo_head_servo.py             # Demo head servo movement
│   ├── train_head_servo.py            # Train head servo with RL
│   └── run_isaac.sh                   # Isaac Sim launcher script
│
├── src/
│   ├── humanoid_description/
│   │   └── usd/
│   │       └── humanoid.usda          # 15-DOF full robot model
│   └── humanoid_hardware/             # ROS 2 driver for Jetson
│
├── README.md                          # This file
└── CLAUDE.md                          # Claude AI instructions
```

---

## Joint Mapping

| Link | Joint | Axis | Range |
|------|-------|------|-------|
| head | head_joint | Y | ±60° |
| l_shoulder | l_shoulder_pitch | Y | ±90° |
| l_forearm | l_forearm_roll | X | ±180° |
| r_shoulder | r_shoulder_pitch | Y | ±90° |
| r_forearm | r_forearm_roll | X | ±180° |
| l_hip | l_hip_roll | X | ±45° |
| l_knee | l_knee_pitch | Y | -120°~0° |
| l_ankle | l_ankle_pitch | Y | ±45° |
| l_foot | l_foot_roll | X | ±30° |
| r_hip | r_hip_roll | X | ±45° |
| r_knee | r_knee_pitch | Y | -120°~0° |
| r_ankle | r_ankle_pitch | Y | ±45° |
| r_foot | r_foot_roll | X | ±30° |

**Servo Channels (Hiwonder LSC-24):**
- Channel 0: head
- Channels 1-7: left body
- Channels 12-19: right body

---

## Commands

```bash
# Navigate to models directory
cd /home/kenpeter/work/biped_robot/models

# Launch Isaac Sim with humanoid robot
./run_isaac.sh load_humanoid.py

# Train head servo
./run_isaac.sh train_head_servo.py

# Demo head servo movement
./run_isaac.sh demo_head_servo.py
```

---

## Troubleshooting

### Robot not visible?
- Press F in Isaac Sim viewport to frame
- Check console for USD loading errors

### Mesh not loading?
- Verify `models/humanoid.glb` exists (58KB)
- Check GLB reference path in humanoid.usda

---

See CLAUDE.md for development workflow.
