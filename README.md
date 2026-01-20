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

## Current Status (2026-01-19)

**✅ Robot model visible in Isaac Sim!**

Simplified robot structure as colored cubes in standing humanoid pose:
- 🔴 **RED** - Torso (center, z=0.3m)
- 🟡 **YELLOW** - Head (above torso)
- 🟢 **GREEN** - Left arm (at sides) + Left leg (standing vertical)
- 🔵 **BLUE** - Right arm (at sides) + Right leg (standing vertical)

---

## Robot File Location

```
/home/kenpeter/work/biped_robot/models/humanoid_articulated.usda
```

**Test with GUI:**
```bash
cd /home/kenpeter/work/biped_robot
./models/run_isaac.sh models/test_humanoid_visible.py
```

---

## Quick Start

### View Robot in Isaac Sim

```bash
cd /home/kenpeter/work/biped_robot
./models/run_isaac.sh models/test_humanoid_visible.py
```

**Expected:** Isaac Sim opens with UI showing humanoid robot:
- Red cube at center (torso)
- Yellow cube above (head)
- Green cubes on left (arm + leg)
- Blue cubes on right (arm + leg)
- All parts spread out in humanoid stick-figure formation

---

## Project Structure

```
biped_robot/
├── models/
│   ├── humanoid_articulated.usda      # Robot USD with 17 DOF physics
│   ├── humanoid.glb                   # 3D mesh visual (from Blender)
│   ├── humanoid_robot.blend           # Blender source file
│   ├── test_humanoid_visible.py       # Test robot in Isaac Sim
│   ├── isaac_sim_training_env.py      # RL training environment
│   ├── train_humanoid.py              # Training script
│   └── humanoid_direct_env.py         # Isaac Lab DirectRLEnv
│
├── src/
│   ├── humanoid_description/          # URDF/ROS robot description
│   └── humanoid_hardware/             # ROS 2 driver for Jetson
│
├── verify_hardware.py                 # Hardware test for Jetson
├── README.md                          # This file
├── MEMORY.md                          # Development notes
└── CLAUDE.md                          # Claude AI instructions
```

---

## Robot Structure (15 DOF)

**Joint Breakdown:**
- **HEAD:** 1 servo - head_joint (left/right rotation, Z-axis)
- **LEFT ARM:** 3 servos
  - l_shoulder_pitch (forward/backward, Y-axis)
  - l_shoulder_roll (close/away from body, X-axis)
  - l_forearm_roll (close/away from body, X-axis)
- **RIGHT ARM:** 3 servos
  - r_shoulder_pitch (forward/backward, Y-axis)
  - r_shoulder_roll (close/away from body, X-axis)
  - r_forearm_roll (close/away from body, X-axis)
- **LEFT LEG:** 4 servos
  - l_hip_roll (close/away from body, X-axis)
  - l_knee_pitch (forward/backward, Y-axis)
  - l_ankle_pitch (forward/backward, Y-axis)
  - l_foot_roll (close/away from body, X-axis)
- **RIGHT LEG:** 4 servos
  - r_hip_roll (close/away from body, X-axis)
  - r_knee_pitch (forward/backward, Y-axis)
  - r_ankle_pitch (forward/backward, Y-axis)
  - r_foot_roll (close/away from body, X-axis)

**Servo Channels (Hiwonder LSC-24):**
- Channel 0: head
- Channels 1-7: left body (arm + leg)
- Channels 12-19: right body (arm + leg)

**Physics (Isaac Sim):**
- Flat sibling structure (all links as siblings under /Humanoid)
- LEGO cart with 4 wheels connected via 2 strings to waist
- USD articulation for physics simulation
- PD controllers on all joints

---

## Current Training Plan (2026-01-20)

**Objective:** Train head servo (channel 0) to move left/right in 10-degree increments

**Approach:**
1. **Isaac Sim Training**
   - Create simple RL environment with head joint only
   - Observations: joint angle + velocity (2 values)
   - Actions: target angle change (discrete: -10°, 0°, +10°)
   - Reward: smooth movement to target angles
   - Train with PPO algorithm

2. **Export & Deploy**
   - Save trained PyTorch model
   - Create ROS 2 node on Jetson
   - Convert model output → Hiwonder servo commands
   - Test on real hardware (channel 0, /dev/ttyUSB1)

**Why start with head?**
- Simplest DOF (1 joint, channel 0)
- Validates full pipeline: Sim → Train → Deploy → Hardware
- Foundation for full 15-DOF walking later

---

## Development Commands

```bash
# View robot in Isaac Sim (simple viewer)
./models/run_isaac.sh models/view_robot.py

# Train head servo (50 episodes, saves head_servo_policy.pth)
./models/run_isaac.sh models/train_head_servo.py

# Check USD file structure
head -100 models/simple_robot.usda

# Verify hardware on Jetson
python3 verify_hardware.py
```

**Models directory (2 scripts, 242 lines total):**
- `view_robot.py` (41 lines) - Simple robot viewer
- `train_head_servo.py` (201 lines) - Train head servo with RL
- `simple_robot.usda` - 15 DOF robot + LEGO cart + 2 strings

---

## Troubleshooting

### Robot sideways or half in ground?
- Robot uses Blender Y-up coordinate system
- Runtime fix applied: +90° X rotation at spawn time
- Position: 15cm above ground (`[0, 0, 0.15]`)

### Joints don't move visually?
- GLB mesh may not be bound to physics joints
- Joints work (check DOF count: 17) but mesh stays static
- This is a known limitation - mesh binding needs setup

### Robot not visible?
- Press F in Isaac Sim viewport to frame robot
- Check console for USD loading errors
- Verify `models/humanoid.glb` exists (57KB)

---

See MEMORY.md for detailed development history.
