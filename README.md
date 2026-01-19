# Biped Robot

Train in Isaac Sim, deploy to Jetson hardware.

---

## Current Status (2026-01-19)

**✅ Robot is UPRIGHT and all parts visible!**

12 body parts as colored cubes in standing humanoid pose:
- 🔴 **RED** - Torso (center, z=0)
- 🟡 **YELLOW** - Head (above torso, z=0.3) - facing forward
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
./run_isaac.sh test_humanoid_visible.py
```

---

## Quick Start

### View Robot in Isaac Sim

```bash
cd /home/kenpeter/work/biped_robot
./run_isaac.sh test_humanoid_visible.py
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
│   ├── humanoid_articulated.usda   # Robot USD with 17 DOF physics + GLB mesh
│   └── humanoid.glb                 # 3D mesh visual (from Blender)
│
├── src/
│   ├── humanoid_description/        # URDF/ROS robot description
│   └── humanoid_hardware/           # ROS 2 driver for Jetson
│
├── test_humanoid_visible.py        # Test robot in Isaac Sim (with wave motion)
├── isaac_sim_training_env.py       # RL training environment
├── train_humanoid.py                # Training script
├── verify_hardware.py               # Hardware test for Jetson
│
├── run_isaac.sh                     # Isaac Sim launcher script
├── README.md                        # This file
├── MEMORY.md                        # Development notes
└── CLAUDE.md                        # Claude AI instructions
```

---

## Robot Structure

**17 DOF Articulated Humanoid:**
- HEAD: 1 joint (head_joint - Z axis rotation)
- LEFT ARM: 3 joints (shoulder_pitch, shoulder_roll, forearm_roll)
- RIGHT ARM: 3 joints (shoulder_pitch, shoulder_roll, forearm_roll)
- LEFT LEG: 5 joints (hip_roll, hip_pitch, knee_pitch, ankle_pitch, foot_roll)
- RIGHT LEG: 5 joints (hip_roll, hip_pitch, knee_pitch, ankle_pitch, foot_roll)

**Physics:**
- Flat sibling structure (all links as siblings under /Humanoid)
- GLB mesh for visuals
- USD articulation for physics simulation
- PD controllers on all joints (damping=10, maxForce=100)

---

## Development Commands

```bash
# Test robot with GUI (wave motion demo)
./run_isaac.sh test_humanoid_visible.py

# Run RL training environment
./run_isaac.sh isaac_sim_training_env.py

# Train robot (PPO/SAC)
./run_isaac.sh train_humanoid.py

# Check USD file structure
head -100 models/humanoid_articulated.usda

# Verify hardware on Jetson
python3 verify_hardware.py
```

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
