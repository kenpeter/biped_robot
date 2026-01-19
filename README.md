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
│   ├── humanoid_articulated.usda   # ✅ Robot USD with 12 cubes in flat structure
│   └── robot.png                    # Reference image for robot structure
│
├── src/
│   ├── humanoid_description/        # URDF/ROS robot description
│   └── humanoid_hardware/           # ROS 2 driver for Jetson
│
├── setup_isaac_sim_robot.py        # Generates USD from robot.png structure
├── test_humanoid_visible.py        # Test script to view robot with GUI
├── run_isaac.sh                     # Isaac Sim launcher
│
├── README.md                        # This file
├── MEMORY.md                        # Development notes
└── CLAUDE.md                        # Claude AI instructions
```

---

## Robot Hierarchy (FLAT sibling structure)

```
/Robot/
├── torso (RED, z=0, size=0.24) [ArticulationRoot]
├── head (YELLOW, z=0.3, size=0.16)
├── l_shoulder (GREEN, x=-0.3, z=0.1, size=0.12)
├── l_elbow (GREEN, x=-0.6, z=0.1, size=0.10)
├── r_shoulder (BLUE, x=0.3, z=0.1, size=0.12)
├── r_elbow (BLUE, x=0.6, z=0.1, size=0.10)
├── l_hip (GREEN, x=-0.15, z=-0.3, size=0.12)
├── l_knee (GREEN, x=-0.15, z=-0.6, size=0.10)
├── l_ankle (GREEN, x=-0.15, z=-0.9, size=0.10)
├── r_hip (BLUE, x=0.15, z=-0.3, size=0.12)
├── r_knee (BLUE, x=0.15, z=-0.6, size=0.10)
└── r_ankle (BLUE, x=0.15, z=-0.9, size=0.10)
```

**Total: 12 cubes with 11 revolute joints**

---

## Development Commands

```bash
# Regenerate robot USD from robot.png structure
./run_isaac.sh setup_isaac_sim_robot.py

# Test robot with GUI (view movement)
./run_isaac.sh test_humanoid_visible.py

# Check USD file structure
head -100 models/humanoid_articulated.usda

# Verify cubes in flat structure
grep -E "(def Xform|def Cube|xformOp:translate)" models/humanoid_articulated.usda
```

---

## Troubleshooting

### Robot not visible?
- Check Isaac Sim viewport - may need to zoom/pan
- Click "Show All" in viewport or press F
- Verify USD was regenerated with cubes (not spheres)

### Parts in wrong position?
- Regenerate USD: `./run_isaac.sh setup_isaac_sim_robot.py`
- Should match robot.png with cubes spread out in flat sibling structure

### Parts snap together?
- This means nested hierarchy was used - regenerate with flat structure
- All body parts must be siblings under /Robot with absolute world positions

---

See MEMORY.md for detailed development history.
