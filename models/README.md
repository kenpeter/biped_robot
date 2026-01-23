# Biped Robot - 15 DOF Humanoid

## Project Goal

**Train a walking model in Isaac Sim/Lab, then deploy to Jetson Nano for real robot locomotion.**

1. **Train** walking behavior in Isaac Sim physics simulation
2. **Export** trained model (neural network weights)
3. **Deploy** to Jetson Nano hardware for real-world control

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
- Position servos: 500-2500 (0-180 degrees)
- **Head servo (channel 0): Continuous rotation servo** (360° capable)
- Binary protocol: `[0x55, 0x55, 0x08, 0x03, 0x01, time_lo, time_hi, servo_id, pos_lo, pos_hi]`

**Sensors:**
- FLIR thermal camera (mounted on head servo), USB to Jetson

---

## Robot Model

**Location:** `models/humanoid.usda`

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
└── models/
    ├── # Neural Network
    │   head_model.py           # Network architecture
    │   head_model_weights.json # Trained weights
    │
    ├── # Training (Isaac Sim)
    │   train_head.py           # Train in Isaac Sim
    │   run_isaac.sh            # Isaac Sim launcher
    │
    ├── # Robot Models
    │   humanoid.usda           # 15-DOF full model
    │   humanoid_simple.usda    # Simplified model
    │   head_robot.usda         # Head servo (1 DOF)
    │   humanoid.glb            # 3D mesh (35 parts)
    │
    ├── # Deployment (Jetson)
    │   deploy_head_jetson.py   # Deploy trained model
    │   move_head.py            # Manual servo control
    │   calibrate_head.py       # Calibration utility
    │   verify_hardware.py      # Hardware check
    │
    └── # Docs
        README.md
        CLAUDE.md
```

---

## Quick Start

### Train Head Servo Model

```bash
cd /home/kenpeter/work/biped_robot/models
./run_isaac.sh train_head.py
```

### Manual Head Servo Control (Jetson)

The head servo is a **continuous rotation servo** (360° capable), not a positional servo.

```bash
cd /home/jetson/work/biped_robot/models

python3 move_head.py              # show status
python3 move_head.py stop         # stop rotation
python3 move_head.py left 90      # rotate left 90 degrees
python3 move_head.py right 45     # rotate right 45 degrees
python3 move_head.py left         # rotate left continuously
python3 move_head.py right        # rotate right continuously
python3 move_head.py speed 1600   # set raw speed value
python3 move_head.py off          # release servo (torque off)
```

**Continuous rotation servo characteristics (robot's perspective):**
- Deadband (stop): 1440-1558
- Above 1558: robot looks left (higher = faster)
- Below 1440: robot looks right (lower = faster)
- Timing: ~6 seconds for full 360° rotation

### Deploy to Jetson

Copy to Jetson:
```bash
scp head_model.py train_head.py deploy_head_jetson.py jetson:/home/jetson/work/biped_robot/models/
```

Run on Jetson:
```bash
python3 deploy_head_jetson.py
```

### View Full Robot

```bash
./run_isaac.sh load_humanoid.py
```

---

## Workflow: Train → Deploy

### Head Servo Example (Complete Pipeline)

```bash
# 1. Train in Isaac Sim (on development machine)
cd /home/kenpeter/work/biped_robot/models
./run_isaac.sh train_head.py

# 2. Copy trained model to Jetson
scp head_model*.json jetson:/home/jetson/work/biped_robot/models/

# 3. Deploy on Jetson (controls real servo on /dev/ttyUSB1)
ssh jetson
cd /home/jetson/work/biped_robot/models
python3 deploy_head_jetson.py
```

### Other Training Scripts

```bash
./run_isaac.sh demo_head.py      # Visualize ±30° oscillation
./run_isaac.sh load_humanoid.py  # View full 15-DOF humanoid
```

### View Full Robot
```bash
./run_isaac.sh load_humanoid.py  # 15-DOF humanoid model
```

---

## Isaac Sim Joint Tracking Issue (RESOLVED)

### Problem
Joint position control in Isaac Sim exhibited poor tracking accuracy (e.g., commanding 20° resulted in only ~1.6° to 4° movement).

### Root Causes
1.  **Extreme Mass Ratio:** The robot base was set to 1000kg while the head was 0.5kg (2000:1 ratio). This caused the physics solver to dampen the interaction, absorbing the drive force.
2.  **Incorrect Control API:** Using `set_joint_positions()` (teleportation) instead of driving the joint with physics forces.

### The Fix
1.  **Corrected Mass:** Reduced base mass to **10kg** in `head_robot.usda` to create a stable ~20:1 mass ratio.
2.  **Corrected API:** Switched to `_articulation_view.set_joint_position_targets()` to properly drive the joints using the PhysX solver.

```python
# CORRECT - Access underlying view for proper drive targeting
robot._articulation_view.set_joint_position_targets(
    positions=np.array([target_rad]),
    joint_indices=np.array([0])  # specify joint index
)
```

### Verification
Run the comparison script to see the fix in action:
```bash
cd models
./run_isaac.sh compare_old_vs_new.py
```
**Expected Result:** The "NEW method" should show ~100% tracking accuracy (e.g., 20° command = 20° actual).

---

## Fixed Base Articulation (RESOLVED)

### Problem
When testing the head servo, the base would fly away during rotation, moving up to 101+ meters despite `fixedBase=1` being set in the USD file.

### Root Cause
The `physicsArticulation:fixedBase` attribute alone doesn't reliably fix articulations in PhysX. The base was being treated as a dynamic rigid body that reacted to joint forces.

### The Fix
Create a **Fixed joint to the world** (leave `physics:body0` empty to connect to world):

```usd
def PhysicsFixedJoint "world_base_joint"
{
    prepend rel physics:body1 = </Robot/base>
    point3f physics:localPos0 = (0, 0, 0)
    point3f physics:localPos1 = (0, 0, 0)
}
```

This is the correct PhysX method for creating fixed articulations (like robot arms bolted to the floor).

### Verification
```bash
cd models
./run_isaac.sh test_fixed_base.py
```
**Expected Result:** Base stays at (0, 0, 0.5) with 0.000000m movement while head rotates 360°.

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
