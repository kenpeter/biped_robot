# Biped Robot - 15 DOF Humanoid

## Project Goal

**Train a walking model in Isaac Sim/Lab, then deploy to Jetson Nano for real robot locomotion.**

1. **Train** walking behavior in Isaac Sim physics simulation
2. **Export** trained model (neural network weights)
3. **Deploy** to Jetson Nano hardware for real-world control

---

## Head Servo Model (Example: Train → Deploy)

Simple neural network trained in simulation, deployed to Jetson to control servo on `/dev/ttyUSB1`.

### Files

| File | Purpose |
|------|---------|
| `models/head_model.py` | Neural network architecture (8 hidden neurons) |
| `models/train_head.py` | Train in Isaac Sim, collect data, save weights |
| `models/deploy_head_jetson.py` | Load weights, control real servo |

### Model Architecture

```
Input:  target_angle (-30° to +30°)
        ↓
Hidden: 8 neurons, ReLU activation
        ↓
Output: position_offset (-1 to +1)
        ↓
Servo:  1500 + offset * 1000 → position [500, 2500]
```

### Training Workflow

**Step 1: Train in Isaac Sim**
```bash
cd /home/kenpeter/work/biped_robot/models
./run_isaac.sh train_head.py
```
- Collects data at ±30°, ±20°, ±10°, 0° from simulation
- Trains neural network (2000 epochs)
- Saves `head_model_weights.json` and `head_model_config.json`

**Step 2: Copy to Jetson**

Option A - Google Drive (recommended):
```bash
# Files auto-sync if using Drive sync
# On Jetson, download:
cd /home/jetson/work/biped_robot/models
wget "https://drive.google.com/uc?export=download&id=YOUR_FILE_ID" -O head_model_weights.json
```

Option B - USB flash drive:
```bash
# On your PC
cp head_model.py head_model_weights.json /media/yourname/USB/
# Plug into Jetson, copy files
```

Option C - Direct network:
```bash
scp head_model.py head_model_weights.json jetson@jetson.local:/home/jetson/work/biped_robot/models/
```

**Step 3: Deploy on Jetson**
```bash
# On Jetson
cd /home/jetson/work/biped_robot/models
python3 deploy_head_jetson.py
```

### Test Trained Model

```bash
python3 -c "
from head_model import HeadServoModel
model = HeadServoModel()
model.load('head_model_weights.json')
for angle in [-30, -15, 0, 15, 30]:
    pos = model.predict(angle)
    print(f'{angle:+3d}° -> {pos:.0f}')
"
```

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
│   ├── head_model.py                 # Neural network for head servo
│   ├── train_head.py                 # Train in Isaac Sim
│   ├── deploy_head_jetson.py         # Deploy to Jetson hardware
│   ├── head_robot.usda               # Head servo robot (1 DOF, fixed base)
│   ├── load_humanoid.py              # View full 15-DOF robot in Isaac Sim
│   ├── demo_head.py                  # Demo: visualize head oscillation
│   ├── run_isaac.sh                  # Isaac Sim launcher
│   ├── humanoid.glb                  # 3D mesh (from 3D scanner)
│   └── 15dof.png                     # Robot diagram
│
├── src/
│   ├── humanoid_description/usd/humanoid.usda  # 15-DOF full model
│   └── humanoid_hardware/                      # ROS 2 Jetson driver
│
├── README.md                         # This file
└── CLAUDE.md                         # Claude AI instructions
```

---

## Quick Start

### View Robot in Isaac Sim

```bash
cd /home/kenpeter/work/biped_robot/models
./run_isaac.sh load_humanoid.py
```

**Expected:** Isaac Sim opens showing humanoid robot with GLB mesh.

### Train Head Servo Model

```bash
cd /home/kenpeter/work/biped_robot/models
./run_isaac.sh train_head.py
```

**Expected:** Robot oscillates ±30°, model trains, weights saved to JSON.

### Deploy to Jetson

```bash
# Copy to Jetson
scp head_model*.json jetson:/home/jetson/work/biped_robot/models/

# On Jetson
python3 deploy_head_jetson.py
```

**Expected:** Servo on channel 0 oscillates ±30°.

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
