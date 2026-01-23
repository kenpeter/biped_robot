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

## Matching Simulation Physics with Reality (Continuous Rotation Servos)

### Problem
Training a model in Isaac Sim for continuous rotation servo failed with:
1. **Movement too fast visually** - looked unrealistic
2. **Overshoot** - servo kept rotating past target (e.g., 30° became 180°+)
3. **Training failed** - NaN losses, model couldn't learn

### Root Cause
The default Isaac Sim joint configuration simulates **position-controlled servos** (stiff, fast), not **continuous rotation servos** (weak, velocity-controlled).

**Wrong Configuration:**
```python
stiffness = 10000  # Position control (fights against any movement)
damping = 1000     # High resistance
maxForce = 100     # Too weak to overcome damping
```

This resulted in:
- Simulation completing 90° in 0.467s vs real hardware 1.5s
- Gradient explosion in training → NaN losses
- Model learned wrong physics

### The Fix

**1. Joint Configuration (`head_robot.usda`)**
```python
# Continuous rotation servo (velocity-controlled, weak, slow)
stiffness = 0        # No position holding (free rotation)
damping = 1000       # High friction (slow, realistic movement)
maxForce = 200       # Weak motor (can't accelerate instantly)
mass = 2.0           # Heavier head (slower acceleration)
```

**2. Movement Control with Deceleration**
```python
# Monitor position while moving
for _ in range(max_steps):
    current_pos = robot.get_joint_positions()[0]
    error_deg = target_deg - current_pos

    # Stop when within 1° of target
    if abs(error_deg) < 1.0:
        break

    # Decelerate in last 5° to prevent overshoot
    if abs(error_deg) < 5.0:
        scale = abs(error_deg) / 5.0
        velocity = base_velocity * max(scale, 0.2)
    else:
        velocity = base_velocity
```

**3. Training Data Collection**
Train on **angle errors**, not timing errors:
```python
# Collect: angle_delta → angle_error
X_train.append([delta_deg])           # Input: -30°, +45°, etc.
y_train.append([actual - target])     # Output: +0.8°, -1.0°, etc.
```

### Results After Fix

**Movement Quality:**
- ✅ Stops at target ±30° (error ~0.8°, realistic)
- ✅ Smooth deceleration (no sudden stops)
- ✅ Realistic slow servo speed
- ✅ No overshoot past target

**Training Quality:**
```
Training converged: Loss 0.000039 (no NaN!)
Errors during training: ±1° (very good)

Test predictions:
  -90° → 1.531s right  (expected: 1.5s) ✅
  -45° → 0.753s right  (expected: 0.75s) ✅
   +0° → 0.021s left   (expected: 0.0s) ✅
  +45° → 0.767s left   (expected: 0.75s) ✅
  +90° → 1.515s left   (expected: 1.5s) ✅
```

### Key Lessons

1. **Match servo type in simulation:**
   - Continuous rotation → stiffness=0, high damping
   - Position servo → high stiffness, low damping

2. **Simulate weakness:**
   - Real cheap servos are slow and weak
   - Lower maxForce, increase mass/damping

3. **Stop at target:**
   - Velocity control needs position monitoring
   - Add deceleration zone to prevent overshoot

4. **Train on right metric:**
   - Continuous rotation: learn angle corrections
   - Position servos: learn timing corrections

5. **NO RESETS during training (CRITICAL):**
   - Physical resets (`set_joint_positions`) between movements cause physics glitches
   - test_head_speed.py works because it NEVER resets - just continuous movement
   - Let position accumulate naturally with error tracking
   - Only use ±30° movements like test_head_speed.py does

6. **Exact pattern matching:**
   - Training MUST use exact same sequence as test script: -30° → 0° → +30° → 0°
   - Must include 0.5s pause between moves (30 sim steps)
   - Movement function must be byte-for-byte identical
   - Different angles (±60°, ±90°, ±120°) cause drift even with resets

### What Made Training Finally Work

**Problem:** Training would drift past 180° while test_head_speed.py worked perfectly.

**Root Cause:** Physical resets between cycles caused physics solver instabilities.

**Solution:**
```python
# WRONG - causes drift:
for angle in [15, 30, 45, 60, 90, 120]:
    robot.set_joint_positions([0.0])  # ❌ Reset breaks physics
    for target in [-angle, 0, angle, 0]:
        move_to(target)

# CORRECT - works like test_head_speed.py:
current = 0.0
for cycle in range(10):
    for target in [-30.0, 0.0, 30.0, 0.0]:  # ✅ Only ±30°, no resets
        current = move_to(target, current)
        pause(0.5s)
```

**Key differences:**
- ❌ Multiple angles (±15° to ±120°) → use only ±30°
- ❌ Physical resets → continuous movement only
- ❌ Multiple test angles per run → single ±30° pattern repeated
- ✅ Exact copy of test_head_speed.py movement logic
- ✅ Pauses between moves (0.5s)
- ✅ Accumulated position tracking (no resets)

### Deployment Drift Issue (Real Robot)

**Problem:** When deployed to Jetson with `--demo`, robot would drift left continuously toward 180°+ after 2-3 cycles.

**Root Cause:** Position tracking with assumed perfect accuracy:
```python
# WRONG - causes drift on real hardware:
def go_to(self, target_deg):
    delta = target_deg - self.current_angle
    rotate(delta)
    self.current_angle = target_deg  # ❌ Assumes perfect accuracy!

# Real servo has ±0.8° errors → accumulated over cycles → drift
```

**Solution:** Use relative movements WITHOUT position tracking:
```python
# CORRECT - no drift:
rotate(ser, 30, model)   # Relative: +30° from current position
rotate(ser, -30, model)  # Relative: -30° (back to center)
rotate(ser, -30, model)  # Relative: -30° (right 30°)
rotate(ser, 30, model)   # Relative: +30° (back to center)
# No position tracking → errors don't accumulate!
```

**Demo mode pattern (matches simulation):**
- left 30° → center → right 30° → center (repeat)
- Uses direct `rotate()` calls (relative movement)
- No `HeadController` position tracking
- Must train model first before deployment!

**Training → Deployment workflow:**
```bash
# 1. Train model (dev machine with Isaac Sim)
cd /home/kenpeter/work/biped_robot/models
./run_isaac.sh train_head.py
# Creates: head_model_weights.json

# 2. Copy to Jetson
scp head_model_weights.json deploy_head_jetson.py jetson:/home/jetson/work/biped_robot/models/

# 3. Run on real robot
ssh jetson
cd /home/jetson/work/biped_robot/models
python3 deploy_head_jetson.py --demo
```

### Industry Standard: Periodic Home Pose Recalibration

Based on research (TALOS humanoid, Boston Dynamics, industrial robots), drift compensation uses **periodic recalibration to known home pose**:

**Implementation:**
- Every 20-30 movements, robot returns to calibration pose
- Estimated drift at recalibration: ±16° (20 × 0.8°/movement)
- Manual or automatic (using sensors: IMU, force sensors, camera)

**Why this works:**
- Drift is bounded (never exceeds 20-30 movements of error)
- Works for ALL servos (not just head with camera)
- Industry proven approach (used by commercial humanoid robots)
- Simple to implement

**Example output:**
```
-> left 30°
-> center
... (20 movements) ...

[DRIFT COMPENSATION] 20 movements completed
[DRIFT COMPENSATION] Estimated drift: ±16.0°
[RECALIBRATE] Please manually center the head (forward facing)
[RECALIBRATE] Press Enter when centered...
[RECALIBRATE] ✓ Home pose restored, drift reset
```

**Calibration frequency guidelines:**

| Activity | Movements | Recalibrate Every |
|----------|-----------|-------------------|
| Walking | ~10/sec | 50 movements (5s) |
| Standing/gestures | ~1/sec | 30 movements (30s) |
| Demo movements | varies | 20 movements |

**Physical references for each joint type:**

1. **HEAD (1 servo):**
   - Reference: Forward facing
   - Detection: Thermal camera, manual centering, or IMU

2. **ARMS (6 servos):**
   - Reference: Hanging straight down (gravity)
   - Detection: IMU/accelerometer, mechanical stop, or manual

3. **LEGS (8 servos):**
   - Reference: Standing with feet flat on ground
   - Detection: Force sensors, IMU, mechanical stops (knee extended), or manual

**Future sensor upgrades (optional):**
- IMU per joint: ~$5-10 each = $75-150 total (eliminates drift)
- Absolute encoders: ~$20-30 per servo = $300-450 (no drift by design)
- Vision-based pose estimation: External camera + real-time processing

**Current implementation:** Manual recalibration every 20 movements for head servo. Easily extensible to all 15 servos using same pattern.

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
