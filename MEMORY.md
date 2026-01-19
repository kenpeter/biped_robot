# Biped Robot - Project Memory

## Current Status (2026-01-19)

**Progress:** Robot structure created but parts snap together

**✅ Fixed (2026-01-19):**
1. Flat hierarchy → Fixed: Links now properly nested (parent-child)
2. Rigid body errors → Fixed: Added XformStack reset for nested bodies
3. Critical errors eliminated → Robot loads and simulates

**⚠️ Remaining Problem:**
- "Disjointed body transforms" warnings for elbows, knees, ankles
- Robot parts snap together at torso location
- Only 1-2 cubes visible instead of 11 body parts
- Need to add joint localPos0/localPos1 attributes

---

## Recent Fixes (2026-01-19)

### Fix 1: Hierarchical Structure
- **File:** `setup_isaac_sim_robot.py`
- **Line:** ~68-72
- **Change:** Added proper parent-child nesting and XformStack reset
- **Result:** Robot has correct hierarchical structure

### Fix 2: Embedded Geometry
- **File:** `setup_isaac_sim_robot.py`
- **Change:** Created visible cubes directly in USD (no GLB reference)
- **Result:** Robot renders as colored cubes (RED torso, YELLOW head, GREEN/BLUE limbs)

### Fix 3: Joint Configuration
- **File:** `models/humanoid_articulated.usda`
- **Change:** Regenerated with correct drive parameters
- **Result:** All 11 joints recognized by ArticulationView

### Still Broken: Joint Local Positions
- **Problem:** Joints don't define attachment points (localPos0/localPos1)
- **Symptom:** "Disjointed body transforms" warnings
- **Result:** All body parts snap together at torso location
- **Fix needed:** Add `physics:localPos0` and `physics:localPos1` to each joint

---

## USD Joint Structure

Current (broken):
```
def PhysicsRevoluteJoint "l_shoulder_pitch" (
    rel physics:body0 = </Humanoid/torso>
    rel physics:body1 = </Humanoid/left_upper_arm>
    # Missing: localPos0 and localPos1!
)
```

Needed (fixed):
```
def PhysicsRevoluteJoint "l_shoulder_pitch" (
    rel physics:body0 = </Humanoid/torso>
    rel physics:body1 = </Humanoid/left_upper_arm>
    double3 physics:localPos0 = (0, 0.15, 0)    # Shoulder position on torso
    double3 physics:localPos1 = (0, -0.15, 0)   # Shoulder position on arm
)
```

---

## File Structure

```
biped_robot/
├── models/
│   ├── humanoid.glb                # Original Blender export (reference only)
│   ├── humanoid_articulated.usda   # ✅ Working USD with embedded geometry
│   ├── create_robot.py             # Blender robot generator
│   └── export_usd.py               # USD exporter
│
├── src/
│   ├── humanoid_description/       # Robot description (legacy)
│   └── humanoid_hardware/          # ROS 2 hardware driver (for Jetson)
│
├── setup_isaac_sim_robot.py        # ✅ USD generator with embedded geometry
├── test_humanoid_visible.py        # ✅ Test robot visibility and movement
├── isaac_sim_training_env.py       # Basic RL environment
├── humanoid_direct_env.py          # Isaac Lab DirectRLEnv (WIP)
├── train_humanoid.py               # RL training script (WIP)
├── verify_hardware.py              # Hardware diagnostic (Jetson)
├── run_isaac.sh                    # Isaac Sim launcher
│
├── README.md
├── CLAUDE.md
└── MEMORY.md
```

---

## Hardware (Jetson Deployment)

**Servo Board:** Hiwonder LSC-24
**Port:** `/dev/ttyUSB1` @ 9600 baud
**Protocol:** `[0x55, 0x55, 0x08, 0x03, 0x01, time_lo, time_hi, servo_id, pos_lo, pos_hi]`
**Range:** 500-2500 (0-180°)

**Critical Notes:**
- `/dev/ttyTHS1` locked by kernel - use USB adapters only
- LSC-24 switch must be ON (blue LED)
- Wiring: CP2102 RX → Board TX, TX → Board RX

---

## Quick Commands

```bash
# Desktop - Isaac Sim
./run_isaac.sh setup_isaac_sim_robot.py      # Generate USD with embedded geometry
./run_isaac.sh test_humanoid_visible.py      # Test robot (GUI mode, non-headless)

# Check USD structure
head -50 models/humanoid_articulated.usda    # View generated USD

# Jetson - Hardware Control
python3 verify_hardware.py                   # Test servos
ros2 launch humanoid_hardware robot_control.launch.py

# Development
colcon build --packages-select humanoid_hardware
```

---

## Known Issues

**Active:**
- ⚠️ **Joint local positions missing** (CRITICAL): Parts snap together
  - Symptoms: "Disjointed body transforms" warnings
  - Fix: Add physics:localPos0/localPos1 to each joint
- ⚠️ Isaac Lab DirectRLEnv configuration (WIP)

**Resolved:**
- ✅ Hierarchical structure (2026-01-19) - Fixed parent-child nesting
- ✅ Rigid body errors (2026-01-19) - Added XformStack reset
- ✅ Mesh-binding issue (2026-01-19) - Embedded USD geometry
- ✅ Joint drive parameters (2026-01-19) - Set stiffness=0 for effort control
- ✅ Rotation issue (2026-01-18) - Applied Blender Y-up → Isaac Z-up conversion

---

## Next Steps

1. **Fix joint local positions:**
   - Add `physics:localPos0` to define attachment point on parent body
   - Add `physics:localPos1` to define attachment point on child body
   - Values should match the translation offsets of child links

2. **Test robot movement:**
   - Verify all 11 body parts spread out correctly
   - Check joints move without snapping

3. **Resume RL training:**
   - Fix DirectRLEnv asset registration
   - Configure multi-robot training
   - Train walking policy

---

*Last updated: 2026-01-19*
