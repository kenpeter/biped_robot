# Biped Robot - Project Memory

## Current Status (2026-01-19)

**✅ Project cleaned up - 5 focused scripts + working 17-DOF robot**

**Robot:**
- Restored working USD from git commit a7d1930
- 17 DOF articulated humanoid with GLB mesh
- Flat sibling structure (all links under /Humanoid)
- Runtime orientation fix: +90° X rotation (Blender Y-up → Isaac Z-up)
- Spawns 15cm above ground

**Scripts (5 total):**
- `test_humanoid_visible.py` - Test robot with wave motion
- `isaac_sim_training_env.py` - RL training environment
- `train_humanoid.py` - Training script
- `humanoid_direct_env.py` - Isaac Lab DirectRLEnv
- `verify_hardware.py` - Hardware verification for Jetson

**Known Issue:** GLB mesh doesn't move with joints (skeleton binding not set up)

---

## Robot File

```
/home/kenpeter/work/biped_robot/models/humanoid_articulated.usda
```

**Test with GUI:**
```bash
./run_isaac.sh test_humanoid_visible.py
```

---

## Cleanup History

### 2026-01-19: Project Cleanup

**Removed:**
- `.gemini/` directory
- `node_modules/` (needs manual removal with sudo - owned by root)
- `package.json`, `package-lock.json` (needs sudo)
- Redundant test scripts:
  - `launch_robot.py`
  - `simple_test.py`
  - `test_quick_verify.py`
  - `test_visibility.py`
  - `setup_isaac_sim_robot.py`

**Result:** Clean project with 5 focused scripts instead of 10

### 2026-01-19: Restored Working Robot

**Problem:** Robot was lying sideways, half in ground, mesh not moving

**Solution:**
- Restored USD from commit a7d1930 (working 17-DOF version)
- Applied runtime orientation fix in test script:
  - Position: `[0, 0, 0.15]` (15cm above ground)
  - Orientation: `[0.7071, 0.7071, 0, 0]` (+90° X rotation)
- This matches `isaac_sim_training_env.py` spawn configuration

---

## Robot Structure (17 DOF)

**Joint Breakdown:**
- HEAD: 1 joint (head_joint - Z axis)
- LEFT ARM: 3 joints (shoulder_pitch, shoulder_roll, forearm_roll)
- RIGHT ARM: 3 joints (shoulder_pitch, shoulder_roll, forearm_roll)
- LEFT LEG: 5 joints (hip_roll, hip_pitch, knee_pitch, ankle_pitch, foot_roll)
- RIGHT LEG: 5 joints (hip_roll, hip_pitch, knee_pitch, ankle_pitch, foot_roll)

**Physics:**
- Flat sibling structure (all links as siblings under /Humanoid)
- GLB mesh reference: `models/humanoid.glb` (57KB)
- PD controllers: damping=10, maxForce=100
- Full 360° rotation on all joints

---

## Quick Commands

```bash
# Test robot (wave motion demo)
./run_isaac.sh test_humanoid_visible.py

# RL training environment
./run_isaac.sh isaac_sim_training_env.py

# Check robot structure
head -100 models/humanoid_articulated.usda

# Count joints
grep -c "def PhysicsRevoluteJoint" models/humanoid_articulated.usda
# Should output: 17

# Verify hardware on Jetson
python3 verify_hardware.py
```

---

## Manual Cleanup Needed

**Root-owned files (require sudo):**
```bash
sudo rm -rf node_modules/ package.json package-lock.json
```

---

## File Structure

```
biped_robot/
├── models/
│   ├── humanoid_articulated.usda   # 17-DOF robot with physics
│   └── humanoid.glb                 # Visual mesh (57KB)
├── src/
│   ├── humanoid_description/        # URDF/ROS description
│   └── humanoid_hardware/           # ROS 2 driver for Jetson
├── test_humanoid_visible.py        # Main test script
├── isaac_sim_training_env.py       # RL environment
├── train_humanoid.py                # Training
├── humanoid_direct_env.py          # Isaac Lab env
├── verify_hardware.py               # Hardware test
├── run_isaac.sh                     # Launcher
├── README.md
├── MEMORY.md                        # This file
└── CLAUDE.md                        # AI instructions
```

---

## Known Issues

**Current:**
- GLB mesh doesn't follow joint motion (skeleton binding not set up)
- Joints work (physics), but visual mesh stays static
- node_modules/ owned by root (needs manual sudo removal)

**Resolved:**
- ✅ Robot orientation (2026-01-19) - runtime +90° X rotation fix
- ✅ Robot position (2026-01-19) - spawn 15cm above ground
- ✅ Project cleanup (2026-01-19) - removed redundant scripts

---

*Last updated: 2026-01-19*
