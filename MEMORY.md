# Biped Robot - Project Memory

## Current Status (2026-01-19)

**Robot visible in Isaac Sim:** ✅ YES
**Joints recognized:** ✅ YES (17 DOFs with correct names)
**Joints moving:** ❌ NO (visuals not bound to physics)

**Primary Issue:** Mesh-binding problem - visual meshes from GLB are not connected to physics links.

---

## USD/GLB Mesh-Binding Problem - INVESTIGATING

### Symptoms
- Robot loads and renders correctly in Isaac Sim
- `ArticulationView` finds 17 DOFs with correct joint names
- Joint position commands are sent
- But robot visuals don't move - only physics simulation updates

### Root Cause
The USD file (`humanoid_articulated.usda`) references GLB as:
```
def Xform "Meshes" (
    prepend references = @/home/kenpeter/work/biped_robot/models/humanoid.glb@
)
```

This creates:
- Visual meshes at `/Humanoid/Meshes/Head_Servo`, `/Humanoid/Meshes/L_Shoulder1`, etc. (35 meshes)
- Physics links at `/Humanoid/base_link`, `/Humanoid/Head_Servo_link`, `/Humanoid/L_Shoulder1_link`, etc.
- **No binding between visual meshes and physics links**

When joints rotate, the physics links move, but the visual meshes stay in place because they are not parented to or bound with the physics bodies.

### Evidence
1. GLB has 35 mesh nodes (verified via Python script)
2. USD has 17 DOFs recognized by ArticulationView
3. Isaac Sim can render the GLB reference
4. Standard USD library cannot load GLB directly (plugin missing)

### Solutions Attempted
1. ❌ USD GLB plugin not available in isaaclab_env
2. ❌ Isaac Sim asset converter API changes blocking GLB→USD conversion
3. ❌ Mesh-binding script failed (couldn't load GLB via USD)
4. ❌ Direct USD reference to GLB works for rendering but not physics binding

### Solutions to Try

**Option A: Export from Blender as USD (RECOMMENDED)**
- Blender 4.2+ has native USD export
- File > Export > USD
- Select "USDZ" or "USDA" format
- This preserves mesh data in USD format, not as GLB reference

**Option B: Use URDF Importer**
- If robot has URDF file: `File > Import > URDF`
- Isaac Sim's URDF importer handles physics binding automatically
- Creates proper articulation structure

**Option C: Isaac Sim "Create Reference"**
- In Isaac Sim UI: `File > Create > Reference` (not `Open`)
- This may handle binding better than direct USD reference syntax

**Option D: Manual USD Creation**
- Create USD with mesh data embedded (not referenced)
- Use `UsdGeom.Mesh` to define meshes directly in USD
- Parent meshes to physics links via `xformOp:transform`

---

## Recent Key Issues

### USD/GLB Mesh-Binding - INVESTIGATING (2026-01-19)
- **Issue:** Robot visible but joints don't move
- **Cause:** Visual meshes from GLB reference not bound to physics links
- **Files:** `models/humanoid_articulated.usda`, `models/humanoid.glb`
- **Status:** Need to re-export robot from Blender as USD format

### Isaac Lab UI Observations - 2026-01-19
- **Finding:** Isaac Lab UI launches but starts empty (no scene loaded)
- **To load robot:** File > Open > `/home/kenpeter/work/biped_robot/models/humanoid_articulated.usda`
- **Script execution:** Run scripts via `./run_isaac.sh script.py`

### Isaac Lab Articulation Loading - FIXED (2026-01-19)
- **Issue:** `Failed to find articulation at '/World/Humanoid'`
- **Cause:** Scripts using `ArticulationView` before physics initialization
- **Fix:** Use `World.reset()` and wait for `num_dof` to initialize
- **Files:** `test_joint_final.py`

### Isaac Lab DirectRLEnv - BLOCKED (Earlier)
- **Issue:** `KeyError: 'robot'` when accessing `self.scene.articulations["robot"]`
- **Cause:** Incorrect ArticulationCfg/UsdFileCfg configuration
- **Files:** `humanoid_direct_env.py`, `train_humanoid.py`
- **Status:** Requires mesh-binding fix first

### Servo Not Moving Issue - FIXED (2026-01-19)
- **Issue:** Joints not responding to PD controller commands in Isaac Sim
- **Cause:** Joint drive stiffness set to 0.0 in USD file
- **Fix:** Set stiffness=500, damping=20 in joint drive API
- **Files:** `setup_isaac_sim_robot.py:156`, `models/humanoid_articulated.usda`

### Rotation Issue - FIXED (2026-01-18)
- **Issue:** Robot appeared rotated 90° in Isaac Sim
- **Cause:** Blender Y-up vs Isaac Sim Z-up coordinate systems
- **Fix:** Applied -90° X-axis rotation at spawn (quaternion: `[0.7071, -0.7071, 0.0, 0.0]`)
- **File:** `isaac_sim_training_env.py:45`

---

## File Structure

```
biped_robot/
├── models/                          # 3D models and USD files
│   ├── humanoid.glb                # Blender export (Y-up)
│   ├── humanoid_articulated.usda   # Isaac Sim physics model (Z-up)
│   ├── create_robot.py             # Blender robot generator
│   └── export_usd.py               # GLB → USD converter
│
├── src/
│   ├── humanoid_description/       # Robot description (legacy)
│   └── humanoid_hardware/          # ROS 2 hardware driver (for Jetson)
│
├── setup_isaac_sim_robot.py        # GLB → Articulated USD converter
├── isaac_sim_training_env.py       # ✅ Working: Basic RL environment + PD demo
├── humanoid_direct_env.py          # ❌ Blocked: Isaac Lab DirectRLEnv
├── train_humanoid.py               # ❌ Blocked: RL training script
├── verify_hardware.py              # Hardware diagnostic tool
├── run_isaac.sh                    # Isaac Sim launcher wrapper
│
├── README.md                       # Project documentation
├── CLAUDE.md                       # AI assistant instructions
└── MEMORY.md                       # This file
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
# Desktop - Isaac Sim Training
./run_isaac.sh setup_isaac_sim_robot.py      # Convert GLB to USD
./run_isaac.sh isaac_sim_training_env.py     # Run PD demo (working)

# Launch Isaac Lab UI (empty by default)
cd /home/kenpeter/work/IsaacLab && ./isaaclab.sh -s
# Then: File > Open > /home/kenpeter/work/biped_robot/models/humanoid_articulated.usda

# Test joint control
./run_isaac.sh test_joint_final.py           # Test if joints respond to commands

# Jetson - Hardware Control
python3 verify_hardware.py                   # Test servos
ros2 launch humanoid_hardware robot_control.launch.py

# Development
colcon build --packages-select humanoid_hardware
```

---

## USD File Structure Issue - INVESTIGATING (2026-01-19)

**Problem:** Robot visible but joints not responding to commands

**Root Cause Analysis:**
1. GLB file contains 35 mesh nodes (Head_Servo, L_Shoulder1, L_Shoulder2, etc.)
2. USD file references GLB as a single reference in `/Humanoid/Meshes`
3. Physics links (base_link, Head_Servo_link, etc.) are separate xforms with physics schemas
4. Visual meshes are NOT bound to physics links - they're just referenced together

**Evidence:**
- USD can't load GLB directly: "Cannot determine file format for @humanoid.glb@"
- Isaac Sim imports GLB as reference but mesh-binding is lost
- Articulation has 17 DOFs recognized, but visuals don't move with joints

**Solutions Tried:**
1. ✅ Isaac Sim can load the USD with GLB reference (visuals appear)
2. ❌ USD library can't open GLB directly (plugin missing)
3. ❌ GLB to USD converter not working (API issues)
4. ❌ Mesh-to-link binding script failed (GLB not loadable)

**Alternative Approaches:**
1. Export robot from Blender as USD (not GLB)
2. Use URDF importer which handles physics bindings automatically
3. Manually create USD with USD-formatted meshes (not GLB reference)
4. Use Isaac Sim's "Create Reference" dialog which may handle bindings

---

## Known Issues

**Active:**
- ⚠️ **USD/GLB mesh-binding issue (CRITICAL)**: Robot visible but joints don't move
  - Root cause: Visual meshes (from GLB) not bound to physics links
  - Articulation recognized (17 DOFs) but visuals don't follow physics
  - Need to either: (1) Export from Blender as USD, (2) Use URDF importer, or (3) Fix GLB binding
- ⚠️ Isaac Lab DirectRLEnv asset registration (blocking multi-robot training)

**Resolved:**
- ✅ Isaac Lab UI launches successfully
- ✅ Robot USD loads and shows visual meshes
- ✅ ArticulationView finds 17 DOFs with correct joint names
- ✅ Script paths fixed (were pointing to wrong directories)
- ✅ Servos not moving in simulation (2026-01-19)
- ✅ 90° rotation bug (2026-01-18)

---

## Next Steps

1. **Fix mesh-binding issue:**
   - Option A: Re-export robot from Blender as USD format (not GLB)
   - Option B: Use Isaac Sim URDF importer on robot's URDF
   - Option C: Manually create USD with embedded mesh geometry

2. **Test joint control:**
   - Run `./run_isaac.sh test_joint_final.py`
   - Watch if robot joints actually move

3. **Once joints work:**
   - Resume RL training development
   - Fix DirectRLEnv configuration

---

*Last updated: 2026-01-19*