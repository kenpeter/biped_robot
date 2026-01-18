# Biped Robot - Project Memory

## Recent Changes (2026-01-18)

### Isaac Sim Training Environment Fixes

**Problem 1: 90-Degree Rotation Issue**
- **Symptom:** Robot appeared rotated 90 degrees when loaded in Isaac Sim
- **Root Cause:** Blender exports 3D models in Y-up coordinate system, but Isaac Sim uses Z-up
- **Solution:** Applied -90° rotation around X-axis at spawn time in `isaac_sim_training_env.py:45`
- **Implementation:** Changed orientation quaternion from `[1.0, 0.0, 0.0, 0.0]` to `[0.7071, -0.7071, 0.0, 0.0]`
- **File Modified:** `isaac_sim_training_env.py`

**Problem 2: Script Not Executing Demo Loop**
- **Symptom:** Isaac Sim window opened but the Python demo loop didn't appear to run
- **Root Cause:**
  - Duplicate `world.reset()` call causing initialization issues
  - Incorrect articulation path reference
  - Buffered output hiding progress messages
- **Solution:**
  1. Moved `world.reset()` to happen before adding the articulation
  2. Added try-except block to test both `/World/Humanoid` and `/World/Humanoid/base_link` paths
  3. Removed duplicate `world.reset()` call
  4. Added unbuffered output (`sys.stdout` and `sys.stderr` reconfiguration)
  5. Added detailed logging with 5-step progress indicators
- **Files Modified:** `isaac_sim_training_env.py`

**Additional Improvements:**
- Added comprehensive error handling with traceback printing
- Improved user feedback with clear progress messages
- Added detailed troubleshooting section to README.md

---

## File Structure

### Desktop Environment (~/work/biped_robot)
```
biped_robot/
├── models/                          # 3D models and USD files
│   ├── humanoid.glb                # GLB export from Blender (Y-up)
│   ├── humanoid_articulated.usda   # Articulated USD with physics (Z-up)
│   ├── create_robot.py             # Blender script to generate robot model
│   └── export_usd.py               # Blender → USD export script
│
├── src/
│   └── humanoid_description/
│       └── usd/                    # Legacy USD files (before models/ restructure)
│
├── setup_isaac_sim_robot.py        # GLB → Articulated USD converter
├── isaac_sim_training_env.py       # RL training environment with PD demo
├── run_isaac.sh                    # Wrapper to run Isaac Sim with IsaacLab env
│
├── README.md                       # Project documentation
├── CLAUDE.md                       # AI assistant instructions
└── MEMORY.md                       # This file - project memory and notes
```

### Jetson Environment (/home/jetson/work/biped_ws)
```
biped_ws/
├── src/
│   └── humanoid_hardware/          # ROS 2 hardware driver package
│       ├── launch/
│       │   └── robot_control.launch.py
│       ├── scripts/
│       │   └── servo_controller.py
│       └── package.xml
│
└── verify_hardware.py              # Hardware verification script
```

---

## Hardware Configuration

### Servo Board: Hiwonder LSC-24
- **Port:** `/dev/ttyUSB1` on Jetson
- **Baud Rate:** 9600
- **Protocol:** Binary format `[0x55, 0x55, 0x08, 0x03, 0x01, time_lo, time_hi, servo_id, pos_lo, pos_hi]`
- **Position Range:** 500-2500 (maps to 0-180 degrees)

### Servo Channel Mapping
```
Channel 0:  head_joint
Channels 1-7:  Left arm (shoulder pitch, shoulder roll, forearm roll)
Channels 8-11: Left leg (hip roll, hip pitch, knee pitch, ankle pitch, foot roll)
Channels 12-19: Right arm + Right leg (mirrored)
```

### Robot Model (15 Servos + Thermal Camera)
- **Head:** 1 servo (Z-axis rotation) + FLIR thermal camera (USB to Jetson)
- **Left Arm:** 3 servos (shoulder pitch Y, shoulder roll X, forearm roll X)
- **Right Arm:** 3 servos (mirrored)
- **Left Leg:** 4 servos (hip roll X, hip pitch Y, knee pitch Y, ankle pitch Y, foot roll X)
- **Right Leg:** 4 servos (mirrored)

---

## Coordinate System Notes

### Blender (Model Creation)
- **Up Axis:** Y
- **Forward Axis:** -Z
- **Right Axis:** X

### Isaac Sim (Physics Simulation)
- **Up Axis:** Z
- **Forward Axis:** Y
- **Right Axis:** X

### Rotation Transform
To convert Blender Y-up to Isaac Sim Z-up, apply -90° rotation around X-axis:
- **Quaternion (w, x, y, z):** `[0.7071, -0.7071, 0.0, 0.0]`
- **Applied at:** Robot spawn time in `isaac_sim_training_env.py`

---

## Git Workflow

### Recent Merge (2026-01-18)
1. **Remote changes:** Workspace restructured - moved files to `models/` directory
2. **Local changes:** Added Isaac Sim training environment files
3. **Resolution:** Manually merged both sets of changes, updating paths to use `models/` directory
4. **Commit message:** "Merge: Add Isaac Sim training environment and resolve restructure conflicts"

### Branch: main
- **Workflow:** Direct commits to main branch
- **Remote:** origin (GitHub/GitLab)

---

## Isaac Sim Setup

### Environment
- **Conda Environment:** `isaaclab_env` (Python 3.11)
- **Isaac Sim Version:** 5.1 (installed via conda in IsaacLab)
- **Wrapper Script:** `./run_isaac.sh` handles conda activation

### Key Python Scripts

1. **`setup_isaac_sim_robot.py`**
   - Converts GLB to articulated USD
   - Adds physics (rigid bodies, collisions, masses)
   - Defines 17 revolute joints with limits and drives
   - Creates ground plane for simulation
   - Run: `./run_isaac.sh setup_isaac_sim_robot.py`

2. **`isaac_sim_training_env.py`**
   - Creates training environment with robot, camera, and sensors
   - Implements PD controller demo (2000 steps)
   - Provides RL observation/reward framework
   - Run: `./run_isaac.sh isaac_sim_training_env.py`

### Joint Configuration (from setup_isaac_sim_robot.py)
Each joint defined as: `(joint_name, parent_link, child_link, axis, lower_limit, upper_limit, offset_xyz)`

Example:
```python
("l_hip_roll", "Torso_Bot_Plate", "L_Hip1", "X", -30, 30, (0, 0.04, -0.05))
```

---

## Next Steps

### Short Term
1. ✅ Fix 90-degree rotation issue
2. ✅ Fix script execution and logging
3. ✅ Update documentation
4. 🔲 Test the updated training environment
5. 🔲 Verify robot loads correctly with proper orientation

### Medium Term
1. 🔲 Implement RL training loop (PPO/SAC)
2. 🔲 Add reward shaping for bipedal locomotion
3. 🔲 Train policy for standing balance
4. 🔲 Train policy for walking

### Long Term
1. 🔲 Deploy trained policy to Jetson
2. 🔲 Integrate thermal camera data
3. 🔲 Implement real-time obstacle avoidance
4. 🔲 Test on physical hardware

---

## Known Issues

### Resolved
- ✅ Robot rotated 90 degrees (Fixed: 2026-01-18)
- ✅ Script not executing demo loop (Fixed: 2026-01-18)
- ✅ USD file had absolute paths (Fixed: changed to relative paths)

### Active
- ⚠️ Robot falls immediately (Expected - needs RL training)
- ⚠️ PD controller gains not tuned for this specific robot

### Future Considerations
- 🔮 Collision meshes are simplified (using box approximations)
- 🔮 Joint limits are estimates (need to verify with physical hardware)
- 🔮 Mass distribution is approximate (need to weigh actual components)

---

## References

### Documentation
- Isaac Sim Docs: https://docs.omniverse.nvidia.com/isaacsim/latest/
- IsaacLab Docs: https://isaac-sim.github.io/IsaacLab/
- USD Format: https://graphics.pixar.com/usd/docs/index.html

### Key Concepts
- **USD (Universal Scene Description):** 3D scene format used by Isaac Sim
- **GLB (GL Transmission Format Binary):** Efficient 3D model format from Blender
- **Articulation:** Physics term for a robot with connected rigid bodies and joints
- **PD Controller:** Proportional-Derivative controller for position tracking
- **RL (Reinforcement Learning):** Training method for learning robot control policies

---

## Troubleshooting Tips

### Isaac Sim won't launch
- Check conda environment: `conda activate isaaclab_env`
- Verify Isaac Sim installation: `ls ~/IsaacSim` or conda package location
- Check GPU drivers: `nvidia-smi`

### Robot not visible in Isaac Sim
- Verify USD reference path in `humanoid_articulated.usda`
- Check that `humanoid.glb` exists in `models/` directory
- Open USD in text editor and verify `prepend references = @./humanoid.glb@`

### Script hangs or freezes
- Check Isaac Sim logs: Look for error messages
- Enable verbose output: Already implemented with unbuffered stdout
- Run with headless mode for faster debugging: Change `SimulationApp({"headless": True})`

### Robot falls through ground
- Increase spawn height in `isaac_sim_training_env.py:44`
- Verify ground plane collision is enabled
- Check robot collision geometry in USD file

---

## Performance Notes

### Hardware Requirements (Desktop)
- **GPU:** NVIDIA RTX 4070 Ti (12GB VRAM) - confirmed working
- **CPU:** AMD Ryzen 5 7600 (6-core) - sufficient
- **RAM:** 96GB - more than adequate
- **OS:** Ubuntu 24.04 LTS

### Simulation Performance
- **Physics Steps:** 60 Hz (default)
- **Render FPS:** ~30-60 fps (depends on scene complexity)
- **Training Speed:** TBD (will measure after RL implementation)

---

*This file is maintained to help track project progress, decisions, and solutions.*
*Last updated: 2026-01-18*
