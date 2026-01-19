# Biped Robot - Project Memory

## Current Status (2026-01-19)

**Working:** Isaac Sim basic training environment with PD controller demo
**Blocked:** Isaac Lab DirectRLEnv multi-robot training (articulation registration fails)

---

## Recent Key Issues

### Isaac Lab UI Observations - 2026-01-19
- **Finding:** Isaac Lab UI launches but starts empty (no scene loaded)
- **To load robot:** File > Open > `/home/kenpeter/work/biped_robot/models/humanoid_articulated.usda`
- **Script execution:** Run scripts via `./run_isaac.sh script.py`

### Isaac Lab Articulation Loading - BLOCKED (2026-01-19)
- **Issue:** `Failed to find articulation at '/World/Humanoid'`
- **Error:** `Pattern '/World/Humanoid' did not match any rigid bodies`
- **Cause:** USD file physics schema not properly configured for Isaac Lab articulation views
- **Files:** `test_quick.py`, `humanoid_direct_env.py`
- **Path issue:** Scripts using wrong USD path (e.g., `/home/kenpeter/work/IsaacLab/models/` instead of `/home/kenpeter/work/biped_robot/models/`)

### Isaac Lab DirectRLEnv - BLOCKED (Earlier)
- **Issue:** `KeyError: 'robot'` when accessing `self.scene.articulations["robot"]`
- **Cause:** Incorrect ArticulationCfg/UsdFileCfg configuration for custom USD assets
- **Files:** `humanoid_direct_env.py`, `train_humanoid.py`
- **Status:** Requires investigation into Isaac Lab asset registration

### Servo Not Moving Issue - FIXED (2026-01-19)
- **Issue:** Joints not responding to PD controller commands in Isaac Sim
- **Cause:** Joint drive stiffness set to 0.0 in USD file (no position feedback)
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

# Jetson - Hardware Control
python3 verify_hardware.py                   # Test servos
ros2 launch humanoid_hardware robot_control.launch.py

# Development
colcon build --packages-select humanoid_hardware
```

---

## Known Issues

**Active:**
- ⚠️ Isaac Lab articulation loading (USD physics schema not recognized by ArticulationView)
- ⚠️ Isaac Lab DirectRLEnv asset registration (blocking multi-robot training)
- ⚠️ Robot falls in simulation (expected - needs RL training)
- ⚠️ test_quick.py has wrong USD path

**Resolved:**
- ✅ Servos not moving in simulation (2026-01-19)
- ✅ 90° rotation bug (2026-01-18)
- ✅ Script execution/logging issues (2026-01-18)

---

*Last updated: 2026-01-19*