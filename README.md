# Biped Robot

Train in Isaac Sim, deploy to Jetson hardware.

---

## Current Status (2026-01-19)

**Robot visible:** ✅ YES - loads and renders correctly
**Joints recognized:** ✅ YES - 17 DOFs detected
**Joints moving:** ❌ NO - visuals not bound to physics links

**Issue:** Visual meshes from GLB are not bound to physics links, so joint commands don't move the robot visuals. This is a mesh-binding problem.

---

## Quick Start

### Desktop (Simulation)

```bash
# Launch Isaac Lab UI
cd /home/kenpeter/work/IsaacLab && ./isaaclab.sh -s
# Then: File > Open > /home/kenpeter/work/biped_robot/models/humanoid_articulated.usda

# Run joint control test
./run_isaac.sh test_joint_final.py
```

### Jetson (Hardware)

```bash
# Test servos
python3 verify_hardware.py

# Launch ROS 2 hardware driver
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch humanoid_hardware robot_control.launch.py

# Rebuild after changes
colcon build --packages-select humanoid_hardware
```

**Hardware:**
- Servo board: Hiwonder LSC-24 on /dev/ttyUSB1 @ 9600 baud
- Head: servo 0 (with thermal camera)
- Arms/legs: servos 1-19
- See CLAUDE.md for full servo mapping

---

## Problem: Mesh Not Bound to Physics

**Symptom:** Robot appears but joints don't move when sending commands.

**Root Cause:** The USD file (`humanoid_articulated.usda`) references the GLB as a single reference:
```
def Xform "Meshes" (
    prepend references = @humanoid.glb@
)
```
This imports all 35 meshes under `/Humanoid/Meshes/`, but physics links are separate (`/Humanoid/base_link`, `/Humanoid/Head_Servo_link`, etc.). No binding exists between visuals and physics.

**Solutions:**

1. **Export from Blender as USD** (recommended):
   - Blender can export directly to USD format
   - Meshes will be in USD format, not GLB reference
   - Physics bindings can be preserved

2. **Use URDF importer:**
   - If you have a URDF file, Isaac Sim can import it
   - URDF importer handles mesh-physics binding automatically

3. **Use Isaac Sim's "Create Reference" dialog:**
   - File > Create > Reference (not File > Open)
   - May handle binding better than USD reference syntax

---

## Troubleshooting

### Isaac Sim Issues

**Robot rotated/upside down:**
- Check orientation quaternion in `isaac_sim_training_env.py:45`
- Should be `[0.7071, -0.7071, 0.0, 0.0]` for Blender Y-up → Isaac Z-up

**Robot visible but joints not moving:**
- This is the mesh-binding issue (see above)
- Need to re-export robot from Blender as USD format

**Script hangs:**
- Check Isaac Sim logs: `~/.local/share/ov/pkg/isaac_sim-*/logs/`
- Try headless mode: edit script → `SimulationApp({"headless": True})`

### Jetson Hardware Issues

**Servos not moving:**
- Check battery voltage (>7.4V) and LSC-24 blue LED is ON
- Verify `/dev/ttyUSB1` exists: `ls -l /dev/ttyUSB*`
- Check wiring: CP2102 RX→Board TX, TX→Board RX

**UART errors:**
- DO NOT use `/dev/ttyTHS1` (locked by kernel on Orin Nano)
- Use USB adapters or Rosmaster USB port instead

---

## Project Structure

```
biped_robot/
├── models/                          # 3D models
│   ├── humanoid.glb                # Blender export (35 mesh nodes)
│   ├── humanoid_articulated.usda   # USD with physics (mesh-binding broken)
│   ├── create_robot.py             # Blender robot generator
│   └── export_usd.py               # USD exporter
│
├── src/
│   ├── humanoid_description/       # Robot URDF/USD
│   └── humanoid_hardware/          # ROS 2 driver (Jetson)
│
├── setup_isaac_sim_robot.py        # GLB → USD setup script
├── isaac_sim_training_env.py       # Training environment
├── humanoid_direct_env.py          # Isaac Lab DirectRLEnv (blocked)
├── train_humanoid.py               # RL training (blocked)
├── verify_hardware.py              # Hardware diagnostic
├── run_isaac.sh                    # Isaac Sim launcher
│
├── README.md                       # This file
├── CLAUDE.md                       # Servo mapping & instructions
└── MEMORY.md                       # Development notes
```

---

## Development Notes

**Workflow:**
1. Fix mesh-binding issue (export from Blender as USD)
2. Test joint control works
3. Resume RL training development
4. Train policy in Isaac Sim (Desktop: RTX 4070 Ti, 96GB RAM, Ubuntu 24.04)
5. Export to ONNX/TensorRT
6. Deploy to Jetson Orin Nano with ROS 2

**Environment:** `isaaclab_env` (Python 3.11, Isaac Sim 5.1 via conda)

See MEMORY.md for detailed issue tracking and recent fixes.
