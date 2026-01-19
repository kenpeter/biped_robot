# Biped Robot - Project Memory

## Current Status (2026-01-19)

**Robot visible:** ✅ YES - colored cubes rendering correctly
**Joints recognized:** ✅ YES - 17 DOFs detected
**Joints moving:** ✅ YES - all joints responding to commands

**✅ FIXED:** Mesh-binding issue resolved by embedding geometry directly in USD

---

## Recent Fix: USD Mesh-Binding Issue (2026-01-19)

### Problem
Robot loaded but joints didn't move visually. Root cause: GLB meshes were referenced but not bound to physics links.

### Solution
Completely rewrote `setup_isaac_sim_robot.py` to:
1. Remove GLB reference entirely
2. Create embedded visible geometry using `UsdGeom.Cube` primitives
3. Color-code body parts (RED torso, YELLOW head, GREEN/BLUE limbs)
4. Apply Blender Y-up → Isaac Z-up rotation fix
5. Set correct drive parameters (stiffness=0, damping=1 for effort control)

### Result
Robot now appears with colored cubes and all joints move correctly in Isaac Sim.

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
./run_isaac.sh test_humanoid_visible.py      # Test robot visibility & movement

# Jetson - Hardware Control
python3 verify_hardware.py                   # Test servos
ros2 launch humanoid_hardware robot_control.launch.py

# Development
colcon build --packages-select humanoid_hardware
```

---

## Known Issues

**Active:**
- ⚠️ Isaac Lab DirectRLEnv configuration (WIP - need to configure asset registration)

**Resolved:**
- ✅ Mesh-binding issue (2026-01-19) - Fixed with embedded USD geometry
- ✅ Joint drive parameters (2026-01-19) - Set stiffness=0 for effort control
- ✅ Rotation issue (2026-01-18) - Applied Blender Y-up → Isaac Z-up conversion

---

## Next Steps

1. **Resume RL Training Development:**
   - Fix DirectRLEnv asset registration
   - Configure multi-robot training environment
   - Train walking policy

2. **Hardware Deployment:**
   - Export trained policy to ONNX/TensorRT
   - Deploy to Jetson Orin Nano with ROS 2

---

*Last updated: 2026-01-19*
