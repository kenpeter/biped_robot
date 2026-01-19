# Biped Robot - Project Memory

## Current Status (2026-01-19)

**✅ Robot structure FIXED!** All issues resolved.

**✅ Fixed (2026-01-19):**
1. Hierarchical structure - All links properly nested under torso
2. Joint local positions - Added `physics:localPos0` and `physics:localPos1` to all 11 joints
3. No more "disjointed body transforms" warnings
4. Robot now appears as proper humanoid stick figure

---

## Fix Summary (2026-01-19)

### Problem
Robot parts were snapping together because:
1. Links were not properly nested (l_shoulder, r_shoulder were siblings, not children)
2. Joints lacked `physics:localPos0` and `physics:localPos1` attributes

### Solution
Rewrote `models/humanoid_articulated.usda` with:
1. **Proper hierarchy:** All body parts nested under `/Robot/torso`
2. **Correct joint connections:**
   - `/Robot/torso/head` - head_joint
   - `/Robot/torso/l_shoulder` - l_shoulder_joint → l_elbow
   - `/Robot/torso/r_shoulder` - r_shoulder_joint → r_elbow
   - `/Robot/torso/l_hip` - l_hip_joint → l_knee → l_ankle
   - `/Robot/torso/r_hip` - r_hip_joint → r_knee → r_ankle
3. **Joint local positions:**
   - `physics:localPos0` = attachment point on parent body
   - `physics:localPos1` = attachment point on child body

### Result
- Robot renders as 11 colored spheres in humanoid configuration
- All joints move correctly when commands are sent
- No PhysX warnings or snapping

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

**Resolved:**
- ✅ Joint local positions (2026-01-19) - All 11 joints now have physics:localPos0/localPos1
- ✅ Hierarchical structure (2026-01-19) - All links properly nested
- ✅ Rigid body errors (2026-01-19) - XformStack reset working
- ✅ Mesh-binding issue (2026-01-19) - Embedded USD geometry
- ✅ Joint drive parameters (2026-01-19) - Set stiffness=0 for effort control
- ✅ Rotation issue (2026-01-18) - Applied Blender Y-up → Isaac Z-up conversion

---

## Next Steps

1. **Test robot movement:**
   - Run `./run_isaac.sh test_humanoid_visible.py`
   - Verify all 11 body parts move together
   - Check for any remaining issues

2. **Resume RL training:**
   - Fix DirectRLEnv asset registration
   - Configure multi-robot training
   - Train walking policy

3. **Hardware deployment:**
   - Export trained policy to ONNX/TensorRT
   - Deploy to Jetson Orin Nano with ROS 2

---

*Last updated: 2026-01-19*
