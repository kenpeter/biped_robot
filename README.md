# Biped Robot

Train in Isaac Sim, deploy to Jetson hardware.

---

## Current Status (2026-01-19)

**Progress:** Robot structure created but parts snap together

**✅ Fixed Issues:**
1. Flat hierarchy → Links now properly nested (parent-child relationships)
2. Rigid body errors → Added XformStack reset for nested bodies
3. No more critical errors → Robot loads and simulates

**⚠️ Remaining Problem:**
- Robot parts collapse to single location (1-2 cubes visible instead of humanoid)
- "Disjointed body transforms" warnings for joints (elbows, knees, ankles)
- PhysX snaps objects together during simulation
- Only see 1-2 overlapping cubes, not 11 spread-out body parts

**Root Cause:** Joints missing localPos0/localPos1 attributes
- Child links have position offsets (head at z=0.15, shoulders at x=±0.15, etc.)
- BUT joints don't define where on parent/child the connection point is
- Result: PhysX snaps all parts to torso location

**Next Step:** Add joint local position offsets to prevent snapping

---

## Quick Start

### Desktop (Simulation)

```bash
# Generate USD file with embedded geometry
./run_isaac.sh setup_isaac_sim_robot.py

# Test robot - GUI mode (non-headless)
./run_isaac.sh test_humanoid_visible.py
```

**Expected (current):** 1-2 overlapping cubes at center
**Goal:** Stick figure humanoid with 11 spread-out body parts

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

## Project Structure

```
biped_robot/
├── models/                          # 3D models
│   ├── humanoid.glb                # Blender export (reference)
│   ├── humanoid_articulated.usda   # Working USD file
│   ├── create_robot.py             # Blender generator
│   └── export_usd.py               # USD exporter
│
├── src/
│   ├── humanoid_description/       # Robot URDF/USD
│   └── humanoid_hardware/          # ROS 2 driver (Jetson)
│
├── setup_isaac_sim_robot.py        # USD generator
├── test_humanoid_visible.py        # Visibility test
├── isaac_sim_training_env.py       # Basic RL environment
├── humanoid_direct_env.py          # Isaac Lab DirectRLEnv (WIP)
├── train_humanoid.py               # RL training (WIP)
├── verify_hardware.py              # Hardware diagnostic
├── run_isaac.sh                    # Isaac Sim launcher
│
├── README.md
├── CLAUDE.md
└── MEMORY.md
```

---

## Development Workflow

1. **Train** policy in Isaac Sim
   - Desktop: RTX 4070 Ti, 96GB RAM, Ubuntu 24.04
   - Environment: `isaaclab_env` (Python 3.11, Isaac Sim 5.1)

2. **Export** to ONNX/TensorRT

3. **Deploy** to Jetson Orin Nano with ROS 2

---

## Troubleshooting

### Isaac Sim Issues

**Script hangs:**
- Check logs: `~/.local/share/ov/pkg/isaac_sim-*/logs/`
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

See MEMORY.md for detailed development notes and issue tracking.
