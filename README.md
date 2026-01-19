# Biped Robot

Train in Isaac Sim, deploy to Jetson hardware.

---

## Quick Start

### Desktop (Simulation)

```bash
# 1. Convert GLB model to articulated USD
./run_isaac.sh setup_isaac_sim_robot.py

# 2. Run training environment (PD controller demo)
./run_isaac.sh isaac_sim_training_env.py
```

**Expected behavior:** Robot spawns upright (0.45m above ground), runs for 2000 steps, then falls. This is normal - it needs RL training to balance.

**Blocked:** Multi-robot training (`train_humanoid.py`) has Isaac Lab asset registration issues. See MEMORY.md.

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

## Troubleshooting

### Isaac Sim Issues

**Robot rotated/upside down:**
- Check orientation quaternion in `isaac_sim_training_env.py:45`
- Should be `[0.7071, -0.7071, 0.0, 0.0]` for Blender Y-up → Isaac Z-up

**Robot not visible:**
- Verify `models/humanoid.glb` exists
- Open `models/humanoid_articulated.usda` and check reference path

**Script hangs:**
- Check Isaac Sim logs: `~/.local/share/ov/pkg/isaac_sim-*/logs/`
- Try headless mode: edit `isaac_sim_training_env.py` → `SimulationApp({"headless": True})`

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
│   ├── humanoid.glb                # Blender export
│   ├── humanoid_articulated.usda   # Isaac Sim physics model
│   ├── create_robot.py             # Model generator
│   └── export_usd.py               # USD exporter
│
├── src/
│   ├── humanoid_description/       # Robot URDF/USD
│   └── humanoid_hardware/          # ROS 2 driver (Jetson)
│
├── setup_isaac_sim_robot.py        # GLB → USD converter
├── isaac_sim_training_env.py       # ✅ Working training environment
├── humanoid_direct_env.py          # ❌ Blocked (Isaac Lab issue)
├── train_humanoid.py               # ❌ Blocked (depends on above)
├── verify_hardware.py              # Hardware test script
├── run_isaac.sh                    # Isaac Sim launcher
│
├── README.md                       # This file
├── CLAUDE.md                       # Servo mapping & instructions
└── MEMORY.md                       # Development notes
```

---

## Development Notes

**Workflow:**
1. Train policy in Isaac Sim (Desktop: RTX 4070 Ti, 96GB RAM, Ubuntu 24.04)
2. Export to ONNX/TensorRT
3. Deploy to Jetson Orin Nano with ROS 2

**Environment:** `isaaclab_env` (Python 3.11, Isaac Sim 5.1 via conda)

See MEMORY.md for detailed issue tracking and recent fixes.
