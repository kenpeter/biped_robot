# Biped Robot

## Workflow
1. **Simulate** using MuJoCo (Jetson/ARM) or Isaac Sim (x86_64)
2. **Calibrate** on real hardware with `--calibrate`
3. **Deploy** to Jetson using `deploy_head_jetson.py`

## Hardware
- Servo board: Hiwonder LSC-24 on /dev/ttyUSB1 @ 9600 baud
- Head: channel 0 (continuous rotation servo with thermal camera)
- Left body: channels 1-7
- Right body: channels 12-19
- Thermal camera: FLIR, mounted on head (servo 0), USB to Jetson

## Training / Simulation

### MuJoCo (runs on Jetson)
```bash
python3 train_head_mujoco.py           # with viewer (real-time)
python3 train_head_mujoco.py --fast    # with viewer (fast)
python3 train_head_mujoco.py --headless  # no viewer (fastest)
```

### Isaac Sim (requires x86_64 + NVIDIA GPU)
```bash
./run_isaac.sh train_head_isaac.py
```

**Note:** Simulation gives initial timing values. Real hardware calibration is required for accurate movement.

## Calibration (Real Hardware)
```bash
# Calibrate timing on real robot (REQUIRED after simulation)
python3 deploy_head_jetson.py --calibrate
```
- Runs ±30° test movements
- Asks which way head drifted (l=left, r=right, c=centered)
- Adjusts timing by 3% per iteration
- Saves when centered

## Deployment
```bash
# Interactive mode
python3 deploy_head_jetson.py

# Demo mode (oscillates ±30°)
python3 deploy_head_jetson.py --demo
```

### Interactive Commands
- `left 45` / `l 45` - rotate left 45°
- `right 30` / `r 30` - rotate right 30°
- `center` / `c` - return to center
- `stop` / `s` - stop rotation
- `go 60` - go to absolute angle 60°
- `quit` / `q` - exit

## Calibration File
`head_model_weights.json`:
```json
{
  "seconds_per_degree_left": 0.01767,
  "seconds_per_degree_right": 0.01867
}
```

## Robot Model (15 Servos + Thermal Camera)
USD file: `humanoid.usda` | MuJoCo: `head_robot.xml`

```
HEAD (1 servo + camera):
  head_joint            → left/right (Z)
  thermal_camera        → FLIR (USB to Jetson)

LEFT ARM (3):
  l_shoulder_pitch      → forward/backward (Y)
  l_shoulder_roll       → close/away body (X)
  l_forearm_roll        → close/away body (X)

RIGHT ARM (3):
  r_shoulder_pitch      → forward/backward (Y)
  r_shoulder_roll       → close/away body (X)
  r_forearm_roll        → close/away body (X)

LEFT LEG (4):
  l_hip_roll            → close/away body (X)
  l_knee_pitch          → forward/backward (Y)
  l_ankle_pitch         → forward/backward (Y)
  l_foot_roll           → close/away body (X)

RIGHT LEG (4):
  r_hip_roll            → close/away body (X)
  r_knee_pitch          → forward/backward (Y)
  r_ankle_pitch         → forward/backward (Y)
  r_foot_roll           → close/away body (X)
```

## Hiwonder Protocol
Binary format: `[0x55, 0x55, 0x08, 0x03, 0x01, time_lo, time_hi, servo_id, pos_lo, pos_hi]`
Position range: 500-2500 (0-180 degrees)

## Quick Reference
```bash
# Full workflow
python3 train_head_mujoco.py --headless  # 1. Simulate
python3 deploy_head_jetson.py --calibrate  # 2. Calibrate on real hardware
python3 deploy_head_jetson.py --demo       # 3. Test

# Manual servo control
python3 move_head.py left 90
python3 move_head.py right 45
python3 move_head.py stop

# Verify hardware
python3 verify_hardware.py
```
