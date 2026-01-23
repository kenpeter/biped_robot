# Biped Robot

## Workflow
1. **Train** in Isaac Sim using `models/humanoid.usda`
2. **Deploy** to Jetson using `deploy_head_jetson.py` (direct serial)

## Hardware
- Servo board: Hiwonder LSC-24 on /dev/ttyUSB1 @ 9600 baud
- Head: channel 0 (with thermal camera mounted)
- Left body: channels 1-7
- Right body: channels 12-19
- Thermal camera: FLIR, mounted on head (servo 0), USB to Jetson

## Robot Model (15 Servos + Thermal Camera)
USD file: `models/humanoid.usda`

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

## Commands
```bash
# Train (on dev machine with Isaac Sim)
cd models && ./run_isaac.sh train_head.py

# Deploy (on Jetson)
cd models && python3 deploy_head_jetson.py

# Manual servo control
python3 move_head.py left 90    # rotate left 90°
python3 move_head.py right 45   # rotate right 45°
python3 move_head.py stop       # stop rotation

# Verify hardware
python3 verify_hardware.py
```
