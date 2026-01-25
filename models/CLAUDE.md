# Biped Robot

## IMPORTANT: All Servos are Continuous Rotation
All servos use **continuous rotation** (speed + timing), NOT position control.
- Send speed value (1500=stop, >1500=one direction, <1500=other direction)
- Control rotation time to achieve desired angle
- Requires calibration on real hardware

## Hardware
- Servo board: Hiwonder LSC-24 on /dev/ttyUSB1 @ 9600 baud
- All servos: Continuous rotation (6 sec for 360°)
- Thermal camera: FLIR, mounted on head, USB to Jetson

## Servo Channels
```
HEAD (channel 0):
  Servo 0: left/right rotation

LEFT ARM (channels 12, 13, 14):
  Servo 12: shoulder (forward/backward)
  Servo 13: upper arm (inward/outward)
  Servo 14: forearm (inward/outward)
```

## Files
```
HEAD:
  head_model.py           - timing model
  head_robot.xml          - MuJoCo model
  train_head_mujoco.py    - simulation training
  train_head_isaac.py     - Isaac Sim training
  deploy_head_jetson.py   - deploy + calibrate
  head_model_weights.json - calibration data

LEFT ARM:
  left_arm_model.py           - timing model
  left_arm_robot.xml          - MuJoCo model
  train_left_arm_mujoco.py    - simulation training
  deploy_left_arm_jetson.py   - deploy + calibrate
  left_arm_model_weights.json - calibration data

OTHER:
  verify_hardware.py      - check servo connection
```

## Workflow
1. **Simulate** (optional): Get initial timing
2. **Calibrate** (required): Adjust timing on real hardware
3. **Deploy**: Use calibrated model

## Head Servo (Channel 0)

### Calibrate
```bash
python3 deploy_head_jetson.py --calibrate
```
- Manually center head, press Enter
- Runs ±30° test cycles
- Answer: l=drifted left, r=drifted right, c=centered

### Use
```bash
python3 deploy_head_jetson.py          # interactive
python3 deploy_head_jetson.py --demo   # demo
```
Commands: `left 45`, `right 30`, `center`, `stop`, `quit`

## Left Arm Servos (Channels 12, 13, 14)

### Calibrate
```bash
python3 deploy_left_arm_jetson.py --calibrate      # all servos
python3 deploy_left_arm_jetson.py --calibrate 12   # servo 12 only
python3 deploy_left_arm_jetson.py --calibrate 13   # servo 13 only
python3 deploy_left_arm_jetson.py --calibrate 14   # servo 14 only
```
- Manually center servo, press Enter
- Runs ±30° test cycles (FWD/BACK or OUT/IN)
- Answer: f=drifted forward/out, b=drifted back/in, c=centered

### Use
```bash
python3 deploy_left_arm_jetson.py          # interactive
python3 deploy_left_arm_jetson.py --demo   # demo
```
Commands:
- `12 fwd 30` / `12 back 30` - shoulder
- `13 out 30` / `13 in 30` - upper arm
- `14 out 30` / `14 in 30` - forearm
- `stop`, `quit`

## MuJoCo Simulation
```bash
python3 train_head_mujoco.py --headless
python3 train_left_arm_mujoco.py --headless
```

## Hiwonder Protocol (Continuous Rotation)
```
Command: [0x55, 0x55, 0x08, 0x03, 0x01, 0, 0, servo_id, speed_lo, speed_hi]

Speed: 1500 = stop
Timing: ~6 seconds for 360° = 16.67ms per degree
```

## Servo Speed Values (IMPORTANT)
Each servo may have different direction mapping!

```
HEAD (servo 0):
  1630 = left
  1350 = right

LEFT ARM:
  Servo 12 (shoulder): 1350 = forward, 1630 = backward  (REVERSED!)
  Servo 13 (upper arm): 1350 = out, 1630 = in  (REVERSED! limited inward range)
  Servo 14 (forearm):   1350 = out, 1630 = in  (REVERSED! limited inward range)
```

## Calibrated Timing Values

`left_arm_model_weights.json` (after calibration):
```json
{
  "servo_12": {"spd_pos": 0.01431, "spd_neg": 0.01504},
  "servo_13": {"spd_pos": 0.01667, "spd_neg": 0.01667},
  "servo_14": {"spd_pos": 0.01667, "spd_neg": 0.01667}
}
```

Servo 12 calibrated:
- Forward: 14.31 ms/deg
- Backward: 15.04 ms/deg
