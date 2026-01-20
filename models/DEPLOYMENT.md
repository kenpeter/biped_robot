# Training to Deployment Workflow

## 1. Training in Isaac Sim

```
humanoid.usda (USD model with joints)
    ↓
Train RL policy (e.g., PPO)
    ↓ outputs actions for each joint
trained_policy.pt (neural network)
```

**What the policy learns:**
- Input: robot state (joint positions, velocities, IMU, camera)
- Output: target joint positions/velocities (one per joint)

Example action output:
```python
{
    'head_joint': 0.15,        # radians
    'l_shoulder_pitch': -0.5,
    'l_knee_pitch': 1.2,
    ...
}
```

## 2. Deployment to Hardware

### Step 1: Load Policy in ROS 2 Node

```python
import torch
policy = torch.load('trained_policy.pt')

# Load servo mapping
servo_map = yaml.load('servo_mapping.yaml')
```

### Step 2: Read Robot State

```python
# From ROS topics published by humanoid_hardware
joint_states = subscribe('/joint_states')
imu_data = subscribe('/imu')
camera_img = subscribe('/thermal_camera')
```

### Step 3: Run Policy Inference

```python
# Prepare observation
obs = {
    'joint_pos': joint_states.position,
    'joint_vel': joint_states.velocity,
    'imu': imu_data,
    'camera': process_image(camera_img)
}

# Get action from policy
action = policy(obs)  # Returns target joint positions
```

### Step 4: Convert to Servo Commands

```python
for joint_name, target_pos in action.items():
    # Get servo ID from mapping
    servo_id = servo_map[joint_name]

    # Convert radians to servo position (500-2500)
    # Assuming ±90° range → 0° = 1500
    servo_pos = int(1500 + (target_pos * 180/π) * (2000/180))
    servo_pos = np.clip(servo_pos, 500, 2500)

    # Send to hardware
    send_servo_command(servo_id, servo_pos)
```

### Step 5: Hardware Command

```python
def send_servo_command(servo_id, position, time_ms=20):
    """Send Hiwonder protocol command"""
    cmd = [
        0x55, 0x55,           # Header
        0x08,                 # Length
        0x03,                 # Command (move servo)
        0x01,                 # Count (1 servo)
        time_ms & 0xFF,       # Time low byte
        (time_ms >> 8) & 0xFF,# Time high byte
        servo_id,             # Servo ID (0-19)
        position & 0xFF,      # Position low byte
        (position >> 8) & 0xFF# Position high byte
    ]
    serial_port.write(bytes(cmd))
```

## 3. Full System Diagram

```
┌─────────────────────────────────────────────────────┐
│ TRAINING (Isaac Sim)                                │
│                                                     │
│  humanoid.usda → RL Training → trained_policy.pt   │
└─────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────┐
│ JETSON (ROS 2)                                      │
│                                                     │
│  ┌──────────────────┐                              │
│  │ Policy Node      │                              │
│  │ - Load policy.pt │                              │
│  │ - Subscribe state│                              │
│  │ - Publish actions│                              │
│  └────────┬─────────┘                              │
│           │ /joint_commands                         │
│           ↓                                         │
│  ┌──────────────────┐                              │
│  │ Hardware Driver  │                              │
│  │ - Read mapping   │                              │
│  │ - Convert actions│                              │
│  │ - Send to servos │                              │
│  └────────┬─────────┘                              │
└───────────┼─────────────────────────────────────────┘
            │ /dev/ttyUSB1 @ 9600 baud
            ↓
┌─────────────────────────────────────────────────────┐
│ HIWONDER LSC-24 SERVO BOARD                         │
│                                                     │
│  Ch 0: head_joint                                   │
│  Ch 1-7: left_arm + left_leg                        │
│  Ch 12-19: right_arm + right_leg                    │
└─────────────────────────────────────────────────────┘
            ↓
┌─────────────────────────────────────────────────────┐
│ PHYSICAL ROBOT (15 servos)                          │
└─────────────────────────────────────────────────────┘
```

## 4. Key Requirements

### USD Model Must Match Hardware

| Requirement | USD Training | Physical Hardware |
|------------|--------------|-------------------|
| Joint names | Must match | servo_mapping.yaml |
| Joint limits | Set in USD | Servo mechanical limits |
| Joint axes | Correct in USD | Match servo orientation |
| Mass/inertia | Realistic | Measure actual robot |

### Example Mismatch Problems

**Problem:** USD says `head_joint` rotates ±90°, but hardware can only do ±45°
**Solution:** Set correct limits in USD during training

**Problem:** USD has left leg on channels 1-4, hardware uses 4-7
**Solution:** Update servo_mapping.yaml to match hardware

**Problem:** Policy outputs velocities, but hardware wants positions
**Solution:** Integrate velocities to positions in deployment node

## 5. Testing Workflow

1. **Test individual servos** (already done with `test_hiwonder_servo.py`)
2. **Test joint mapping** (send policy action → verify correct servo moves)
3. **Test policy in sim** (ensure it works before deploying)
4. **Deploy with safety limits** (start with small movements)
5. **Tune timing** (adjust servo move times for smooth motion)

## 6. Next Steps for Your Robot

- [ ] Create detailed USD model matching your physical robot
- [ ] Measure actual servo ranges and update joint limits
- [ ] Test servo mapping (send test commands to each servo)
- [ ] Build deployment ROS node that reads policy outputs
- [ ] Add safety checks (fall detection, joint limit enforcement)
- [ ] Test simple policies before training complex locomotion
