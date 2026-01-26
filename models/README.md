# Humanoid Robot Training Guide

## Overview

This project trains a full 15-DOF humanoid robot to walk using **PPO (Proximal Policy Optimization)** reinforcement learning in MuJoCo physics simulation.

## Robot Structure

### 15 Servo Channels (Continuous Rotation):
- **Head (1 DOF)**: Channel 0 - left/right pan
- **Right Arm (3 DOF)**: Channels 1, 2, 3 - shoulder pitch, shoulder roll, elbow
- **Left Arm (3 DOF)**: Channels 12, 13, 14 - shoulder pitch, shoulder roll, elbow
- **Right Leg (4 DOF)**: Channels 4, 5, 6, 7 - hip roll, hip pitch, knee, ankle roll
- **Left Leg (4 DOF)**: Channels 15, 16, 17, 18 - hip roll, hip pitch, knee, ankle roll

All servos use continuous rotation (speed + timing control), not position control.

---

## Training the Robot

### Quick Start

**Train with visualization (shows MuJoCo viewer)**:
```bash
cd models
python3 train_full_robot_mujoco.py              # start new training
python3 train_full_robot_mujoco.py --resume     # resume from checkpoint
```

**Train without UI (headless mode - faster)**:
```bash
python3 train_full_robot_mujoco.py --headless
python3 train_full_robot_mujoco.py --resume --headless
```

**Test trained policy**:
```bash
python3 train_full_robot_mujoco.py --test
```

### Training Details

- **Algorithm**: PPO (Proximal Policy Optimization)
- **Default timesteps**: 500,000 (configurable in code)
- **Checkpoints**: Saved every 10,000 steps to `models/ppo_logs/`
- **Final policy**: Saved as `models/full_robot_ppo.zip`
- **Expected time**: 20+ hours for good walking (380M+ samples)

---

## Reward Function Design

The reward function is optimized for bipedal walking based on 2024-2025 research. Here's what the robot learns:

### 1. **Alive Bonus** (+2.0)
- Stay alive and keep training

### 2. **Height Reward** (+5.0 max)
- Encourages standing tall
- Penalty if too low (prevents crouching)

### 3. **Upright Reward** (+3.0 max)
- Critical for bipedal stability
- Uses quaternion w-component to measure uprightness

### 4. **Forward Velocity** (+0.5 × velocity)
- Main objective: walk forward
- **Started at 0.5** (not 1.0) for gradual learning
- Increase this after robot learns to balance

### 5. **Foot Alternation Reward** (+1.5 max)
- **Key for walking gait**
- Rewards when one foot is on ground, other in air
- Prevents shuffling with both feet down

### 6. **Leg Symmetry Reward** (+0.5 max)
- Prevents limping
- Encourages symmetric left/right leg movements
- Measures hip and knee joint similarity

### 7. **Energy Penalty** (-0.3 × action²)
- Encourages smooth, efficient movements
- Important for slow servo constraints

### 8. **Lateral Drift Penalty** (-1.0 × |lateral_vel|)
- Walk straight, don't drift sideways

### 9. **Rotation Penalty** (-0.8 × |angular_z|)
- Don't spin around

### 10. **Foot Dragging Penalty** (-0.5)
- Mild penalty when both feet on ground
- Encourages lifting feet during walking

---

## Observation Space (44 dimensions)

The robot observes:
- **Joint positions** (15): All servo angles
- **Joint velocities** (15): Angular velocities
- **Torso orientation** (4): Quaternion (w, x, y, z)
- **Torso velocity** (6): Linear (x, y, z) + Angular (roll, pitch, yaw)
- **Foot contacts** (2): Binary ground contact for each foot
- **Gait phase clock** (2): sin/cos of walking cycle phase

### Gait Phase Clock
- Helps robot learn periodic walking patterns
- 2 Hz cycle (0.5 second per step)
- Provides temporal awareness for coordination

---

## Key Training Insights (Research-Based)

### From 2024-2025 Studies:

1. **Curriculum Learning Works**
   - Start with "stand upright" (increase height reward)
   - Progress to "walk slowly" (low velocity reward)
   - Finally "walk fast" (increase velocity reward)

2. **Foot Contact is Critical**
   - Alternating foot contacts = natural gait
   - Symmetry prevents limping
   - Research shows this dramatically improves walking quality

3. **Clock Signals Help**
   - Giving robot a "sense of time" helps coordination
   - 2 Hz is typical for slow humanoid walking
   - Can be adjusted for faster/slower gaits

4. **Energy Efficiency Matters**
   - Helps with real hardware deployment
   - Reduces servo wear
   - Creates more natural-looking motion

5. **Training Time**
   - 20+ hours typical for decent walking
   - 380M samples for full task mastery
   - Checkpoints every 10K steps allow resuming

---

## Hyperparameters (Optimized for Walking)

Current PPO settings in `train_full_robot_mujoco.py`:

```python
learning_rate = 1e-4        # Slow, stable learning
n_steps = 4096              # More samples per update
batch_size = 128            # Larger batches for stability
n_epochs = 15               # More training per batch
gamma = 0.995               # Long-term reward horizon
gae_lambda = 0.95          # Advantage estimation
clip_range = 0.1           # Conservative policy updates
ent_coef = 0.005           # Low exploration (focused learning)
```

These are based on research showing slower, more conservative updates work better for complex locomotion.

---

## Improving Performance

### If robot falls too much:
- Increase `height_reward` weight (currently 5.0)
- Increase `upright` weight (currently 3.0)
- Reduce `forward_vel` weight (currently 0.5)

### If robot walks but limps:
- Increase `leg_symmetry` weight (currently 0.5)
- Add more foot contact rewards

### If robot shuffles instead of walking:
- Increase `foot_alternation` weight (currently 1.5)
- Increase penalty for both feet down

### If training is too slow:
- Use `--headless` mode
- Reduce `n_epochs` (currently 15)
- Train on faster CPU/GPU

---

## Files

```
models/
├── full_robot.xml              # MuJoCo model (15 DOF humanoid)
├── train_full_robot_mujoco.py  # Training script
├── full_robot_ppo.zip          # Trained policy
├── ppo_logs/                   # Checkpoints and tensorboard logs
│   ├── humanoid_10000_steps.zip
│   ├── humanoid_20000_steps.zip
│   └── ...
└── README.md                   # This file
```

---

## Visualization

When training with UI (no `--headless` flag), you'll see:
- MuJoCo viewer window opens automatically
- Real-time robot simulation
- Cube-based red/maroon robot design
- Watch robot learn to walk over time!

---

## Next Steps

1. **Train the robot**: Start with `python3 train_full_robot_mujoco.py`
2. **Monitor progress**: Watch reward increase over time
3. **Test the policy**: Use `--test` to see trained behavior
4. **Deploy to hardware**: Use deployment scripts in CLAUDE.md

---

## Research References

Based on 2024-2025 research on humanoid locomotion with deep RL:
- PPO for continuous control (Schulman et al.)
- Foot contact alternation rewards (Singh et al., 2024)
- Clock-based gait coordination (Radosavovic et al., 2024)
- Curriculum learning for bipedal robots
- Symmetric movement rewards for natural gait

---

## Troubleshooting

**Issue**: Training crashes or robot explodes
- **Fix**: Check joint limits in `full_robot.xml`

**Issue**: Reward doesn't increase
- **Fix**: Try `--resume` to continue from checkpoint
- Check if robot is just shuffling (increase foot alternation reward)

**Issue**: Can't see visualization
- **Fix**: Make sure you're NOT using `--headless` flag
- Check display settings (requires GUI)

**Issue**: Training too slow
- **Fix**: Use `--headless` mode or faster hardware
- Consider reducing `total_timesteps`

---

## Contact

See `CLAUDE.md` for hardware deployment and servo calibration details.
