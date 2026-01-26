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

## Deploying to Jetson

Once you have a trained policy (`full_robot_ppo.zip`), deploy it to your Jetson.

### Two Deployment Options:

#### Option 1: **Legs Only** (RECOMMENDED FIRST)
Controls only 8 leg servos, freezes upper body. Prevents twisting!

```bash
python3 deploy_legs_only_jetson.py --demo    # Test demo mode
python3 deploy_legs_only_jetson.py           # Run trained policy
```

**Why legs only:**
- ✅ Prevents upper body twisting
- ✅ Easier to control and debug
- ✅ Safer for initial testing
- ✅ Focuses on walking stability

**Frozen servos (7):** head (1) + both arms (6)
**Active servos (8):** both legs (hip roll, hip pitch, knee, ankle × 2)

#### Option 2: Full Body (Advanced)
Controls all 15 servos. Only use after legs-only works!

```bash
python3 deploy_full_robot_jetson.py --demo    # Test demo mode
python3 deploy_full_robot_jetson.py           # Run trained policy
```

---

### Setup on Jetson:

**1. Copy files from your computer:**
```bash
scp models/full_robot_ppo.zip jetson@<jetson-ip>:~/biped_robot/models/
scp models/deploy_legs_only_jetson.py jetson@<jetson-ip>:~/biped_robot/models/
scp models/deploy_full_robot_jetson.py jetson@<jetson-ip>:~/biped_robot/models/
```

**2. On Jetson, install dependencies:**
```bash
pip install stable-baselines3 pyserial

# Verify servo board connection
ls /dev/ttyUSB*  # Should show /dev/ttyUSB1
```

**3. Test with legs-only first:**
```bash
cd ~/biped_robot/models
python3 deploy_legs_only_jetson.py --demo    # Safe demo mode
python3 deploy_legs_only_jetson.py           # Trained policy (60 sec)
```

---

### Safety Notes:
- **START WITH LEGS-ONLY** - prevents twisting
- **Hold the robot** or mount on stand during first tests
- Press **Ctrl+C** to emergency stop
- Servos automatically stop when script exits
- Test demo mode before running trained policy

---

## Next Steps

1. **Train the robot**: Start with `python3 train_full_robot_mujoco.py`
2. **Monitor progress**: Watch explained variance reach 0.9+
3. **Test the policy**: Use `--test` to see trained behavior in simulation
4. **Deploy to hardware**: Follow deployment guide above

---

## Research References

Based on 2024-2025 research on humanoid locomotion with deep RL:
- PPO for continuous control (Schulman et al.)
- Foot contact alternation rewards (Singh et al., 2024)
- Clock-based gait coordination (Radosavovic et al., 2024)
- Curriculum learning for bipedal robots
- Symmetric movement rewards for natural gait

---

## Understanding Training Metrics

### Most Important Metric: **Explained Variance**

**What it measures:** How well the robot understands what's happening (0.0 to 1.0)

```
-∞ to 0.0  → Robot is confused / worse than guessing
0.0 to 0.5 → Robot is learning the basics
0.5 to 0.8 → Robot understands pretty well
0.8 to 0.95 → Robot is an expert ⭐
0.95 to 1.0 → Robot is a master (rare!)
```

**When to test:** Once explained variance reaches **0.9+**, test the robot!

---

### The 4 Different Losses:

#### 1. **Loss** (Total Combined Loss)
- **What:** Sum of all losses (value + policy + entropy)
- **Scale:** 100-500 typical
- **Good:** Trending downward (with fluctuations)

#### 2. **Value Loss** (Critic Network)
- **What:** Error in predicting future rewards
- **Scale:** 100-1000+ (large because predicting cumulative rewards)
- **Normal:** Fluctuates up and down - this is OK!
- **Why large:** Predicting sums of many future rewards

#### 3. **Policy Gradient Loss** (Actor Network)
- **What:** How much to update action choices
- **Scale:** -0.01 to -0.05 (very small)
- **Negative is OK:** Just indicates gradient direction
- **Good:** Small magnitude = stable updates

#### 4. **Entropy Loss** (Exploration)
- **What:** How random/exploratory actions are
- **Scale:** -21 (random) to -5 (confident)
- **Good:** Decreasing over time
- **Your progress:** -21 → -9 = robot getting confident!

---

### Other Metrics:

- **std (standard deviation)**: Action randomness
  - Start: 1.0 (very random)
  - End: 0.4-0.6 (confident but still some exploration)

- **clip_fraction**: How often policy updates are clipped
  - 0.1-0.3: Good, stable learning
  - >0.3: Learning aggressively (might need tuning)

- **approx_kl**: Policy change magnitude
  - <0.01: Small, safe updates ✓
  - >0.02: Large updates (might be unstable)

---

### Training Progress Example:

```
Step 0K:      explained_var=0.0,  loss=300, value_loss=800, entropy=-21
Step 100K:    explained_var=0.5,  loss=150, value_loss=400, entropy=-18
Step 500K:    explained_var=0.8,  loss=100, value_loss=300, entropy=-12
Step 1M:      explained_var=0.9,  loss=80,  value_loss=250, entropy=-10  ← TEST NOW!
Step 3M:      explained_var=0.93, loss=70,  value_loss=200, entropy=-9
```

**Key insight:** Metrics fluctuate! Value loss can go 300→500→250. This is NORMAL for reinforcement learning. Watch the trend over thousands of steps, not individual updates.

---

## When to Stop Training

**Stop and test when:**
1. ✅ Explained variance > 0.9
2. ✅ Trained for 1M+ steps
3. ✅ Loss is trending down (even with fluctuations)
4. ✅ Entropy is decreasing (robot gaining confidence)

**Don't wait for:**
- ❌ Loss to reach exactly 0
- ❌ Value loss to stop fluctuating
- ❌ Perfect smooth curves

**Reinforcement learning is noisy!** Test periodically (every 500K-1M steps).

---

## Troubleshooting

**Issue**: Training crashes or robot explodes
- **Fix**: Check joint limits in `full_robot.xml`

**Issue**: Explained variance stuck below 0.5 after 1M steps
- **Fix**: Reward function may need tuning
- Check if robot is getting any positive rewards

**Issue**: Value loss keeps increasing
- **Fix**: This is normal short-term! Check the trend over 100K+ steps
- If still increasing after 1M steps, learning rate may be too high

**Issue**: Can't see visualization
- **Fix**: Make sure you're NOT using `--headless` flag
- Check display settings (requires GUI)

**Issue**: Training too slow
- **Fix**: Use `--headless` mode or faster hardware
- Consider reducing `total_timesteps`

**Issue**: Robot walks in test but falls during training visualization
- **Fix**: This is normal! During training it explores randomly (high entropy)
- Test mode uses deterministic policy (no randomness)

---

## Contact

See `CLAUDE.md` for hardware deployment and servo calibration details.
