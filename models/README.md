# Humanoid Robot Training Guide

## Overview

This project trains a full 17-DOF humanoid robot to walk using **PPO (Proximal Policy Optimization)** reinforcement learning in MuJoCo physics simulation.

## Robot Structure

### 17 Servo Channels (Continuous Rotation):
- **Head (1 DOF)**: Channel 0 - left/right pan
- **Right Arm (3 DOF)**: Channels 1, 2, 3 - shoulder pitch, shoulder roll, elbow
- **Left Arm (3 DOF)**: Channels 12, 13, 14 - shoulder pitch, shoulder roll, elbow
- **Right Leg (5 DOF)**: Channels 4, 5, 6, 7, 8 - hip roll, hip pitch, knee, ankle roll, ankle pitch
- **Left Leg (5 DOF)**: Channels 15, 16, 17, 18, 19 - hip roll, hip pitch, knee, ankle roll, ankle pitch

**NOTE**: Ankle pitch joints (8 & 19) added for proper bipedal walking. These enable forward push-off during walking, critical for natural gait.

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

**Train with PARALLEL ROBOTS (like Isaac Sim!) - MUCH FASTER! 🚀**:
```bash
python3 train_full_robot_mujoco.py --headless --num-envs 8   # 8 robots in parallel (3x faster!)
python3 train_full_robot_mujoco.py --headless --num-envs 16  # 16 robots (3-4x faster!)
python3 train_full_robot_mujoco.py --resume --headless --num-envs 8  # resume with 8 robots
```

**Specify training duration**:
```bash
python3 train_full_robot_mujoco.py --timesteps 5000000       # Train for 5M steps
python3 train_full_robot_mujoco.py --timesteps 10000000 --headless --num-envs 8  # 10M steps with 8 robots
python3 train_full_robot_mujoco.py --resume --timesteps 3000000  # Resume and train 3M more steps
```

**Default**: 1,000,000 steps (~15-30 minutes with 8 robots)

**Performance comparison** (tested on RTX 4070 Ti + multi-core CPU):
- 1 robot: ~1,300 FPS (baseline)
- 8 robots: ~4,300 FPS (**3.3x faster!**)
- 16 robots: ~4,500 FPS (**3.5x faster!** - good for powerful CPUs)

**Important notes**:
- **No UI when using multiple robots** - Rendering is automatically disabled for speed
- **CPU-based parallelization** - Each robot runs in a separate process
- **Optimal performance**: Match `--num-envs` to your CPU core count
- **GPU note**: MuJoCo MJX (GPU physics) conflicts with Stable-Baselines3 dependencies (NumPy version incompatibility). CPU parallelization provides excellent speedup without compatibility issues.

**Test trained policy**:
```bash
python3 train_full_robot_mujoco.py --test
```

### Training Details

- **Algorithm**: PPO (Proximal Policy Optimization)
- **Default timesteps**: 1,000,000 (use `--timesteps` to change)
- **Checkpoints**: Saved every 10,000 steps to `models/ppo_logs/`
- **Final policy**: Saved as `models/full_robot_ppo.zip`
- **Expected time**:
  - 1M steps: ~15-30 min (quick test)
  - 5M steps: ~1-2 hours (decent walking)
  - 10M+ steps: ~3-6 hours (good walking)

---

## Reward Function Design

**UPDATED v3: Anti-Hopping with Explicit Foot Switching (January 2025)**

After v1 (sparse) and v2 (dense) the robot **STILL hopped on one leg** after extensive training. We discovered the fundamental flaw: **hopping satisfies almost all walking rewards!**

### The Critical Insight: Why Hopping Kept Winning 🦘❌

**Hopping on one leg provides:**
- ✓ Forward velocity (moves forward fast)
- ✓ Single foot contact (one foot always down)
- ✓ Foot height (other foot stays lifted)
- ✓ Airtime (lifted foot in air constantly)
- ✓ No penalties (doesn't have both feet down/up simultaneously)

**The ONLY difference between hopping and walking:**
- **Hopping**: Same foot on ground every time
- **Walking**: Alternates which foot is on ground

### The Solution (v3 - Foot Switching Detection)

**Make foot switching the PRIMARY reward signal:**
1. **Track stance leg** - which foot is currently supporting weight
2. **HUGE reward (+2.0) when stance leg switches** - this IS walking!
3. **Heavy penalty after 20 steps on same leg** - prevents hopping
4. **Reward switch frequency** - encourages consistent alternation
5. **Stance-aware rewards** - only reward lifting the swing foot, not stance foot

### Reward Components (v3 - Anti-Hopping):

### 1. **Foot Switch Reward** (+2.0) ⭐⭐⭐ HIGHEST PRIORITY!
- **Massive reward when stance leg changes**
- This is THE defining characteristic of walking vs hopping
- Tracks which foot is currently supporting weight
- Only triggers on actual stance transitions (not random flailing)

### 2. **Same-Leg Penalty** (up to -2.0) 🚫 PREVENTS HOPPING!
- After 20 steps (~1 second) on same stance leg, penalty grows
- Formula: -(steps_on_same_leg - 20) × 0.05
- Capped at -2.0 maximum
- Forces robot to switch legs regularly

### 3. **Switch Frequency Reward** (+0.5 × switch_rate)
- Rewards consistent alternation over the episode
- Typical rate: 0.01-0.02 switches per step = +0.005 to +0.01 per step
- Prevents random switching at start then hopping

### 4. **Stance-Aware Swing Foot Height** (+0.3)
- **Only rewards lifting the NON-stance foot**
- Key difference from v2: doesn't reward lifting the stance foot!
- Normalized to 15cm maximum height

### 5. **Upright Orientation** (+0.5)
- **INCREASED from 0.3** - more emphasis on stability
- Quaternion w-component squared

### 6. **Forward Velocity** (+0.3)
- **REDUCED from 0.5** - no longer primary motivation
- Clipped to [0, 1.0] to prevent exploits
- Foot switching is more important than speed

### 7. **Both Feet In Air Penalty** (-1.0) 🚫
- **INCREASED from -0.5** - very strong penalty
- Prevents jumping, hopping, and falling

### 8. **Both Feet On Ground Penalty** (-0.1)
- Small penalty - brief double-support is OK during walking
- Just discourages standing still

### 9. **Lateral Velocity Penalty** (-0.2 × |lateral_vel|)
- Walk straight, don't drift sideways

### 10. **Action Smoothness** (-0.005 × action²)
- REDUCED - allow dynamic motion for leg switching

### 11. **Base Acceleration Penalty** (-0.02 × |acceleration|)
- REDUCED - allow dynamic motion

### Comparison: v2 (Dense) vs v3 (Anti-Hopping)

| Component | v2 (Dense) | v3 (Anti-Hopping) | Why Changed |
|-----------|------------|-------------------|-------------|
| Foot switching | None | **+2.0 sparse** | THE key signal! |
| Same-leg penalty | None | **up to -2.0** | Prevents hopping |
| Switch frequency | None | **+0.5 × rate** | Consistent alternation |
| Foot height | +0.5 any foot | **+0.3 swing only** | Stance-aware |
| Forward velocity | +0.5 | **+0.3** | Not primary anymore |
| Upright | +0.3 | **+0.5** | More stability |
| Both in air | -0.5 | **-1.0** | Stronger penalty |
| Action smoothness | -0.01 | **-0.005** | More dynamic |

**Result:** Hopping gives negative reward, walking gives massive positive reward!

---

## Observation Space (48 dimensions)

The robot observes:
- **Joint positions** (17): All servo angles
- **Joint velocities** (17): Angular velocities
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

Current PPO settings in `train_full_robot_mujoco.py` (v2):

```python
learning_rate = 3e-4       # INCREASED from 1e-4 for faster adaptation
n_steps = 4096              # More samples per update
batch_size = 128            # Larger batches for stability
n_epochs = 15               # More training per batch
gamma = 0.995               # Long-term reward horizon
gae_lambda = 0.95          # Advantage estimation
clip_range = 0.2           # INCREASED from 0.1 for more aggressive updates
ent_coef = 0.01            # INCREASED from 0.005 for more exploration
```

**Changes from v1:**
- **Learning rate 3x higher** - adapts faster to dense reward signals
- **Clip range 2x higher** - allows bigger policy updates per iteration
- **Entropy 2x higher** - encourages more exploration to discover foot lifting

These changes enable faster learning with the new dense reward structure while maintaining stability.

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

### Three Deployment Options:

#### Option 1: **Slow Walking** (START HERE!) 🐾
Scripted slow, human-like walking - like a baby learning to walk.

```bash
python3 deploy_slow_walk_jetson.py --demo    # Test individual movements
python3 deploy_slow_walk_jetson.py           # Walk 20 slow steps
```

**Why start here:**
- ✅ VERY SLOW movements (safe for hardware)
- ✅ Explicit gait: lift leg → swing forward → put down
- ✅ No ML policy - simple scripted walking
- ✅ Each step takes ~5.5 seconds (deliberate)
- ✅ Easy to debug and understand

**How it works:**
- Right leg: lift (1.5s) → swing (2s) → down (1.5s) → pause (0.5s)
- Left leg: same pattern
- Alternates legs like human walking
- Upper body stays frozen

#### Option 2: **Trained Policy (Legs Only)** (After slow walk works)
Uses ML-trained policy, controls only legs.

```bash
python3 deploy_legs_only_jetson.py --demo    # Test demo mode
python3 deploy_legs_only_jetson.py           # Run trained policy
```

**Features:**
- Uses trained PPO model for walking
- Freezes upper body (prevents twisting)
- Faster than scripted but still controlled
- **Active servos (8):** both legs (hip roll, hip pitch, knee, ankle × 2)
- **Frozen servos (7):** head (1) + both arms (6)

#### Option 3: **Full Body** (Advanced - use last!)
Controls all 15 servos with trained policy.

```bash
python3 deploy_full_robot_jetson.py --demo    # Test demo mode
python3 deploy_full_robot_jetson.py           # Run trained policy
```

**Warning:** Can cause twisting! Only use after options 1 & 2 work perfectly.

---

### Setup on Jetson:

**1. Copy files from your computer:**
```bash
scp models/deploy_slow_walk_jetson.py jetson@<jetson-ip>:~/biped_robot/models/
scp models/deploy_legs_only_jetson.py jetson@<jetson-ip>:~/biped_robot/models/
scp models/deploy_full_robot_jetson.py jetson@<jetson-ip>:~/biped_robot/models/
scp models/full_robot_ppo.zip jetson@<jetson-ip>:~/biped_robot/models/
```

**2. On Jetson, install dependencies:**
```bash
# For scripted walking (Option 1) - only needs pyserial
pip install pyserial

# For trained policy (Options 2 & 3) - also needs ML libs
pip install stable-baselines3

# Verify servo board connection
ls /dev/ttyUSB*  # Should show /dev/ttyUSB1
```

**3. Test progression (do in order!):**
```bash
cd ~/biped_robot/models

# START HERE: Slow scripted walking
python3 deploy_slow_walk_jetson.py --demo    # Test individual movements
python3 deploy_slow_walk_jetson.py           # 20 slow steps

# THEN: Trained policy (legs only)
python3 deploy_legs_only_jetson.py --demo
python3 deploy_legs_only_jetson.py

# FINALLY: Full body (if legs work well)
python3 deploy_full_robot_jetson.py --demo
python3 deploy_full_robot_jetson.py
```

---

### Safety Notes:
- **START WITH SLOW WALKING SCRIPT** - safest, easiest to debug
- **Progress gradually**: slow walk → legs-only policy → full body
- **Hold the robot** or mount on stand during ALL tests
- **Press Ctrl+C** to emergency stop anytime
- Servos automatically stop when script exits
- Watch for overheating - continuous rotation servos can heat up
- If robot tips over, press Ctrl+C immediately

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

**Issue**: Robot hops on one leg instead of walking ✅ SOLVED (v3)!
- **Root Cause**: ALL previous reward functions rewarded hopping!
  - v0: Forward velocity too high (10.0) → "hop fast = max reward"
  - v1: Sparse rewards (airtime 0.4s) → "hop satisfies airtime"
  - v2: Dense rewards → "hop satisfies foot height, single contact, velocity"
  - **THE PROBLEM**: Hopping achieves most walking rewards!

- **v3 Solution (CURRENT)**: Explicit foot switching detection
  - **Track stance leg** - which foot is supporting weight
  - **+2.0 reward when stance switches** - THE key signal!
  - **Growing penalty for same leg** - prevents hopping
  - **Stance-aware rewards** - only reward lifting swing foot

- **Why this works**:
  - **Hopping**: Same foot down → accumulates penalty → negative reward
  - **Walking**: Alternating feet → +2.0 per switch → massive positive reward
  - Finally makes walking strictly better than hopping!

**Issue**: Robot still hopping after many training steps
- **CRITICAL**: You MUST start fresh, not resume!
  ```bash
  # Backup old model (it learned hopping!)
  mv models/full_robot_ppo.zip models/full_robot_ppo_hopping.zip

  # Start completely fresh with v3 anti-hopping rewards
  python3 train_full_robot_mujoco.py --headless --num-envs 32 --timesteps 3000000
  ```
- **Why**: Old policy is optimized for hopping, it will resist learning to walk
- v3 makes hopping unprofitable from the start

---

## Contact

See `CLAUDE.md` for hardware deployment and servo calibration details.
