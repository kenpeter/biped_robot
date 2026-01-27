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

**UPDATED v4: HYBRID (Our v3 + Isaac Lab) - January 2025** 🚀

After analyzing NVIDIA's Isaac Lab H1 humanoid implementation, we've combined the best of both approaches!

### Quick Comparison: Which Version to Use?

| Version | Status | Key Feature | Problem |
|---------|--------|-------------|---------|
| v0 | ❌ Failed | High velocity reward (10.0) | Robot hopped for speed |
| v1 | ❌ Failed | Sparse airtime (0.4s threshold) | Too hard to discover, still hopped |
| v2 | ❌ Failed | Dense foot height rewards | Hopping satisfied all rewards |
| v3 | ⚠️ Good | Stance switching tracking | Worked but feet could slide |
| **v4** | ✅ **CURRENT** | v3 + Isaac Lab techniques | **Use this!** |

**v4 = Our stance switching + Isaac Lab's single-stance gate + foot slide penalty**

### Evolution of Solutions

- **v1-v2**: Robot hopped - rewards didn't distinguish hopping from walking
- **v3**: Explicit stance switching tracking - good but unstable gait
- **v4 (CURRENT)**: Hybrid approach = v3 stability + Isaac Lab gait quality

### The Critical Insights

**Why hopping kept winning (v1-v2):**
- ✓ Forward velocity (hops move forward)
- ✓ Single foot contact (one foot always down)
- ✓ Foot height (other foot lifted)
- ✓ Airtime (lifted foot in air)
- **Problem**: All these are satisfied by hopping!

**Our v3 solution:** Track stance leg switching
**Isaac Lab solution:** Require single-stance for rewards + foot slide penalty
**v4 (Best of both):** Combine both techniques!

### The v4 Hybrid Approach

**Three-layer defense against hopping:**

1. **Single-Stance Requirement (Isaac Lab)**
   - Only give rewards when exactly ONE foot is on ground
   - Hopping (both feet in air) → no rewards
   - Standing (both feet down) → no rewards

2. **Stance Switching Bonus (Our v3)**
   - Track which foot is stance leg
   - +2.0 reward when stance switches
   - Hopping never switches → no bonus

3. **Foot Slide Penalty (NEW from Isaac Lab)**
   - Penalize foot moving during stance
   - Stabilizes gait and prevents shuffling
   - -0.25 × foot_velocity during contact

### Reward Components (v4 - Hybrid):

**PRIMARY OBJECTIVES (Isaac Lab inspired):**

### 1. **Forward Velocity Tracking** (+1.0)
- **INCREASED to Isaac Lab weight** - primary locomotion goal
- Clipped to [0, 1.0] to prevent exploits
- At 1 m/s forward = +1.0 reward

### 2. **Upright Orientation** (+0.5)
- Stay balanced - quaternion w-component squared
- Essential for stability

### 3. **Lateral Velocity Penalty** (-0.2 × |lateral_vel|)
- Walk straight, don't drift sideways

### 4. **Base Height** (-0.02 × error)
- Maintain target height (0.42m)

**ANTI-HOPPING MECHANISMS:**

### 5. **Foot Switch Reward** (+2.0) ⭐ CRITICAL!
- **From our v3**: Massive bonus when stance leg changes
- Only triggers during single-stance transitions
- This is THE defining signal of walking vs hopping

### 6. **Same-Leg Penalty** (up to -2.0) 🚫
- **From our v3**: After 20 steps on same leg, penalty grows
- Formula: -(steps_on_same_leg - 20) × 0.05
- Forces regular leg switching

### 7. **Single-Stance Requirement** (Isaac Lab)
- Swing foot rewards ONLY given during single stance
- Hopping (both feet in air) → zero swing rewards
- Standing (both feet down) → zero swing rewards

### 8. **Swing Foot Height** (+0.25) - Only during single stance!
- **From Isaac Lab**: Matched their weight
- Only rewards lifting the NON-stance foot
- Normalized to 15cm maximum

### 9. **Foot Slide Penalty** (-0.25 × foot_velocity) 🆕 CRITICAL!
- **NEW from Isaac Lab**: Penalize foot moving during stance
- Stabilizes gait and prevents shuffling
- Measures XY velocity of foot in contact
- This is what makes Isaac Lab gaits so stable!

**SAFETY PENALTIES:**

### 10. **Both Feet In Air** (-1.0)
- Strong penalty prevents jumping/hopping/falling

### 11. **Both Feet On Ground** (-0.1)
- Small penalty - brief double-support OK during walking
- Discourages standing still

**SMOOTHNESS:**

### 12. **Switch Frequency** (+0.5 × rate)
- Rewards consistent alternation throughout episode

### 13. **Action Smoothness** (-0.005 × action²)
- Encourages smooth control signals

### 14. **Base Acceleration** (-0.02 × |acceleration|)
- Reduces jerky movements

### Comparison: v3 vs v4 (Hybrid)

| Component | v3 | v4 (Hybrid) | Why Changed |
|-----------|----|-----------  |-------------|
| Forward velocity | +0.3 | **+1.0** | Match Isaac Lab (primary goal) |
| Foot switching | +2.0 | **+2.0** | Kept - works great! |
| Swing height | +0.3 anytime | **+0.25 single-stance only** | Isaac Lab gating |
| Foot slide | None | **-0.25 NEW** | Stabilizes stance |
| Same-leg penalty | up to -2.0 | **up to -2.0** | Kept - prevents hopping |
| Single-stance gate | None | **Required for swing rewards** | Isaac Lab technique |

**Result:** Three-layer anti-hopping defense + stable gait from foot slide penalty!

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

## Next Steps - Training with v4

### Quick Start (v4 Hybrid Rewards)

**⚠️ CRITICAL: You MUST start fresh! Don't resume from old models!**

```bash
cd models

# Step 1: Backup old model (it learned hopping!)
mv full_robot_ppo.zip full_robot_ppo_hopping_old.zip

# Step 2: Train from scratch with v4 hybrid rewards
python3 train_full_robot_mujoco.py --headless --num-envs 32 --timesteps 3000000

# Monitor these metrics:
# - explained_variance should reach 0.85+ (robot understands task)
# - Look for regular foot switches in logs
# - Forward velocity should increase steadily
```

**Why start fresh?**
- Old model learned that hopping works
- Neural network is optimized for that exploit
- New v4 rewards make hopping unprofitable
- Fresh start = faster convergence to walking

### What to Expect with v4

**First 100K steps:**
- Robot discovers stance switching gives big rewards (+2.0)
- Learns that keeping foot planted avoids slide penalty
- Forward velocity starts increasing

**100K-500K steps:**
- Regular foot switching established
- Gait becomes smoother (foot slide penalty working)
- Explained variance reaches 0.7-0.8

**500K-1M steps:**
- Stable bipedal walking with alternating feet
- No hopping (three-layer defense prevents it)
- Ready to test!

**1M-3M steps:**
- Fine-tuning gait efficiency
- Smoother transitions
- Higher forward velocity

### Testing Your Trained Policy

```bash
# After 1M+ steps with explained_variance > 0.85
python3 train_full_robot_mujoco.py --test

# Watch for:
# ✓ Alternating foot contacts (not hopping!)
# ✓ Stance foot stays planted (no sliding)
# ✓ Forward movement
# ✓ Stays upright
```

### Detailed Training Steps

1. **Train the robot**: Use v4 command above
2. **Monitor progress**: Watch explained variance reach 0.85+
3. **Check foot switches**: Look for regular alternation
4. **Test the policy**: Use `--test` after 1M steps
5. **Deploy to hardware**: Follow deployment guide if walking looks good

---

## Advanced Techniques

### Can We Use Transformers for Robot Training?

**Yes!** Transformers are increasingly used in robot learning. Here's what you need to know:

#### Recent Transformer-Based Approaches:

1. **Decision Transformer (2021)**
   - Treats RL as sequence modeling instead of traditional RL
   - Predicts actions conditioned on desired return
   - Better for offline learning from demonstrations

2. **Trajectory Transformer (2021)**
   - Plans entire action sequences at once
   - Good for long-horizon planning
   - Can reason about multi-step coordination

3. **ACT (Action Chunking Transformer, 2023)**
   - Predicts sequences of actions (chunks)
   - Excellent for manipulation tasks
   - Reduces compounding errors

4. **Gato (DeepMind, 2022)**
   - Multi-task transformer agent
   - Single model for many different tasks
   - Can transfer knowledge between tasks

5. **RT-1 & RT-2 (Google, 2022-2023)**
   - Vision transformers for robot manipulation
   - Trained on huge datasets of robot interactions
   - Strong generalization capabilities

#### For Bipedal Walking Specifically:

**Advantages:**
- **Temporal patterns**: Can learn periodic gait cycles (left-right-left-right)
- **Long-term planning**: Better than LSTM for multi-step foot coordination
- **Attention mechanism**: Can focus on critical joints (hip/knee/ankle) during different gait phases
- **Sequence modeling**: Natural fit for walking (sequence of alternating steps)

**Disadvantages:**
- **More complex**: Harder to debug than PPO
- **Data hungry**: Needs 5-10x more training samples
- **Slower training**: Larger models = longer training time
- **Less stable**: Can have mode collapse issues

#### Recommendation for Your Project:

```
Phase 1: Get PPO v4 working FIRST ✓
├── Simpler to debug
├── Faster to train (1-3M steps)
├── Well-understood failure modes
└── Proven for locomotion

Phase 2: Then experiment with Transformers
├── Decision Transformer for offline learning from PPO demonstrations
├── Action Chunking for smoother gait sequences
└── Compare performance vs PPO baseline
```

#### Implementation Path (If You Want to Try):

```python
# After PPO works, try Decision Transformer:
from decision_transformer import DecisionTransformerGym

# Collect demonstrations from trained PPO policy
demonstrations = collect_demos(ppo_policy, num_episodes=1000)

# Train Decision Transformer
dt_model = DecisionTransformerGym(
    state_dim=48,
    act_dim=17,
    max_length=200,  # Context length
    n_layer=3,
    n_head=4,
    n_inner=256
)
dt_model.train(demonstrations, target_return=500)
```

**Bottom line**: Start with PPO v4 (current implementation), then explore transformers once you have a working baseline!

---

## Research References

Based on 2024-2025 research on humanoid locomotion with deep RL:
- **PPO for continuous control** (Schulman et al., 2017)
- **NVIDIA Isaac Lab H1 implementation** - Our v4 reward structure
- **Foot contact alternation rewards** (Singh et al., 2024)
- **Clock-based gait coordination** (Radosavovic et al., 2024)
- **Decision Transformer** (Chen et al., 2021) - Transformers for RL
- **Action Chunking Transformer** (Zhao et al., 2023) - Sequence prediction
- **Curriculum learning** for bipedal robots
- **Symmetric movement rewards** for natural gait

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

**Issue**: Robot hops on one leg instead of walking ✅ SOLVED (v4)!
- **Root Cause**: ALL previous reward functions rewarded hopping!
  - v0: Forward velocity too high (10.0) → "hop fast = max reward"
  - v1: Sparse rewards (airtime 0.4s) → "hop satisfies airtime"
  - v2: Dense rewards → "hop satisfies foot height, single contact, velocity"
  - v3: Good but feet can slide during stance → unstable gait

- **v4 Solution (CURRENT)**: Hybrid of v3 + Isaac Lab H1
  - **From v3**: Stance switching tracking (+2.0 bonus, same-leg penalty)
  - **From Isaac Lab**: Single-stance requirement for rewards
  - **NEW**: Foot slide penalty (-0.25 × velocity during contact)
  - **Result**: Three-layer anti-hopping defense + stable stance

- **Why this works**:
  - **Hopping**: Both feet in air → no single-stance → no rewards → negative total
  - **Walking**: Alternating stance → +2.0 per switch + swing rewards → positive
  - **Stable**: Foot slide penalty keeps stance foot planted firmly

**Issue**: Robot still hopping after many training steps
- **CRITICAL**: You MUST start fresh, not resume!
  ```bash
  # Backup old model (it learned hopping!)
  mv models/full_robot_ppo.zip models/full_robot_ppo_hopping_old.zip

  # Start completely fresh with v4 hybrid rewards
  python3 train_full_robot_mujoco.py --headless --num-envs 32 --timesteps 3000000
  ```
- **Why**: Old policy is optimized for hopping, it will resist learning to walk
- v4 combines best of our v3 + professional Isaac Lab techniques

**Issue**: Robot shuffles or feet slide during walking
- **Fixed in v4!** Foot slide penalty (-0.25) penalizes foot movement during stance
- This is the key technique NVIDIA uses for stable H1 walking

---

## Contact

See `CLAUDE.md` for hardware deployment and servo calibration details.
