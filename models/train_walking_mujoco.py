#!/usr/bin/env python3
"""Train biped walking using MuJoCo simulation.

Trains servos 4,5,6,7 (right leg) and 15,16,17,18 (left leg) to walk.
Uses PPO-style reinforcement learning.

Usage:
  python3 train_walking_mujoco.py              # with viewer
  python3 train_walking_mujoco.py --headless   # headless training
  python3 train_walking_mujoco.py --test       # test saved policy
"""

import numpy as np
import json
import sys
import os

try:
    import mujoco
except ImportError:
    print("MuJoCo not installed. Install with:")
    print("  pip install mujoco")
    exit(1)

# Servo mapping
# Right leg: 4=hip_roll, 5=hip_pitch, 6=knee, 7=ankle_roll
# Left leg: 15=hip_roll, 16=hip_pitch, 17=knee, 18=ankle_roll

SERVO_TO_JOINT = {
    4: 0,   # right_hip_roll
    5: 1,   # right_hip_pitch
    6: 2,   # right_knee
    7: 3,   # right_ankle_roll
    15: 4,  # left_hip_roll
    16: 5,  # left_hip_pitch
    17: 6,  # left_knee
    18: 7,  # left_ankle_roll
}

JOINT_NAMES = [
    "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle_roll",
    "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_roll"
]

# Get the script directory to make paths portable
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, 'biped_legs_robot.xml')
POLICY_PATH = os.path.join(SCRIPT_DIR, 'walking_policy.json')
TIMING_PATH = os.path.join(SCRIPT_DIR, 'walking_timing.json')


class WalkingEnv:
    """MuJoCo environment for biped walking."""

    def __init__(self, render=False):
        self.model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self.data = mujoco.MjData(self.model)
        self.render_mode = render
        self.viewer = None

        # Environment parameters
        self.dt = self.model.opt.timestep
        self.frame_skip = 5  # Control frequency
        self.max_episode_steps = 1000

        # State/action dimensions
        # State: joint positions (8) + joint velocities (8) + torso orientation (4) + torso velocity (6)
        self.obs_dim = 8 + 8 + 4 + 6  # 26
        self.act_dim = 8  # 8 joint motors

        # Initial standing pose with human-like bent knees
        self.init_qpos = np.zeros(self.model.nq)
        # freejoint: 7 DOF (3 pos + 4 quat), then 8 joint angles
        self.init_qpos[2] = 0.48  # torso height (lower due to more bent knees)
        self.init_qpos[3] = 1.0   # quaternion w
        # Human-like stance: knees bent ~30-35 degrees (more bent start)
        self.init_qpos[7 + 2] = np.radians(35)  # right knee
        self.init_qpos[7 + 6] = np.radians(35)  # left knee
        # Hip pitch to compensate for knee bend
        self.init_qpos[7 + 1] = np.radians(-15)  # right hip pitch
        self.init_qpos[7 + 5] = np.radians(-15)  # left hip pitch

        self.step_count = 0

        # Gait phase tracking (0 to 2*pi = one full cycle)
        self.gait_phase = 0.0
        self.gait_freq = 1.5  # Hz - steps per second

    def _get_target_pose(self, phase):
        """Get target joint angles for given gait phase.

        Human-like gait reference trajectory.
        Phase 0-pi: right leg swing, left leg stance
        Phase pi-2pi: left leg swing, right leg stance
        """
        # Knee bend profile: max bend at mid-swing
        def swing_knee(p):  # p in [0, pi]
            # Peak at p=pi/2, ~50 degrees
            return np.radians(50) * np.sin(p)

        def stance_knee(p):  # slight bend throughout
            return np.radians(15)

        # Hip pitch profile
        def swing_hip(p):  # forward during swing
            return np.radians(-25 + 40 * (p / np.pi))  # -25 to +15

        def stance_hip(p):  # backward during stance
            return np.radians(15 - 30 * (p / np.pi))  # +15 to -15

        targets = np.zeros(8)

        if phase < np.pi:
            # Right leg swinging, left leg stance
            p = phase
            targets[1] = swing_hip(p)      # right hip pitch
            targets[2] = swing_knee(p)     # right knee - BEND!
            targets[5] = stance_hip(p)     # left hip pitch
            targets[6] = stance_knee(p)    # left knee
        else:
            # Left leg swinging, right leg stance
            p = phase - np.pi
            targets[1] = stance_hip(p)     # right hip pitch
            targets[2] = stance_knee(p)    # right knee
            targets[5] = swing_hip(p)      # left hip pitch
            targets[6] = swing_knee(p)     # left knee - BEND!

        return targets

    def reset(self):
        """Reset environment to initial state."""
        mujoco.mj_resetData(self.model, self.data)

        # Set initial pose with small random perturbation
        self.data.qpos[:] = self.init_qpos + np.random.uniform(-0.01, 0.01, self.model.nq)
        self.data.qvel[:] = np.random.uniform(-0.01, 0.01, self.model.nv)

        mujoco.mj_forward(self.model, self.data)
        self.step_count = 0
        self.gait_phase = 0.0

        return self._get_obs()

    def _get_obs(self):
        """Get observation vector."""
        # Joint positions (8 leg joints, skip freejoint)
        joint_pos = self.data.qpos[7:15].copy()

        # Joint velocities (8 leg joints, skip freejoint)
        joint_vel = self.data.qvel[6:14].copy()

        # Torso orientation (quaternion)
        torso_quat = self.data.qpos[3:7].copy()

        # Torso velocity (linear + angular)
        torso_vel = self.data.qvel[0:6].copy()

        obs = np.concatenate([joint_pos, joint_vel, torso_quat, torso_vel])
        return obs.astype(np.float32)

    def step(self, action):
        """Take a step in the environment."""
        # Clip actions to valid range
        action = np.clip(action, -1, 1)

        # Apply action to motors
        self.data.ctrl[:] = action

        # Step simulation
        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)

        self.step_count += 1

        # Advance gait phase
        dt = self.dt * self.frame_skip
        self.gait_phase += 2 * np.pi * self.gait_freq * dt
        self.gait_phase = self.gait_phase % (2 * np.pi)

        # Get observation
        obs = self._get_obs()

        # Calculate reward
        reward = self._get_reward()

        # Check termination
        done = self._is_done()

        # Render if needed
        if self.render_mode and self.viewer is not None:
            self.viewer.sync()

        return obs, reward, done, {}

    def _get_reward(self):
        """Calculate reward - TRAJECTORY TRACKING is PRIMARY."""
        reward = 0.0

        # Get current joint positions (8 joints)
        current_pos = self.data.qpos[7:15].copy()

        # Get target pose from reference trajectory
        target_pos = self._get_target_pose(self.gait_phase)

        # === PRIMARY REWARD: TRAJECTORY TRACKING ===
        # Especially for knees (indices 2 and 6)
        joint_weights = np.array([
            0.5,   # right_hip_roll
            1.0,   # right_hip_pitch
            3.0,   # right_knee - HIGHEST WEIGHT!
            0.3,   # right_ankle_roll
            0.5,   # left_hip_roll
            1.0,   # left_hip_pitch
            3.0,   # left_knee - HIGHEST WEIGHT!
            0.3,   # left_ankle_roll
        ])

        # Tracking error (weighted)
        tracking_error = np.sum(joint_weights * np.square(current_pos - target_pos))
        tracking_reward = 5.0 * np.exp(-2.0 * tracking_error)  # Max 5.0 when perfect
        reward += tracking_reward

        # === KNEE TRACKING BONUS (extra emphasis) ===
        right_knee_error = abs(current_pos[2] - target_pos[2])
        left_knee_error = abs(current_pos[6] - target_pos[6])

        # Bonus for good knee tracking
        if right_knee_error < np.radians(10):
            reward += 1.0
        if left_knee_error < np.radians(10):
            reward += 1.0

        # Penalty for bad knee tracking
        if right_knee_error > np.radians(30):
            reward -= 2.0
        if left_knee_error > np.radians(30):
            reward -= 2.0

        # === SECONDARY: Forward progress ===
        forward_vel = self.data.qvel[0]
        reward += 1.0 * forward_vel

        # === STABILITY ===
        torso_z = self.data.qpos[2]
        if torso_z > 0.35:
            reward += 0.5
        else:
            reward -= 2.0

        # Lateral penalty
        lateral_vel = abs(self.data.qvel[1])
        reward -= 0.3 * lateral_vel

        # Energy (minimal)
        ctrl_cost = 0.0005 * np.sum(np.square(self.data.ctrl))
        reward -= ctrl_cost

        return reward

    def _get_body_id(self, name):
        """Get body ID by name."""
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)

    def _is_done(self):
        """Check if episode is done."""
        # Fallen over
        torso_z = self.data.qpos[2]
        if torso_z < 0.25:
            return True

        # Too tilted
        torso_quat = self.data.qpos[3:7]
        # Simple tilt check: w component of quaternion should be close to 1
        if abs(torso_quat[0]) < 0.7:
            return True

        # Max steps reached
        if self.step_count >= self.max_episode_steps:
            return True

        return False

    def render(self):
        """Render the environment."""
        if self.viewer is None:
            try:
                import mujoco.viewer
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            except:
                print("Could not create viewer")
                self.render_mode = False

    def close(self):
        """Close the environment."""
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None


class SimplePolicy:
    """Simple neural network policy."""

    def __init__(self, obs_dim, act_dim, hidden_size=64):
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.hidden_size = hidden_size

        # Initialize weights with small random values
        self.w1 = np.random.randn(obs_dim, hidden_size) * 0.1
        self.b1 = np.zeros(hidden_size)
        self.w2 = np.random.randn(hidden_size, hidden_size) * 0.1
        self.b2 = np.zeros(hidden_size)
        self.w3 = np.random.randn(hidden_size, act_dim) * 0.1
        self.b3 = np.zeros(act_dim)

        # Log std for action noise
        self.log_std = np.zeros(act_dim)

    def forward(self, obs):
        """Forward pass through network."""
        x = np.tanh(obs @ self.w1 + self.b1)
        x = np.tanh(x @ self.w2 + self.b2)
        mean = np.tanh(x @ self.w3 + self.b3)
        return mean

    def get_action(self, obs, deterministic=False):
        """Get action from policy."""
        mean = self.forward(obs)
        if deterministic:
            return mean
        std = np.exp(self.log_std)
        action = mean + std * np.random.randn(self.act_dim)
        return np.clip(action, -1, 1)

    def get_params(self):
        """Get all parameters as flat array."""
        return np.concatenate([
            self.w1.flatten(), self.b1,
            self.w2.flatten(), self.b2,
            self.w3.flatten(), self.b3,
            self.log_std
        ])

    def set_params(self, params):
        """Set parameters from flat array."""
        idx = 0

        size = self.obs_dim * self.hidden_size
        self.w1 = params[idx:idx+size].reshape(self.obs_dim, self.hidden_size)
        idx += size

        size = self.hidden_size
        self.b1 = params[idx:idx+size]
        idx += size

        size = self.hidden_size * self.hidden_size
        self.w2 = params[idx:idx+size].reshape(self.hidden_size, self.hidden_size)
        idx += size

        size = self.hidden_size
        self.b2 = params[idx:idx+size]
        idx += size

        size = self.hidden_size * self.act_dim
        self.w3 = params[idx:idx+size].reshape(self.hidden_size, self.act_dim)
        idx += size

        size = self.act_dim
        self.b3 = params[idx:idx+size]
        idx += size

        self.log_std = params[idx:idx+self.act_dim]

    def save(self, path):
        """Save policy to file."""
        data = {
            'obs_dim': self.obs_dim,
            'act_dim': self.act_dim,
            'hidden_size': self.hidden_size,
            'w1': self.w1.tolist(),
            'b1': self.b1.tolist(),
            'w2': self.w2.tolist(),
            'b2': self.b2.tolist(),
            'w3': self.w3.tolist(),
            'b3': self.b3.tolist(),
            'log_std': self.log_std.tolist()
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"[SAVED] Policy saved to {path}")

    def load(self, path):
        """Load policy from file."""
        with open(path, 'r') as f:
            data = json.load(f)
        self.w1 = np.array(data['w1'])
        self.b1 = np.array(data['b1'])
        self.w2 = np.array(data['w2'])
        self.b2 = np.array(data['b2'])
        self.w3 = np.array(data['w3'])
        self.b3 = np.array(data['b3'])
        self.log_std = np.array(data['log_std'])
        print(f"[LOADED] Policy loaded from {path}")


class EvolutionStrategy:
    """Simple Evolution Strategy for policy optimization."""

    def __init__(self, policy, population_size=100, sigma=0.12, learning_rate=0.03):
        self.policy = policy
        self.population_size = population_size
        self.sigma = sigma
        self.learning_rate = learning_rate
        self.best_reward = -float('inf')

    def train_step(self, env):
        """One training step using ES."""
        params = self.policy.get_params()
        n_params = len(params)

        # Generate population
        noise = np.random.randn(self.population_size, n_params)
        rewards = np.zeros(self.population_size)

        # Evaluate population
        for i in range(self.population_size):
            # Positive perturbation
            self.policy.set_params(params + self.sigma * noise[i])
            rewards[i] = self._evaluate(env)

        # Normalize rewards
        rewards = (rewards - np.mean(rewards)) / (np.std(rewards) + 1e-8)

        # Update params
        grad = (1.0 / (self.population_size * self.sigma)) * (noise.T @ rewards)
        new_params = params + self.learning_rate * grad
        self.policy.set_params(new_params)

        # Evaluate new policy
        current_reward = self._evaluate(env)
        if current_reward > self.best_reward:
            self.best_reward = current_reward

        return current_reward

    def _evaluate(self, env, n_episodes=1):
        """Evaluate policy."""
        total_reward = 0
        for _ in range(n_episodes):
            obs = env.reset()
            done = False
            while not done:
                action = self.policy.get_action(obs, deterministic=True)
                obs, reward, done, _ = env.step(action)
                total_reward += reward
        return total_reward / n_episodes


def extract_timing(policy, env):
    """Extract timing parameters from trained policy for deployment."""
    print("\nExtracting timing parameters for deployment...")

    timing = {
        'gait_cycle': [],
        'description': 'Timing for one complete walking cycle'
    }

    # Run policy and record joint trajectories
    obs = env.reset()
    positions = []

    for step in range(200):  # One gait cycle worth of data
        action = policy.get_action(obs, deterministic=True)
        obs, _, done, _ = env.step(action)

        # Record joint positions
        joint_pos = env.data.qpos[7:15].copy()
        positions.append(joint_pos)

        if done:
            break

    positions = np.array(positions)

    # Analyze gait pattern
    for i, name in enumerate(JOINT_NAMES):
        joint_data = positions[:, i]
        timing['gait_cycle'].append({
            'joint': name,
            'min_angle': float(np.degrees(np.min(joint_data))),
            'max_angle': float(np.degrees(np.max(joint_data))),
            'mean_angle': float(np.degrees(np.mean(joint_data)))
        })

    # Save timing
    with open(TIMING_PATH, 'w') as f:
        json.dump(timing, f, indent=2)
    print(f"[SAVED] Timing saved to {TIMING_PATH}")

    return timing


def train(headless=False, episodes=500):
    """Train the walking policy."""
    print("="*50)
    print("BIPED WALKING TRAINING")
    print("="*50)
    print(f"\nServos: Right leg (4,5,6,7), Left leg (15,16,17,18)")
    print(f"Episodes: {episodes}")
    print(f"Mode: {'headless' if headless else 'with viewer'}\n")

    env = WalkingEnv(render=not headless)
    policy = SimplePolicy(env.obs_dim, env.act_dim)
    trainer = EvolutionStrategy(policy)

    if not headless:
        env.render()

    try:
        for episode in range(episodes):
            reward = trainer.train_step(env)

            if episode % 10 == 0:
                print(f"Episode {episode:4d}: Reward = {reward:8.2f}, Best = {trainer.best_reward:8.2f}")

            # Save checkpoint
            if episode % 100 == 0 and episode > 0:
                policy.save(POLICY_PATH)

    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")

    # Save final policy
    policy.save(POLICY_PATH)

    # Extract timing for deployment
    extract_timing(policy, env)

    env.close()
    print("\n[DONE] Training complete!")


def test():
    """Test saved policy."""
    print("="*50)
    print("TESTING WALKING POLICY")
    print("="*50)

    env = WalkingEnv(render=True)
    policy = SimplePolicy(env.obs_dim, env.act_dim)

    try:
        policy.load(POLICY_PATH)
    except FileNotFoundError:
        print(f"[ERROR] No policy found at {POLICY_PATH}")
        print("Train first with: python3 train_walking_mujoco.py")
        return

    env.render()

    try:
        for episode in range(10):
            obs = env.reset()
            total_reward = 0
            done = False
            steps = 0

            while not done:
                action = policy.get_action(obs, deterministic=True)
                obs, reward, done, _ = env.step(action)
                total_reward += reward
                steps += 1

                # Slow down for visualization
                import time
                time.sleep(0.01)

            print(f"Episode {episode+1}: Steps = {steps}, Reward = {total_reward:.2f}")

    except KeyboardInterrupt:
        print("\n[STOPPED]")

    env.close()


def main():
    if "--test" in sys.argv:
        test()
    elif "--headless" in sys.argv:
        train(headless=True)
    else:
        train(headless=False)


if __name__ == "__main__":
    main()
