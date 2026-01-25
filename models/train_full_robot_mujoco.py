#!/usr/bin/env python3
"""Train full humanoid robot using MuJoCo simulation.

All servos:
  - Head: 0 (left/right)
  - Right arm: 1 (shoulder), 2 (upper arm), 3 (forearm)
  - Left arm: 12 (shoulder), 13 (upper arm), 14 (forearm)
  - Right leg: 4 (hip roll), 5 (hip pitch), 6 (knee), 7 (ankle)
  - Left leg: 15 (hip roll), 16 (hip pitch), 17 (knee), 18 (ankle)

Usage:
  python3 train_full_robot_mujoco.py              # with viewer
  python3 train_full_robot_mujoco.py --headless   # headless training
  python3 train_full_robot_mujoco.py --test       # test saved policy
  python3 train_full_robot_mujoco.py --slow       # slow motion viewer
"""

import numpy as np
import json
import sys
import time

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print("MuJoCo not installed. Install with:")
    print("  pip install mujoco")
    exit(1)

# Servo to actuator index mapping
# Actuator order in XML: head, right_arm(3), left_arm(3), right_leg(4), left_leg(4)
SERVO_TO_ACTUATOR = {
    0: 0,    # head
    1: 1,    # right shoulder pitch
    2: 2,    # right shoulder roll
    3: 3,    # right elbow
    12: 4,   # left shoulder pitch
    13: 5,   # left shoulder roll
    14: 6,   # left elbow
    4: 7,    # right hip roll
    5: 8,    # right hip pitch
    6: 9,    # right knee
    7: 10,   # right ankle roll
    15: 11,  # left hip roll
    16: 12,  # left hip pitch
    17: 13,  # left knee
    18: 14,  # left ankle roll
}

ACTUATOR_NAMES = [
    "head",
    "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
    "left_shoulder_pitch", "left_shoulder_roll", "left_elbow",
    "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle_roll",
    "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_roll"
]

MODEL_PATH = '/home/jetson/work/biped_robot/models/full_robot.xml'
POLICY_PATH = '/home/jetson/work/biped_robot/models/full_robot_policy.json'


class FullRobotEnv:
    """MuJoCo environment for full humanoid robot."""

    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self.data = mujoco.MjData(self.model)
        self.viewer = None

        # Environment parameters
        self.dt = self.model.opt.timestep
        self.frame_skip = 5
        self.max_episode_steps = 1000

        # Dimensions
        self.n_actuators = 15  # 1 head + 6 arms + 8 legs
        self.obs_dim = self._get_obs_dim()
        self.act_dim = self.n_actuators

        # Initial pose - standing with bent knees
        self.init_qpos = np.zeros(self.model.nq)
        # Freejoint: pos(3) + quat(4)
        self.init_qpos[2] = 0.42  # torso height (lower, feet on ground)
        self.init_qpos[3] = 1.0   # quaternion w

        # Joint indices after freejoint (7):
        # 7=head, 8-10=right_arm, 11-13=left_arm
        # 14=right_hip_roll, 15=right_hip_pitch, 16=right_knee, 17=right_ankle
        # 18=left_hip_roll, 19=left_hip_pitch, 20=left_knee, 21=left_ankle

        # Standing pose with bent knees for stability
        self.init_qpos[7 + 8] = np.radians(15)   # right hip pitch (forward)
        self.init_qpos[7 + 9] = np.radians(30)   # right knee (bent)
        self.init_qpos[7 + 12] = np.radians(15)  # left hip pitch (forward)
        self.init_qpos[7 + 13] = np.radians(30)  # left knee (bent)

        self.step_count = 0

    def _get_obs_dim(self):
        """Calculate observation dimension."""
        # Joint positions + velocities + torso quat + torso vel
        n_joints = self.model.nq - 7  # exclude freejoint
        n_vels = self.model.nv - 6    # exclude freejoint vel
        return n_joints + n_vels + 4 + 6

    def reset(self):
        """Reset to initial state."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self.init_qpos.copy()
        # Small noise, but don't disturb torso position/orientation much
        noise = np.random.uniform(-0.005, 0.005, self.model.nq)
        noise[0:7] = 0  # No noise on freejoint (keep standing)
        self.data.qpos[:] += noise
        self.data.qvel[:] = 0  # Start with zero velocity
        mujoco.mj_forward(self.model, self.data)

        # Let robot settle for a moment
        for _ in range(50):
            mujoco.mj_step(self.model, self.data)

        self.step_count = 0
        return self._get_obs()

    def _get_obs(self):
        """Get observation."""
        joint_pos = self.data.qpos[7:].copy()
        joint_vel = self.data.qvel[6:].copy()
        torso_quat = self.data.qpos[3:7].copy()
        torso_vel = self.data.qvel[0:6].copy()
        return np.concatenate([joint_pos, joint_vel, torso_quat, torso_vel]).astype(np.float32)

    def step(self, action):
        """Take a step."""
        action = np.clip(action, -1, 1)
        self.data.ctrl[:] = action

        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)

        self.step_count += 1
        obs = self._get_obs()
        reward = self._get_reward()
        done = self._is_done()

        return obs, reward, done, {}

    def _get_reward(self):
        """Calculate reward."""
        reward = 0.0

        # Forward velocity
        forward_vel = self.data.qvel[0]
        reward += 2.0 * forward_vel

        # Stay upright
        torso_z = self.data.qpos[2]
        if torso_z > 0.45:
            reward += 1.0
        else:
            reward -= 2.0

        # Energy penalty
        ctrl_cost = 0.005 * np.sum(np.square(self.data.ctrl))
        reward -= ctrl_cost

        # Lateral penalty
        lateral_vel = abs(self.data.qvel[1])
        reward -= 0.3 * lateral_vel

        return reward

    def _is_done(self):
        """Check if done."""
        torso_z = self.data.qpos[2]
        if torso_z < 0.3:
            return True
        torso_quat = self.data.qpos[3:7]
        if abs(torso_quat[0]) < 0.6:
            return True
        if self.step_count >= self.max_episode_steps:
            return True
        return False

    def render(self):
        """Create viewer."""
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.cam.distance = 2.0
            self.viewer.cam.elevation = -20
            self.viewer.cam.azimuth = 90

    def sync_viewer(self):
        """Sync viewer."""
        if self.viewer is not None:
            self.viewer.sync()

    def close(self):
        """Close viewer."""
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None


class Policy:
    """Neural network policy."""

    def __init__(self, obs_dim, act_dim, hidden=64):
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.hidden = hidden

        # Weights
        self.w1 = np.random.randn(obs_dim, hidden) * 0.1
        self.b1 = np.zeros(hidden)
        self.w2 = np.random.randn(hidden, hidden) * 0.1
        self.b2 = np.zeros(hidden)
        self.w3 = np.random.randn(hidden, act_dim) * 0.1
        self.b3 = np.zeros(act_dim)
        self.log_std = np.zeros(act_dim) - 0.5

    def forward(self, obs):
        """Forward pass."""
        x = np.tanh(obs @ self.w1 + self.b1)
        x = np.tanh(x @ self.w2 + self.b2)
        return np.tanh(x @ self.w3 + self.b3)

    def get_action(self, obs, deterministic=False):
        """Get action."""
        mean = self.forward(obs)
        if deterministic:
            return mean
        std = np.exp(self.log_std)
        return np.clip(mean + std * np.random.randn(self.act_dim), -1, 1)

    def get_params(self):
        """Get flat params."""
        return np.concatenate([
            self.w1.flatten(), self.b1,
            self.w2.flatten(), self.b2,
            self.w3.flatten(), self.b3,
            self.log_std
        ])

    def set_params(self, params):
        """Set params from flat array."""
        idx = 0
        size = self.obs_dim * self.hidden
        self.w1 = params[idx:idx+size].reshape(self.obs_dim, self.hidden)
        idx += size
        self.b1 = params[idx:idx+self.hidden]
        idx += self.hidden
        size = self.hidden * self.hidden
        self.w2 = params[idx:idx+size].reshape(self.hidden, self.hidden)
        idx += size
        self.b2 = params[idx:idx+self.hidden]
        idx += self.hidden
        size = self.hidden * self.act_dim
        self.w3 = params[idx:idx+size].reshape(self.hidden, self.act_dim)
        idx += size
        self.b3 = params[idx:idx+self.act_dim]
        idx += self.act_dim
        self.log_std = params[idx:idx+self.act_dim]

    def save(self, path):
        """Save to file."""
        data = {
            'obs_dim': self.obs_dim,
            'act_dim': self.act_dim,
            'hidden': self.hidden,
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
        print(f"[SAVED] {path}")

    def load(self, path):
        """Load from file."""
        with open(path, 'r') as f:
            data = json.load(f)
        self.w1 = np.array(data['w1'])
        self.b1 = np.array(data['b1'])
        self.w2 = np.array(data['w2'])
        self.b2 = np.array(data['b2'])
        self.w3 = np.array(data['w3'])
        self.b3 = np.array(data['b3'])
        self.log_std = np.array(data['log_std'])
        print(f"[LOADED] {path}")


class ESTrainer:
    """Evolution Strategy trainer."""

    def __init__(self, policy, pop_size=40, sigma=0.05, lr=0.02):
        self.policy = policy
        self.pop_size = pop_size
        self.sigma = sigma
        self.lr = lr
        self.best_reward = -float('inf')

    def train_step(self, env, render=False, slow=False):
        """One training step."""
        params = self.policy.get_params()
        n = len(params)

        noise = np.random.randn(self.pop_size, n)
        rewards = np.zeros(self.pop_size)

        for i in range(self.pop_size):
            self.policy.set_params(params + self.sigma * noise[i])
            rewards[i] = self._evaluate(env, render=(render and i == 0), slow=slow)

        # Normalize and update
        rewards = (rewards - np.mean(rewards)) / (np.std(rewards) + 1e-8)
        grad = (1.0 / (self.pop_size * self.sigma)) * (noise.T @ rewards)
        self.policy.set_params(params + self.lr * grad)

        # Evaluate new policy
        current = self._evaluate(env, render=render, slow=slow)
        if current > self.best_reward:
            self.best_reward = current

        return current

    def _evaluate(self, env, render=False, slow=False, n_ep=1):
        """Evaluate policy."""
        total = 0
        for _ in range(n_ep):
            obs = env.reset()
            done = False
            while not done:
                action = self.policy.get_action(obs, deterministic=True)
                obs, reward, done, _ = env.step(action)
                total += reward

                if render:
                    env.sync_viewer()
                    if slow:
                        time.sleep(0.02)  # Slow down visualization

        return total / n_ep


def train(headless=False, slow=False, episodes=1000):
    """Train the robot."""
    print("=" * 60)
    print("FULL HUMANOID ROBOT TRAINING")
    print("=" * 60)
    print(f"\nServos:")
    print(f"  Head: 0")
    print(f"  Right arm: 1, 2, 3")
    print(f"  Left arm: 12, 13, 14")
    print(f"  Right leg: 4, 5, 6, 7")
    print(f"  Left leg: 15, 16, 17, 18")
    print(f"\nEpisodes: {episodes}")
    print(f"Mode: {'headless' if headless else 'with viewer'}")
    if slow:
        print("Speed: SLOW MOTION")
    print()

    env = FullRobotEnv()
    policy = Policy(env.obs_dim, env.act_dim)
    trainer = ESTrainer(policy)

    if not headless:
        env.render()
        print("[VIEWER] Camera controls:")
        print("  - Left mouse: rotate")
        print("  - Right mouse: pan")
        print("  - Scroll: zoom")
        print()

    try:
        for ep in range(episodes):
            reward = trainer.train_step(env, render=not headless, slow=slow)

            if ep % 5 == 0:
                print(f"Episode {ep:4d}: Reward = {reward:8.2f}, Best = {trainer.best_reward:8.2f}")

            if ep % 50 == 0 and ep > 0:
                policy.save(POLICY_PATH)

    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")

    policy.save(POLICY_PATH)
    env.close()
    print("\n[DONE]")


def test(slow=True):
    """Test saved policy."""
    print("=" * 60)
    print("TESTING FULL ROBOT POLICY")
    print("=" * 60)

    env = FullRobotEnv()
    policy = Policy(env.obs_dim, env.act_dim)

    try:
        policy.load(POLICY_PATH)
    except FileNotFoundError:
        print(f"[ERROR] No policy at {POLICY_PATH}")
        print("Train first: python3 train_full_robot_mujoco.py")
        return

    env.render()
    print("\n[RUNNING] Press Ctrl+C to stop\n")

    try:
        for ep in range(100):
            obs = env.reset()
            total_reward = 0
            steps = 0
            done = False

            while not done:
                action = policy.get_action(obs, deterministic=True)
                obs, reward, done, _ = env.step(action)
                total_reward += reward
                steps += 1

                env.sync_viewer()
                if slow:
                    time.sleep(0.02)

            print(f"Episode {ep+1}: Steps = {steps:4d}, Reward = {total_reward:8.2f}")

    except KeyboardInterrupt:
        print("\n[STOPPED]")

    env.close()


def main():
    slow = "--slow" in sys.argv

    if "--test" in sys.argv:
        test(slow=True)
    elif "--headless" in sys.argv:
        train(headless=True, slow=False)
    else:
        train(headless=False, slow=slow)


if __name__ == "__main__":
    main()
