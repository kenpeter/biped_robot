#!/usr/bin/env python3
"""Train biped walking using PPO (Proximal Policy Optimization).

PPO is much more effective than Evolution Strategy for locomotion.

Usage:
  python3 train_walking_ppo.py              # train with viewer
  python3 train_walking_ppo.py --headless   # headless training
  python3 train_walking_ppo.py --test       # test saved policy
  python3 train_walking_ppo.py --resume     # resume training
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import sys
import os

try:
    import mujoco
except ImportError:
    print("MuJoCo not installed: pip install mujoco")
    exit(1)

try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
except ImportError:
    print("stable-baselines3 not installed: pip install stable-baselines3")
    exit(1)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, 'biped_legs_robot.xml')
PPO_PATH = os.path.join(SCRIPT_DIR, 'walking_ppo_model')


class BipedWalkingEnv(gym.Env):
    """Gymnasium environment for biped walking with human-like gait."""

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, render_mode=None):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self.data = mujoco.MjData(self.model)
        self.render_mode = render_mode
        self.viewer = None

        # Timing
        self.dt = self.model.opt.timestep
        self.frame_skip = 5
        self.max_episode_steps = 1000
        self.step_count = 0

        # Gait phase
        self.gait_phase = 0.0
        self.gait_freq = 1.2  # Hz - slower for stability

        # Action and observation spaces
        # Actions: 8 joint torques normalized to [-1, 1]
        self.action_space = spaces.Box(low=-1, high=1, shape=(8,), dtype=np.float32)

        # Observations: joint pos (8) + joint vel (8) + torso quat (4) + torso vel (6)
        #              + gait phase (2: sin/cos) + target pose (8) = 36
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(36,), dtype=np.float32
        )

        # Initial pose
        self.init_qpos = np.zeros(self.model.nq)
        self.init_qpos[2] = 0.48  # torso height
        self.init_qpos[3] = 1.0   # quaternion w
        self.init_qpos[7 + 2] = np.radians(30)  # right knee
        self.init_qpos[7 + 6] = np.radians(30)  # left knee
        self.init_qpos[7 + 1] = np.radians(-12)  # right hip pitch
        self.init_qpos[7 + 5] = np.radians(-12)  # left hip pitch

    def _get_target_pose(self, phase):
        """Human-like gait reference trajectory."""
        def swing_knee(p):
            # Peak 45° at mid-swing
            return np.radians(45) * np.sin(p)

        def stance_knee(p):
            return np.radians(12)

        def swing_hip(p):
            return np.radians(-20 + 35 * (p / np.pi))

        def stance_hip(p):
            return np.radians(15 - 25 * (p / np.pi))

        targets = np.zeros(8)

        if phase < np.pi:
            p = phase
            targets[1] = swing_hip(p)
            targets[2] = swing_knee(p)  # right knee bends
            targets[5] = stance_hip(p)
            targets[6] = stance_knee(p)
        else:
            p = phase - np.pi
            targets[1] = stance_hip(p)
            targets[2] = stance_knee(p)
            targets[5] = swing_hip(p)
            targets[6] = swing_knee(p)  # left knee bends

        return targets

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        mujoco.mj_resetData(self.model, self.data)

        # Small random perturbation
        self.data.qpos[:] = self.init_qpos + self.np_random.uniform(-0.02, 0.02, self.model.nq)
        self.data.qvel[:] = self.np_random.uniform(-0.05, 0.05, self.model.nv)

        mujoco.mj_forward(self.model, self.data)

        self.step_count = 0
        self.gait_phase = 0.0

        return self._get_obs(), {}

    def _get_obs(self):
        joint_pos = self.data.qpos[7:15].copy()
        joint_vel = np.clip(self.data.qvel[6:14].copy(), -10, 10)
        torso_quat = self.data.qpos[3:7].copy()
        torso_vel = np.clip(self.data.qvel[0:6].copy(), -10, 10)

        # Gait phase as sin/cos (continuous representation)
        phase_obs = np.array([np.sin(self.gait_phase), np.cos(self.gait_phase)])

        # Target pose for current phase
        target_pose = self._get_target_pose(self.gait_phase)

        obs = np.concatenate([joint_pos, joint_vel, torso_quat, torso_vel, phase_obs, target_pose])
        return obs.astype(np.float32)

    def step(self, action):
        action = np.clip(action, -1, 1)
        self.data.ctrl[:] = action

        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)

        self.step_count += 1

        # Advance gait phase
        dt = self.dt * self.frame_skip
        self.gait_phase += 2 * np.pi * self.gait_freq * dt
        self.gait_phase = self.gait_phase % (2 * np.pi)

        obs = self._get_obs()
        reward = self._get_reward()
        terminated = self._is_terminated()
        truncated = self.step_count >= self.max_episode_steps

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, truncated, {}

    def _get_reward(self):
        reward = 0.0

        # Current and target joint positions
        current_pos = self.data.qpos[7:15].copy()
        target_pos = self._get_target_pose(self.gait_phase)

        # === 1. TRAJECTORY TRACKING (primary) ===
        joint_weights = np.array([0.5, 1.0, 4.0, 0.3, 0.5, 1.0, 4.0, 0.3])  # knees = 4x
        tracking_error = np.sum(joint_weights * np.square(current_pos - target_pos))
        tracking_reward = 3.0 * np.exp(-1.5 * tracking_error)
        reward += tracking_reward

        # === 2. KNEE TRACKING BONUS ===
        right_knee_error = abs(current_pos[2] - target_pos[2])
        left_knee_error = abs(current_pos[6] - target_pos[6])

        if right_knee_error < np.radians(8):
            reward += 0.5
        if left_knee_error < np.radians(8):
            reward += 0.5

        # === 3. FORWARD VELOCITY ===
        forward_vel = self.data.qvel[0]
        reward += 1.5 * np.clip(forward_vel, -0.5, 1.0)

        # === 4. STABILITY ===
        torso_z = self.data.qpos[2]
        torso_quat = self.data.qpos[3:7]

        # Height reward
        if torso_z > 0.38:
            reward += 0.3
        else:
            reward -= 1.0

        # Upright reward (quaternion w close to 1)
        upright = torso_quat[0]
        reward += 0.5 * upright

        # === 5. PENALTIES ===
        # Lateral velocity
        lateral_vel = abs(self.data.qvel[1])
        reward -= 0.2 * lateral_vel

        # Angular velocity (spinning)
        angular_vel = abs(self.data.qvel[5])
        reward -= 0.1 * angular_vel

        # Energy
        ctrl_cost = 0.0003 * np.sum(np.square(self.data.ctrl))
        reward -= ctrl_cost

        # Joint velocity (smoothness)
        joint_vel = self.data.qvel[6:14]
        reward -= 0.001 * np.sum(np.square(joint_vel))

        return reward

    def _is_terminated(self):
        torso_z = self.data.qpos[2]
        if torso_z < 0.25:
            return True

        torso_quat = self.data.qpos[3:7]
        if abs(torso_quat[0]) < 0.6:
            return True

        return False

    def render(self):
        if self.render_mode == "human":
            if self.viewer is None:
                import mujoco.viewer
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None


class RewardLoggerCallback(BaseCallback):
    """Log training progress."""

    def __init__(self, verbose=0):
        super().__init__(verbose)
        self.episode_rewards = []

    def _on_step(self):
        if len(self.model.ep_info_buffer) > 0:
            ep_info = self.model.ep_info_buffer[-1]
            if 'r' in ep_info:
                self.episode_rewards.append(ep_info['r'])
                if len(self.episode_rewards) % 10 == 0:
                    avg = np.mean(self.episode_rewards[-10:])
                    print(f"Episodes: {len(self.episode_rewards)}, Avg Reward: {avg:.2f}")
        return True


def make_env(render_mode=None):
    def _init():
        return BipedWalkingEnv(render_mode=render_mode)
    return _init


def train(headless=True, total_timesteps=2_000_000, resume=False):
    print("=" * 50)
    print("BIPED WALKING - PPO TRAINING")
    print("=" * 50)
    print(f"Timesteps: {total_timesteps:,}")
    print(f"Mode: {'headless' if headless else 'with viewer'}")
    print(f"Resume: {resume}")
    print()

    # Create environment
    n_envs = 8 if headless else 1
    render_mode = None if headless else "human"

    if headless and n_envs > 1:
        env = SubprocVecEnv([make_env() for _ in range(n_envs)])
    else:
        env = DummyVecEnv([make_env(render_mode)])

    # Create or load model
    if resume and os.path.exists(PPO_PATH + ".zip"):
        print(f"Resuming from {PPO_PATH}.zip")
        model = PPO.load(PPO_PATH, env=env)
    else:
        print("Creating new PPO model")
        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.01,  # Encourage exploration
            vf_coef=0.5,
            max_grad_norm=0.5,
            policy_kwargs=dict(
                net_arch=dict(pi=[256, 256], vf=[256, 256])
            ),
            verbose=1,
            tensorboard_log=os.path.join(SCRIPT_DIR, "walking_tb_logs")
        )

    # Callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=50000 // n_envs,
        save_path=SCRIPT_DIR,
        name_prefix="walking_ppo_checkpoint"
    )

    # Train
    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=[checkpoint_callback, RewardLoggerCallback()],
            progress_bar=True
        )
    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")

    # Save
    model.save(PPO_PATH)
    print(f"\n[SAVED] Model saved to {PPO_PATH}.zip")

    env.close()


def test():
    print("=" * 50)
    print("TESTING PPO WALKING POLICY")
    print("=" * 50)

    if not os.path.exists(PPO_PATH + ".zip"):
        print(f"[ERROR] No model at {PPO_PATH}.zip")
        print("Train first: python3 train_walking_ppo.py --headless")
        return

    env = BipedWalkingEnv(render_mode="human")
    model = PPO.load(PPO_PATH)

    try:
        for episode in range(10):
            obs, _ = env.reset()
            total_reward = 0
            done = False
            steps = 0

            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, _ = env.step(action)
                total_reward += reward
                done = terminated or truncated
                steps += 1

                import time
                time.sleep(0.01)

            print(f"Episode {episode+1}: Steps={steps}, Reward={total_reward:.2f}")

    except KeyboardInterrupt:
        print("\n[STOPPED]")

    env.close()


def main():
    if "--test" in sys.argv:
        test()
    elif "--resume" in sys.argv:
        train(headless="--headless" in sys.argv, resume=True)
    elif "--headless" in sys.argv:
        train(headless=True)
    else:
        train(headless=False)


if __name__ == "__main__":
    main()
