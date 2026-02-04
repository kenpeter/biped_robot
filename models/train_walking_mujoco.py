#!/usr/bin/env python3
"""Train biped walking using MuJoCo simulation with PPO.

Trains servos 4,5,6,7 (right leg) and 15,16,17,18 (left leg) to walk.
Uses PPO (Proximal Policy Optimization) for better learning.

Usage:
  python3 train_walking_mujoco.py              # train with viewer
  python3 train_walking_mujoco.py --headless   # headless training
  python3 train_walking_mujoco.py --test       # test saved policy
  python3 train_walking_mujoco.py --resume     # resume training
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import json
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

# Servo mapping
SERVO_TO_JOINT = {
    4: 0, 5: 1, 6: 2, 7: 3,      # right leg
    15: 4, 16: 5, 17: 6, 18: 7,  # left leg
}

JOINT_NAMES = [
    "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle_roll",
    "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_roll"
]

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, 'biped_legs_robot.xml')
PPO_MODEL_PATH = os.path.join(SCRIPT_DIR, 'walking_ppo_model')
TIMING_PATH = os.path.join(SCRIPT_DIR, 'walking_timing.json')


class WalkingEnv(gym.Env):
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

        # Gait phase tracking
        self.gait_phase = 0.0
        self.gait_freq = 1.2  # Hz

        # Action space: 8 joint torques [-1, 1]
        self.action_space = spaces.Box(low=-1, high=1, shape=(8,), dtype=np.float32)

        # Observation: joint pos(8) + vel(8) + quat(4) + torso_vel(6) + phase(2) + target(8) = 36
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(36,), dtype=np.float32
        )

        # Initial pose with bent knees
        self.init_qpos = np.zeros(self.model.nq)
        self.init_qpos[2] = 0.48
        self.init_qpos[3] = 1.0
        self.init_qpos[7 + 2] = np.radians(30)  # right knee
        self.init_qpos[7 + 6] = np.radians(30)  # left knee
        self.init_qpos[7 + 1] = np.radians(-12)
        self.init_qpos[7 + 5] = np.radians(-12)

    def _get_target_pose(self, phase):
        """Human-like gait reference - knees MUST bend during swing."""
        def swing_knee(p):
            return np.radians(50) * np.sin(p)  # Peak 50° at mid-swing

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
            targets[2] = swing_knee(p)  # RIGHT KNEE BENDS
            targets[5] = stance_hip(p)
            targets[6] = stance_knee(p)
        else:
            p = phase - np.pi
            targets[1] = stance_hip(p)
            targets[2] = stance_knee(p)
            targets[5] = swing_hip(p)
            targets[6] = swing_knee(p)  # LEFT KNEE BENDS

        return targets

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        mujoco.mj_resetData(self.model, self.data)
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

        # Phase as sin/cos for smooth representation
        phase_obs = np.array([np.sin(self.gait_phase), np.cos(self.gait_phase)])

        # Target pose - policy sees what it should do
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
        """Reward: trajectory tracking (especially knees) + forward progress."""
        reward = 0.0

        current_pos = self.data.qpos[7:15].copy()
        target_pos = self._get_target_pose(self.gait_phase)

        # === TRAJECTORY TRACKING (primary) ===
        # Knees get 5x weight - MUST follow reference
        joint_weights = np.array([0.5, 1.0, 5.0, 0.3, 0.5, 1.0, 5.0, 0.3])
        tracking_error = np.sum(joint_weights * np.square(current_pos - target_pos))
        tracking_reward = 4.0 * np.exp(-1.5 * tracking_error)
        reward += tracking_reward

        # === KNEE TRACKING BONUS ===
        right_knee_error = abs(current_pos[2] - target_pos[2])
        left_knee_error = abs(current_pos[6] - target_pos[6])

        if right_knee_error < np.radians(8):
            reward += 1.0
        if left_knee_error < np.radians(8):
            reward += 1.0
        if right_knee_error > np.radians(25):
            reward -= 1.5
        if left_knee_error > np.radians(25):
            reward -= 1.5

        # === FORWARD VELOCITY ===
        forward_vel = self.data.qvel[0]
        reward += 1.5 * np.clip(forward_vel, -0.5, 1.0)

        # === STABILITY ===
        torso_z = self.data.qpos[2]
        torso_quat = self.data.qpos[3:7]

        if torso_z > 0.38:
            reward += 0.3
        else:
            reward -= 1.5

        # Upright bonus
        reward += 0.5 * torso_quat[0]

        # === PENALTIES ===
        lateral_vel = abs(self.data.qvel[1])
        reward -= 0.2 * lateral_vel

        angular_vel = abs(self.data.qvel[5])
        reward -= 0.1 * angular_vel

        ctrl_cost = 0.0002 * np.sum(np.square(self.data.ctrl))
        reward -= ctrl_cost

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


class ProgressCallback(BaseCallback):
    """Log training progress."""

    def __init__(self, verbose=0):
        super().__init__(verbose)
        self.episode_rewards = []

    def _on_step(self):
        if len(self.model.ep_info_buffer) > 0:
            ep_info = self.model.ep_info_buffer[-1]
            if 'r' in ep_info:
                self.episode_rewards.append(ep_info['r'])
                if len(self.episode_rewards) % 20 == 0:
                    avg = np.mean(self.episode_rewards[-20:])
                    print(f"Episodes: {len(self.episode_rewards)}, Avg Reward: {avg:.1f}")
        return True


def make_env(render_mode=None):
    def _init():
        return WalkingEnv(render_mode=render_mode)
    return _init


def train(headless=True, total_timesteps=2_000_000, resume=False):
    print("=" * 50)
    print("BIPED WALKING - PPO TRAINING")
    print("=" * 50)
    print(f"Timesteps: {total_timesteps:,}")
    print(f"Mode: {'headless' if headless else 'with viewer'}")
    print(f"Resume: {resume}")
    print()

    # Parallel envs for faster training
    n_envs = 8 if headless else 1
    render_mode = None if headless else "human"

    if headless and n_envs > 1:
        env = SubprocVecEnv([make_env() for _ in range(n_envs)])
    else:
        env = DummyVecEnv([make_env(render_mode)])

    # Create or load model
    if resume and os.path.exists(PPO_MODEL_PATH + ".zip"):
        print(f"Resuming from {PPO_MODEL_PATH}.zip")
        model = PPO.load(PPO_MODEL_PATH, env=env)
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
            ent_coef=0.01,
            vf_coef=0.5,
            max_grad_norm=0.5,
            policy_kwargs=dict(net_arch=dict(pi=[256, 256], vf=[256, 256])),
            verbose=1,
            tensorboard_log=os.path.join(SCRIPT_DIR, "walking_tb_logs")
        )

    checkpoint_cb = CheckpointCallback(
        save_freq=50000 // n_envs,
        save_path=SCRIPT_DIR,
        name_prefix="walking_checkpoint"
    )

    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=[checkpoint_cb, ProgressCallback()],
            progress_bar=True
        )
    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")

    model.save(PPO_MODEL_PATH)
    print(f"\n[SAVED] {PPO_MODEL_PATH}.zip")

    env.close()


def test():
    print("=" * 50)
    print("TESTING WALKING POLICY")
    print("=" * 50)

    if not os.path.exists(PPO_MODEL_PATH + ".zip"):
        print(f"[ERROR] No model at {PPO_MODEL_PATH}.zip")
        print("Train first: python3 train_walking_mujoco.py --headless")
        return

    env = WalkingEnv(render_mode="human")
    model = PPO.load(PPO_MODEL_PATH)

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

            print(f"Episode {episode+1}: Steps={steps}, Reward={total_reward:.1f}")

    except KeyboardInterrupt:
        print("\n[STOPPED]")

    env.close()


def main():
    if "--test" in sys.argv:
        test()
    elif "--resume" in sys.argv:
        headless = "--headless" in sys.argv
        train(headless=headless, resume=True)
    elif "--headless" in sys.argv:
        train(headless=True)
    else:
        train(headless=False)


if __name__ == "__main__":
    main()
