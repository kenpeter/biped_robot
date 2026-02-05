#!/usr/bin/env python3
"""Train walking with frozen upper body using MuJoCo + PPO.

Uses full_robot.xml but FREEZES upper body (head + arms).
Only trains legs (8 DOF).

Full robot actuators (15):
  0: head, 1-3: right arm, 4-6: left arm  <- FROZEN
  7-10: right leg, 11-14: left leg        <- TRAINED

Usage:
  python3 train_walking_mujoco.py              # train with viewer
  python3 train_walking_mujoco.py --headless   # headless training
  python3 train_walking_mujoco.py --test       # test saved policy
  python3 train_walking_mujoco.py --resume     # resume training
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
MODEL_PATH = os.path.join(SCRIPT_DIR, 'full_robot.xml')
PPO_MODEL_PATH = os.path.join(SCRIPT_DIR, 'walking_ppo_model')

# Full robot actuator indices
HEAD = 0
RIGHT_SHOULDER_PITCH = 1
RIGHT_SHOULDER_ROLL = 2
RIGHT_ELBOW = 3
LEFT_SHOULDER_PITCH = 4
LEFT_SHOULDER_ROLL = 5
LEFT_ELBOW = 6
RIGHT_HIP_ROLL = 7
RIGHT_HIP_PITCH = 8
RIGHT_KNEE = 9
RIGHT_ANKLE = 10
LEFT_HIP_ROLL = 11
LEFT_HIP_PITCH = 12
LEFT_KNEE = 13
LEFT_ANKLE = 14

# Frozen (upper body): 0-6
# Trained (legs): 7-14
FROZEN_ACTUATORS = [0, 1, 2, 3, 4, 5, 6]  # head + arms
LEG_ACTUATORS = [7, 8, 9, 10, 11, 12, 13, 14]  # legs only
N_LEG_ACTIONS = 8


class WalkingEnv(gym.Env):
    """Full robot with frozen upper body - only legs trained."""

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, render_mode=None):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self.data = mujoco.MjData(self.model)
        self.render_mode = render_mode
        self.viewer = None

        self.dt = self.model.opt.timestep
        self.frame_skip = 5
        self.max_episode_steps = 1000
        self.step_count = 0

        # Gait phase
        self.gait_phase = 0.0
        self.gait_freq = 1.2  # Hz

        # Action: only 8 leg actuators
        self.action_space = spaces.Box(low=-1, high=1, shape=(N_LEG_ACTIONS,), dtype=np.float32)

        # Observation: leg pos(8) + leg vel(8) + quat(4) + torso_vel(6) + phase(2) + target(8) = 36
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(36,), dtype=np.float32
        )

        # Initial pose
        self.init_qpos = np.zeros(self.model.nq)
        self.init_qpos[2] = 0.30  # torso height
        self.init_qpos[3] = 1.0   # quaternion w

        # Bent knees for stability
        self.init_qpos[7 + RIGHT_KNEE] = np.radians(25)
        self.init_qpos[7 + LEFT_KNEE] = np.radians(25)
        self.init_qpos[7 + RIGHT_HIP_PITCH] = np.radians(-10)
        self.init_qpos[7 + LEFT_HIP_PITCH] = np.radians(-10)

        # Frozen upper body pose (arms down, head centered)
        self.frozen_ctrl = np.zeros(len(FROZEN_ACTUATORS))
        # Arms slightly bent, close to body
        self.frozen_ctrl[RIGHT_ELBOW] = 0.2  # slight bend
        self.frozen_ctrl[LEFT_ELBOW] = 0.2

    def _get_target_pose(self, phase):
        """Human-like gait for legs only."""
        targets = np.zeros(N_LEG_ACTIONS)

        # Indices within leg actions (0-7)
        R_HIP_ROLL = 0
        R_HIP_PITCH = 1
        R_KNEE = 2
        R_ANKLE = 3
        L_HIP_ROLL = 4
        L_HIP_PITCH = 5
        L_KNEE = 6
        L_ANKLE = 7

        def swing_knee(p):
            return np.radians(45) * np.sin(p)

        def stance_knee(p):
            return np.radians(15)

        def swing_hip(p):
            return np.radians(-20 + 35 * (p / np.pi))

        def stance_hip(p):
            return np.radians(15 - 25 * (p / np.pi))

        def swing_ankle(p):
            return np.radians(-10 + 15 * np.sin(p))

        def stance_ankle(p):
            return np.radians(5)

        if phase < np.pi:
            p = phase
            # Right leg swing
            targets[R_HIP_PITCH] = swing_hip(p)
            targets[R_KNEE] = swing_knee(p)
            targets[R_ANKLE] = swing_ankle(p)
            # Left leg stance
            targets[L_HIP_PITCH] = stance_hip(p)
            targets[L_KNEE] = stance_knee(p)
            targets[L_ANKLE] = stance_ankle(p)
        else:
            p = phase - np.pi
            # Left leg swing
            targets[L_HIP_PITCH] = swing_hip(p)
            targets[L_KNEE] = swing_knee(p)
            targets[L_ANKLE] = swing_ankle(p)
            # Right leg stance
            targets[R_HIP_PITCH] = stance_hip(p)
            targets[R_KNEE] = stance_knee(p)
            targets[R_ANKLE] = stance_ankle(p)

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
        # Only leg joint positions/velocities
        leg_pos = np.array([self.data.qpos[7 + i] for i in LEG_ACTUATORS])
        leg_vel = np.clip([self.data.qvel[6 + i] for i in LEG_ACTUATORS], -10, 10)

        torso_quat = self.data.qpos[3:7].copy()
        torso_vel = np.clip(self.data.qvel[0:6].copy(), -10, 10)

        phase_obs = np.array([np.sin(self.gait_phase), np.cos(self.gait_phase)])
        target_pose = self._get_target_pose(self.gait_phase)

        obs = np.concatenate([leg_pos, leg_vel, torso_quat, torso_vel, phase_obs, target_pose])
        return obs.astype(np.float32)

    def step(self, action):
        action = np.clip(action, -1, 1)

        # Apply frozen controls to upper body
        for i, act_idx in enumerate(FROZEN_ACTUATORS):
            self.data.ctrl[act_idx] = self.frozen_ctrl[i]

        # Apply policy actions to legs
        for i, act_idx in enumerate(LEG_ACTUATORS):
            self.data.ctrl[act_idx] = action[i]

        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)

        self.step_count += 1

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

        # Get leg positions
        leg_pos = np.array([self.data.qpos[7 + i] for i in LEG_ACTUATORS])
        target_pos = self._get_target_pose(self.gait_phase)

        # Indices within leg actions
        R_KNEE = 2
        L_KNEE = 6

        # Joint weights - knees highest
        joint_weights = np.array([0.5, 1.5, 5.0, 1.0, 0.5, 1.5, 5.0, 1.0])

        tracking_error = np.sum(joint_weights * np.square(leg_pos - target_pos))
        tracking_reward = 4.0 * np.exp(-1.0 * tracking_error)
        reward += tracking_reward

        # Knee tracking bonus
        right_knee_error = abs(leg_pos[R_KNEE] - target_pos[R_KNEE])
        left_knee_error = abs(leg_pos[L_KNEE] - target_pos[L_KNEE])

        if right_knee_error < np.radians(10):
            reward += 1.0
        if left_knee_error < np.radians(10):
            reward += 1.0
        if right_knee_error > np.radians(25):
            reward -= 1.0
        if left_knee_error > np.radians(25):
            reward -= 1.0

        # Forward velocity
        forward_vel = self.data.qvel[0]
        reward += 1.5 * np.clip(forward_vel, -0.5, 1.0)

        # Stability
        torso_z = self.data.qpos[2]
        torso_quat = self.data.qpos[3:7]

        if torso_z > 0.25:
            reward += 0.3
        else:
            reward -= 2.0

        reward += 0.5 * torso_quat[0]

        # Penalties
        lateral_vel = abs(self.data.qvel[1])
        reward -= 0.2 * lateral_vel

        angular_vel = abs(self.data.qvel[5])
        reward -= 0.1 * angular_vel

        # Only penalize leg control cost
        leg_ctrl = np.array([self.data.ctrl[i] for i in LEG_ACTUATORS])
        ctrl_cost = 0.0002 * np.sum(np.square(leg_ctrl))
        reward -= ctrl_cost

        return reward

    def _is_terminated(self):
        torso_z = self.data.qpos[2]
        if torso_z < 0.15:
            return True

        torso_quat = self.data.qpos[3:7]
        if abs(torso_quat[0]) < 0.5:
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


def train(headless=True, total_timesteps=3_000_000, resume=False, n_envs=8):
    print("=" * 50)
    print("WALKING TRAINING (UPPER BODY FROZEN)")
    print("=" * 50)
    print("Upper body: FROZEN (head + arms)")
    print("Training: LEGS ONLY (8 DOF)")
    print(f"Timesteps: {total_timesteps:,}")
    print(f"Parallel envs: {n_envs}")
    print(f"Mode: {'headless' if headless else 'with viewer'}")
    print(f"Resume: {resume}")
    print()

    if not headless:
        n_envs = 1  # Force 1 env for viewer mode
    render_mode = None if headless else "human"

    if headless and n_envs > 1:
        env = SubprocVecEnv([make_env() for _ in range(n_envs)])
    else:
        env = DummyVecEnv([make_env(render_mode)])

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
    print("TESTING WALKING (UPPER BODY FROZEN)")
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
    import argparse
    parser = argparse.ArgumentParser(description="Train walking with PPO")
    parser.add_argument("--headless", action="store_true", help="Train without viewer")
    parser.add_argument("--test", action="store_true", help="Test saved policy")
    parser.add_argument("--resume", action="store_true", help="Resume training")
    parser.add_argument("--num-envs", type=int, default=8, help="Number of parallel envs")
    parser.add_argument("--timesteps", type=int, default=3_000_000, help="Total timesteps")
    args = parser.parse_args()

    if args.test:
        test()
    else:
        train(
            headless=args.headless,
            total_timesteps=args.timesteps,
            resume=args.resume,
            n_envs=args.num_envs
        )


if __name__ == "__main__":
    main()
