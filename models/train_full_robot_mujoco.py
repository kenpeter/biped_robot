#!/usr/bin/env python3
"""Train full humanoid robot using MuJoCo + Stable Baselines3 PPO.

All servos (17 DOF):
  - Head: 0 (left/right)
  - Right arm: 1 (shoulder), 2 (upper arm), 3 (forearm)
  - Left arm: 12 (shoulder), 13 (upper arm), 14 (forearm)
  - Right leg: 4 (hip roll), 5 (hip pitch), 6 (knee), 7 (ankle roll), 8 (ankle pitch)
  - Left leg: 15 (hip roll), 16 (hip pitch), 17 (knee), 18 (ankle roll), 19 (ankle pitch)

Usage:
  python3 train_full_robot_mujoco.py              # train with PPO (shows UI, 1M steps)
  python3 train_full_robot_mujoco.py --headless   # train without UI
  python3 train_full_robot_mujoco.py --num-envs 8 # train with 8 parallel robots (faster!)
  python3 train_full_robot_mujoco.py --timesteps 5000000  # train for 5M steps (default: 1M)
  python3 train_full_robot_mujoco.py --resume     # resume training (shows UI)
  python3 train_full_robot_mujoco.py --resume --headless --num-envs 8 --timesteps 10000000
  python3 train_full_robot_mujoco.py --test       # test saved policy
"""

import numpy as np
import sys
import time
import os

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print("MuJoCo not installed. Install with:")
    print("  pip install mujoco")
    exit(1)

try:
    import gymnasium as gym
    from gymnasium import spaces
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
except ImportError:
    print("Stable Baselines3 not installed. Install with:")
    print("  pip install stable-baselines3 gymnasium")
    exit(1)

MODEL_PATH = '/home/kenpeter/work/biped_robot/models/full_robot.xml'
POLICY_PATH = '/home/kenpeter/work/biped_robot/models/full_robot_ppo.zip'
LOG_PATH = '/home/kenpeter/work/biped_robot/models/ppo_logs'


class HumanoidEnv(gym.Env):
    """Gymnasium environment for full humanoid robot."""

    metadata = {'render_modes': ['human', 'rgb_array'], 'render_fps': 50}

    def __init__(self, render_mode=None):
        super().__init__()

        self.mj_model = mujoco.MjModel.from_xml_path(MODEL_PATH)
        self.mj_data = mujoco.MjData(self.mj_model)
        self.render_mode = render_mode
        self.viewer = None

        # Timing - realistic for slow servos
        self.dt = self.mj_model.opt.timestep
        self.frame_skip = 10  # More simulation steps per action for stability
        self.max_episode_steps = 2000  # Longer episodes for slow movements
        self.step_count = 0

        # Gait tracking for foot alternation rewards
        self.last_foot_contact = [0, 0]  # [right, left]
        self.gait_phase = 0.0

        # Track airtime for feet (for airtime reward)
        self.right_foot_airtime = 0.0
        self.left_foot_airtime = 0.0
        self.right_foot_last_contact_time = 0.0
        self.left_foot_last_contact_time = 0.0

        # CRITICAL: Track which foot is stance leg (prevents hopping exploit)
        self.stance_leg = None  # 'right', 'left', or None
        self.steps_on_same_leg = 0  # How long on same stance leg
        self.total_foot_switches = 0  # Total times stance leg switched
        self.last_switch_step = 0

        # Action space: 17 actuators, range [-1, 1]
        self.n_actuators = 17
        self.action_space = spaces.Box(
            low=-1.0, high=1.0,
            shape=(self.n_actuators,),
            dtype=np.float32
        )

        # Observation space
        obs_dim = self._get_obs_dim()
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf,
            shape=(obs_dim,),
            dtype=np.float32
        )

        # Initial pose
        self._setup_init_pose()

    def _get_obs_dim(self):
        """Get observation dimension."""
        # Joint pos (17) + joint vel (17) + torso quat (4) + torso vel (6) + foot contacts (2) + gait phase (2: sin, cos)
        return 17 + 17 + 4 + 6 + 2 + 2

    def _setup_init_pose(self):
        """Setup initial standing pose."""
        self.init_qpos = np.zeros(self.mj_model.nq)
        self.init_qpos[2] = 0.28  # torso height (scaled for 42cm robot)
        self.init_qpos[3] = 1.0   # quaternion w

        # Bent knees for stability
        # Joint order: head(1), r_arm(3), l_arm(3), r_leg(5), l_leg(5) = 17 total
        # r_hip_pitch=8, r_knee=9, l_hip_pitch=13, l_knee=14 (relative to qpos[7])
        self.init_qpos[7 + 8] = np.radians(15)   # right hip pitch
        self.init_qpos[7 + 9] = np.radians(30)   # right knee
        self.init_qpos[7 + 13] = np.radians(15)  # left hip pitch
        self.init_qpos[7 + 14] = np.radians(30)  # left knee

    def reset(self, seed=None, options=None):
        """Reset environment."""
        super().reset(seed=seed)

        mujoco.mj_resetData(self.mj_model, self.mj_data)
        self.mj_data.qpos[:] = self.init_qpos.copy()

        # Small noise on joints only
        noise = np.random.uniform(-0.02, 0.02, self.mj_model.nq)
        noise[0:7] = 0  # No noise on freejoint
        self.mj_data.qpos[:] += noise
        self.mj_data.qvel[:] = 0

        mujoco.mj_forward(self.mj_model, self.mj_data)

        # Settle
        for _ in range(30):
            mujoco.mj_step(self.mj_model, self.mj_data)

        self.step_count = 0
        self.gait_phase = 0.0
        self.last_foot_contact = [0, 0]

        # Reset airtime trackers
        self.right_foot_airtime = 0.0
        self.left_foot_airtime = 0.0

        # Reset foot switching trackers
        self.stance_leg = None
        self.steps_on_same_leg = 0
        self.total_foot_switches = 0
        self.last_switch_step = 0

        return self._get_obs(), {}

    def _get_obs(self):
        """Get observation vector."""
        # Joint positions (skip freejoint) - 17 joints
        joint_pos = self.mj_data.qpos[7:24].copy()

        # Joint velocities (skip freejoint) - 17 joints
        joint_vel = self.mj_data.qvel[6:23].copy()
        joint_vel = np.clip(joint_vel, -10, 10)  # Clip velocities

        # Torso orientation
        torso_quat = self.mj_data.qpos[3:7].copy()

        # Torso velocity
        torso_vel = self.mj_data.qvel[0:6].copy()
        torso_vel = np.clip(torso_vel, -10, 10)

        # Foot contacts (simple height-based)
        right_foot_z = self.mj_data.xpos[self._get_body_id("right_foot")][2]
        left_foot_z = self.mj_data.xpos[self._get_body_id("left_foot")][2]
        foot_contacts = np.array([
            1.0 if right_foot_z < 0.05 else 0.0,
            1.0 if left_foot_z < 0.05 else 0.0
        ])

        # Gait phase clock (helps robot learn periodic walking pattern)
        gait_clock = np.array([np.sin(self.gait_phase), np.cos(self.gait_phase)])

        obs = np.concatenate([joint_pos, joint_vel, torso_quat, torso_vel, foot_contacts, gait_clock])
        return obs.astype(np.float32)

    def _get_body_id(self, name):
        """Get body ID by name."""
        return mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, name)

    def step(self, action):
        """Take a step."""
        action = np.clip(action, -1, 1)
        self.mj_data.ctrl[:] = action

        # Step simulation
        for _ in range(self.frame_skip):
            mujoco.mj_step(self.mj_model, self.mj_data)

        self.step_count += 1

        # Update gait phase (2 Hz walking cycle)
        self.gait_phase += 2.0 * np.pi * self.dt * self.frame_skip * 2.0
        self.gait_phase = self.gait_phase % (2.0 * np.pi)

        obs = self._get_obs()
        reward = self._get_reward(action)
        terminated = self._is_terminated()
        truncated = self.step_count >= self.max_episode_steps

        # Render if in human mode
        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, truncated, {}

    def _get_reward(self, action):
        """Calculate reward - v7 ANTI-HOP (Human-like walking).

        Built on v6 MINIMAL but tuned to eliminate slight hopping:
        - Increased foot switch reward (5.0 → 8.0) - make walking MORE attractive
        - Reduced forward velocity (1.0 → 0.5) - less rushing, smoother gait
        - Added ground contact bonus (+0.3) - reward foot on ground (anti-hop)

        Based on the principle: "Reward what you want, ignore what you don't."
        """
        reward = 0.0

        torso_z = self.mj_data.qpos[2]
        torso_quat = self.mj_data.qpos[3:7]
        forward_vel = self.mj_data.qvel[0]

        # Get foot contacts
        right_foot_z = self.mj_data.xpos[self._get_body_id("right_foot")][2]
        left_foot_z = self.mj_data.xpos[self._get_body_id("left_foot")][2]
        right_contact = right_foot_z < 0.05
        left_contact = left_foot_z < 0.05

        # Track stance leg for switching detection
        single_stance = (right_contact and not left_contact) or (left_contact and not right_contact)
        if single_stance:
            current_stance = 'right' if right_contact else 'left'
            if self.stance_leg is None:
                self.stance_leg = current_stance
                self.steps_on_same_leg = 0
            elif self.stance_leg != current_stance:
                # FOOT SWITCHED! Big reward!
                reward += 8.0  # v7: Increased from 5.0 to make walking MORE attractive than hopping
                self.stance_leg = current_stance
                self.total_foot_switches += 1
                self.steps_on_same_leg = 0
            else:
                self.steps_on_same_leg += 1

        # v7: Ground contact bonus (anti-hop) - reward having foot on ground
        if right_contact or left_contact:
            reward += 0.3  # Small bonus for foot on ground

        # ========== ONLY 3 SIMPLE REWARDS ==========

        # 1. Stay upright (weight: 2.0) - Most important!
        upright = torso_quat[0] ** 2  # 1.0 when perfectly upright
        reward += 2.0 * upright

        # 2. Move forward (weight: 0.5) - Secondary goal (v7: reduced from 1.0 to discourage rushing/hopping)
        reward += 0.5 * np.clip(forward_vel, 0, 1.0)

        # 3. Maintain height (weight: 0.5) - Bonus for staying up
        height_bonus = 1.0 - abs(torso_z - 0.28)  # Max 1.0 at perfect height (42cm robot)
        reward += 0.5 * np.clip(height_bonus, 0, 1.0)

        # v7 improvements over v6:
        # - Foot switch +8.0 (was +5.0) = walking is MORE rewarding than hopping
        # - Forward vel +0.5 (was +1.0) = less rushing, smoother gait
        # - Ground contact +0.3 = direct anti-hop incentive
        # Result: Human-like walking instead of hop-walking!

        return reward

    def _is_terminated(self):
        """Check if episode should end."""
        torso_z = self.mj_data.qpos[2]
        torso_quat = self.mj_data.qpos[3:7]

        # Fell down
        if torso_z < 0.15:  # Fallen threshold (scaled for 42cm robot)
            return True

        # Too tilted
        if abs(torso_quat[0]) < 0.5:
            return True

        return False

    def render(self):
        """Render the environment."""
        if self.render_mode == "human":
            if self.viewer is None:
                self.viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)
                self.viewer.cam.distance = 2.5
                self.viewer.cam.elevation = -15
                self.viewer.cam.azimuth = 90
            self.viewer.sync()

    def close(self):
        """Close environment."""
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None


class RenderCallback(BaseCallback):
    """Callback to render during training."""

    def __init__(self, render_freq=1000, verbose=0):
        super().__init__(verbose)
        self.render_freq = render_freq

    def _on_step(self):
        if self.n_calls % self.render_freq == 0:
            env = self.training_env.envs[0]
            if hasattr(env, 'render'):
                env.render()
        return True


def make_env(render=False):
    """Create environment."""
    def _init():
        env = HumanoidEnv(render_mode="human" if render else None)
        return env
    return _init


def train(resume=False, total_timesteps=1_000_000, headless=False, num_envs=1):
    """Train with PPO."""
    print("=" * 60)
    print("FULL HUMANOID TRAINING (PPO)")
    print("=" * 60)
    print(f"\nTimesteps: {total_timesteps:,}")
    print(f"Resume: {resume}")
    print(f"Headless: {headless}")
    print(f"Parallel robots: {num_envs}")
    print()

    # Create environment(s)
    if num_envs == 1:
        # Single environment with optional rendering
        print("[ENV] Creating single environment...")
        env = DummyVecEnv([make_env(render=not headless)])
    else:
        # Multiple parallel environments (no rendering when parallel)
        print(f"[ENV] Creating {num_envs} parallel environments...")
        env = SubprocVecEnv([make_env(render=False) for _ in range(num_envs)])

    # Create or load model
    if resume and os.path.exists(POLICY_PATH):
        print(f"[LOAD] Loading {POLICY_PATH}")
        model = PPO.load(POLICY_PATH, env=env)
        # Update learning rate when resuming (helps adapt to new reward structure)
        model.learning_rate = 3e-4
        print(f"[INFO] Adjusted learning rate to {model.learning_rate} for continued training")
    else:
        print("[NEW] Creating new PPO model...")
        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=3e-4,  # Increased from 1e-4 for faster initial learning
            n_steps=4096,         # More steps for better statistics
            batch_size=128,       # Larger batch for stability
            n_epochs=15,         # More epochs per update
            gamma=0.995,          # Longer horizon for slow movements
            gae_lambda=0.95,
            clip_range=0.2,      # Increased from 0.1 for more aggressive updates
            ent_coef=0.01,       # Increased exploration to discover foot lifting
            verbose=1,
            tensorboard_log=LOG_PATH
        )

    # Callbacks
    checkpoint_cb = CheckpointCallback(
        save_freq=10000,
        save_path=LOG_PATH,
        name_prefix="humanoid"
    )

    # Train
    print("\n[TRAIN] Starting training...")
    print("Press Ctrl+C to stop and save\n")

    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=[checkpoint_cb],
            progress_bar=True
        )
    except KeyboardInterrupt:
        print("\n[INTERRUPTED]")

    # Save
    model.save(POLICY_PATH)
    print(f"\n[SAVED] {POLICY_PATH}")

    env.close()
    print("[DONE]")


def test():
    """Test trained policy."""
    print("=" * 60)
    print("TESTING HUMANOID POLICY")
    print("=" * 60)

    if not os.path.exists(POLICY_PATH):
        print(f"[ERROR] No policy at {POLICY_PATH}")
        print("Train first: python3 train_full_robot_mujoco.py")
        return

    # Create environment with rendering
    env = HumanoidEnv(render_mode="human")

    # Load model
    print(f"[LOAD] {POLICY_PATH}")
    model = PPO.load(POLICY_PATH)

    print("\n[RUN] Press Ctrl+C to stop\n")

    try:
        for ep in range(100):
            obs, _ = env.reset()
            total_reward = 0
            steps = 0
            done = False

            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, _ = env.step(action)
                done = terminated or truncated
                total_reward += reward
                steps += 1

                env.render()
                time.sleep(0.02)

            print(f"Episode {ep+1:3d}: Steps = {steps:4d}, Reward = {total_reward:8.2f}")

    except KeyboardInterrupt:
        print("\n[STOPPED]")

    env.close()


def main():
    headless = "--headless" in sys.argv

    # Parse --num-envs argument (default: 1)
    num_envs = 1
    for i, arg in enumerate(sys.argv):
        if arg == "--num-envs" and i + 1 < len(sys.argv):
            try:
                num_envs = int(sys.argv[i + 1])
                if num_envs < 1:
                    print("Error: --num-envs must be >= 1")
                    sys.exit(1)
            except ValueError:
                print(f"Error: Invalid --num-envs value: {sys.argv[i + 1]}")
                sys.exit(1)
            break

    # Parse --timesteps argument (default: 1,000,000)
    timesteps = 1_000_000
    for i, arg in enumerate(sys.argv):
        if arg == "--timesteps" and i + 1 < len(sys.argv):
            try:
                timesteps = int(sys.argv[i + 1])
                if timesteps < 1000:
                    print("Error: --timesteps must be >= 1000")
                    sys.exit(1)
            except ValueError:
                print(f"Error: Invalid --timesteps value: {sys.argv[i + 1]}")
                sys.exit(1)
            break

    if "--test" in sys.argv:
        test()
    elif "--resume" in sys.argv:
        train(resume=True, total_timesteps=timesteps, headless=headless, num_envs=num_envs)
    else:
        train(resume=False, total_timesteps=timesteps, headless=headless, num_envs=num_envs)


if __name__ == "__main__":
    main()
