
import gymnasium as gym
import torch
import os
import argparse
from datetime import datetime
from isaacsim import SimulationApp

# Initialize simulation app first
print("Initializing Isaac Sim...")
simulation_app = SimulationApp({"headless": False})

# Import the environment we just created
from humanoid_direct_env import HumanoidEnv, HumanoidEnvCfg

# Import RSL-RL (Standard RL library for Isaac Lab)
from rsl_rl.runners import OnPolicyRunner

def main():
    print("Setting up training...")
    # 1. Create Environment
    env_cfg = HumanoidEnvCfg()
    env = HumanoidEnv(cfg=env_cfg, render_mode="rgb_array")
    
    print(f"Created environment with {env.num_envs} robots!")

    # 2. Setup Log Directory
    log_dir = os.path.join("logs", datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
    os.makedirs(log_dir, exist_ok=True)

    # 3. Define PPO Agent
    # This configuration is standard for humanoid training
    runner = OnPolicyRunner(
        env=env,
        train_cfg={
            "seed": 42,
            "runner_class_name": "OnPolicyRunner",
            "policy": {
                "class_name": "ActorCritic",
                "init_noise_std": 1.0,
                "actor_hidden_dims": [256, 128, 64],
                "critic_hidden_dims": [256, 128, 64],
                "activation": "elu",
            },
            "algorithm": {
                "class_name": "PPO",
                "value_loss_coef": 1.0,
                "use_clipped_value_loss": True,
                "clip_param": 0.2,
                "entropy_coef": 0.01,
                "num_learning_epochs": 5,
                "num_mini_batches": 4,
                "learning_rate": 1.0e-3,
                "schedule": "adaptive",
                "gamma": 0.99,
                "lam": 0.95,
                "desired_kl": 0.01,
                "max_grad_norm": 1.0,
            },
            "num_steps_per_env": 24, # Steps per interaction
            "max_iterations": 1000,  # Total training iterations
            "save_interval": 50,
            "experiment_name": "humanoid_test",
            "run_name": "v1",
        },
        log_dir=log_dir,
        device=env.device
    )

    # 4. Start Training
    print("Starting training... check the Isaac Sim window!")
    runner.learn(num_learning_iterations=1000, init_at_random_ep_len=True)
    
    # 5. Close
    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
