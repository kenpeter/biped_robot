#!/usr/bin/env python3
"""
Minimal Isaac Lab Test
Test if robot can be loaded in Isaac Lab environment
"""

import os
import sys
from isaacsim import SimulationApp

print("=" * 60)
print("MINIMAL ISAAC LAB TEST")
print("=" * 60)

# Launch Isaac Sim (headless for faster startup)
simulation_app = SimulationApp({"headless": True})

try:
    import gymnasium as gym
    from humanoid_direct_env import HumanoidEnvCfg
    from isaaclab.envs import DirectRLEnv
    from isaaclab.sim.spawners.from_files import UsdFileCfg
    from isaaclab.assets import ArticulationCfg
    
    print("✅ Isaac Lab imports successful")
    
    # Test environment creation only
    print("Testing environment configuration...")
    env_cfg = HumanoidEnvCfg()
    env_cfg.num_envs = 1  # Just test one environment
    env_cfg.episode_length_s = 5.0  # Short test
    
    print(f"✅ Environment config created:")
    print(f"   - Robot name: {env_cfg.robot.name}")
    print(f"   - Robot path: {env_cfg.robot.prim_path}")
    print(f"   - USD path: {env_cfg.robot.spawn.usd_path}")
    print(f"   - Num environments: {env_cfg.num_envs}")
    
    # Try to create the environment (without full simulation)
    print("Testing asset registration...")
    try:
        # Just test the asset loading part
        from isaaclab.sim import SimulationContext
        sim_context = SimulationContext(physics_dt=1/120, rendering_dt=1/60)
        
        # Create scene
        sim_context.reset()
        
        # Load robot asset using Isaac Lab's system
        robot_cfg = env_cfg.robot
        robot_prim_cfg = UsdFileCfg(
            usd_path=robot_cfg.spawn.usd_path,
            prim_path=robot_cfg.prim_path
        )
        
        # This should trigger asset registration
        from isaaclab.utils.assets import register_asset
        register_asset(robot_cfg.name, robot_prim_cfg)
        
        print("✅ Asset registration successful!")
        print("✅ Isaac Lab test PASSED - robot should be visible!")
        
    except Exception as e:
        print(f"❌ Asset registration failed: {e}")
        import traceback
        traceback.print_exc()
        
    simulation_app.close()
    print("Test complete!")
    
except Exception as e:
    print(f"❌ Import/configuration failed: {e}")
    import traceback
    traceback.print_exc()
    simulation_app.close()