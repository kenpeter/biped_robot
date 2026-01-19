
import torch
import math
import os
import numpy as np
import gymnasium as gym

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import create_prim

f = open("/tmp/env_output.txt", "w")

class HumanoidEnv:
    def __init__(self, num_envs=4, headless=True):
        self.num_envs = num_envs
        self.device = "cuda:0"
        
        self._simulation_app = simulation_app
        
        self._world = World(stage_units_in_meters=1.0)
        self._world.scene.add_default_ground_plane()
        
        usd_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "humanoid_articulated.usda")
        f.write(f"Loading USD from: {usd_path}\n")
        f.flush()
        
        spacing = 2.0
        self._robots = []
        for env_idx in range(num_envs):
            env_pos = np.array([
                (env_idx % 2) * spacing - spacing/2,
                (env_idx // 2) * spacing - spacing/2,
                0.35
            ])
            prim_path = f"/World/Robot_{env_idx}"
            
            create_prim(
                prim_path=prim_path,
                prim_type="Xform",
                usd_path=usd_path,
                position=env_pos,
                orientation=np.array([0.7071, 0.7071, 0.0, 0.0])
            )
            
            robot = self._world.scene.add(
                Articulation(
                    prim_path=prim_path,
                    name=f"robot_{env_idx}"
                )
            )
            self._robots.append(robot)
            f.write(f"Created robot {env_idx} at {prim_path}\n")
            f.flush()
        
        self._world.reset()
        
    @property
    def num_actions(self):
        return self._robots[0].num_dof
    
    @property
    def observation_space(self):
        return gym.spaces.Box(low=-math.inf, high=math.inf, shape=(42,))
    
    @property
    def action_space(self):
        return gym.spaces.Box(low=-1.0, high=1.0, shape=(self.num_actions,))
    
    def step(self, actions):
        actions = torch.tensor(actions, device=self.device).reshape(self.num_envs, -1)
        
        for i, robot in enumerate(self._robots):
            targets = actions[i].cpu().numpy()
            robot.set_joint_positions(targets)
        
        self._world.step()
        
        obs = self._get_obs()
        reward = self._get_rewards(actions)
        done = self._get_dones()
        
        return obs, reward, done, {}
    
    def reset(self):
        self._world.reset()
        return self._get_obs()
    
    def _get_obs(self):
        obs_list = []
        for robot in self._robots:
            joint_pos = torch.from_numpy(robot.get_joint_positions()).flatten()
            joint_vel = torch.from_numpy(robot.get_joint_velocities()).flatten()
            root_pos, root_quat = robot.get_world_pose()
            root_pos = torch.from_numpy(root_pos).flatten()
            root_quat = torch.from_numpy(root_quat).flatten()
            obs = torch.cat([joint_pos, joint_vel, root_pos, root_quat])
            obs_list.append(obs)
        return torch.stack(obs_list).cpu().numpy()
    
    def _get_rewards(self, actions):
        rewards = []
        for i, robot in enumerate(self._robots):
            root_pos, _ = robot.get_world_pose()
            root_height = root_pos[2]
            r = 0.1 + (root_height > 0.25) * 1.0
            r -= torch.sum(actions[i]**2) * 0.01
            rewards.append(r)
        return torch.stack(rewards).cpu().numpy()
    
    def _get_dones(self):
        dones = []
        for robot in self._robots:
            root_pos, _ = robot.get_world_pose()
            root_height = root_pos[2]
            died = root_height < 0.15
            dones.append(died)
        return torch.stack(dones).cpu().numpy()
    
    def close(self):
        self._simulation_app.close()

if __name__ == "__main__":
    env = HumanoidEnv(num_envs=4, headless=True)
    
    f.write(f"Number of actions: {env.num_actions}\n")
    f.write("Testing environment...\n")
    f.flush()
    
    for i in range(10):
        obs = env.reset()
        f.write(f"Episode {i}: obs shape = {obs.shape}\n")
        f.flush()
    
    f.write("Environment working!\n")
    f.flush()
    f.close()
    
    env.close()
    print("Test complete!")
