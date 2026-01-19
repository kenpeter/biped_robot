
import torch
import math
import os
import numpy as np
import gymnasium as gym
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, Articulation
from isaaclab.utils import configclass
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sim.spawners.from_files import GroundPlaneCfg
import isaacsim.core.utils.prims as omni_prims_utils

@configclass
class HumanoidEnvCfg(DirectRLEnvCfg):
    decimation = 2
    episode_length_s = 20.0
    
    num_actions = 17
    action_scale = 1.0
    action_space_limits = (-1.0, 1.0)
    action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(17,))
    
    num_observations = 42
    observation_space = gym.spaces.Box(low=-math.inf, high=math.inf, shape=(42,))
    
    num_states = 0
    state_space = gym.spaces.Box(low=-math.inf, high=math.inf, shape=(0,))

    ground: AssetBaseCfg = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    
    sim: SimulationCfg = SimulationCfg(
        dt=1/120,
        render_interval=4,
        gravity=(0.0, 0.0, -9.81),
    )
    
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4,
        env_spacing=2.0,
        replicate_physics=True,
    )

class HumanoidEnv(DirectRLEnv):
    cfg: HumanoidEnvCfg

    def __init__(self, cfg: HumanoidEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self.action_scale = self.cfg.action_scale
        self.joint_pos_target = torch.zeros(
            (self.num_envs, self.num_actions), device=self.device
        )
        
    def _setup_scene(self):
        super()._setup_scene()
        
        usd_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "humanoid_articulated.usda")
        
        spacing = self.cfg.scene.env_spacing
        for env_idx in range(self.num_envs):
            env_pos = np.array([
                (env_idx % 2) * spacing,
                (env_idx // 2) * spacing,
                0.35
            ])
            prim_path = f"/World/envs/env_{env_idx}/Robot"
            
            omni_prims_utils.create_prim(
                prim_path=prim_path,
                prim_type="Xform",
                usd_path=usd_path,
                position=env_pos,
                orientation=np.array([0.7071, 0.7071, 0.0, 0.0])
            )
        
        self._robot = Articulation(
            prim_path="/World/envs/env_*/Robot",
            name="robot"
        )

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.actions = actions.clone()
        targets = self.actions * self.action_scale
        self._robot.set_joint_position_targets(targets)

    def _get_observations(self) -> dict:
        obs = torch.cat(
            (
                self._robot.data.joint_pos,
                self._robot.data.joint_vel,
                self._robot.data.root_pos_w - self.scene.env_origins,
                self._robot.data.root_quat_w,
            ),
            dim=-1,
        )
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        total_reward = torch.ones(self.num_envs, device=self.device) * 0.1
        root_height = self._robot.data.root_pos_w[:, 2]
        total_reward += torch.where(root_height > 0.25, 1.0, 0.0)
        total_reward -= torch.sum(torch.square(self.actions), dim=1) * 0.01
        return total_reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        root_height = self._robot.data.root_pos_w[:, 2]
        died = root_height < 0.15
        time_out = self.episode_length_buf >= self.max_episode_length
        return died, time_out
