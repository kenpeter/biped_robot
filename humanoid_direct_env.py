
import torch
import math
import os
import gymnasium as gym
import omni.isaac.core.utils.prims as prim_utils
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.utils import configclass
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, UsdFileCfg # Re-added UsdFileCfg

@configclass
class HumanoidEnvCfg(DirectRLEnvCfg):
    # World Settings
    decimation = 2
    episode_length_s = 20.0  # Reset every 20 seconds
    
    # Action Space (17 joints)
    num_actions = 17
    action_scale = 1.0
    action_space_limits = (-1.0, 1.0)
    action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(17,))
    
    # Observation Space
    num_observations = 42
    observation_space = gym.spaces.Box(low=-math.inf, high=math.inf, shape=(42,))
    
    # State Space
    num_states = 0
    state_space = gym.spaces.Box(low=-math.inf, high=math.inf, shape=(0,))

    # Ground plane asset
    ground: AssetBaseCfg = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    
    # Simulation Settings
    sim: SimulationCfg = SimulationCfg(
        dt=1/120,  # 120Hz physics
        render_interval=4,  # Render every 4th step (30fps)
        gravity=(0.0, 0.0, -9.81),
    )
    
    # Scene Setup
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4096,  # SPAWN 4096 ROBOTS!
        env_spacing=2.0,
        replicate_physics=True,
    )
    
    # Robot Articulation (single source for spawning and control properties)
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_0/Humanoid", # Explicit path for the first environment's robot
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.35), # Initial position
            rot=(0.7071, 0.7071, 0.0, 0.0) # Initial orientation (w, x, y, z)
        ),
        spawn=UsdFileCfg(
            usd_path=os.path.join(os.path.dirname(os.path.abspath(__file__)), "models/humanoid_articulated.usda"),
        ),
        actuators={
            "body": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                stiffness=100.0,
                damping=10.0,
            ),
        }
    )

class HumanoidEnv(DirectRLEnv):
    cfg: HumanoidEnvCfg

    def __init__(self, cfg: HumanoidEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        
        # Action scaling
        self.action_scale = self.cfg.action_scale
        
        # Joint targets
        self.joint_pos_target = torch.zeros(
            (self.num_envs, self.num_actions), device=self.device
        )
        
    def _setup_scene(self):
        """Load the robot USD into the scene"""
        # Initialize the scene (clones environments and spawns assets configured in cfg)
        super()._setup_scene()
        
        # Get robot articulation view (registered under the name "robot" from ArticulationCfg)
        self._robot = self.scene.articulations["robot"]

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        """Apply actions before physics step"""
        self.actions = actions.clone()
        
        # Compute targets: Current Position + Action * Scale
        # (Simple position control)
        targets = self.actions * self.action_scale
        
        # Apply to robot
        self._robot.set_joint_position_targets(targets)

    def _get_observations(self) -> dict:
        """Return observations"""
        obs = torch.cat(
            (
                self._robot.data.joint_pos,
                self._robot.data.joint_vel,
                self._robot.data.root_pos_w - self.scene.env_origins, # Relative position
                self._robot.data.root_quat_w,
            ),
            dim=-1,
        )
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        """Compute rewards"""
        # Reward 1: Stay alive (upright)
        total_reward = torch.ones(self.num_envs, device=self.device) * 0.1
        
        # Reward 2: Height target (stand up)
        root_height = self._robot.data.root_pos_w[:, 2]
        total_reward += torch.where(root_height > 0.25, 1.0, 0.0)
        
        # Penalty: High energy usage
        total_reward -= torch.sum(torch.square(self.actions), dim=1) * 0.01
        
        return total_reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Check termination conditions"""
        # Die if fallen (head too low)
        root_height = self._robot.data.root_pos_w[:, 2]
        died = root_height < 0.15
        
        # Time out
        time_out = self.episode_length_buf >= self.max_episode_length
        
        return died, time_out
