"""Demo: Head oscillates ±30° continuously"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
from pxr import UsdLux
import numpy as np
import os

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
UsdLux.DistantLight.Define(world.stage, "/World/Light").CreateIntensityAttr(3000)

usd_path = os.path.join(os.path.dirname(__file__), "head_robot.usda")
add_reference_to_stage(usd_path=usd_path, prim_path="/World/Robot")
robot = world.scene.add(SingleArticulation(prim_path="/World/Robot", name="robot"))
world.reset()

print("▶ Head oscillating ±30° (Press Ctrl+C to exit)")

angles = [-30, 30]
idx = 0
frames = 0

try:
    while simulation_app.is_running():
        if frames == 0:
            robot._articulation_view.set_joint_position_targets(
                positions=np.array([np.radians(angles[idx])]),
                joint_indices=np.array([0])
            )
        world.step(render=True)
        frames = (frames + 1) % 120
        if frames == 0:
            idx = (idx + 1) % 2
except KeyboardInterrupt:
    pass

simulation_app.close()
