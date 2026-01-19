"""
Test simple robot with visible cubes
"""

import os
import numpy as np
from isaacsim import SimulationApp

print("="*60)
print("SIMPLE ROBOT TEST")
print("="*60)

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import create_prim

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

robot_usd_path = "/home/kenpeter/work/biped_robot/models/simple_robot.usda"
print(f"\nLoading: {robot_usd_path}")

create_prim(
    prim_path="/World/Robot",
    prim_type="Xform",
    usd_path=robot_usd_path,
    position=np.array([0.0, 0.0, 0.0]),
)

robot = world.scene.add(
    Articulation(
        prim_path="/World/Robot",
        name="robot"
    )
)

world.reset()

print(f"\nRobot loaded!")
print(f"  DOF: {robot.num_dof}")
print(f"  Joints: {robot.dof_names}")

print("\n" + "="*60)
print("MOVING SERVOS")
print("="*60)

try:
    for step in range(300):
        time_val = step / 30.0
        
        positions = np.zeros(robot.num_dof)
        for i in range(robot.num_dof):
            positions[i] = np.sin(time_val + i) * 0.3
        
        robot.set_joint_positions(positions)
        world.step(render=True)
        
        if step % 60 == 0:
            print(f"Step {step}: Moving all joints...")

except KeyboardInterrupt:
    print("\nStopped")
finally:
    simulation_app.close()
    print("Done!")
