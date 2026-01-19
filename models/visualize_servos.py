"""
Visualize robot servo movement with simple cubes
Shows how each servo connects and rotates
"""

import os
import numpy as np
from isaacsim import SimulationApp

print("="*80)
print("ROBOT SERVO VISUALIZATION")
print("="*80)

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import create_prim

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

robot_usd_path = "/home/kenpeter/work/biped_robot/models/humanoid_articulated.usda"
print(f"\nLoading robot from: {robot_usd_path}")

prim_path = "/World/Robot"
create_prim(
    prim_path=prim_path,
    prim_type="Xform",
    usd_path=robot_usd_path,
    position=np.array([0.0, 0.0, 0.0]),
    orientation=np.array([0.7071, 0.7071, 0.0, 0.0])
)

robot = world.scene.add(
    Articulation(
        prim_path=prim_path,
        name="humanoid"
    )
)

world.reset()

print(f"\nRobot loaded!")
print(f"  DOF: {robot.num_dof}")
print(f"  Joint names: {robot.dof_names}")

print("\n" + "="*80)
print("MOVING EACH SERVO INDIVIDUALLY")
print("="*80)

joint_names = robot.dof_names

try:
    step = 0
    current_servo = 0
    
    while simulation_app.is_running():
        time_val = step / 30.0
        
        if step % 90 == 0:
            joint_name = joint_names[current_servo]
            print(f"\n{'='*60}")
            print(f"Moving servo {current_servo}: {joint_name}")
            print(f"{'='*60}")
        
        angle = np.sin(time_val) * 0.5
        positions = np.zeros(robot.num_dof)
        positions[current_servo] = angle
        robot.set_joint_positions(positions)
        
        world.step(render=True)
        
        if step % 15 == 0:
            print(f"  Angle: {np.degrees(angle):+6.1f}°")
        
        step += 1
        
        if step > 90:
            step = 0
            current_servo += 1
            if current_servo >= robot.num_dof:
                current_servo = 0

except KeyboardInterrupt:
    print("\n\nStopped by user")
finally:
    simulation_app.close()
    print("\nVisualization complete!")
