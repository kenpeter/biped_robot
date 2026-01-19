
import os
import sys
import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import create_prim

f = open("/tmp/robot_debug.txt", "w")

f.write("Creating world...\n")
f.flush()

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

usd_path = "/home/kenpeter/work/biped_robot/models/humanoid_articulated.usda"
f.write(f"Loading robot from: {usd_path}\n")
f.flush()

prim_path = "/World/Robot"
create_prim(
    prim_path=prim_path,
    prim_type="Xform",
    usd_path=usd_path,
    position=np.array([0.0, 0.0, 0.35]),
    orientation=np.array([0.7071, 0.7071, 0.0, 0.0])
)

robot = world.scene.add(
    Articulation(
        prim_path=prim_path,
        name="humanoid"
    )
)

f.write(f"Robot type: {type(robot)}\n")
f.write(f"Robot class bases: {type(robot).__bases__}\n")
f.flush()

world.reset()

f.write(f"Robot loaded!\n")
f.write(f"  DOF: {robot.num_dof}\n")
f.write(f"  Joint names: {robot.dof_names}\n")
f.flush()

f.write("\nChecking data attribute...\n")
f.flush()

if hasattr(robot, 'data'):
    f.write(f"  data type: {type(robot.data)}\n")
    f.write(f"  data methods: {[x for x in dir(robot.data) if not x.startswith('_')][:20]}\n")
else:
    f.write("  No 'data' attribute!\n")
    f.write(f"  Available methods: {[x for x in dir(robot) if not x.startswith('_')][:20]}\n")
    f.write(f"  Full dir: {[x for x in dir(robot) if not x.startswith('_')]}\n")
    
    # Check if it's a SingleArticulation
    f.write(f"\n  Checking class hierarchy...\n")
    for cls in type(robot).__mro__:
        f.write(f"    {cls}\n")

f.flush()
f.close()

simulation_app.close()
print("Test complete!")
