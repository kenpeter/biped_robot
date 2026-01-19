"""
Test with VISIBLE geometry attached to joints
Creates simple cubes at each joint so you can see movement
"""

import os
import numpy as np
from isaacsim import SimulationApp

print("="*80)
print("VISIBLE JOINTS TEST")
print("="*80)

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.prims import XFormPrim
from pxr import Usd, UsdGeom, Gf, UsdPhysics
import omni.isaac.core.utils.stage as stage_utils

# Create world
print("\nCreating world...")
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Get stage
stage = stage_utils.get_current_stage()

# Create simple articulated robot with VISIBLE CUBES
print("Creating robot with visible geometry...")

# Root
robot_root = UsdGeom.Xform.Define(stage, "/World/Robot")
robot_root.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0.5))

# Base link with physics
base_prim = UsdGeom.Xform.Define(stage, "/World/Robot/base")
UsdPhysics.ArticulationRootAPI.Apply(base_prim.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(base_prim.GetPrim())

# Add VISIBLE cube to base
base_cube = UsdGeom.Cube.Define(stage, "/World/Robot/base/visual")
base_cube.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0))
base_cube.CreateSizeAttr(0.2)
base_cube.CreateDisplayColorAttr([(0.8, 0.2, 0.2)])  # Red

# Create 3 joints with visible cubes
joints_data = [
    ("joint1", (0.0, 0.0, 0.15), (0.2, 0.8, 0.2)),  # Green
    ("joint2", (0.0, 0.0, 0.15), (0.2, 0.2, 0.8)),  # Blue
    ("joint3", (0.0, 0.0, 0.15), (0.8, 0.8, 0.2)),  # Yellow
]

parent_path = "/World/Robot/base"
for i, (joint_name, offset, color) in enumerate(joints_data):
    # Create link
    link_path = f"/World/Robot/link{i+1}"
    link = UsdGeom.Xform.Define(stage, link_path)
    link.AddTranslateOp().Set(Gf.Vec3f(*offset))

    # Add physics
    UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())

    # Add VISIBLE cube
    cube = UsdGeom.Cube.Define(stage, f"{link_path}/visual")
    cube.CreateSizeAttr(0.15)
    cube.CreateDisplayColorAttr([color])

    # Create joint
    joint_path = f"/World/Robot/{joint_name}"
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([parent_path])
    joint.CreateBody1Rel().SetTargets([link_path])
    joint.CreateAxisAttr("Z")
    joint.CreateLowerLimitAttr(-3.14)
    joint.CreateUpperLimitAttr(3.14)

    # Add drive with NO stiffness
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateMaxForceAttr(50.0)
    drive.CreateDampingAttr(1.0)
    drive.CreateStiffnessAttr(0.0)  # No spring

    parent_path = link_path

print("Initializing...")
world.reset()

# Get articulation
from omni.isaac.core.articulations import Articulation
robot = world.scene.add(Articulation(prim_path="/World/Robot", name="robot"))
print(f"Robot has {robot.num_dof} DOF")

print("\n" + "="*80)
print("SPINNING JOINTS - YOU SHOULD SEE COLORED CUBES ROTATING")
print("Red (base) -> Green -> Blue -> Yellow")
print("="*80 + "\n")

# Spin the joints
step = 0
try:
    while simulation_app.is_running():
        # Create wave
        time = step / 60.0
        positions = np.array([
            np.radians(90) * np.sin(2 * np.pi * 0.2 * time),
            np.radians(90) * np.sin(2 * np.pi * 0.2 * time + np.pi/2),
            np.radians(90) * np.sin(2 * np.pi * 0.2 * time + np.pi),
        ])

        # Set positions
        robot.set_joint_positions(positions)
        world.step(render=True)

        if step % 30 == 0:
            print(f"Step {step:4d} | J1={np.degrees(positions[0]):+6.1f}° | J2={np.degrees(positions[1]):+6.1f}° | J3={np.degrees(positions[2]):+6.1f}°")

        step += 1

except KeyboardInterrupt:
    print("\nStopped")
finally:
    simulation_app.close()
    print("Closed.")
