#!/usr/bin/env python3
"""
Test basic joint control with a simple articulation
"""

import os
import numpy as np
from isaacsim import SimulationApp

print("=" * 60)
print("BASIC JOINT CONTROL TEST")
print("=" * 60)

simulation_app = SimulationApp({"headless": False})

from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
import omni.usd

# Get stage
stage = omni.usd.get_context().get_stage()

# Create physics scene
scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)

# Create ground
ground = UsdGeom.Cube.Define(stage, "/World/Ground")
ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
ground.AddScaleOp().Set(Gf.Vec3f(5, 5, 0.1))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath("/World/Ground"))

print("Created simple stage with ground")

# Now test articulation with humanoid robot
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import create_prim

PROJECT_ROOT = "/home/kenpeter/work/biped_robot"
ROBOT_USD = os.path.join(PROJECT_ROOT, "models/humanoid_articulated.usda")

print(f"\nLoading robot from: {ROBOT_USD}")
create_prim(
    prim_path="/World/Humanoid",
    prim_type="Xform",
    usd_path=ROBOT_USD,
    position=np.array([0.0, 0.0, 0.15]),
    orientation=np.array([0.7071, 0.7071, 0.0, 0.0])
)

# Need to wait for physics to initialize
for _ in range(20):
    simulation_app.update()

print("Looking for articulation...")
robot = ArticulationView("/World/Humanoid")
print(f"Found: {robot.num_dof} DOFs")

if robot.num_dof > 0:
    print("\nInitial position:", robot.get_joint_positions().flatten()[:3])
    
    print("\nSetting joint positions...")
    for step in range(100):
        target = np.zeros((1, robot.num_dof))
        target[0, 0] = 0.5  # Head
        target[0, 1] = 0.3  # Left shoulder
        target[0, 4] = -0.3  # Right shoulder
        
        robot.set_joint_positions(target)
        simulation_app.update()
        
        if step % 20 == 0:
            pos = robot.get_joint_positions()
            print(f"  Step {step}: positions = {pos.flatten()[:5]}")
else:
    print("ERROR: No DOFs found!")

print("\nTest complete. Window stays open.")
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
