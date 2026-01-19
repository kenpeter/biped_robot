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

from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf, PhysxSchema
import omni.usd

# Get stage
stage = omni.usd.get_context().get_stage()

# Create ground plane
UsdGeom.Cube.Define(stage, "/World/Ground").AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
UsdGeom.Cube.Define(stage, "/World/Ground/Physics").AddScaleOp().Set(Gf.Vec3f(5, 5, 0.1))

# Create a simple two-link arm
# Root link
root = UsdGeom.Xform.Define(stage, "/World/Arm")
root.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.5))
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/World/Arm"))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath("/World/Arm"))
UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath("/World/Arm")).CreateMassAttr(1.0)

# Physics scene
scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)

# Second link (child)
link2 = UsdGeom.Xform.Define(stage, "/World/Arm/Link2")
link2.AddTranslateOp().Set(Gf.Vec3d(0, 0.5, 0))
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/World/Arm/Link2"))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath("/World/Arm/Link2"))
UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath("/World/Arm/Link2")).CreateMassAttr(0.5)

# Joint between root and link2
joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Arm/Joint")
joint.CreateBody0Rel().SetTargets(["/World/Arm"])
joint.CreateBody1Rel().SetTargets(["/World/Arm/Link2"])
joint.CreateAxisAttr("Y")
joint.CreateLowerLimitAttr(-1.0)
joint.CreateUpperLimitAttr(1.0)

# Add drive
joint.CreateTypeAttr("angular")
drive = PhysxSchema.PhysxPhysicsDriveAPI.Apply(stage.GetPrimAtPath("/World/Arm/Joint"))
drive.CreateMaxForceAttr(100.0)
drive.CreateDampingAttr(10.0)
drive.CreateStiffnessAttr(100.0)

print("Created simple arm articulation")
print("Stage contents:")
for prim in stage.Traverse():
    print(f"  {prim.GetPath()}")

# Now test joint control
from omni.isaac.core.articulations import ArticulationView

print("\nLooking for articulation...")
robot = ArticulationView("/World/Arm")
print(f"Found: {robot.num_dof} DOFs")

print("\nInitial position:", robot.get_joint_positions())

# Try to move the joint
print("\nMoving joint to 0.5 radians...")
for step in range(50):
    target = np.array([[0.5]])  # Target position for the joint
    robot.set_joint_positions(target)
    
    # Step simulation
    simulation_app.update()
    
    if step % 10 == 0:
        pos = robot.get_joint_positions()
        print(f"  Step {step}: position = {pos[0,0]:.4f}")

print("\nTest complete. Window stays open.")
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
