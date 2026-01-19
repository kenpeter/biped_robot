"""
Isaac Sim Setup Script for Humanoid Robot
Creates simple stick-figure robot with spheres at joints

Run: ./run_isaac.sh setup_isaac_sim_robot.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import os
from pxr import Usd, UsdGeom, UsdPhysics, Gf
import math

PROJECT_ROOT = os.getcwd()
USD_OUTPUT = os.path.join(PROJECT_ROOT, "models/humanoid_articulated.usda")

# Simple robot: torso + 2 arms + 2 legs
# Each limb segment is just a sphere
JOINTS = [
    # (name, parent_path, offset_xyz, axis, color)
    ("head", "/Robot/torso", (0, 0, 0.15), "Z", (1, 1, 0)),  # Yellow head

    # Left arm
    ("l_shoulder", "/Robot/torso", (-0.15, 0, 0.10), "Y", (0, 1, 0)),  # Green
    ("l_elbow", "/Robot/l_shoulder", (-0.15, 0, 0), "Y", (0, 1, 0)),

    # Right arm
    ("r_shoulder", "/Robot/torso", (0.15, 0, 0.10), "Y", (0, 0, 1)),  # Blue
    ("r_elbow", "/Robot/r_shoulder", (0.15, 0, 0), "Y", (0, 0, 1)),

    # Left leg
    ("l_hip", "/Robot/torso", (-0.08, 0, -0.15), "Y", (0, 1, 0)),
    ("l_knee", "/Robot/l_hip", (0, 0, -0.20), "Y", (0, 1, 0)),
    ("l_ankle", "/Robot/l_knee", (0, 0, -0.20), "Y", (0, 1, 0)),

    # Right leg
    ("r_hip", "/Robot/torso", (0.08, 0, -0.15), "Y", (0, 0, 1)),
    ("r_knee", "/Robot/r_hip", (0, 0, -0.20), "Y", (0, 0, 1)),
    ("r_ankle", "/Robot/r_knee", (0, 0, -0.20), "Y", (0, 0, 1)),
]

print("Creating simple stick-figure robot...")
stage = Usd.Stage.CreateNew(USD_OUTPUT)

# Physics scene
scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr().Set(9.81)

# Robot root
robot = stage.DefinePrim("/Robot", "Xform")
stage.SetDefaultPrim(robot)

# Torso (base link)
torso = UsdGeom.Xform.Define(stage, "/Robot/torso")
torso_sphere = UsdGeom.Sphere.Define(stage, "/Robot/torso/visual")
torso_sphere.CreateRadiusAttr(0.12)
torso_sphere.CreateDisplayColorAttr([(1, 0, 0)])  # Red

UsdPhysics.ArticulationRootAPI.Apply(torso.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(torso.GetPrim())
UsdPhysics.MassAPI.Apply(torso.GetPrim()).CreateMassAttr(2.0)
UsdPhysics.CollisionAPI.Apply(torso_sphere.GetPrim())

# Create joints
for name, parent_path, offset, axis, color in JOINTS:
    link_path = f"{parent_path}/{name}"  # Create as child of parent, not sibling

    # Create link as child of parent
    link = UsdGeom.Xform.Define(stage, link_path)
    link.SetResetXformStack(True)  # CRITICAL: Reset xform stack for nested rigid bodies
    link.AddTranslateOp().Set(Gf.Vec3f(*offset))

    # Visual sphere
    sphere = UsdGeom.Sphere.Define(stage, f"{link_path}/visual")
    sphere.CreateRadiusAttr(0.05)
    sphere.CreateDisplayColorAttr([color])

    # Physics
    UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
    UsdPhysics.MassAPI.Apply(link.GetPrim()).CreateMassAttr(0.3)
    UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())

    # Joint
    joint = UsdPhysics.RevoluteJoint.Define(stage, f"{link_path}_joint")
    joint.CreateBody0Rel().SetTargets([parent_path])
    joint.CreateBody1Rel().SetTargets([link_path])
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(-math.pi)
    joint.CreateUpperLimitAttr(math.pi)

    # Drive
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateMaxForceAttr(20.0)
    drive.CreateDampingAttr(1.0)
    drive.CreateStiffnessAttr(0.0)

# Ground
ground = UsdGeom.Mesh.Define(stage, "/Ground")
ground.CreatePointsAttr([Gf.Vec3f(-10, -10, 0), Gf.Vec3f(10, -10, 0),
                         Gf.Vec3f(10, 10, 0), Gf.Vec3f(-10, 10, 0)])
ground.CreateFaceVertexCountsAttr([4])
ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

stage.Save()
print(f"✅ Created stick-figure robot: {USD_OUTPUT}")
print(f"  - {len(JOINTS)} joints (11 DOF)")
print(f"  - Colored spheres: Red torso, Yellow head, Green left, Blue right")
simulation_app.close()
