"""
Isaac Sim Setup Script for Humanoid Robot
Converts GLB to articulated USD with physics and joints for RL training

Run from Isaac Sim:
  /home/kenpeter/work/IsaacSim/python.sh setup_isaac_sim_robot.py
"""

from isaacsim import SimulationApp

# Launch Isaac Sim in headless mode
simulation_app = SimulationApp({"headless": True})

import os
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, Vt
import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import create_prim, define_prim
import omni

# Paths
PROJECT_ROOT = os.getcwd()
GLB_PATH = os.path.join(PROJECT_ROOT, "models/humanoid.glb")
USD_OUTPUT = os.path.join(PROJECT_ROOT, "models/humanoid_articulated.usda")

# Joint configuration
# Format: (joint_name, parent_link, child_link, axis, lower_limit, upper_limit, offset_xyz)
# NOTE: All servos have 360° rotation capability
JOINT_CONFIG = [
    # HEAD
    ("head_joint", "Torso_Main", "Head_Servo", "Z", -180, 180, (0, 0, 0.08)),

    # LEFT ARM (y+ is left)
    ("l_shoulder_pitch", "Torso_Main", "L_Shoulder1", "Y", -180, 180, (0, 0.06, 0.05)),
    ("l_shoulder_roll", "L_Shoulder1", "L_Shoulder2", "X", -180, 180, (0, 0.04, 0)),
    ("l_forearm_roll", "L_Shoulder2", "L_Elbow", "X", -180, 180, (0, 0, -0.08)),

    # RIGHT ARM (y- is right)
    ("r_shoulder_pitch", "Torso_Main", "R_Shoulder1", "Y", -180, 180, (0, -0.06, 0.05)),
    ("r_shoulder_roll", "R_Shoulder1", "R_Shoulder2", "X", -180, 180, (0, -0.04, 0)),
    ("r_forearm_roll", "R_Shoulder2", "R_Elbow", "X", -180, 180, (0, 0, -0.08)),

    # LEFT LEG
    ("l_hip_roll", "Torso_Bot_Plate", "L_Hip1", "X", -180, 180, (0, 0.04, -0.05)),
    ("l_hip_pitch", "L_Hip1", "L_Hip2", "Y", -180, 180, (0, 0, -0.04)),
    ("l_knee_pitch", "L_Hip2", "L_Knee", "Y", -180, 180, (0, 0, -0.10)),
    ("l_ankle_pitch", "L_Knee", "L_Ankle", "Y", -180, 180, (0, 0, -0.10)),
    ("l_foot_roll", "L_Ankle", "L_Foot", "X", -180, 180, (0, 0, -0.04)),

    # RIGHT LEG
    ("r_hip_roll", "Torso_Bot_Plate", "R_Hip1", "X", -180, 180, (0, -0.04, -0.05)),
    ("r_hip_pitch", "R_Hip1", "R_Hip2", "Y", -180, 180, (0, 0, -0.04)),
    ("r_knee_pitch", "R_Hip2", "R_Knee", "Y", -180, 180, (0, 0, -0.10)),
    ("r_ankle_pitch", "R_Knee", "R_Ankle", "Y", -180, 180, (0, 0, -0.10)),
    ("r_foot_roll", "R_Ankle", "R_Foot", "X", -180, 180, (0, 0, -0.04)),
]

def create_articulated_robot():
    """Create articulated robot from GLB with physics"""

    print("Creating new stage...")
    stage = Usd.Stage.CreateNew(USD_OUTPUT)

    # Set up physics scene
    print("Setting up physics scene...")
    scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    # Create root prim for robot
    print("Creating robot root...")
    robot_prim = stage.DefinePrim("/Humanoid", "Xform")
    stage.SetDefaultPrim(robot_prim)

    # Import GLB as payload (preserves mesh data)
    print(f"Importing GLB from {GLB_PATH}...")
    mesh_root = stage.DefinePrim("/Humanoid/Meshes", "Xform")
    mesh_root.GetReferences().AddReference(GLB_PATH)

    # Create base link (torso)
    print("Creating base link...")
    base_link = UsdGeom.Xform.Define(stage, "/Humanoid/base_link")

    # Make it an articulation root
    UsdPhysics.ArticulationRootAPI.Apply(base_link.GetPrim())

    # Add rigid body to base
    rigid_body = UsdPhysics.RigidBodyAPI.Apply(base_link.GetPrim())
    rigid_body.CreateRigidBodyEnabledAttr(True)

    # Add collision approximation for base (torso)
    collision_api = UsdPhysics.CollisionAPI.Apply(base_link.GetPrim())

    # Add mass properties to base
    mass_api = UsdPhysics.MassAPI.Apply(base_link.GetPrim())
    mass_api.CreateMassAttr(2.5)  # 2.5 kg torso

    print(f"Creating {len(JOINT_CONFIG)} articulated joints...")

    for joint_name, parent_name, child_name, axis, lower, upper, offset in JOINT_CONFIG:
        print(f"  Creating joint: {joint_name} ({parent_name} -> {child_name})")

        # Create link for this joint
        link_path = f"/Humanoid/{child_name}_link"
        link = UsdGeom.Xform.Define(stage, link_path)

        # Set relative position (offset from parent)
        # Note: In USD physics, joint anchors define the pivot. Here we simplify by moving the child link.
        # Ideally, we should set joint body0/body1 poses, but moving the child prim is a quick way to visualize structure.
        link.AddTranslateOp().Set(Gf.Vec3f(*offset))

        # Add rigid body
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        rigid_body.CreateRigidBodyEnabledAttr(True)

        # Add mass (lighter for extremities)
        mass_api = UsdPhysics.MassAPI.Apply(link.GetPrim())
        if "Foot" in child_name or "Hand" in child_name:
            mass_api.CreateMassAttr(0.1)
        elif "Leg" in child_name or "Arm" in child_name:
            mass_api.CreateMassAttr(0.3)
        else:
            mass_api.CreateMassAttr(0.2)

        # Add collision
        collision_api = UsdPhysics.CollisionAPI.Apply(link.GetPrim())

        # Create revolute joint
        joint_path = f"/Humanoid/{joint_name}"
        joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)

        # Set parent and child relationships
        if parent_name == "Torso_Main" or parent_name == "Torso_Bot_Plate":
            parent_path = "/Humanoid/base_link"
        else:
            parent_path = f"/Humanoid/{parent_name}_link"

        joint.CreateBody0Rel().SetTargets([parent_path])
        joint.CreateBody1Rel().SetTargets([link_path])

        # Set joint axis (UsdPhysics expects "X", "Y", "Z" token)
        joint.CreateAxisAttr(axis)

        # Set joint limits (convert degrees to radians)
        import math
        lower_rad = math.radians(lower)
        upper_rad = math.radians(upper)
        joint.CreateLowerLimitAttr(lower_rad)
        joint.CreateUpperLimitAttr(upper_rad)

        # Add drive for actuation
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateMaxForceAttr(100.0)  # Max torque in N⋅m
        drive.CreateDampingAttr(10.0)
        drive.CreateStiffnessAttr(0.0)

    # Add ground plane for training
    print("Adding ground plane...")
    ground = stage.DefinePrim("/World/Ground", "Xform")
    ground_geom = UsdGeom.Mesh.Define(stage, "/World/Ground/CollisionMesh")

    # Create simple ground plane
    points = [
        Gf.Vec3f(-5, -5, 0), Gf.Vec3f(5, -5, 0),
        Gf.Vec3f(5, 5, 0), Gf.Vec3f(-5, 5, 0)
    ]
    ground_geom.CreatePointsAttr(points)
    ground_geom.CreateFaceVertexCountsAttr([4])
    ground_geom.CreateFaceVertexIndicesAttr([0, 1, 2, 3])

    # Add collision to ground
    UsdPhysics.CollisionAPI.Apply(ground_geom.GetPrim())

    # Save the stage
    print(f"Saving articulated robot to {USD_OUTPUT}...")
    stage.Save()
    print("Done! Robot ready for Isaac Sim training.")
    print(f"\nTo use in Isaac Sim:")
    print(f"  1. Launch Isaac Sim")
    print(f"  2. File -> Open: {USD_OUTPUT}")
    print(f"  3. Press Play to test physics")

    simulation_app.close()
    return stage

if __name__ == "__main__":
    # Check if GLB exists
    if not os.path.exists(GLB_PATH):
        print(f"ERROR: GLB file not found at {GLB_PATH}")
        print("Run 'blender --background --python export_model.py' first")
        exit(1)

    create_articulated_robot()
