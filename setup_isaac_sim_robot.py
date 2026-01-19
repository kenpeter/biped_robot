"""
Isaac Sim Setup Script for Humanoid Robot
Creates articulated USD with EMBEDDED visible geometry (not GLB reference)
This fixes the mesh-binding problem by creating geometry directly in USD

Run from Isaac Sim:
  ./run_isaac.sh setup_isaac_sim_robot.py
"""

from isaacsim import SimulationApp

# Launch Isaac Sim in headless mode
simulation_app = SimulationApp({"headless": True})

import os
from pxr import Usd, UsdGeom, UsdPhysics, Gf
import math

# Paths
PROJECT_ROOT = os.getcwd()
USD_OUTPUT = os.path.join(PROJECT_ROOT, "models/humanoid_articulated.usda")

# Joint configuration
# Format: (joint_name, parent_link, child_link, axis, lower_limit, upper_limit, offset_xyz, visual_size)
JOINT_CONFIG = [
    # HEAD
    ("head_joint", "base_link", "Head_Servo", "Z", -180, 180, (0, 0, 0.08), 0.06),

    # LEFT ARM (y+ is left)
    ("l_shoulder_pitch", "base_link", "L_Shoulder1", "Y", -180, 180, (0, 0.06, 0.05), 0.05),
    ("l_shoulder_roll", "L_Shoulder1", "L_Shoulder2", "X", -180, 180, (0, 0.04, 0), 0.04),
    ("l_forearm_roll", "L_Shoulder2", "L_Elbow", "X", -180, 180, (0, 0, -0.08), 0.04),

    # RIGHT ARM (y- is right)
    ("r_shoulder_pitch", "base_link", "R_Shoulder1", "Y", -180, 180, (0, -0.06, 0.05), 0.05),
    ("r_shoulder_roll", "R_Shoulder1", "R_Shoulder2", "X", -180, 180, (0, -0.04, 0), 0.04),
    ("r_forearm_roll", "R_Shoulder2", "R_Elbow", "X", -180, 180, (0, 0, -0.08), 0.04),

    # LEFT LEG
    ("l_hip_roll", "base_link", "L_Hip1", "X", -180, 180, (0, 0.04, -0.05), 0.05),
    ("l_hip_pitch", "L_Hip1", "L_Hip2", "Y", -180, 180, (0, 0, -0.04), 0.05),
    ("l_knee_pitch", "L_Hip2", "L_Knee", "Y", -180, 180, (0, 0, -0.10), 0.04),
    ("l_ankle_pitch", "L_Knee", "L_Ankle", "Y", -180, 180, (0, 0, -0.10), 0.04),
    ("l_foot_roll", "L_Ankle", "L_Foot", "X", -180, 180, (0, 0, -0.04), 0.06),

    # RIGHT LEG
    ("r_hip_roll", "base_link", "R_Hip1", "X", -180, 180, (0, -0.04, -0.05), 0.05),
    ("r_hip_pitch", "R_Hip1", "R_Hip2", "Y", -180, 180, (0, 0, -0.04), 0.05),
    ("r_knee_pitch", "R_Hip2", "R_Knee", "Y", -180, 180, (0, 0, -0.10), 0.04),
    ("r_ankle_pitch", "R_Knee", "R_Ankle", "Y", -180, 180, (0, 0, -0.10), 0.04),
    ("r_foot_roll", "R_Ankle", "R_Foot", "X", -180, 180, (0, 0, -0.04), 0.06),
]

def create_visible_link(stage, link_path, size, color):
    """Create a link with embedded visible cube geometry"""
    # Create the link xform
    link = UsdGeom.Xform.Define(stage, link_path)

    # Add visible cube as child
    cube_path = f"{link_path}/visual"
    cube = UsdGeom.Cube.Define(stage, cube_path)
    cube.CreateSizeAttr(size)
    cube.CreateDisplayColorAttr([color])

    # Add collision to the cube
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    return link

def create_articulated_robot():
    """Create articulated robot with embedded visible geometry"""

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

    # Apply rotation: Blender Y-up to Isaac Z-up conversion
    # Quaternion [w, x, y, z] = [0.7071, -0.7071, 0, 0] = -90° around X-axis
    robot_xform = UsdGeom.Xform(robot_prim)
    robot_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.3))  # Lift robot above ground
    robot_xform.AddOrientOp().Set(Gf.Quatf(0.7071, -0.7071, 0.0, 0.0))

    # Create base link (torso) with visible cube
    print("Creating base link with visible geometry...")
    base_link = create_visible_link(
        stage,
        "/Humanoid/base_link",
        size=0.15,
        color=(0.8, 0.2, 0.2)  # Red torso
    )

    # Make it an articulation root
    UsdPhysics.ArticulationRootAPI.Apply(base_link.GetPrim())

    # Add rigid body to base
    rigid_body = UsdPhysics.RigidBodyAPI.Apply(base_link.GetPrim())
    rigid_body.CreateRigidBodyEnabledAttr(True)

    # Add mass properties to base
    mass_api = UsdPhysics.MassAPI.Apply(base_link.GetPrim())
    mass_api.CreateMassAttr(2.5)  # 2.5 kg torso

    print(f"Creating {len(JOINT_CONFIG)} articulated joints with visible geometry...")

    # Colors for different body parts
    colors = {
        "Head": (0.9, 0.9, 0.3),      # Yellow
        "L_": (0.2, 0.8, 0.2),         # Green (left side)
        "R_": (0.2, 0.2, 0.8),         # Blue (right side)
        "Foot": (0.5, 0.5, 0.5),       # Gray
    }

    for joint_name, parent_name, child_name, axis, lower, upper, offset, size in JOINT_CONFIG:
        print(f"  Creating joint: {joint_name} ({parent_name} -> {child_name})")

        # Determine color based on part name
        color = (0.6, 0.6, 0.6)  # Default gray
        for key, col in colors.items():
            if key in child_name:
                color = col
                break

        # Create link with visible geometry
        link_path = f"/Humanoid/{child_name}_link"
        link = create_visible_link(stage, link_path, size, color)

        # Set relative position (offset from parent)
        link.AddTranslateOp().Set(Gf.Vec3f(*offset))

        # Add rigid body
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        rigid_body.CreateRigidBodyEnabledAttr(True)

        # Add mass (lighter for extremities)
        mass_api = UsdPhysics.MassAPI.Apply(link.GetPrim())
        if "Foot" in child_name or "Hand" in child_name:
            mass_api.CreateMassAttr(0.1)
        elif "Leg" in child_name or "Arm" in child_name or "Hip" in child_name:
            mass_api.CreateMassAttr(0.3)
        else:
            mass_api.CreateMassAttr(0.2)

        # Create revolute joint
        joint_path = f"/Humanoid/{joint_name}"
        joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)

        # Set parent and child relationships
        parent_path = f"/Humanoid/{parent_name}"
        if parent_name == "base_link":
            parent_path = "/Humanoid/base_link"

        joint.CreateBody0Rel().SetTargets([parent_path])
        joint.CreateBody1Rel().SetTargets([link_path])

        # Set joint axis
        joint.CreateAxisAttr(axis)

        # Set joint limits (convert degrees to radians)
        lower_rad = math.radians(lower)
        upper_rad = math.radians(upper)
        joint.CreateLowerLimitAttr(lower_rad)
        joint.CreateUpperLimitAttr(upper_rad)

        # Add drive for actuation (EFFORT CONTROL - no stiffness spring)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateMaxForceAttr(50.0)      # Max torque: 50 N⋅m
        drive.CreateDampingAttr(1.0)        # Low damping for smooth motion
        drive.CreateStiffnessAttr(0.0)      # NO spring - pure effort control

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
    ground_geom.CreateDisplayColorAttr([(0.3, 0.3, 0.3)])  # Dark gray

    # Add collision to ground
    UsdPhysics.CollisionAPI.Apply(ground_geom.GetPrim())

    # Save the stage
    print(f"Saving articulated robot to {USD_OUTPUT}...")
    stage.Save()
    print("Done! Robot ready for Isaac Sim training.")
    print(f"\nRobot created with EMBEDDED visible geometry (not GLB reference)")
    print(f"  - Base link: RED cube")
    print(f"  - Head: YELLOW cube")
    print(f"  - Left arm/leg: GREEN cubes")
    print(f"  - Right arm/leg: BLUE cubes")
    print(f"  - Feet: GRAY cubes")
    print(f"\nTo use in Isaac Sim:")
    print(f"  1. ./run_isaac.sh test_joint_final.py")
    print(f"  2. You should see COLORED CUBES moving")

    simulation_app.close()
    return stage

if __name__ == "__main__":
    create_articulated_robot()
