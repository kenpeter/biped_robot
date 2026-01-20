"""Build accurate USD model from physical measurements"""
import json
import math
from pxr import Usd, UsdGeom, UsdPhysics, Gf

# Load measurements (from measure_robot.py output)
try:
    with open('robot_measurements.json', 'r') as f:
        measurements = json.load(f)
    print(f"✓ Loaded measurements for {len(measurements)} joints")
except FileNotFoundError:
    print("✗ robot_measurements.json not found!")
    print("Run measure_robot.py on the Jetson first to capture joint ranges.")
    exit(1)

# Create USD stage
stage = Usd.Stage.CreateNew('humanoid_measured.usda')
print("✓ Creating USD stage: humanoid_measured.usda")

# Define world
world = UsdGeom.Xform.Define(stage, '/World')

# Robot root
robot = UsdGeom.Xform.Define(stage, '/World/Robot')

# Base (fixed to ground)
base = UsdGeom.Cube.Define(stage, '/World/Robot/base')
base.GetSizeAttr().Set(0.2)
base.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0.5))

# Apply physics
UsdPhysics.CollisionAPI.Apply(base.GetPrim())
mass_api = UsdPhysics.MassAPI.Apply(base.GetPrim())
mass_api.GetMassAttr().Set(1.0)

# HEAD
if 'head_joint' in measurements:
    print("\n✓ Adding head with measured limits...")
    data = measurements['head_joint']

    # Head link
    head = UsdGeom.Cube.Define(stage, '/World/Robot/head')
    head.GetSizeAttr().Set(0.15)
    head.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0.65))

    # Physics
    UsdPhysics.CollisionAPI.Apply(head.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(head.GetPrim())
    mass_api.GetMassAttr().Set(0.5)

    # Camera (visual)
    camera_box = UsdGeom.Cube.Define(stage, '/World/Robot/head/camera')
    camera_box.GetSizeAttr().Set(0.04)
    camera_box.AddTranslateOp().Set(Gf.Vec3f(0, 0.10, 0))

    # Revolute joint with measured limits
    joint = UsdPhysics.RevoluteJoint.Define(stage, '/World/Robot/head_joint')
    joint.CreateAxisAttr("Z")
    joint.CreateBody0Rel().SetTargets(['/World/Robot/base'])
    joint.CreateBody1Rel().SetTargets(['/World/Robot/head'])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0.15))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, -0.075))

    # Set measured limits
    joint.CreateLowerLimitAttr().Set(data['min_rad'])
    joint.CreateUpperLimitAttr().Set(data['max_rad'])

    print(f"  head_joint: {data['min_deg']:.1f}° to {data['max_deg']:.1f}°")

# Helper to create a limb with multiple joints
def create_limb(parent_path, name, joints_data, positions):
    """Create a limb (arm or leg) with measured joint limits"""
    print(f"\n✓ Adding {name}...")

    prev_path = parent_path
    z = 0.5

    for i, (joint_name, pos) in enumerate(zip(joints_data, positions)):
        if joint_name not in measurements:
            print(f"  ⚠️  {joint_name} not measured, skipping...")
            continue

        data = measurements[joint_name]

        # Link
        link_path = f'{parent_path}/{name}_link_{i}'
        link = UsdGeom.Cube.Define(stage, link_path)
        link.GetSizeAttr().Set(0.04)
        link.AddTranslateOp().Set(Gf.Vec3f(pos[0], pos[1], pos[2]))

        # Physics
        UsdPhysics.CollisionAPI.Apply(link.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(link.GetPrim())
        mass_api.GetMassAttr().Set(0.2)

        # Joint
        joint_path = f'{parent_path}/{joint_name}'
        joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)

        # Determine axis from joint name (this is critical!)
        if 'roll' in joint_name:
            axis = "X"  # Roll = side-to-side
        elif 'pitch' in joint_name:
            axis = "Y"  # Pitch = forward-back
        elif 'yaw' in joint_name:
            axis = "Z"  # Yaw = rotation
        else:
            axis = "Y"  # Default

        joint.CreateAxisAttr(axis)
        joint.CreateBody0Rel().SetTargets([prev_path])
        joint.CreateBody1Rel().SetTargets([link_path])

        # Set measured limits
        joint.CreateLowerLimitAttr().Set(data['min_rad'])
        joint.CreateUpperLimitAttr().Set(data['max_rad'])

        print(f"  {joint_name:20s}: {data['min_deg']:6.1f}° to {data['max_deg']:6.1f}° (axis: {axis})")

        prev_path = link_path

# TODO: Add arms and legs using measurements
# For now, just demonstrating with head

# Apply articulation root
UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

# Save
stage.Save()
print(f"\n✓ Saved humanoid_measured.usda")
print("\nNext steps:")
print("  1. Open in Isaac Sim to verify")
print("  2. Use for training")
print("  3. Servo mapping is already in robot_measurements.json")
