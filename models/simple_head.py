"""Simple head servo test - rotates left/right 10 degrees with camera"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import UsdGeom, Gf, UsdPhysics, UsdShade, Sdf
import numpy as np
import math

# Create world
world = World()
world.scene.add_default_ground_plane()

print("Creating simple head servo robot...")

stage = world.stage

# Create base (fixed to ground)
base_prim = stage.DefinePrim("/World/Robot/base", "Xform")
UsdGeom.Xform(base_prim).AddTranslateOp().Set(Gf.Vec3f(0, 0, 0.5))

base_geom = UsdGeom.Cube.Define(stage, "/World/Robot/base/geometry")
base_geom.GetSizeAttr().Set(0.2)
base_geom.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0))

# Create head (rotates on Z axis)
head_prim = stage.DefinePrim("/World/Robot/head", "Xform")
UsdGeom.Xform(head_prim).AddTranslateOp().Set(Gf.Vec3f(0, 0, 0.65))

head_geom = UsdGeom.Cube.Define(stage, "/World/Robot/head/geometry")
head_geom.GetSizeAttr().Set(0.15)
head_geom.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0))

# Color the head (blue)
material = UsdShade.Material.Define(stage, "/World/Robot/head/material")
shader = UsdShade.Shader.Define(stage, "/World/Robot/head/material/shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.2, 0.2, 0.8))
material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI(head_geom.GetPrim()).Bind(material)

# Create camera visual (small black box on front of head)
camera_geom = UsdGeom.Cube.Define(stage, "/World/Robot/head/camera_box")
camera_geom.GetSizeAttr().Set(0.04)
camera_geom.AddTranslateOp().Set(Gf.Vec3f(0, 0.10, 0))  # In front of head

# Color the camera (black)
camera_material = UsdShade.Material.Define(stage, "/World/Robot/head/camera_material")
camera_shader = UsdShade.Shader.Define(stage, "/World/Robot/head/camera_material/shader")
camera_shader.CreateIdAttr("UsdPreviewSurface")
camera_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.1, 0.1, 0.1))
camera_material.CreateSurfaceOutput().ConnectToSource(camera_shader.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI(camera_geom.GetPrim()).Bind(camera_material)

# Create actual camera (for rendering view)
camera_prim = stage.DefinePrim("/World/Robot/head/camera", "Camera")
UsdGeom.Xform(camera_prim).AddTranslateOp().Set(Gf.Vec3f(0, 0.12, 0))
camera = UsdGeom.Camera(camera_prim)
camera.GetFocalLengthAttr().Set(18.0)

# Create revolute joint (head rotates on Z axis)
joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/head_joint")
joint.CreateAxisAttr("Z")
joint.CreateBody0Rel().SetTargets(["/World/Robot/base"])
joint.CreateBody1Rel().SetTargets(["/World/Robot/head"])
joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0.15))
joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, -0.075))

# Set joint limits: -10 to +10 degrees
joint.CreateLowerLimitAttr().Set(math.radians(-10))
joint.CreateUpperLimitAttr().Set(math.radians(10))

# Add articulation root
UsdPhysics.ArticulationRootAPI.Apply(base_prim)

world.reset()

print("\n=== SIMPLE HEAD SERVO ROBOT ===")
print("Components:")
print("  - Base (gray cube, fixed to ground)")
print("  - Head (blue cube, rotates on Z axis)")
print("  - Camera (attached to head)")
print("\nJoint limits: -10° to +10°")
print("\nControls:")
print("  Press LEFT/RIGHT arrow to rotate head")
print("  Press Ctrl+C to exit")
print()

# Animation state
angle = 0.0
direction = 1
max_angle = math.radians(10)
speed = 0.5  # radians per second

# Get articulation
from omni.isaac.core.utils.prims import get_prim_at_path
robot_prim = get_prim_at_path("/World/Robot")

# Main loop
try:
    frame_count = 0
    while simulation_app.is_running():
        world.step(render=True)

        # Animate head rotation (oscillate between -10 and +10 degrees)
        frame_count += 1
        if frame_count % 2 == 0:  # Update every other frame for smooth motion
            angle += direction * speed * 0.016  # Assuming ~60 fps

            if angle >= max_angle:
                angle = max_angle
                direction = -1
            elif angle <= -max_angle:
                angle = -max_angle
                direction = 1

            # Set head rotation
            head_xform = UsdGeom.Xform(head_prim)
            head_xform.ClearXformOpOrder()
            head_xform.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0.65))
            head_xform.AddRotateZOp().Set(math.degrees(angle))

except KeyboardInterrupt:
    pass

simulation_app.close()
