"""Load humanoid USD in Isaac Sim"""
from isaacsim import SimulationApp
import time

USD_PATH = "/home/kenpeter/work/biped_robot/src/humanoid_description/usd/humanoid.usda"

simulation_app = SimulationApp({"headless": False})

from omni.usd import get_context
from pxr import UsdLux, Gf

ctx = get_context()
success = ctx.open_stage(USD_PATH)

if success:
    print(f"Opened: {USD_PATH}")
    stage = ctx.get_stage()

    # Add lighting to the scene
    dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome_light.CreateIntensityAttr(300)

    dist_light = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    dist_light.CreateIntensityAttr(500)
    dist_light.AddRotateXYZOp().Set(Gf.Vec3f(-45, 45, 0))

    print("Added lighting to scene")

    # Wait a bit for stage to fully load
    time.sleep(2)
    
    robot = stage.GetPrimAtPath("/World/robot")
    if robot:
        print(f"Robot prim found: {robot.GetPath()}")
        for child in robot.GetChildren():
            print(f"  - {child.GetPath()}")
    else:
        print("Robot prim not found!")
        
    # Print all children of World
    world = stage.GetPrimAtPath("/World")
    if world:
        print("\nAll children of /World:")
        for child in world.GetChildren():
            print(f"  - {child.GetPath()}")
else:
    print("Failed to open stage!")

print("\nScript complete. Close window to exit.")

# Keep the simulation running so user can interact with UI
while simulation_app.is_running():
    simulation_app.update()
