"""Load humanoid USD in Isaac Sim - Auto demo each servo"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
from pxr import UsdLux, Gf
import numpy as np

USD_PATH = "/home/jetson/work/biped_robot/models/humanoid_simple.usda"

# Setup world
world = World(stage_units_in_meters=1.0, physics_dt=1.0/60.0)
world.scene.add_default_ground_plane()

# Add lighting
dome_light = UsdLux.DomeLight.Define(world.stage, "/World/DomeLight")
dome_light.CreateIntensityAttr(300)
dist_light = UsdLux.DistantLight.Define(world.stage, "/World/DistantLight")
dist_light.CreateIntensityAttr(500)
dist_light.AddRotateXYZOp().Set(Gf.Vec3f(-45, 45, 0))

# Load robot
add_reference_to_stage(usd_path=USD_PATH, prim_path="/World/robot")
robot = world.scene.add(SingleArticulation(prim_path="/World/robot", name="robot"))
world.reset()

print(f"\nRobot: {robot.num_dof} joints")
print(f"Names: {robot.dof_names}\n")

if robot.num_dof == 0:
    print("ERROR: No joints detected!")
    simulation_app.close()
    exit(1)

print("Auto-testing each servo (2s each)...\n")

# Demo loop
servo_idx = 0
frame_count = 0
moving_to_test = True
frames_per_move = 120  # 2 seconds at 60fps

while simulation_app.is_running():
    if frame_count == 0:
        positions = np.zeros(robot.num_dof)

        if moving_to_test:
            angle = 30 if servo_idx not in [8, 12] else -45  # Knees bend back
            positions[servo_idx] = np.radians(angle)
            print(f"[{servo_idx+1}/{robot.num_dof}] {robot.dof_names[servo_idx]}: {angle}°")

        robot._articulation_view.set_joint_position_targets(
            positions=positions, joint_indices=np.arange(robot.num_dof)
        )

    world.step(render=True)
    frame_count += 1

    if frame_count >= frames_per_move:
        frame_count = 0
        if moving_to_test:
            moving_to_test = False
        else:
            moving_to_test = True
            servo_idx = (servo_idx + 1) % robot.num_dof
            if servo_idx == 0:
                print("\n--- Restarting demo ---\n")

simulation_app.close()
