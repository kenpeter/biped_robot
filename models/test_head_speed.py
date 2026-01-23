"""Test head servo speed - visual check.
Should move left 30° -> right 30° slowly (0.5s each way).
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
from pxr import UsdLux
import numpy as np
import os
import time as pytime

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
UsdLux.DistantLight.Define(world.stage, "/World/Light").CreateIntensityAttr(3000)

usd_path = os.path.join(os.path.dirname(__file__), "head_robot.usda")
add_reference_to_stage(usd_path=usd_path, prim_path="/World/Robot")
robot = world.scene.add(SingleArticulation(prim_path="/World/Robot", name="robot"))
world.reset()

# Real hardware timing
SECONDS_PER_360 = 6.0
VELOCITY_DEG_PER_SEC = 60.0  # 360 / 6
SIM_DT = 1.0 / 60.0

print("\n" + "="*50)
print("HEAD SERVO SPEED TEST")
print("="*50)
print(f"Real hardware: {VELOCITY_DEG_PER_SEC}°/s (6s per 360°)")
print(f"Should take 0.5s to move 30°")
print("\nWatch the robot head - does it look realistic?")
print("Press Ctrl+C to stop\n")

def move_to_angle(target_deg, from_deg=0.0):
    """Move from current position to target angle with proper stopping."""
    delta_deg = target_deg - from_deg
    if abs(delta_deg) < 0.5:
        return from_deg

    # Calculate time and velocity
    time_needed = abs(delta_deg) / VELOCITY_DEG_PER_SEC
    velocity_rad = np.radians(VELOCITY_DEG_PER_SEC) * np.sign(delta_deg)
    target_rad = np.radians(target_deg)

    print(f"Moving {from_deg:+5.1f}° → {target_deg:+5.1f}° (should take {time_needed:.2f}s)...", end='', flush=True)
    start_real_time = pytime.time()

    # Move until we reach target or timeout
    max_steps = int((time_needed + 0.5) / SIM_DT)  # Add buffer time
    for i in range(max_steps):
        current_rad = robot.get_joint_positions()[0]
        current_deg = np.degrees(current_rad)
        error_deg = target_deg - current_deg

        # Stop when within 1 degree of target
        if abs(error_deg) < 1.0:
            break

        # Reduce velocity as we approach target (deceleration)
        if abs(error_deg) < 5.0:
            # Slow down in last 5 degrees
            scale = abs(error_deg) / 5.0
            current_velocity = velocity_rad * max(scale, 0.2)
        else:
            current_velocity = velocity_rad

        robot._articulation_view.set_joint_velocity_targets(
            velocities=np.array([current_velocity]),
            joint_indices=np.array([0])
        )
        world.step(render=True)

    # Stop completely
    robot._articulation_view.set_joint_velocity_targets(
        velocities=np.array([0.0]),
        joint_indices=np.array([0])
    )
    for _ in range(20):
        world.step(render=True)

    actual_deg = np.degrees(robot.get_joint_positions()[0])
    elapsed = pytime.time() - start_real_time
    error = actual_deg - target_deg
    print(f" reached {actual_deg:+5.1f}° in {elapsed:.2f}s (error: {error:+.1f}°)")
    return actual_deg

# Continuous loop: center -> left 30 -> center -> right 30 -> repeat
try:
    current = 0.0
    while True:
        # Left 30
        current = move_to_angle(-30.0, current)
        pytime.sleep(0.5)

        # Center
        current = move_to_angle(0.0, current)
        pytime.sleep(0.5)

        # Right 30
        current = move_to_angle(30.0, current)
        pytime.sleep(0.5)

        # Center
        current = move_to_angle(0.0, current)
        pytime.sleep(0.5)

except KeyboardInterrupt:
    print("\nStopped")

simulation_app.close()
