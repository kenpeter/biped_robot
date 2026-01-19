#!/usr/bin/env python3
"""Minimal test"""
import sys
import os
import numpy as np
print("TEST STARTED", flush=True)

try:
    from isaacsim import SimulationApp
    print("Imported SimulationApp", flush=True)
    
    simulation_app = SimulationApp({"headless": True})
    print("Created SimulationApp", flush=True)
    
    from omni.isaac.core import World
    from omni.isaac.core.articulations import ArticulationView
    from omni.isaac.core.utils.prims import create_prim
    print("Imported World", flush=True)
    
    ROBOT_USD = os.path.join(os.getcwd(), "models/humanoid_articulated.usda")
    print(f"Robot USD: {ROBOT_USD}", flush=True)
    
    world = World(stage_units_in_meters=1.0)
    print("Created World", flush=True)
    
    world.scene.add_default_ground_plane()
    print("Added ground plane", flush=True)
    
    create_prim(
        prim_path="/World/Humanoid",
        prim_type="Xform",
        usd_path=ROBOT_USD,
        position=np.array([0.0, 0.0, 0.2]),
        orientation=np.array([0.7071, 0.7071, 0.0, 0.0])
    )
    print("Created robot prim", flush=True)
    
    world.reset()
    print("Reset world", flush=True)
    
    robot = ArticulationView("/World/Humanoid")
    print(f"Robot DOFs: {robot.num_dof}", flush=True)
    print(f"Joint positions: {robot.get_joint_positions()[0]}", flush=True)
    
    # Apply torque
    print("\nApplying torque...", flush=True)
    for i in range(10):
        actions = np.zeros(robot.num_dof)
        actions[0] = 5.0
        robot.set_joint_efforts(actions)
        world.step(render=False)
        
        vel = robot.get_joint_velocities()
        pos = robot.get_joint_positions()
        print(f"Step {i}: Joint 0 vel={vel[0,0]:.4f}, pos={pos[0,0]:.4f}", flush=True)
    
    print(f"\nFinal joint 0 position: {robot.get_joint_positions()[0,0]:.4f}", flush=True)
    print("ROBOT IS MOVING!" if robot.get_joint_positions()[0,0] != 0 else "Robot not moving", flush=True)
    
    simulation_app.close()
    print("TEST PASSED", flush=True)
    
except Exception as e:
    print(f"ERROR: {e}", flush=True)
    import traceback
    traceback.print_exc()
    sys.exit(1)
