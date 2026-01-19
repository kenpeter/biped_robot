#!/usr/bin/env python3
"""
Simple Robot Test - Direct Torque Control
Tests if robot responds to direct joint torques
"""

import os
import sys
import numpy as np
from isaacsim import SimulationApp

print("=" * 80)
print("SIMPLE ROBOT TORQUE TEST")
print("=" * 80)

# Launch Isaac Sim
print("\n[1/3] Launching Isaac Sim...")
simulation_app = SimulationApp({"headless": False})

# IMPORTANT: Update simulation to process initialization
for _ in range(10):
    simulation_app.update()

print("      Isaac Sim window opened!", flush=True)

import omni
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import create_prim

# Paths
PROJECT_ROOT = os.getcwd()
ROBOT_USD = os.path.join(PROJECT_ROOT, "models/humanoid_articulated.usda")

# Update simulation after imports
for _ in range(10):
    simulation_app.update()

class SimpleRobotTest:
    """Simple test for direct torque control"""

    def __init__(self):
        # Create world
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        print("Loading humanoid robot...")
        # Add robot to scene using create_prim
        robot_prim_path = "/World/Humanoid"
        create_prim(
            prim_path=robot_prim_path,
            prim_type="Xform",
            usd_path=ROBOT_USD,
            position=np.array([0.0, 0.0, 0.15]),  # Spawn 15cm above ground
            orientation=np.array([0.7071, 0.7071, 0.0, 0.0]) # +90° X rotation
        )

        # Initialize world to load USD references
        print("Initializing world...")
        self.world.reset()

        # Get articulation view
        print(f"Adding articulation at path: {robot_prim_path}")
        try:
            self.robot = self.world.scene.add(
                Articulation(
                    prim_path=robot_prim_path,
                    name="humanoid_robot"
                )
            )
            print(f"Successfully added articulation!")
        except Exception as e:
            print(f"ERROR adding articulation: {e}")
            raise

        # Get number of DOFs
        self.num_dof = self.robot.num_dof
        print(f"Robot has {self.num_dof} degrees of freedom")

        print("Simple test ready!")

    def test_direct_torques(self):
        """Test direct torque control on each joint"""
        print("\n" + "="*60)
        print("TESTING DIRECT TORQUE CONTROL")
        print("="*60)

        # Reset robot to neutral positions
        initial_positions = np.zeros(self.num_dof)
        self.robot.set_joint_positions(initial_positions)
        self.robot.set_joint_velocities(np.zeros(self.num_dof))
        self.world.reset()

        # Test each joint with direct torque
        test_torque = 5.0  # Small test torque
        test_steps = 50   # Short test

        for joint_idx in range(min(5, self.num_dof)):  # Test first 5 joints only
            print(f"\n[Testing Joint {joint_idx + 1}]")
            print(f"  Applying torque {test_torque:.1f} N⋅m for {test_steps} steps...")
            
            for step in range(test_steps):
                # Apply torque to only this joint
                actions = np.zeros(self.num_dof)
                actions[joint_idx] = test_torque
                self.robot.set_joint_efforts(actions)
                
                # Step simulation
                self.world.step(render=True)
                
                # Get joint velocity to verify movement
                velocities = self.robot.get_joint_velocities()
                if abs(velocities[joint_idx]) > 0.01:  # If joint is moving
                    print(f"  ✓ Joint {joint_idx + 1} is moving! Velocity: {velocities[joint_idx]:.3f}")
                    break
            else:
                if step == test_steps - 1:
                    print(f"  ✗ Joint {joint_idx + 1} not responding")
            
            # Stop torque
            self.robot.set_joint_efforts(np.zeros(self.num_dof))
            self.world.step(render=True)
            
            # Brief pause
            for _ in range(10):
                self.world.step(render=True)

        print("\n" + "="*60)
        print("TORQUE TEST COMPLETE!")
        print("="*60)

    def close(self):
        """Clean up"""
        simulation_app.close()


if __name__ == "__main__":
    print("\n[2/3] Checking USD file...")
    # Check if articulated USD exists
    if not os.path.exists(ROBOT_USD):
        print(f"      ERROR: Articulated USD not found at {ROBOT_USD}")
        print("      Run setup_isaac_sim_robot.py first:")
        simulation_app.close()
        exit(1)
    print(f"      Found USD at: {ROBOT_USD}")

    # Create environment and run test
    print("\n[3/3] Creating simple test...")
    env = SimpleRobotTest()
    print("      Test environment created successfully!")

    try:
        # Run torque test
        env.test_direct_torques()
        
        # Keep window open
        print("\nTest complete! Window will stay open.")
        print("Press Ctrl+C in terminal to exit.")
        print("=" * 80)
        
        while simulation_app.is_running():
            env.world.step(render=True)

    except KeyboardInterrupt:
        print("\n\nShutting down Isaac Sim...")
    except Exception as e:
        print(f"\n\nERROR during test: {e}")
        import traceback
        traceback.print_exc()
    finally:
        env.close()
        print("Isaac Sim closed.")