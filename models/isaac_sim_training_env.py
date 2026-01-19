"""
Isaac Sim Training Environment for Humanoid Robot
Sets up complete RL training environment with robot, sensors, and rewards

Run from Isaac Sim:
  /home/kenpeter/work/IsaacSim/python.sh isaac_sim_training_env.py
"""

import os
import sys
import numpy as np
from isaacsim import SimulationApp

print("=" * 80, flush=True)
print("ISAAC SIM TRAINING ENVIRONMENT STARTING", flush=True)
print("=" * 80, flush=True)

# Launch Isaac Sim
print("\n[1/5] Launching Isaac Sim...", flush=True)
simulation_app = SimulationApp({"headless": False})

# IMPORTANT: Update simulation to process initialization
for _ in range(10):
    simulation_app.update()

print("      Isaac Sim window opened!", flush=True)

import omni
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Gf, UsdPhysics

# Paths
PROJECT_ROOT = os.getcwd()
ROBOT_USD = os.path.join(PROJECT_ROOT, "models/humanoid_articulated.usda")

# Update simulation after imports
for _ in range(10):
    simulation_app.update()

class HumanoidTrainingEnv:
    """Training environment for bipedal humanoid robot"""

    def __init__(self):
        # Create world
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        print("Loading humanoid robot...")
        # Add robot to scene using create_prim
        robot_prim_path = "/World/Humanoid"
        # Fix Blender Y-up to Isaac Sim Z-up: rotate +90° around X axis
        # Quaternion for +90° X rotation: [cos(45°), sin(45°), 0, 0]
        create_prim(
            prim_path=robot_prim_path,
            prim_type="Xform",
            usd_path=ROBOT_USD,
            position=np.array([0.0, 0.0, 0.15]),  # Spawn 15cm above ground
            orientation=np.array([0.7071, 0.7071, 0.0, 0.0]) # +90° X rotation (w, x, y, z)
        )

        # Initialize world to load USD references
        print("Initializing world (this loads USD references)...")
        self.world.reset()

        # Debug: Print stage hierarchy to confirm paths
        print("\nStage Hierarchy after world reset:")
        for prim in self.world.stage.Traverse():
            print(f"  {prim.GetPath()}")

        # Get articulation view (for controlling joints)
        # The articulation root should be at /World/Humanoid (not /World/Humanoid/base_link)
        print(f"\nAdding articulation at path: {robot_prim_path}")
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
            print("Trying alternate path: {robot_prim_path}/base_link")
            self.robot = self.world.scene.add(
                Articulation(
                    prim_path=f"{robot_prim_path}/base_link",
                    name="humanoid_robot"
                )
            )

        print("Setting up camera and sensors...")
        # Add overhead camera for monitoring
        camera_path = "/World/Camera"
        create_prim(
            camera_path,
            "Camera",
            position=np.array([2.0, 2.0, 1.5]),
            orientation=np.array([0.92, -0.38, 0, 0])  # Looking at robot
        )

        # Get number of DOFs
        self.num_dof = self.robot.num_dof
        print(f"Robot has {self.num_dof} degrees of freedom")

        # Joint names (from CLAUDE.md)
        self.joint_names = [
            "head_joint",
            "l_shoulder_pitch", "l_shoulder_roll", "l_forearm_roll",
            "r_shoulder_pitch", "r_shoulder_roll", "r_forearm_roll",
            "l_hip_roll", "l_hip_pitch", "l_knee_pitch", "l_ankle_pitch", "l_foot_roll",
            "r_hip_roll", "r_hip_pitch", "r_knee_pitch", "r_ankle_pitch", "r_foot_roll",
        ]

        print("Training environment ready!")

    def reset(self):
        """Reset robot to initial standing pose"""
        print("Resetting robot...")

        # Initial standing pose (in radians)
        # 0: head, 1-3: l_arm, 4-6: r_arm, 7-11: l_leg, 12-16: r_leg
        initial_pose = np.array([
            0.0,  # head center
            0.0, 0.0, 0.0,  # left arm down
            0.0, 0.0, 0.0,  # right arm down
            0.0, 0.0, -0.2, 0.1, 0.0,  # left leg slightly bent
            0.0, 0.0, -0.2, 0.1, 0.0,  # right leg slightly bent
        ])

        # Set joint positions
        self.robot.set_joint_positions(initial_pose)

        # Zero velocities
        self.robot.set_joint_velocities(np.zeros(self.num_dof))

        self.world.reset()

    def step(self, actions):
        """
        Apply actions and step simulation

        Args:
            actions: numpy array of joint torques (shape: num_dof)

        Returns:
            observations, rewards, done
        """
        # Apply joint torques
        self.robot.set_joint_efforts(actions)

        # Step physics
        self.world.step(render=True)

        # Get observations
        joint_positions = self.robot.get_joint_positions()
        joint_velocities = self.robot.get_joint_velocities()
        root_position, root_orientation = self.robot.get_world_pose()

        observations = {
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "root_position": root_position,
            "root_orientation": root_orientation,
        }

        # Calculate reward (basic: encourage standing upright and forward motion)
        height = root_position[2]  # Z coordinate
        height_reward = height - 0.25  # Target height ~25cm (standing)

        velocity = joint_velocities.mean()
        energy_penalty = -0.001 * np.sum(actions ** 2)  # Penalize high torques

        reward = height_reward + energy_penalty

        # Done if robot falls
        done = height < 0.1  # If center of mass below 10cm, consider fallen

        return observations, reward, done

    def test_servos_individually(self):
        """Test each servo individually to verify joint mapping and direction"""
        print("\n" + "="*80)
        print("SERVO TEST MODE - Testing each joint individually")
        print("="*80)

        self.reset()

        # Test parameters
        test_angle = np.radians(30)  # 30 degrees
        hold_steps = 100  # Hold each position for 100 steps (~1.6 seconds at 60Hz)
        kp = 100.0  # Strong position control
        kd = 10.0

        for joint_idx in range(self.num_dof):
            joint_name = self.joint_names[joint_idx] if joint_idx < len(self.joint_names) else f"joint_{joint_idx}"

            print(f"\n[Testing Joint {joint_idx + 1}/{self.num_dof}] {joint_name}")
            print(f"  Moving to +{np.degrees(test_angle):.1f}° ...")

            # Move this joint to test angle
            for step in range(hold_steps):
                current_pos = self.robot.get_joint_positions()
                current_vel = self.robot.get_joint_velocities()

                # Target: all neutral except current joint
                target_pos = np.zeros(self.num_dof)
                target_pos[joint_idx] = test_angle

                # PD control
                actions = kp * (target_pos - current_pos) - kd * current_vel
                self.step(actions)

            print(f"  Moving to -{np.degrees(test_angle):.1f}° ...")

            # Move to negative angle
            for step in range(hold_steps):
                current_pos = self.robot.get_joint_positions()
                current_vel = self.robot.get_joint_velocities()

                target_pos = np.zeros(self.num_dof)
                target_pos[joint_idx] = -test_angle

                actions = kp * (target_pos - current_pos) - kd * current_vel
                self.step(actions)

            print(f"  Returning to neutral (0°) ...")

            # Return to neutral
            for step in range(hold_steps):
                current_pos = self.robot.get_joint_positions()
                current_vel = self.robot.get_joint_velocities()

                target_pos = np.zeros(self.num_dof)

                actions = kp * (target_pos - current_pos) - kd * current_vel
                self.step(actions)

            print(f"  ✓ Joint {joint_idx + 1} test complete")

        print("\n" + "="*80)
        print("ALL SERVOS TESTED!")
        print("="*80)

    def run_demo(self, steps=1000):
        """Run a demo with simple PD control"""
        print(f"Running {steps} step demo...")

        self.reset()

        for step in range(steps):
            # Simple PD controller to maintain standing pose
            current_pos = self.robot.get_joint_positions()
            current_vel = self.robot.get_joint_velocities()

            # Target: slight knee bend for balance
            target_pos = np.array([
                0.0,  # head center
                0.0, 0.0, 0.0,  # left arm
                0.0, 0.0, 0.0,  # right arm
                0.0, 0.0, -0.3, 0.15, 0.0,  # left leg bent
                0.0, 0.0, -0.3, 0.15, 0.0,  # right leg bent
            ])

            # PD control
            kp = 50.0  # Position gain
            kd = 5.0   # Velocity gain
            actions = kp * (target_pos - current_pos) - kd * current_vel

            # Apply and step
            obs, reward, done = self.step(actions)

            if step % 100 == 0:
                height = obs["root_position"][2]
                print(f"Step {step}: Height={height:.3f}m, Reward={reward:.3f}")

            if done:
                print(f"Robot fell at step {step}!")
                break

        print("Demo complete!")

    def close(self):
        """Clean up"""
        simulation_app.close()


if __name__ == "__main__":
    print("\n[2/5] Checking USD file...")
    # Check if articulated USD exists
    if not os.path.exists(ROBOT_USD):
        print(f"      ERROR: Articulated USD not found at {ROBOT_USD}")
        print("      Run setup_isaac_sim_robot.py first:")
        print("      /home/kenpeter/work/IsaacSim/python.sh setup_isaac_sim_robot.py")
        simulation_app.close()
        exit(1)
    print(f"      Found USD at: {ROBOT_USD}")

    # Create environment and run demo
    print("\n[3/5] Creating training environment...")
    env = HumanoidTrainingEnv()
    print("      Environment created successfully!")

    try:
        # Run servo test to verify joint mapping
        print("\n[4/5] Running servo test (each joint individually)...")
        print("      Watch each servo move in the Isaac Sim window!")
        env.test_servos_individually()

        # Keep window open
        print("\n[5/5] Demo finished!")
        print("=" * 80)
        print("Robot demo complete! Window will stay open.")
        print("Press Ctrl+C in terminal to exit.")
        print("=" * 80)
        while simulation_app.is_running():
            env.world.step(render=True)

    except KeyboardInterrupt:
        print("\n\nShutting down Isaac Sim...")
    except Exception as e:
        print(f"\n\nERROR during demo: {e}")
        import traceback
        traceback.print_exc()
    finally:
        env.close()
        print("Isaac Sim closed.")
