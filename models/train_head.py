"""Train head servo model in Isaac Sim.
Uses head_model.py for neural network, adds Isaac Sim training loop.
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation
from pxr import UsdLux
import numpy as np
import os

from head_model import HeadServoModel

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
UsdLux.DistantLight.Define(world.stage, "/World/Light").CreateIntensityAttr(3000)

usd_path = os.path.join(os.path.dirname(__file__), "head_robot.usda")
add_reference_to_stage(usd_path=usd_path, prim_path="/World/Robot")
robot = world.scene.add(SingleArticulation(prim_path="/World/Robot", name="robot"))
world.reset()


print("\n" + "="*50)
print("TRAINING HEAD SERVO MODEL")
print("="*50)

print("\n[1/3] Collecting training data...")
X_train = []
y_train = []
target_angles = [-30, -20, -10, 0, 10, 20, 30]

for target_deg in target_angles:
    target_rad = np.radians(target_deg)
    robot._articulation_view.set_joint_position_targets(
        positions=np.array([target_rad]),
        joint_indices=np.array([0])
    )
    for _ in range(60):
        world.step(render=True)
    actual_rad = robot.get_joint_positions()[0]
    actual_deg = np.degrees(actual_rad)
    X_train.append([target_deg])
    y_train.append([actual_deg - target_deg])
    print(f"  Target: {target_deg:+3d}° -> Actual: {actual_deg:+6.1f}°")

X_train = np.array(X_train)
y_train = np.array(y_train)

print("\n[2/3] Training neural network...")
model = HeadServoModel()
model.train(X_train, y_train, epochs=2000, lr=0.01)

print("\n[3/3] Saving model...")
model.save("head_model_weights.json")

print("\nTest predictions:")
for angle in [-30, -15, 0, 15, 30]:
    pos = model.predict(angle)
    print(f"  {angle:+3d}° -> {pos:.0f}")

simulation_app.close()
print("\n✓ Saved to head_model_weights.json")
print("✓ Ready for Jetson deployment")
