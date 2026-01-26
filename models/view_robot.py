#!/usr/bin/env python3
"""View the full robot model in MuJoCo viewer."""

import mujoco
import mujoco.viewer
import numpy as np
import time

MODEL_PATH = '/home/kenpeter/work/biped_robot/models/full_robot.xml'

print("Loading robot model...")
mj_model = mujoco.MjModel.from_xml_path(MODEL_PATH)
mj_data = mujoco.MjData(mj_model)

# Set initial pose (standing)
mj_data.qpos[2] = 0.42  # torso height
mj_data.qpos[3] = 1.0   # quaternion w

# Bent knees for stability
mj_data.qpos[7 + 8] = np.radians(15)   # right hip pitch
mj_data.qpos[7 + 9] = np.radians(30)   # right knee
mj_data.qpos[7 + 12] = np.radians(15)  # left hip pitch
mj_data.qpos[7 + 13] = np.radians(30)  # left knee

mujoco.mj_forward(mj_model, mj_data)

print("\nOpening viewer...")
print("Controls:")
print("  - Mouse drag: rotate camera")
print("  - Scroll: zoom")
print("  - Press ESC or close window to exit")
print()

with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
    viewer.cam.distance = 2.5
    viewer.cam.elevation = -15
    viewer.cam.azimuth = 90

    print("Viewer opened! Robot is standing.")

    # Keep viewer open and simulate
    while viewer.is_running():
        step_start = time.time()

        mujoco.mj_step(mj_model, mj_data)
        viewer.sync()

        # Maintain real-time rate
        time_until_next_step = mj_model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

print("Viewer closed.")
