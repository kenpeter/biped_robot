# Biped Robot - ROS2 Workspace

> **Note:** The Jetson directory structure is totally different from the Desktop environment.
> - **Desktop:** Used for Isaac Sim, Training, and Development.
> - **Jetson:** Used for Deployment, Hardware Control, and Inference.

## Project Workflow

```
┌─────────────────────────────────────────────────────────────────┐
│  1. DESKTOP (Isaac Sim)         2. JETSON (Orin Nano)           │
│  ─────────────────────          ────────────────────────────    │
│  humanoid_description/    →     humanoid_hardware/              │
│  - USD robot model              - ROS 2 servo driver            │
│  - Training Environment         - Real robot control            │
└─────────────────────────────────────────────────────────────────┘
```

---

# SECTION A: DESKTOP (Simulation)

**Goal:** Run physics simulation and train the policy.
**Working Directory:** `~/work/biped_ws`

### 1. Visualization & Model Editing
To view the full 3D robot model (geometry and materials) or edit the structure:

```bash
# Navigate to workspace
cd ~/work/biped_ws

# View in Blender
blender --python models/create_robot.py
```

### 2. Run Isaac Sim
Launch Isaac Sim and load the `humanoid.glb` model.

```bash
# 1. Navigate to workspace
cd ~/work/biped_ws

# 2. Run Isaac Sim (Check your specific install path)
# Common path example:
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh --file models/humanoid.glb
```

### 3. Massive Multi-Robot RL Training (BLOCKED)

An attempt was made to set up the Isaac Lab `DirectRLEnv` for multi-robot reinforcement learning. However, this is currently blocked by a `KeyError: 'robot'` during articulation registration, indicating a configuration issue with custom USD assets in `humanoid_direct_env.py`.

Please refer to `MEMORY.md` for a detailed log of the attempts and the current status of this issue.

```bash
# To attempt running the currently blocked training script:
# (Note: This will likely result in a KeyError)
# cd ~/work/biped_robot
# ./run_isaac.sh train_humanoid.py
```

**What to Expect (if the issue is resolved):**
-   Isaac Sim will launch and display 4096 instances of your humanoid robot.
-   A training loop will start, and you will see messages about episodes, rewards, and policy updates in the console.
-   Initially, the robots will flail or fall, but over time, they should learn to stand and perform desired behaviors.

---

# SECTION B: JETSON (Robot)

**Goal:** Control the physical hardware using the trained policy or test scripts.
**Working Directory:** `/home/jetson/work/biped_ws`

### 1. Hardware Configuration
- **Control Board:** Yahboom Rosmaster (Green/Black Board)
- **Port:** `/dev/ttyUSB0` (Direct USB connection)
- **Servo Config:**
  - **Port S1:** Head/Test Servo
  - **Pins 1-19:** Body Servos (via CP2102 adapter on `/dev/ttyUSB1` if applicable, or direct)
  
*Note: See `memory.md` for detailed pinouts.*

### 2. Quick Start (Real Robot)
```bash
cd /home/jetson/work/biped_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch the hardware driver
ros2 launch humanoid_hardware robot_control.launch.py
```

### 3. Test Scripts
**Method 1: Rosmaster Direct (Head/S1)**
```bash
sudo python3 /home/jetson/test_rosmaster_s1.py
```

**Method 2: Full Hardware Verification**
Checks battery, servo connection, and centers all servos.
```bash
python3 verify_hardware.py
```

### 4. Build Workspace
On the Jetson, you must build the packages to generate the ROS 2 interfaces.
```bash
colcon build --packages-select humanoid_hardware
source install/setup.bash
```

---

## Troubleshooting (Jetson)

**Hardware Issues:**
- **No Board Response:** Check if `/dev/ttyUSB0` or `/dev/ttyUSB1` exists.
- **Servos Not Moving:** Check battery voltage (should be > 7.4V) and switch on the board.
- **UART/Serial Errors:** The Orin Nano has locked UART ports (`/dev/ttyTHS1`). Use USB adapters or the Rosmaster USB port.

---

## Setting Up for Isaac Sim (Desktop)

To set up and train the biped robot in Isaac Sim, use the provided `run_isaac.sh` wrapper script. This script manages the necessary conda environment activation and interfaces with IsaacLab.

### 1. Convert GLB to Articulated USD

First, convert your `humanoid.glb` model into an articulated USD format. This script has been updated to:
-   **Add Structural Offsets:** Expands the robot parts from the origin so they form a proper humanoid shape.
-   **Fix Joint Hierarchy:** Correctly links parents and children for the kinematic chain.
-   **Apply Physics:** Adds mass, rigid body, and collision APIs.

```bash
./run_isaac.sh setup_isaac_sim_robot.py
```
*Output: `models/humanoid_articulated.usda`*

### 2. Run the Training Environment

Once the articulated USD is created, you can launch the Isaac Sim training environment. This will open the Isaac Sim application and run a demo loop with a PD controller.

```bash
./run_isaac.sh isaac_sim_training_env.py
```

**What to Expect:**
-   The script will print progress messages as it initializes (5 steps total).
-   Isaac Sim window will open showing the robot in a scene with a ground plane.
-   The robot spawns 0.45m above the ground to prevent initial clipping.
-   The orientation is automatically corrected (Blender Y-up → Isaac Sim Z-up).
-   The demo runs for 2000 simulation steps with a PD controller trying to maintain balance.
-   **Note:** The robot will likely **fall down** after a few seconds. This is expected! The simple PD controller is not sufficient for bipedal balancing. The goal of RL training (next steps) is to learn a policy that can maintain balance.

**Recent Fixes (2026-01-18):**
-   ✅ Fixed 90-degree rotation issue by applying -90° X-axis rotation at spawn time.
-   ✅ Added detailed logging and error handling for better debugging.
-   ✅ Improved initialization sequence to ensure USD references load correctly.
-   ✅ Output is now unbuffered so you can see progress in real-time.

**Troubleshooting:**
-   If the robot appears sideways or upside down, check the orientation quaternion in `isaac_sim_training_env.py:45`.
-   If the script hangs, check the Isaac Sim log file at `~/.local/share/ov/pkg/isaac_sim-*/logs/`.
-   If the articulation fails to load, verify the USD file structure by opening `models/humanoid_articulated.usda` in a text editor.
