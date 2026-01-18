# Biped Robot - ROS2 Workspace

## Project Workflow

```
┌─────────────────────────────────────────────────────────────────┐
│  1. TRAIN (Isaac Sim)           2. DEPLOY (Jetson Orin Nano)    │
│  ─────────────────────          ────────────────────────────    │
│  humanoid_description/    →     humanoid_hardware/              │
│  - USD robot model              - ROS 2 servo driver            │
│  - Thermal camera sensor        - Real robot control            │
│  - RL training environment      - Trained policy execution      │
└─────────────────────────────────────────────────────────────────┘
```

### Folder Structure
| Folder | Purpose |
|--------|---------|
| `humanoid_description` | Robot model (USD) for Isaac Sim training |
| `humanoid_hardware` | ROS 2 driver to control real robot on Jetson |

### Workflow Steps
1. **Isaac Sim**: Load `humanoid_description/usd/humanoid.usda`
2. **Train**: RL policy using Isaac Sim + thermal camera
3. **Export**: Trained model (ONNX/TensorRT)
4. **Deploy**: Run on Jetson with `humanoid_hardware` ROS 2 nodes

## Quick Start (Real Robot)

```bash
cd /home/jetson/biped_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch humanoid_hardware robot_control.launch.py
```

## Hardware Configuration (Updated)

**Critical Change:** The direct Jetson GPIO connection to the Hiwonder board was unreliable due to UART port mapping issues on the Orin Nano.

**Current Setup:**
- **Control Board:** Yahboom Rosmaster (Green/Black Board)
- **Connection:** USB Cable to Jetson (`/dev/ttyUSB0`)
- **Servo Connection:** Servos are plugged **directly into the Rosmaster Board**.
  - **Port S1:** Head Servo (or Test Servo)
  - **Pinout:** Yellow=Signal, Red=VCC, Black=GND
- **Thermal Camera (FLIR):** Mounted on head (Servo 0), connected to Jetson via USB.
- **Driver:** Uses `Rosmaster_Lib` (not raw LSC commands).

## Test Scripts

### New Method (Rosmaster Direct)
Use this to test servos plugged into the Rosmaster board (S1, S2, etc):
```bash
sudo python3 /home/jetson/test_rosmaster_s1.py
```

### Hiwonder LSC-24 (17-Servo Body)
**Driver:** Custom ROS 2 node (`servo_driver.py`) communicating via USB-to-TTL.

**Hardware Setup (Critical):**
1.  Use a **CP2102** or **CH340** USB-to-TTL adapter.
2.  Connect to Jetson USB port (usually appears as `/dev/ttyUSB1`).
3.  **Wiring:**
    -   **Adapter TX** $\rightarrow$ **Board RX**
    -   **Adapter RX** $\rightarrow$ **Board TX**
    -   **Adapter GND** $\rightarrow$ **Board GND**
4.  **Power:** Ensure LSC-24 board switch is ON (Blue LED active).

**Troubleshooting:**
-   **No Board Response (Timeout):**
    -   Swap **TX** and **RX** wires on the Hiwonder board.
    -   Check if the **Blue LED** on the board is ON.
-   **Specific Servo/Port Not Moving:**
    -   **Reseat the Plug:** If a port seems "dead", unplug and replug the servo firmly. Use `test_sweep_ports.py` to confirm.
    -   **Wrong Port:** You might be commanding Port 0 but plugged into Port 1.
    -   **Orientation:** Yellow wire must face the **Inside** (towards the chip).

The Hiwonder manual targets original Jetson Nano, not Orin Nano. On Orin Nano (JetPack 6), `/dev/ttyTHS1` is locked by the kernel and cannot be easily enabled via `jetson-io`.

**Solution:** Use CP2102 USB-to-TTL adapter
```
CP2102    LSC-24
───────────────
TX    →   RX
RX    →   TX
GND   →   GND
VCC   →   (don't connect)
```

## Visualization

To view the full 3D robot model (geometry and materials) directly on the Jetson without external tools:

```bash
blender --python biped_ws/create_robot.py
```
This script rebuilds the robot from scratch inside Blender for immediate inspection.

## Hardware Verification
To verify battery voltage and servo movement (Head + Body):
```bash
python3 verify_hardware.py
```
This script checks the connection, reads voltage, centers all servos, and performs a small wiggle test.

## Build

```bash
colcon build --packages-select humanoid_hardware
```

## Setting Up for Isaac Sim

To set up and train the biped robot in Isaac Sim, use the provided `run_isaac.sh` wrapper script. This script manages the necessary conda environment activation and interfaces with IsaacLab.

### 1. Convert GLB to Articulated USD

First, convert your `humanoid.glb` model into an articulated USD format. This script has been updated to:
-   **Add Structural Offsets:** Expands the robot parts from the origin so they form a proper humanoid shape.
-   **Fix Joint Hierarchy:** Correctly links parents and children for the kinematic chain.
-   **Apply Physics:** Adds mass, rigid body, and collision APIs.

```bash
./run_isaac.sh setup_isaac_sim_robot.py
```
*Output: `src/humanoid_description/usd/humanoid_articulated.usda`*

### 2. Run the Training Environment

Once the articulated USD is created, you can launch the Isaac Sim training environment. This will open the Isaac Sim application and run a demo loop.

```bash
./run_isaac.sh isaac_sim_training_env.py
```

**Current Status:**
-   The robot spawns 0.45m above the ground to prevent initial clipping.
-   It is a "floating base" robot (simulating a real robot not attached to a pole).
-   **Note:** The robot will likely **fall down** immediately. This is expected behavior! It is currently running a passive "standing" demo with a simple PD controller, which is not sufficient for bipedal balancing without a trained RL policy. The goal of the RL training (next steps) is to learn the policy to keep it upright.

## Project Workflow
