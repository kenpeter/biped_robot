# Servo Board Debugging - Status

## System Profile
- **Mainboard:** Jetson Orin Nano (JetPack 6, L4T R36.4.3)
- **Primary Controller:** Yahboom Rosmaster Board (USB connected)
- **Port:** `/dev/ttyUSB0`
- **Driver:** `Rosmaster_Lib` (Binary Protocol `0xFF 0xFC`)

## Resolution Strategy
**Issue:** The direct UART connection between the Jetson GPIO (Pins 8/10) and the Hiwonder LSC-24 board failed because the Orin Nano's UART pins (`/dev/ttyAMA0` or `/dev/ttyTHS1`) are locked by the system kernel or device tree and cannot be easily enabled via `jetson-io`.

**Solution:** Bypass the Hiwonder board and plug servos directly into the **Yahboom Rosmaster Board**.
- The Rosmaster board is already detected and working on `/dev/ttyUSB0`.
- It has 4 PWM Servo ports (S1, S2, S3, S4).
- This eliminates the need for level shifters, custom device trees, or UART debugging.

## Hardware Configuration (Final)
- **Head Servo:** Plug directly into Rosmaster Board **Port S1**. (Verified)
- **Hiwonder LSC-24 (17-Servo Body):** Connected via CP2102 USB-to-TTL adapter on `/dev/ttyUSB1`.
    - **Status (2026-01-18):** **VERIFIED WORKING**.
    - **Issue:** Communication timeout (0 bytes received).
    - **Fix:** Swapped TX/RX wires.
        - **Adapter TX** -> **Board RX**
        - **Adapter RX** -> **Board TX**
    - **Voltage:** 8.08V detected.
    - **Movement:** Confirmed via port sweep and precision tests.
    - **Troubleshooting Log:**
        - **Symptom 1:** Head Servo didn't move on Pin 0. -> **Fix:** Replaced faulty servo cable.
        - **Symptom 2:** Head moved "too little" compared to arm. -> **Cause:** 5 degrees looks small on a head; verified with 20-degree test.
        - **Status:** **ALL SYSTEMS GO.** Head on Pin 0, Body on Pins 1-19.

    - **Final Configuration:**
        - **Head:** Pin 0 (Verified Working).
        - **Right Body:** Pins 1-7.
        - **Left Body:** Pins 12-19.

## Verified Scripts
- `test_head_body_sync.py`: Synchronized test for Head and Arm.
- `test_head_big_move.py`: Demonstrates full range of motion for Head.
- `reset_all_servos.py`: Resets robot to neutral pose.

## Abandoned Paths
- **Direct GPIO UART:** Loopback tests failed consistently on `/dev/ttyAMA0` and `/dev/ttyTHS1`.
- **Hiwonder LSC-24 (Direct):** Could not establish communication via Jetson GPIO due to the UART lock.
- **Hiwonder Manual Method:** Manual assumes Jetson Nano (original), not Orin Nano. The `/dev/ttyTHS1` port that works on Nano is locked on Orin Nano.

## Robot Model (USD for Isaac Sim)
**File:** `/home/jetson/biped_ws/src/humanoid_description/usd/humanoid.usda`

**15 Servos Total:**

| Part | Joint | Motion | Axis |
|------|-------|--------|------|
| HEAD | head_joint | left/right | Z (yaw) |
| L_ARM | l_shoulder_pitch | forward/backward | Y |
| L_ARM | l_shoulder_roll | close/away body | X |
| L_ARM | l_forearm_roll | close/away body | X |
| R_ARM | r_shoulder_pitch | forward/backward | Y |
| R_ARM | r_shoulder_roll | close/away body | X |
| R_ARM | r_forearm_roll | close/away body | X |
| L_LEG | l_hip_roll | close/away body | X |
| L_LEG | l_knee_pitch | forward/backward | Y |
| L_LEG | l_ankle_pitch | forward/backward | Y |
| L_LEG | l_foot_roll | close/away body | X |
| R_LEG | r_hip_roll | close/away body | X |
| R_LEG | r_knee_pitch | forward/backward | Y |
| R_LEG | r_ankle_pitch | forward/backward | Y |
| R_LEG | r_foot_roll | close/away body | X |

## Reference Documents
- `/home/jetson/Downloads/lsc_24_dev/2. LSC-24 Controller Secondary Development/05 Jeson Development/01 Developer Guide/Jetson Development.pdf`
- `/home/jetson/Downloads/lsc_24_dev/2. LSC-24 Controller Secondary Development/LSC Series Controller Communication Protocol V1.1.pdf`
