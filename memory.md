# Biped Robot Project Memory

> **Note:** The Jetson directory structure is totally different from the Desktop environment.

---

## 1. DESKTOP CONTEXT (Simulation)

### Environment Info
- **Workspace:** `~/work/biped_ws`
- **Model Info:**
  - **Source:** `models/humanoid.glb`
  - **Blender File:** `models/humanoid_robot.blend`
- **Format:** GLB (Binary glTF) / USD (Universal Scene Description)
- **Tools:** Blender, Isaac Sim

### Simulation Workflow
1. Load USD/GLB model in Isaac Sim.
2. Train RL policy (walking, balance) with thermal camera input.
3. Export trained policy to ONNX/TensorRT.

---

## 2. JETSON CONTEXT (Hardware)

### System Profile
- **Device:** Jetson Orin Nano
- **OS:** JetPack 6 (L4T R36.4.3)
- **Workspace:** `/home/jetson/work/biped_ws`

### Hardware Configuration (Verified)
- **Control Board:** Yahboom Rosmaster
- **Connection:** USB (`/dev/ttyUSB0`)
- **Servos:**
    - **Head:** Port S1 (Rosmaster direct)
    - **Body:** Ports 1-19 (via LSC-24 / CP2102 on `/dev/ttyUSB1` or Rosmaster expansion)

### Pinout Mapping
| Part | Joint | Connection |
|------|-------|------------|
| HEAD | head_joint | Rosmaster S1 |
| HEAD | thermal_camera | USB |
| BODY | (Various) | LSC-24 Board |

### Critical Notes
- **UART Lock:** `/dev/ttyTHS1` on Orin Nano is locked by the kernel. Do not use for servo control.
- **Power:** Ensure LSC-24 switch is ON (Blue LED).
- **Wiring:** CP2102 Adapter RX -> Board TX, Adapter TX -> Board RX.

### Verified Scripts
- `verify_hardware.py`: Main diagnostic tool.
- `test_rosmaster_s1.py`: Simple head test.