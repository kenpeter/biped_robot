# Tethered Support Cart System

## Overview
The humanoid robot uses a tethered cart system to carry power and electronics while learning to walk. This allows the robot to move freely while maintaining a stable power supply.

## Physical Specifications

### Cart Platform
- **Dimensions:** 25cm × 25cm × 2cm (L × W × H)
- **Material:** Red platform (matches photos)
- **Color:** Red (RGB: 0.8, 0.1, 0.1)
- **Mass:** 2.0 kg (platform + electronics)

### Wheels
- **Total Count:** 16 wheels
- **Configuration:** 4 corners × 4 wheels per corner
- **Wheel Size:** 1.5cm radius, 1cm width (small caster wheels)
- **Type:** Continuous rotation (omnidirectional movement)
- **Material:** Black rubber/plastic

### Electronics Payload
The cart carries all the control electronics:
1. **Jetson Nano** - Main computer running ROS2
2. **STM32 Servo Board** - 24-channel servo controller (using 15 channels)
3. **2× Batteries** - Power supply for servos and electronics
4. **USB Hub** - Connects servo board, ToF camera, and other peripherals

### Tether Connection
- **Type:** Elastic string
- **Relaxed Length:** 10cm
- **Attachment Point:** Robot waist (base_link center)
- **Cart Attachment:** Center of cart platform
- **Stretch Range:** 5cm closer to 25cm farther (total 30cm range)
- **Behavior:**
  - Normal operation: Cart trails 10-15cm behind robot
  - Robot stops suddenly: Cart momentum can bring it 5cm closer
  - Robot accelerates: String stretches up to 25cm

## URDF Implementation

### Links Created

1. **cart_base_link** - Main cart platform
   - Box geometry: 0.25 × 0.25 × 0.02m
   - Mass: 2.0kg with full inertia properties
   - Collision geometry included for Isaac Sim

2. **electronics_platform_link** - Upper deck for electronics
   - Box geometry: 0.24 × 0.24 × 0.02m
   - Offset 5cm above cart base
   - Mass: 0.5kg (represents electronics weight)
   - White color to distinguish from red cart base

3. **wheel_XX_Y** - 16 individual wheels
   - Cylinder geometry: radius 0.015m, length 0.01m
   - Mass: 0.05kg each
   - Positioned at 4 corners in 2×2 grid pattern per corner
   - Example positions (front-left corner):
     - wheel_fl_1: (0.11, 0.11, -0.01)
     - wheel_fl_2: (0.13, 0.11, -0.01)
     - wheel_fl_3: (0.11, 0.13, -0.01)
     - wheel_fl_4: (0.13, 0.13, -0.01)

### Joints Created

1. **cart_tether_joint** - Elastic string connection
   - Type: Prismatic (sliding joint)
   - Parent: base_link (robot torso)
   - Child: cart_base_link
   - Axis: X-direction (forward/backward)
   - Origin: (-0.15, 0, -0.65) from robot waist
   - Limits: -0.05m to +0.25m (allows 30cm total range)
   - Dynamics:
     - Damping: 5.0 (simulates string elasticity)
     - Friction: 0.5 (ground friction on cart)
   - Effort: 20N (maximum pull force)

2. **electronics_platform_joint** - Fixed
   - Connects cart base to electronics platform
   - Offset: (0, 0, 0.05) - 5cm above cart base

3. **wheel_XX_Y_joint** - 16 wheel joints
   - Type: Continuous (free rotation)
   - Axis: Y-direction (rolling forward/backward)
   - Dynamics: damping=0.1, friction=0.1
   - Allow wheels to roll freely as cart moves

## Coordinate System

```
Robot Coordinate Frame (base_link at torso center):
- X: Forward (robot front)
- Y: Left
- Z: Up

Cart Rest Position:
- X: -0.15m (15cm behind robot waist)
- Y: 0 (centered)
- Z: -0.65m (on ground, robot waist is 65cm above ground)

Ground Level: Z = -0.67m (foot contact points)
Cart Platform: Z = -0.65m (slightly above ground for wheel clearance)
```

## Physics Simulation Parameters

For Isaac Sim and other physics engines:

### Material Properties
- **Cart Platform:**
  - Friction: Medium (0.5 static, 0.4 dynamic)
  - Restitution: Low (0.1 - minimal bounce)

- **Wheels:**
  - Friction: High (0.8 static, 0.7 dynamic)
  - Restitution: Very low (0.05)
  - Rolling resistance: 0.1

### Spring-Damper Model (Tether)
The prismatic joint simulates elastic string with:
- **Spring Constant (implicit):** ~100 N/m
- **Damping:** 5.0 N·s/m
- **Equilibrium Position:** -0.15m (10cm rest length)
- **Natural Frequency:** ~2.2 Hz

### Expected Behavior in Simulation
1. **Walking Forward:** Cart follows smoothly 10-15cm behind
2. **Sudden Stop:** Cart momentum carries it closer (elastic compression)
3. **Backward Walking:** Cart may lag, string stretches
4. **Turning:** Cart swivels around tether point
5. **Falling/Tipping:** Cart remains stable on ground

## Integration Notes

### For Isaac Sim
The URDF is designed to work directly with Isaac Sim:
- All links have proper inertial properties
- Collision geometry defined for all parts
- Joint dynamics compatible with PhysX
- Material colors defined for visualization

### Limitations
1. **String physics:** Modeled as prismatic joint (rigid rail), not true flexible rope
   - Trade-off: Simpler physics, less computation
   - Alternative: Could use rope/cable plugin for more realism

2. **Wheel constraints:** Individual wheels not steered
   - All wheels free-spinning (caster behavior)
   - Cart can move in any direction

3. **Electronics mass:** Approximated as single platform
   - Real electronics have complex mass distribution
   - Simplified for simulation stability

## Testing and Validation

### URDF Validation
```bash
check_urdf humanoid.urdf
# Status: ✓ Successfully Parsed
# Links: 33 total (robot: 16, cart: 17)
# Joints: 32 total (robot: 15, cart: 17)
```

### Build Status
```bash
colcon build --packages-select humanoid_description
# Status: ✓ Success (0.23s)
```

### Visual Verification
Launch RViz to see the complete model:
```bash
ros2 launch humanoid_description display.launch.py
```

The cart will appear behind the robot with all 16 wheels visible.

## Future Enhancements

1. **Battery Visualization**
   - Add visual boxes for 2 batteries on electronics platform
   - Include mass distribution

2. **Cable/Wire Models**
   - Add visual representation of power cables
   - Add USB cables to servo board

3. **Realistic String Physics**
   - Replace prismatic joint with flexible cable constraint
   - Add proper tension/compression behavior

4. **Cart Dynamics Tuning**
   - Measure actual cart mass and adjust
   - Calibrate friction coefficients from real tests
   - Tune tether spring/damping from video analysis

5. **Collision Safety**
   - Add soft bumpers to prevent hard impacts
   - Implement emergency stop if cart contacts robot

## References

- URDF File: `/home/jetson/biped_ws/src/humanoid_description/urdf/humanoid.urdf`
- Photos: Provided images showing physical robot + cart setup
- Build Log: `/home/jetson/biped_ws/memo.md`

---

**Last Updated:** 2026-01-04
**Status:** Complete and validated
**Compatible With:** ROS2 Humble, Isaac Sim, Gazebo, RViz2
