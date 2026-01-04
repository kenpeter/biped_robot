# Biped Workspace Memo

**Last Updated:** 2026-01-05

---

## Latest Progress & Findings

The root cause of the servo control failure was identified and fixed. The ROS `servo_driver.py` was sending commands in an incorrect format (`$A090#`). This has been corrected to use the board's documented ASCII protocol: `#<index>P<position>T<time>!`.

The `servo_driver.py` script was updated to:
1.  Use correct servo indices (0-14).
2.  Implement the correct protocol format.
3.  Convert joint angles to PWM pulse widths (500-2500μs).

The ROS 2 package was successfully rebuilt with the fix.

---

## Current Critical Issues

**Issue:** On servo board power cycle (off/on), servo 17 has been observed to move instead of servo 18, even when the command is for servo 18.

**Theory:** This suggests a potential state confusion or conflict in the servo controller's firmware upon reboot. It may not be correctly re-initializing or mapping the servo channels after a sudden power loss. Further investigation is needed to see if this is consistent or a one-off event.

---

## Hardware Configuration

-   **/dev/ttyUSB0**: 24-channel Servo Controller (CH340 serial)
-   **/dev/ttyUSB1**: SIPEED Meta Sense Lite Camera
-   **/dev/ttyUSB2**: SIPEED Meta Sense Lite Camera

**Servo Board Model:** 24路舵机控制板 (24-channel servo control board)
    -   Arduino-based open source controller
    -   16-channel PWM drive capability
    -   CH340 USB-serial converter
    -   Product listing: Chinese e-commerce (¥128 / ~$18 USD)
    -   Full name: "24路舵机控制板16路PWM驱动板机械臂开发板模块 arduino开源控制器"

**Product Image:** See `/home/jetson/Downloads/` screenshots

---

## Architecture Flow Diagram

1.  **ROS 2 Launch (`humanoid_hardware_node.py`)**: Starts the main hardware interface node.
2.  **`servo_driver` Node**:
    -   Subscribes to the `/joint_states` topic for angle commands.
    -   Converts the desired angles into the correct PWM format.
    -   Sends the command string to the servo board via `/dev/ttyUSB0`.
3.  **Servo Controller Board**:
    -   Receives the serial command.
    -   Parses the `#<index>P<position>T<time>!` string.
    -   Generates the corresponding PWM signal for the target servo.
4.  **Servos**: Move to the commanded position.

### Servo Protocol Details

**Format:** `#<index>P<position>T<time>!`

Where:
-   `index` = 0-254 (servo ID/channel)
-   `position` = 500-2500 (pulse width in microseconds)
-   `time` = 0-9999 (movement time in milliseconds)

**Examples:**
```
#000P1500T1000!  - Servo 0 to 1500μs (center) in 1000ms
#001P2000T0500!  - Servo 1 to 2000μs (right) in 500ms
#002P0500T1000!  - Servo 2 to 500μs (left) in 1000ms
```

**Multiple servos:**
```
{#000P1500T1000#001P1500T1000#002P1500T1000}
```

**Servo position reference:**
-   500μs = Full left/min
-   1500μs = Center (90°)
-   2500μs = Full right/max

---

## Code Simplification Rule

**Rule:** All hardware communication logic must be abstracted and isolated.

**Rationale:** The primary issue was that the protocol logic was tightly coupled within the ROS driver, making it difficult to debug. Future modifications should ensure that any new hardware protocol is implemented in a separate, self-contained Python module that can be tested independently of ROS.

**Example:**
-   **Bad:** Protocol string formatting mixed inside the ROS callback.
-   **Good:** A `protocol.py` file with a function like `format_servo_command(index, position, time)` that returns the final string. The ROS node should only call this function.

---

## How to Run the System

1.  **Open a terminal** and navigate to the workspace:
    ```bash
    cd /home/jetson/biped_ws
    ```

2.  **Source the ROS 2 environment** and the workspace setup file:
    ```bash
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ```

3.  **Launch the hardware node**:
    ```bash
    ros2 launch humanoid_hardware humanoid_hardware_node.py
    ```

---

## Debugging Servo Movement

The servos still aren't moving at 9600 baud. Let's troubleshoot this systematically:

### Next Steps to Debug

**Step 1: Try 115200 Baud (Most Likely Fix)**

Stop the current `servo_driver` (Ctrl+C), then restart at 115200 baud:

```bash
# In your terminal where servo_driver is running:
ros2 run humanoid_hardware servo_driver --ros-args -p baud_rate:=115200
```

Then in another terminal, run the test again:
```bash
source /opt/ros/humble/setup.bash
python3 /home/jetson/biped_ws/test_servo_fix.py
```

**Step 2: Direct Hardware Test (If Step 1 Fails)**

If 115200 doesn't work, let's bypass ROS entirely and scan all channels to find which servos are actually connected:

Stop the `servo_driver` first (Ctrl+C), then run:

```bash
# Try scanning at 115200 baud first (most common)
python3 /home/jetson/biped_ws/direct_servo_test.py 2
```

```bash
# If that doesn't work, try 9600 baud:
python3 /home/jetson/biped_ws/direct_servo_test.py 1
```

This will test channels 0-23 one by one. Watch the robot carefully and note which channel number makes each servo move.

**Step 3: Monitor Serial Traffic (Advanced Debug)**

If you want to see what commands are actually being sent:

```bash
# Terminal 1: Monitor serial port
python3 /home/jetson/biped_ws/monitor_serial_traffic.py 115200
```

```bash
# Terminal 2: Run servo driver (it will fail to open port, but that's OK)
# Or run the direct test
```

### Why Servos Might Not Move

Based on your memo mentioning "servo 17 moves instead of 18":

1.  Servos ARE working - they moved during power-up
2.  Channels might be different - servo 17/18 suggests they're on higher channels (not 0-14)
3.  Baud rate might be wrong - 115200 is more common than 9600

Let me know what happens with Step 1 (115200 baud test). That's the most likely fix!