# Biped Workspace Memo

**Last Updated:** 2026-01-05

---

## Latest Progress & Findings

The `servo_driver.py` script has been significantly updated to integrate with `ZideConfig.ini` for dynamic servo configuration. This includes:
1.  Loading `ZideConfig.ini` using `configparser`.
2.  Dynamically populating the `joint_to_servo` mapping and `servo_metadata` (including `pmin`, `pmax`, `bias`) from the configuration file.
3.  Removing the hardcoded `joint_to_servo` map.
4.  Updating the angle conversion logic in `radians_to_servo_angle` and `send_servo_command` to utilize the `pmin`, `pmax`, and `bias` values specific to each servo as defined in `ZideConfig.ini`.

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

## Latest Update (2026-01-05 21:17)

### HID vs SERIAL INVESTIGATION ✓

**Context:** User noticed PC software (Zide) shows "HID Connected" and suggested the Jetson should use HID instead of serial.

**Investigation Results:**

1. **USB Device Enumeration:**
   - `lsusb` shows: `1a86:7523 QinHeng Electronics CH340 serial converter`
   - Device appears at `/dev/ttyUSB0` (serial device)
   - Kernel module: `ch34x` (USB-to-serial driver)

2. **HID Device Check:**
   - Installed `hidapi` Python library (v0.15.0)
   - Enumerated all HID devices on system
   - Found: Razer mouse (1532:0083), Compx keyboard (25a7:fa61)
   - **NOT FOUND:** CH340 (1a86:7523) as HID device

3. **Conclusion:**
   - ✓ The CH340 is **definitely a serial device** on Linux
   - ✗ It does NOT appear as an HID device on the Jetson
   - The PC software "HID Connected" label is misleading or Windows-specific

**Why PC Software Works but Python Doesn't:**

The PC software (Zide) successfully controls servos with the SAME hardware, but Python scripts don't work. Since both should be using serial communication, the difference must be:
- Hidden initialization sequence PC sends
- Timing or handshaking requirement
- Subtle protocol difference we haven't discovered

**New Test Scripts Created:**

1. `/home/jetson/biped_ws/test_with_init.py`
   - Tests 9 different initialization sequences
   - Tries commands like `$RST!`, `$SEN:1!`, `$PON!`
   - Tests if board needs "unlock" command before servos work

2. `/home/jetson/biped_ws/test_hid_access.py`
   - Attempts to open CH340 as HID device
   - Tries different HID report formats
   - Confirms HID access is not possible

**Critical Next Action:**

Use the spy script to capture what PC software ACTUALLY sends:
```bash
python3 /home/jetson/biped_ws/spy_on_pc.py
```
Then open Zide on PC and move one servo. This will show the exact commands and timing the PC uses.

---

## Previous Update (2026-01-05 16:50)

### CODE FIXES APPLIED ✓

1. **Fixed critical bug in servo_driver.py** (line 171)
   - `servo_index` was used before being defined
   - Moved `servo_index = self.joint_to_servo[joint_name]` to line 171 (before usage at line 174)
   - File: `/home/jetson/biped_ws/src/humanoid_hardware/humanoid_hardware/servo_driver.py`

2. **Updated baud rate**
   - Changed default from 9600 to 115200 (line 22)
   - Added `import time` (line 13)

3. **Added DTR/RTS hardware control signals** (lines 70-73)
   ```python
   self.serial_port.dtr = False
   self.serial_port.rts = False
   time.sleep(0.1)
   ```

4. **Updated protocol format** (line 152)
   - Changed from `#indexPpwmTtime!` to `{#indexPpwmTtime!}` (with braces)
   - Added `ser.flush()` after every write (line 156)

5. **Package rebuilt successfully**
   ```bash
   colcon build --packages-select humanoid_hardware
   ```

---

### SERVO CONFIGURATION

From `ZideConfig.ini`:
- **Enabled servos**: 0-7, 12-18 (15 servos total)
- **Disabled servos**: 8-11, 19-23
- **PWM range**: 500-2500μs for all servos
- **Default position**: 1500μs (center)

From `servo_doc/servo.md`:
- **Board model**: ZL-IS2 (24-channel controller)
- **Physical connections**: Channels 4-11, 12-18
- **Baud rates supported**: 9600 (default), 115200
- **Protocol**: `#<index>P<position>T<time>!` or `{#...#...}`

---

### WHAT WE'VE TESTED ✓

**Baud Rates:**
- ✓ 9600 baud - No movement
- ✓ 115200 baud - No movement

**DTR/RTS Signals:**
- ✓ DTR=OFF, RTS=OFF - No movement
- ✓ DTR=ON, RTS=OFF - No movement
- ✓ DTR=OFF, RTS=ON - No movement
- ✓ DTR=ON, RTS=ON - No movement

**Protocol Formats:**
- ✓ `#000P1500T0500!` (no braces) - No movement
- ✓ `{#000P1500T0500!}` (with braces) - No movement
- ✓ `{G0000#000P1500T0500!}` (group format) - No movement

**Servo Channels Tested:**
- ✓ Channel 0 (full range 500→2500) - No movement
- ✓ Channel 4 (full range 500→2500) - No movement
- ✓ Channel 5 (full range 500→2500) - No movement
- ✓ Channel 12 (full range 500→2500) - No movement
- ✓ Channel 17 (full range 500→2500) - No movement
- ✓ Channel 18 (full range 500→2500) - No movement

**Line Endings:**
- ✓ No line ending - No movement
- ✓ CR only (\r) - No movement
- ✓ LF only (\n) - No movement
- ✓ CR+LF (\r\n) - No movement

**Hardware Verification:**
- ✓ Serial port `/dev/ttyUSB0` opens successfully
- ✓ Board responds with sensor/IMU data (1312+ bytes of binary data)
- ✓ Power LED confirmed ON
- ✓ Servo power supply confirmed connected
- ✓ Servos confirmed physically connected

---

### CURRENT STATUS ⚠

**What Works:**
- ✓ Serial communication (port opens, data transmits)
- ✓ Board is powered and responding (sends IMU sensor data)
- ✓ PC software (Zide) DOES control servos successfully
- ✓ Commands are formatted correctly and sent

**What Doesn't Work:**
- ✗ Python scripts cannot make servos move
- ✗ ROS servo_driver cannot make servos move

**The Mystery:**
- PC software works with the SAME hardware
- Python sends identical commands but no movement
- Board receives commands (no errors) but servos don't respond

---

### CRITICAL NEXT STEPS (WHEN YOU COME BACK)

**Option 1 - Identify Working Servo (5 minutes):**
1. Open Zide PC software
2. Click green Connect button
3. Test each servo box one by one:
   - Click S00, move slider → Did it move? Note Yes/No
   - Click S04, move slider → Did it move? Note Yes/No
   - Click S05, move slider → Did it move? Note Yes/No
   - Click S12, move slider → Did it move? Note Yes/No
   - Click S17, move slider → Did it move? Note Yes/No
   - Click S18, move slider → Did it move? Note Yes/No
4. **Write down which servo number(s) actually moved**
5. Run: `python3 /home/jetson/biped_ws/test_servo_auto.py` with that servo number

**Option 2 - Capture PC Software Commands:**
```bash
# Terminal 1 - Run spy (BEFORE opening PC software):
python3 /home/jetson/biped_ws/spy_on_pc.py

# Then open PC software, click Connect, move ONE servo
# The spy will show EXACTLY what commands PC sends
```

**Option 3 - Check for Initialization Command:**
- PC software might send a special command on Connect (before servo commands)
- Commands to try:
  - `$RST!` (software reset)
  - `$DGS:0!` (query firmware version)
  - Initialization sequence from ZideConfig line 8

---

### POSSIBLE CAUSES

1. **PC software sends initialization command we're missing**
   - Special unlock/enable command
   - Baud rate change command (`$UBBS:1,115200!`)
   - Reset command (`$RST!`)

2. **Wrong servo channels**
   - Documentation says 4-11, 12-18
   - Config says 0-7, 12-18
   - Need to confirm which channels are PHYSICALLY connected

3. **Timing issue**
   - PC software might wait between commands
   - Might need delay after opening port

4. **Hidden protocol requirement**
   - Special character sequence
   - Checksum we're not calculating
   - Different packet structure

---

### TEST SCRIPTS AVAILABLE

All in `/home/jetson/biped_ws/`:

**Diagnostic/Spy Scripts:**
- `spy_on_pc.py` - **[RECOMMENDED]** Monitor what PC software sends
- `test_with_init.py` - **[NEW]** Test different initialization sequences
- `test_hid_access.py` - **[NEW]** Attempt HID access (confirms not possible)
- `find_the_problem.py` - Comprehensive diagnostic with questions
- `monitor_serial_traffic.py` - Monitor serial port traffic

**Direct Servo Test Scripts:**
- `simple_test.py` - Simple servo 0 full range test
- `test_correct_channels.py` - Tests channels 4-18 with full range
- `find_working_servo.py` - Test all 24 channels sequentially
- `test_large_movement.py` - Obvious full range movements
- `final_test.py` - Tests both baud rates, multiple servos
- `check_enabled_servos.py` - Show enabled servos from config

---

### FILES MODIFIED

1. `/home/jetson/biped_ws/src/humanoid_hardware/humanoid_hardware/servo_driver.py`
   - Lines 13, 22, 70-76, 152, 156 (see fixes above)

2. `/home/jetson/biped_ws/memo.md`
   - This file (updated with HID investigation results)

3. **NEW FILES CREATED (2026-01-05 21:17):**
   - `/home/jetson/biped_ws/test_with_init.py` - Tests initialization sequences
   - `/home/jetson/biped_ws/test_hid_access.py` - Tests HID access method

4. **SYSTEM CHANGES:**
   - Installed `hidapi` Python library (v0.15.0) via pip3

---

### WHEN YOU RETURN

**RECOMMENDED APPROACH - Spy on PC Software:**
This is the ONLY way to know definitively what commands work:

1. On Jetson terminal, run:
   ```bash
   python3 /home/jetson/biped_ws/spy_on_pc.py
   ```

2. On your PC:
   - Open Zide software
   - Click the green "Connect" button
   - Move ONE servo slider

3. Check Jetson terminal - it will show:
   - What commands PC sends on Connect (initialization?)
   - Exact servo command format and timing
   - Any responses from the board

4. Share the spy output with Claude

**ALTERNATIVE 1 - Try Initialization Sequences:**
Test if board needs special init command before servos work:
```bash
python3 /home/jetson/biped_ws/test_with_init.py
```
Watch robot carefully for any movement.

**ALTERNATIVE 2 - Rule Out HID Access:**
Confirm HID access doesn't work (expected to fail):
```bash
python3 /home/jetson/biped_ws/test_hid_access.py
```

**ALTERNATIVE 3 - Identify Working Servo:**
If you know which servo number moves in PC software:
1. Note the servo number from Zide (e.g., S05, S12, S17)
2. Tell Claude: "Servo XX moves in PC software"
3. We'll test that exact servo with Python

---

## Debugging Servo Movement (OLD NOTES)

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