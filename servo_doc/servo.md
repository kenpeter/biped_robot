# 24-Channel Servo Controller Board Manual

**Board Model:** 24路舵机控制板 (ZL-IS2)

---

## CRITICAL: Physical Servo Connections

**Connected Channels:** 4-11, 12-18 (15 servos total)

- **Channels 4-11:** 8 servos connected
- **Channels 12-18:** 7 servos connected

**Note:** The ROS driver must send commands to these channel numbers, NOT 0-14!

---

## Baud Rate Configuration

**Supported Rates:** 9600 (default) or 115200

**Set Baud Rate Command:**
```
$UBBS:1,9600!     - Set to 9600 baud
$UBBS:1,115200!   - Set to 115200 baud
```

Changes take effect after power cycle.

---

## Servo Control Protocol

### Single Servo Command
```
#<index>P<position>T<time>!
```

### Multiple Servo Command
```
{#<index>P<position>T<time>#<index>P<position>T<time>...}
```

### Parameter Ranges
- **index:** 000-254 (servo channel number)
- **position:** 0500-2500 (PWM pulse width in microseconds)
- **time:** 0000-9999 (movement duration in milliseconds)

### Examples
```
#004P1500T1000!   - Servo 4 to center (1500μs) in 1 second
#012P2000T0500!   - Servo 12 to 2000μs in 0.5 seconds
#018P0500T1000!   - Servo 18 to 500μs (0°) in 1 second
```

Multiple servos:
```
{#004P1500T1000#005P1500T1000#012P1500T1000}
```

---

## PWM to Angle Mapping

**Standard 0-180° Servos:**

| PWM Value | Servo Angle |
|-----------|-------------|
| 500μs     | 0°          |
| 1500μs    | 90° (center)|
| 2500μs    | 180°        |

**Formula:** `PWM = 500 + (angle / 180.0) * 2000`

---

## Other Commands

| Command | Description |
|---------|-------------|
| `$RST!` | Software reset |
| `$DGS:0!` | Query firmware version |
| `$DS:T!` | Stop all servos |
| `$DS:T×!` | Stop servo × |
| `#005PSCK+010!` | Adjust servo 5 bias by +10 (-500 to +500) |

---

## Hardware Details

- **Total Channels:** 24 (0-23)
- **Actually Connected:** Channels 4-11, 12-18
- **Communication:** USB serial via CH340 (/dev/ttyUSB0)
- **Default Baud:** 9600
- **Voltage:** 5-6V for servos

---

## Troubleshooting

### Why Servos Don't Move

1. **Wrong channel numbers** - Driver sending to 0-14, but servos are on 4-11, 12-18
2. **Wrong baud rate** - Try both 9600 and 115200
3. **No power** - Ensure servo board has external power supply
4. **Wrong protocol** - Must use `#indexPpwmTtime!` format with zero-padding

### Quick Test

Run direct hardware test to verify:
```bash
python3 /home/jetson/biped_ws/direct_servo_test.py 1  # 9600 baud
python3 /home/jetson/biped_ws/direct_servo_test.py 2  # 115200 baud
```

Watch which servos move and note their channel numbers!

---

## Documentation Source

Manual screenshots: `/home/jetson/biped_ws/servo_doc/*.png`
