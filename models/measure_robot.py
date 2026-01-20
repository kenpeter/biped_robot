"""Measure physical robot joint ranges - Run this on Jetson connected to servos"""
import serial
import time
import json

# Connect to Hiwonder board
PORT = '/dev/ttyUSB1'
BAUD = 9600

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"✓ Connected to {PORT}")
except:
    print(f"✗ Cannot connect to {PORT}")
    print("This script must run on the Jetson with servos connected!")
    exit(1)

def send_servo_command(servo_id, position, time_ms=500):
    """Send Hiwonder protocol command to move servo"""
    cmd = [
        0x55, 0x55,           # Header
        0x08,                 # Length
        0x03,                 # Command (move servo)
        0x01,                 # Count (1 servo)
        time_ms & 0xFF,       # Time low byte
        (time_ms >> 8) & 0xFF,# Time high byte
        servo_id,             # Servo ID
        position & 0xFF,      # Position low byte
        (position >> 8) & 0xFF# Position high byte
    ]
    ser.write(bytes(cmd))
    time.sleep(time_ms / 1000.0 + 0.1)

# Servo definitions (from hardware specs)
SERVOS = {
    0: 'head_joint',
    1: 'l_shoulder_pitch',
    2: 'l_shoulder_roll',
    3: 'l_forearm_roll',
    4: 'l_hip_roll',
    5: 'l_knee_pitch',
    6: 'l_ankle_pitch',
    7: 'l_foot_roll',
    12: 'r_shoulder_pitch',
    13: 'r_shoulder_roll',
    14: 'r_forearm_roll',
    15: 'r_hip_roll',
    16: 'r_knee_pitch',
    17: 'r_ankle_pitch',
    18: 'r_foot_roll',
}

print("\n=== ROBOT JOINT MEASUREMENT TOOL ===\n")
print("This will move each servo through its range to find limits.")
print("⚠️  WARNING: Make sure robot is supported and won't fall!\n")
input("Press ENTER when ready, or Ctrl+C to abort...")

measurements = {}

for servo_id, joint_name in SERVOS.items():
    print(f"\n--- Testing {joint_name} (servo {servo_id}) ---")

    # Move to center
    print("  Moving to center (1500)...")
    send_servo_command(servo_id, 1500, 1000)

    # Test minimum
    print("  Finding minimum safe position...")
    print("  Press ENTER when servo reaches safe MINIMUM, or type position:")
    min_pos = input("  Min position (or ENTER for 700): ").strip()
    min_pos = int(min_pos) if min_pos else 700
    send_servo_command(servo_id, min_pos, 1000)

    # Test maximum
    print("  Finding maximum safe position...")
    print("  Press ENTER when servo reaches safe MAXIMUM, or type position:")
    max_pos = input("  Max position (or ENTER for 2300): ").strip()
    max_pos = int(max_pos) if max_pos else 2300
    send_servo_command(servo_id, max_pos, 1000)

    # Return to center
    send_servo_command(servo_id, 1500, 1000)

    # Convert to degrees (500-2500 = 0-180°, center 1500 = 90°)
    min_deg = (min_pos - 500) * 180 / 2000
    max_deg = (max_pos - 500) * 180 / 2000
    center_deg = 90.0

    # Convert to radians relative to center (90°)
    import math
    min_rad = math.radians(min_deg - center_deg)
    max_rad = math.radians(max_deg - center_deg)

    measurements[joint_name] = {
        'servo_id': servo_id,
        'min_pos': min_pos,
        'max_pos': max_pos,
        'min_deg': round(min_deg, 1),
        'max_deg': round(max_deg, 1),
        'min_rad': round(min_rad, 3),
        'max_rad': round(max_rad, 3),
    }

    print(f"  ✓ Range: {min_deg:.1f}° to {max_deg:.1f}° ({min_rad:.2f} to {max_rad:.2f} rad)")

# Save measurements
output_file = 'robot_measurements.json'
with open(output_file, 'w') as f:
    json.dump(measurements, f, indent=2)

print(f"\n✓ Measurements saved to {output_file}")
print("\nSummary:")
for joint_name, data in measurements.items():
    print(f"  {joint_name:20s}: {data['min_deg']:6.1f}° to {data['max_deg']:6.1f}°")

ser.close()
