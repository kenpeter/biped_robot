#!/usr/bin/env python3
"""Move head servo by small steps.

Usage:
  python3 move_head.py              # show current position
  python3 move_head.py 2200         # go to position
  python3 move_head.py +10          # move +10 from current
  python3 move_head.py -10          # move -10 from current
  python3 move_head.py center       # set current position as center
  python3 move_head.py center 2200  # set 2200 as center
  python3 move_head.py home         # go to center
  python3 move_head.py left         # go to left limit
  python3 move_head.py right        # go to right limit
  python3 move_head.py left 2150    # set left limit
  python3 move_head.py right 2250   # set right limit
  python3 move_head.py show         # show calibration
"""
import serial
import sys
import os
import json

SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 9600
SERVO_CHANNEL = 0
CALIB_FILE = '/tmp/head_servo_calib.json'

def load_calib():
    if os.path.exists(CALIB_FILE):
        with open(CALIB_FILE) as f:
            return json.load(f)
    return {'pos': 2200, 'center': 2200, 'left': None, 'right': None}

def save_calib(calib):
    with open(CALIB_FILE, 'w') as f:
        json.dump(calib, f)

def move_servo(pos):
    pos = int(pos)
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    cmd = bytes([0x55, 0x55, 0x08, 0x03, 0x01, 0xF4, 0x01,
                 SERVO_CHANNEL, pos & 0xFF, (pos >> 8) & 0xFF])
    ser.write(cmd)
    ser.close()
    return pos

if __name__ == "__main__":
    calib = load_calib()

    if len(sys.argv) < 2:
        print(f"pos={calib['pos']}  center={calib['center']}  left={calib['left']}  right={calib['right']}")
        sys.exit(0)

    cmd = sys.argv[1]

    if cmd == 'show':
        print(f"pos={calib['pos']}  center={calib['center']}  left={calib['left']}  right={calib['right']}")

    elif cmd == 'center':
        if len(sys.argv) > 2:
            calib['center'] = int(sys.argv[2])
        else:
            calib['center'] = calib['pos']
        save_calib(calib)
        print(f"Center set to {calib['center']}")

    elif cmd == 'home':
        pos = calib['center']
        move_servo(pos)
        calib['pos'] = pos
        save_calib(calib)
        print(f"-> {pos} (center)")

    elif cmd == 'left':
        if len(sys.argv) > 2:
            calib['left'] = int(sys.argv[2])
            save_calib(calib)
            print(f"Left limit set to {calib['left']}")
        elif calib['left']:
            pos = calib['left']
            move_servo(pos)
            calib['pos'] = pos
            save_calib(calib)
            print(f"-> {pos} (left)")
        else:
            print("No left limit set. Use: python3 move_head.py left 2150")

    elif cmd == 'right':
        if len(sys.argv) > 2:
            calib['right'] = int(sys.argv[2])
            save_calib(calib)
            print(f"Right limit set to {calib['right']}")
        elif calib['right']:
            pos = calib['right']
            move_servo(pos)
            calib['pos'] = pos
            save_calib(calib)
            print(f"-> {pos} (right)")
        else:
            print("No right limit set. Use: python3 move_head.py right 2250")

    elif cmd.startswith('+') or cmd.startswith('-'):
        pos = calib['pos'] + int(cmd)
        pos = max(500, min(2500, pos))
        move_servo(pos)
        print(f"{calib['pos']} -> {pos}")
        calib['pos'] = pos
        save_calib(calib)

    else:
        pos = int(cmd)
        pos = max(500, min(2500, pos))
        move_servo(pos)
        print(f"{calib['pos']} -> {pos}")
        calib['pos'] = pos
        save_calib(calib)
