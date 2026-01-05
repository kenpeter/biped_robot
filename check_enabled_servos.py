#!/usr/bin/env python3
"""
Check which servos are enabled in ZideConfig.ini
"""

import configparser

config = configparser.ConfigParser()
config.read('/home/jetson/biped_ws/ZideConfig.ini')

print("\n" + "="*70)
print("ENABLED SERVOS IN ZideConfig.ini")
print("="*70 + "\n")

enabled_servos = []

for section in config.sections():
    if section.startswith('steer'):
        try:
            servo_id = config.getint(section, 'id')
            enabled = config.getboolean(section, 'enable')
            title = config.get(section, 'title', fallback='N/A')
            pmin = config.getint(section, 'pmin', fallback=500)
            pmax = config.getint(section, 'pmax', fallback=2500)

            if enabled and servo_id < 24:  # Only channels 0-23
                enabled_servos.append(servo_id)
                print(f"Servo {servo_id:02d}: ✓ ENABLED  (title={title}, range={pmin}-{pmax})")
        except:
            pass

print(f"\n{'='*70}")
print(f"SUMMARY: {len(enabled_servos)} servos enabled")
print(f"{'='*70}")
print(f"\nEnabled channels: {sorted(enabled_servos)}")
print(f"\n{'='*70}\n")
