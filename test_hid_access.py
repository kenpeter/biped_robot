#!/usr/bin/env python3
"""
Attempt to access servo controller via HID interface
Even though it doesn't enumerate as HID, worth trying
"""

import hid
import time

# CH340 IDs from lsusb
VENDOR_ID = 0x1a86
PRODUCT_ID = 0x7523

print("\n" + "="*70)
print("HID ACCESS TEST")
print("="*70)
print(f"\nAttempting to open device as HID:")
print(f"  Vendor ID:  0x{VENDOR_ID:04x}")
print(f"  Product ID: 0x{PRODUCT_ID:04x}")
print("="*70 + "\n")

try:
    # Try to open as HID
    device = hid.device()
    device.open(VENDOR_ID, PRODUCT_ID)

    print("✓ SUCCESS! Device opened as HID!")
    print("\nDevice info:")
    print(f"  Manufacturer: {device.get_manufacturer_string()}")
    print(f"  Product: {device.get_product_string()}")
    print(f"  Serial: {device.get_serial_number_string()}")

    # Try sending servo command via HID
    print("\n" + "="*60)
    print("Attempting servo command via HID...")
    print("="*60)

    # HID requires report ID as first byte (usually 0)
    # Format: #000P1500T1000!
    command = "#000P1500T1000!"
    print(f"\nCommand: {command}")

    # Try different report formats
    formats = [
        ("Report ID 0", [0] + [ord(c) for c in command]),
        ("No Report ID", [ord(c) for c in command]),
        ("Report ID 1", [1] + [ord(c) for c in command]),
        ("With braces", [0] + [ord(c) for c in "{#000P1500T1000!}"]),
    ]

    for name, data in formats:
        print(f"\n  Trying: {name}")
        print(f"  Data length: {len(data)} bytes")
        try:
            # Pad to 64 bytes if needed (common HID report size)
            if len(data) < 64:
                data = data + [0] * (64 - len(data))

            result = device.write(data)
            print(f"  ✓ Wrote {result} bytes")
            time.sleep(0.5)

            # Try to read response
            device.set_nonblocking(1)
            response = device.read(64, timeout_ms=100)
            if response:
                print(f"  Response: {bytes(response)}")
        except Exception as e:
            print(f"  ✗ Failed: {e}")

    print("\n" + "="*60)
    print("Did any servo move?")
    print("="*60)

    device.close()

except IOError as e:
    print(f"✗ Cannot open device as HID: {e}")
    print("\nThis confirms the device is NOT accessible via HID on Linux.")
    print("It only works as a serial device (/dev/ttyUSB0).")
    print("\nThe PC software 'HID Connected' label might be misleading,")
    print("or Windows presents the CH340 differently than Linux.")

except Exception as e:
    print(f"✗ Error: {e}")

print("\n" + "="*70)
print("\nRECOMMENDATION:")
print("  Use the spy script to see what PC software actually sends:")
print("  python3 /home/jetson/biped_ws/spy_on_pc.py")
print("="*70 + "\n")
