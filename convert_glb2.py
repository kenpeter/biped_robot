#!/usr/bin/env python3
"""
Convert GLB to USD using Isaac Sim's asset converter
"""

import os
import asyncio
from isaacsim import SimulationApp

print("Starting Isaac Sim for GLB conversion...")

simulation_app = SimulationApp({"headless": True})

import omni.usd
import omni.kit.asset_converter
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

GLB_PATH = "/home/kenpeter/work/biped_robot/models/humanoid.glb"
OUTPUT_PATH = "/home/kenpeter/work/biped_robot/models/humanoid_converted.usda"

# Create a new stage
stage = omni.usd.get_context().get_stage()

# Create a task for the converter
def converter_task(source_path, dest_path):
    """Convert asset using Isaac Sim's converter"""
    converter = omni.kit.asset_converter.get_instance()
    
    task = converter.create_converter_task(
        source_path,
        dest_path,
        lambda success, path: print(f"Conversion {'succeeded' if success else 'failed'}: {path}"),
        lambda current, total: print(f"Progress: {current}/{total}")
    )
    
    return task

# Try direct conversion
print(f"Converting {GLB_PATH} to {OUTPUT_PATH}...")

# Use the converter
from omni.kit.asset_converter import AssetConverter
converter = AssetConverter()

# Convert the GLB to USD
success = converter.convert(GLB_PATH, OUTPUT_PATH)
print(f"Conversion success: {success}")

if success:
    # Check the output
    if os.path.exists(OUTPUT_PATH):
        print(f"\nOutput file size: {os.path.getsize(OUTPUT_PATH)} bytes")
        
        # Open and examine
        new_stage = Usd.Stage.Open(OUTPUT_PATH)
        print(f"\nStage contents:")
        for prim in new_stage.Traverse():
            print(f"  {prim.GetPath()}")
        
        # Count mesh prims
        mesh_count = sum(1 for prim in new_stage.Traverse() if prim.IsA(UsdGeom.Mesh))
        print(f"\nFound {mesh_count} mesh prims")
else:
    print("Conversion failed")

simulation_app.close()
print("Done.")
