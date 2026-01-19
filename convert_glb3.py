#!/usr/bin/env python3
"""
Convert GLB to USD using Isaac Sim's asset converter
"""

import os
from isaacsim import SimulationApp

print("Starting Isaac Sim for GLB conversion...")

simulation_app = SimulationApp({"headless": True})

import omni.usd
import omni.kit.asset_converter
from pxr import Usd, UsdGeom, Gf

GLB_PATH = "/home/kenpeter/work/biped_robot/models/humanoid.glb"
OUTPUT_PATH = "/home/kenpeter/work/biped_robot/models/humanoid_converted.usda"

# Create a new stage
stage = omni.usd.get_context().get_stage()

print(f"Converting {GLB_PATH} to {OUTPUT_PATH}...")

# Use the asset converter
converter = omni.kit.asset_converter.get_instance()

# Define the task
task = converter.create_converter_task(
    GLB_PATH,
    OUTPUT_PATH,
    on_complete=lambda success, path: print(f"Conversion {'succeeded' if success else 'failed'}: {path}"),
    on_progress=lambda current, total: print(f"Progress: {current}/{total}")
)

# Wait for completion
import time
timeout = 30
start = time.time()
while task.is_done() == False and (time.time() - start) < timeout:
    time.sleep(0.1)

if task.is_done():
    print(f"Task completed: {task.get_status()}")
    
    # Check the output
    if os.path.exists(OUTPUT_PATH):
        print(f"\nOutput file size: {os.path.getsize(OUTPUT_PATH)} bytes")
        
        # Open and examine
        new_stage = Usd.Stage.Open(OUTPUT_PATH)
        print(f"\nStage contents (first 20):")
        count = 0
        for prim in new_stage.Traverse():
            print(f"  {prim.GetPath()}")
            count += 1
            if count >= 20:
                print("  ...")
                break
        
        # Count mesh prims
        mesh_count = sum(1 for prim in new_stage.Traverse() if prim.IsA(UsdGeom.Mesh))
        print(f"\nFound {mesh_count} mesh prims")
else:
    print("Conversion timed out")

simulation_app.close()
print("Done.")
