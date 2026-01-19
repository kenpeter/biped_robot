#!/usr/bin/env python3
"""
Convert GLB to USD with proper physics bindings using Isaac Sim
"""

import os
import numpy as np
from isaacsim import SimulationApp

print("Starting Isaac Sim for GLB conversion...")

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
import omni.kit.commands

GLB_PATH = "/home/kenpeter/work/biped_robot/models/humanoid.glb"
OUTPUT_PATH = "/home/kenpeter/work/biped_robot/models/humanoid_new.usda"

# Create new stage
stage = omni.usd.get_context().get_stage()

# Delete existing content
for prim in stage.Traverse():
    stage.RemovePrim(prim.GetPath())

# Import GLB using Isaac Sim's asset importer
print(f"Importing GLB: {GLB_PATH}")

# Create the import task
from omni.kit.commands import execute

# Import the GLB as references
result = execute(
    'CreateReference',
    usd_context=omni.usd.get_context(),
    path_to='/World/Humanoid',
    asset_path=GLB_PATH,
)

print(f"Import result: {result}")

# Check what was created
print("\nStage contents:")
for prim in stage.Traverse():
    print(f"  {prim.GetPath()}")

# Now add physics to the imported meshes
# First, let's find all the mesh prims
mesh_prims = []
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Mesh):
        mesh_prims.append(prim)
        
print(f"\nFound {len(mesh_prims)} mesh prims")

# Save the stage
stage.GetRootLayer().Export(OUTPUT_PATH)
print(f"\nSaved to: {OUTPUT_PATH}")

simulation_app.close()
print("Done.")
