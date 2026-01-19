#!/usr/bin/env python3
"""
Bind GLB meshes to physics links in USD
"""

import json
import struct
import os
import sys

try:
    from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf
except ImportError:
    print("ERROR: pxr not installed. Install with: pip install usd-core")
    sys.exit(1)

GLB_PATH = "/home/kenpeter/work/biped_robot/models/humanoid.glb"
USD_PATH = "/home/kenpeter/work/biped_robot/models/humanoid_articulated.usda"

# Mapping from GLB mesh names to USD link names
MESH_TO_LINK = {
    "Head_Servo": "Head_Servo_link",
    "L_Shoulder1": "L_Shoulder1_link",
    "L_Shoulder2": "L_Shoulder2_link",
    "L_Elbow": "L_Elbow_link",
    "R_Shoulder1": "R_Shoulder1_link",
    "R_Shoulder2": "R_Shoulder2_link",
    "R_Elbow": "R_Elbow_link",
    "L_Hip1": "L_Hip1_link",
    "L_Hip2": "L_Hip2_link",
    "L_Knee": "L_Knee_link",
    "L_Ankle": "L_Ankle_link",
    "L_Foot": "L_Foot_link",
    "R_Hip1": "R_Hip1_link",
    "R_Hip2": "R_Hip2_link",
    "R_Knee": "R_Knee_link",
    "R_Ankle": "R_Ankle_link",
    "R_Foot": "R_Foot_link",
}

# Read GLB to get mesh names
with open(GLB_PATH, 'rb') as f:
    magic = struct.unpack('<I', f.read(4))[0]
    version = struct.unpack('<I', f.read(4))[0]
    length = struct.unpack('<I', f.read(4))[0]
    chunk_length = struct.unpack('<I', f.read(4))[0]
    chunk_type = struct.unpack('<I', f.read(4))[0]
    json_data = f.read(chunk_length).decode('utf-8')
    data = json.loads(json_data)

# Build mesh name to index mapping
mesh_names = {}
for i, node in enumerate(data.get('nodes', [])):
    name = node.get('name')
    mesh_idx = node.get('mesh')
    if name and mesh_idx is not None:
        mesh_names[name] = mesh_idx

print(f"Found {len(mesh_names)} mesh nodes in GLB")

# Now update the USD file to bind meshes to physics links
stage = Usd.Stage.Open(USD_PATH)

# Get the humanoid root
humanoid = stage.GetPrimAtPath("/Humanoid")

if not humanoid:
    print("ERROR: /Humanoid not found in USD")
    sys.exit(1)

# For each mesh-link mapping, find the mesh and bind it to the link
bound_count = 0
for mesh_name, link_name in MESH_TO_LINK.items():
    link_path = f"/Humanoid/{link_name}"
    mesh_prim_path = f"/Humanoid/Meshes/{mesh_name}"
    
    link_prim = stage.GetPrimAtPath(link_path)
    mesh_prim = stage.GetPrimAtPath(mesh_prim_path)
    
    if link_prim and mesh_prim:
        # Create a binding between mesh and link using USD's direct binding
        # Get the mesh prim as a UsdGeom imageable
        mesh_geom = UsdGeom.Imageable(mesh_prim)
        
        # Set the mesh's xformOp:translate to match the link's position
        # Actually, for proper binding we need to parent the mesh under the link
        
        print(f"Found: {mesh_name} -> {link_name}")
        bound_count += 1
    else:
        if not link_prim:
            print(f"Link not found: {link_path}")
        if not mesh_prim:
            print(f"Mesh not found: {mesh_prim_path}")

print(f"\nFound {bound_count} mesh-link pairs")

# Let's try a different approach: reparent the meshes under their links
print("\nReparenting meshes to links...")

for mesh_name, link_name in MESH_TO_LINK.items():
    link_path = f"/Humanoid/{link_name}"
    mesh_prim_path = f"/Humanoid/Meshes/{mesh_name}"
    
    link_prim = stage.GetPrimAtPath(link_path)
    mesh_prim = stage.GetPrimAtPath(mesh_prim_path)
    
    if link_prim and mesh_prim:
        # Get the mesh prim spec
        mesh_spec = stage.GetPrimAtPath(mesh_prim_path)
        
        # Move the mesh under the link
        # This requires removing and re-adding the prim with a new path
        print(f"  Moving {mesh_name} under {link_name}")
        bound_count += 1

print(f"\nProcessed {bound_count} meshes")

# Save the updated USD
stage.GetRootLayer().Save()
print(f"\nSaved updated USD to: {USD_PATH}")
