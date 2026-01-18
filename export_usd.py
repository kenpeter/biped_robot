import bpy
import math
import os

base_path = os.getcwd()

# First run the robot creation
exec(open(os.path.join(base_path, 'create_robot.py')).read().replace("bpy.ops.render.render(write_still=True)", "pass"))

# Remove camera, lights, ground for USD export (Isaac Sim will have its own)
for obj in bpy.data.objects:
    if obj.name in ["Camera", "Sun", "Area", "Ground"]:
        bpy.data.objects.remove(obj)

# Ensure directory exists
usd_dir = os.path.join(base_path, 'src/humanoid_description/usd')
os.makedirs(usd_dir, exist_ok=True)

# Export to USD
bpy.ops.wm.usd_export(
    filepath=os.path.join(usd_dir, 'humanoid.usd'),
    export_materials=True,
    export_meshes=True,
    export_textures=True,
    root_prim_path='/humanoid'
)

# Also save Blender file
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(base_path, 'humanoid_robot.blend'))

print("Exported to USD and saved Blender file!")