import bpy
import math

# First run the robot creation
exec(open('/home/jetson/work/biped_ws/create_robot.py').read().replace("bpy.ops.render.render(write_still=True)", "pass"))

# Remove camera, lights, ground for USD export (Isaac Sim will have its own)
for obj in bpy.data.objects:
    if obj.name in ["Camera", "Sun", "Area", "Ground"]:
        bpy.data.objects.remove(obj)

# Export to USD
bpy.ops.wm.usd_export(
    filepath='/home/jetson/work/biped_ws/src/humanoid_description/usd/humanoid.usd',
    export_materials=True,
    export_meshes=True,
    export_textures=True,
    root_prim_path='/humanoid'
)

# Also save Blender file
bpy.ops.wm.save_as_mainfile(filepath='/home/jetson/work/biped_ws/humanoid_robot.blend')

print("Exported to USD and saved Blender file!")
