import bpy
import math

# Run the robot creation first
exec(compile(open('/home/jetson/work/biped_ws/models/create_robot.py').read().replace(
    "bpy.ops.render.render(write_still=True)",
    "# skip render"
), '<string>', 'exec'))

# Save Blender file
bpy.ops.wm.save_as_mainfile(filepath='/home/jetson/work/biped_ws/models/humanoid_robot.blend')
print("Saved Blender file!")
