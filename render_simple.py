import bpy
import math

bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

def box(name, size, loc):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    o = bpy.context.active_object
    o.name = name
    o.scale = size
    return o

# Torso at 0.35
box("torso", (0.06, 0.10, 0.12), (0, 0, 0.35))

# Head
box("head", (0.04, 0.04, 0.04), (0, 0, 0.43))

# Waist
box("waist", (0.05, 0.08, 0.04), (0, 0, 0.27))

# Left arm
box("l_upper_arm", (0.025, 0.08, 0.025), (0, 0.11, 0.39))
box("l_forearm", (0.02, 0.07, 0.02), (0, 0.185, 0.39))
box("l_hand", (0.02, 0.02, 0.02), (0, 0.24, 0.39))

# Right arm
box("r_upper_arm", (0.025, 0.08, 0.025), (0, -0.11, 0.39))
box("r_forearm", (0.02, 0.07, 0.02), (0, -0.185, 0.39))
box("r_hand", (0.02, 0.02, 0.02), (0, -0.24, 0.39))

# Left leg
box("l_hip", (0.03, 0.03, 0.03), (0, 0.04, 0.21))
box("l_thigh", (0.03, 0.03, 0.12), (0, 0.04, 0.12))
box("l_shin", (0.025, 0.025, 0.11), (0, 0.04, -0.015))
box("l_foot", (0.08, 0.05, 0.02), (0.02, 0.04, -0.10))

# Right leg
box("r_thigh", (0.03, 0.03, 0.12), (0, -0.04, 0.12))
box("r_shin", (0.025, 0.025, 0.11), (0, -0.04, -0.015))
box("r_foot", (0.08, 0.05, 0.02), (0.02, -0.04, -0.10))

# Camera
bpy.ops.object.camera_add(location=(0.5, -0.5, 0.25), rotation=(1.2, 0, 0.75))
bpy.context.scene.camera = bpy.context.active_object
bpy.context.active_object.data.lens = 35

# Light
bpy.ops.object.light_add(type='SUN', location=(2, -1, 3))
bpy.context.active_object.data.energy = 3

# Render
bpy.context.scene.render.engine = 'BLENDER_EEVEE'
bpy.context.scene.render.resolution_x = 600
bpy.context.scene.render.resolution_y = 600
bpy.context.scene.render.filepath = '/home/jetson/biped_ws/simple_robot.png'
bpy.ops.render.render(write_still=True)
