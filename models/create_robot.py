import bpy
import math

# Clear everything
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

for mat in bpy.data.materials:
    bpy.data.materials.remove(mat)

def create_material(name, color, metallic=0.8):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs['Base Color'].default_value = color
    bsdf.inputs['Metallic'].default_value = metallic
    bsdf.inputs['Roughness'].default_value = 0.3
    return mat

aluminum = create_material("Aluminum", (0.78, 0.78, 0.80, 1), 0.9)
servo_pink = create_material("ServoPink", (0.85, 0.15, 0.30, 1), 0.3)

def add_bracket(name, size, location, rotation=(0,0,0)):
    bpy.ops.mesh.primitive_cube_add(size=1, location=location, rotation=rotation)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = size
    obj.data.materials.append(aluminum)
    return obj

def add_servo(name, location, rotation=(0,0,0)):
    bpy.ops.mesh.primitive_cube_add(size=1, location=location, rotation=rotation)
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = (0.045, 0.023, 0.045)
    obj.data.materials.append(servo_pink)
    return obj

def add_L_bracket(name, pos, size_v, size_h, rotation=(0,0,0)):
    """L-shaped bracket like in the real robot"""
    # Vertical part
    add_bracket(f"{name}_V", (size_v[0], size_v[1], size_v[2]),
                (pos[0], pos[1], pos[2]))
    # Horizontal part (offset)
    add_bracket(f"{name}_H", (size_h[0], size_h[1], size_h[2]),
                (pos[0] + size_h[0]/2, pos[1], pos[2] - size_v[2]/2 + size_h[2]/2))

def add_foot_with_holes(name, location, material):
    """Foot plate with 4 holes pattern"""
    # Main plate
    bpy.ops.mesh.primitive_cube_add(size=1, location=location)
    foot = bpy.context.active_object
    foot.name = name
    foot.scale = (0.085, 0.055, 0.006)
    foot.data.materials.append(material)
    return foot

# ============ TORSO ============
add_bracket("Torso_Main", (0.045, 0.075, 0.085), (0, 0, 0.28))
add_bracket("Torso_Top_Plate", (0.055, 0.095, 0.012), (0, 0, 0.33))
add_bracket("Torso_Bot_Plate", (0.055, 0.095, 0.012), (0, 0, 0.23))

# ============ HEAD ============
add_servo("Head_Servo", (0, 0, 0.355))
add_bracket("Head_Mount", (0.035, 0.038, 0.022), (0, 0, 0.385))

# ============ THERMAL CAMERA (on head) ============
camera_black = create_material("CameraBlack", (0.15, 0.15, 0.18, 1), 0.3)
# Camera body
bpy.ops.mesh.primitive_cube_add(size=1, location=(0.04, 0, 0.385))
cam_body = bpy.context.active_object
cam_body.name = "Thermal_Camera_Body"
cam_body.scale = (0.03, 0.025, 0.02)
cam_body.data.materials.append(camera_black)
# Camera lens
bpy.ops.mesh.primitive_cylinder_add(radius=0.008, depth=0.012, location=(0.058, 0, 0.385), rotation=(0, math.pi/2, 0))
cam_lens = bpy.context.active_object
cam_lens.name = "Thermal_Camera_Lens"
cam_lens.data.materials.append(camera_black)

# ============ LEFT ARM (hanging down) ============
add_servo("L_Shoulder1", (0, 0.06, 0.31), (math.pi/2, 0, 0))
add_bracket("L_Shoulder_Plate", (0.028, 0.035, 0.028), (0, 0.085, 0.31))
add_servo("L_Shoulder2", (0, 0.105, 0.31))
# Upper arm - hanging down
add_bracket("L_UpperArm", (0.022, 0.025, 0.065), (0, 0.105, 0.265))
add_servo("L_Elbow", (0, 0.105, 0.225), (0, math.pi/2, 0))
# Lower arm
add_bracket("L_LowerArm", (0.020, 0.022, 0.055), (0, 0.105, 0.185))

# ============ RIGHT ARM (hanging down) ============
add_servo("R_Shoulder1", (0, -0.06, 0.31), (math.pi/2, 0, 0))
add_bracket("R_Shoulder_Plate", (0.028, 0.035, 0.028), (0, -0.085, 0.31))
add_servo("R_Shoulder2", (0, -0.105, 0.31))
add_bracket("R_UpperArm", (0.022, 0.025, 0.065), (0, -0.105, 0.265))
add_servo("R_Elbow", (0, -0.105, 0.225), (0, math.pi/2, 0))
add_bracket("R_LowerArm", (0.020, 0.022, 0.055), (0, -0.105, 0.185))

# ============ LEFT LEG ============
ly = 0.038

# Hip (Servo 1: Hip Roll)
add_servo("L_Hip1", (0, ly, 0.205))
add_bracket("L_Hip_Bracket", (0.038, 0.042, 0.022), (0, ly, 0.18))

# Hip (Servo 2: Hip Pitch)
add_servo("L_Hip2", (0, ly, 0.155), (0, math.pi/2, 0))

# Thigh (Connects Hip2 to Knee)
add_bracket("L_Thigh_Structure", (0.028, 0.034, 0.09), (0, ly, 0.09))

# Knee (Servo 3: Knee Pitch)
add_servo("L_Knee", (0, ly, 0.028), (0, math.pi/2, 0))

# Shin (Connects Knee to Ankle)
add_bracket("L_Shin_Structure", (0.026, 0.030, 0.09), (0, ly, -0.035))

# Ankle (Servo 4: Ankle Pitch)
add_servo("L_Ankle", (0, ly, -0.095), (0, math.pi/2, 0))

# Foot
add_foot_with_holes("L_Foot", (0.012, ly, -0.115), aluminum)

# ============ RIGHT LEG ============
ry = -0.038

# Hip (Servo 1: Hip Roll)
add_servo("R_Hip1", (0, ry, 0.205))
add_bracket("R_Hip_Bracket", (0.038, 0.042, 0.022), (0, ry, 0.18))

# Hip (Servo 2: Hip Pitch)
add_servo("R_Hip2", (0, ry, 0.155), (0, math.pi/2, 0))

# Thigh
add_bracket("R_Thigh_Structure", (0.028, 0.034, 0.09), (0, ry, 0.09))

# Knee (Servo 3: Knee Pitch)
add_servo("R_Knee", (0, ry, 0.028), (0, math.pi/2, 0))

# Shin
add_bracket("R_Shin_Structure", (0.026, 0.030, 0.09), (0, ry, -0.035))

# Ankle (Servo 4: Ankle Pitch)
add_servo("R_Ankle", (0, ry, -0.095), (0, math.pi/2, 0))

# Foot
add_foot_with_holes("R_Foot", (0.012, ry, -0.115), aluminum)

# ============ GROUND ============
bpy.ops.mesh.primitive_plane_add(size=1, location=(0, 0, -0.12))
ground = bpy.context.active_object
ground.name = "Ground"
ground.scale = (0.5, 0.5, 1)
ground_mat = create_material("Ground", (0.25, 0.25, 0.25, 1), 0.0)
ground.data.materials.append(ground_mat)

# ============ CAMERA ============
# Front-side angle to show thermal camera on head
bpy.ops.object.camera_add(location=(0.55, -0.35, 0.35), rotation=(1.25, 0, 0.55))
cam = bpy.context.active_object
cam.name = "Camera"
cam.data.lens = 32
bpy.context.scene.camera = cam

# Lights
bpy.ops.object.light_add(type='SUN', location=(2, -1, 3), rotation=(0.5, 0.2, 0))
bpy.context.active_object.data.energy = 2.5

bpy.ops.object.light_add(type='AREA', location=(-1, 1, 2))
bpy.context.active_object.data.energy = 120

# Render
bpy.context.scene.render.engine = 'BLENDER_EEVEE'
bpy.context.scene.render.resolution_x = 800
bpy.context.scene.render.resolution_y = 800
bpy.context.scene.render.filepath = '/home/jetson/work/biped_ws/models/robot_render.png'

bpy.ops.render.render(write_still=True)
print("Render saved to models/robot_render.png!")
