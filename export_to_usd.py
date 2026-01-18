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

def add_foot_with_holes(name, location, material):
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
bpy.ops.mesh.primitive_cube_add(size=1, location=(0.04, 0, 0.385))
cam_body = bpy.context.active_object
cam_body.name = "Thermal_Camera_Body"
cam_body.scale = (0.03, 0.025, 0.02)
cam_body.data.materials.append(camera_black)
bpy.ops.mesh.primitive_cylinder_add(radius=0.008, depth=0.012, location=(0.058, 0, 0.385), rotation=(0, math.pi/2, 0))
cam_lens = bpy.context.active_object
cam_lens.name = "Thermal_Camera_Lens"
cam_lens.data.materials.append(camera_black)

# ============ LEFT ARM ============
add_servo("L_Shoulder1", (0, 0.06, 0.31), (math.pi/2, 0, 0))
add_bracket("L_Shoulder_Plate", (0.028, 0.035, 0.028), (0, 0.085, 0.31))
add_servo("L_Shoulder2", (0, 0.105, 0.31))
add_bracket("L_UpperArm", (0.022, 0.025, 0.065), (0, 0.105, 0.265))
add_servo("L_Elbow", (0, 0.105, 0.225), (0, math.pi/2, 0))
add_bracket("L_LowerArm", (0.020, 0.022, 0.055), (0, 0.105, 0.185))

# ============ RIGHT ARM ============
add_servo("R_Shoulder1", (0, -0.06, 0.31), (math.pi/2, 0, 0))
add_bracket("R_Shoulder_Plate", (0.028, 0.035, 0.028), (0, -0.085, 0.31))
add_servo("R_Shoulder2", (0, -0.105, 0.31))
add_bracket("R_UpperArm", (0.022, 0.025, 0.065), (0, -0.105, 0.265))
add_servo("R_Elbow", (0, -0.105, 0.225), (0, math.pi/2, 0))
add_bracket("R_LowerArm", (0.020, 0.022, 0.055), (0, -0.105, 0.185))

# ============ LEFT LEG ============
ly = 0.038
add_servo("L_Hip1", (0, ly, 0.205))
add_bracket("L_Hip_Bracket", (0.038, 0.042, 0.022), (0, ly, 0.18))
add_servo("L_Hip2", (0, ly, 0.155), (0, math.pi/2, 0))
add_bracket("L_Thigh_Upper", (0.028, 0.034, 0.048), (0, ly, 0.12))
add_servo("L_Thigh_Servo", (0, ly, 0.085))
add_bracket("L_Thigh_Lower", (0.028, 0.034, 0.038), (0, ly, 0.055))
add_servo("L_Knee", (0, ly, 0.028), (0, math.pi/2, 0))
add_bracket("L_Shin_Upper", (0.026, 0.030, 0.048), (0, ly, -0.01))
add_servo("L_Shin_Servo", (0, ly, -0.042))
add_bracket("L_Shin_Lower", (0.026, 0.030, 0.038), (0, ly, -0.07))
add_servo("L_Ankle", (0, ly, -0.095), (0, math.pi/2, 0))
add_foot_with_holes("L_Foot", (0.012, ly, -0.115), aluminum)

# ============ RIGHT LEG ============
ry = -0.038
add_servo("R_Hip1", (0, ry, 0.205))
add_bracket("R_Hip_Bracket", (0.038, 0.042, 0.022), (0, ry, 0.18))
add_servo("R_Hip2", (0, ry, 0.155), (0, math.pi/2, 0))
add_bracket("R_Thigh_Upper", (0.028, 0.034, 0.048), (0, ry, 0.12))
add_servo("R_Thigh_Servo", (0, ry, 0.085))
add_bracket("R_Thigh_Lower", (0.028, 0.034, 0.038), (0, ry, 0.055))
add_servo("R_Knee", (0, ry, 0.028), (0, math.pi/2, 0))
add_bracket("R_Shin_Upper", (0.026, 0.030, 0.048), (0, ry, -0.01))
add_servo("R_Shin_Servo", (0, ry, -0.042))
add_bracket("R_Shin_Lower", (0.026, 0.030, 0.038), (0, ry, -0.07))
add_servo("R_Ankle", (0, ry, -0.095), (0, math.pi/2, 0))
add_foot_with_holes("R_Foot", (0.012, ry, -0.115), aluminum)

# Export to USD
bpy.ops.wm.usd_export(
    filepath='/home/jetson/biped_ws/src/humanoid_description/usd/humanoid.usda',
    export_materials=True,
    export_meshes=True,
    export_textures=False,
    selected_objects_only=False,
    evaluation_mode='RENDER'
)

print("Exported to humanoid.usda!")
