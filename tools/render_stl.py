"""Render STL parts to PNG with Blender (headless).

Usage:
  blender -b -P render_stl.py -- OUT.png VIEW part.stl@#RRGGBB[@dx,dy,dz[,rx,ry,rz]] ...
VIEW: iso | top | front | side
Each part may carry an offset (mm) and rotation (deg) so assemblies can be shown.
A light-grey 3/4" PVC pipe (OD 26.7 mm) can be added with the pseudo-part
  PIPE@#F2F2F2@y_len   (pipe along Y, resting on z=0)
"""
import sys
import math
import bpy

argv = sys.argv[sys.argv.index("--") + 1:]
out, view, parts = argv[0], argv[1], argv[2:]

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.unit_settings.scale_length = 0.001

def hex_rgba(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)) + (1.0,)

def material(col):
    mat = bpy.data.materials.new("m")
    mat.diffuse_color = hex_rgba(col)
    return mat

objs = []
for spec in parts:
    bits = spec.split("@")
    path, col = bits[0], bits[1] if len(bits) > 1 else "#CC2222"
    xf = [float(v) for v in bits[2].split(",")] if len(bits) > 2 else []
    if path == "PIPE":
        length = xf[0] if xf else 200
        bpy.ops.mesh.primitive_cylinder_add(radius=13.35, depth=length, vertices=96,
                                            location=(0, 0, 13.35), rotation=(math.radians(90), 0, 0))
        ob = bpy.context.active_object
    else:
        bpy.ops.wm.stl_import(filepath=path)
        ob = bpy.context.selected_objects[0]
        if len(xf) >= 3:
            ob.location = xf[0:3]
        if len(xf) >= 6:
            ob.rotation_euler = [math.radians(v) for v in xf[3:6]]
    ob.data.materials.append(material(col))
    objs.append(ob)

bpy.context.view_layer.update()
pts = [ob.matrix_world @ v.co for ob in objs for v in ob.data.vertices]
lo = [min(p[i] for p in pts) for i in range(3)]
hi = [max(p[i] for p in pts) for i in range(3)]
c = [(lo[i] + hi[i]) / 2 for i in range(3)]
size = max(hi[i] - lo[i] for i in range(3))

cam_data = bpy.data.cameras.new("cam")
cam_data.type = "ORTHO"
cam_data.ortho_scale = size * 1.35
cam = bpy.data.objects.new("cam", cam_data)
scene.collection.objects.link(cam)
scene.camera = cam
d = size * 3
dirs = {"iso": (1, -1, 0.8), "top": (0, -0.0001, 1), "front": (0, -1, 0.0001), "side": (1, 0, 0.0001)}
v = dirs[view]
n = math.sqrt(sum(x * x for x in v))
cam.location = [c[i] + v[i] / n * d for i in range(3)]
direction = [c[i] - cam.location[i] for i in range(3)]
import mathutils
cam.rotation_euler = mathutils.Vector(direction).to_track_quat("-Z", "Y").to_euler()
cam_data.clip_end = d * 4

scene.render.engine = "BLENDER_WORKBENCH"
shading = scene.display.shading
shading.light = "STUDIO"
shading.color_type = "MATERIAL"
shading.show_cavity = True
shading.show_shadows = True
shading.background_type = "VIEWPORT"
shading.background_color = (0.93, 0.93, 0.95)
scene.world = bpy.data.worlds.new("w")
scene.world.color = (0.93, 0.93, 0.95)
scene.render.resolution_x = 900
scene.render.resolution_y = 900
scene.render.filepath = out
bpy.ops.render.render(write_still=True)
print("RENDERED", out)
