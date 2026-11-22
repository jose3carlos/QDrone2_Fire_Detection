# -*- coding: utf-8 -*-
"""
Generador de dataset sintético (formato YOLO) para el detector de anomalías del
proyecto QDrone 2: clase 0 = fuego, clase 1 = toma (válvula clandestina).

Se ejecuta con Blender en modo consola (sin interfaz):

  "C:/Program Files/Blender Foundation/Blender 5.3/blender.exe" -b -P vision/sintetico/generar_blender.py -- \
      --salida vision/datos/sintetico_prueba --n 600 --semilla 1

Escena:
  - Piso de tapete EVA gris oscuro tipo rompecabezas (textura generada con numpy) y piso del laboratorio alrededor.
  - Tubos de PVC blancos (DE 26.7 mm) en polilíneas tipo "N" aleatorias, con coples en uniones y a lo largo.
  - Fuegos (STL impresion3d/stl/fuego_{A,B,C}_rojo + _amarillo ensamblados, rojo #C81E1E / amarillo #F5C400;
    a veces la versión monocolor) junto al tubo o encima de él.
  - Tomas (válvula: cuerpo + tuerca + vástago/volante + zapata, en rojo) ABRAZADAS al tubo.
    Se usan las mallas en posición de uso (impresion3d/renders/_mallas_uso) o, si no existen, las STL de
    impresión re-orientadas. Si faltan, se usan modelos procedurales de reemplazo. Las STL se buscan en cada
    ejecución, así que en cuanto aparezcan se usan automáticamente.
  - Distractores: plantas artificiales, carritos de juguete, cilindros grises, cruces de cinta blanca,
    cintas de colores, calcomanías PLANAS de flamas (son FONDO, no se etiquetan), objetos rojos/amarillos,
    marcadores de OptiTrack, cables, hojas de papel y, a veces, el otro QDrone 2 volando debajo.
  - Cámara cenital entre 0.3 y 1.8 m con inclinación de ±10°, intrínsecos de la RealSense D435i
    (VFOV 42.5°; 1280x720 -> HFOV 69.4°, 640x480 -> HFOV ~55°, el modo 4:3 recorta los lados).
  - Iluminación, exposición, balance de blancos, desenfoque de movimiento, ruido y JPEG aleatorios.
  - Etiquetas YOLO a partir de la proyección de los vértices de cada objeto, con oclusión (ray casting)
    y recorte por el borde de la imagen.
  - Partición train/val por escena (las vistas de una misma escena nunca se mezclan entre train y val).

Argumentos (después de "--"):
  --salida DIR          carpeta del dataset (se crean images/{train,val}, labels/{train,val}, data.yaml)
  --n N                 número de imágenes a generar (por defecto 600)
  --semilla S           semilla base (cada escena usa semilla + índice: reproducible y reanudable)
  --vistas K            vistas de cámara por escena (por defecto 6)
  --motor cycles|eevee  motor de render (por defecto cycles con GPU OptiX)
  --muestras M          muestras de Cycles (por defecto 24, con denoiser)
  --res mix|640|1280    resolución (mix = 70 % 640x480, 30 % 1280x720)
  --frac_val F          fracción de escenas de validación (por defecto 0.15)
  --sin_post            no aplicar post-proceso de cámara (ruido, blur, etc.)
  --guardar_blend       guarda la primera escena como .blend para inspección
Si se vuelve a correr con la misma salida y semilla, se saltan las escenas ya generadas (reanudar).
"""
import argparse
import glob
import json
import math
import os
import random
import sys
import time

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

# ---------------------------------------------------------------------------
# Rutas y constantes
# ---------------------------------------------------------------------------
AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.normpath(os.path.join(AQUI, "..", ".."))
DIR_STL = os.path.join(RAIZ, "impresion3d", "stl")
DIR_USO = os.path.join(RAIZ, "impresion3d", "renders", "_mallas_uso")

R_TUBO = 0.0267 / 2.0          # radio exterior PVC 3/4" SCH40 (m)
R_COPLE = 0.0165               # radio exterior de un cople/codo
VFOV_D435 = 42.5               # campo de visión vertical del sensor de color (grados)

ROJO_FUEGO = (0xC8 / 255, 0x1E / 255, 0x1E / 255)
AMARILLO_FUEGO = (0xF5 / 255, 0xC4 / 255, 0x00 / 255)

CLASE_FUEGO, CLASE_TOMA = 0, 1
NOMBRES = {CLASE_FUEGO: "fuego", CLASE_TOMA: "toma"}


def srgb_a_lineal(c):
    c = np.asarray(c, dtype=float)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def color_lin(rgb, a=1.0):
    r, g, b = srgb_a_lineal(rgb)
    return (float(r), float(g), float(b), a)


def leer_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser(description="Generador sintético YOLO (Blender)")
    ap.add_argument("--salida", default=os.path.join(RAIZ, "vision", "datos", "sintetico_prueba"))
    ap.add_argument("--n", type=int, default=600)
    ap.add_argument("--semilla", type=int, default=1)
    ap.add_argument("--vistas", type=int, default=6)
    ap.add_argument("--motor", choices=["cycles", "eevee"], default="cycles")
    ap.add_argument("--muestras", type=int, default=24)
    ap.add_argument("--res", choices=["mix", "640", "1280"], default="mix")
    ap.add_argument("--frac_val", type=float, default=0.15)
    ap.add_argument("--sin_post", action="store_true")
    ap.add_argument("--guardar_blend", action="store_true")
    a = ap.parse_args(argv)
    if not os.path.isabs(a.salida):
        a.salida = os.path.join(os.getcwd(), a.salida)
    return a


# ---------------------------------------------------------------------------
# Utilidades de mallas
# ---------------------------------------------------------------------------
def malla_desde_arrays(nombre, verts, tris, mat_idx=None):
    """Crea un Mesh a partir de arrays numpy (verts Nx3, tris Mx3)."""
    me = bpy.data.meshes.new(nombre)
    me.vertices.add(len(verts))
    me.vertices.foreach_set("co", np.asarray(verts, dtype=np.float32).ravel())
    nt = len(tris)
    me.loops.add(nt * 3)
    me.loops.foreach_set("vertex_index", np.asarray(tris, dtype=np.int32).ravel())
    me.polygons.add(nt)
    me.polygons.foreach_set("loop_start", np.arange(0, nt * 3, 3, dtype=np.int32))
    if mat_idx is not None:
        me.polygons.foreach_set("material_index", np.asarray(mat_idx, dtype=np.int32))
    me.update(calc_edges=True)
    me.validate(clean_customdata=False)
    return me


def arrays_de_malla(me):
    me.calc_loop_triangles()
    v = np.empty(len(me.vertices) * 3, dtype=np.float32)
    me.vertices.foreach_get("co", v)
    t = np.empty(len(me.loop_triangles) * 3, dtype=np.int32)
    me.loop_triangles.foreach_get("vertices", t)
    return v.reshape(-1, 3), t.reshape(-1, 3)


def importar_stl(ruta):
    """Importa una STL (mm) y devuelve (verts en m, tris). No deja objetos en la escena."""
    antes = set(bpy.data.objects)
    bpy.ops.wm.stl_import(filepath=ruta, global_scale=0.001)
    nuevos = [o for o in bpy.data.objects if o not in antes]
    if not nuevos:
        raise RuntimeError("no se importó " + ruta)
    ob = nuevos[0]
    v, t = arrays_de_malla(ob.data)
    # el importador pone la escala en el objeto, no en los vértices: se aplica la matriz completa
    M = np.array(ob.matrix_world)
    v = v.astype(np.float64) @ M[:3, :3].T + M[:3, 3]
    me = ob.data
    bpy.data.objects.remove(ob, do_unlink=True)
    bpy.data.meshes.remove(me)
    return v, t


def suavizar(me, ang=35.0):
    try:
        me.shade_smooth()
        me.set_sharp_from_angle(angle=math.radians(ang))
    except Exception:
        pass


def bm_a_malla(bm, nombre):
    me = bpy.data.meshes.new(nombre)
    bm.to_mesh(me)
    bm.free()
    return me


def bm_cilindro(bm, r1, r2, h, segs, matriz, mat=0):
    res = bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=segs, radius1=r1, radius2=r2,
                                depth=h, matrix=matriz)
    for v in res["verts"]:
        for f in v.link_faces:
            f.material_index = mat
    return res


def bm_caja(bm, sx, sy, sz, matriz, mat=0):
    res = bmesh.ops.create_cube(bm, size=1.0, matrix=matriz @ Matrix.Diagonal((sx, sy, sz, 1.0)))
    for v in res["verts"]:
        for f in v.link_faces:
            f.material_index = mat
    return res


def bm_esfera(bm, r, matriz, mat=0, segs=12):
    res = bmesh.ops.create_uvsphere(bm, u_segments=segs, v_segments=max(6, segs // 2), radius=r, matrix=matriz)
    for v in res["verts"]:
        for f in v.link_faces:
            f.material_index = mat
    return res


def T(x=0.0, y=0.0, z=0.0):
    return Matrix.Translation((x, y, z))


def R(ang, eje):
    return Matrix.Rotation(ang, 4, eje)


# ---------------------------------------------------------------------------
# Texturas generadas con numpy
# ---------------------------------------------------------------------------
def imagen_desde_array(nombre, arr):
    """arr: HxWx4 float en [0,1] con el origen ARRIBA (se voltea para Blender)."""
    h, w = arr.shape[:2]
    img = bpy.data.images.new(nombre, w, h, alpha=True)
    img.pixels.foreach_set(np.ascontiguousarray(arr[::-1]).astype(np.float32).ravel())
    img.pack()
    return img


def textura_tapete(rng, S=2048):
    """2x2 losetas de tapete EVA tipo rompecabezas. Gris base ~1 (se tiñe en el shader)."""
    u = (np.arange(S) + 0.5) / S * 2.0           # 2 losetas por eje
    U, V = np.meshgrid(u, u)
    n_dientes = 5
    A = 0.028

    def diente(s):
        return np.tanh(5.0 * np.sin(2 * np.pi * n_dientes * s)) / np.tanh(5.0)

    du = A * diente(V)
    dv = A * diente(U + 0.1)
    uu = U - du
    vv = V - dv
    fu = np.abs(uu - np.round(uu))
    fv = np.abs(vv - np.round(vv))
    w = 0.0022
    junta = np.clip(1.0 - np.minimum(fu, fv) / w, 0.0, 1.0)
    # brillo por loseta
    idx = (np.floor(uu).astype(int) % 2) + 2 * (np.floor(vv).astype(int) % 2)
    tono = 1.0 + rng.uniform(-0.07, 0.07, size=4)
    base = tono[idx]
    # grano fino del EVA
    grano = rng.normal(0.0, 0.035, size=(S // 4, S // 4))
    grano = np.kron(grano, np.ones((4, 4)))
    base = base * (1.0 + grano) * (1.0 - 0.55 * junta)
    alt = 1.0 - junta  # para el bump
    out = np.zeros((S, S, 4), dtype=np.float32)
    out[..., 0] = base
    out[..., 1] = base
    out[..., 2] = base
    out[..., 3] = alt
    out[..., :3] = np.clip(out[..., :3] * 0.8, 0, 1)
    return out


def textura_flama(rng, S=512, fondo_blanco=False):
    """Calcomanía de flama (dibujo plano rojo/naranja/amarillo)."""
    y, x = np.mgrid[0:S, 0:S].astype(np.float32) / S
    yy = 1.0 - y  # hacia arriba
    rgba = np.zeros((S, S, 4), dtype=np.float32)

    def flama(cx, ancho, alto, inclin, ondas):
        t = np.clip((yy - 0.06) / alto, 0, 1)
        hw = ancho * np.sqrt(np.clip(t * 5, 0, 1)) * (1 - t) ** 0.9
        hw = np.where((yy > 0.06) & (yy < 0.06 + alto), hw, 0)
        off = inclin * t ** 2 + 0.03 * np.sin(ondas * t * np.pi)
        return np.abs(x - cx - off) < hw

    capas = [((0.85, 0.12, 0.08), 0.30, 0.85), ((0.98, 0.45, 0.05), 0.20, 0.62), ((1.0, 0.85, 0.1), 0.11, 0.42)]
    for k, (col, anc, alt) in enumerate(capas):
        m = np.zeros((S, S), dtype=bool)
        n_leng = rng.integers(1, 4) if k < 2 else 1
        for _ in range(n_leng):
            m |= flama(0.5 + rng.uniform(-0.08, 0.08), anc * rng.uniform(0.7, 1.1), alt * rng.uniform(0.8, 1.05),
                       rng.uniform(-0.15, 0.15), rng.uniform(1, 4))
        rgba[m, :3] = col
        rgba[m, 3] = 1.0
    if fondo_blanco:
        borde = (x > 0.03) & (x < 0.97) & (y > 0.03) & (y < 0.97)
        sin = rgba[..., 3] == 0
        rgba[sin & borde, :3] = 0.95
        rgba[borde, 3] = 1.0
    return rgba


# ---------------------------------------------------------------------------
# Materiales
# ---------------------------------------------------------------------------
def ent(nodo, nombre, tipo="RGBA"):
    """Entrada de un nodo por nombre y tipo (el nodo Mix repite nombres para cada tipo de dato)."""
    for sk in nodo.inputs:
        if sk.name == nombre and sk.type == tipo and sk.enabled:
            return sk
    for sk in nodo.inputs:
        if sk.name == nombre and sk.type == tipo:
            return sk
    return nodo.inputs[nombre]


def sal(nodo, nombre, tipo="RGBA"):
    for sk in nodo.outputs:
        if sk.name == nombre and sk.type == tipo:
            return sk
    return nodo.outputs[nombre]


def nodos_basicos(nombre):
    m = bpy.data.materials.new(nombre)
    m.use_nodes = True
    nt = m.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
    nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    return m, nt, bsdf, out


def mat_color_objeto(nombre, rough=0.45, spec=0.5, bump=0.0, escala_bump=800.0, metal=0.0):
    """Material cuyo color base viene de Object.color (variación por objeto sin crear materiales)."""
    m, nt, bsdf, out = nodos_basicos(nombre)
    info = nt.nodes.new("ShaderNodeObjectInfo")
    nt.links.new(info.outputs["Color"], bsdf.inputs["Base Color"])
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    try:
        bsdf.inputs["Specular IOR Level"].default_value = spec
    except Exception:
        pass
    if bump > 0:
        tc = nt.nodes.new("ShaderNodeTexCoord")
        nz = nt.nodes.new("ShaderNodeTexNoise")
        nz.inputs["Scale"].default_value = escala_bump
        nt.links.new(tc.outputs["Object"], nz.inputs["Vector"])
        bp = nt.nodes.new("ShaderNodeBump")
        bp.inputs["Strength"].default_value = bump
        bp.inputs["Distance"].default_value = 0.0005
        nt.links.new(nz.outputs["Fac"], bp.inputs["Height"])
        nt.links.new(bp.outputs["Normal"], bsdf.inputs["Normal"])
    return m


def mat_amarillo_fijo():
    """Amarillo de la flama: color fijo con ligera variación por objeto (Object Info > Random)."""
    m, nt, bsdf, out = nodos_basicos("pla_amarillo")
    info = nt.nodes.new("ShaderNodeObjectInfo")
    mix = nt.nodes.new("ShaderNodeMix")
    mix.data_type = "RGBA"
    ent(mix, "A").default_value = color_lin((0.93, 0.72, 0.0))
    ent(mix, "B").default_value = color_lin((1.0, 0.82, 0.08))
    nt.links.new(info.outputs["Random"], ent(mix, "Factor", "VALUE"))
    nt.links.new(sal(mix, "Result"), bsdf.inputs["Base Color"])
    bsdf.inputs["Roughness"].default_value = 0.42
    return m


def mat_tapete(img):
    m, nt, bsdf, out = nodos_basicos("tapete_eva")
    tc = nt.nodes.new("ShaderNodeTexCoord")
    mp = nt.nodes.new("ShaderNodeMapping")
    mp.name = "map_tapete"
    nt.links.new(tc.outputs["Object"], mp.inputs["Vector"])
    tex = nt.nodes.new("ShaderNodeTexImage")
    tex.image = img
    tex.interpolation = "Linear"
    nt.links.new(mp.outputs["Vector"], tex.inputs["Vector"])
    info = nt.nodes.new("ShaderNodeObjectInfo")
    mul = nt.nodes.new("ShaderNodeMix")
    mul.data_type = "RGBA"
    mul.blend_type = "MULTIPLY"
    ent(mul, "Factor", "VALUE").default_value = 1.0
    nt.links.new(tex.outputs["Color"], ent(mul, "A"))
    nt.links.new(info.outputs["Color"], ent(mul, "B"))
    nt.links.new(sal(mul, "Result"), bsdf.inputs["Base Color"])
    bsdf.inputs["Roughness"].default_value = 0.85
    try:
        bsdf.inputs["Specular IOR Level"].default_value = 0.3
    except Exception:
        pass
    # bump: juntas + grano
    nz = nt.nodes.new("ShaderNodeTexNoise")
    nz.inputs["Scale"].default_value = 900.0
    nt.links.new(tc.outputs["Object"], nz.inputs["Vector"])
    suma = nt.nodes.new("ShaderNodeMath")
    suma.operation = "MULTIPLY_ADD"
    nt.links.new(nz.outputs["Fac"], suma.inputs[0])
    suma.inputs[1].default_value = 0.15
    nt.links.new(tex.outputs["Alpha"], suma.inputs[2])
    bp = nt.nodes.new("ShaderNodeBump")
    bp.inputs["Strength"].default_value = 0.5
    bp.inputs["Distance"].default_value = 0.002
    nt.links.new(suma.outputs["Value"], bp.inputs["Height"])
    nt.links.new(bp.outputs["Normal"], bsdf.inputs["Normal"])
    return m


def mat_piso_lab():
    m, nt, bsdf, out = nodos_basicos("piso_lab")
    tc = nt.nodes.new("ShaderNodeTexCoord")
    nz = nt.nodes.new("ShaderNodeTexNoise")
    nz.inputs["Scale"].default_value = 3.0
    nz.inputs["Detail"].default_value = 8.0
    nt.links.new(tc.outputs["Object"], nz.inputs["Vector"])
    info = nt.nodes.new("ShaderNodeObjectInfo")
    mul = nt.nodes.new("ShaderNodeMix")
    mul.data_type = "RGBA"
    mul.blend_type = "MULTIPLY"
    ent(mul, "Factor", "VALUE").default_value = 0.35
    nt.links.new(info.outputs["Color"], ent(mul, "A"))
    nt.links.new(nz.outputs["Color"], ent(mul, "B"))
    nt.links.new(sal(mul, "Result"), bsdf.inputs["Base Color"])
    bsdf.inputs["Roughness"].default_value = 0.6
    return m


def mat_calcomania(img):
    m, nt, bsdf, out = nodos_basicos("calco_" + img.name)
    tex = nt.nodes.new("ShaderNodeTexImage")
    tex.image = img
    tc = nt.nodes.new("ShaderNodeTexCoord")
    nt.links.new(tc.outputs["UV"], tex.inputs["Vector"])
    nt.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    bsdf.inputs["Roughness"].default_value = 0.35
    tr = nt.nodes.new("ShaderNodeBsdfTransparent")
    mix = nt.nodes.new("ShaderNodeMixShader")
    nt.links.new(tex.outputs["Alpha"], mix.inputs["Fac"])
    nt.links.new(tr.outputs["BSDF"], mix.inputs[1])
    nt.links.new(bsdf.outputs["BSDF"], mix.inputs[2])
    nt.links.new(mix.outputs["Shader"], out.inputs["Surface"])
    try:
        m.blend_method = "HASHED"
    except Exception:
        pass
    return m


def mat_helice():
    """Disco de hélice girando: gris oscuro semitransparente."""
    m, nt, bsdf, out = nodos_basicos("helice")
    bsdf.inputs["Base Color"].default_value = (0.02, 0.02, 0.02, 1)
    tr = nt.nodes.new("ShaderNodeBsdfTransparent")
    mix = nt.nodes.new("ShaderNodeMixShader")
    mix.inputs["Fac"].default_value = 0.35
    nt.links.new(tr.outputs["BSDF"], mix.inputs[1])
    nt.links.new(bsdf.outputs["BSDF"], mix.inputs[2])
    nt.links.new(mix.outputs["Shader"], out.inputs["Surface"])
    return m


# ---------------------------------------------------------------------------
# Biblioteca de piezas (STL o procedurales)
# ---------------------------------------------------------------------------
class Pieza:
    """Malla de biblioteca + vértices locales (para etiquetas)."""

    def __init__(self, nombre, malla, clase, radio_xy, fuente):
        self.nombre = nombre
        self.malla = malla
        self.clase = clase
        self.radio_xy = radio_xy
        self.fuente = fuente
        v = np.empty(len(malla.vertices) * 3, dtype=np.float32)
        malla.vertices.foreach_get("co", v)
        self.verts = v.reshape(-1, 3).astype(np.float64)
        malla.use_fake_user = True


def fuego_procedural(rng, k):
    bm = bmesh.new()
    rb = 0.045 + 0.005 * k
    bm_cilindro(bm, rb, rb * 0.97, 0.004, 24, T(0, 0, 0.002), mat=0)
    bm_cilindro(bm, 0.012, 0.012, 0.007, 16, T(0, 0, 0.0075), mat=0)
    n = 5 + k
    for i in range(n):
        a = 2 * math.pi * i / n
        h = rng.uniform(0.04, 0.065)
        m = T(0.028 * math.cos(a), 0.028 * math.sin(a), h / 2) @ R(a, "Z") @ R(0.25, "Y")
        bm_cilindro(bm, 0.009, 0.0005, h, 10, m, mat=0)
    bm_cilindro(bm, 0.013, 0.0005, 0.10, 12, T(0, 0, 0.055), mat=1)
    me = bm_a_malla(bm, "fuego_proc_%d" % k)
    suavizar(me)
    return me


def valvula_procedural():
    bm = bmesh.new()
    # coordenadas de uso (m): tubo a lo largo de Y con centro en (0, y, R_TUBO)
    bm_caja(bm, 0.007, 0.030, 0.070, T(-0.0183, 0, 0.031))        # lomo
    bm_caja(bm, 0.040, 0.030, 0.004, T(0.0, 0, -0.002))           # mandíbula
    bm_caja(bm, 0.039, 0.030, 0.014, T(-0.002, 0, 0.058))         # brazo
    bm_cilindro(bm, 0.015, 0.015, 0.006, 8, T(0, 0, 0.068))        # niple
    bm_caja(bm, 0.046, 0.030, 0.028, T(0, 0, 0.085))              # cuerpo
    bm_caja(bm, 0.074, 0.030, 0.020, T(0, 0, 0.085))              # puertos
    bm_caja(bm, 0.026, 0.030, 0.010, T(0, 0, 0.104))              # bonete
    bm_cilindro(bm, 0.008, 0.008, 0.10, 12, T(0, 0, 0.07))         # vástago
    bm_cilindro(bm, 0.012, 0.012, 0.013, 16, T(0, 0, 0.118))       # cubo
    bmesh.ops.create_circle(bm, cap_ends=False, segments=24, radius=0.033, matrix=T(0, 0, 0.122))
    for i in range(5):
        a = 2 * math.pi * i / 5
        bm_caja(bm, 0.030, 0.007, 0.006, T(0.016 * math.cos(a), 0.016 * math.sin(a), 0.122) @ R(a, "Z"))
    res = bmesh.ops.create_cone(bm, cap_ends=True, segments=24, radius1=0.033, radius2=0.033, depth=0.008,
                                matrix=T(0, 0, 0.122))
    bmesh.ops.delete(bm, geom=[f for f in bm.faces if len(f.verts) == 24], context="FACES_ONLY")
    bm_caja(bm, 0.027, 0.028, 0.012, T(0, 0, 0.032))              # zapata
    me = bm_a_malla(bm, "valvula_proc")
    return me


DZ_ENSAMBLE_DEF = 0.0036   # impresion3d/cad/fuegos.py: DZ_ENSAMBLE = ESPESOR_PLACA = 3.6 mm


def dz_ensamble_amarilla(va, ruta_mono):
    """Altura (m) a la que va la pieza amarilla sobre la roja. La amarilla se exporta en orientación de
    impresión (base en z = 0); el ensamble real se deduce del STL monocolor (que ya viene ensamblado): su punta
    más alta coincide con la punta amarilla. Si no hay monocolor, se usa DZ_ENSAMBLE de fuegos.py (3.6 mm)."""
    try:
        if os.path.exists(ruta_mono):
            vm, _ = importar_stl(ruta_mono)
            dz = float(vm[:, 2].max() - va[:, 2].max())
            if -0.001 <= dz <= 0.02:
                return dz
            print("AVISO: dz de ensamble raro (%.4f m); uso %.4f" % (dz, DZ_ENSAMBLE_DEF))
    except Exception as e:
        print("AVISO: no pude deducir el ensamble del monocolor:", e)
    return DZ_ENSAMBLE_DEF - float(va[:, 2].min())


CABECERA_META = "imagen,split,escena,vista,W,H,altura_m,n_fuego,n_toma,seg_render,fuente_fuego,fuente_toma\n"


def cargar_fuegos(rng):
    piezas = []
    for k, letra in enumerate("ABC"):
        rojo = os.path.join(DIR_STL, "fuego_%s_rojo.stl" % letra)
        amar = os.path.join(DIR_STL, "fuego_%s_amarillo.stl" % letra)
        mono = os.path.join(DIR_STL, "fuego_%s_monocolor.stl" % letra)
        if os.path.exists(rojo) and os.path.exists(amar):
            try:
                vr, tr = importar_stl(rojo)
                va, ta = importar_stl(amar)
                va = va + np.array([0, 0, dz_ensamble_amarilla(va, mono)])
                v = np.vstack([vr, va])
                t = np.vstack([tr, ta + len(vr)])
                mi = np.concatenate([np.zeros(len(tr)), np.ones(len(ta))])
                me = malla_desde_arrays("fuego_%s" % letra, v, t, mi)
                suavizar(me)
                r = float(np.max(np.linalg.norm(v[:, :2], axis=1)))
                piezas.append(Pieza("fuego_%s" % letra, me, CLASE_FUEGO, r, "stl"))
            except Exception as e:
                print("AVISO: no se pudo cargar fuego", letra, e)
        if os.path.exists(mono):
            try:
                v, t = importar_stl(mono)
                me = malla_desde_arrays("fuego_%s_mono" % letra, v, t, np.zeros(len(t)))
                suavizar(me)
                r = float(np.max(np.linalg.norm(v[:, :2], axis=1)))
                p = Pieza("fuego_%s_mono" % letra, me, CLASE_FUEGO, r, "stl")
                p.mono = True
                piezas.append(p)
            except Exception as e:
                print("AVISO: no se pudo cargar fuego mono", letra, e)
    if not [p for p in piezas if not getattr(p, "mono", False)]:
        print("AVISO: faltan STL de fuegos; se usan modelos procedurales")
        for k in range(3):
            piezas.append(Pieza("fuego_proc_%d" % k, fuego_procedural(rng, k), CLASE_FUEGO, 0.05, "procedural"))
    return piezas


# cotas de valvula.py (mm) para re-orientar las STL de impresión si no hay mallas de uso
_D = 30.0
_Z_MAND_INF = -4.0


def cargar_valvula():
    """Devuelve la válvula ensamblada en coordenadas de USO (m): tubo a lo largo de Y, centro del tubo en
    (0, y, R_TUBO), z = 0 = cara superior de la mandíbula, C abre hacia +X, vástago hacia +Z."""
    partes_uso = ["cuerpo.stl", "tuerca.stl", "vastago.stl", "zapata_balero.stl"]
    rutas = [os.path.join(DIR_USO, p) for p in partes_uso]
    vs, ts = [], []
    fuente = None
    if all(os.path.exists(r) for r in rutas):
        try:
            for r in rutas:
                v, t = importar_stl(r)
                ts.append(t + sum(len(x) for x in vs))
                vs.append(v)
            fuente = "stl_uso"
        except Exception as e:
            print("AVISO: mallas de uso de la válvula inválidas:", e)
            vs, ts = [], []
    if fuente is None:
        imp = {k: os.path.join(DIR_STL, "valvula_%s.stl" % k) for k in
               ("cuerpo", "tuerca", "vastago_volante", "zapata_balero")}
        if all(os.path.exists(r) for r in imp.values()):
            try:
                # Inversa de las transformaciones de impresión de valvula.py (aprox. por caja envolvente)
                vc, tc = importar_stl(imp["cuerpo"])        # impresión: (x, -z, y) centrado
                vc = vc * 1000.0
                zmin_u, zmax_u = _Z_MAND_INF, _Z_MAND_INF + (vc[:, 1].max() - vc[:, 1].min())
                cu = np.stack([vc[:, 0], vc[:, 2] - _D / 2, (zmin_u + zmax_u) / 2 - vc[:, 1]], 1)
                vv, tv = importar_stl(imp["vastago_volante"])  # Rot180X, volante sobre la cama
                vv = vv * 1000.0
                altura_v = vv[:, 2].max()
                z_cubo_sup = zmax_u + 6.0 + 13.0            # bonete + holgura + cubo (aprox.)
                va = np.stack([vv[:, 0], -vv[:, 1], z_cubo_sup - vv[:, 2]], 1)
                vt, tt = importar_stl(imp["tuerca"])
                vt = vt * 1000.0
                vt = vt + np.array([0, 0, zmax_u - 10.0 - 4.0 - 6.0 - 28.0 - 10.0 - 4.0 - 10.2])
                vz, tz = importar_stl(imp["zapata_balero"])
                vz = vz * 1000.0
                z_zap_sup = 26.67 + 3.5 + 7.0
                vz = np.stack([vz[:, 0], -vz[:, 1], z_zap_sup - vz[:, 2]], 1)
                for v, t in ((cu, tc), (vt, tt), (va, tv), (vz, tz)):
                    ts.append(t + sum(len(x) for x in vs))
                    vs.append(v / 1000.0)
                fuente = "stl_impresion"
            except Exception as e:
                print("AVISO: STL de impresión de la válvula inválidas:", e)
                vs, ts = [], []
    if fuente is None:
        print("AVISO: faltan STL de la válvula; se usa modelo procedural")
        me = valvula_procedural()
        fuente = "procedural"
    else:
        v = np.vstack(vs)
        t = np.vstack(ts)
        me = malla_desde_arrays("valvula", v, t, np.zeros(len(t)))
    suavizar(me, 30)
    v = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", v)
    v = v.reshape(-1, 3)
    r = float(np.max(np.linalg.norm(v[:, :2], axis=1)))
    return Pieza("valvula", me, CLASE_TOMA, r, fuente)


# ---------------------------------------------------------------------------
# Generador
# ---------------------------------------------------------------------------
class Generador:
    def __init__(self, a):
        self.a = a
        self.rng0 = np.random.default_rng(a.semilla)
        bpy.ops.wm.read_factory_settings(use_empty=True)
        self.sc = bpy.context.scene
        self.configurar_render()
        self.crear_materiales()
        self.mallas_base()
        self.fuegos = cargar_fuegos(self.rng0)
        self.valvula = cargar_valvula()
        # OJO: Mesh.materials.clear() borra los índices de material; se guardan y se reponen
        for p in self.fuegos + [self.valvula]:
            me = p.malla
            mi = np.zeros(len(me.polygons), dtype=np.int32)
            me.polygons.foreach_get("material_index", mi)
            me.materials.clear()
            me.materials.append(self.m_pla)
            if p.clase == CLASE_FUEGO:
                me.materials.append(self.m_amar)
            me.polygons.foreach_set("material_index", mi)
            me.update()
        print("PIEZAS:", [(p.nombre, p.fuente, len(p.verts)) for p in self.fuegos],
              (self.valvula.nombre, self.valvula.fuente, len(self.valvula.verts)))
        self.col = bpy.data.collections.new("escena")
        self.sc.collection.children.link(self.col)
        self.mallas_temp = []
        self.crear_fijos()

    # ----------------------------------------------------------- render
    def configurar_render(self):
        sc = self.sc
        r = sc.render
        if self.a.motor == "cycles":
            r.engine = "CYCLES"
            prefs = bpy.context.preferences.addons["cycles"].preferences
            dispositivo = "CPU"
            for tipo in ("OPTIX", "CUDA"):
                try:
                    prefs.compute_device_type = tipo
                    prefs.get_devices()
                    gpus = [d for d in prefs.devices if d.type == tipo]
                    if gpus:
                        for d in prefs.devices:
                            d.use = d.type == tipo
                        dispositivo = tipo
                        break
                except Exception:
                    pass
            sc.cycles.device = "GPU" if dispositivo != "CPU" else "CPU"
            print("CYCLES dispositivo:", dispositivo)
            sc.cycles.samples = self.a.muestras
            sc.cycles.use_adaptive_sampling = True
            sc.cycles.adaptive_threshold = 0.03
            sc.cycles.use_denoising = True
            try:
                sc.cycles.denoiser = "OPTIX" if dispositivo == "OPTIX" else "OPENIMAGEDENOISE"
            except Exception:
                pass
            sc.cycles.max_bounces = 4
            sc.cycles.diffuse_bounces = 2
            sc.cycles.glossy_bounces = 2
            sc.cycles.transparent_max_bounces = 6
            sc.cycles.caustics_reflective = False
            sc.cycles.caustics_refractive = False
            r.use_persistent_data = True
        else:
            r.engine = "BLENDER_EEVEE"
        r.image_settings.file_format = "PNG"
        r.image_settings.color_mode = "RGB"
        r.resolution_percentage = 100
        r.film_transparent = False
        try:
            sc.view_settings.look = "None"
        except Exception:
            pass
        self.tmp_png = os.path.join(self.a.salida, "_tmp_render_%d.png" % os.getpid())

    def crear_materiales(self):
        rng = self.rng0
        self.img_tapete = imagen_desde_array("tapete", textura_tapete(rng))
        self.m_tapete = mat_tapete(self.img_tapete)
        self.m_piso = mat_piso_lab()
        self.m_pvc = mat_color_objeto("pvc", rough=0.3, spec=0.6)
        self.m_pla = mat_color_objeto("pla", rough=0.45, spec=0.5, bump=0.15, escala_bump=1500)
        self.m_amar = mat_amarillo_fijo()
        self.m_mate = mat_color_objeto("mate", rough=0.8, spec=0.3)
        self.m_goma = mat_color_objeto("goma", rough=0.7, spec=0.3)
        self.m_hoja = mat_color_objeto("hoja", rough=0.55, spec=0.4)
        self.m_metal = mat_color_objeto("metal", rough=0.35, spec=0.5, metal=0.6)
        self.m_helice = mat_helice()
        self.m_negro, _nt, _b, _o = nodos_basicos("negro_fijo")
        _b.inputs["Base Color"].default_value = (0.015, 0.015, 0.015, 1)
        _b.inputs["Roughness"].default_value = 0.7
        self.m_calcos = []
        for i in range(8):
            img = imagen_desde_array("flama_%d" % i, textura_flama(rng, fondo_blanco=(i % 2 == 1)))
            self.m_calcos.append(mat_calcomania(img))

    def mallas_base(self):
        bm = bmesh.new()
        bmesh.ops.create_cone(bm, cap_ends=True, segments=32, radius1=1.0, radius2=1.0, depth=1.0)
        self.me_cil = bm_a_malla(bm, "cil_unit")
        suavizar(self.me_cil, 40)
        self.me_cil.use_fake_user = True
        bm = bmesh.new()
        bmesh.ops.create_uvsphere(bm, u_segments=24, v_segments=12, radius=1.0)
        self.me_esf = bm_a_malla(bm, "esf_unit")
        for p in self.me_esf.polygons:
            p.use_smooth = True
        self.me_esf.use_fake_user = True
        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=1.0)
        self.me_caja = bm_a_malla(bm, "caja_unit")
        self.me_caja.use_fake_user = True
        bm = bmesh.new()
        bmesh.ops.create_grid(bm, x_segments=1, y_segments=1, size=0.5, calc_uvs=True)
        self.me_plano = bm_a_malla(bm, "plano_unit")
        self.me_plano.use_fake_user = True

    def crear_fijos(self):
        sc = self.sc
        # cámara
        cd = bpy.data.cameras.new("cam")
        cd.sensor_fit = "VERTICAL"
        cd.angle_y = math.radians(VFOV_D435)
        cd.clip_start = 0.02
        cd.clip_end = 50
        self.cam = bpy.data.objects.new("cam", cd)
        sc.collection.objects.link(self.cam)
        sc.camera = self.cam
        # cuerpo del propio dron (solo proyecta sombra; invisible para la cámara)
        self.dron_sombra = self.crear_dron(sc.collection, "dron_propio")
        for o in [self.dron_sombra] + list(self.dron_sombra.children):
            o.visible_camera = False
        # mundo
        w = bpy.data.worlds.new("mundo")
        w.use_nodes = True
        self.bg = w.node_tree.nodes["Background"]
        sc.world = w
        # pisos
        self.tapete = self.obj(self.me_plano, self.m_tapete, col=sc.collection, nombre="tapete")
        self.piso = self.obj(self.me_plano, self.m_piso, col=sc.collection, nombre="piso_lab")

    # ----------------------------------------------------------- helpers
    def obj(self, malla, mat, loc=(0, 0, 0), rot=None, esc=(1, 1, 1), color=(1, 1, 1, 1), col=None, nombre="o"):
        """Crea un objeto. Si 'mat' no es None se asigna a nivel de OBJETO (las mallas unitarias se comparten
        entre materiales); si es None se usan los materiales de la malla."""
        o = bpy.data.objects.new(nombre, malla)
        (col or self.col).objects.link(o)
        if mat is not None:
            if len(malla.materials) == 0:
                malla.materials.append(mat)
            o.material_slots[0].link = "OBJECT"
            o.material_slots[0].material = mat
        if rot is not None:
            o.matrix_world = Matrix.Translation(Vector(loc)) @ rot.to_4x4() @ Matrix.Diagonal((*esc, 1.0))
        else:
            o.location = loc
            o.scale = esc
        o.color = color
        return o

    def cilindro_entre(self, p0, p1, r, mat, color, nombre="tubo"):
        p0, p1 = Vector(p0), Vector(p1)
        d = p1 - p0
        L = d.length
        rot = d.to_track_quat("Z", "Y").to_matrix()
        return self.obj(self.me_cil, mat, loc=(p0 + p1) / 2, rot=rot, esc=(r, r, L), color=color, nombre=nombre)

    def crear_dron(self, col, nombre):
        """QDrone 2 simplificado (~0.44 x 0.49 m): cuerpo, brazos, motores y discos de hélice."""
        bm = bmesh.new()
        bm_caja(bm, 0.20, 0.13, 0.07, T(0, 0, 0), mat=0)
        bm_caja(bm, 0.12, 0.09, 0.04, T(-0.03, 0, -0.05), mat=0)           # batería
        for sx in (1, -1):
            for sy in (1, -1):
                ang = math.atan2(sy * 0.175, sx * 0.155)
                bm_caja(bm, 0.235, 0.025, 0.02, T(sx * 0.155 / 2, sy * 0.175 / 2, 0) @ R(ang, "Z"), mat=0)
                bm_cilindro(bm, 0.018, 0.018, 0.03, 12, T(sx * 0.155, sy * 0.175, 0.02), mat=0)
                bm_cilindro(bm, 0.0635, 0.0635, 0.002, 24, T(sx * 0.155, sy * 0.175, 0.04), mat=1)
        bm_caja(bm, 0.03, 0.09, 0.025, T(0.11, 0, -0.01), mat=0)          # RealSense
        me = bm_a_malla(bm, nombre)
        o = bpy.data.objects.new(nombre, me)
        col.objects.link(o)
        me.materials.append(self.m_mate)
        me.materials.append(self.m_helice)
        o.color = (0.02, 0.02, 0.022, 1)
        if col is not self.sc.collection:
            self.mallas_temp.append(me)
        return o

    # ----------------------------------------------------------- escena
    def limpiar_escena(self):
        for o in list(self.col.objects):
            bpy.data.objects.remove(o, do_unlink=True)
        for me in self.mallas_temp:
            try:
                bpy.data.meshes.remove(me)
            except Exception:
                pass
        self.mallas_temp = []

    def polilinea_n(self, rng):
        """Vértices de la tubería: N invertida aleatoria (o forma libre)."""
        if rng.random() < 0.8:
            a = rng.uniform(0.6, 1.3)
            b = rng.uniform(0.6, 1.3)
            v = np.array([[-a, b], [a, b], [-a, -b], [a, -b]], dtype=float)
            v += rng.uniform(-0.2, 0.2, size=v.shape)
            if rng.random() < 0.5:
                v[:, 1] *= -1
            if rng.random() < 0.3:      # sólo una parte de la N
                i0 = rng.integers(0, 2)
                v = v[i0:i0 + rng.integers(2, 4)]
        else:
            n = rng.integers(2, 6)
            v = rng.uniform(-1.3, 1.3, size=(n, 2))
        ang = rng.uniform(0, 2 * np.pi) if rng.random() < 0.5 else rng.integers(0, 4) * np.pi / 2 + \
            rng.uniform(-0.25, 0.25)
        c, s = np.cos(ang), np.sin(ang)
        v = v @ np.array([[c, s], [-s, c]])
        return v

    def construir(self, rng, idx):
        self.limpiar_escena()
        sc = self.sc
        ocupado = []   # (x, y, r)

        def libre(x, y, r, margen=0.01):
            return all((x - ox) ** 2 + (y - oy) ** 2 > (r + orr + margen) ** 2 for ox, oy, orr in ocupado)

        # --- pisos
        g = rng.uniform(0.06, 0.16)                 # gris del EVA (lineal)
        tinte = rng.uniform(0.97, 1.03, size=3)
        self.tapete.color = (g * tinte[0], g * tinte[1], g * tinte[2] * rng.uniform(0.97, 1.08), 1)
        ext = rng.uniform(1.7, 2.5)
        self.tapete.scale = (2 * ext, 2 * ext, 1)
        self.tapete.location = (rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), 0)
        self.tapete.rotation_euler = (0, 0, rng.integers(0, 4) * np.pi / 2)
        tam_loseta = rng.choice([0.5, 0.6, 0.6, 0.6, 0.62, 1.0])
        mp = self.m_tapete.node_tree.nodes["map_tapete"]
        e = 1.0 / (2 * tam_loseta) * (1.0 / (2 * ext))   # coords de objeto (plano unitario escalado)
        mp.inputs["Scale"].default_value = (2 * ext / (2 * tam_loseta), 2 * ext / (2 * tam_loseta), 1)
        mp.inputs["Location"].default_value = (rng.uniform(0, 1), rng.uniform(0, 1), 0)
        gp = rng.uniform(0.25, 0.6)
        self.piso.color = (gp, gp * rng.uniform(0.95, 1.02), gp * rng.uniform(0.9, 1.05), 1)
        self.piso.scale = (20, 20, 1)
        self.piso.location = (0, 0, -0.002)

        # --- tubería
        verts = self.polilinea_n(rng)
        blanco = rng.uniform(0.75, 0.92)
        col_pvc = (blanco, blanco * rng.uniform(0.97, 1.0), blanco * rng.uniform(0.9, 1.0), 1)
        self.m_pvc.node_tree.nodes["Principled BSDF"].inputs["Roughness"].default_value = rng.uniform(0.2, 0.5)
        segmentos = []
        coples = [np.array([v[0], v[1]]) for v in verts]
        for i in range(len(verts) - 1):
            p0 = np.array([verts[i][0], verts[i][1], R_TUBO])
            p1 = np.array([verts[i + 1][0], verts[i + 1][1], R_TUBO])
            L = np.linalg.norm(p1 - p0)
            if L < 0.15:
                continue
            u = (p1 - p0) / L
            # el tubo entra en el codo/cople
            self.cilindro_entre(p0 + u * 0.012, p1 - u * 0.012, R_TUBO, self.m_pvc, col_pvc)
            segmentos.append((p0, p1, L, u))
            # coples a lo largo (tramos comerciales)
            n_cop = int(L // rng.uniform(0.9, 1.6))
            for k in range(n_cop):
                t = (k + 1) / (n_cop + 1) + rng.uniform(-0.05, 0.05)
                c = p0 + u * L * t
                self.cilindro_entre(c - u * 0.021, c + u * 0.021, R_COPLE, self.m_pvc, col_pvc, "cople")
                ocupado.append((c[0], c[1], 0.03))
                coples.append(c[:2].copy())
        for i, v in enumerate(verts):
            c = np.array([v[0], v[1], R_TUBO])
            ocupado.append((c[0], c[1], 0.04))
            # codo: esfera + manguitos
            self.obj(self.me_esf, self.m_pvc, loc=tuple(c), esc=(R_COPLE * 1.05,) * 3, color=col_pvc, nombre="codo")
            for (p0, p1, L, u) in segmentos:
                if np.linalg.norm(p0 - c) < 1e-6:
                    self.cilindro_entre(c, c + u * 0.035, R_COPLE, self.m_pvc, col_pvc, "manguito")
                elif np.linalg.norm(p1 - c) < 1e-6:
                    self.cilindro_entre(c, c - u * 0.035, R_COPLE, self.m_pvc, col_pvc, "manguito")
        for (p0, p1, L, u) in segmentos:
            for t in np.linspace(0, 1, max(2, int(L / 0.08))):
                c = p0 + (p1 - p0) * t
                ocupado.append((c[0], c[1], R_TUBO + 0.005))
        etiquetables = []

        def punto_en_tubo(margen=0.08):
            for _ in range(30):
                p0, p1, L, u = segmentos[rng.integers(0, len(segmentos))]
                if L < 2 * margen + 0.05:
                    continue
                s = rng.uniform(margen, L - margen)
                return p0 + u * s, u
            return None, None

        # --- tomas (válvulas abrazadas al tubo)
        n_tomas = rng.choice([0, 1, 1, 2, 2, 3, 4]) if segmentos else 0
        for _ in range(n_tomas):
            for intento in range(25):
                c, u = punto_en_tubo(0.10)
                if c is None:
                    break
                # sólo choca con cosas que no sean el propio tubo: revisa contra otros objetos etiquetables
                if any(np.hypot(c[0] - o.location.x, c[1] - o.location.y) < 0.13 for o, _ in etiquetables):
                    continue
                if any(np.hypot(c[0] - q[0], c[1] - q[1]) < 0.07 for q in coples):
                    continue
                # ejes: Y local = dirección del tubo; X local = boca de la C; Z = arriba
                yl = Vector(u)
                if rng.random() < 0.5:
                    yl = -yl
                zl = Vector((0, 0, 1))
                xl = yl.cross(zl)
                rot = Matrix((xl, yl, zl)).transposed()
                rot = Matrix.Rotation(rng.uniform(-0.1, 0.1), 3, yl) @ rot      # ligero giro sobre el tubo
                centro_tubo = Vector(c)
                origen = centro_tubo - rot @ Vector((0, 0, R_TUBO))
                # 92 % rojo (la pieza real). 8 % otros colores de PLA para que el modelo aprenda la FORMA y no
                # sólo el color: negro, gris, azul o rojo oscuro. No se usa amarillo (es el color de los fuegos).
                rojo = ROJO_FUEGO if rng.random() < 0.92 else \
                    [(0.05, 0.05, 0.05), (0.6, 0.6, 0.6), (0.1, 0.25, 0.7), (0.4, 0.04, 0.04)][rng.integers(0, 4)]
                rojo = np.clip(np.array(rojo) * rng.uniform(0.85, 1.1), 0, 1)
                o = self.obj(self.valvula.malla, None, loc=tuple(origen), rot=rot,
                             color=color_lin(rojo), nombre="toma")
                etiquetables.append((o, self.valvula))
                ocupado.append((c[0], c[1], 0.06))
                break

        # --- fuegos
        n_fuegos = rng.choice([0, 1, 1, 2, 2, 3, 4])
        bicolor = [p for p in self.fuegos if not getattr(p, "mono", False)]
        mono = [p for p in self.fuegos if getattr(p, "mono", False)]
        for _ in range(n_fuegos):
            pz = bicolor[rng.integers(0, len(bicolor))]
            if mono and rng.random() < 0.15:
                pz = mono[rng.integers(0, len(mono))]
            esc = 1.0
            for intento in range(40):
                modo = rng.random()
                yaw = rng.uniform(0, 2 * np.pi)
                rot = Matrix.Rotation(yaw, 3, "Z")
                if segmentos and modo < 0.72:                 # junto al tubo
                    c, u = punto_en_tubo(0.05)
                    if c is None:
                        continue
                    nrm = np.array([-u[1], u[0], 0]) * (1 if rng.random() < 0.5 else -1)
                    d = R_TUBO + pz.radio_xy * rng.uniform(0.85, 1.0) + rng.uniform(0.0, 0.15)
                    x, y = c[0] + nrm[0] * d, c[1] + nrm[1] * d
                    z = 0.0
                    rad = pz.radio_xy * 0.8
                    if not libre(x, y, rad, 0.0):
                        continue
                elif segmentos and modo < 0.85:              # encima del tubo, inclinado
                    c, u = punto_en_tubo(0.1)
                    if c is None:
                        continue
                    x, y, z = c[0], c[1], 2 * R_TUBO - 0.004
                    rad = pz.radio_xy
                    if any(np.hypot(x - o.location.x, y - o.location.y) < 0.15 for o, _ in etiquetables):
                        continue
                    rot = Matrix.Rotation(rng.uniform(-0.3, 0.3), 3, Vector(u)) @ rot
                else:                                           # en cualquier lugar
                    x, y = rng.uniform(-1.5, 1.5, size=2)
                    z = 0.0
                    rad = pz.radio_xy
                    if not libre(x, y, rad):
                        continue
                if pz.fuente == "stl" and getattr(pz, "mono", False):
                    colm = [ROJO_FUEGO, (0.95, 0.35, 0.05), AMARILLO_FUEGO][rng.integers(0, 3)]
                else:
                    colm = ROJO_FUEGO
                colm = np.clip(np.array(colm) * rng.uniform(0.85, 1.1), 0, 1)
                o = self.obj(pz.malla, None, loc=(x, y, z), rot=rot, esc=(esc, esc, esc), color=color_lin(colm),
                             nombre="fuego")
                etiquetables.append((o, pz))
                ocupado.append((x, y, rad))
                break

        # --- distractores
        self.distractores(rng, libre, ocupado, segmentos)
        return etiquetables, verts, segmentos

    def distractores(self, rng, libre, ocupado, segmentos):
        def lugar(r, intentos=30, cerca=None):
            for _ in range(intentos):
                if cerca is not None and rng.random() < 0.6:
                    x, y = cerca + rng.normal(0, 0.35, size=2)
                else:
                    x, y = rng.uniform(-1.7, 1.7, size=2)
                if libre(x, y, r):
                    ocupado.append((x, y, r))
                    return x, y
            return None

        centro_tubo = None
        if segmentos:
            p0, p1, L, u = segmentos[rng.integers(0, len(segmentos))]
            centro_tubo = (p0[:2] + p1[:2]) / 2

        # plantas artificiales
        for _ in range(rng.integers(0, 4)):
            rp = rng.uniform(0.05, 0.12)
            pos = lugar(rp + 0.1, cerca=centro_tubo)
            if pos is None:
                continue
            self.planta(rng, pos, rp)
        # carritos de juguete
        for _ in range(rng.integers(0, 4)):
            L = rng.uniform(0.08, 0.30)
            pos = lugar(L / 2, cerca=centro_tubo)
            if pos is None:
                continue
            self.carrito(rng, pos, L)
        # cilindros grises
        for _ in range(rng.integers(0, 4)):
            r = rng.uniform(0.03, 0.15)
            pos = lugar(r, cerca=centro_tubo)
            if pos is None:
                continue
            h = rng.uniform(0.05, 0.6)
            gg = rng.uniform(0.15, 0.6)
            self.obj(self.me_cil, self.m_mate if rng.random() < 0.6 else self.m_metal, loc=(pos[0], pos[1], h / 2),
                     esc=(r, r, h), color=(gg, gg, gg * rng.uniform(0.95, 1.05), 1), nombre="cilindro")
        # cruces de cinta blanca y cintas de colores
        for _ in range(rng.integers(0, 6)):
            x, y = rng.uniform(-1.7, 1.7, size=2)
            L = rng.uniform(0.1, 0.45)
            w = rng.uniform(0.018, 0.05)
            yaw = rng.uniform(0, np.pi)
            if rng.random() < 0.7:
                b = rng.uniform(0.7, 0.95)
                col = (b, b, b * rng.uniform(0.92, 1.0), 1)
            else:
                col = [color_lin(c) for c in [(0.9, 0.8, 0.1), (0.1, 0.3, 0.8), (0.8, 0.1, 0.1),
                                              (0.1, 0.6, 0.2), (0.05, 0.05, 0.05)]][rng.integers(0, 5)]
            brazos = [0, np.pi / 2] if rng.random() < 0.7 else [0]
            for da in brazos:
                self.obj(self.me_caja, self.m_mate, loc=(x, y, 0.0003),
                         rot=Matrix.Rotation(yaw + da, 3, "Z"), esc=(L, w, 0.0006), color=col, nombre="cinta")
        # calcomanías planas de flama (FONDO, no se etiquetan)
        for _ in range(rng.integers(0, 4) if rng.random() < 0.6 else 0):
            s = rng.uniform(0.05, 0.25)
            pos = lugar(s / 2, cerca=centro_tubo)
            if pos is None:
                continue
            m = self.m_calcos[rng.integers(0, len(self.m_calcos))]
            self.obj(self.me_plano, m, loc=(pos[0], pos[1], 0.0008), rot=Matrix.Rotation(rng.uniform(0, 2 * np.pi), 3, "Z"),
                     esc=(s, s, 1), nombre="calcomania")
        # objetos rojos / amarillos / de colores (negativos difíciles) y papel
        for _ in range(rng.integers(0, 5)):
            tipo = rng.integers(0, 4)
            col = [(0.8, 0.08, 0.08), (0.95, 0.75, 0.05), (0.95, 0.4, 0.05), (0.1, 0.3, 0.8), (0.1, 0.6, 0.2),
                   (0.9, 0.9, 0.9), (0.05, 0.05, 0.05)][rng.integers(0, 7)]
            col = color_lin(np.clip(np.array(col) * rng.uniform(0.8, 1.15), 0, 1))
            s = rng.uniform(0.03, 0.15)
            pos = lugar(s, cerca=centro_tubo)
            if pos is None:
                continue
            yaw = Matrix.Rotation(rng.uniform(0, 2 * np.pi), 3, "Z")
            if tipo == 0:
                self.obj(self.me_caja, self.m_pla, loc=(pos[0], pos[1], s * 0.4), rot=yaw, esc=(s, s * 0.7, s * 0.8),
                         color=col, nombre="caja")
            elif tipo == 1:
                self.obj(self.me_cil, self.m_pla, loc=(pos[0], pos[1], s * 0.6), esc=(s * 0.4, s * 0.4, s * 1.2),
                         color=col, nombre="vaso")
            elif tipo == 2:
                self.obj(self.me_esf, self.m_pla, loc=(pos[0], pos[1], s * 0.4), esc=(s * 0.4,) * 3, color=col,
                         nombre="pelota")
            else:
                b = rng.uniform(0.75, 0.95)
                self.obj(self.me_caja, self.m_mate, loc=(pos[0], pos[1], 0.0004), rot=yaw, esc=(0.216, 0.279, 0.0004),
                         color=(b, b, b, 1), nombre="papel")
        # marcadores OptiTrack sueltos
        for _ in range(rng.integers(0, 6)):
            x, y = rng.uniform(-1.6, 1.6, size=2)
            self.obj(self.me_esf, self.m_metal, loc=(x, y, 0.007), esc=(0.007,) * 3, color=(0.7, 0.7, 0.7, 1),
                     nombre="marcador")
        # cables
        for _ in range(rng.integers(0, 3)):
            p = rng.uniform(-1.5, 1.5, size=2)
            ang = rng.uniform(0, 2 * np.pi)
            puntos = [p.copy()]
            for k in range(rng.integers(3, 9)):
                ang += rng.normal(0, 0.5)
                p = p + 0.12 * np.array([np.cos(ang), np.sin(ang)])
                puntos.append(p.copy())
            gc = rng.choice([0.02, 0.03, 0.8])
            for a, b in zip(puntos[:-1], puntos[1:]):
                self.cilindro_entre((a[0], a[1], 0.003), (b[0], b[1], 0.003), 0.003, self.m_goma, (gc, gc, gc, 1),
                                    "cable")
        # el otro QDrone 2 (visible debajo del explorador; caso MPC apilado)
        self.otro_dron = None
        if rng.random() < 0.15:
            self.otro_dron = self.crear_dron(self.col, "dron_otro")

    def planta(self, rng, pos, rp):
        """Planta artificial: maceta (cilindro) + hojas alargadas que salen hacia arriba y afuera."""
        bm = bmesh.new()
        hm = rng.uniform(0.08, 0.2)
        n = rng.integers(12, 40)
        largo_max = rng.uniform(0.12, 0.4)
        for i in range(n):
            a = rng.uniform(0, 2 * np.pi)
            elev = rng.uniform(0.2, 1.3)
            L = largo_max * rng.uniform(0.5, 1.0)
            w = rng.uniform(0.012, 0.04)
            base = Vector((rp * 0.3 * np.cos(a), rp * 0.3 * np.sin(a), hm))
            d = Vector((np.cos(a) * np.cos(elev), np.sin(a) * np.cos(elev), np.sin(elev)))
            lado = Vector((-np.sin(a), np.cos(a), 0)) * w / 2
            punta = base + d * L - Vector((0, 0, L * 0.25 * rng.uniform(0, 1)))
            mid = base + d * L * 0.45
            vs = [bm.verts.new(base), bm.verts.new(mid + lado), bm.verts.new(punta), bm.verts.new(mid - lado)]
            bm.faces.new(vs)
        me = bm_a_malla(bm, "planta")
        me.materials.append(self.m_hoja)
        self.mallas_temp.append(me)
        verde = color_lin(np.clip(np.array([0.15, 0.45, 0.12]) * rng.uniform(0.6, 1.3, size=3), 0, 1))
        o = self.obj(me, None, loc=(pos[0], pos[1], 0), rot=Matrix.Rotation(rng.uniform(0, 6.3), 3, "Z"),
                     color=verde, nombre="planta")
        maceta = [(0.03, 0.03, 0.03), (0.5, 0.18, 0.08), (0.8, 0.8, 0.8), (0.3, 0.3, 0.3)][rng.integers(0, 4)]
        self.obj(self.me_cil, self.m_mate, loc=(pos[0], pos[1], hm / 2), esc=(rp, rp, hm),
                 color=color_lin(maceta), nombre="maceta")
        return o

    def carrito(self, rng, pos, L):
        bm = bmesh.new()
        W, H = L * rng.uniform(0.4, 0.55), L * rng.uniform(0.3, 0.45)
        rw = H * 0.3
        bm_caja(bm, L * 0.62, W, H * 0.55, T(-L * 0.19, 0, rw + H * 0.3), mat=0)       # caja de carga
        bm_caja(bm, L * 0.34, W * 0.95, H * 0.8, T(L * 0.31, 0, rw + H * 0.42), mat=0)   # cabina
        for sx in (-0.33, 0.3):
            for sy in (1, -1):
                bm_cilindro(bm, rw, rw, W * 0.18, 14, T(sx * L, sy * W * 0.5, rw) @ R(np.pi / 2, "X"), mat=1)
        me = bm_a_malla(bm, "carrito")
        me.materials.append(self.m_pla)
        me.materials.append(self.m_negro)
        self.mallas_temp.append(me)
        col = [(0.8, 0.1, 0.1), (0.95, 0.8, 0.05), (0.1, 0.3, 0.8), (0.1, 0.6, 0.2), (0.9, 0.9, 0.9),
               (0.95, 0.45, 0.05), (0.2, 0.2, 0.2)][rng.integers(0, 7)]
        o = self.obj(me, None, loc=(pos[0], pos[1], 0), rot=Matrix.Rotation(rng.uniform(0, 6.3), 3, "Z"),
                     color=color_lin(col), nombre="carrito")
        return o

    # ----------------------------------------------------------- cámara y luces
    def poner_camara(self, rng, etiquetables, segmentos):
        u = rng.random()
        if u < 0.6:
            h = rng.uniform(0.3, 1.8)
        else:
            h = float(np.clip(rng.normal(1.5, 0.15), 0.3, 1.8))
        # huella aproximada en el piso
        mitad_y = h * math.tan(math.radians(VFOV_D435 / 2))
        mitad_x = mitad_y * 16 / 9
        r = rng.random()
        if etiquetables and r < 0.72:
            o, _ = etiquetables[rng.integers(0, len(etiquetables))]
            p = np.array([o.location.x, o.location.y])
            p = p + rng.uniform(-0.9, 0.9, size=2) * np.array([mitad_x, mitad_y]) * 0.8
        elif segmentos and r < 0.92:
            p0, p1, L, uu = segmentos[rng.integers(0, len(segmentos))]
            p = (p0 + (p1 - p0) * rng.uniform(0, 1))[:2] + rng.normal(0, 0.15, size=2)
        else:
            p = rng.uniform(-1.5, 1.5, size=2)
        yaw = rng.uniform(0, 2 * np.pi)
        tx, ty = np.radians(rng.normal(0, 4, size=2).clip(-10, 10))
        rot = Matrix.Rotation(yaw, 4, "Z") @ Matrix.Rotation(tx, 4, "X") @ Matrix.Rotation(ty, 4, "Y")
        self.cam.matrix_world = Matrix.Translation((p[0], p[1], h)) @ rot
        # cuerpo del propio dron: la cámara va 0.20 m adelante y 0.05 m abajo del centro; la "adelante" del dron
        # se toma como el eje "arriba" de la imagen (el yaw aleatorio cubre cualquier montaje)
        adelante = rot.to_3x3() @ Vector((0, 1, 0))
        centro = Vector((p[0], p[1], h)) - adelante * 0.20 + Vector((0, 0, 0.06))
        self.dron_sombra.matrix_world = Matrix.Translation(centro) @ Matrix.Rotation(yaw + np.pi / 2, 4, "Z")
        # otro dron debajo
        if self.otro_dron is not None:
            hz = rng.uniform(0.25, max(0.3, h - 0.35))
            if hz < h - 0.25:
                q = np.array(p) + rng.uniform(-1, 1, size=2) * np.array([mitad_x, mitad_y]) * (hz / h) * 0.9
                self.otro_dron.matrix_world = Matrix.Translation((q[0], q[1], hz)) @ \
                    Matrix.Rotation(rng.uniform(0, 6.3), 4, "Z")
            else:
                self.otro_dron.location = (10, 10, -5)
        return h

    def poner_luces(self, rng):
        for o in [o for o in self.sc.collection.objects if o.type == "LIGHT"]:
            ld = o.data
            bpy.data.objects.remove(o, do_unlink=True)
            bpy.data.lights.remove(ld)
        kelvin = rng.uniform(3200, 6800)
        base = self.color_kelvin(kelvin)
        n = rng.integers(1, 7)
        alto = rng.uniform(2.4, 3.6)
        cx, cy = self.cam.location.x, self.cam.location.y
        for i in range(n):
            ld = bpy.data.lights.new("panel%d" % i, "AREA")
            ld.shape = "RECTANGLE"
            ld.size = rng.choice([0.6, 1.2, 0.3])
            ld.size_y = rng.choice([0.6, 0.3, 1.2])
            ld.energy = rng.uniform(120, 700)
            c = np.clip(np.array(base) * rng.uniform(0.96, 1.04, size=3), 0, None)
            ld.color = tuple(c / c.max())
            o = bpy.data.objects.new("panel%d" % i, ld)
            self.sc.collection.objects.link(o)
            o.location = (cx + rng.uniform(-2.5, 2.5), cy + rng.uniform(-2.5, 2.5), alto)
        if rng.random() < 0.25:     # luz dura (ventana/sol o lámpara puntual)
            if rng.random() < 0.5:
                ld = bpy.data.lights.new("sol", "SUN")
                ld.energy = rng.uniform(1.0, 5.0)
                ld.angle = math.radians(rng.uniform(0.5, 5))
                o = bpy.data.objects.new("sol", ld)
                o.rotation_euler = (rng.uniform(0.1, 0.9), 0, rng.uniform(0, 6.3))
            else:
                ld = bpy.data.lights.new("foco", "POINT")
                ld.energy = rng.uniform(100, 600)
                ld.shadow_soft_size = rng.uniform(0.02, 0.2)
                o = bpy.data.objects.new("foco", ld)
                o.location = (cx + rng.uniform(-1.5, 1.5), cy + rng.uniform(-1.5, 1.5), rng.uniform(2.0, 3.2))
            self.sc.collection.objects.link(o)
        g = rng.uniform(0.25, 0.7)
        self.bg.inputs["Color"].default_value = (g, g, g * rng.uniform(0.9, 1.1), 1)
        self.bg.inputs["Strength"].default_value = rng.uniform(0.1, 0.9)
        try:
            self.sc.view_settings.view_transform = "AgX" if rng.random() < 0.15 else "Standard"
        except Exception:
            pass
        self.sc.view_settings.exposure = rng.uniform(-1.0, 0.3)

    @staticmethod
    def color_kelvin(k):
        t = k / 100.0
        r = 255 if t <= 66 else 329.7 * (t - 60) ** -0.1332
        g = 99.47 * math.log(t) - 161.12 if t <= 66 else 288.1 * (t - 60) ** -0.0755
        b = 255 if t >= 66 else (0 if t <= 19 else 138.5 * math.log(t - 10) - 305.04)
        return [min(max(c, 0), 255) / 255.0 for c in (r, g, b)]

    # ----------------------------------------------------------- etiquetas
    def proyectar(self, pts):
        """Proyecta puntos del mundo (Nx3) a coordenadas normalizadas (u derecha, v abajo) y profundidad."""
        cam = self.cam
        M = np.array(cam.matrix_world.inverted())
        P = pts @ M[:3, :3].T + M[:3, 3]
        z = -P[:, 2]
        frame = cam.data.view_frame(scene=self.sc)
        hx = max(abs(v.x) for v in frame)
        hy = max(abs(v.y) for v in frame)
        dz = abs(frame[0].z)
        with np.errstate(divide="ignore", invalid="ignore"):
            u = 0.5 + (P[:, 0] / z) * dz / (2 * hx)
            v = 0.5 - (P[:, 1] / z) * dz / (2 * hy)
        return u, v, z

    def rayo_visible(self, dg, origen, direc, dist, o):
        """True si el rayo cámara->punto llega al objeto 'o' sin chocar con otro objeto opaco.
        Los discos de hélice del otro dron (material semitransparente) no cuentan como oclusores."""
        org = Vector(origen)
        dv = Vector(direc)
        restante = dist - 0.002
        for _ in range(6):
            ok, loc, nrm, fi, hit, mat = self.sc.ray_cast(dg, org, dv, distance=restante)
            if not ok:
                return True
            orig = hit.original if hit.original is not None else hit
            if orig == o or hit.name == o.name:
                return True
            transparente = False
            try:
                me = orig.data
                if orig.type == "MESH" and fi < len(me.polygons):
                    mi = me.polygons[fi].material_index
                    m = orig.material_slots[mi].material if mi < len(orig.material_slots) else None
                    transparente = m is not None and (m.name.startswith("helice") or m.name.startswith("calco"))
            except Exception:
                transparente = False
            if not transparente:
                return False
            avance = (loc - org).length + 0.001
            org = loc + dv * 0.001
            restante -= avance
            if restante <= 0:
                return True
        return True

    def etiquetas(self, etiquetables, W, H, rng):
        dg = bpy.context.evaluated_depsgraph_get()
        cam_pos = np.array(self.cam.matrix_world.translation)
        filas = []
        info = []
        for o, pz in etiquetables:
            Mw = np.array(o.matrix_world)
            pts = pz.verts @ Mw[:3, :3].T + Mw[:3, 3]
            u, v, z = self.proyectar(pts)
            delante = z > 0.02
            if not delante.any():
                continue
            u, v, pts_d = u[delante], v[delante], pts[delante]
            x1, x2, y1, y2 = u.min(), u.max(), v.min(), v.max()
            area_total = max((x2 - x1) * (y2 - y1), 1e-12)
            dentro = (u >= 0) & (u <= 1) & (v >= 0) & (v <= 1)
            if dentro.sum() == 0:
                continue
            cx1, cx2, cy1, cy2 = max(x1, 0), min(x2, 1), max(y1, 0), min(y2, 1)
            if cx2 <= cx1 or cy2 <= cy1:
                continue
            frac_marco = (cx2 - cx1) * (cy2 - cy1) / area_total
            # oclusión por ray casting sobre una muestra de vértices dentro del cuadro
            idx = np.flatnonzero(dentro)
            if len(idx) > 220:
                idx = rng.choice(idx, 220, replace=False)
            vis = np.zeros(len(idx), dtype=bool)
            for k, j in enumerate(idx):
                p = pts_d[j]
                d = p - cam_pos
                dist = float(np.linalg.norm(d))
                vis[k] = self.rayo_visible(dg, cam_pos, d / dist, dist, o)
            frac_vis = vis.mean()
            if frac_vis < 0.25:
                info.append((NOMBRES[pz.clase], "ocluido", round(float(frac_vis), 2)))
                continue
            if frac_vis < 0.85:
                uu, vv = u[idx][vis], v[idx][vis]
                cx1, cx2 = max(uu.min(), cx1), min(uu.max(), cx2)
                cy1, cy2 = max(vv.min(), cy1), min(vv.max(), cy2)
            wpx, hpx = (cx2 - cx1) * W, (cy2 - cy1) * H
            if wpx < 5 or hpx < 5 or (frac_marco < 0.2 and (wpx < 20 or hpx < 20)):
                info.append((NOMBRES[pz.clase], "recortado", round(float(frac_marco), 2)))
                continue
            filas.append((pz.clase, (cx1 + cx2) / 2, (cy1 + cy2) / 2, cx2 - cx1, cy2 - cy1))
            info.append((NOMBRES[pz.clase], "ok", round(float(frac_vis), 2), round(float(frac_marco), 2)))
        return filas, info

    # ----------------------------------------------------------- post-proceso de cámara
    def post(self, rgb, rng):
        """rgb: HxWx3 float (sRGB). Simula la cámara real: exposición automática, balance de blancos,
        desenfoque de movimiento / foco, viñeteo y ruido."""
        H, W = rgb.shape[:2]
        lum = (rgb * np.array([0.299, 0.587, 0.114])).sum(-1)
        objetivo = rng.uniform(0.14, 0.36)
        ganancia = np.clip(objetivo / max(float(np.median(lum)), 1e-3), 0.3, 2.5)
        rgb = rgb * ganancia
        rgb = rgb * rng.uniform(0.94, 1.06, size=3)        # balance de blancos
        if rng.random() < 0.5:                              # curva S de contraste (procesado de la cámara)
            k = rng.uniform(0.1, 0.35)
            x = np.clip(rgb, 0, 1)
            rgb = x + k * x * (1 - x) * (2 * x - 1) * 2
        if rng.random() < 0.5:                              # contraste / gamma
            rgb = np.clip(rgb, 0, 1) ** rng.uniform(0.85, 1.2)
        # desenfoque de movimiento
        if rng.random() < 0.35:
            L = int(rng.integers(2, 7 if W <= 640 else 11))
            ang = rng.uniform(0, np.pi)
            dx, dy = np.cos(ang), np.sin(ang)
            acc = np.zeros_like(rgb)
            pad = L + 1
            P = np.pad(rgb, ((pad, pad), (pad, pad), (0, 0)), mode="edge")
            for k in range(L):
                t = k - (L - 1) / 2
                ox, oy = int(round(t * dx)), int(round(t * dy))
                acc += P[pad + oy:pad + oy + H, pad + ox:pad + ox + W]
            rgb = acc / L
        # desenfoque de foco (gaussiano separable)
        if rng.random() < 0.25:
            s = rng.uniform(0.5, 1.4)
            r = int(math.ceil(2.5 * s))
            k = np.exp(-0.5 * (np.arange(-r, r + 1) / s) ** 2)
            k /= k.sum()
            P = np.pad(rgb, ((r, r), (r, r), (0, 0)), mode="edge")
            tmp = sum(k[i] * P[:, i:i + W] for i in range(2 * r + 1))
            rgb = sum(k[i] * tmp[i:i + H] for i in range(2 * r + 1))
        # viñeteo
        if rng.random() < 0.6:
            yy, xx = np.mgrid[0:H, 0:W]
            rr = ((xx - W / 2) / (W / 2)) ** 2 + ((yy - H / 2) / (H / 2)) ** 2
            rgb = rgb * (1 - rng.uniform(0.05, 0.25) * rr / 2)[..., None]
        # ruido de sensor (luminancia + croma)
        s = rng.uniform(0.003, 0.02)
        ruido = rng.normal(0, s, size=(H, W, 1)) + rng.normal(0, s * 0.5, size=(H, W, 3))
        rgb = rgb + ruido * np.sqrt(np.clip(rgb, 0.02, 1))
        return np.clip(rgb, 0, 1)

    # ----------------------------------------------------------- bucle
    def ejecutar(self):
        a = self.a
        for s in ("train", "val"):
            os.makedirs(os.path.join(a.salida, "images", s), exist_ok=True)
            os.makedirs(os.path.join(a.salida, "labels", s), exist_ok=True)
        meta_ruta = os.path.join(a.salida, "meta_s%d.csv" % a.semilla)
        self.depurar_meta(meta_ruta)                       # al reanudar: una fila por imagen
        nuevo = not os.path.exists(meta_ruta)
        meta = open(meta_ruta, "a", encoding="utf-8")
        if nuevo:
            meta.write(CABECERA_META)
        n_escenas = int(math.ceil(a.n / a.vistas))
        t0 = time.time()
        hechas = 0
        t_render = 0.0
        stats = {"fuego": 0, "toma": 0, "vacias": 0}
        for e in range(n_escenas):
            rng = np.random.default_rng([a.semilla, e])
            split = "val" if rng.random() < a.frac_val else "train"
            base = "s%d_e%05d" % (a.semilla, e)
            n_v = min(a.vistas, a.n - e * a.vistas)
            ultima = os.path.join(a.salida, "images", split, "%s_v%02d.jpg" % (base, n_v - 1))
            ultima_txt = os.path.join(a.salida, "labels", split, "%s_v%02d.txt" % (base, n_v - 1))
            if os.path.exists(ultima) and os.path.exists(ultima_txt):
                continue                                    # reanudar: escena completa
            etiquetables, verts, segmentos = self.construir(rng, e)
            if a.guardar_blend and e == 0:
                bpy.ops.wm.save_as_mainfile(filepath=os.path.join(a.salida, "escena_ejemplo.blend"), copy=True)
            for vi in range(n_v):
                if a.res == "mix":
                    W, H = (640, 480) if rng.random() < 0.7 else (1280, 720)
                else:
                    W, H = (640, 480) if a.res == "640" else (1280, 720)
                self.sc.render.resolution_x, self.sc.render.resolution_y = W, H
                h = self.poner_camara(rng, etiquetables, segmentos)
                self.poner_luces(rng)
                bpy.context.view_layer.update()
                filas, info = self.etiquetas(etiquetables, W, H, rng)
                tr = time.time()
                self.sc.render.filepath = self.tmp_png
                bpy.ops.render.render(write_still=True)
                t_render += time.time() - tr
                img = bpy.data.images.load(self.tmp_png)
                px = np.empty(W * H * 4, dtype=np.float32)
                img.pixels.foreach_get(px)
                bpy.data.images.remove(img)
                rgb = px.reshape(H, W, 4)[::-1, :, :3]
                if not a.sin_post:
                    rgb = self.post(rgb, rng)
                nombre = "%s_v%02d" % (base, vi)
                out = np.ones((H, W, 4), dtype=np.float32)
                out[..., :3] = rgb
                im2 = bpy.data.images.new("salida", W, H, alpha=False)
                im2.pixels.foreach_set(np.ascontiguousarray(out[::-1]).ravel())
                # primero la etiqueta y después la imagen, que se escribe aparte y se mueve de golpe: si se mata
                # el proceso nunca queda una imagen sin su .txt (se tomaría como fondo)
                with open(os.path.join(a.salida, "labels", split, nombre + ".txt"), "w") as f:
                    for c, x, y, w, hh in filas:
                        f.write("%d %.6f %.6f %.6f %.6f\n" % (c, x, y, w, hh))
                im2.file_format = "JPEG"
                tmp_jpg = os.path.join(a.salida, "_tmp_%d.jpg" % os.getpid())
                im2.save(filepath=tmp_jpg, quality=int(rng.integers(72, 96)))
                bpy.data.images.remove(im2)
                os.replace(tmp_jpg, os.path.join(a.salida, "images", split, nombre + ".jpg"))
                nf = sum(1 for r in filas if r[0] == 0)
                nt = sum(1 for r in filas if r[0] == 1)
                stats["fuego"] += nf
                stats["toma"] += nt
                stats["vacias"] += int(not filas)
                meta.write("%s,%s,%d,%d,%d,%d,%.3f,%d,%d,%.2f,%s,%s\n" % (
                    nombre, split, e, vi, W, H, h, nf, nt, time.time() - tr, self.fuegos[0].fuente,
                    self.valvula.fuente))
                hechas += 1
                if hechas % 10 == 0:
                    dt = time.time() - t0
                    print("PROGRESO %d imgs, %.1f img/min (render %.2f s/img) escena %d/%d" % (
                        hechas, hechas / dt * 60, t_render / hechas, e + 1, n_escenas), flush=True)
            meta.flush()
            # purga periódica de datos huérfanos
            if e % 20 == 19:
                for blk in (bpy.data.meshes, bpy.data.lights):
                    for d in list(blk):
                        if d.users == 0 and not d.use_fake_user:
                            blk.remove(d)
        meta.close()
        self.depurar_meta(meta_ruta)
        dt = time.time() - t0
        try:
            os.remove(self.tmp_png)
        except Exception:
            pass
        self.escribir_yaml()
        # totales de TODO el dataset en disco (no sólo de esta corrida, que puede ser una reanudación)
        res = self.contar_dataset()
        res.update({"motor": a.motor, "muestras": a.muestras,
                    "fuente_fuegos": sorted(set(p.fuente for p in self.fuegos)),
                    "fuente_toma": self.valvula.fuente,
                    "ultima_corrida": {"imagenes": hechas, "segundos": round(dt, 1),
                                       "img_por_min": round(hechas / dt * 60, 2) if hechas else 0,
                                       "render_s_por_img": round(t_render / max(hechas, 1), 3),
                                       "objetos": stats}})
        with open(os.path.join(a.salida, "stats_s%d.json" % a.semilla), "w") as f:
            json.dump(res, f, indent=2)
        print("RESUMEN", json.dumps(res))

    @staticmethod
    def depurar_meta(ruta):
        """Deja una sola fila por imagen (la última) en meta_s*.csv; al reanudar, las escenas a medias se
        vuelven a renderizar y sus filas se repetían."""
        if not os.path.exists(ruta):
            return
        with open(ruta, encoding="utf-8") as f:
            lineas = f.read().splitlines()
        if not lineas:
            return
        cab, filas = lineas[0], {}
        for ln in lineas[1:]:
            if ln.strip():
                filas[ln.split(",", 1)[0]] = ln             # dict conserva el orden de la 1.a aparición
        if len(filas) != len(lineas) - 1:
            tmp = ruta + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                f.write(cab + "\n" + "".join(v + "\n" for v in filas.values()))
            os.replace(tmp, ruta)

    def contar_dataset(self):
        res = {"imagenes": 0, "train": 0, "val": 0, "objetos": {"fuego": 0, "toma": 0, "vacias": 0},
               "imagenes_sin_etiqueta": 0}
        for split in ("train", "val"):
            for img in glob.glob(os.path.join(self.a.salida, "images", split, "*.jpg")):
                res["imagenes"] += 1
                res[split] += 1
                lbl = os.path.join(self.a.salida, "labels", split,
                                   os.path.splitext(os.path.basename(img))[0] + ".txt")
                if not os.path.exists(lbl):
                    res["imagenes_sin_etiqueta"] += 1
                    continue
                with open(lbl) as f:
                    cl = [ln.split()[0] for ln in f if ln.strip()]
                res["objetos"]["fuego"] += cl.count("0")
                res["objetos"]["toma"] += cl.count("1")
                res["objetos"]["vacias"] += int(not cl)
        return res

    def escribir_yaml(self):
        ruta = os.path.join(self.a.salida, "data.yaml")
        with open(ruta, "w", encoding="utf-8") as f:
            f.write("# Dataset sintético QDrone 2 (generado por vision/sintetico/generar_blender.py)\n")
            f.write("path: %s\n" % self.a.salida.replace("\\", "/"))
            f.write("train: images/train\nval: images/val\n")
            f.write("names:\n  0: fuego\n  1: toma\n")


if __name__ == "__main__":
    args = leer_args()
    os.makedirs(args.salida, exist_ok=True)
    Generador(args).ejecutar()
