"""Genera casos de referencia con el código REAL de la estación (web/app/geo.py).

Sirve para comprobar que la cámara virtual y los clusters del simulador 3D (MATLAB) dan
exactamente lo mismo que la estación Python. Escribe ``referencia_estacion.json`` junto a
este archivo; ``probar_sim3d.m`` lo lee y compara.

Casos con actitud (``pixeles_actitud``, ``huellas_actitud``, ``pose_geo``): roll y pitch de y(61:62),
pivote de Motive (``optitrack.bias_z_dronN``), ``vision.usar_actitud`` / ``actitud_signos`` /
``actitud_offset_deg`` y el descarte por ``vision.inclinacion_max_deg`` (``pose_para_geo`` y
``process`` de web/app/vision_service.py). Se generan DESPUÉS de los casos de siempre con el mismo
generador aleatorio, así que los casos nivelados no cambian.

Uso (necesita numpy; por ejemplo la venv de la web):
    C:\\Users\\<usuario>\\.venvs\\qd2_web\\Scripts\\python.exe simulador3d\\pruebas\\generar_referencia_estacion.py
"""
import copy
import json
import math
import os
import random
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(os.path.dirname(AQUI))
sys.path.insert(0, os.path.join(RAIZ, "web"))
from app.config import actitud_vision, filtros_vision  # noqa: E402
from app.geo import CameraModel, Clusterer  # noqa: E402
from app.mock_world import default_anomalies  # noqa: E402
from app.vision_service import pose_para_geo  # noqa: E402

with open(os.path.join(RAIZ, "config", "estacion.json"), encoding="utf-8") as f:
    cfg = json.load(f)
cam = CameraModel.from_config(cfg.get("camara", {}), cfg)

random.seed(3)
casos_huella = []
for _ in range(12):
    q = [random.uniform(-1.3, 1.3), random.uniform(-1.3, 1.3), random.uniform(0.4, 1.9),
         random.uniform(-3.1, 3.1)]
    casos_huella.append({"pose": q, "huella": cam.footprint(*q)})
casos_pixel = []
for _ in range(20):
    q = [random.uniform(-1.3, 1.3), random.uniform(-1.3, 1.3), random.uniform(0.4, 1.9),
         random.uniform(-3.1, 3.1)]
    u, v = random.uniform(0, cam.w - 1), random.uniform(0, cam.h - 1)
    casos_pixel.append({"pose": q, "uv": [u, v], "suelo": cam.pixel_to_ground(u, v, *q)})

vis = cfg.get("vision", {})
cl = Clusterer(vis.get("radio_cluster_m", 0.25), vis.get("min_observaciones", 3))
obs = []
centros = [(0, 0.3, 1.2), (1, 0.45, 0.35), (0, 0.5, 0.42), (1, -0.55, -0.55), (0, 1.0, -1.0)]
for _ in range(60):
    c, x, y = random.choice(centros)
    o = [c, x + random.gauss(0, 0.06), y + random.gauss(0, 0.06), random.uniform(0.5, 0.95)]
    obs.append(o)
    cl.add(*o)
objetivos = [[t["id"], t["cls"], t["x"], t["y"], t["conf"]] for t in cl.targets()]


# ---------------------------------------------------------------- casos con actitud
def pose_aleatoria(max_ang):
    return [random.uniform(-1.3, 1.3), random.uniform(-1.3, 1.3), random.uniform(0.4, 1.9),
            random.uniform(-3.1, 3.1), random.uniform(-max_ang, max_ang), random.uniform(-max_ang, max_ang)]


def lista(g):
    return None if g is None else [g[0], g[1]]


# píxel -> suelo con roll/pitch y el pivote de Motive del estacion.json (hasta ±20°; algunos nivelados)
casos_pixel_att = []
for k in range(30):
    q = pose_aleatoria(math.radians(20.0))
    if k < 3:
        q[4] = q[5] = 0.0
    u, v = random.uniform(0, cam.w - 1), random.uniform(0, cam.h - 1)
    casos_pixel_att.append({"pose": q, "uv": [u, v], "suelo": lista(cam.pixel_to_ground(u, v, *q, z_plane=0.02))})
# huella con actitud SIN redondear (footprint redondea a mm)
casos_huella_att = []
esquinas = [(0, 0), (cam.w - 1, 0), (cam.w - 1, cam.h - 1), (0, cam.h - 1)]
for _ in range(8):
    q = pose_aleatoria(math.radians(12.0))
    casos_huella_att.append({"pose": q, "huella": [lista(cam.pixel_to_ground(u, v, *q, z_plane=0.02))
                                                   for u, v in esquinas]})

# Variantes de vision / optitrack: pose de la telemetría (roll/pitch crudos de y(61:62), None = y(63) = 0)
# -> pose_para_geo -> descarte por inclinación -> píxel al suelo, igual que VisionService.process.
VARIANTES = [
    ({}, {}),                                                               # estacion.json tal cual
    ({"usar_actitud": False}, {}),
    ({"actitud_signos": [-1, 1], "actitud_offset_deg": [1.5, -2.0]}, {}),
    ({"actitud_signos": [1, -1], "actitud_offset_deg": [0.0, 3.0], "inclinacion_max_deg": 1.5}, {}),
    ({"inclinacion_max_deg": None}, {"bias_z_dron1": -0.05, "bias_z_dron2": -0.06}),
    ({"inclinacion_max_deg": 0}, {"bias_z_dron1": "x", "bias_z_dron2": -0.09}),
    ({"inclinacion_max_deg": "texto", "actitud_signos": [2, -1], "actitud_offset_deg": [45, 1.0],
      "usar_actitud": "si"}, {}),
    ({"inclinacion_max_deg": 8}, {}),
    # tipos mezclados en el JSON: la estación corrige elemento por elemento, y [] (lista) deja 15
    ({"actitud_signos": [True, -1], "actitud_offset_deg": [-30, "a"], "inclinacion_max_deg": []}, {}),
]
casos_geo = []
for vis_var, opt_var in VARIANTES:
    cfg_v = copy.deepcopy(cfg)
    cfg_v.setdefault("vision", {}).update(vis_var)
    cfg_v.setdefault("optitrack", {}).update(opt_var)
    cam_v = CameraModel.from_config(cfg_v.get("camara", {}), cfg_v)
    act = actitud_vision(cfg_v)
    lim = filtros_vision(cfg_v).get("inclinacion_max_deg")
    for k in range(6):
        q = pose_aleatoria(math.radians(18.0))
        if k == 0:
            q[4] = q[5] = None                    # sin actitud (y(63) = 0)
        u, v = random.uniform(0, cam_v.w - 1), random.uniform(0, cam_v.h - 1)
        q6, info = pose_para_geo(tuple(q), act)
        inclinado = bool(info["ok"] and lim is not None and math.degrees(info["incl"]) > lim)
        casos_geo.append({
            "vision": {kk: cfg_v["vision"].get(kk) for kk in ("usar_actitud", "actitud_signos", "actitud_offset_deg",
                                                             "inclinacion_max_deg")},
            # el mismo bloque como texto JSON: jsondecode no distingue null de [] (sim3d_leer_json sí)
            "vision_json": json.dumps({kk: cfg_v["vision"][kk] for kk in ("usar_actitud", "actitud_signos",
                                       "actitud_offset_deg", "inclinacion_max_deg") if kk in cfg_v["vision"]}),
            "optitrack": {kk: cfg_v["optitrack"].get(kk) for kk in ("bias_z_dron1", "bias_z_dron2")},
            "pivote": cam_v.pivote_m, "lim_incl_deg": lim,
            "pose": q, "uv": [u, v], "q6": list(q6), "ok": info["ok"], "disponible": info["disponible"],
            "incl_deg": math.degrees(info["incl"]) if info["ok"] else None, "inclinado": inclinado,
            "suelo": lista(cam_v.pixel_to_ground(u, v, *q6, z_plane=0.02))})

ref = {"camara": {"w": cam.w, "h": cam.h, "fx": cam.fx, "fy": cam.fy, "cx": cam.cx, "cy": cam.cy,
                  "origen": cam.origen, "pivote": cam.pivote_m},
       "huellas": casos_huella, "pixeles": casos_pixel,
       "pixeles_actitud": casos_pixel_att, "huellas_actitud": casos_huella_att, "pose_geo": casos_geo,
       "clusters": {"radio": cl.radius, "min_obs": cl.min_obs, "observaciones": obs, "objetivos": objetivos,
                    "pendientes": len(cl.pending())},
       # objetos que dibuja la cámara sintética de la UI (igual que Station en web/app/server.py)
       "anomalias_web": cfg.get("web", {}).get("mock_anomalias") or default_anomalies(cfg["pista"]["vertices"])}
ruta = os.path.join(AQUI, "referencia_estacion.json")
with open(ruta, "w", encoding="utf-8") as f:
    json.dump(ref, f, indent=1)
print(f"escrito {ruta}: {len(casos_huella)} huellas, {len(casos_pixel)} píxeles, "
      f"{len(obs)} observaciones -> {len(objetivos)} objetivos; con actitud: {len(casos_pixel_att)} píxeles, "
      f"{len(casos_huella_att)} huellas, {len(casos_geo)} casos de pose_para_geo "
      f"({sum(c['inclinado'] for c in casos_geo)} descartados por inclinación)")
