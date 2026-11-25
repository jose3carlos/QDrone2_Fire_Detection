"""Genera casos de referencia con el código REAL de la estación (web/app/geo.py).

Sirve para comprobar que la cámara virtual y los clusters del simulador 3D (MATLAB) dan
exactamente lo mismo que la estación Python. Escribe ``referencia_estacion.json`` junto a
este archivo; ``probar_sim3d.m`` lo lee y compara.

Casos con actitud (``pixeles_actitud``, ``huellas_actitud``, ``pose_geo``): roll y pitch de y(61:62),
pivote de Motive (``optitrack.bias_z_dronN``), ``vision.usar_actitud`` / ``actitud_signos`` /
``actitud_offset_deg`` y el descarte por ``vision.inclinacion_max_deg`` (``pose_para_geo`` y
``process`` de web/app/vision_service.py). Se generan DESPUÉS de los casos de siempre con el mismo
generador aleatorio, así que los casos nivelados no cambian.

Casos de tubos (``tubos``): ``geo.tubos_pista`` / ``tramos_sin_tubo`` / ``dist_tubos`` con y sin
``pista.tubos`` (ruta partida en ``tramos_sin_tubo``, valores inválidos), los umbrales
``dist_max_tubo_toma_m`` / ``dist_max_tubo_fuego_m`` de ``filtros_vision`` y la decisión "lejos del tubo"
de ``VisionService.filtrar`` con la cámara real. Usan un generador aleatorio APARTE (semilla 24) y se
escriben al final, así que no cambian ninguno de los casos anteriores.

``anomalias_web`` se calcula con la N de 4 vértices fija de ``probar_sim3d.m`` (la que usan las pruebas
en modo dual), no con la pista del día de config/estacion.json.

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
from app.geo import CameraModel, Clusterer, dist_tubos, tramos_sin_tubo, tubos_pista  # noqa: E402
from app.mock_world import default_anomalies  # noqa: E402
try:                                          # cámara sintética con tubos reales (mock_world.anomalias_mock)
    from app.mock_world import anomalias_mock  # noqa: E402
except ImportError:                           # pragma: no cover (estación anterior a pista.tubos en el mock)
    anomalias_mock = None
from app.vision_service import MOTIVO_TUBO, VisionService, pose_para_geo  # noqa: E402

# N de 4 vértices fija de las pruebas en modo dual (probar_sim3d.m: cfg_dual.pista.vertices)
N_PRUEBAS = [[-1.2, 1.2], [1.2, 1.2], [-1.2, -1.2], [1.2, -1.2]]

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


# ---------------------------------------------------------------- tubos reales (pista.tubos)
rt = random.Random(24)          # generador aparte: los casos anteriores no cambian
pista_real = cfg.get("pista") or {}
sin_tubos = {k: v for k, v in pista_real.items() if k != "tubos"}
V_real = pista_real.get("vertices") or N_PRUEBAS
PISTAS = [
    ("config real (pista.tubos)", pista_real),
    ("config real sin pista.tubos: ruta partida en tramos_sin_tubo", sin_tubos),
    ("ruta entera (sin tubos ni tramos_sin_tubo)", {"vertices": V_real}),
    ("varios huecos [1, 4, 6]", {"vertices": V_real, "tramos_sin_tubo": [1, 4, 6]}),
    ("tramos_sin_tubo con basura", {"vertices": V_real, "tramos_sin_tubo": [4, "2", 0, -1, 2.7, None, "x", True]}),
    ("tubos con entradas inválidas", {"vertices": V_real, "tramos_sin_tubo": [4], "tubos": [
        [[0, 0], [1, 0]], [[0.5, 0.5]], "x", [[1, 1], ["a", 2], ["1.5", 1]], [], [1, 2]]}),
    ("tubos todos inválidos: ruta partida", {"vertices": V_real, "tramos_sin_tubo": [4], "tubos": [[[0, 0]]]}),
    ("tubos vacío: ruta partida", {"vertices": V_real, "tramos_sin_tubo": [4], "tubos": []}),
    ("tubos que no son lista", {"vertices": V_real, "tubos": "abc"}),
    ("tubos objeto", {"vertices": V_real, "tramos_sin_tubo": [2], "tubos": {"a": 1}}),
    ("tubos del mismo largo", {"vertices": V_real, "tubos": [[[0, 0], [1, 0]], [[0, 1], [1, 1]]]}),
    ("un solo tubo", {"vertices": V_real, "tubos": [[[-1, -1], [0, 0], [1, -1]]]}),
    ("puntos con 3 coordenadas", {"tubos": [[[0, 0, 5], [1, 1, 5]], [[-1, 0, 0], [-1, 1, 0], [-1.5, 1.5, 0]]]}),
    ("vértices con basura", {"vertices": [[0, 0], [1, "a"], [2, 0], [3], [2, 1]], "tramos_sin_tubo": [2]}),
    ("pista vacía", {}),
    ("la N de las pruebas", {"vertices": N_PRUEBAS}),
]
casos_tubos = []
for nombre, pista in PISTAS:
    T = tubos_pista(pista)
    pts = []
    for _ in range(15):
        x, y = rt.uniform(-1.8, 1.8), rt.uniform(-1.8, 1.8)
        pts.append([x, y, dist_tubos(x, y, T)])
    casos_tubos.append({"nombre": nombre, "pista_json": json.dumps(pista), "tubos": T,
                        "sin_tubo": sorted(tramos_sin_tubo(pista)), "puntos": pts})

# umbrales del filtro de tubo (filtros_vision) con valores raros
VARIANTES_TUBO = [
    {},
    {"dist_max_tubo_toma_m": 0.45},
    {"dist_max_tubo_toma_m": None},
    {"dist_max_tubo_toma_m": 0},
    {"dist_max_tubo_toma_m": -1, "dist_max_tubo_fuego_m": 0.3},
    {"dist_max_tubo_toma_m": "x", "dist_max_tubo_fuego_m": True},
    {"dist_max_tubo_toma_m": [], "dist_max_tubo_fuego_m": [0.3]},
    {"dist_max_tubo_toma_m": 0.1, "dist_max_tubo_fuego_m": 0.2},
]
casos_lim = []
for var in VARIANTES_TUBO:
    cfg_v = copy.deepcopy(cfg)
    cfg_v.setdefault("vision", {}).update(var)
    f = filtros_vision(cfg_v)
    casos_lim.append({"vision_json": json.dumps(var), "toma": f["dist_max_tubo_toma_m"],
                      "fuego": f["dist_max_tubo_fuego_m"]})

# decisión "lejos del tubo" de VisionService.filtrar: caja de 24 px vista por la cámara real desde una
# pose nivelada sobre la arena, con los filtros de tamaño apagados (el simulador no tiene cajas)
casos_filtrar = []
for nombre, pista, var in [("config real (pista.tubos)", pista_real, {}),
                           ("sin pista.tubos (ruta partida)", sin_tubos, {}),
                           ("toma 0.45, fuego 0.15", pista_real,
                            {"dist_max_tubo_toma_m": 0.45, "dist_max_tubo_fuego_m": 0.15})]:
    cfg_v = copy.deepcopy(cfg)
    cfg_v["pista"] = pista
    cfg_v.setdefault("vision", {}).update(var)
    f = dict(filtros_vision(cfg_v), tam_max_m=None, tam_min_m=None)
    T = tubos_pista(cfg_v.get("pista"))
    for _ in range(20):
        q = (rt.uniform(-1.3, 1.3), rt.uniform(-1.3, 1.3), rt.uniform(1.0, 1.8), rt.uniform(-3.1, 3.1), 0.0, 0.0)
        u, v = rt.uniform(12, cam.w - 13), rt.uniform(12, cam.h - 13)
        cls = rt.randint(0, 1)
        g = cam.pixel_to_ground(u, v, *q, z_plane=0.02)
        motivo, _, dist = VisionService.filtrar(VisionService, cam, q, cls, (u - 12, v - 12, u + 12, v + 12), g, f,
                                                cfg_v["pista"].get("vertices") or [], tubos=T)
        casos_filtrar.append({"nombre": nombre, "pista_json": json.dumps(pista), "vision_json": json.dumps(var),
                              "pose": list(q), "uv": [u, v], "cls": cls, "suelo": [g[0], g[1]], "dist": dist,
                              "lejos": motivo == MOTIVO_TUBO})

ref = {"camara": {"w": cam.w, "h": cam.h, "fx": cam.fx, "fy": cam.fy, "cx": cam.cx, "cy": cam.cy,
                  "origen": cam.origen, "pivote": cam.pivote_m},
       "huellas": casos_huella, "pixeles": casos_pixel,
       "pixeles_actitud": casos_pixel_att, "huellas_actitud": casos_huella_att, "pose_geo": casos_geo,
       "clusters": {"radio": cl.radius, "min_obs": cl.min_obs, "observaciones": obs, "objetivos": objetivos,
                    "pendientes": len(cl.pending())},
       # objetos que dibuja la cámara sintética de la UI (igual que Station en web/app/server.py)
       "anomalias_web": cfg.get("web", {}).get("mock_anomalias") or default_anomalies(N_PRUEBAS),
       # tubos reales y filtro "lejos del tubo" (geo.tubos_pista / dist_tubos, filtros_vision, filtrar)
       "tubos": {"pistas": casos_tubos, "limites": casos_lim, "filtrar": casos_filtrar}}
if anomalias_mock is not None:
    # cámara sintética con la pista REAL de config/estacion.json (sobre pista.tubos si los hay)
    ref["anomalias_web_pista_real"] = anomalias_mock(cfg)
ruta = os.path.join(AQUI, "referencia_estacion.json")
with open(ruta, "w", encoding="utf-8") as f:
    json.dump(ref, f, indent=1)
print(f"escrito {ruta}: {len(casos_huella)} huellas, {len(casos_pixel)} píxeles, "
      f"{len(obs)} observaciones -> {len(objetivos)} objetivos; con actitud: {len(casos_pixel_att)} píxeles, "
      f"{len(casos_huella_att)} huellas, {len(casos_geo)} casos de pose_para_geo "
      f"({sum(c['inclinado'] for c in casos_geo)} descartados por inclinación); tubos: {len(casos_tubos)} pistas, "
      f"{len(casos_lim)} variantes de umbral, {len(casos_filtrar)} casos de filtrar "
      f"({sum(c['lejos'] for c in casos_filtrar)} lejos del tubo)")
