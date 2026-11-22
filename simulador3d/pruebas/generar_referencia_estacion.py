"""Genera casos de referencia con el código REAL de la estación (web/app/geo.py).

Sirve para comprobar que la cámara virtual y los clusters del simulador 3D (MATLAB) dan
exactamente lo mismo que la estación Python. Escribe ``referencia_estacion.json`` junto a
este archivo; ``probar_sim3d.m`` lo lee y compara.

Uso (necesita numpy; por ejemplo la venv de la web):
    C:\\Users\\<usuario>\\.venvs\\qd2_web\\Scripts\\python.exe simulador3d\\pruebas\\generar_referencia_estacion.py
"""
import json
import os
import random
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(os.path.dirname(AQUI))
sys.path.insert(0, os.path.join(RAIZ, "web"))
from app.geo import CameraModel, Clusterer  # noqa: E402
from app.mock_world import default_anomalies  # noqa: E402

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

ref = {"camara": {"w": cam.w, "h": cam.h, "fx": cam.fx, "fy": cam.fy, "cx": cam.cx, "cy": cam.cy,
                  "origen": cam.origen},
       "huellas": casos_huella, "pixeles": casos_pixel,
       "clusters": {"radio": cl.radius, "min_obs": cl.min_obs, "observaciones": obs, "objetivos": objetivos,
                    "pendientes": len(cl.pending())},
       # objetos que dibuja la cámara sintética de la UI (igual que Station en web/app/server.py)
       "anomalias_web": cfg.get("web", {}).get("mock_anomalias") or default_anomalies(cfg["pista"]["vertices"])}
ruta = os.path.join(AQUI, "referencia_estacion.json")
with open(ruta, "w", encoding="utf-8") as f:
    json.dump(ref, f, indent=1)
print(f"escrito {ruta}: {len(casos_huella)} huellas, {len(casos_pixel)} píxeles, "
      f"{len(obs)} observaciones -> {len(objetivos)} objetivos")
