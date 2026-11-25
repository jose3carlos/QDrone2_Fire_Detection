"""Ruta de vuelo y tubos reales separados en el mundo sintético (laboratorio del 24-nov).

``pista.vertices`` es la RUTA DE VUELO (la compila Simulink; el mock la recorre) y ``pista.tubos`` la posición REAL
de los tubos, a 0.1-0.45 m de la ruta. La cámara sintética dibuja los tubos reales y las anomalías por defecto van
sobre ellos (incluidas las orillas en y ~ ±1.35-1.48), así que la estación debe detectarlas, pasarlas por el filtro
de tubos (``geo.dist_tubos``) y geolocalizarlas a < 5 cm. Sin ``pista.tubos`` todo queda como antes.

Las pruebas fijan su propio umbral (0.5); ``config/estacion.json`` usa 0.9 y la misión mock también se corre con
0.9 para comprobar que el detector falso da confianzas realistas (>= 0.9) con piezas completas.
"""
import copy
import json
import math
import time

import numpy as np
import pytest

from app import protocol as P
from app.geo import CameraModel, dist_polilinea, dist_tubos, tubos_pista
from app.mock_world import (COL_FIRE, COL_TOMA, ColorDetector, anomalias_mock, default_anomalies, path_points,
                            render_view, tubos_reales)
from app.server import Station
from app.vision_service import MOTIVO_TUBO, PoseHistory, VisionService
from conftest import PISTA_LAB
from test_mock_mision import DT, Driver

HOMES_LAB = ((1.25, 0.0), (1.45, -1.45))
V_LAB, T_LAB = PISTA_LAB["vertices"], PISTA_LAB["tubos"]
Z = 1.5


def _cfg(path, conf=0.5):
    with open(path, encoding="utf-8") as fh:
        cfg = json.load(fh)
    cfg["vision"]["confianza_min"] = conf            # umbral de las pruebas (el real puede ser 0.9)
    cfg.setdefault("web", {})["camara_latencia_s"] = 0.0
    return cfg


class _Cam:
    def wait_newer(self, *a):
        return None


def _servicio(cfg, gate=lambda: True, publish=lambda t: None):
    return VisionService(_Cam(), PoseHistory(), gate, publish, lambda: cfg, ColorDetector(), "falso")


def _pose_sobre(x, y, yaw=0.0):
    """Pose del explorador para que la cámara (adelantada 0.20 m) quede encima de (x, y)."""
    return (x - 0.20 * math.cos(yaw), y - 0.20 * math.sin(yaw), Z, yaw)


# =============================================================================== anomalías por defecto
def test_anomalias_por_defecto_sobre_los_tubos_reales():
    A = default_anomalies(V_LAB, T_LAB)
    assert [a["cls"] for a in A] == [1, 0, 1, 0]                       # toma, fuego, toma, fuego (2 por pieza)
    assert A == [{"cls": 1, "x": 0.46, "y": 1.4}, {"cls": 0, "x": -0.91, "y": 1.176},
                 {"cls": 1, "x": 0.962, "y": -1.347}, {"cls": 0, "x": -0.313, "y": -1.476}]
    T = tubos_pista(PISTA_LAB)
    for a in A:
        assert dist_tubos(a["x"], a["y"], T) < 0.002                    # en el eje del tubo real
        assert 0.05 < dist_polilinea(a["x"], a["y"], V_LAB) < 0.40      # y fuera de la ruta de vuelo
    # orillas: lado +y en y ~1.40 y lado -y en y ~-1.35 / -1.48 (donde la ruta ya no llega)
    assert max(a["y"] for a in A) >= 1.39 and min(a["y"] for a in A) <= -1.47
    # una toma de la orilla -y queda a > 0.25 m de la ruta: contra la ruta se descartaría "lejos del tubo"
    assert max(dist_polilinea(a["x"], a["y"], V_LAB, {4}) for a in A if a["cls"] == 1) > 0.25
    # con una sola pieza: 3 anomalías (fuego en medio)
    U = default_anomalies(V_LAB, T_LAB[:1])
    assert [a["cls"] for a in U] == [1, 0, 1] and all(dist_tubos(a["x"], a["y"], T_LAB[:1]) < 0.002 for a in U)


def test_sin_tubos_reales_comportamiento_anterior():
    """Sin pista.tubos (o con tubos inválidos): 3 anomalías a 8 cm de la ruta, igual que antes."""
    N = [[-1.2, 1.2], [1.2, 1.2], [-1.2, -1.2], [1.2, -1.2]]
    viejo = []
    for k, (x, y, dx, dy) in enumerate(path_points(N, [0.22, 0.52, 0.83])):
        s = 0.08 if k % 2 == 0 else -0.08
        viejo.append({"cls": k % 2, "x": round(x - s * dy, 3), "y": round(y + s * dx, 3)})
    assert default_anomalies(N) == default_anomalies(N, None) == default_anomalies(N, []) == viejo
    assert default_anomalies(N, [[[0, 0]], "x"]) == viejo                       # tubos inválidos se ignoran
    assert tubos_reales({"vertices": N}) is None and tubos_reales(None) is None
    assert tubos_reales({"vertices": N, "tubos": [[[0, 0]]]}) is None
    assert tubos_reales(PISTA_LAB) == T_LAB
    # lo que usa la estación: web.mock_anomalias manda; si no, la pista (con o sin tubos)
    assert anomalias_mock({"pista": {"vertices": N, "tramos_sin_tubo": [2]}}) == viejo
    assert anomalias_mock({"pista": copy.deepcopy(PISTA_LAB)}) == default_anomalies(V_LAB, T_LAB)
    propias = [{"cls": 1, "x": 0.1, "y": 0.2}]
    assert anomalias_mock({"pista": copy.deepcopy(PISTA_LAB), "web": {"mock_anomalias": propias}}) == propias


def test_config_real_anomalias_sobre_sus_tubos():
    """Con la pista de config/estacion.json (la del día), las anomalías del mock quedan sobre su tubería."""
    from conftest import REAL_CFG
    with open(REAL_CFG, encoding="utf-8") as fh:
        real = json.load(fh)
    pista = real.get("pista") or {}
    A = anomalias_mock({"pista": pista})
    assert len(A) >= 3
    if tubos_reales(pista):
        T = tubos_pista(pista)
        assert all(dist_tubos(a["x"], a["y"], T) < 0.002 for a in A)


# =============================================================================== cámara sintética
def _brillo(img, cam, pose, x, y):
    uv, front = cam.world_to_pixel([[x, y, 0.0134]], *pose)
    u, v = int(round(uv[0, 0])), int(round(uv[0, 1]))
    assert front.all() and 2 <= u < cam.w - 2 and 2 <= v < cam.h - 2
    return int(img[v - 1:v + 2, u - 1:u + 2].mean())


def test_camara_sintetica_dibuja_los_tubos_reales_no_la_ruta():
    cfg = {"camara": {"resolucion": [640, 480], "hfov_deg": 69.4, "vfov_deg": 42.5, "offset_camara_m": [0.2, 0.0, -0.05]}}
    cam = CameraModel.from_config(cfg["camara"], cfg)
    # lado +y: tubo real en y ~1.39, ruta en y ~1.15 (a 0.24 m)
    pose = _pose_sobre(0.0, 1.27)
    y_tubo = 1.41 + (1.37 - 1.41) * (0.99 - 0.0) / (0.99 + 1.05)
    y_ruta = 1.2 + (1.1 - 1.2) * (0.99 - 0.0) / (0.99 + 1.05)
    img = render_view(cam, pose, V_LAB, [], tubos=tubos_pista(PISTA_LAB))
    assert _brillo(img, cam, pose, 0.0, y_tubo) > 150            # tubo blanco
    assert _brillo(img, cam, pose, 0.0, y_ruta) < 80             # la ruta de vuelo no se dibuja
    # comportamiento anterior (sin tubos): la ruta completa
    img0 = render_view(cam, pose, V_LAB, [])
    assert _brillo(img0, cam, pose, 0.0, y_ruta) > 150 and _brillo(img0, cam, pose, 0.0, y_tubo) < 80
    # hueco del tramo 4 (entre las dos piezas): sin tubo con tubos_pista, aunque no haya pista.tubos
    xm, ym = (V_LAB[3][0] + V_LAB[4][0]) / 2, (V_LAB[3][1] + V_LAB[4][1]) / 2
    pose = _pose_sobre(xm, ym)
    sin_tubos = {"vertices": V_LAB, "tramos_sin_tubo": [4]}
    assert _brillo(render_view(cam, pose, V_LAB, [], tubos=tubos_pista(sin_tubos)), cam, pose, xm, ym) < 80
    assert _brillo(render_view(cam, pose, V_LAB, []), cam, pose, xm, ym) > 150


class _Puente:
    """Puente falso: telemetría con el explorador (dron 1) en ``pose``."""

    def __init__(self, pose):
        x, y, z, yaw = pose
        self.telemetry = {"explorer_id": 1, "explorer_att_ok": 0,
                          "drones": {1: {"tracked": 1, "x": x, "y": y, "z": z, "yaw": yaw}}}

    def fresh(self):
        return True

    def stop(self):
        pass


def test_estacion_render_sintetico_con_tubos_reales(cfg_path_lab):
    """La cámara sintética de la estación (Station.render_synthetic) usa pista.tubos y sus anomalías."""
    st = Station(cfg_path_lab, "mock")
    st.world = anomalias_mock(st.cfg.data)                     # lo que hace _start_source
    assert st.world == default_anomalies(V_LAB, T_LAB)
    toma = st.world[2]                                          # orilla -y, a 0.33 m de la ruta
    pose = _pose_sobre(toma["x"], toma["y"] + 0.25)             # el dron sobre la ruta, la toma a un lado
    st.bridge = _Puente(pose)
    st.rng = None
    img = st.render_synthetic()
    cam = st._cam_model()
    uv, _ = cam.world_to_pixel([[toma["x"], toma["y"], 0.03]], *pose)
    assert tuple(img[int(uv[0, 1]), int(uv[0, 0])]) == COL_TOMA
    # el tubo real (primer tramo de la pieza 2) pasa por la toma: 9 cm antes a lo largo del tubo se ve blanco
    d = np.subtract(T_LAB[1][1], T_LAB[1][0])
    d = d / np.linalg.norm(d)
    assert _brillo(img, cam, pose, toma["x"] - 0.09 * d[0], toma["y"] - 0.09 * d[1]) > 150
    # y la ruta de vuelo (a 0.25 m, bajo el dron) no se dibuja
    assert dist_polilinea(toma["x"], toma["y"] + 0.33, V_LAB) < 0.02
    assert _brillo(img, cam, pose, toma["x"], toma["y"] + 0.33) < 80


# =============================================================================== detector falso
def test_detector_falso_confianza_realista():
    """Pieza completa >= 0.9 (como el detector real con las piezas del lab); cortada por el borde < 0.6."""
    import cv2
    img = np.full((480, 640, 3), 36, np.uint8)
    cv2.circle(img, (320, 240), 18, COL_FIRE, -1, cv2.LINE_AA)               # fuego completo
    cv2.rectangle(img, (100, 100), (130, 122), COL_TOMA, -1)                 # toma completa
    cv2.circle(img, (630, 400), 18, COL_FIRE, -1, cv2.LINE_AA)               # fuego cortado (borde derecho)
    cv2.rectangle(img, (0, 300), (20, 330), COL_TOMA, -1)                    # toma cortada (borde izquierdo)
    dets = ColorDetector().detect(img)
    assert len(dets) == 4
    completas = [d for d in dets if 20 < (d[2] + d[4]) / 2 < 600]
    cortadas = [d for d in dets if d not in completas]
    assert sorted(d[0] for d in completas) == [0, 1] and sorted(d[0] for d in cortadas) == [0, 1]
    assert all(0.9 <= d[1] <= 0.99 for d in completas), completas
    assert all(d[1] < 0.6 for d in cortadas), cortadas


# =============================================================================== filtro de tubos
def test_toma_de_la_orilla_pasa_el_filtro_con_los_tubos_reales(cfg_path_lab):
    cfg = _cfg(cfg_path_lab)
    cam = CameraModel.from_config(cfg["camara"], cfg, image_size=(640, 480))
    truth = anomalias_mock(cfg)
    toma = truth[2]
    pose = _pose_sobre(toma["x"], toma["y"] + 0.2)
    img = render_view(cam, pose, V_LAB, truth, rng=np.random.default_rng(2), tubos=tubos_pista(cfg["pista"]))
    dets = [d for d in _servicio(cfg).process(img, 10.0, pose=pose) if d["cls"] == 1]
    assert len(dets) == 1 and dets[0]["descartada"] is None and dets[0]["conf"] >= 0.9
    assert dets[0]["dist_tubo_m"] < 0.05 and math.hypot(dets[0]["xy"][0] - toma["x"], dets[0]["xy"][1] - toma["y"]) < 0.05
    # la MISMA imagen con una config sin pista.tubos (tubos = la ruta partida): se descarta lejos del tubo
    viejo = copy.deepcopy(cfg)
    viejo["pista"].pop("tubos")
    dets = [d for d in _servicio(viejo).process(img, 11.0, pose=pose) if d["cls"] == 1]
    assert dets[0]["descartada"] == MOTIVO_TUBO and dets[0]["dist_tubo_m"] > 0.25


# =============================================================================== misión completa
def _mision_mock_con_vision(cfg, t_max=200.0):
    """Misión mock (solo explorador) con la cámara sintética y la visión de la estación en el lazo: cada 4 pasos
    (12.5 Hz) se renderiza lo que ve el explorador, se procesa y los objetivos publicados van al mock."""
    d = Driver(cfg, HOMES_LAB)
    assert d.m.solo
    pista = cfg["pista"]
    truth, tubos = anomalias_mock(cfg), tubos_pista(pista)
    published = []
    gate = {"on": False}
    vs = _servicio(cfg, lambda: gate["on"], published.append)
    cam = CameraModel.from_config(cfg["camara"], cfg)
    rng = np.random.default_rng(0)
    zmin = float(cfg.get("web", {}).get("z_min_deteccion", 0.5))
    assert d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    d.send("START")
    estados, n_frames, k = [], 0, 0
    E = d.m.drones[d.m.ds]
    while d.m.state != P.DONE and k < int(t_max / DT):
        d.step()
        k += 1
        if not estados or estados[-1] != d.m.state:
            estados.append(d.m.state)
        if d.m.state == P.RETURN_HOME and math.hypot(E.p[0] - E.home[0], E.p[1] - E.home[1]) < 0.12 \
                and d.cmd != P.COMMANDS["LAND_ALL"]:
            d.send("LAND_ALL")
        if k % 4 or E.p[2] < 0.15:
            continue
        tel = d.m.telemetry()
        pose = (E.p[0], E.p[1], E.p[2], E.yaw, tel["explorer_roll"], tel["explorer_pitch"])
        gate["on"] = tel["mission_state"] in (P.MISSION, P.PAUSED) and E.p[2] >= zmin
        vs.process(render_view(cam, pose, pista["vertices"], truth, rng=rng, tubos=tubos), d.m.t, pose=pose)
        n_frames += 1
        if published and published[-1]:
            d.set_targets([{"cls": c, "x": x, "y": y} for _, c, x, y, _ in published[-1]])
    return d, vs, (published[-1] if published else []), truth, tubos, estados, n_frames


@pytest.mark.parametrize("conf", [0.5, 0.9])
def test_mision_mock_pista_del_lab_detecta_los_objetivos_sobre_los_tubos(cfg_path_lab, conf):
    """Solo con la pista real: el explorador vuela la ruta (pista.vertices), ve las anomalías sobre los tubos
    reales (orillas incluidas) y la misión llega a DONE con todas detectadas, sin descartes y a < 5 cm."""
    cfg = _cfg(cfg_path_lab, conf)
    assert cfg["pista"] == PISTA_LAB and cfg["mision"]["modo"] == "solo_explorador"
    t0 = time.time()
    d, vs, objetivos, truth, tubos, estados, n_frames = _mision_mock_con_vision(cfg)
    assert d.m.state == P.DONE, estados
    assert estados[-4:] == [P.MISSION, P.RETURN_HOME, P.LANDING, P.DONE]
    assert len(d.m.wps) == 36                                   # la ruta de Simulink: 8.0 m a 0.25 m
    assert n_frames > 200 and time.time() - t0 < 120
    # cada anomalía verdadera tiene exactamente un objetivo de su clase, a < 5 cm, pegado al tubo real
    assert len(objetivos) == len(truth) == 4
    usados = set()
    for tid, cls, x, y, c in objetivos:
        e, i = min((math.hypot(x - a["x"], y - a["y"]), i) for i, a in enumerate(truth) if a["cls"] == cls)
        assert e < 0.05, (tid, cls, x, y, e)
        assert dist_tubos(x, y, tubos) < 0.05
        usados.add(i)
    assert usados == set(range(len(truth)))
    assert vs.status()["descartadas"]["total"] == 0
    # Simulink (el mock) los registró y, sin respondedor, ninguno se visita
    tel = d.m.telemetry()
    assert tel["n_targets_known"] == 4 and tel["n_targets_visited"] == 0
