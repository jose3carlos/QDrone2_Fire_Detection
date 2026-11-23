"""Actitud del explorador en la geolocalización (y(61:63), ARQUITECTURA §3.3 y §7).

Convención (geo.R_mundo_cuerpo): R = Rz(yaw)·Ry(pitch)·Rx(roll); roll > 0 = lado izquierdo arriba,
pitch > 0 = nariz abajo. La cámara mira hacia abajo, así que con la nariz abajo ve hacia ATRÁS.
"""
import copy
import json
import math
import os
import time

import numpy as np
import pytest

from app import protocol as P
from app.config import actitud_vision, filtros_vision
from app.geo import CameraModel, R_BODY_CAM0, R_mundo_cuerpo, corregir_actitud, inclinacion, pivote_de_config, rot_z
from app.mock_sim import G, actitud_de_aceleracion
from app.mock_world import ColorDetector, render_view
from app.vision_service import MOTIVO_INCLINACION, MOTIVOS, Z_PLANE, PoseHistory, VisionService, pose_para_geo
from test_mock_mision import DT, Driver

PROJECT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
with open(os.path.join(PROJECT, "config", "estacion.json"), encoding="utf-8") as _f:
    CFG_REAL = json.load(_f)

N = [[-1.2, 1.2], [1.2, 1.2], [-1.2, -1.2], [1.2, -1.2]]
D = math.radians


def cam_real(rot=0, offset=None):
    cam_cfg = dict(CFG_REAL["camara"], rotacion_montaje=rot)
    if offset is not None:
        cam_cfg["offset_camara_m"] = list(offset)
    return CameraModel.from_config(cam_cfg, CFG_REAL, image_size=(640, 480))


# ============================================================================ geometría
def _viejo_pixel_to_ground(cam, u, v, x, y, z, yaw, z_plane=0.02):
    """La versión anterior (solo yaw), copiada tal cual, como referencia."""
    Rz = rot_z(yaw)
    R_wc = Rz @ cam.R_bc
    p_c = np.array([x, y, z], dtype=float) + Rz @ cam.offset
    d = R_wc @ np.array([(u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, 1.0])
    if d[2] >= -1e-9:
        return None
    t = (z_plane - p_c[2]) / d[2]
    g = p_c + t * d
    return float(g[0]), float(g[1])


def test_convencion_de_ejes():
    p = D(5)
    nariz = R_mundo_cuerpo(0.0, 0.0, p) @ [1.0, 0.0, 0.0]
    assert nariz[2] < 0                                  # pitch > 0: nariz abajo
    izq = R_mundo_cuerpo(0.0, D(5), 0.0) @ [0.0, 1.0, 0.0]
    assert izq[2] > 0                                    # roll > 0: lado izquierdo arriba
    # empuje (z del cuerpo): pitch > 0 lo inclina a +x, roll > 0 a -y (como qd2_quad_plant)
    emp = R_mundo_cuerpo(0.0, D(5), D(5)) @ [0.0, 0.0, 1.0]
    assert emp[0] > 0 and emp[1] < 0
    # orden ZYX intrínseco: Rz·Ry·Rx
    y, r, p = D(30), D(7), D(-4)
    c = math.cos
    s = math.sin
    Rx = np.array([[1, 0, 0], [0, c(r), -s(r)], [0, s(r), c(r)]])
    Ry = np.array([[c(p), 0, s(p)], [0, 1, 0], [-s(p), 0, c(p)]])
    Rz = np.array([[c(y), -s(y), 0], [s(y), c(y), 0], [0, 0, 1]])
    assert np.allclose(R_mundo_cuerpo(y, r, p), Rz @ Ry @ Rx, atol=1e-15)


def test_nivelado_es_identico_al_anterior():
    for rot in (0, 90):
        cam = cam_real(rot)
        for pose in ((0.3, -0.2, 1.5, D(37)), (-1.0, 0.8, 1.2, D(-120)), (0.0, 0.0, 0.9, 0.0)):
            for u, v in ((0, 0), (320, 240), (639, 479), (100, 400)):
                ref = _viejo_pixel_to_ground(cam, u, v, *pose)
                assert cam.pixel_to_ground(u, v, *pose, z_plane=0.02) == ref
                assert cam.pixel_to_ground(u, v, *pose, 0.0, 0.0, z_plane=0.02) == ref
                assert cam.pixel_to_ground(u, v, *pose, roll=0.0, pitch=0.0) == ref
            assert cam.footprint(*pose) == cam.footprint(*pose, 0.0, 0.0, z_plane=0.02)
            P_ = np.array([[0.1, 0.2, 0.0], [0.5, -0.3, 0.03]])
            a, fa = cam.world_to_pixel(P_, *pose)
            b, fb = cam.world_to_pixel(P_, *pose, 0.0, 0.0)
            assert (a == b).all() and (fa == fb).all()


@pytest.mark.parametrize("rot,offset", [(0, None), (90, None), (0, (0.10, -0.05, -0.08)), (180, (0.0, 0.0, 0.0))])
def test_ida_y_vuelta_con_actitud(rot, offset):
    """Punto conocido del piso -> píxel (world_to_pixel) -> piso (pixel_to_ground): el mismo punto (< 1 mm)."""
    cam = cam_real(rot, offset)
    peor = 0.0
    for yaw in (0.0, D(37), D(-120), D(180)):
        for roll in (D(-12), D(-5), 0.0, D(5), D(12)):
            for pitch in (D(-12), D(-5), 0.0, D(5), D(12)):
                x, y, z = 0.3, -0.2, 1.5
                # puntos alrededor de donde mira la cámara (dentro de la imagen)
                g0 = cam.pixel_to_ground(cam.cx, cam.cy, x, y, z, yaw, roll, pitch, z_plane=Z_PLANE)
                for dx, dy in ((0, 0), (0.25, 0.1), (-0.2, 0.15), (0.1, -0.3)):
                    Pw = np.array([[g0[0] + dx, g0[1] + dy, Z_PLANE]])
                    uv, front = cam.world_to_pixel(Pw, x, y, z, yaw, roll, pitch)
                    assert front[0]
                    g = cam.pixel_to_ground(uv[0, 0], uv[0, 1], x, y, z, yaw, roll, pitch, z_plane=Z_PLANE)
                    peor = max(peor, math.hypot(g[0] - Pw[0, 0], g[1] - Pw[0, 1]))
    assert peor < 1e-3


def test_caso_a_mano_pitch_5_nariz_abajo():
    """Dron en (0, 0, 1.5), yaw 0, pitch +5° (nariz abajo), píxel central, cámara real (offset (0.20, 0, -0.05)),
    pivote de Motive b = 0.07 m sobre el punto de la pose (bias_z = -0.07).

    Centro óptico: p + b·ẑ + Ry(5°)·(offset - b·ẑ) = (0.20·cos5 - 0.12·sin5, 0, 1.57 - 0.20·sin5 - 0.12·cos5).
    Eje óptico: Ry(5°)·(0, 0, -1) = (-sin5, 0, -cos5) -> mira hacia ATRÁS.
    Piso (z = 0.02): x = x_c - tan5·(z_c - 0.02) ≈ 0.0652 m; nivelado daba 0.200 m: ≈ 0.135 m hacia atrás."""
    cam = cam_real()
    b = cam.pivote_m
    assert b == pytest.approx(0.07)
    s, c, t = math.sin(D(5)), math.cos(D(5)), math.tan(D(5))
    xc, zc = 0.20 * c - (0.05 + b) * s, 1.5 + b - 0.20 * s - (0.05 + b) * c
    esperado = xc - t * (zc - 0.02)
    assert esperado == pytest.approx(0.0652, abs=5e-4)
    gx, gy = cam.pixel_to_ground(cam.cx, cam.cy, 0.0, 0.0, 1.5, 0.0, 0.0, D(5), z_plane=0.02)
    assert gx == pytest.approx(esperado, abs=1e-9) and gy == pytest.approx(0.0, abs=1e-9)
    nivelado = cam.pixel_to_ground(cam.cx, cam.cy, 0.0, 0.0, 1.5, 0.0, z_plane=0.02)
    assert nivelado[0] == pytest.approx(0.20)
    assert gx - nivelado[0] == pytest.approx(-0.1348, abs=1e-3)     # ≈ -1.5·tan5 - efecto del offset y el pivote
    # con yaw = 90° el "atrás" del dron es -y del mundo
    gx, gy = cam.pixel_to_ground(cam.cx, cam.cy, 0.0, 0.0, 1.5, D(90), 0.0, D(5), z_plane=0.02)
    assert gx == pytest.approx(0.0, abs=1e-9) and gy == pytest.approx(esperado, abs=1e-9)


def test_caso_a_mano_roll_5_ve_a_la_izquierda():
    """roll +5° (lado izquierdo arriba): el eje óptico Rx(5°)·(0,0,-1) = (0, sin5, -cos5) mira a +y."""
    cam = cam_real()
    b = cam.pivote_m
    s, c, t = math.sin(D(5)), math.cos(D(5)), math.tan(D(5))
    # p + b·ẑ + Rx(5°)·(0.20, 0, -0.05 - b) = (0.20, (0.05 + b)·sin5, 1.5 + b - (0.05 + b)·cos5)
    yc, zc = (0.05 + b) * s, 1.5 + b - (0.05 + b) * c
    gx, gy = cam.pixel_to_ground(cam.cx, cam.cy, 0.0, 0.0, 1.5, 0.0, D(5), 0.0, z_plane=0.02)
    assert gx == pytest.approx(0.20, abs=1e-9)
    assert gy == pytest.approx(yc + t * (zc - 0.02), abs=1e-9) and gy > 0.12


def test_offset_se_rota_con_la_actitud_completa():
    cam = cam_real(offset=(0.0, 0.0, -0.10))
    b = cam.pivote_m
    _, p_c = cam.pose(0.0, 0.0, 1.5, 0.0, 0.0, D(10))
    # el centro óptico gira alrededor del pivote (b sobre el punto de la pose): se mueve en x con el pitch
    assert p_c[0] == pytest.approx(-(0.10 + b) * math.sin(D(10)))
    assert p_c[2] == pytest.approx(1.5 + b - (0.10 + b) * math.cos(D(10)))
    R_wc, _ = cam.pose(0.0, 0.0, 1.5, D(20), D(3), D(4))
    assert np.allclose(R_wc, R_mundo_cuerpo(D(20), D(3), D(4)) @ R_BODY_CAM0)


def test_offset_gira_alrededor_del_pivote_de_motive():
    """La pose es el punto corregido = pivote + bias_z·ẑ_MUNDO (no fijo al cuerpo). El vector fijo al cuerpo es
    pivote -> lente = offset - b·ẑ (offset medido nivelado desde el punto corregido)."""
    cam = cam_real(offset=(0.212, 0.0325, 0.019))
    b = cam.pivote_m
    p = np.array([0.3, -0.2, 1.5])
    pivote = p + b * np.array([0.0, 0.0, 1.0])
    for yaw, roll, pitch in ((0.3, 0.0, D(10)), (0.3, D(10), 0.0), (D(-120), D(-7), D(12)), (1.0, 0.0, 0.0)):
        _, p_c = cam.pose(*p, yaw, roll, pitch)
        esperado = pivote + R_mundo_cuerpo(yaw, roll, pitch) @ (cam.offset - b * np.array([0.0, 0.0, 1.0]))
        assert np.allclose(p_c, esperado, atol=1e-12)
        assert np.linalg.norm(p_c - pivote) == pytest.approx(np.linalg.norm(cam.offset - [0, 0, b]))
    # nivelado: exactamente p + Rz·offset (el término del pivote es 0)
    _, p_c = cam.pose(*p, 0.7)
    assert (p_c == p + rot_z(0.7) @ cam.offset).all()
    # sin el pivote (b = 0), a 10° el centro de la imagen caería ≈ b·sin10 ≈ 12 mm de donde debe
    sin_piv = CameraModel(640, 480, 0, 0, 0, cam.offset, fx=cam.fx, fy=cam.fy, cx=cam.cx, cy=cam.cy)
    a = cam.pixel_to_ground(cam.cx, cam.cy, *p, 0.3, 0.0, D(10), z_plane=0.02)
    c = sin_piv.pixel_to_ground(cam.cx, cam.cy, *p, 0.3, 0.0, D(10), z_plane=0.02)
    assert math.hypot(a[0] - c[0], a[1] - c[1]) == pytest.approx(b * math.sin(D(10)), rel=0.05)
    # el pivote sale de optitrack.bias_z_dron1/2 y se conserva al reescalar
    assert pivote_de_config({}) == pytest.approx(0.07)
    assert pivote_de_config({"optitrack": {"bias_z_dron1": -0.05, "bias_z_dron2": -0.07}}) == pytest.approx(0.06)
    assert pivote_de_config({"optitrack": {"bias_z_dron1": "x", "bias_z_dron2": True}}) == pytest.approx(0.07)
    assert pivote_de_config(CFG_REAL) == pytest.approx(0.07)
    assert cam.scaled(320, 240).pivote_m == b


def test_huella_inclinada_se_desplaza():
    cam = cam_real()
    plano = np.array(cam.footprint(0.0, 0.0, 1.5, 0.0))
    incl = np.array(cam.footprint(0.0, 0.0, 1.5, 0.0, 0.0, D(8)))
    assert len(incl) == 4
    assert incl[:, 0].mean() < plano[:, 0].mean() - 0.15       # nariz abajo: la huella se va hacia atrás


def test_inclinacion_y_correccion():
    for r, p in ((0.0, 0.0), (D(5), 0.0), (0.0, D(-7)), (D(10), D(10)), (D(-3), D(4))):
        ez = R_mundo_cuerpo(D(123), r, p) @ [0, 0, 1]
        assert inclinacion(r, p) == pytest.approx(math.acos(ez[2]), abs=1e-12)
        assert inclinacion(r, p) == pytest.approx(math.acos(math.cos(r) * math.cos(p)), abs=1e-12)
    assert corregir_actitud(0.1, 0.2) == (0.1, 0.2)
    assert corregir_actitud(0.1, 0.2, -1, 1) == (-0.1, 0.2)
    assert corregir_actitud(0.1, 0.2, 1, 1, 0.02, -0.01) == pytest.approx((0.08, 0.21))


# ============================================================================ protocolo
def _crudo(**idx):
    """Telemetría de 64 doubles con los índices base 1 dados."""
    v = list(P._TEL.unpack(P.pack_telemetry({"explorer_id": 1})))
    for k, val in idx.items():
        v[int(k[1:]) - 1] = val
    return P.unpack_telemetry(P._TEL.pack(*v))


def test_protocolo_lee_y61_63():
    tel = _crudo(y61=0.05, y62=-0.08, y63=1.0)
    assert tel["explorer_roll"] == 0.05 and tel["explorer_pitch"] == -0.08 and tel["explorer_att_ok"] is True
    assert tel["reserved"] == [0.0]                                          # y(64)
    for malo in (dict(y63=0.0), dict(y63=float("nan")), dict(y61=float("nan"), y63=1.0),
                 dict(y62=float("inf"), y63=1.0), dict(y63=2.0)):
        t = _crudo(**{"y61": 0.05, "y62": -0.08, **malo})
        assert t["explorer_att_ok"] is False and t["explorer_roll"] is None and t["explorer_pitch"] is None, malo
    # modelo viejo: y(61:64) = 0 -> sin actitud
    viejo = _crudo()
    assert viejo["explorer_att_ok"] is False and viejo["explorer_roll"] is None


def test_protocolo_ida_y_vuelta():
    raw = P._TEL.unpack(P.pack_telemetry({"explorer_roll": 0.03, "explorer_pitch": -0.04, "explorer_att_ok": 1}))
    assert raw[60:64] == (0.03, -0.04, 1.0, 0.0)
    tel = P.unpack_telemetry(P.pack_telemetry({"explorer_roll": 0.03, "explorer_pitch": -0.04, "explorer_att_ok": True}))
    assert (tel["explorer_roll"], tel["explorer_pitch"], tel["explorer_att_ok"]) == (0.03, -0.04, True)
    # sin att_ok o con un ángulo no finito: 0, 0, 0 (como el TELEMETRY PACK)
    for tel_in in ({"explorer_roll": 0.03, "explorer_pitch": -0.04, "explorer_att_ok": 0},
                   {"explorer_roll": float("nan"), "explorer_pitch": 0.1, "explorer_att_ok": 1},
                   {"explorer_roll": None, "explorer_pitch": None, "explorer_att_ok": 1}):
        assert P._TEL.unpack(P.pack_telemetry(tel_in))[60:64] == (0.0, 0.0, 0.0, 0.0)


# ============================================================================ configuración
def test_config_actitud():
    a = actitud_vision({})
    assert a == {"usar": True, "signo_roll": 1.0, "signo_pitch": 1.0, "offset_roll": 0.0, "offset_pitch": 0.0}
    assert actitud_vision(CFG_REAL) == a
    assert actitud_vision({"vision": {"usar_actitud": False}})["usar"] is False
    for raro in ("abc", 0, None, 1, [True]):
        assert actitud_vision({"vision": {"usar_actitud": raro}})["usar"] is True, raro
    b = actitud_vision({"vision": {"actitud_signos": [-1, 1], "actitud_offset_deg": [1.0, -2.0]}})
    assert (b["signo_roll"], b["signo_pitch"]) == (-1.0, 1.0)
    assert (b["offset_roll"], b["offset_pitch"]) == pytest.approx((D(1.0), D(-2.0)))
    c = actitud_vision({"vision": {"actitud_signos": [2, "x"], "actitud_offset_deg": [True, 99]}})
    assert (c["signo_roll"], c["signo_pitch"], c["offset_roll"], c["offset_pitch"]) == (1.0, 1.0, 0.0, 0.0)
    assert actitud_vision({"vision": {"actitud_signos": "abc"}})["signo_roll"] == 1.0
    assert filtros_vision({})["inclinacion_max_deg"] == 15.0
    assert filtros_vision({"vision": {"inclinacion_max_deg": None}})["inclinacion_max_deg"] is None
    assert filtros_vision({"vision": {"inclinacion_max_deg": "abc"}})["inclinacion_max_deg"] == 15.0
    assert filtros_vision(CFG_REAL)["inclinacion_max_deg"] == 15.0


# ============================================================================ historial de poses
def test_historial_interpola_roll_pitch():
    h = PoseHistory()
    h.add(10.0, 0.0, 0.0, 1.5, 0.0, D(2), D(-4))
    h.add(10.2, 0.2, 0.0, 1.5, 0.0, D(4), D(-2))
    x, y, z, yaw, roll, pitch = h.at(10.1)
    assert x == pytest.approx(0.1) and roll == pytest.approx(D(3)) and pitch == pytest.approx(D(-3))
    # cerca de ±180° (rigid body al revés): por el camino corto, como el yaw
    h2 = PoseHistory()
    h2.add(1.0, 0, 0, 1, 0, D(179), 0.0)
    h2.add(2.0, 0, 0, 1, 0, D(-179), 0.0)
    r = h2.at(1.5)[4]
    assert abs(math.degrees(math.atan2(math.sin(r), math.cos(r)))) == pytest.approx(180.0)
    # un vecino sin actitud (y(63) = 0): None, no se mezcla con 0
    h.add(10.4, 0.4, 0.0, 1.5, 0.0)
    assert h.at(10.3)[4:] == (None, None)
    assert h.at(10.45)[4:] == (None, None)                 # más allá del último: el último tal cual
    # ráfaga con la misma hora: gana la actitud más reciente
    h.add(10.4, 0.4, 0.0, 1.5, 0.0, D(1), D(1))
    assert h.at(10.4)[4:] == pytest.approx((D(1), D(1)))
    # solo uno de los dos ángulos -> sin actitud
    h.add(10.6, 0.4, 0.0, 1.5, 0.0, D(1), None)
    assert h.at(10.6)[4:] == (None, None)


# ============================================================================ servicio de visión
class _Cam:
    def wait_newer(self, *a):
        return None


class _Det:
    name = "prueba"
    classes = {0: "fuego", 1: "toma"}

    def __init__(self, out=()):
        self.out = list(out)

    def detect(self, frame):
        return list(self.out)


FRAME = np.zeros((480, 640, 3), np.uint8)
CAJA_CENTRO = (1, 0.9, 305, 225, 335, 255)


def make_cfg(**vision):
    cfg = copy.deepcopy(CFG_REAL)
    cfg["pista"] = {"vertices": copy.deepcopy(N)}
    cfg.setdefault("web", {})["camara_latencia_s"] = 0.0
    cfg["vision"].update(vision)
    return cfg


def servicio(cfg, det, gate=True, poses=None):
    return VisionService(_Cam(), poses or PoseHistory(), lambda: gate, lambda t: None, lambda: cfg, det, det.name)


def _pose_que_mira(cam, x, y, yaw, roll, pitch, z=1.5):
    """Pose del dron para que el centro de la imagen caiga en (x, y) con esa actitud."""
    g = cam.pixel_to_ground(cam.cx, cam.cy, 0.0, 0.0, z, yaw, roll, pitch, z_plane=Z_PLANE)
    return (x - g[0], y - g[1], z, yaw, roll, pitch)


@pytest.mark.parametrize("yaw", [0.0, D(-60)])
def test_toma_renderizada_inclinada_se_geolocaliza_con_actitud(yaw):
    cfg = make_cfg()
    cam = CameraModel.from_config(cfg["camara"], cfg, image_size=(640, 480))
    toma = {"cls": 1, "x": 0.0, "y": 1.28}
    for roll, pitch in ((0.0, D(8)), (D(-8), 0.0), (D(5), D(-6))):
        pose = _pose_que_mira(cam, toma["x"] + 0.05, toma["y"] - 0.03, yaw, roll, pitch)
        img = render_view(cam, pose, N, [toma], rng=np.random.default_rng(1))
        vs = servicio(cfg, ColorDetector())
        d = [d for d in vs.process(img, 100.0, pose=pose) if d["cls"] == 1][0]
        assert d["descartada"] is None and d["inclinacion_deg"] == pytest.approx(math.degrees(inclinacion(roll, pitch)), abs=0.1)
        assert math.hypot(d["xy"][0] - toma["x"], d["xy"][1] - toma["y"]) < 0.02
        assert vs.status()["actitud"]["ok"] and vs.status()["actitud"]["texto"].startswith("corrigiendo")
        # la misma imagen con usar_actitud = false: el error de la suposición nivelada (≈ 1.5·tan 8° ≈ 0.2 m)
        cfg_off = make_cfg(usar_actitud=False)
        vs2 = servicio(cfg_off, ColorDetector())
        d2 = [d for d in vs2.process(img, 100.0, pose=pose) if d["cls"] == 1][0]
        assert math.hypot(d2["xy"][0] - toma["x"], d2["xy"][1] - toma["y"]) > 0.15
        assert d2["inclinacion_deg"] is None and "ignorada" in vs2.status()["actitud"]["texto"]


def test_descarte_por_inclinacion():
    cfg = make_cfg(tam_max_m=None, dist_max_tubo_toma_m=None)
    vs = servicio(cfg, _Det([CAJA_CENTRO]))
    for k in range(3):
        dets = vs.process(FRAME, 100.0 + k, pose=(0.0, 0.0, 1.5, 0.0, D(3), D(20)))
    d = dets[0]
    assert d["descartada"] == MOTIVO_INCLINACION and d["xy"] is not None
    assert d["inclinacion_deg"] == pytest.approx(math.degrees(inclinacion(D(3), D(20))), abs=0.1)
    assert vs.targets() == ([], []) and vs.n_obs == 0                     # no entra al Clusterer
    st = vs.status()
    assert st["descartadas"] == {**{m: 0 for m in MOTIVOS}, MOTIVO_INCLINACION: 3, "total": 3}
    assert "INCLINADO" in st["actitud"]["texto"] and st["actitud"]["inclinacion_deg"] > 15
    # 14° pasa
    assert vs.process(FRAME, 104.0, pose=(0.0, 0.0, 1.5, 0.0, 0.0, D(14)))[0]["descartada"] is None
    # sin gate (fuera de MISSION) se marca pero no se cuenta
    vs_ng = servicio(cfg, _Det([CAJA_CENTRO]), gate=False)
    assert vs_ng.process(FRAME, 100.0, pose=(0.0, 0.0, 1.5, 0.0, 0.0, D(20)))[0]["descartada"] == MOTIVO_INCLINACION
    assert vs_ng.status()["descartadas"]["total"] == 0
    # gris en el video, con la "ó" quitada para Hershey
    vs.process(FRAME, time.time(), pose=(0.0, 0.0, 1.5, 0.0, 0.0, D(20)))
    assert tuple(vs.annotate(FRAME)[240, 305]) == (150, 150, 150)
    vs.reset()
    assert vs.status()["descartadas"]["total"] == 0


def test_inclinacion_max_null_y_usar_actitud_false_no_descartan():
    pose = (0.0, 0.0, 1.5, 0.0, 0.0, D(20))
    vs = servicio(make_cfg(tam_max_m=None, dist_max_tubo_toma_m=None, inclinacion_max_deg=None), _Det([CAJA_CENTRO]))
    d = vs.process(FRAME, 100.0, pose=pose)[0]
    assert d["descartada"] is None and vs.targets()[1]                    # candidato
    cam = vs.camera_model(640, 480)
    esperado = cam.pixel_to_ground(320, 240, *pose, z_plane=Z_PLANE)
    assert d["xy"] == [round(esperado[0], 3), round(esperado[1], 3)]
    # usar_actitud = false: nivelado (como antes) y sin descarte por inclinación
    vs2 = servicio(make_cfg(usar_actitud=False, tam_max_m=None, dist_max_tubo_toma_m=None), _Det([CAJA_CENTRO]))
    d2 = vs2.process(FRAME, 100.0, pose=pose)[0]
    nivelado = cam.pixel_to_ground(320, 240, *pose[:4], z_plane=Z_PLANE)
    assert d2["descartada"] is None and d2["xy"] == [round(nivelado[0], 3), round(nivelado[1], 3)]
    st = vs2.status()["actitud"]
    assert st["usar"] is False and st["disponible"] is True and st["ok"] is False


def test_modelo_viejo_sin_actitud_sigue_funcionando():
    """y(63) = 0 (modelo de Simulink viejo): la pose llega sin roll/pitch -> nivelado, sin descarte, aviso."""
    cfg = make_cfg(tam_max_m=None, dist_max_tubo_toma_m=None)
    poses = PoseHistory()
    vs = servicio(cfg, _Det([CAJA_CENTRO]), poses=poses)
    tel = P.unpack_telemetry(P.pack_telemetry({"explorer_id": 1, "drones": {1: {"tracked": 1, "x": 0.3, "y": 0.1,
                                                                                 "z": 1.5, "yaw": 0.4}}}))
    assert tel["explorer_att_ok"] is False
    poses.add(100.0, 0.3, 0.1, 1.5, 0.4, tel["explorer_roll"], tel["explorer_pitch"])
    d = vs.process(FRAME, 100.0)[0]
    cam = vs.camera_model(640, 480)
    ref = cam.pixel_to_ground(320, 240, 0.3, 0.1, 1.5, 0.4, z_plane=Z_PLANE)
    assert d["descartada"] is None and d["xy"] == [round(ref[0], 3), round(ref[1], 3)]
    st = vs.status()["actitud"]
    assert st["disponible"] is False and st["ok"] is False and st["texto"].startswith("no disponible")
    assert st["frames_nivelados"] == 1 and st["frames_con_actitud"] == 0
    # pose de 4 valores (pruebas viejas) = igual
    assert vs.process(FRAME, 101.0, pose=(0.3, 0.1, 1.5, 0.4))[0]["xy"] == d["xy"]


def test_signos_configurables():
    """actitud_signos = [1, -1] equivale a mandar el pitch con el signo contrario."""
    cfg = make_cfg(tam_max_m=None, dist_max_tubo_toma_m=None, actitud_signos=[1, -1])
    a = servicio(cfg, _Det([CAJA_CENTRO])).process(FRAME, 100.0, pose=(0.0, 0.0, 1.5, 0.0, D(2), D(6)))[0]["xy"]
    b = servicio(make_cfg(tam_max_m=None, dist_max_tubo_toma_m=None), _Det([CAJA_CENTRO])).process(
        FRAME, 100.0, pose=(0.0, 0.0, 1.5, 0.0, D(2), D(-6)))[0]["xy"]
    assert a == b
    pose, info = pose_para_geo((0, 0, 1.5, 0, D(2), D(6)), actitud_vision(cfg))
    assert pose[4:] == pytest.approx((D(2), D(-6))) and info["ok"]


# ============================================================================ mock
def test_mock_manda_actitud_coherente(cfg_path):
    with open(cfg_path, encoding="utf-8") as fh:
        cfg = json.load(fh)
    d = Driver(cfg)
    tel = P.unpack_telemetry(P.pack_telemetry(d.m.telemetry()))
    assert d.m.E == 0 and tel["explorer_att_ok"] is False                  # sin explorador
    assert d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    tel = P.unpack_telemetry(P.pack_telemetry(d.m.telemetry()))
    assert tel["explorer_att_ok"] is True and (tel["explorer_roll"], tel["explorer_pitch"]) == (0.0, 0.0)  # en el piso
    d.send("START")
    assert d.run_until(lambda: d.m.state == P.MISSION, 15)
    incl, coher_p, coher_r = [], [], []
    for _ in range(int(25 / DT)):
        d.step()
        E = d.m.drones[d.m.E]
        tel = P.unpack_telemetry(P.pack_telemetry(d.m.telemetry()))
        assert tel["explorer_att_ok"] is True
        r, p = tel["explorer_roll"], tel["explorer_pitch"]
        incl.append(math.degrees(inclinacion(r, p)))
        c, s = math.cos(E.yaw), math.sin(E.yaw)
        abx, aby = c * E.a[0] + s * E.a[1], -s * E.a[0] + c * E.a[1]
        if abs(abx) > 0.3:
            coher_p.append(math.copysign(1, abx) == math.copysign(1, p))  # acelera adelante -> nariz abajo
        if abs(aby) > 0.3:
            coher_r.append(math.copysign(1, aby) == -math.copysign(1, r))  # acelera a la izquierda -> roll < 0
    assert coher_p and all(coher_p) and coher_r and all(coher_r)
    assert np.percentile(incl, 99) < 15.0 and max(incl) > 0.5
    # empuje = a + g: exacto con R_mundo_cuerpo
    r, p = actitud_de_aceleracion(np.array([1.0, -0.5, 0.3]), D(30))
    emp = R_mundo_cuerpo(D(30), r, p) @ [0, 0, 1]
    assert np.allclose(emp, np.array([1.0, -0.5, G + 0.3]) / np.linalg.norm([1.0, -0.5, G + 0.3]))
    # explorador sin rastreo o modelo viejo: y(63) = 0
    d.m.drones[d.m.E].tracked = 0
    assert P.unpack_telemetry(P.pack_telemetry(d.m.telemetry()))["explorer_att_ok"] is False
    d.m.drones[d.m.E].tracked = 1
    d.m.actitud_en_tel = False
    raw = P._TEL.unpack(P.pack_telemetry(d.m.telemetry()))
    assert raw[60:63] == (0.0, 0.0, 0.0)


# ============================================================================ estación
def test_estacion_guarda_actitud_y_huella(cfg_path):
    from test_alertas_objetivos_sim import _station
    from types import SimpleNamespace
    st = _station(cfg_path)
    base = {"mission_state": P.MISSION, "explorer_id": 2, "responder_id": 1,
            "drones": {1: {"tracked": 1}, 2: {"tracked": 1, "x": 0.2, "y": -0.3, "z": 1.5, "yaw": 0.5}}}
    tel = P.unpack_telemetry(P.pack_telemetry({**base, "explorer_roll": D(3), "explorer_pitch": D(-6),
                                               "explorer_att_ok": 1}))
    st._on_telemetry(tel, 50.0)
    assert st.poses.at(50.0)[4:] == pytest.approx((D(3), D(-6)))
    st.bridge = SimpleNamespace(telemetry=tel, fresh=lambda: True, status=lambda: {})
    cam = st._cam_model()
    assert st.huella() == cam.footprint(0.2, -0.3, 1.5, 0.5, D(3), D(-6))
    assert st.huella() != cam.footprint(0.2, -0.3, 1.5, 0.5)
    assert st.explorer_attitude() == pytest.approx((D(3), D(-6)))
    assert not any("Actitud" in a["msg"] for a in st.avisos())
    # tarjeta del explorador: cruda (y(61:62)) y usada (con signos / offset) sin depender del video
    assert st.actitud_ui() == {"usar": True, "ajustada": False, "cruda": [3.0, -6.0], "usada": [3.0, -6.0]}
    st.cfg.data["vision"].update(actitud_signos=[1, -1], actitud_offset_deg=[1.0, 0.0])
    ui = st.actitud_ui()
    assert ui["ajustada"] and ui["cruda"] == [3.0, -6.0] and ui["usada"] == pytest.approx([2.0, 6.0])
    st.cfg.data["vision"].update(actitud_signos=[1, 1], actitud_offset_deg=[0.0, 0.0])
    # usar_actitud = false: huella nivelada
    st.cfg.data["vision"]["usar_actitud"] = False
    assert st.huella() == cam.footprint(0.2, -0.3, 1.5, 0.5)
    assert st.actitud_ui()["usada"] is None and st.actitud_ui()["cruda"] == [3.0, -6.0]
    st.cfg.data["vision"]["usar_actitud"] = True
    # modelo viejo (y(63) = 0): pose sin actitud, huella nivelada y aviso amarillo
    viejo = P.unpack_telemetry(P.pack_telemetry(base))
    st._on_telemetry(viejo, 51.0)
    assert st.poses.at(51.0)[4:] == (None, None)
    st.bridge = SimpleNamespace(telemetry=viejo, fresh=lambda: True, status=lambda: {})
    assert st.huella() == cam.footprint(0.2, -0.3, 1.5, 0.5)
    assert any(a["nivel"] == "amarillo" and "y(63) = 0" in a["msg"] for a in st.avisos())
    assert st.actitud_ui()["cruda"] is None and st.actitud_ui()["usada"] is None
