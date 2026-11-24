"""Filtros físicos de la estación sobre las detecciones (vision_service.VisionService.process).

Cámara real (CameraModel con los intrínsecos de config/estacion.json) y el explorador a z = 1.5 m
mirando hacia abajo, como vuela en la misión. Las cajas se construyen proyectando al píxel un
cuadrado del tamaño indicado sobre el piso, así el "tamaño real" que mide la estación es conocido.
"""
import copy
import json
import math
import os
import time

import numpy as np
import pytest

from app.config import filtros_vision
from app.geo import CameraModel, dist_polilinea
from app.mock_world import ColorDetector, default_anomalies, render_view
from app.vision_service import MOTIVO_INCLINACION, MOTIVO_TAMANO, MOTIVO_TUBO, Z_PLANE, PoseHistory, VisionService

PROJECT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
with open(os.path.join(PROJECT, "config", "estacion.json"), encoding="utf-8") as _f:
    CFG_REAL = json.load(_f)

N = [[-1.2, 1.2], [1.2, 1.2], [-1.2, -1.2], [1.2, -1.2]]   # la N de la pista (orden = recorrido)
Z = 1.5


def make_cfg(**vision):
    cfg = copy.deepcopy(CFG_REAL)
    cfg["pista"] = {"vertices": copy.deepcopy(N)}
    cfg.setdefault("web", {})["camara_latencia_s"] = 0.0
    # umbral fijo de las pruebas (el del lab en estacion.json puede ser 0.9; los detectores sintéticos dan < 0.9)
    cfg["vision"]["confianza_min"] = 0.5
    cfg["vision"].update(vision)
    return cfg


class _Cam:
    def wait_newer(self, *a):
        return None


class _Det:
    """Detector de prueba: devuelve las cajas que se le pongan."""
    name = "prueba"
    classes = {0: "fuego", 1: "toma"}

    def __init__(self):
        self.out = []

    def detect(self, frame):
        return list(self.out)


def servicio(cfg_holder, gate=True):
    det = _Det()
    published = []
    vs = VisionService(_Cam(), PoseHistory(), lambda: gate, published.append, lambda: cfg_holder["cfg"],
                       det, det.name)
    return vs, det, published


def pose_sobre(x, y, yaw=0.0):
    """Pose del dron para que la cámara (adelantada 0.20 m) quede encima de (x, y)."""
    return (x - 0.20 * math.cos(yaw), y - 0.20 * math.sin(yaw), Z, yaw)


def caja(cfg, pose, x, y, lado, cls=1, conf=0.9):
    """Caja en píxeles de un cuadrado de ``lado`` m sobre el piso centrado en (x, y)."""
    cam = CameraModel.from_config(cfg["camara"], cfg, image_size=(640, 480))
    h = lado / 2.0
    P = [[x - h, y - h, Z_PLANE], [x + h, y - h, Z_PLANE], [x + h, y + h, Z_PLANE], [x - h, y + h, Z_PLANE]]
    uv, front = cam.world_to_pixel(P, *pose)
    assert front.all()
    return (cls, conf, float(uv[:, 0].min()), float(uv[:, 1].min()), float(uv[:, 0].max()), float(uv[:, 1].max()))


FRAME = np.zeros((480, 640, 3), np.uint8)


# ------------------------------------------------------------------------------ geometría
def test_distancia_a_polilinea_respeta_el_orden():
    # (-1.2, 0) está sobre el "lado izquierdo" de un cuadrado, pero la N no tiene ese tramo
    assert dist_polilinea(-1.2, 0.0, N) == pytest.approx(1.2 / math.sqrt(2), abs=1e-9)
    assert dist_polilinea(0.3, 0.3, N) == pytest.approx(0.0, abs=1e-12)       # sobre la diagonal
    assert dist_polilinea(0.0, 1.5, N) == pytest.approx(0.3)                   # sobre el tramo de arriba
    assert dist_polilinea(0.0, 0.0, [[1.0, 1.0]]) is None                      # < 2 vértices
    assert dist_polilinea(0.0, 0.0, []) is None


def test_tamano_de_pieza_real_a_1_5_m():
    """Una toma de 0.10 m a 1.5 m ocupa ~40 px y la estación la mide ~0.10 m."""
    cfg = make_cfg()
    pose = pose_sobre(0.3, 0.3, math.radians(25))
    cls, conf, x1, y1, x2, y2 = caja(cfg, pose, 0.3, 0.3, 0.10)
    assert 30 < x2 - x1 < 60
    cam = CameraModel.from_config(cfg["camara"], cfg, image_size=(640, 480))
    ancho, alto = VisionService.medir_caja(cam, pose, x1, y1, x2, y2)
    # la caja es paralela a la imagen: la pieza girada 25° respecto a la cámara mide L (cos + sin)
    esperado = 0.10 * (math.cos(math.radians(25)) + math.sin(math.radians(25)))
    assert ancho == pytest.approx(esperado, abs=0.01) and alto == pytest.approx(esperado, abs=0.01)
    cls, conf, x1, y1, x2, y2 = caja(cfg, pose_sobre(0.3, 0.3), 0.3, 0.3, 0.10)
    ancho, alto = VisionService.medir_caja(cam, pose_sobre(0.3, 0.3), x1, y1, x2, y2)
    assert ancho == pytest.approx(0.10, abs=0.005) and alto == pytest.approx(0.10, abs=0.005)


# ------------------------------------------------------------------------------ filtros
def test_caja_enorme_tipo_cara_se_descarta_por_tamano():
    h = {"cfg": make_cfg()}
    vs, det, published = servicio(h)
    det.out = [(1, 0.85, 170, 90, 470, 390)]          # una cara de 300 px: ~0.7 m sobre el piso
    for k in range(5):
        dets = vs.process(FRAME, 100.0 + k, pose=pose_sobre(0.3, 0.3))
    d = dets[0]
    assert d["descartada"] == MOTIVO_TAMANO and d["tam_m"] > 0.6
    assert d["xy"] is not None                         # se geolocalizó (para la UI), pero no entró al cluster
    assert vs.targets() == ([], []) and published[-1] == []
    st = vs.status()
    assert st["descartadas"] == {MOTIVO_TAMANO: 5, MOTIVO_TUBO: 0, MOTIVO_INCLINACION: 0, "total": 5}
    assert st["candidatos"] == 0 and st["observaciones"] == 0
    vs.reset()
    assert vs.status()["descartadas"]["total"] == 0    # acumulado desde reset


def test_toma_lejos_del_tubo_se_descarta():
    cfg = make_cfg()
    h = {"cfg": cfg}
    vs, det, _ = servicio(h)
    assert dist_polilinea(0.85, 0.0, N) == pytest.approx(0.60, abs=0.01)
    pose = pose_sobre(0.85, 0.0)
    det.out = [caja(cfg, pose, 0.85, 0.0, 0.10, cls=1)]
    d = vs.process(FRAME, 100.0, pose=pose)[0]
    assert d["descartada"] == MOTIVO_TUBO and d["dist_tubo_m"] == pytest.approx(0.60, abs=0.02)
    assert d["tam_m"] == pytest.approx(0.10, abs=0.01)
    assert vs.targets() == ([], [])
    assert vs.status()["descartadas"][MOTIVO_TUBO] == 1


def test_toma_sobre_el_tubo_es_candidato_y_luego_objetivo():
    cfg = make_cfg()
    vs, det, published = servicio({"cfg": cfg})
    for k in range(3):
        pose = pose_sobre(0.3 + 0.02 * k, 0.3, math.radians(10 * k))
        det.out = [caja(cfg, pose, 0.3, 0.3, 0.10, cls=1)]
        d = vs.process(FRAME, 100.0 + k, pose=pose)[0]
        assert d["descartada"] is None and d["dist_tubo_m"] < 0.02
        if k == 0:
            assert len(vs.targets()[1]) == 1 and vs.status()["candidatos"] == 1
    tid, cls, x, y, conf = published[-1][0]
    assert (tid, cls) == (1, 1) and math.hypot(x - 0.3, y - 0.3) < 0.02
    assert vs.status()["descartadas"]["total"] == 0


def test_fuego_lejos_del_tubo_se_acepta_por_defecto():
    cfg = make_cfg()
    assert cfg["vision"]["dist_max_tubo_fuego_m"] is None
    vs, det, _ = servicio({"cfg": cfg})
    pose = pose_sobre(0.85, 0.0)
    det.out = [caja(cfg, pose, 0.85, 0.0, 0.10, cls=0)]
    d = vs.process(FRAME, 100.0, pose=pose)[0]
    assert d["descartada"] is None and d["dist_tubo_m"] == pytest.approx(0.60, abs=0.02)
    assert len(vs.targets()[1]) == 1


def test_sin_pose_no_hay_xy_ni_filtro():
    cfg = make_cfg()
    vs, det, _ = servicio({"cfg": cfg})
    det.out = [(1, 0.9, 170, 90, 470, 390)]           # sería descartada por tamaño si hubiera pose
    d = vs.process(FRAME, 100.0)[0]                    # historial de poses vacío -> pose None
    assert d["xy"] is None and d["descartada"] is None and d["tam_m"] is None
    assert vs.status()["descartadas"]["total"] == 0 and vs.targets() == ([], [])


def test_baja_confianza_no_se_filtra_ni_se_cuenta():
    cfg = make_cfg()
    vs, det, _ = servicio({"cfg": cfg})
    det.out = [(1, 0.3, 170, 90, 470, 390)]
    d = vs.process(FRAME, 100.0, pose=pose_sobre(0.3, 0.3))[0]
    assert d["xy"] is None and d["descartada"] is None
    assert vs.status()["descartadas"]["total"] == 0


def test_sin_compuerta_se_marca_pero_no_se_cuenta():
    """Fuera de MISSION no se agregan observaciones; tampoco se cuentan las descartadas (sí se marcan)."""
    cfg = make_cfg()
    vs, det, _ = servicio({"cfg": cfg}, gate=False)
    det.out = [(1, 0.9, 170, 90, 470, 390)]
    d = vs.process(FRAME, 100.0, pose=pose_sobre(0.3, 0.3))[0]
    assert d["descartada"] == MOTIVO_TAMANO
    assert vs.status()["descartadas"]["total"] == 0


def test_parametros_en_caliente():
    h = {"cfg": make_cfg()}
    vs, det, _ = servicio(h)
    pose_g = pose_sobre(0.3, 0.3)
    grande = caja(h["cfg"], pose_g, 0.3, 0.3, 0.50, cls=1)
    det.out = [grande]
    assert vs.process(FRAME, 100.0, pose=pose_g)[0]["descartada"] == MOTIVO_TAMANO
    # se sube el umbral en el mismo dict (como cuando Config recarga estacion.json)
    h["cfg"]["vision"]["tam_max_m"] = 0.6
    assert vs.process(FRAME, 101.0, pose=pose_g)[0]["descartada"] is None
    assert vs.status()["filtros"]["tam_max_m"] == 0.6
    # tamaño mínimo: una pieza de 0.10 m con tam_min_m = 0.15 se descarta; con 0 (desactivado) pasa
    h["cfg"]["vision"]["tam_min_m"] = 0.15
    det.out = [caja(h["cfg"], pose_g, 0.3, 0.3, 0.10, cls=1)]
    assert vs.process(FRAME, 102.0, pose=pose_g)[0]["descartada"] == MOTIVO_TAMANO
    h["cfg"]["vision"]["tam_min_m"] = 0.0
    assert vs.process(FRAME, 103.0, pose=pose_g)[0]["descartada"] is None
    # tubo: fuego con límite activado se descarta; toma con el filtro desactivado (null) se acepta
    pose_l = pose_sobre(0.85, 0.0)
    h["cfg"]["vision"]["dist_max_tubo_fuego_m"] = 0.3
    h["cfg"]["vision"]["dist_max_tubo_toma_m"] = None
    det.out = [caja(h["cfg"], pose_l, 0.85, 0.0, 0.10, cls=0), caja(h["cfg"], pose_l, 0.85, 0.0, 0.10, cls=1)]
    fuego, toma = vs.process(FRAME, 104.0, pose=pose_l)
    assert fuego["descartada"] == MOTIVO_TUBO and toma["descartada"] is None
    # otra pista (cfg completo nuevo, como tras guardar vértices capturados): la toma ya queda cerca
    nuevo = make_cfg()
    nuevo["pista"]["vertices"] = [[0.85, -1.0], [0.85, 1.0]]
    h["cfg"] = nuevo
    det.out = [caja(nuevo, pose_l, 0.85, 0.0, 0.10, cls=1)]
    assert vs.process(FRAME, 105.0, pose=pose_l)[0]["descartada"] is None
    # pista con menos de 2 vértices: el filtro del tubo no aplica
    nuevo["pista"]["vertices"] = [[-1.0, -1.0]]
    det.out = [caja(nuevo, pose_l, 0.85, 0.0, 0.10, cls=1)]
    d = vs.process(FRAME, 106.0, pose=pose_l)[0]
    assert d["descartada"] is None and d["dist_tubo_m"] is None


def test_proyeccion_fallida_se_descarta_por_tamano():
    """Si un lado de la caja no corta el piso no se puede medir: se descarta (no se valida a ciegas)."""
    cfg = make_cfg()
    real = CameraModel.from_config(cfg["camara"], cfg, image_size=(640, 480))

    class CamHorizonte:
        def pixel_to_ground(self, u, v, *pose, z_plane=0.02):
            return None if v < 100 else real.pixel_to_ground(u, v, *pose, z_plane=z_plane)

    vs, _, _ = servicio({"cfg": cfg})
    pose = pose_sobre(0.3, 0.3)
    motivo, tam, dist = vs.filtrar(CamHorizonte(), pose, 1, (300, 50, 340, 300), (0.3, 0.3),
                                   filtros_vision(cfg), N)
    assert motivo == MOTIVO_TAMANO and tam is None
    # con los dos filtros de tamaño desactivados, no medir no es motivo para descartar
    sin_tam = make_cfg(tam_max_m=None, tam_min_m=0.0)
    motivo, tam, dist = vs.filtrar(CamHorizonte(), pose, 1, (300, 50, 340, 300), (0.3, 0.3),
                                   filtros_vision(sin_tam), N)
    assert motivo is None and tam is None and dist == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("altura", [1.3, 1.5, 1.8])
def test_piezas_reales_con_altura_en_las_orillas_no_se_descartan(altura):
    """Piezas impresas reales (impresion3d/README.md): fuego ~0.097 x 0.097 x 0.125 m, válvula
    ~0.10 x 0.07 x 0.137 m de alto. En las orillas de la imagen la proyección es oblicua y la parte alta
    se "estira" hacia afuera al proyectarla al piso; con caja holgada (+15 %) siguen bajo 0.35 m."""
    cfg = make_cfg(dist_max_tubo_toma_m=None)            # aquí solo se prueba el tamaño
    cam = CameraModel.from_config(cfg["camara"], cfg, image_size=(640, 480))
    vs, det, _ = servicio({"cfg": cfg})
    pose = (0.0, 0.0, altura, 0.3)
    peor = 0.0
    for cls, (L, W, H) in ((0, (0.097, 0.097, 0.125)), (1, (0.10, 0.07, 0.137))):
        for ox in np.linspace(-1.2, 1.4, 14):
            for oy in np.linspace(-1.0, 1.0, 11):
                for gir in (0.0, math.radians(45), math.radians(60)):
                    c, s = math.cos(gir), math.sin(gir)
                    P = [[ox + c * a - s * b, oy + s * a + c * b, z] for a in (-L / 2, L / 2)
                         for b in (-W / 2, W / 2) for z in (0.0, H)]
                    uv, front = cam.world_to_pixel(P, *pose)
                    u1, v1 = uv.min(0)
                    u2, v2 = uv.max(0)
                    if not front.all() or u1 < 0 or v1 < 0 or u2 > 639 or v2 > 479:
                        continue                         # pieza completa dentro de la imagen
                    pu, pv = 0.075 * (u2 - u1), 0.075 * (v2 - v1)
                    det.out = [(cls, 0.9, u1 - pu, v1 - pv, u2 + pu, v2 + pv)]
                    d = vs.process(FRAME, 100.0, pose=pose)[0]
                    assert d["descartada"] is None, d
                    peor = max(peor, d["tam_m"])
    assert 0.15 < peor < 0.30                            # orillas: ~0.2-0.25 m, margen hasta 0.35


def test_lectura_de_parametros():
    assert filtros_vision({}) == {"tam_max_m": 0.35, "tam_min_m": None, "dist_max_tubo_toma_m": 0.25,
                                  "dist_max_tubo_fuego_m": None, "inclinacion_max_deg": 15.0}
    f = filtros_vision({"vision": {"tam_max_m": None, "dist_max_tubo_toma_m": "abc", "dist_max_tubo_fuego_m": 0.4,
                                   "tam_min_m": 0.05}})
    assert f == {"tam_max_m": None, "tam_min_m": 0.05, "dist_max_tubo_toma_m": 0.25, "dist_max_tubo_fuego_m": 0.4,
                 "inclinacion_max_deg": 15.0}
    assert filtros_vision({"vision": {"tam_max_m": 0, "dist_max_tubo_toma_m": True}})["tam_max_m"] is None
    assert filtros_vision({"vision": {"dist_max_tubo_toma_m": True}})["dist_max_tubo_toma_m"] == 0.25
    # estacion.json real trae las claves con los valores por defecto
    assert filtros_vision(CFG_REAL) == filtros_vision({})


def test_anomalias_del_mock_pasan_los_filtros():
    """El render sintético de las anomalías sobre la N (default_anomalies) no se descarta."""
    cfg = make_cfg()
    vs = VisionService(_Cam(), PoseHistory(), lambda: True, lambda t: None, lambda: cfg, ColorDetector(), "falso")
    cam = CameraModel.from_config(cfg["camara"], cfg, image_size=(640, 480))
    truth = default_anomalies(N)
    rng = np.random.default_rng(3)
    vistos = set()
    for k, a in enumerate(truth):
        for yaw in (0.0, math.radians(45), math.radians(-120)):
            pose = pose_sobre(a["x"] + 0.1, a["y"] - 0.05, yaw)
            dets = vs.process(render_view(cam, pose, N, truth, rng=rng), 100.0 + k, pose=pose)
            for d in dets:
                if d["xy"] is not None:
                    assert d["descartada"] is None, d
                    assert d["tam_m"] < 0.2
                    vistos.add(d["cls"])
    assert vistos == {0, 1}
    assert vs.status()["descartadas"]["total"] == 0


def test_annotate_dibuja_descartadas_en_gris():
    cfg = make_cfg()
    vs, det, _ = servicio({"cfg": cfg})
    det.out = [(1, 0.85, 170, 90, 470, 390)]
    vs.process(FRAME, time.time(), pose=pose_sobre(0.3, 0.3))
    img = vs.annotate(FRAME)
    assert tuple(img[240, 170]) == (150, 150, 150)     # borde izquierdo de la caja, gris
