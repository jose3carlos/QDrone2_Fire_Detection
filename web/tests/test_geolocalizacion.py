"""Geometría de cámara: rayo -> suelo, proyección inversa, huella y clusters."""
import math

import numpy as np
import pytest

from app.geo import CameraModel, Clusterer


def cam(rot=0, offset=(0.20, 0.0, -0.05)):
    return CameraModel(640, 480, 69.4, 42.5, rot, offset)


def test_centro_mira_abajo_desde_el_offset():
    c = cam()
    g = c.pixel_to_ground(c.cx, c.cy, 0.3, -0.2, 1.5, 0.0, z_plane=0.02)
    assert g == pytest.approx((0.3 + 0.20, -0.2), abs=1e-9)
    # con yaw = 90° el offset de 20 cm hacia adelante apunta a +y
    g = c.pixel_to_ground(c.cx, c.cy, 0.3, -0.2, 1.5, math.pi / 2, z_plane=0.02)
    assert g == pytest.approx((0.3, -0.2 + 0.20), abs=1e-9)


def test_orientacion_de_la_imagen():
    """Montaje 0: arriba de la imagen = adelante (+x); derecha de la imagen = derecha del dron (-y)."""
    c = cam(offset=(0, 0, 0))
    up = c.pixel_to_ground(c.cx, 0, 0, 0, 1.0, 0.0)
    right = c.pixel_to_ground(639, c.cy, 0, 0, 1.0, 0.0)
    assert up[0] > 0.3 and abs(up[1]) < 1e-9
    assert right[1] < -0.5 and abs(right[0]) < 1e-9
    # montaje 90: arriba de la imagen apunta a la derecha del dron (-y)
    c90 = cam(rot=90, offset=(0, 0, 0))
    up90 = c90.pixel_to_ground(c90.cx, 0, 0, 0, 1.0, 0.0)
    assert up90[1] < -0.3 and abs(up90[0]) < 1e-9


def test_huella_a_15m():
    c = cam(offset=(0, 0, 0))
    f = np.array(c.footprint(0, 0, 1.52, 0.0, z_plane=0.02))
    h = 1.5
    ancho_y = f[:, 1].max() - f[:, 1].min()   # ancho de la imagen = eje y del dron
    largo_x = f[:, 0].max() - f[:, 0].min()
    assert ancho_y == pytest.approx(2 * h * math.tan(math.radians(69.4 / 2)), rel=0.01)
    assert largo_x == pytest.approx(2 * h * math.tan(math.radians(42.5 / 2)), rel=0.01)


@pytest.mark.parametrize("rot", [0, 90, 180, 270])
@pytest.mark.parametrize("yaw_deg", [0, 37, -120])
def test_ida_y_vuelta_mundo_pixel_mundo(rot, yaw_deg):
    c = cam(rot)
    pose = (0.4, -0.7, 1.45, math.radians(yaw_deg))
    rng = np.random.default_rng(rot + yaw_deg + 500)
    pts = np.column_stack([rng.uniform(-0.3, 0.9, 20), rng.uniform(-1.2, -0.2, 20), np.full(20, 0.02)])
    uv, front = c.world_to_pixel(pts, *pose)
    assert front.all()
    for (u, v), p in zip(uv, pts):
        g = c.pixel_to_ground(u, v, *pose, z_plane=0.02)
        assert g == pytest.approx((p[0], p[1]), abs=1e-9)


def test_rayo_que_no_corta_el_suelo():
    c = cam()
    assert c.pixel_to_ground(c.cx, c.cy, 0, 0, 0.01, 0.0, z_plane=0.02) is None  # cámara bajo el plano


def test_imagen_de_otro_tamano_misma_geometria():
    c = cam()
    small = c.scaled(320, 240)
    pose = (0.0, 0.0, 1.5, 0.3)
    a = c.pixel_to_ground(100, 50, *pose)
    b = small.pixel_to_ground((100 + 0.5) / 2 - 0.5, (50 + 0.5) / 2 - 0.5, *pose)
    assert a == pytest.approx(b, abs=1e-6)


def test_cluster_radio_y_minimo():
    cl = Clusterer(radius=0.25, min_obs=3)
    cl.add(0, 0.50, 0.50, 0.9)
    cl.add(0, 0.55, 0.48, 0.8)
    assert cl.targets() == []                     # aún 2 observaciones
    cl.add(1, 0.52, 0.50, 0.9)                    # otra clase: otro cluster
    cl.add(0, 0.60, 0.55, 0.7)                    # 3a observación de fuego
    t = cl.targets()
    assert len(t) == 1 and t[0]["id"] == 1 and t[0]["cls"] == 0 and t[0]["n"] == 3
    assert t[0]["x"] == pytest.approx(0.55) and t[0]["y"] == pytest.approx((0.50 + 0.48 + 0.55) / 3)
    assert t[0]["conf"] == pytest.approx(0.8)
    cl.add(0, 1.2, 0.5, 0.9)                      # fuera del radio: cluster nuevo
    assert len(cl.pending()) == 2
    assert cl.add_manual(1, -1.0, 1.0)["id"] == 2  # manual: se publica de inmediato con el siguiente id


def test_maximo_32_objetivos():
    cl = Clusterer(radius=0.05, min_obs=1)
    for k in range(40):
        cl.add(0, -1.4 + 0.07 * k, 0.0, 0.9)
    assert len(cl.targets()) == 32 and max(t["id"] for t in cl.targets()) == 32
    with pytest.raises(ValueError):
        cl.add_manual(0, 0, 0)


def test_cadena_completa_imagen_sintetica():
    """Render de la cámara virtual -> detector por color -> geolocalización: error < 3 cm."""
    from app.mock_world import ColorDetector, render_view
    c = cam()
    verts = [[-1.2, 1.2], [1.2, 1.2], [-1.2, -1.2], [1.2, -1.2]]
    truth = [{"cls": 0, "x": 0.35, "y": 0.1}, {"cls": 1, "x": -0.25, "y": -0.3}]
    pose = (-0.1, 0.05, 1.5, math.radians(25))
    img = render_view(c, pose, verts, truth, rng=np.random.default_rng(0))
    dets = ColorDetector().detect(img)
    assert sorted(d[0] for d in dets) == [0, 1]
    for cls, conf, x1, y1, x2, y2 in dets:
        g = c.pixel_to_ground((x1 + x2) / 2, (y1 + y2) / 2, *pose, z_plane=0.02)
        t = [a for a in truth if a["cls"] == cls][0]
        assert math.hypot(g[0] - t["x"], g[1] - t["y"]) < 0.03
