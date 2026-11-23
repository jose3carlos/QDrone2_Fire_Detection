"""Servicio de visión: carga del detector enchufable (vision/detector.py) y publicación de objetivos."""
import math
import sys

import numpy as np

from app.mock_world import render_view
from app.geo import CameraModel
from app.vision_service import PoseHistory, VisionService, load_detector

CFG = {"camara": {"resolucion": [640, 480], "hfov_deg": 69.4, "vfov_deg": 42.5, "rotacion_montaje": 0,
                  "offset_camara_m": [0.2, 0.0, -0.05]},
       "vision": {"modelo": "vision/modelos/mejor.pt", "confianza_min": 0.5, "radio_cluster_m": 0.25, "min_observaciones": 3},
       "web": {"camara_latencia_s": 0.0}}


def test_carga_vision_detector(tmp_path):
    """La estación usa ``vision/detector.py`` (clase Detector con detect(frame_bgr)) cuando existe."""
    (tmp_path / "vision").mkdir()
    (tmp_path / "vision" / "__init__.py").write_text("", encoding="utf-8")
    (tmp_path / "vision" / "detector.py").write_text(
        "class Detector:\n"
        "    def __init__(self, modelo):\n"
        "        self.modelo = modelo\n"
        "    def detect(self, frame_bgr):\n"
        "        return [(1, 0.9, 300, 220, 340, 260)]\n", encoding="utf-8")
    for k in [k for k in sys.modules if k == "vision" or k.startswith("vision.")]:
        del sys.modules[k]
    try:
        det, name, err = load_detector("vision", str(tmp_path), CFG)
        assert det is not None and err == "" and "vision/detector.py" in name
        assert det.modelo.replace("\\", "/").endswith("vision/modelos/mejor.pt")
    finally:
        for k in [k for k in sys.modules if k == "vision" or k.startswith("vision.")]:
            del sys.modules[k]
        sys.path.remove(str(tmp_path))
    det, name, _ = load_detector("auto", str(tmp_path / "no_hay"), CFG, synthetic=True)
    assert name.startswith("falso")
    det, name, err = load_detector("vision", str(tmp_path / "no_hay"), CFG)
    assert det is None and "vision" in err


class _Cam:
    def wait_newer(self, *a):
        return None


def test_servicio_publica_tras_min_observaciones():
    published = []
    poses = PoseHistory()
    vs = VisionService(_Cam(), poses, lambda: True, published.append, lambda: CFG,
                       *load_detector("falso", ".", CFG, synthetic=True))
    cam = CameraModel.from_config(CFG["camara"])
    verts = [[-1.2, 1.2], [1.2, 1.2], [-1.2, -1.2], [1.2, -1.2]]
    truth = [{"cls": 1, "x": 0.1, "y": 0.6}]
    for k in range(3):
        pose = (-0.2 + 0.05 * k, 0.5, 1.5, math.radians(10))
        poses.add(100.0 + k, *pose)
        img = render_view(cam, pose, verts, truth, rng=np.random.default_rng(k))
        vs.process(img, 100.0 + k)
        assert (len(published[-1]) == 1) == (k == 2)    # se publica con la 3a observación
    tid, cls, x, y, conf = published[-1][0]
    assert (tid, cls) == (1, 1) and math.hypot(x - 0.1, y - 0.6) < 0.03 and conf > 0.5


def test_historial_de_pose_interpola():
    h = PoseHistory()
    h.add(10.0, 0.0, 0.0, 1.0, math.radians(170))
    h.add(10.2, 0.2, -0.2, 1.2, math.radians(-170))   # cruza ±180°
    x, y, z, yaw, roll, pitch = h.at(10.1)
    assert roll is None and pitch is None               # sin actitud (y(63) = 0): no se inventa
    assert (round(x, 6), round(y, 6), round(z, 6)) == (0.1, -0.1, 1.1)
    assert abs(math.degrees(math.atan2(math.sin(yaw), math.cos(yaw)))) > 179.9
    assert h.at(12.0) is None                           # demasiado viejo
