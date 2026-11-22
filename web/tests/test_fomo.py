"""Detector FOMO original (respaldo, solo fuego). Se salta si falta el runtime LiteRT o el modelo."""
import os

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL = os.path.normpath(os.path.join(ROOT, "..", "models", "fire_hazard-cpp-mcu-v7-impulse-#1.zip"))

fomo = pytest.importorskip("app.fomo")


@pytest.fixture(scope="module")
def det():
    if not os.path.isfile(MODEL):
        pytest.skip("no está el modelo FOMO")
    try:
        return fomo.FomoDetector(MODEL, 0.5)
    except Exception as exc:  # sin ai-edge-litert / tflite_runtime / tensorflow
        pytest.skip(f"runtime TFLite no disponible: {exc}")


def test_preprocesado_como_matlab():
    img = np.zeros((2, 2, 3), np.uint8)
    img[0, 0] = (0, 0, 255)  # BGR rojo puro -> round(0.299*255) = 76
    assert int(fomo.to_gray(img)[0, 0]) == 76


def test_metadatos_e_imagen_negra(det):
    i = det.info
    assert (i.width, i.height, i.channels) == (180, 180, 1) and i.labels == ["Fire"]
    r = det.infer_model_ready(np.zeros((180, 180), np.uint8))
    assert r.status == 2 and not r.detected


def test_adaptador_interfaz_nueva(det):
    from app.vision_service import FomoAdapter
    a = FomoAdapter.__new__(FomoAdapter)
    a._det, a._prep = det, fomo.camera_to_model
    out = a.detect(np.random.default_rng(1).integers(0, 256, (480, 640, 3), dtype=np.uint8))
    for cls, conf, x1, y1, x2, y2 in out:
        assert cls == 0 and 0 <= conf <= 1 and 0 <= x1 <= x2 <= 640 and 0 <= y1 <= y2 <= 480
