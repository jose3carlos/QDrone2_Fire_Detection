"""Motor OpenCV de vision/detector.py: forma de entrada del .onnx (480x640 o 640x640) y letterbox.

Se salta lo que necesita el modelo (vision/modelos/mejor.onnx) o las imágenes sintéticas (vision/datos/, no van a git)
si no están.
"""
import glob
import json
import os
import sys

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
PROJECT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, PROJECT)
from vision.detector import Detector, forma_entrada_onnx  # noqa: E402

ONNX = os.path.join(PROJECT, "vision", "modelos", "mejor.onnx")
VAL = os.path.join(PROJECT, "vision", "datos", "sintetico_smoke", "images", "val")


def _iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    i = ix * iy
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i
    return i / u if u > 0 else 0.0


def _gt(img_path, w, h):
    lbl = os.path.splitext(img_path.replace(os.sep + "images" + os.sep, os.sep + "labels" + os.sep))[0] + ".txt"
    out = []
    for ln in open(lbl):
        c, x, y, bw, bh = ln.split()[:5]
        x, y, bw, bh = float(x) * w, float(y) * h, float(bw) * w, float(bh) * h
        out.append((int(c), x - bw / 2, y - bh / 2, x + bw / 2, y + bh / 2))
    return out


def test_forma_desde_json_sin_metadatos(tmp_path):
    falso = tmp_path / "m.onnx"
    falso.write_bytes(b"\x00" * 100)          # sin metadatos de ultralytics
    assert forma_entrada_onnx(str(falso)) == (640, 640)          # ni metadatos ni json: cuadrada por defecto
    (tmp_path / "m.onnx.json").write_text(json.dumps({"imgsz": [480, 640]}), encoding="utf-8")
    assert forma_entrada_onnx(str(falso)) == (480, 640)
    (tmp_path / "m.onnx.json").write_text(json.dumps({"imgsz": 640}), encoding="utf-8")   # formato viejo
    assert forma_entrada_onnx(str(falso)) == (640, 640)


@pytest.fixture(scope="module")
def det():
    if not os.path.isfile(ONNX):
        pytest.skip("no está vision/modelos/mejor.onnx")
    return Detector(ONNX, conf=0.5, backend="opencv")


def test_mejor_onnx_es_480x640_y_coincide_con_json(det):
    assert forma_entrada_onnx(ONNX) == (480, 640)
    with open(ONNX + ".json", encoding="utf-8") as f:
        assert json.load(f)["imgsz"] == [480, 640]
    assert (det._motor.h, det._motor.w, det._motor.n_anclas) == (480, 640, 6300)


def test_letterbox_horizontal_y_vertical(det):
    imgs = sorted(glob.glob(os.path.join(VAL, "*.jpg")))[:20]
    if not imgs:
        pytest.skip("no están las imágenes de vision/datos/sintetico_smoke")
    n_gt = n_ok = n_ok_rot = 0
    for p in imgs:
        im = cv2.imread(p)
        h, w = im.shape[:2]
        gt = _gt(p, w, h)
        dets = det.detect(im)
        n_gt += len(gt)
        n_ok += sum(any(d[0] == g[0] and _iou(d[2:], g[1:]) >= 0.5 for d in dets) for g in gt)
        # imagen vertical (girada 90° horario): entra con letterbox a 480x640 (escala 0.75, bandas a los lados) y
        # las cajas deben volver a los píxeles de la imagen girada: (x, y) -> (h - y, x)
        rot = cv2.rotate(im, cv2.ROTATE_90_CLOCKWISE)
        gt_rot = [(c, h - y2, x1, h - y1, x2) for c, x1, y1, x2, y2 in gt]
        dets_rot = det.detect(rot)
        for _, _, x1, y1, x2, y2 in dets_rot:
            assert 0 <= x1 <= x2 <= rot.shape[1] - 1 and 0 <= y1 <= y2 <= rot.shape[0] - 1
        n_ok_rot += sum(any(d[0] == g[0] and _iou(d[2:], g[1:]) >= 0.5 for d in dets_rot) for g in gt_rot)
    assert n_gt and n_ok / n_gt >= 0.9, (n_ok, n_gt)
    assert n_ok_rot / n_gt >= 0.7, (n_ok_rot, n_gt)


def test_onnx_cuadrado_viejo_sigue_funcionando(tmp_path):
    viejos = sorted(glob.glob(os.path.join(PROJECT, "vision", "modelos", "mejor_640x640_*.onnx")))
    if not viejos:
        pytest.skip("no está el respaldo 640x640 (no va a git)")
    d = Detector(viejos[-1], conf=0.5, backend="opencv")
    assert (d._motor.h, d._motor.w, d._motor.n_anclas) == (640, 640, 8400)
    assert d.detect(np.full((480, 640, 3), 114, np.uint8)) == []
