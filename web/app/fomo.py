"""Detector FOMO de Edge Impulse en Python.

Reemplaza al DLL nativo + MEX del proyecto MATLAB. Lee el mismo ZIP de
exportacion (``models/*.zip``), usa el ``.tflite`` que viene dentro y replica:

* el preprocesamiento de ``prepare_image.m`` / ``CaptureFOMOBatch`` (squash +
  vecino mas cercano + luma ITU-R 601 con ``round``),
* el post-procesamiento FOMO del SDK (``process_fomo_i8``: celdas >= umbral
  exportado, fusion de celdas vecinas, caja = celda * factor entero),
* la seleccion de ``ei_detection_selector.cpp`` (la deteccion de mayor confianza
  con confianza >= ``min_confidence``).
"""
from __future__ import annotations

import os
import re
import threading
import time
import zipfile
from dataclasses import dataclass, field

import numpy as np

STATUS_SUCCESS = 1
STATUS_NO_DETECTION = 2


def _interpreter_class():
    errors = []
    try:
        from ai_edge_litert.interpreter import Interpreter  # type: ignore
        return Interpreter, "ai-edge-litert"
    except Exception as exc:  # pragma: no cover - depende del entorno
        errors.append(f"ai_edge_litert: {exc}")
    try:
        from tflite_runtime.interpreter import Interpreter  # type: ignore
        return Interpreter, "tflite-runtime"
    except Exception as exc:  # pragma: no cover
        errors.append(f"tflite_runtime: {exc}")
    try:
        import tensorflow as tf  # type: ignore
        return tf.lite.Interpreter, "tensorflow"
    except Exception as exc:  # pragma: no cover
        errors.append(f"tensorflow: {exc}")
    raise RuntimeError(
        "No se encontro un runtime de TensorFlow Lite. Instala uno con\n"
        "    python -m pip install ai-edge-litert\n" + "\n".join(errors))


@dataclass
class ModelInfo:
    project: str
    deploy_version: int
    width: int
    height: int
    channels: int
    labels: list
    threshold: float
    resize_mode: str
    out_width: int
    out_height: int
    tflite_name: str
    runtime: str = ""


@dataclass
class Detection:
    label: str
    confidence: float  # fraccion [0, 1]
    x: int
    y: int
    width: int
    height: int


@dataclass
class Result:
    status: int
    detected: bool
    label: str
    confidence: float  # porcentaje [0, 100], igual que result.confidence en MATLAB
    box: list  # [x, y, w, h] en coordenadas del modelo (180 x 180)
    detections: list = field(default_factory=list)
    elapsed_ms: float = 0.0
    result_id: int = 0

    def to_dict(self):
        return {
            "status": self.status,
            "detected": self.detected,
            "label": self.label,
            "confidence": round(self.confidence, 2),
            "box": self.box,
            "detections": [d.__dict__ for d in self.detections],
            "elapsedMs": round(self.elapsed_ms, 2),
            "resultID": self.result_id,
        }


def _read_export(path):
    """Devuelve (bytes_tflite, nombre_tflite, model_metadata.h, model_variables.h)."""
    def pick(names):
        tfl = [n for n in names if n.startswith("tflite-model/") and n.endswith(".tflite")]
        if len(tfl) != 1:
            raise ValueError(f"Se esperaba exactamente un .tflite en tflite-model/, hay {len(tfl)}")
        return tfl[0]

    if os.path.isdir(path):
        names = []
        for root, _, files in os.walk(path):
            for f in files:
                names.append(os.path.relpath(os.path.join(root, f), path).replace("\\", "/"))
        name = pick(names)
        with open(os.path.join(path, name), "rb") as fh:
            blob = fh.read()
        with open(os.path.join(path, "model-parameters/model_metadata.h"), encoding="utf-8") as fh:
            meta = fh.read()
        with open(os.path.join(path, "model-parameters/model_variables.h"), encoding="utf-8") as fh:
            variables = fh.read()
        return blob, name, meta, variables
    with zipfile.ZipFile(path) as zf:
        name = pick(zf.namelist())
        return (zf.read(name), name,
                zf.read("model-parameters/model_metadata.h").decode("utf-8"),
                zf.read("model-parameters/model_variables.h").decode("utf-8"))


def _define(text, key, cast=str, default=None):
    m = re.search(r"#define\s+" + key + r"\s+(\S+)", text)
    if not m:
        if default is not None:
            return default
        raise ValueError(f"No se encontro {key} en model_metadata.h")
    return cast(m.group(1).strip('"'))


def _parse_info(meta, variables, tflite_name):
    last_layer = _define(meta, "EI_CLASSIFIER_OBJECT_DETECTION_LAST_LAYER", default="")
    if "FOMO" not in last_layer:
        raise ValueError("Este modulo solo soporta modelos FOMO (EI_CLASSIFIER_LAST_LAYER_FOMO).")
    labels_m = re.search(r"ei_classifier_inferencing_categories_\w+\[\]\s*=\s*\{([^}]*)\}", variables)
    labels = re.findall(r'"([^"]*)"', labels_m.group(1)) if labels_m else []
    chan_m = re.search(r'"(Grayscale|RGB)"\s*//\s*select channels', variables)
    if not chan_m:
        raise ValueError("No se encontro la configuracion de canales del bloque DSP de imagen.")
    out_w = re.search(r"\.out_width\s*=\s*(\d+)", variables)
    out_h = re.search(r"\.out_height\s*=\s*(\d+)", variables)
    resize = _define(meta, "EI_CLASSIFIER_RESIZE_MODE", default="EI_CLASSIFIER_RESIZE_SQUASH")
    return ModelInfo(
        project=_define(meta, "EI_CLASSIFIER_PROJECT_NAME", default="?"),
        deploy_version=_define(meta, "EI_CLASSIFIER_PROJECT_DEPLOY_VERSION", int, 0),
        width=_define(meta, "EI_CLASSIFIER_INPUT_WIDTH", int),
        height=_define(meta, "EI_CLASSIFIER_INPUT_HEIGHT", int),
        channels=1 if chan_m.group(1) == "Grayscale" else 3,
        labels=labels,
        threshold=_define(meta, "EI_CLASSIFIER_OBJECT_DETECTION_THRESHOLD", float),
        resize_mode=resize.replace("EI_CLASSIFIER_RESIZE_", "").lower(),
        out_width=int(out_w.group(1)) if out_w else 0,
        out_height=int(out_h.group(1)) if out_h else 0,
        tflite_name=tflite_name,
    )


# --------------------------------------------------------------------------
# Preprocesamiento (equivalente exacto a las funciones MATLAB del proyecto)
# --------------------------------------------------------------------------

def _round_half_away(x):
    """``round`` de MATLAB (mitades se alejan de cero); numpy redondea a par."""
    return np.floor(np.asarray(x, dtype=np.float64) + 0.5)


def _sample(img, rows_f, cols_f):
    h, w = img.shape[:2]
    r = np.clip(np.floor(rows_f).astype(np.int64), 0, h - 1)
    c = np.clip(np.floor(cols_f).astype(np.int64), 0, w - 1)
    return img[r][:, c]


def resize_squash(img, out_h, out_w):
    h, w = img.shape[:2]
    return _sample(img, (np.arange(out_h) + 0.5) * h / out_h, (np.arange(out_w) + 0.5) * w / out_w)


def to_gray(img_bgr):
    """Luma ITU-R 601 con round de MATLAB. Entrada BGR (convencion OpenCV/Quanser)."""
    if img_bgr.ndim == 2:
        return img_bgr
    b = img_bgr[..., 0].astype(np.float64)
    g = img_bgr[..., 1].astype(np.float64)
    r = img_bgr[..., 2].astype(np.float64)
    return _round_half_away(0.299 * r + 0.587 * g + 0.114 * b).clip(0, 255).astype(np.uint8)


def prepare_image(img_bgr, info: ModelInfo):
    """Igual que ``matlab/prepare_image.m``: cualquier tamano -> uint8 listo para el modelo."""
    img = np.asarray(img_bgr)
    if img.dtype != np.uint8:
        raise ValueError("Se espera una imagen uint8")
    H, W = info.height, info.width
    h, w = img.shape[:2]
    if (h, w) != (H, W):
        mode = info.resize_mode
        if mode == "squash":
            img = resize_squash(img, H, W)
        elif mode == "fit_shortest":
            s = max(H / h, W / w)
            img = _sample(img, (np.arange(H) + 0.5) / s + (h - H / s) / 2,
                          (np.arange(W) + 0.5) / s + (w - W / s) / 2)
        elif mode == "fit_longest":
            s = min(H / h, W / w)
            hh = max(1, min(H, int(np.floor(h * s))))
            ww = max(1, min(W, int(np.floor(w * s))))
            small = resize_squash(img, hh, ww)
            canvas = np.zeros((H, W) + img.shape[2:], np.uint8)
            y0, x0 = (H - hh) // 2, (W - ww) // 2
            canvas[y0:y0 + hh, x0:x0 + ww] = small
            img = canvas
        else:
            raise ValueError(f"El modo de resize '{mode}' requiere imagenes de {W}x{H}")
    if info.channels == 1:
        return to_gray(img)
    if img.ndim == 2:
        img = np.repeat(img[..., None], 3, axis=2)
    return img[..., ::-1].copy()  # BGR -> RGB para modelos RGB


def camera_to_model(frame_bgr, info: ModelInfo):
    """Ruta de la camara en ``Trayectory_Model``: ``VideoDeviceToQUARC`` reescala a
    480x640 y ``CaptureFOMOBatch`` reescala a 180x180 en gris. Se respetan los dos pasos
    para obtener exactamente los mismos pixeles que en Simulink."""
    rgb640 = resize_squash(frame_bgr, 480, 640)
    small = resize_squash(rgb640, info.height, info.width)
    return to_gray(small) if info.channels == 1 else small[..., ::-1].copy()


# --------------------------------------------------------------------------
# Detector
# --------------------------------------------------------------------------

class FomoDetector:
    def __init__(self, model_path, min_confidence=0.5):
        blob, name, meta, variables = _read_export(model_path)
        self.info = _parse_info(meta, variables, name)
        cls, runtime = _interpreter_class()
        self.info.runtime = runtime
        self._interp = cls(model_content=blob)
        self._interp.allocate_tensors()
        self._in = self._interp.get_input_details()[0]
        self._out = self._interp.get_output_details()[0]
        shape = tuple(self._in["shape"])
        expected = (1, self.info.height, self.info.width, self.info.channels)
        if shape != expected:
            raise ValueError(f"Entrada del tflite {shape} != metadatos {expected}")
        o = tuple(self._out["shape"])
        if not self.info.out_width:
            self.info.out_height, self.info.out_width = int(o[1]), int(o[2])
        self.min_confidence = float(min_confidence)
        self._lock = threading.Lock()
        self._result_id = 0
        self.model_path = model_path

    # -- utilidades -----------------------------------------------------
    def _quantize_input(self, img):
        x = img.astype(np.float32).reshape(1, self.info.height, self.info.width, self.info.channels) / 255.0
        dtype = self._in["dtype"]
        scale, zp = self._in.get("quantization", (0.0, 0))
        if np.issubdtype(dtype, np.integer) and scale:
            info = np.iinfo(dtype)
            q = np.round(x / scale) + zp
            return np.clip(q, info.min, info.max).astype(dtype)
        return x.astype(dtype)

    def _dequantize_output(self, raw):
        scale, zp = self._out.get("quantization", (0.0, 0))
        if np.issubdtype(raw.dtype, np.integer) and scale:
            return (raw.astype(np.float32) - zp) * scale
        return raw.astype(np.float32)

    def _fomo_boxes(self, probs):
        """Port de ``fill_result_struct_i8_fomo`` + ``ei_handle_cube`` del SDK."""
        oh, ow, ch = probs.shape
        thr = self.info.threshold
        cubes = []  # [cls, x, y, w, h, conf]
        ys, xs, cs = np.nonzero(probs[:, :, 1:] >= thr)  # orden fila-mayor, igual que el SDK
        for y, x, c in zip(ys.tolist(), xs.tolist(), cs.tolist()):
            vf = float(probs[y, x, c + 1])
            overlapped = False
            for cube in cubes:
                if cube[0] != c:
                    continue
                cx, cy, cw, chh = cube[1], cube[2], cube[3], cube[4]
                if cx + cw < x or cx > x + 1 or cy + chh < y or cy > y + 1:
                    continue
                nx, ny = min(cx, x), min(cy, y)
                cube[3] = max(cx + cw, x + 1) - nx
                cube[4] = max(cy + chh, y + 1) - ny
                cube[1], cube[2] = nx, ny
                cube[5] = max(cube[5], vf)
                overlapped = True
            if not overlapped:
                cubes.append([c, x, y, 1, 1, vf])
        factor = self.info.width // ow
        labels = self.info.labels or ["object"]
        return [Detection(labels[c] if c < len(labels) else str(c), conf,
                          x * factor, y * factor, w * factor, h * factor)
                for c, x, y, w, h, conf in cubes]

    # -- API publica ----------------------------------------------------
    def infer_model_ready(self, img):
        """Inferencia sobre una imagen ya preparada (H x W gris o H x W x 3 RGB)."""
        expected = (self.info.height, self.info.width) + ((3,) if self.info.channels == 3 else ())
        if img.dtype != np.uint8 or img.shape != expected:
            raise ValueError(f"Se esperaba uint8 {expected}, llego {img.dtype} {img.shape}")
        with self._lock:
            t0 = time.perf_counter()
            self._interp.set_tensor(self._in["index"], self._quantize_input(img))
            self._interp.invoke()
            raw = self._interp.get_tensor(self._out["index"])[0]
            probs = self._dequantize_output(raw)
            boxes = self._fomo_boxes(probs)
            elapsed = (time.perf_counter() - t0) * 1000.0
            self._result_id += 1
            rid = self._result_id
        best, best_i = None, -1
        for i, d in enumerate(boxes):
            if not (0 < d.confidence <= 1) or d.confidence < self.min_confidence:
                continue
            if best is None or d.confidence > best.confidence:
                best, best_i = d, i
        if best is None:
            return Result(STATUS_NO_DETECTION, False, "", 0.0, [0, 0, 0, 0], boxes, elapsed, rid)
        return Result(STATUS_SUCCESS, True, best.label, 100.0 * best.confidence,
                      [best.x, best.y, best.width, best.height], boxes, elapsed, rid)

    def infer(self, img_bgr):
        """Equivalente a ``run_ei_inference(imagen)`` en MATLAB."""
        return self.infer_model_ready(prepare_image(img_bgr, self.info))

    def describe(self):
        d = dict(self.info.__dict__)
        d["min_confidence"] = self.min_confidence
        d["model_path"] = os.path.basename(self.model_path)
        return d
