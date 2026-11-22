"""Servicio de visión de la estación: detector intercambiable + geolocalización + clusters.

Punto de integración con el ingeniero de visión (ARQUITECTURA §7):
    ``vision/detector.py`` debe exponer ``class Detector`` con
    ``detect(frame_bgr) -> list[(cls, conf, x1, y1, x2, y2)]`` (cls 0 = fuego, 1 = toma,
    coordenadas en píxeles del frame recibido).

Flujo por frame: detector → centro de cada caja → rayo de cámara (intrínsecos fx, fy, cx, cy de
``camara.intrinsecos`` o ``vision.intrinsecos_nominales``; HFOV/VFOV solo si faltan;
montaje hacia abajo + ``rotacion_montaje`` + ``offset_camara_m``) → rotado por la pose OptiTrack
del explorador (yaw y altura; roll y pitch ≈ 0) → intersección con z = 0.02 m → cluster por radio
→ con ``min_observaciones`` se publica como objetivo por el puente.

Filtros físicos (independientes del modelo; ``vision.tam_max_m``, ``tam_min_m``,
``dist_max_tubo_toma_m``, ``dist_max_tubo_fuego_m``, leídos en cada frame): antes del cluster se
proyectan al suelo los puntos medios de los 4 lados de la caja y se descarta lo que mide más que una
pieza real (caras, personas) y las tomas lejos de la tubería ``pista.vertices``. Las descartadas no
llegan al Clusterer, pero quedan en ``last["dets"]`` con ``descartada`` = motivo para la UI.
"""
from __future__ import annotations

import bisect
import collections
import importlib
import math
import os
import sys
import threading
import time

from . import protocol as P
from .config import filtros_vision
from .geo import CameraModel, Clusterer, dist_polilinea

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None

Z_PLANE = 0.02
MOTIVO_TAMANO = "tamaño"
MOTIVO_TUBO = "lejos del tubo"
MOTIVOS = (MOTIVO_TAMANO, MOTIVO_TUBO)


# ---------------------------------------------------------------------------- detectores
class FomoAdapter:
    """Respaldo: el detector FOMO original (solo fuego, 180x180 gris) con la interfaz nueva."""
    name = "fomo (Edge Impulse)"
    classes = {0: "fuego"}

    def __init__(self, model_path, conf=0.5):
        from .fomo import FomoDetector, camera_to_model
        self._det = FomoDetector(model_path, conf)
        self._prep = camera_to_model

    def detect(self, frame_bgr):
        info = self._det.info
        res = self._det.infer_model_ready(self._prep(frame_bgr, info))
        sx = frame_bgr.shape[1] / float(info.width)
        sy = frame_bgr.shape[0] / float(info.height)
        out = []
        for d in res.detections:
            if d.confidence >= self._det.min_confidence:
                out.append((0, float(d.confidence), d.x * sx, d.y * sy, (d.x + d.width) * sx, (d.y + d.height) * sy))
        return out


def load_detector(kind, project_dir, cfg, synthetic=False):
    """Devuelve (detector o None, descripción, error)."""
    vis = cfg.get("vision", {})
    conf = float(vis.get("confianza_min", 0.5))
    errors = []
    order = {"auto": (["falso"] if synthetic else ["vision", "fomo"]), "vision": ["vision"],
             "fomo": ["fomo"], "falso": ["falso"], "ninguno": []}.get(kind, ["vision"])
    for k in order:
        try:
            if k == "falso":
                if not synthetic:
                    # sobre imágenes reales detectaría cualquier cosa roja/azul y publicaría objetivos falsos
                    raise RuntimeError("el detector falso solo se permite con la cámara sintética")
                from .mock_world import ColorDetector
                d = ColorDetector()
                return d, d.name, " | ".join(errors)
            if k == "vision":
                if project_dir not in sys.path:
                    sys.path.insert(0, project_dir)
                mod = importlib.import_module("vision.detector")
                model = os.path.join(project_dir, vis.get("modelo", "vision/modelos/mejor.pt"))
                try:
                    d = mod.Detector(model)
                except TypeError:
                    d = mod.Detector()
                return d, f"vision/detector.py ({os.path.basename(model)})", " | ".join(errors)
            if k == "fomo":
                model = os.path.join(project_dir, "models", "fire_hazard-cpp-mcu-v7-impulse-#1.zip")
                d = FomoAdapter(model, conf)
                return d, d.name + " — respaldo, solo fuego", " | ".join(errors)
        except Exception as exc:
            errors.append(f"{k}: {type(exc).__name__}: {exc}")
    return None, "ninguno", " | ".join(errors)


# ---------------------------------------------------------------------------- pose del explorador
class PoseHistory:
    """Historial (≈10 s) de la pose del explorador para sincronizar con la hora del frame."""

    def __init__(self, maxlen=600):
        self.t = collections.deque(maxlen=maxlen)
        self.p = collections.deque(maxlen=maxlen)
        self.lock = threading.Lock()

    def add(self, stamp, x, y, z, yaw):
        with self.lock:
            if self.t and stamp < self.t[-1]:
                return
            if self.t and stamp == self.t[-1]:
                # varios mensajes de telemetría que llegaron juntos (misma hora): gana el más reciente
                self.p[-1] = (x, y, z, yaw)
                return
            self.t.append(stamp)
            self.p.append((x, y, z, yaw))

    def at(self, stamp, max_gap=0.5):
        with self.lock:
            if not self.t:
                return None
            ts = list(self.t)
            i = bisect.bisect_left(ts, stamp)
            if i <= 0:
                return self.p[0] if ts[0] - stamp < max_gap else None
            if i >= len(ts):
                return self.p[-1] if stamp - ts[-1] < max_gap else None
            t0, t1 = ts[i - 1], ts[i]
            a = (stamp - t0) / (t1 - t0) if t1 > t0 else 0.0
            p0, p1 = self.p[i - 1], self.p[i]
            dyaw = math.atan2(math.sin(p1[3] - p0[3]), math.cos(p1[3] - p0[3]))
            return (p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]),
                    p0[2] + a * (p1[2] - p0[2]), p0[3] + a * dyaw)

    def clear(self):
        with self.lock:
            self.t.clear()
            self.p.clear()


# ---------------------------------------------------------------------------- servicio
class VisionService(threading.Thread):
    def __init__(self, camera, poses, gate_fn, publish_fn, cfg_fn, detector=None, detector_name="ninguno",
                 detector_error="", max_hz=10.0, synthetic_fn=None):
        super().__init__(daemon=True, name="vision")
        self.camera, self.poses, self.gate_fn, self.publish_fn, self.cfg_fn = camera, poses, gate_fn, publish_fn, cfg_fn
        self.detector, self.detector_name, self.detector_error = detector, detector_name, detector_error
        self.period = 1.0 / max_hz
        self.synthetic_fn = synthetic_fn or (lambda: False)  # cámara virtual: sin retraso de video
        vis = cfg_fn().get("vision", {})
        self.clusters = Clusterer(vis.get("radio_cluster_m", 0.25), vis.get("min_observaciones", 3))
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self.last = {"frame_id": 0, "stamp": 0.0, "dets": [], "ms": 0.0, "pose": None}
        self.n_frames = self.n_obs = 0
        self.descartadas = {m: 0 for m in MOTIVOS}   # observaciones filtradas desde el último reset
        self.filtros = filtros_vision(cfg_fn())
        self.error = ""
        self.cam_info = {}

    # -------------------------------------------------------------- modelo de cámara
    def detector_classes(self):
        """Clases que puede detectar el detector actual (si no lo declara, se suponen fuego y toma)."""
        d = self.detector
        if d is None:
            return set()
        c = getattr(d, "classes", None)
        return set(int(k) for k in c) if c else {0, 1}

    def camera_model(self, width, height):
        """Modelo de cámara para el frame recibido: intrínsecos de estacion.json (camara.intrinsecos
        o vision.intrinsecos_nominales), girados si el dron gira la imagen (camara.giro_en_dron)."""
        cfg = self.cfg_fn()
        cam = CameraModel.from_config(cfg.get("camara", {}), cfg, image_size=(width, height))
        self.cam_info = {"origen": cam.origen, "fx": round(cam.fx, 1), "fy": round(cam.fy, 1),
                         "cx": round(cam.cx, 1), "cy": round(cam.cy, 1), "w": cam.w, "h": cam.h,
                         "aviso": cam.aviso}
        return cam

    # -------------------------------------------------------------- filtros físicos
    @staticmethod
    def medir_caja(cam, pose, x1, y1, x2, y2):
        """(ancho_m, alto_m) de la caja sobre el piso: distancia entre las proyecciones de los puntos
        medios de los lados izquierdo/derecho y arriba/abajo (plano Z_PLANE). None si alguno no corta el piso."""
        um, vm = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        pts = [cam.pixel_to_ground(u, v, *pose, z_plane=Z_PLANE) for u, v in ((x1, vm), (x2, vm), (um, y1), (um, y2))]
        if any(p is None for p in pts):
            return None
        izq, der, arr, aba = pts
        return math.hypot(der[0] - izq[0], der[1] - izq[1]), math.hypot(aba[0] - arr[0], aba[1] - arr[1])

    def filtrar(self, cam, pose, cls, box, g, filtros, vertices):
        """Aplica los filtros a una detección ya geolocalizada en ``g`` = (x, y).
        Devuelve (motivo o None, tamaño en m o None, distancia al tubo en m o None)."""
        med = self.medir_caja(cam, pose, *box)
        filtro_tam = filtros["tam_max_m"] is not None or filtros["tam_min_m"] is not None
        if med is None and filtro_tam:
            # un lado de la caja no corta el piso: no se puede medir, así que no se puede validar
            # (con los dos filtros de tamaño desactivados no se descarta por esto)
            return MOTIVO_TAMANO, None, None
        tam = max(med) if med is not None else None
        if tam is not None:
            if filtros["tam_max_m"] is not None and tam > filtros["tam_max_m"]:
                return MOTIVO_TAMANO, tam, None
            if filtros["tam_min_m"] is not None and tam < filtros["tam_min_m"]:
                return MOTIVO_TAMANO, tam, None
        dist = dist_polilinea(g[0], g[1], vertices)
        lim = filtros["dist_max_tubo_toma_m"] if cls == 1 else filtros["dist_max_tubo_fuego_m"] if cls == 0 else None
        if dist is not None and lim is not None and dist > lim:
            return MOTIVO_TUBO, tam, dist
        return None, tam, dist

    # -------------------------------------------------------------- procesamiento
    def process(self, frame, stamp, pose=None):
        """Procesa un frame (lo llama el hilo; también se usa directo en las pruebas)."""
        cfg = self.cfg_fn()
        vis = cfg.get("vision", {})
        conf_min = float(vis.get("confianza_min", 0.5))
        filtros = self.filtros = filtros_vision(cfg)         # en cada frame: se pueden cambiar en caliente
        vertices = (cfg.get("pista") or {}).get("vertices") or []
        latency = 0.0 if self.synthetic_fn() else float(cfg.get("web", {}).get("camara_latencia_s", 0.12))
        if pose is None:
            pose = self.poses.at(stamp - latency)
        t0 = time.perf_counter()
        raw = list(self.detector.detect(frame)) if self.detector is not None else []
        ms = (time.perf_counter() - t0) * 1000.0
        cam = self.camera_model(frame.shape[1], frame.shape[0])
        dets = []
        gate = self.gate_fn() and pose is not None
        for cls, conf, x1, y1, x2, y2 in raw:
            cls, conf = int(cls), float(conf)
            d = {"cls": cls, "conf": round(conf, 3), "box": [int(x1), int(y1), int(x2), int(y2)], "xy": None,
                 "descartada": None, "tam_m": None, "dist_tubo_m": None}
            if conf >= conf_min and pose is not None:
                g = cam.pixel_to_ground((x1 + x2) / 2.0, (y1 + y2) / 2.0, *pose, z_plane=Z_PLANE)
                if g is not None:
                    d["xy"] = [round(g[0], 3), round(g[1], 3)]
                    motivo, tam, dist = self.filtrar(cam, pose, cls, (x1, y1, x2, y2), g, filtros, vertices)
                    d["tam_m"] = round(tam, 3) if tam is not None else None
                    d["dist_tubo_m"] = round(dist, 3) if dist is not None else None
                    if motivo:
                        d["descartada"] = motivo
                        if gate:            # se cuentan como las observaciones: solo lo que habría entrado al cluster
                            with self.lock:
                                self.descartadas[motivo] += 1
                    elif gate:
                        with self.lock:
                            self.clusters.add(cls, g[0], g[1], conf)
                        self.n_obs += 1
            dets.append(d)
        with self.lock:
            self.last = {"frame_id": self.n_frames, "stamp": stamp, "dets": dets, "ms": round(ms, 1),
                         "pose": [round(v, 3) for v in pose] if pose else None}
        self.publish()
        return dets

    def publish(self):
        with self.lock:
            tg = self.clusters.targets()
        self.publish_fn([(t["id"], t["cls"], t["x"], t["y"], t["conf"]) for t in tg])

    def add_manual(self, cls, x, y):
        with self.lock:
            c = self.clusters.add_manual(cls, x, y)
        self.publish()
        return c["id"]

    def reset(self, next_id=1):
        with self.lock:
            self.clusters.reset(next_id)
            self.descartadas = {m: 0 for m in MOTIVOS}
        self.publish()

    def ensure_next_id(self, floor):
        with self.lock:
            self.clusters.ensure_next_id(floor)

    def export_state(self):
        with self.lock:
            return self.clusters.to_dict()

    def import_state(self, d):
        with self.lock:
            self.clusters.load_dict(d)
        self.publish()

    def targets(self):
        with self.lock:
            return self.clusters.targets(), self.clusters.pending()

    def blocked(self):
        """Anomalías confirmadas sin id (se acabaron los ids 1..32): no se publican."""
        with self.lock:
            return self.clusters.blocked()

    def run(self):
        last_id = 0
        while not self._stop.is_set():
            got = self.camera.wait_newer(last_id, 1.0)
            if got is None:
                continue
            frame, last_id, stamp = got
            t0 = time.time()
            try:
                self.n_frames += 1
                self.process(frame, stamp)
                self.error = ""
            except Exception as exc:
                self.error = f"{type(exc).__name__}: {exc}"
            rest = self.period - (time.time() - t0)
            if rest > 0:
                self._stop.wait(rest)

    def stop(self):
        self._stop.set()

    def status(self):
        with self.lock:
            last = dict(self.last)
            n_t = len(self.clusters.targets())
            n_p = len(self.clusters.pending())
            desc = dict(self.descartadas)
        desc["total"] = sum(desc[m] for m in MOTIVOS)
        return {"detector": self.detector_name, "detector_error": self.detector_error, "error": self.error,
                "frames": self.n_frames, "observaciones": self.n_obs, "objetivos": n_t, "candidatos": n_p,
                "ultimo": last, "activo": bool(self.gate_fn()),
                "camara_modelo": self.cam_info, "descartadas": desc, "filtros": dict(self.filtros)}

    # -------------------------------------------------------------- dibujo
    def annotate(self, frame):
        with self.lock:
            last = self.last
        img = frame.copy()
        if time.time() - last["stamp"] > 1.5:
            return img
        for d in last["dets"]:
            x1, y1, x2, y2 = d["box"]
            label = f"{P.CLASSES.get(d['cls'], d['cls'])} {d['conf']:.2f}"
            if d.get("descartada"):
                # descartada por los filtros físicos: gris, línea delgada y el motivo
                color = (150, 150, 150)
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 1)
                # cv2.putText (Hershey) no dibuja la ñ
                label += f" DESCARTADA: {d['descartada'].replace('ñ', 'n')}"
                if d["descartada"] == MOTIVO_TUBO and d.get("dist_tubo_m") is not None:
                    label += f" a {d['dist_tubo_m']:.2f} m"
                elif d.get("tam_m") is not None:
                    label += f" {d['tam_m']:.2f} m"
            else:
                color = (0, 90, 255) if d["cls"] == 0 else (255, 160, 40)
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
                if d["xy"]:
                    label += f" ({d['xy'][0]:+.2f}, {d['xy'][1]:+.2f})"
            cv2.putText(img, label, (x1, max(14, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        return img
