"""Servicio de visión de la estación: detector intercambiable + geolocalización + clusters.

Punto de integración con el ingeniero de visión (ARQUITECTURA §7):
    ``vision/detector.py`` debe exponer ``class Detector`` con
    ``detect(frame_bgr) -> list[(cls, conf, x1, y1, x2, y2)]`` (cls 0 = fuego, 1 = toma,
    coordenadas en píxeles del frame recibido).

Flujo por frame: detector → centro de cada caja → rayo de cámara (intrínsecos fx, fy, cx, cy de
``camara.intrinsecos`` o ``vision.intrinsecos_nominales``; HFOV/VFOV solo si faltan;
montaje hacia abajo + ``rotacion_montaje`` + ``offset_camara_m``) → rotado por la pose OptiTrack
del explorador (x, y, z, yaw y, si la telemetría trae y(63) = 1, roll y pitch de y(61:62), todo
interpolado a la hora del frame; ``geo.R_mundo_cuerpo``) → intersección con z = 0.02 m → cluster
por radio → con ``min_observaciones`` se publica como objetivo por el puente.

Actitud (``vision.usar_actitud``, ``actitud_signos``, ``actitud_offset_deg``): sin actitud válida
(modelo de Simulink viejo con y(61:63) = 0, explorador sin rastreo) se geolocaliza nivelado, como
antes, y ``status()["actitud"]`` lo dice ("no disponible"). Con actitud, si la inclinación total
supera ``vision.inclinacion_max_deg`` la detección se descarta con motivo "inclinación".

Filtros físicos (independientes del modelo; ``vision.tam_max_m``, ``tam_min_m``,
``dist_max_tubo_toma_m``, ``dist_max_tubo_fuego_m``, leídos en cada frame): antes del cluster se
proyectan al suelo los puntos medios de los 4 lados de la caja y se descarta lo que mide más que una
pieza real (caras, personas) y las tomas lejos de la tubería ``pista.vertices``. Las descartadas no
llegan al Clusterer, pero quedan en ``last["dets"]`` con ``descartada`` = motivo para la UI.

Registro de vuelo (``obs_fn``, lo pone la estación: ``registro_vuelo.RegistroVuelos``): con el gate activo, cada
frame entrega las observaciones con conf >= ``confianza_min`` y punto en el piso (aceptadas y descartadas, con su
motivo y el tubo más cercano de ``geo.tubo_cercano``) y cuántas quedaron bajo el umbral. Un error ahí nunca tumba
la visión (queda en ``status()["registro_error"]``).
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
import unicodedata

from . import protocol as P
from .config import actitud_vision, filtros_vision
from .geo import (CameraModel, Clusterer, corregir_actitud, dist_polilinea, dist_tubos, inclinacion, tubo_cercano,
                  tubos_pista)

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None

Z_PLANE = 0.02
MOTIVO_TAMANO = "tamaño"
MOTIVO_TUBO = "lejos del tubo"
MOTIVO_INCLINACION = "inclinación"
MOTIVOS = (MOTIVO_TAMANO, MOTIVO_TUBO, MOTIVO_INCLINACION)


def _ascii(texto):
    """Quita acentos y la tilde de la ñ (cv2.putText con Hershey solo dibuja ASCII)."""
    return unicodedata.normalize("NFKD", texto).encode("ascii", "ignore").decode("ascii")


def pose_para_geo(pose, act):
    """Pose del explorador (4 valores, o 6 con roll/pitch que pueden ser None) -> (pose de 6 floats
    para ``CameraModel``, info de actitud).

    ``act`` = ``config.actitud_vision(cfg)``. Con ``act["usar"]`` falso o sin roll/pitch (None) la pose
    sale nivelada (roll = pitch = 0, como la estación sin actitud). Info: ``ok`` (se aplicó la actitud),
    ``disponible`` (la pose traía actitud), ``roll``, ``pitch`` (ya corregidos con signo y offset, rad) e
    ``incl`` (inclinación total, rad; None sin actitud aplicada)."""
    x, y, z, yaw = pose[:4]
    roll, pitch = (tuple(pose[4:6]) + (None, None))[:2]
    disponible = roll is not None and pitch is not None and math.isfinite(roll) and math.isfinite(pitch)
    ok = bool(act["usar"]) and disponible
    if ok:
        roll, pitch = corregir_actitud(roll, pitch, act["signo_roll"], act["signo_pitch"],
                                       act["offset_roll"], act["offset_pitch"])
    else:
        roll = pitch = 0.0
    info = {"ok": ok, "disponible": disponible, "roll": roll if ok else None, "pitch": pitch if ok else None,
            "incl": inclinacion(roll, pitch) if ok else None}
    return (x, y, z, yaw, roll, pitch), info


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
    """Historial (≈10 s) de la pose del explorador para sincronizar con la hora del frame.

    Cada muestra es (x, y, z, yaw, roll, pitch); roll y pitch son None si la telemetría no trae actitud
    válida (y(63) = 0). ``at()`` devuelve siempre 6 valores."""

    def __init__(self, maxlen=600):
        self.t = collections.deque(maxlen=maxlen)
        self.p = collections.deque(maxlen=maxlen)
        self.lock = threading.Lock()

    def add(self, stamp, x, y, z, yaw, roll=None, pitch=None):
        if roll is None or pitch is None:
            roll = pitch = None
        with self.lock:
            if self.t and stamp < self.t[-1]:
                return
            if self.t and stamp == self.t[-1]:
                # varios mensajes de telemetría que llegaron juntos (misma hora): gana el más reciente
                self.p[-1] = (x, y, z, yaw, roll, pitch)
                return
            self.t.append(stamp)
            self.p.append((x, y, z, yaw, roll, pitch))

    @staticmethod
    def _ang(a0, a1, a):
        """Interpolación angular (por el camino corto, como el yaw)."""
        return a0 + a * math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))

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
            if None in (p0[4], p1[4]):
                roll = pitch = None            # a un vecino le falta la actitud: no se mezcla con 0
            else:
                roll, pitch = self._ang(p0[4], p1[4], a), self._ang(p0[5], p1[5], a)
            return (p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]),
                    p0[2] + a * (p1[2] - p0[2]), self._ang(p0[3], p1[3], a), roll, pitch)

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
        # serializa "leer la lista + mandarla" (publish): la lista vacía de reset() siempre sale DESPUÉS de una
        # publicación vieja que estuviera en curso en otro hilo (si no, la vieja podía pisarla y llegar a un Simulink
        # recién reiniciado). publish_fn no vuelve a llamar a publish y solo toma self.lock (export_state): sin deadlock
        self._pub_lock = threading.Lock()
        self._stop = threading.Event()
        self.last = {"frame_id": 0, "stamp": 0.0, "dets": [], "ms": 0.0, "pose": None, "actitud": None}
        self.n_frames = self.n_obs = 0
        self.n_geo_con_actitud = self.n_geo_nivelado = 0   # frames con pose: con actitud aplicada / nivelados
        self.descartadas = {m: 0 for m in MOTIVOS}   # observaciones filtradas desde el último reset
        self.filtros = filtros_vision(cfg_fn())
        self.actitud = actitud_vision(cfg_fn())
        self.error = ""
        self.cam_info = {}
        self.obs_fn = None           # f(stamp, lineas, n_bajo_umbral): registro de vuelo (None = sin registro)
        self.registro_error = ""

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

    def filtrar(self, cam, pose, cls, box, g, filtros, vertices, sin_tubo=None, tubos=None):
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
        # tubos reales (pista.tubos o la ruta partida en los huecos); sin ellos, la polilínea de la ruta
        dist = dist_tubos(g[0], g[1], tubos) if tubos is not None else dist_polilinea(g[0], g[1], vertices, sin_tubo)
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
        act_cfg = self.actitud = actitud_vision(cfg)
        vertices = (cfg.get("pista") or {}).get("vertices") or []
        tubos = tubos_pista(cfg.get("pista"))
        latency = 0.0 if self.synthetic_fn() else float(cfg.get("web", {}).get("camara_latencia_s", 0.12))
        if pose is None:
            pose = self.poses.at(stamp - latency)
        att = None
        if pose is not None:
            # pose de 4 (pruebas, sin actitud) o 6 valores (PoseHistory) -> 6 floats para la geometría
            pose, att = pose_para_geo(pose, act_cfg)
            if att["ok"]:
                self.n_geo_con_actitud += 1
            else:
                self.n_geo_nivelado += 1
        lim_incl = filtros.get("inclinacion_max_deg")
        inclinado = (att is not None and att["ok"] and lim_incl is not None
                     and math.degrees(att["incl"]) > lim_incl)
        t0 = time.perf_counter()
        raw = list(self.detector.detect(frame)) if self.detector is not None else []
        ms = (time.perf_counter() - t0) * 1000.0
        cam = self.camera_model(frame.shape[1], frame.shape[0])
        dets = []
        gate = self.gate_fn() and pose is not None
        reg, n_bajo = [], 0          # para el registro de vuelo: (det, punto en el piso) y cuántas bajo el umbral
        for cls, conf, x1, y1, x2, y2 in raw:
            cls, conf = int(cls), float(conf)
            if conf < conf_min:
                n_bajo += 1
            d = {"cls": cls, "conf": round(conf, 3), "box": [int(x1), int(y1), int(x2), int(y2)], "xy": None,
                 "descartada": None, "tam_m": None, "dist_tubo_m": None,
                 "inclinacion_deg": round(math.degrees(att["incl"]), 1) if att and att["ok"] else None}
            if conf >= conf_min and pose is not None and inclinado:
                # explorador demasiado inclinado: la proyección (y el tamaño / distancia al tubo) no es fiable.
                # Se geolocaliza solo para dibujarla en la UI (si el rayo corta el piso) y no entra al cluster.
                g = cam.pixel_to_ground((x1 + x2) / 2.0, (y1 + y2) / 2.0, *pose, z_plane=Z_PLANE)
                d["xy"] = [round(g[0], 3), round(g[1], 3)] if g is not None else None
                d["descartada"] = MOTIVO_INCLINACION
                if g is not None:
                    reg.append((d, g))
                if gate:
                    with self.lock:
                        self.descartadas[MOTIVO_INCLINACION] += 1
            elif conf >= conf_min and pose is not None:
                g = cam.pixel_to_ground((x1 + x2) / 2.0, (y1 + y2) / 2.0, *pose, z_plane=Z_PLANE)
                if g is not None:
                    d["xy"] = [round(g[0], 3), round(g[1], 3)]
                    reg.append((d, g))
                    motivo, tam, dist = self.filtrar(cam, pose, cls, (x1, y1, x2, y2), g, filtros, vertices, tubos=tubos)
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
        actitud = None
        if att is not None:
            actitud = {"ok": att["ok"], "disponible": att["disponible"], "inclinado": bool(inclinado),
                       "roll_deg": round(math.degrees(att["roll"]), 2) if att["ok"] else None,
                       "pitch_deg": round(math.degrees(att["pitch"]), 2) if att["ok"] else None,
                       "inclinacion_deg": round(math.degrees(att["incl"]), 2) if att["ok"] else None}
        with self.lock:
            self.last = {"frame_id": self.n_frames, "stamp": stamp, "dets": dets, "ms": round(ms, 1),
                         "pose": [round(float(v), 3) for v in pose[:4]] if pose else None, "actitud": actitud}
        if gate and self.obs_fn is not None and (reg or n_bajo):
            self._registrar(stamp, pose, att, reg, n_bajo, tubos)
        self.publish()
        return dets

    def _registrar(self, stamp, pose, att, reg, n_bajo, tubos):
        """Entrega al registro de vuelo las observaciones de este frame (nunca lanza: la visión sigue)."""
        try:
            ok = bool(att and att["ok"])
            base = {"pose": [round(float(v), 3) for v in pose[:3]] + [round(float(pose[3]), 4)],
                    "roll_deg": round(math.degrees(att["roll"]), 2) if ok else None,
                    "pitch_deg": round(math.degrees(att["pitch"]), 2) if ok else None}
            lineas = []
            for d, g in reg:
                tc = tubo_cercano(g[0], g[1], tubos)
                dist = d["dist_tubo_m"] if d["dist_tubo_m"] is not None else (round(tc["dist"], 3) if tc else None)
                tubo = None
                if tc is not None:
                    tubo = {"pieza": tc["pieza"], "tramo": tc["tramo"],
                            "punto": [round(tc["punto"][0], 3), round(tc["punto"][1], 3)],
                            "desplazamiento_m": round(tc["desplazamiento"], 3), "u": round(tc["u"], 3)}
                lineas.append({"frame": self.n_frames, "stamp": round(float(stamp), 3), "cls": d["cls"],
                               "conf": d["conf"], "xy": d["xy"], "motivo": d["descartada"], "dist_tubo_m": dist,
                               "tubo": tubo, "tam_m": d["tam_m"], **base, "inclinacion_deg": d["inclinacion_deg"],
                               "box": d["box"]})
            self.obs_fn(stamp, lineas, n_bajo)
            self.registro_error = ""
        except Exception as exc:
            self.registro_error = f"{type(exc).__name__}: {exc}"

    def publish(self):
        with self._pub_lock:
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
                "camara_modelo": self.cam_info, "descartadas": desc, "filtros": dict(self.filtros),
                "registro_error": self.registro_error,
                "actitud": self.actitud_status(last.get("actitud"))}

    def actitud_status(self, ult):
        """Estado de la corrección de actitud para la UI (``ult`` = ``last["actitud"]`` del último frame)."""
        usar = bool(self.actitud.get("usar", True))
        disponible = bool(ult and ult.get("disponible"))
        if not usar:
            texto = "ignorada (vision.usar_actitud = false): geolocalización nivelada"
        elif ult is None:
            texto = "sin pose del explorador todavía"
        elif ult.get("ok"):
            texto = "corrigiendo roll/pitch" + (" · INCLINADO: detecciones descartadas" if ult.get("inclinado") else "")
        else:
            texto = ("no disponible (y(63) = 0: modelo de Simulink sin actitud o explorador sin rastreo): "
                     "geolocalización nivelada")
        out = {"usar": usar, "disponible": disponible, "ok": bool(ult and ult.get("ok")), "texto": texto,
               "roll_deg": None, "pitch_deg": None, "inclinacion_deg": None,
               "inclinacion_max_deg": self.filtros.get("inclinacion_max_deg"),
               "signos": [self.actitud.get("signo_roll", 1.0), self.actitud.get("signo_pitch", 1.0)],
               "frames_con_actitud": self.n_geo_con_actitud, "frames_nivelados": self.n_geo_nivelado}
        if ult and ult.get("ok"):
            out.update(roll_deg=ult["roll_deg"], pitch_deg=ult["pitch_deg"], inclinacion_deg=ult["inclinacion_deg"])
        return out

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
                # cv2.putText (Hershey) no dibuja la ñ ni los acentos
                label += f" DESCARTADA: {_ascii(d['descartada'])}"
                if d["descartada"] == MOTIVO_INCLINACION:
                    if d.get("inclinacion_deg") is not None:
                        label += f" {d['inclinacion_deg']:.0f} deg"
                elif d["descartada"] == MOTIVO_TUBO and d.get("dist_tubo_m") is not None:
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
