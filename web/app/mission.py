"""Logica de mision portada de ``Trayectory_Model.slx``.

* ``TrajectoryStateMachine`` = bloque "Trayectory SM" (chart_34), linea por linea.
* ``BatchCapture``           = "Image Batch Capture" (CaptureFOMOBatch) + "FOMO Inference"
                               (InferFOMOBatch: mayoria estricta y confianza promedio).
* ``MissionController``      = el diagrama raiz: Switch/Home, conexion ImgCaptured -> nextPoint,
                               lazo discreto a Ts = 0.05 s.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

from .fomo import camera_to_model

IDLE, MOVE, VERIFY, CAMERA, ADVANCE, COMPLETE = range(6)
STATE_NAMES = {IDLE: "IDLE", MOVE: "MOVE", VERIFY: "VERIFY", CAMERA: "CAMERA",
               ADVANCE: "ADVANCE", COMPLETE: "COMPLETE"}


class TrajectoryStateMachine:
    def __init__(self, waypoints, cfg):
        self.Ts = float(cfg.get("Ts", 0.05))
        self.window = int(round(float(cfg.get("settling_time", 3.0)) / self.Ts))
        self.x_lim = cfg.get("x_limits", [-1.5, 1.5])
        self.y_lim = cfg.get("y_limits", [-1.5, 1.5])
        self.z = float(cfg.get("z", 1.0))
        self.yaw = float(cfg.get("yaw", 0.0))
        tm = cfg.get("tol_mean", [0.10, 0.10, 0.10, 5.0])
        ti = cfg.get("tol_instant", [0.15, 0.15, 0.15, 8.0])
        self.tol_mean = np.array(tm[:3] + [math.radians(tm[3])])
        self.tol_instant = np.array(ti[:3] + [math.radians(ti[3])])
        self.set_waypoints(waypoints)
        self.reset()

    def set_waypoints(self, waypoints):
        wp = np.asarray(waypoints, dtype=float).reshape(-1, 4)
        if len(wp) == 0:
            raise ValueError("Se necesita al menos un waypoint")
        self.waypoints = wp

    def reset(self):
        self.state = IDLE
        self.index = 0  # 0-based (waypointIndex - 1 en MATLAB)
        self._clear_window()
        self.camera_armed = False

    def _clear_window(self):
        self.history = np.zeros((4, self.window))
        self.err_sum = np.zeros(4)
        self.count = 0
        self.write = 0

    def _clamp(self, p):
        p = p.copy()
        p[0] = min(max(p[0], self.x_lim[0]), self.x_lim[1])
        p[1] = min(max(p[1], self.y_lim[0]), self.y_lim[1])
        p[2], p[3] = self.z, self.yaw
        return p

    def step(self, start, pos, next_point):
        """Devuelve (pointDesired, startCamera, homeToggle)."""
        pos = np.asarray(pos, dtype=float)
        valid = bool(np.all(np.isfinite(pos)))
        if self.state != IDLE:
            desired = self._clamp(self.waypoints[self.index])
        else:
            desired = self._clamp(np.array([pos[0], pos[1], 0, 0]) if valid else np.zeros(4))

        err = np.zeros(4)
        if valid:
            err[:3] = pos[:3] - desired[:3]
            e = pos[3] - desired[3]
            err[3] = math.atan2(math.sin(e), math.cos(e))
        inside = valid and bool(np.all(np.abs(err) <= self.tol_instant))

        s = self.state
        if s == IDLE:
            if start:
                self.index = 0
                self._clear_window()
                self.camera_armed = False
                self.state = MOVE
        elif s == MOVE:
            if inside:
                self._clear_window()
                self.state = VERIFY
        elif s == VERIFY:
            if not inside:
                self._clear_window()
                self.state = MOVE
            else:
                if self.count < self.window:
                    self.history[:, self.write] = err
                    self.err_sum += err
                    self.count += 1
                else:
                    self.err_sum -= self.history[:, self.write]
                    self.history[:, self.write] = err
                    self.err_sum += err
                self.write = (self.write + 1) % self.window
                if self.count >= self.window:
                    mean = self.err_sum / self.window
                    if np.all(np.abs(mean) <= self.tol_mean):
                        self.camera_armed = False
                        self.state = CAMERA
        elif s == CAMERA:
            if not next_point:
                self.camera_armed = True
            elif self.camera_armed:
                self.camera_armed = False
                self.state = ADVANCE
        elif s == COMPLETE:
            pass
        elif s != ADVANCE:
            self.state = COMPLETE

        if self.state == ADVANCE:
            self._clear_window()
            self.camera_armed = False
            if self.index < len(self.waypoints) - 1:
                self.index += 1
                self.state = MOVE
            else:
                self.state = COMPLETE

        if self.state != IDLE:
            desired = self._clamp(self.waypoints[self.index])
        return desired, self.state == CAMERA, self.state != COMPLETE

    def verify_progress(self):
        return min(1.0, self.count / self.window) if self.state == VERIFY else 0.0


class BatchCapture:
    """Captura N fotos en CaptureSeconds (primera al disparo, ultima CaptureSeconds despues),
    las pasa a 180x180 gris como Simulink e infiere el lote completo."""

    def __init__(self, camera, detector, cfg):
        self.camera = camera
        self.detector = detector
        self.N = int(cfg.get("batch_images", 25))
        self.seconds = float(cfg.get("capture_seconds", 3.0))
        self.status = 0  # 0 idle, 1 capturando, 2 completo, -2 frame faltante/tarde, -3 error
        self.ready = False  # ImgCaptured (enganchado hasta el siguiente disparo)
        self.count = 0
        self.error = ""
        self.last = None
        self._thread = None

    @property
    def busy(self):
        return self._thread is not None and self._thread.is_alive()

    def trigger(self, waypoint_index, waypoint):
        if self.busy:
            return False
        # Igual que el flanco de subida en CaptureFOMOBatch: borra el lote y baja ImgCaptured.
        self.ready, self.status, self.count, self.error = False, 1, 0, ""
        self._thread = threading.Thread(target=self._run, args=(waypoint_index, list(waypoint)),
                                        daemon=True, name="batch-capture")
        self._thread.start()
        return True

    def _run(self, wp_index, waypoint):
        info = self.detector.info
        N = self.N
        interval = self.seconds if N == 1 else self.seconds / (N - 1)
        max_late = min(0.05, interval / 2)
        bank, frames = [], []
        _, last_id, _ = self.camera.latest()
        start = time.time()
        try:
            for k in range(N):
                due = start + k * interval
                time.sleep(max(0.0, due - time.time()))
                got = self.camera.wait_newer(last_id, max(0.0, due + max_late - time.time()) + 1e-3)
                if got is None:
                    # misma regla que CaptureFOMOBatch: no se aceptan ranuras perdidas
                    # (se tolera un frame un poco mas viejo si ya llego despues del anterior)
                    frame, fid, _ = self.camera.latest()
                    if frame is None or fid <= last_id:
                        self.status, self.error = -2, f"Faltó el frame {k + 1}/{N} de la cámara"
                        return
                    got = (frame, fid, 0)
                frame, last_id, _ = got
                bank.append(camera_to_model(frame, info))
                frames.append(frame)
                self.count = k + 1
            # ---- InferFOMOBatch ----
            results = [self.detector.infer_model_ready(img) for img in bank]
            positives = [r for r in results if r.detected]
            decision = len(positives) > N / 2.0  # mayoria estricta; empate = False
            avg = round(100.0 * min(1.0, max(0.0, sum(r.confidence for r in positives) / (100.0 * N)))) / 100.0
            best_i = max(range(N), key=lambda i: results[i].confidence)
            self.last = {
                "waypointIndex": wp_index + 1,
                "waypoint": [round(v, 3) for v in waypoint],
                "objectDetected": decision,
                "confidenceAverage": avg,
                "positives": len(positives),
                "n": N,
                "perImage": [round(r.confidence, 1) if r.detected else 0.0 for r in results],
                "bestIndex": best_i,
                "bestResult": results[best_i].to_dict(),
                "time": time.strftime("%H:%M:%S"),
                "_bestImage": bank[best_i],
                "_bestFrame": frames[best_i],
            }
            self.status, self.ready = 2, True
        except Exception as exc:
            self.status, self.error = -3, f"{type(exc).__name__}: {exc}"


class MissionController(threading.Thread):
    def __init__(self, camera, detector, pose_source, setpoints, cfg):
        super().__init__(daemon=True, name="mission")
        self.cfg = cfg
        self.camera = camera
        self.detector = detector
        self.pose = pose_source
        self.setpoints = setpoints
        self.sm = TrajectoryStateMachine(cfg.get("waypoints", []), cfg)
        self.capture = BatchCapture(camera, detector, cfg)
        self.home = np.array(cfg.get("home", [0.0, 0.0, 1.0, 0.0]), dtype=float)
        self.start_flag = False
        self.results = []
        self.commanded = self.home.copy()
        self.position = np.full(4, np.nan)
        self.start_camera = False
        self.home_toggle = True
        self._prev_cam = False
        self.loop_overruns = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()

    # ---- comandos desde la web ----
    def start_mission(self):
        with self._lock:
            if self.sm.state == COMPLETE:
                self._reset_locked()
            self.start_flag = True

    def abort_to_home(self):
        with self._lock:
            self.start_flag = False
            self.sm.state = COMPLETE  # homeToggle = false -> Switch manda Home

    def reset(self):
        with self._lock:
            self._reset_locked()

    def _reset_locked(self):
        self.start_flag = False
        self.sm.reset()
        self.results = []
        self.capture.ready, self.capture.status, self.capture.count = False, 0, 0

    def set_waypoints(self, wps, batch_images=None, capture_seconds=None):
        with self._lock:
            if self.sm.state not in (IDLE, COMPLETE):
                raise RuntimeError("Solo se pueden cambiar los waypoints con la misión detenida")
            self.sm.set_waypoints(wps)
            if batch_images:
                self.capture.N = max(1, int(batch_images))
            if capture_seconds:
                self.capture.seconds = max(0.1, float(capture_seconds))
            self._reset_locked()

    # ---- lazo discreto ----
    def run(self):
        Ts = self.sm.Ts
        next_t = time.perf_counter()
        while not self._stop.is_set():
            with self._lock:
                self._tick(Ts)
            next_t += Ts
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                self.loop_overruns += 1
                next_t = time.perf_counter()

    def _tick(self, Ts):
        self.position = self.pose.read()
        sp, cam, home_toggle = self.sm.step(self.start_flag, self.position, self.capture.ready)
        if cam and not self._prev_cam:
            self.capture.trigger(self.sm.index, sp)
        if self.capture.status == 2 and self.capture.last is not None and \
                (not self.results or self.results[-1] is not self.capture.last):
            self.results.append(self.capture.last)
        self._prev_cam = cam
        # Switch1/Switch del diagrama raiz: con homeToggle falso se comanda Home.
        self.commanded = sp if home_toggle else self.home.copy()
        self.start_camera, self.home_toggle = cam, home_toggle
        self.pose.update(self.commanded, Ts)
        fire = bool(self.results and self.results[-1]["objectDetected"])
        self.setpoints.publish(self.commanded, cam, fire)

    def stop(self):
        self._stop.set()

    def snapshot(self):
        with self._lock:
            cap = self.capture
            return {
                "state": STATE_NAMES[self.sm.state],
                "started": self.start_flag,
                "waypointIndex": self.sm.index + 1,
                "waypoints": self.sm.waypoints.round(3).tolist(),
                "position": [None if not np.isfinite(v) else round(float(v), 3) for v in self.position],
                "setpoint": [round(float(v), 3) for v in self.commanded],
                "startCamera": self.start_camera,
                "homeToggle": self.home_toggle,
                "verifyProgress": round(self.sm.verify_progress(), 3),
                "capture": {"status": cap.status, "count": cap.count, "n": cap.N,
                            "seconds": cap.seconds, "ready": cap.ready, "error": cap.error},
                "results": [{k: v for k, v in r.items() if not k.startswith("_")} for r in self.results],
                "fireWaypoints": [r["waypoint"] for r in self.results if r["objectDetected"]],
                "limits": {"x": self.sm.x_lim, "y": self.sm.y_lim},
                "loopOverruns": self.loop_overruns,
            }
