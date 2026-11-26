"""Fuentes de imagen de la estación. Todas entregan frames BGR uint8 (alto x ancho x 3).

* ``dron``      : video QDF1 del dron con la RealSense (``drone/qdrone2_camera_streamer.py`` en :18800).
                  La IP se toma de ``red.ip_dron{camara.dron}`` (fija: no sigue al explorador de turno; START
                  se bloquea si el explorador no es ``camara.dron``, ver ``Station.camara_info``).
* ``sintetica`` : cámara virtual renderizada desde la pose del explorador (mock y simulador MATLAB).
* ``opencv:N``  : webcam local N (para probar el detector con las piezas reales en la mano).
* ``carpeta``   : recorre ``web/datos/imagenes_prueba``.
* ``ninguna``   : sin video.

Protocolo QDF1 (sin cambios respecto al original): encabezado ``<4sBxxxHHI`` = (b"QDF1", formato,
alto, ancho, bytes) y luego los bytes. Formato 1 = JPEG, 2 = BGR crudo, 3 = profundidad PNG 16 bits.
QUARC usa TCP sin encabezados propios, así que basta un socket normal (no requiere el SDK de Quanser).
"""
from __future__ import annotations

import glob
import os
import socket
import struct
import threading
import time

import numpy as np

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None

FRAME_MAGIC = b"QDF1"
HEADER = struct.Struct("<4sBxxxHHI")
FORMAT_JPEG, FORMAT_RAW_BGR, FORMAT_DEPTH_PNG = 1, 2, 3
MAX_PAYLOAD = 8 * 1024 * 1024


def _recv_exact(sock, n):
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        k = sock.recv_into(view[got:], n - got)
        if k == 0:
            raise ConnectionError("el dron cerró el stream de cámara")
        got += k
    return bytes(buf)


def decode_qdf1(fmt, h, w, payload):
    if fmt == FORMAT_JPEG:
        return cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
    if fmt == FORMAT_RAW_BGR:
        return np.frombuffer(payload, np.uint8).reshape(h, w, 3).copy()
    if fmt == FORMAT_DEPTH_PNG:
        return cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_UNCHANGED)
    raise ValueError(f"formato QDF1 desconocido: {fmt}")


class QDF1Client:
    name = "dron"

    def __init__(self, host, port, timeout=3.0):
        self.host, self.port = host, int(port)
        self.sock = socket.create_connection((host, self.port), timeout=timeout)
        self.sock.settimeout(timeout)
        self.depth = None

    def read(self):
        while True:
            magic, fmt, h, w, n = HEADER.unpack(_recv_exact(self.sock, HEADER.size))
            if magic != FRAME_MAGIC or n > MAX_PAYLOAD:
                raise ValueError("encabezado QDF1 inválido (¿versión distinta del streamer?)")
            payload = _recv_exact(self.sock, n)
            img = decode_qdf1(fmt, h, w, payload)
            if fmt == FORMAT_DEPTH_PNG:
                self.depth = img
                continue
            return img

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


class SyntheticCamera:
    name = "sintetica"

    def __init__(self, render_fn, fps=15.0):
        self.render_fn = render_fn
        self.period = 1.0 / fps
        self.last = 0.0

    def read(self):
        wait = self.period - (time.time() - self.last)
        if wait > 0:
            time.sleep(wait)
        self.last = time.time()
        return self.render_fn()

    def close(self):
        pass


class OpenCVCamera:
    name = "opencv"

    def __init__(self, index, width=640, height=480):
        backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(int(index), backend)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.cap.isOpened():
            raise RuntimeError(f"no se pudo abrir la webcam {index}")

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self):
        self.cap.release()


class FolderCamera:
    name = "carpeta"

    def __init__(self, folder, seconds=2.0, fps=10.0):
        exts = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
        self.files = sorted(f for e in exts for f in glob.glob(os.path.join(folder, e)))
        if not self.files:
            raise RuntimeError(f"no hay imágenes en '{folder}'")
        self.seconds, self.period, self.t0, self.cache = seconds, 1.0 / fps, time.time(), {}

    def read(self):
        time.sleep(self.period)
        path = self.files[int((time.time() - self.t0) / self.seconds) % len(self.files)]
        if path not in self.cache:
            self.cache[path] = cv2.imread(path, cv2.IMREAD_COLOR)
        return self.cache[path].copy()

    def close(self):
        pass


class CameraHub(threading.Thread):
    """Lee la fuente elegida en un hilo y guarda el último frame con su número y hora."""

    def __init__(self, source_fn, factory_fn):
        """``source_fn()`` -> clave de la fuente deseada (si cambia, se reabre).
        ``factory_fn(clave)`` -> objeto con ``read()`` y ``close()`` (o None = sin video)."""
        super().__init__(daemon=True, name="camara")
        self.source_fn, self.factory_fn = source_fn, factory_fn
        self._cv = threading.Condition()
        self._stop = threading.Event()
        self.frame, self.frame_id, self.stamp = None, 0, 0.0
        self.key, self.connected, self.error, self.fps = None, False, "", 0.0

    def run(self):
        while not self._stop.is_set():
            key = self.source_fn()
            self.key = key
            src = None
            try:
                src = self.factory_fn(key)
                if src is None:
                    self.connected, self.error = False, ""
                    self._stop.wait(0.5)
                    continue
                self.connected, self.error = True, ""
                n, t0 = 0, time.time()
                while not self._stop.is_set() and self.source_fn() == key:
                    frame = src.read()
                    if frame is None:
                        time.sleep(0.005)
                        continue
                    if frame.ndim == 2:
                        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    with self._cv:
                        self.frame, self.stamp = frame, time.time()
                        self.frame_id += 1
                        self._cv.notify_all()
                    n += 1
                    if time.time() - t0 >= 1.0:
                        self.fps, n, t0 = n / (time.time() - t0), 0, time.time()
            except Exception as exc:  # p. ej. el streamer del dron aún no arranca: reintenta
                self.connected, self.error, self.fps = False, f"{type(exc).__name__}: {exc}", 0.0
                self._stop.wait(2.0)
            finally:
                if src is not None:
                    try:
                        src.close()
                    except Exception:
                        pass
                self.connected = False

    def latest(self):
        with self._cv:
            return self.frame, self.frame_id, self.stamp

    def wait_newer(self, frame_id, timeout):
        end = time.time() + timeout
        with self._cv:
            while self.frame_id <= frame_id:
                rem = end - time.time()
                if rem <= 0:
                    return None
                self._cv.wait(rem)
            return self.frame, self.frame_id, self.stamp

    def stop(self):
        self._stop.set()

    def status(self):
        return {"fuente": self.key, "conectada": self.connected, "fps": round(self.fps, 1),
                "error": self.error, "frames": self.frame_id,
                "edad_s": round(time.time() - self.stamp, 2) if self.stamp else None}
