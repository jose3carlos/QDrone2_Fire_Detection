"""Fuentes de imagen. Todas entregan frames BGR uint8 (H x W x 3).

* ``synthetic``          : imagen generada, para probar sin hardware.
* ``folder``             : recorre las imagenes de una carpeta (fotos de prueba).
* ``opencv``             : webcam con OpenCV (equivale al bloque From Video Device).
* ``quanser_video``      : SDK Quanser ``quanser.multimedia.VideoCapture``
                           (``video://localhost:<id>``), webcam o camara CSI local.
* ``quanser_realsense``  : SDK Quanser ``quanser.multimedia.Video3D``, stream COLOR
                           de una camara RGB-D local.
* ``qdrone2_remote``     : frames enviados por ``drone/qdrone2_camera_streamer.py``
                           corriendo en el QDrone2, via ``quanser.communications.Stream``.
"""
from __future__ import annotations

import glob
import os
import struct
import threading
import time

import numpy as np

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None

FRAME_MAGIC = b"QDF1"
HEADER = struct.Struct("<4sBxxxHHI")  # magic, formato(1=jpeg,2=bgr crudo), alto, ancho, bytes
FORMAT_JPEG, FORMAT_RAW_BGR = 1, 2


class SyntheticCamera:
    name = "synthetic"

    def __init__(self, cfg):
        self.w, self.h = int(cfg.get("width", 640)), int(cfg.get("height", 480))
        self.period = 1.0 / float(cfg.get("fps", 30))
        self._t0 = time.time()
        self._last = 0.0

    def read(self):
        now = time.time()
        if now - self._last < self.period:
            time.sleep(max(0.0, self.period - (now - self._last)))
        self._last = time.time()
        t = self._last - self._t0
        x = np.linspace(0, 1, self.w, dtype=np.float32)[None, :]
        y = np.linspace(0, 1, self.h, dtype=np.float32)[:, None]
        base = 60 + 40 * np.sin(6.28 * (x + 0.1 * t)) * np.cos(6.28 * y)
        img = np.repeat(base[..., None], 3, axis=2)
        img[..., 1] += 20
        img += np.random.default_rng().normal(0, 4, img.shape).astype(np.float32)
        return np.clip(img, 0, 255).astype(np.uint8)

    def close(self):
        pass


class FolderCamera:
    name = "folder"

    def __init__(self, cfg):
        if cv2 is None:
            raise RuntimeError("La fuente 'folder' requiere opencv-python")
        folder = cfg.get("folder", "test_images")
        exts = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
        self.files = sorted(f for e in exts for f in glob.glob(os.path.join(folder, e)))
        if not self.files:
            raise RuntimeError(f"No hay imagenes en la carpeta '{folder}'")
        self.seconds = float(cfg.get("folder_seconds_per_image", 2.0))
        self.period = 1.0 / float(cfg.get("fps", 30))
        self._cache = {}
        self._t0 = time.time()

    def read(self):
        time.sleep(self.period)
        idx = int((time.time() - self._t0) / self.seconds) % len(self.files)
        path = self.files[idx]
        if path not in self._cache:
            img = cv2.imread(path, cv2.IMREAD_COLOR)
            if img is None:
                raise RuntimeError(f"No se pudo leer {path}")
            self._cache[path] = img
        return self._cache[path].copy()

    def close(self):
        pass


class OpenCVCamera:
    name = "opencv"

    def __init__(self, cfg):
        if cv2 is None:
            raise RuntimeError("La fuente 'opencv' requiere opencv-python")
        index = int(cfg.get("opencv_index", 0))
        backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(index, backend)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg.get("width", 640)))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg.get("height", 480)))
        if not self.cap.isOpened():
            raise RuntimeError(f"No se pudo abrir la webcam {index}")

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self):
        self.cap.release()


class QuanserVideoCamera:
    """Camara 2D con el SDK de Quanser (misma API que ``pal.utilities.vision.Camera2D``)."""
    name = "quanser_video"

    def __init__(self, cfg):
        from quanser.multimedia import VideoCapture, ImageFormat, ImageDataType
        self.w, self.h = int(cfg.get("width", 640)), int(cfg.get("height", 480))
        url = "video://localhost:" + str(cfg.get("quanser_video_id", "0"))
        self.buf = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        self.cap = VideoCapture(url, float(cfg.get("fps", 30)), self.w, self.h,
                                ImageFormat.ROW_MAJOR_INTERLEAVED_BGR, ImageDataType.UINT8, None, 0)
        self.cap.start()

    def read(self):
        if self.cap.read(self.buf):
            return self.buf.copy()
        time.sleep(0.002)
        return None

    def close(self):
        try:
            self.cap.stop()
        finally:
            self.cap.close()


class QuanserRealSenseCamera:
    """Stream COLOR de una camara RGB-D con ``quanser.multimedia.Video3D``."""
    name = "quanser_realsense"

    def __init__(self, cfg):
        from quanser.multimedia import Video3D, Video3DStreamType, ImageFormat, ImageDataType
        self.w, self.h = int(cfg.get("width", 640)), int(cfg.get("height", 480))
        self.buf = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        self.video3d = Video3D(str(cfg.get("realsense_device", "0")))
        self.stream = self.video3d.stream_open(Video3DStreamType.COLOR, 0, float(cfg.get("fps", 30)),
                                               self.w, self.h, ImageFormat.ROW_MAJOR_INTERLEAVED_BGR,
                                               ImageDataType.UINT8)
        self.video3d.start_streaming()

    def read(self):
        frame = self.stream.get_frame()
        if frame is None:
            time.sleep(0.002)
            return None
        try:
            frame.get_data(self.buf)
        finally:
            frame.release()
        return self.buf.copy()

    def close(self):
        try:
            self.video3d.stop_streaming()
            self.stream.close()
        finally:
            self.video3d.close()


class QDrone2RemoteCamera:
    """Cliente de ``drone/qdrone2_camera_streamer.py`` (Stream de Quanser sobre TCP/IP)."""
    name = "qdrone2_remote"
    BUFFER = 4 * 1024 * 1024

    def __init__(self, cfg):
        from quanser.communications import Stream
        self.uri = cfg.get("remote_uri", "tcpip://192.168.2.15:18800")
        self.stream = Stream()
        self.stream.connect(self.uri, False, 1460, self.BUFFER)
        self.header = bytearray(HEADER.size)
        self.payload = bytearray(self.BUFFER)

    def read(self):
        if self.stream.receive_byte_array(self.header, HEADER.size) != 1:
            raise RuntimeError("El QDrone2 cerro el stream de camara")
        magic, fmt, h, w, n = HEADER.unpack(bytes(self.header))
        if magic != FRAME_MAGIC or n > self.BUFFER:
            raise RuntimeError("Encabezado de frame invalido (version distinta del streamer?)")
        view = memoryview(self.payload)[:n]
        if self.stream.receive_byte_array(view, n) != 1:
            raise RuntimeError("El QDrone2 cerro el stream de camara")
        data = np.frombuffer(view, dtype=np.uint8)
        if fmt == FORMAT_JPEG:
            if cv2 is None:
                raise RuntimeError("Se necesita opencv-python para decodificar JPEG")
            return cv2.imdecode(data, cv2.IMREAD_COLOR)
        return data.reshape(h, w, 3).copy()

    def close(self):
        try:
            self.stream.shutdown()
        except Exception:
            pass
        self.stream.close()


SOURCES = {c.name: c for c in (SyntheticCamera, FolderCamera, OpenCVCamera, QuanserVideoCamera,
                               QuanserRealSenseCamera, QDrone2RemoteCamera)}


class CameraWorker(threading.Thread):
    """Lee la fuente continuamente y guarda el ultimo frame con un numero de frame."""

    def __init__(self, cfg):
        super().__init__(daemon=True, name="camera")
        self.cfg = cfg
        self.source_name = cfg.get("source", "synthetic")
        if self.source_name not in SOURCES:
            raise ValueError(f"Fuente de camara desconocida: {self.source_name}. Opciones: {list(SOURCES)}")
        self._lock = threading.Condition()
        self._frame = None
        self._frame_id = 0
        self._stamp = 0.0
        self._stop = threading.Event()
        self.error = ""
        self.fps = 0.0
        self.connected = False

    def run(self):
        while not self._stop.is_set():
            source = None
            try:
                source = SOURCES[self.source_name](self.cfg)
                self.connected, self.error = True, ""
                count, t0 = 0, time.time()
                while not self._stop.is_set():
                    frame = source.read()
                    if frame is None:
                        continue
                    if frame.ndim == 2:
                        frame = np.repeat(frame[..., None], 3, axis=2)
                    with self._lock:
                        self._frame, self._stamp = frame, time.time()
                        self._frame_id += 1
                        self._lock.notify_all()
                    count += 1
                    if time.time() - t0 >= 1.0:
                        self.fps, count, t0 = count / (time.time() - t0), 0, time.time()
            except Exception as exc:  # reintenta: p. ej. el dron aun no transmite
                self.connected, self.error, self.fps = False, f"{type(exc).__name__}: {exc}", 0.0
                self._stop.wait(2.0)
            finally:
                if source is not None:
                    try:
                        source.close()
                    except Exception:
                        pass

    def latest(self):
        with self._lock:
            return self._frame, self._frame_id, self._stamp

    def wait_newer(self, frame_id, timeout):
        """Espera un frame con id > frame_id. Devuelve (frame, id, stamp) o None."""
        deadline = time.time() + timeout
        with self._lock:
            while self._frame_id <= frame_id:
                remaining = deadline - time.time()
                if remaining <= 0:
                    return None
                self._lock.wait(remaining)
            return self._frame, self._frame_id, self._stamp

    def stop(self):
        self._stop.set()

    def status(self):
        return {"source": self.source_name, "connected": self.connected,
                "fps": round(self.fps, 1), "error": self.error, "frameID": self._frame_id}
