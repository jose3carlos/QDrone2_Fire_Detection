"""Fuentes de imagen de la estación. Todas entregan frames BGR uint8 (alto x ancho x 3).

* ``dron``      : video QDF1 del dron con la RealSense (``drone/qdrone2_camera_streamer.py`` en :18800).
                  La IP se toma de ``red.ip_dron{camara.dron}`` (fija: no sigue al explorador de turno; START
                  se bloquea si el explorador no es ``camara.dron``, ver ``Station.camara_info``). Se reconecta sola
                  (Wi-Fi intermitente): espera creciente hasta 2 s y > 2 s sin datos (atasco) = reconectar (``CameraHub``).
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
# Recuperación del video (el Wi-Fi del dron es intermitente): reconexión con espera creciente ESPERA_MIN..ESPERA_MAX
# (se reinicia con el primer frame) y flujo atascado = conectado pero sin recibir NINGÚN byte en ATASCO_S -> reconectar
# (se mide por progreso, no por frame completo: un enlace lento que tarda > ATASCO_S en pasar un JPEG sigue dando video).
CONEXION_TIMEOUT_S = 2.0
ATASCO_S = 2.0
ESPERA_MIN_S, ESPERA_MAX_S = 0.25, 2.0
# Eventos de caída/recuperación con antirrebote: la caída se registra si el video sigue sin llegar CAIDA_MIN_S
# después de cortarse, y la recuperación cuando lleva RECUPERA_MIN_S llegando sin cortes.
CAIDA_MIN_S = 1.0
RECUPERA_MIN_S = 1.0


class FlujoAtascado(TimeoutError):
    """Conectado, pero sin recibir ningún byte en ``ATASCO_S`` s (streamer colgado, Wi-Fi caído sin cerrar)."""


def _recv_exact(sock, n, atasco_s=None):
    """Lee exactamente ``n`` bytes. ``atasco_s``: si pasan más de esos s sin que llegue NINGÚN byte, FlujoAtascado
    (el plazo se reinicia con cada lectura: un enlace lento pero vivo no se corta a mitad de frame)."""
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    if atasco_s is not None:
        sock.settimeout(atasco_s)     # vale para cada recv: cuenta desde el último byte recibido
    while got < n:
        try:
            k = sock.recv_into(view[got:], n - got)
        except socket.timeout:
            raise FlujoAtascado(f"sin datos del stream hace más de {atasco_s:g} s (conectado)") from None
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
    """Cliente del streamer del dron. ``read()`` lanza ``FlujoAtascado`` si pasan ``atasco_s`` s sin recibir ningún
    byte: un streamer que acepta y no manda, o que se corta a mitad de frame sin cerrar el socket (Wi-Fi caído), no
    deja la estación colgada; ``CameraHub`` reconecta. Un enlace lento que sigue mandando bytes no cuenta como atasco
    (aunque un JPEG tarde más de ``atasco_s`` s en llegar)."""
    name = "dron"

    def __init__(self, host, port, timeout=CONEXION_TIMEOUT_S, atasco_s=ATASCO_S):
        self.host, self.port = host, int(port)
        self.atasco_s = float(atasco_s)
        self.sock = socket.create_connection((host, self.port), timeout=timeout)
        self.sock.settimeout(timeout)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except OSError:
            pass
        self.depth = None

    def read(self):
        while True:
            magic, fmt, h, w, n = HEADER.unpack(_recv_exact(self.sock, HEADER.size, self.atasco_s))
            if magic != FRAME_MAGIC or n > MAX_PAYLOAD:
                raise ValueError("encabezado QDF1 inválido (¿versión distinta del streamer?)")
            payload = _recv_exact(self.sock, n, self.atasco_s)
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
    """Lee la fuente elegida en un hilo y guarda el último frame con su número y hora.

    Se recupera solo: si la fuente no abre, lanza al leer o se atasca (``FlujoAtascado`` del cliente QDF1, o
    ``read()`` devolviendo None más de ``atasco_s`` s), se cierra y se reabre tras una espera creciente
    (``espera_min`` y el doble cada vez, tope ``espera_max``; vuelve al mínimo con el primer frame). ``status()``
    trae los contadores (se reinician al cambiar de fuente):

    * ``conexiones`` / ``reconexiones``: aperturas con éxito / las que siguen a la primera; ``fallos``: aperturas
      fallidas (p. ej. el streamer no corre); ``atascos``: conectado sin datos más de ``atasco_s`` s;
    * ``caidas``: el video llegaba y se cortó (un corte cuenta una vez aunque tome varias reconexiones);
      ``caidas_vuelo``: las del vuelo actual o del último (``iniciar_vuelo`` / ``fin_vuelo``, los llama la estación
      con el estado de misión); ``caida_s``: s que lleva el corte actual (None si llega video); ``ultima_caida``;
    * ``edad_s``: edad del último frame (la usa el aviso "Sin video", ``Station.sin_video``).

    Un hilo vigía llama ``evento_fn(texto)`` (Eventos de la estación) con antirrebote: la caída si el video sigue
    cortado ``caida_min_s`` s y la recuperación cuando lleva ``recupera_min_s`` s llegando sin cortes; un corte más
    breve solo suma en los contadores. Solo registra: no actúa sobre nada."""

    def __init__(self, source_fn, factory_fn, evento_fn=None, atasco_s=ATASCO_S, espera_min=ESPERA_MIN_S,
                 espera_max=ESPERA_MAX_S, caida_min_s=CAIDA_MIN_S, recupera_min_s=RECUPERA_MIN_S):
        """``source_fn()`` -> clave de la fuente deseada (si cambia, se reabre).
        ``factory_fn(clave)`` -> objeto con ``read()`` y ``close()`` (o None = sin video)."""
        super().__init__(daemon=True, name="camara")
        self.source_fn, self.factory_fn, self.evento_fn = source_fn, factory_fn, evento_fn
        self.atasco_s, self.espera_min, self.espera_max = float(atasco_s), float(espera_min), float(espera_max)
        self.caida_min_s, self.recupera_min_s = float(caida_min_s), float(recupera_min_s)
        self._cv = threading.Condition()
        self._stop = threading.Event()
        self._lock = threading.Lock()          # contadores y estado de caída (hilo de la cámara, vigía y status)
        self.frame, self.frame_id, self.stamp = None, 0, 0.0
        self.key, self.connected, self.error, self.fps = None, False, "", 0.0
        self.en_vuelo = False
        self._reiniciar_contadores()
        self._vigia = threading.Thread(target=self._vigilar, daemon=True, name="camara-vigia")

    def _reiniciar_contadores(self):
        with self._lock:
            self.conexiones, self.fallos, self.atascos, self.caidas, self.caidas_vuelo = 0, 0, 0, 0, 0
            self.espera = self.espera_min
            self._abajo_desde = None        # hora del último frame antes del corte en curso; None = llega video
            self._arriba_desde = None       # hora en que volvió tras el último corte
            self._caida_evento = None       # hora del corte ya registrado en Eventos (falta su recuperación)
            self._conexiones_al_caer = 0
            self._motivo = ""
            self.ultima_caida = None        # {"hora", "dur_s", "motivo"} del último corte terminado

    @staticmethod
    def _desc(key):
        if key and key[0] == "dron" and len(key) >= 3:
            return f"dron {key[1]}:{key[2]}"
        return " ".join(str(k) for k in key or ())

    # ------------------------------------------------------------------ vuelo (lo llama la estación)
    def iniciar_vuelo(self):
        """La misión entró a un estado de vuelo: ``caidas_vuelo`` vuelve a contar (1 si el video ya está cortado)."""
        with self._lock:
            self.en_vuelo = True
            self.caidas_vuelo = 1 if self._abajo_desde is not None else 0

    def fin_vuelo(self):
        with self._lock:
            self.en_vuelo = False

    # ------------------------------------------------------------------ hilo de la cámara
    def run(self):
        self._vigia.start()
        while not self._stop.is_set():
            key = self.source_fn()
            if key != self.key:
                self._reiniciar_contadores()
                self.key = key
            src, con_frames, ultimo = None, False, time.time()
            try:
                try:
                    src = self.factory_fn(key)
                except Exception:
                    with self._lock:
                        self.fallos += 1
                    raise
                if src is None:
                    self.connected, self.error = False, ""
                    self._stop.wait(0.5)
                    continue
                with self._lock:
                    self.conexiones += 1
                self.connected, self.error = True, ""
                n, t0, ultimo = 0, time.time(), time.time()
                while not self._stop.is_set() and self.source_fn() == key:
                    frame = src.read()
                    if frame is None:
                        if time.time() - ultimo > self.atasco_s:
                            raise FlujoAtascado(f"sin frames hace más de {self.atasco_s:g} s (fuente abierta)")
                        time.sleep(0.005)
                        continue
                    if frame.ndim == 2:
                        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    ahora = time.time()
                    with self._cv:
                        self.frame, self.stamp = frame, ahora
                        self.frame_id += 1
                        self._cv.notify_all()
                    ultimo = ahora
                    if not con_frames:
                        con_frames = True
                        self._video_de_vuelta(ahora)
                    n += 1
                    if ahora - t0 >= 1.0:
                        self.fps, n, t0 = n / (ahora - t0), 0, ahora
            except Exception as exc:  # p. ej. el streamer del dron aún no arranca o se cayó el Wi-Fi: reintenta
                motivo = f"{type(exc).__name__}: {exc}"
                with self._lock:
                    if isinstance(exc, FlujoAtascado):
                        self.atascos += 1
                    if con_frames and self._abajo_desde is None:
                        self.caidas += 1
                        if self.en_vuelo:
                            self.caidas_vuelo += 1
                        # el corte empieza con el último frame (un atasco se detecta ATASCO_S después)
                        self._abajo_desde, self._motivo = ultimo, motivo
                    espera = self.espera
                    self.espera = min(self.espera_max, self.espera * 2)
                self.connected, self.error, self.fps = False, motivo, 0.0
                self._stop.wait(espera)
            finally:
                if src is not None:
                    try:
                        src.close()
                    except Exception:
                        pass
                self.connected = False

    def _video_de_vuelta(self, ahora):
        """Primer frame de una conexión: termina el corte en curso (si había) y reinicia la espera."""
        with self._lock:
            self.espera = self.espera_min
            if self._abajo_desde is not None:
                self.ultima_caida = {"hora": time.strftime("%H:%M:%S", time.localtime(self._abajo_desde)),
                                     "dur_s": round(ahora - self._abajo_desde, 2), "motivo": self._motivo}
                self._abajo_desde, self._arriba_desde = None, ahora

    def _vigilar(self):
        """Eventos de caída / recuperación con antirrebote (cada 0.2 s)."""
        while not self._stop.wait(0.2):
            msg, ahora, desc = None, time.time(), self._desc(self.key)
            with self._lock:
                if self._abajo_desde is not None and self._caida_evento is None \
                        and ahora - self._abajo_desde >= self.caida_min_s:
                    self._caida_evento, self._conexiones_al_caer = self._abajo_desde, self.conexiones
                    msg = (f"Video caído ({desc}): {self._motivo}; se reconecta solo (espera creciente, tope "
                           f"{self.espera_max:g} s)")
                elif self._caida_evento is not None and self._abajo_desde is None and self._arriba_desde is not None \
                        and ahora - self._arriba_desde >= self.recupera_min_s:
                    dur = self._arriba_desde - self._caida_evento
                    n = self.conexiones - self._conexiones_al_caer
                    self._caida_evento = None
                    plural = "reconexión" if n == 1 else "reconexiones"
                    msg = (f"Video de vuelta ({desc}) tras {dur:.1f} s sin video ({n} {plural}; caídas: "
                           f"{self.caidas} en total, {self.caidas_vuelo} en el vuelo)")
            if msg and self.evento_fn is not None:
                try:
                    self.evento_fn(msg)
                except Exception:
                    pass

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
        ahora = time.time()
        with self._lock:
            return {"fuente": self.key, "conectada": self.connected, "fps": round(self.fps, 1),
                    "error": self.error, "frames": self.frame_id,
                    "edad_s": round(ahora - self.stamp, 2) if self.stamp else None,
                    "conexiones": self.conexiones, "reconexiones": max(0, self.conexiones - 1), "fallos": self.fallos,
                    "atascos": self.atascos, "caidas": self.caidas, "caidas_vuelo": self.caidas_vuelo,
                    "en_vuelo": self.en_vuelo,
                    "caida_s": round(ahora - self._abajo_desde, 1) if self._abajo_desde is not None else None,
                    "espera_s": self.espera, "ultima_caida": dict(self.ultima_caida) if self.ultima_caida else None}
