#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Streamer de cámara del QDrone 2 (corre EN el Jetson del dron explorador). ARQUITECTURA §3.4.

Captura el color de la RealSense D435i (640x480, 15 fps) y lo sirve a la PC en TCP :18800 con el
protocolo QDF1 de siempre: encabezado ``<4sBxxxHHI`` = (b"QDF1", formato, alto, ancho, bytes) y
luego los bytes. Formato 1 = JPEG, 2 = BGR crudo, 3 = profundidad PNG 16 bits (opcional).

Orden de cámaras con ``--camara auto``:
  1) pyrealsense2 (librealsense)          -> la forma más directa de leer la D435i
  2) SDK de Quanser (quanser.multimedia)  -> Video3D, como el streamer original (en el Jetson del laboratorio
                                             pyrealsense2 está roto y se usa esta)
  3) OpenCV / V4L2 (/dev/videoN)          -> último recurso (busca un dispositivo que entregue color)
  ``--camara sintetica`` genera imágenes de prueba (para probar la PC sin dron).

Ejemplos (en el dron; lo normal es lanzarlo con ``correr_streamer.sh``, que lo relanza si muere y sobrevive al
cierre de la sesión ssh):
    bash ~/correr_streamer.sh --camara auto --serie <serie>  # en segundo plano, log en ~/streamer_camara.log
    python3 qdrone2_camera_streamer.py                       # auto, 640x480 @ 15 fps, puerto 18800
    python3 qdrone2_camera_streamer.py --camara opencv --dispositivo 4
    python3 qdrone2_camera_streamer.py --rotar 180           # gira la imagen antes de mandarla
    python3 qdrone2_camera_streamer.py --profundidad         # agrega frames de profundidad (formato 3)

Se recupera solo (el Wi-Fi del dron es intermitente):
* Cámara (``Captura``): si no abre, si ``read()`` lanza o si entrega frames vacíos más de ``--vacios-s`` s, la
  cierra y la reabre con espera creciente (0.5 s, 1, 2 ... hasta ``--espera-max`` s); nunca termina por eso. Con
  ``--camara auto`` reabre primero la que funcionó. Si una lectura, apertura o cierre se queda colgado más de
  ``--vigia-s`` s el proceso sale con código 3 para que ``correr_streamer.sh`` lo relance (un hilo colgado no se
  puede matar); si lleva más de ``--sin-camara-s`` s sin un frame bueno (no abre o no entrega, p. ej. 'device busy'
  por handles que la librería dejó abiertos) sale con código 4 y el relanzado empieza con un proceso limpio.
* Clientes: hasta ``--max-clientes`` a la vez, cada uno con su hilo (uno muerto no frena a los demás ni a la
  captura; si llegan más, se cierra el más viejo). Cada cliente recibe el frame más reciente (si es lento se salta
  frames). Envío con tiempo límite (``--timeout-envio`` s), TCP keepalive y TCP_USER_TIMEOUT (Linux): un cliente
  que desapareció sin cerrar (Wi-Fi caído) se suelta en segundos, y una PC que se reconecta entra de inmediato
  como cliente nuevo aunque el socket viejo siga medio abierto.
* Cada ``--cada-estado`` s imprime una línea ``Estado:`` con fps de captura y de envío, reaperturas de la cámara,
  clientes activos / totales / desconectados y frames enviados.

Montaje: la cámara mira hacia abajo. En la PC, ``camara.rotacion_montaje`` de config/estacion.json
describe la imagen TAL COMO LLEGA a la PC. Si aquí usas ``--rotar G``, réstale G a ese valor
(módulo 360) y pon G en ``camara.giro_en_dron`` (para que la estación gire también
fx/fy/cx/cy). Lo normal: no girar aquí y poner el giro en ``rotacion_montaje``.
Al arrancar con pyrealsense2 se imprimen los intrínsecos reales listos para copiar a
``camara.intrinsecos`` de estacion.json.
Compatible con Python 3.6+ (Jetson): sin dependencias nuevas. Dependencias: numpy; OpenCV recomendado (JPEG).
Para probarlo en la laptop: ``Streamer(args, abrir_fn=...)`` acepta cualquier fuente con ``read()`` y ``close()``
(``tests/test_streamer_robusto.py``).
"""
from __future__ import print_function

import argparse
import os
import select
import signal
import socket
import struct
import sys
import threading
import time

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None

HEADER = struct.Struct("<4sBxxxHHI")
FMT_JPEG, FMT_RAW, FMT_DEPTH = 1, 2, 3


_LOG_LOCK = threading.Lock()


def log(msg):
    """Una línea con la hora en la consola (o en el log de correr_streamer.sh); segura entre hilos."""
    linea = "[%s] %s\n" % (time.strftime("%H:%M:%S"), msg)
    with _LOG_LOCK:
        try:
            sys.stdout.write(linea)
            sys.stdout.flush()
        except (OSError, ValueError, UnicodeError):
            pass


# ----------------------------------------------------------------------------- cámaras
class RealSensePyrs(object):
    nombre = "pyrealsense2"

    def __init__(self, a):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipe = rs.pipeline()
        cfg = rs.config()
        if a.serie:
            cfg.enable_device(a.serie)
        cfg.enable_stream(rs.stream.color, a.ancho, a.alto, rs.format.bgr8, int(a.fps))
        if a.profundidad:
            cfg.enable_stream(rs.stream.depth, a.ancho, a.alto, rs.format.z16, int(a.fps))
        prof = self.pipe.start(cfg)
        try:
            self._iniciar(a, prof)
        except Exception:
            self.close()    # sin el pipeline abierto en el proceso (si no, las reaperturas dan 'device busy')
            raise

    def _iniciar(self, a, prof):
        rs = self.rs
        self.align = rs.align(rs.stream.color) if a.profundidad else None
        intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        import math
        hfov = 2 * math.degrees(math.atan(intr.width / (2.0 * intr.fx)))
        vfov = 2 * math.degrees(math.atan(intr.height / (2.0 * intr.fy)))
        print("Intrínsecos color %dx%d: fx=%.1f fy=%.1f cx=%.1f cy=%.1f -> HFOV=%.1f° VFOV=%.1f°"
              % (intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy, hfov, vfov), flush=True)
        # la estación usa fx, fy, cx, cy directamente (más exacto que HFOV/VFOV): copiar a estacion.json
        print('  Copia en config/estacion.json -> "camara": { ..., "intrinsecos": '
              '{"%dx%d": {"fx": %.2f, "fy": %.2f, "cx": %.2f, "cy": %.2f}} }'
              % (intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy), flush=True)
        print("  (son los intrínsecos de la imagen SIN girar; si usas --rotar pon el mismo valor en "
              "camara.giro_en_dron y la estación los gira sola)", flush=True)
        self.depth = None

    def read(self):
        frames = self.pipe.wait_for_frames(2000)
        if self.align is not None:
            frames = self.align.process(frames)
            d = frames.get_depth_frame()
            self.depth = np.asanyarray(d.get_data()).copy() if d else None
        c = frames.get_color_frame()
        return np.asanyarray(c.get_data()).copy() if c else None

    def close(self):
        try:
            self.pipe.stop()
        except Exception:
            pass


class RealSenseQuanser(object):
    nombre = "quanser Video3D"

    def __init__(self, a):
        from quanser.multimedia import Video3D, Video3DStreamType, ImageFormat, ImageDataType
        self.buf = np.zeros((a.alto, a.ancho, 3), np.uint8)
        self.v, self.s = Video3D(a.serie or "0"), None
        try:
            self.s = self.v.stream_open(Video3DStreamType.COLOR, 0, float(a.fps), a.ancho, a.alto,
                                        ImageFormat.ROW_MAJOR_INTERLEAVED_BGR, ImageDataType.UINT8)
            self.v.start_streaming()
        except Exception:
            try:
                self.close()    # sin handles colgando en el proceso (si no, las reaperturas dan 'device busy')
            except Exception:
                pass
            raise
        self.depth = None

    def read(self):
        f = self.s.get_frame()
        if f is None:
            time.sleep(0.002)
            return None
        try:
            f.get_data(self.buf)
        finally:
            f.release()
        return self.buf.copy()

    def close(self):
        # cada paso aunque el anterior lance (tras un fallo USB stop_streaming suele fallar); el primer error se relanza
        error = None
        for paso in (self.v.stop_streaming, getattr(self.s, "close", None), self.v.close):
            if paso is None:
                continue
            try:
                paso()
            except Exception as exc:
                error = error or exc
        if error is not None:
            raise error


class OpenCVCam(object):
    nombre = "OpenCV/V4L2"

    def __init__(self, a):
        if cv2 is None:
            raise RuntimeError("OpenCV no está instalado")
        indices = [a.dispositivo] if a.dispositivo is not None else list(range(0, 10))
        self.cap = None
        for i in indices:
            cap = cv2.VideoCapture(i, cv2.CAP_V4L2) if hasattr(cv2, "CAP_V4L2") and sys.platform.startswith("linux") \
                else cv2.VideoCapture(i)
            if not cap.isOpened():
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.ancho)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.alto)
            cap.set(cv2.CAP_PROP_FPS, a.fps)
            ok, frame = cap.read()
            # la D435i expone varios /dev/video (IR, profundidad, color): se busca uno a color
            if ok and frame is not None and frame.ndim == 3 and frame.shape[2] == 3 and \
                    not np.allclose(frame[..., 0], frame[..., 1], atol=2):
                self.cap = cap
                print("OpenCV: usando /dev/video%d" % i, flush=True)
                break
            cap.release()
        if self.cap is None:
            raise RuntimeError("no se encontró un dispositivo de video a color (%s)" % indices)
        self.depth = None

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self):
        self.cap.release()


class Sintetica(object):
    nombre = "sintética"

    def __init__(self, a):
        self.a, self.t0, self.depth = a, time.time(), None

    def read(self):
        time.sleep(1.0 / self.a.fps)
        t = time.time() - self.t0
        img = np.full((self.a.alto, self.a.ancho, 3), 40, np.uint8)
        x = int((0.5 + 0.4 * np.sin(t)) * self.a.ancho)
        img[:, max(0, x - 6):x + 6] = 230                      # "tubo" que se mueve
        cy, cx = self.a.alto // 3, self.a.ancho // 2
        img[cy - 15:cy + 15, cx - 15:cx + 15] = (20, 70, 245)    # "fuego"
        img[2 * cy - 12:2 * cy + 12, cx - 20:cx + 20] = (205, 95, 25)  # "toma"
        img[0:20, 0:120] = 0
        if cv2 is not None:
            cv2.putText(img, "QDF1 %.1f" % t, (4, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return img

    def close(self):
        pass


CAMARAS = {"pyrealsense2": RealSensePyrs, "quanser": RealSenseQuanser, "opencv": OpenCVCam, "sintetica": Sintetica}
ORDEN_AUTO = ["pyrealsense2", "quanser", "opencv"]


def abrir_camara(a, preferida=None):
    """Abre la cámara de ``--camara`` (con ``auto``, la primera que abra de ORDEN_AUTO; ``preferida`` = la que
    funcionó la vez anterior, se prueba primero al reabrir)."""
    orden = list(ORDEN_AUTO) if a.camara == "auto" else [a.camara]
    if preferida in orden:
        orden.remove(preferida)
        orden.insert(0, preferida)
    errores = []
    for k in orden:
        try:
            cam = CAMARAS[k](a)
            cam.clave = k
            log("Cámara: %s" % cam.nombre)
            return cam
        except Exception as exc:
            errores.append("%s: %s" % (k, exc))
            log("  no disponible %s -> %s" % (k, exc))
    raise RuntimeError("ninguna cámara disponible: " + "; ".join(errores))


# ----------------------------------------------------------------------------- envío
ROT = {0: None, 90: "ROTATE_90_CLOCKWISE", 180: "ROTATE_180", 270: "ROTATE_90_COUNTERCLOCKWISE"}


def rotar(frame, grados):
    if not grados:
        return frame
    if cv2 is not None:
        return cv2.rotate(frame, getattr(cv2, ROT[grados]))
    return np.ascontiguousarray(np.rot90(frame, k=-(grados // 90)))  # horario, igual que OpenCV


def empaquetar(frame, calidad):
    if cv2 is not None:
        ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(calidad)])
        if ok:
            data = jpg.tobytes()
            return HEADER.pack(b"QDF1", FMT_JPEG, frame.shape[0], frame.shape[1], len(data)) + data
    small = np.ascontiguousarray(frame[::2, ::2])  # sin OpenCV: BGR crudo a la mitad para no saturar el WiFi
    return HEADER.pack(b"QDF1", FMT_RAW, small.shape[0], small.shape[1], small.nbytes) + small.tobytes()


def empaquetar_profundidad(depth):
    if cv2 is None or depth is None:
        return b""
    ok, png = cv2.imencode(".png", depth.astype(np.uint16))
    if not ok:
        return b""
    data = png.tobytes()
    return HEADER.pack(b"QDF1", FMT_DEPTH, depth.shape[0], depth.shape[1], len(data)) + data


def frame_vacio(frame):
    return frame is None or not hasattr(frame, "size") or frame.size == 0


def configurar_socket(conn, timeout_envio):
    """Cliente nuevo: sin Nagle, tiempo límite de envío, keepalive agresivo y TCP_USER_TIMEOUT (si existen)."""
    try:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except (OSError, socket.error):
        pass
    conn.settimeout(timeout_envio)
    try:
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    except (OSError, socket.error):
        pass
    for nombre, valor in (("TCP_KEEPIDLE", 2), ("TCP_KEEPINTVL", 1), ("TCP_KEEPCNT", 3),
                          ("TCP_USER_TIMEOUT", int(timeout_envio * 1000))):
        if hasattr(socket, nombre):
            try:
                conn.setsockopt(socket.IPPROTO_TCP, getattr(socket, nombre), valor)
            except (OSError, socket.error):
                pass


# ----------------------------------------------------------------------------- captura robusta
class Captura(threading.Thread):
    """Hilo que lee la cámara sin morir: la (re)abre con espera creciente y publica el último mensaje QDF1.

    ``abrir_fn(preferida)`` -> objeto con ``read()`` (frame BGR o None) y ``close()``; puede lanzar. Si ``read()``
    lanza o entrega frames vacíos más de ``vacios_s`` s, se cierra y se reabre tras ``espera`` s (se duplica hasta
    ``espera_max``; vuelve a ``espera_min`` con el primer frame bueno). Solo se codifica (JPEG) a ``fps_envio`` y si
    hay clientes (``hay_clientes_fn``)."""

    def __init__(self, abrir_fn, empaquetar_fn, fps_envio=15.0, vacios_s=3.0, espera_min=0.5, espera_max=8.0,
                 hay_clientes_fn=None):
        threading.Thread.__init__(self, name="captura")
        self.daemon = True
        self.abrir_fn, self.empaquetar_fn = abrir_fn, empaquetar_fn
        self.periodo = 1.0 / fps_envio if fps_envio > 0 else 0.0
        self.vacios_s, self.espera_min, self.espera_max = vacios_s, espera_min, espera_max
        self.hay_clientes_fn = hay_clientes_fn or (lambda: True)
        self.parar = threading.Event()
        self._cv = threading.Condition()
        self.msg, self.msg_id = None, 0
        self.cam, self.preferida, self.espera = None, None, espera_min
        # contadores (los lee el hilo principal para la línea "Estado:")
        self.aperturas, self.reaperturas, self.fallos_apertura, self.fallos_lectura = 0, 0, 0, 0
        self.frames, self.ultimo_motivo = 0, ""
        self.ocupado_desde, self.ocupado_en = None, ""    # lectura/apertura/cierre en curso (para el vigía)
        self.ultimo_frame = time.time()                   # último frame bueno (o el arranque): tope sin cámara

    def _cerrar(self, motivo):
        self.ultimo_motivo = motivo
        cam, self.cam = self.cam, None
        log("Cámara: %s; se reabre en %.1f s" % (motivo, self.espera))   # antes de close(): queda la causa
        if cam is not None:
            # close() también puede colgarse en la librería (stop_streaming tras un fallo USB): lo cubre el vigía
            self.ocupado_desde, self.ocupado_en = time.time(), "cierre"
            try:
                cam.close()
            except Exception as exc:
                log("Cámara: error al cerrar (%s)" % exc)
            self.ocupado_desde = None
        self.parar.wait(self.espera)
        self.espera = min(self.espera_max, self.espera * 2)

    def run(self):
        ultimo_envio, ultimo_ok = 0.0, time.time()
        self.ultimo_frame = ultimo_ok
        while not self.parar.is_set():
            if self.cam is None:
                self.ocupado_desde, self.ocupado_en = time.time(), "apertura"
                try:
                    cam = self.abrir_fn(self.preferida)
                except Exception as exc:
                    self.ocupado_desde = None
                    self.fallos_apertura += 1
                    self._cerrar("no abre (%s)" % exc)
                    continue
                self.ocupado_desde = None
                if self.parar.is_set():
                    try:
                        cam.close()
                    except Exception:
                        pass
                    break
                self.cam = cam
                self.preferida = getattr(cam, "clave", None)
                self.aperturas += 1
                if self.aperturas > 1:
                    self.reaperturas += 1
                    log("Cámara reabierta (reapertura %d)" % self.reaperturas)
                ultimo_ok = time.time()
            self.ocupado_desde, self.ocupado_en = time.time(), "lectura"
            try:
                frame = self.cam.read()
            except Exception as exc:
                self.ocupado_desde = None
                self.fallos_lectura += 1
                self._cerrar("falló la lectura (%s: %s)" % (type(exc).__name__, exc))
                continue
            self.ocupado_desde = None
            ahora = time.time()
            if frame_vacio(frame):
                if ahora - ultimo_ok > self.vacios_s:
                    self.fallos_lectura += 1
                    self._cerrar("sin frames hace %.1f s" % (ahora - ultimo_ok))
                else:
                    time.sleep(0.005)   # sin busy-loop si la cámara devuelve None sin esperar
                continue
            ultimo_ok = self.ultimo_frame = ahora
            self.espera = self.espera_min
            self.frames += 1
            if ahora - ultimo_envio < self.periodo or not self.hay_clientes_fn():
                continue
            ultimo_envio = ahora
            try:
                msg = self.empaquetar_fn(frame, self.cam)
            except Exception as exc:   # un frame raro (tamaño, tipo) no tira el streamer
                log("Cámara: no se pudo codificar un frame (%s)" % exc)
                continue
            with self._cv:
                self.msg, self.msg_id = msg, self.msg_id + 1
                self._cv.notify_all()
        if self.cam is not None:
            try:
                self.cam.close()
            except Exception:
                pass
            self.cam = None

    def esperar(self, msg_id, timeout):
        """(msg, id) del primer mensaje más nuevo que ``msg_id``, o (None, msg_id) si no llega en ``timeout``."""
        fin = time.time() + timeout
        with self._cv:
            while self.msg_id <= msg_id:
                resto = fin - time.time()
                if resto <= 0 or self.parar.is_set():
                    return None, msg_id
                self._cv.wait(resto)
            return self.msg, self.msg_id

    def detener(self):
        self.parar.set()
        with self._cv:
            self._cv.notify_all()


class Cliente(threading.Thread):
    """Un cliente (PC): le manda el frame más reciente. Termina si el envío excede el tiempo límite, si el socket
    falla o si el otro lado cerró (se revisa cuando no hay frames que mandar)."""

    def __init__(self, conn, addr, captura, al_enviar, al_terminar):
        threading.Thread.__init__(self, name="cliente %s" % (addr[0],))
        self.daemon = True
        self.conn, self.addr, self.captura = conn, addr, captura
        self.al_enviar, self.al_terminar = al_enviar, al_terminar
        self.parar = threading.Event()
        self.t0, self.enviados = time.time(), 0

    def _cerrado_por_el_otro(self):
        try:
            r, _, _ = select.select([self.conn], [], [], 0)
            if not r:
                return False
            return self.conn.recv(4096) == b""     # la PC no manda nada: legible = cerró (o datos que se ignoran)
        except (OSError, socket.error, ValueError):
            return True

    def run(self):
        motivo = "terminado"
        mid = self.captura.msg_id
        try:
            while not self.parar.is_set():
                msg, nuevo = self.captura.esperar(mid, 1.0)
                if msg is None:
                    if self._cerrado_por_el_otro():
                        motivo = "la PC cerró la conexión"
                        break
                    continue
                mid = nuevo
                self.conn.sendall(msg)
                self.enviados += 1
                self.al_enviar()
        except socket.timeout:
            motivo = "la PC no recibe (envío detenido más del tiempo límite)"
        except (OSError, socket.error) as exc:
            motivo = "error de socket (%s)" % exc
        finally:
            if self.parar.is_set() and motivo == "terminado":
                motivo = "cerrado por el streamer"
            self.cerrar()
            self.al_terminar(self, motivo)

    def cerrar(self):
        self.parar.set()
        try:
            self.conn.shutdown(socket.SHUT_RDWR)
        except (OSError, socket.error):
            pass
        try:
            self.conn.close()
        except (OSError, socket.error):
            pass


class Streamer(object):
    """Servidor QDF1: hilo de captura + un hilo por cliente; el hilo que llama ``correr()`` acepta clientes,
    imprime el estado y vigila que la cámara no se cuelgue."""

    def __init__(self, a, abrir_fn=None):
        self.a = a
        self.abrir_fn = abrir_fn or (lambda pref: abrir_camara(a, pref))
        self.parar = threading.Event()
        self._lock = threading.Lock()
        self.clientes = []
        self.clientes_total, self.desconexiones, self.enviados = 0, 0, 0
        self.captura = Captura(self.abrir_fn, self._empaquetar, fps_envio=a.fps_envio, vacios_s=a.vacios_s,
                               espera_max=a.espera_max, hay_clientes_fn=lambda: bool(self.clientes))
        self.srv, self.puerto = None, None
        self.codigo_salida = 0
        self.senal = None       # la pone el manejador de SIGTERM (sin locks); correr() la ve en <= 0.5 s

    def _empaquetar(self, frame, cam):
        msg = empaquetar(rotar(frame, self.a.rotar), self.a.calidad)
        if self.a.profundidad and getattr(cam, "depth", None) is not None:
            msg += empaquetar_profundidad(rotar(cam.depth, self.a.rotar))
        return msg

    def iniciar(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.a.host, self.a.puerto))
        srv.listen(4)
        srv.settimeout(0.5)
        self.srv, self.puerto = srv, srv.getsockname()[1]
        self.captura.start()
        log("Esperando a la PC en el puerto %d ..." % self.puerto)
        return self

    def _al_enviar(self):
        with self._lock:
            self.enviados += 1
            n = self.enviados
        if self.a.max_frames and n >= self.a.max_frames:
            self.parar.set()

    def _al_terminar(self, cli, motivo):
        with self._lock:
            if cli in self.clientes:
                self.clientes.remove(cli)
                self.desconexiones += 1
            activos = len(self.clientes)
        log("PC %s desconectada: %s (%d frames en %.0f s; clientes activos %d)"
            % (cli.addr[0], motivo, cli.enviados, time.time() - cli.t0, activos))

    def _aceptar(self):
        try:
            conn, addr = self.srv.accept()
        except socket.timeout:
            return
        except (OSError, socket.error) as exc:   # p. ej. sin descriptores: se reintenta, no se cae
            if not self.parar.is_set():
                log("accept falló: %s" % exc)
                self.parar.wait(0.5)
            return
        try:
            configurar_socket(conn, self.a.timeout_envio)
        except (OSError, socket.error) as exc:
            log("no se pudo configurar el socket de %s: %s" % (addr[0], exc))
        cli = Cliente(conn, addr, self.captura, self._al_enviar, self._al_terminar)
        viejos = []
        with self._lock:
            self.clientes.append(cli)
            self.clientes_total += 1
            while len(self.clientes) > max(1, self.a.max_clientes):
                viejos.append(self.clientes[0])
                self.clientes.pop(0)
                self.desconexiones += 1
            activos = len(self.clientes)
        log("PC conectada desde %s (cliente %d; activos %d)" % (addr[0], self.clientes_total, activos))
        for v in viejos:
            log("PC %s: se cierra (cliente más viejo; máximo %d a la vez)" % (v.addr[0], self.a.max_clientes))
            v.cerrar()
        cli.start()

    def estado(self):
        c = self.captura
        with self._lock:
            return {"frames_captura": c.frames, "reaperturas": c.reaperturas, "fallos_apertura": c.fallos_apertura,
                    "fallos_lectura": c.fallos_lectura, "camara_abierta": c.cam is not None,
                    "clientes_activos": len(self.clientes), "clientes_total": self.clientes_total,
                    "desconexiones": self.desconexiones, "enviados": self.enviados}

    def _vigia(self):
        """True si la cámara lleva más de ``--vigia-s`` s dentro de una lectura o apertura (hilo colgado)."""
        c = self.captura
        desde = c.ocupado_desde
        if not self.a.vigia_s or desde is None:
            return False
        dur = time.time() - desde
        if dur > self.a.vigia_s:
            log("Cámara colgada en %s hace %.0f s: se sale con código 3 para que correr_streamer.sh relance "
                "el streamer" % (c.ocupado_en, dur))
            return True
        return False

    def _sin_camara(self):
        """True si lleva más de ``--sin-camara-s`` s sin un frame bueno (no abre o se reabre sin éxito una y otra
        vez): un proceso nuevo libera los handles que la librería pudo dejar abiertos ('device busy')."""
        c = self.captura
        if not self.a.sin_camara_s or not c.is_alive():
            return False
        dur = time.time() - c.ultimo_frame
        if dur > self.a.sin_camara_s:
            log("Sin frames de la cámara hace %.0f s (fallos apertura %d, lectura %d): se sale con código 4 para que "
                "correr_streamer.sh relance el streamer limpio" % (dur, c.fallos_apertura, c.fallos_lectura))
            return True
        return False

    def correr(self):
        if self.srv is None:
            self.iniciar()
        t_estado, previo = time.time(), self.estado()
        try:
            while not self.parar.is_set() and self.senal is None:
                self._aceptar()
                ahora = time.time()
                if self._vigia():
                    self.codigo_salida = 3
                    break
                if self._sin_camara():
                    self.codigo_salida = 4
                    break
                if self.a.cada_estado and ahora - t_estado >= self.a.cada_estado:
                    e, dt = self.estado(), ahora - t_estado
                    log("Estado: captura %.1f fps, envío %.1f fps · cámara %s · reaperturas %d (fallos apertura %d, "
                        "lectura %d) · clientes activos %d, total %d, desconectados %d · frames enviados %d"
                        % ((e["frames_captura"] - previo["frames_captura"]) / dt, (e["enviados"] - previo["enviados"]) / dt,
                           "abierta" if e["camara_abierta"] else "CERRADA", e["reaperturas"], e["fallos_apertura"],
                           e["fallos_lectura"], e["clientes_activos"], e["clientes_total"], e["desconexiones"],
                           e["enviados"]))
                    t_estado, previo = ahora, e
        except KeyboardInterrupt:
            pass
        finally:
            self.detener()
        return self.codigo_salida

    def detener(self):
        self.parar.set()
        self.captura.detener()
        with self._lock:
            clientes, self.clientes = list(self.clientes), []
        for c in clientes:
            c.cerrar()
        if self.srv is not None:
            try:
                self.srv.close()
            except (OSError, socket.error):
                pass
        # con la cámara colgada (vigía, código 3) no se espera al hilo de captura: no va a terminar
        if self.captura.is_alive() and self.codigo_salida != 3:
            self.captura.join(timeout=3.0)


def argumentos(argv=None):
    p = argparse.ArgumentParser(description="Streamer QDF1 de la RealSense del QDrone 2")
    p.add_argument("--camara", choices=["auto", "pyrealsense2", "quanser", "opencv", "sintetica"], default="auto")
    p.add_argument("--serie", default="", help="número de serie de la RealSense (opcional)")
    p.add_argument("--dispositivo", type=int, default=None, help="índice /dev/videoN para --camara opencv")
    p.add_argument("--ancho", type=int, default=640)
    p.add_argument("--alto", type=int, default=480)
    p.add_argument("--fps", type=float, default=15.0)
    p.add_argument("--fps-envio", type=float, default=15.0)
    p.add_argument("--calidad", type=int, default=80, help="calidad JPEG")
    p.add_argument("--rotar", type=int, choices=[0, 90, 180, 270], default=0, help="giro horario antes de enviar")
    p.add_argument("--profundidad", action="store_true", help="enviar también profundidad (formato 3, solo pyrealsense2)")
    p.add_argument("--puerto", type=int, default=18800)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--max-clientes", type=int, default=3, help="clientes a la vez (si llegan más se cierra el más viejo)")
    p.add_argument("--timeout-envio", type=float, default=3.0, help="s que puede quedarse detenido un envío")
    p.add_argument("--vacios-s", type=float, default=3.0, help="s sin frames de la cámara antes de reabrirla")
    p.add_argument("--espera-max", type=float, default=8.0, help="tope de la espera creciente entre reaperturas")
    p.add_argument("--vigia-s", type=float, default=30.0,
                   help="s con una lectura/apertura/cierre colgado antes de salir con código 3 (0 = nunca)")
    p.add_argument("--sin-camara-s", type=float, default=60.0,
                   help="s sin frames buenos (la cámara no abre o no entrega) antes de salir con código 4 (0 = nunca)")
    p.add_argument("--cada-estado", type=float, default=10.0, help="s entre líneas 'Estado:' (0 = nunca)")
    p.add_argument("--max-frames", type=int, default=0, help="para pruebas: termina tras N frames enviados")
    return p.parse_args(argv)


def main(argv=None):
    a = argumentos(argv)
    st = Streamer(a)
    if threading.current_thread() is threading.main_thread() and hasattr(signal, "SIGTERM"):
        # SIGTERM (correr_streamer.sh --detener, kill): cierra la cámara antes de salir. SIGHUP no se toca: con
        # nohup/setsid se ignora y así debe seguir. El manejador corre en el hilo principal entre instrucciones:
        # no llama log() ni toca locks/Events (podría interrumpir a log() con _LOG_LOCK tomado -> deadlock).
        def _senal(num, _frame):
            st.senal = num
        try:
            signal.signal(signal.SIGTERM, _senal)
        except (ValueError, OSError):
            pass
    log("Streamer QDF1 (PID %d): cámara %s, %dx%d, envío %.0f fps, máximo %d clientes"
        % (os.getpid(), a.camara, a.ancho, a.alto, a.fps_envio, a.max_clientes))
    st.iniciar()
    codigo = st.correr()
    if st.senal is not None:
        log("Señal %d: streamer detenido" % st.senal)
    if codigo in (3, 4):
        # 3: el hilo de la cámara sigue colgado dentro de la librería; 4: la cámara no vuelve y la librería pudo
        # dejar handles abiertos. Salir sin esperar a nadie (el kernel los libera) y correr_streamer.sh relanza.
        sys.stdout.flush()
        os._exit(codigo)
    return codigo


if __name__ == "__main__":
    sys.exit(main())
