#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Streamer de cámara del QDrone 2 (corre EN el Jetson del dron explorador). ARQUITECTURA §3.4.

Captura el color de la RealSense D435i (640x480, 15 fps) y lo sirve a la PC en TCP :18800 con el
protocolo QDF1 de siempre: encabezado ``<4sBxxxHHI`` = (b"QDF1", formato, alto, ancho, bytes) y
luego los bytes. Formato 1 = JPEG, 2 = BGR crudo, 3 = profundidad PNG 16 bits (opcional).

Orden de cámaras con ``--camara auto``:
  1) pyrealsense2 (librealsense)          -> la forma más directa de leer la D435i
  2) SDK de Quanser (quanser.multimedia)  -> Video3D, como el streamer original
  3) OpenCV / V4L2 (/dev/videoN)          -> último recurso (busca un dispositivo que entregue color)
  ``--camara sintetica`` genera imágenes de prueba (para probar la PC sin dron).

Ejemplos (en el dron):
    python3 qdrone2_camera_streamer.py                       # auto, 640x480 @ 15 fps, puerto 18800
    python3 qdrone2_camera_streamer.py --camara opencv --dispositivo 4
    python3 qdrone2_camera_streamer.py --rotar 180           # gira la imagen antes de mandarla
    python3 qdrone2_camera_streamer.py --profundidad         # agrega frames de profundidad (formato 3)

Montaje: la cámara mira hacia abajo. En la PC, ``camara.rotacion_montaje`` de config/estacion.json
describe la imagen TAL COMO LLEGA a la PC. Si aquí usas ``--rotar G``, réstale G a ese valor
(módulo 360) y pon G en ``camara.giro_en_dron`` (para que la estación gire también
fx/fy/cx/cy). Lo normal: no girar aquí y poner el giro en ``rotacion_montaje``.
Al arrancar con pyrealsense2 se imprimen los intrínsecos reales listos para copiar a
``camara.intrinsecos`` de estacion.json.
Compatible con Python 3.6+ (Jetson). Dependencias: numpy; OpenCV recomendado (JPEG).
"""
from __future__ import print_function

import argparse
import socket
import struct
import sys
import time

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None

HEADER = struct.Struct("<4sBxxxHHI")
FMT_JPEG, FMT_RAW, FMT_DEPTH = 1, 2, 3


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
        self.v = Video3D(a.serie or "0")
        self.s = self.v.stream_open(Video3DStreamType.COLOR, 0, float(a.fps), a.ancho, a.alto,
                                    ImageFormat.ROW_MAJOR_INTERLEAVED_BGR, ImageDataType.UINT8)
        self.v.start_streaming()
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
        try:
            self.v.stop_streaming()
            self.s.close()
        finally:
            self.v.close()


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


def abrir_camara(a):
    orden = ["pyrealsense2", "quanser", "opencv"] if a.camara == "auto" else [a.camara]
    errores = []
    for k in orden:
        try:
            cam = CAMARAS[k](a)
            print("Cámara: %s" % cam.nombre, flush=True)
            return cam
        except Exception as exc:
            errores.append("%s: %s" % (k, exc))
            print("  no disponible %s -> %s" % (k, exc), flush=True)
    raise RuntimeError("ninguna cámara disponible:\n  " + "\n  ".join(errores))


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


def main(argv=None):
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
    p.add_argument("--max-frames", type=int, default=0, help="para pruebas: termina tras N frames enviados")
    a = p.parse_args(argv)

    cam = abrir_camara(a)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((a.host, a.puerto))
    srv.listen(1)
    print("Esperando a la PC en el puerto %d ..." % a.puerto, flush=True)
    enviados = 0
    try:
        while True:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("PC conectada desde %s" % addr[0], flush=True)
            periodo, ultimo = 1.0 / a.fps_envio, 0.0
            try:
                while True:
                    frame = cam.read()
                    if frame is None:
                        continue
                    if time.time() - ultimo < periodo:
                        continue
                    ultimo = time.time()
                    msg = empaquetar(rotar(frame, a.rotar), a.calidad)
                    if a.profundidad and getattr(cam, "depth", None) is not None:
                        msg += empaquetar_profundidad(rotar(cam.depth, a.rotar))
                    conn.sendall(msg)
                    enviados += 1
                    if a.max_frames and enviados >= a.max_frames:
                        return 0
            except (OSError, socket.error) as exc:
                print("PC desconectada: %s" % exc, flush=True)
            finally:
                try:
                    conn.close()
                except Exception:
                    pass
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        srv.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
