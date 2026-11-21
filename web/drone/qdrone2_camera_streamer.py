#!/usr/bin/env python3
"""Se ejecuta EN el QDrone2. Lee una camara con el SDK de Quanser y la sirve a la PC.

    python3 qdrone2_camera_streamer.py --camera realsense            # RGB de la RealSense
    python3 qdrone2_camera_streamer.py --camera video --id 0         # camara 2D video://localhost:0
    python3 qdrone2_camera_streamer.py --camera video --id 2 --width 820 --height 616

La PC se conecta con la fuente ``qdrone2_remote`` (``tcpip://<IP_DEL_DRON>:18800``).
Protocolo: encabezado ``<4sBxxxHHI`` (b"QDF1", formato 1=JPEG/2=BGR crudo, alto,
ancho, bytes) seguido de los bytes del frame. Transporte: ``quanser.communications.Stream``.
Solo depende de ``quanser-api`` y numpy; OpenCV es opcional (si no esta, manda BGR crudo
reducido a la mitad para no saturar el WiFi).
"""
import argparse
import struct
import sys
import time

import numpy as np

try:
    import cv2
except Exception:
    cv2 = None

from quanser.communications import Stream
from quanser.multimedia import (ImageDataType, ImageFormat, Video3D, Video3DStreamType,
                                VideoCapture)

HEADER = struct.Struct("<4sBxxxHHI")
BUFFER = 4 * 1024 * 1024


class RealSense:
    def __init__(self, a):
        self.buf = np.zeros((a.height, a.width, 3), np.uint8)
        self.v = Video3D(a.id)
        self.s = self.v.stream_open(Video3DStreamType.COLOR, 0, a.fps, a.width, a.height,
                                    ImageFormat.ROW_MAJOR_INTERLEAVED_BGR, ImageDataType.UINT8)
        self.v.start_streaming()

    def read(self):
        f = self.s.get_frame()
        if f is None:
            return None
        try:
            f.get_data(self.buf)
        finally:
            f.release()
        return self.buf

    def close(self):
        self.v.stop_streaming()
        self.s.close()
        self.v.close()


class Video2D:
    def __init__(self, a):
        self.buf = np.zeros((a.height, a.width, 3), np.uint8)
        self.c = VideoCapture("video://localhost:" + a.id, a.fps, a.width, a.height,
                              ImageFormat.ROW_MAJOR_INTERLEAVED_BGR, ImageDataType.UINT8, None, 0)
        self.c.start()

    def read(self):
        return self.buf if self.c.read(self.buf) else None

    def close(self):
        self.c.stop()
        self.c.close()


def encode(frame, quality):
    if cv2 is not None:
        ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if ok:
            data = jpg.tobytes()
            return HEADER.pack(b"QDF1", 1, frame.shape[0], frame.shape[1], len(data)), data
    small = np.ascontiguousarray(frame[::2, ::2])
    return HEADER.pack(b"QDF1", 2, small.shape[0], small.shape[1], small.nbytes), small.tobytes()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--camera", choices=["realsense", "video"], default="realsense")
    p.add_argument("--id", default="0", help="RealSense: indice/serie. Video: id de video://localhost:<id>")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--send-fps", type=float, default=15.0, help="frames por segundo enviados a la PC")
    p.add_argument("--quality", type=int, default=80)
    p.add_argument("--port", type=int, default=18800)
    a = p.parse_args()

    cam = RealSense(a) if a.camera == "realsense" else Video2D(a)
    server = Stream()
    server.listen("tcpip://localhost:%d" % a.port, False)
    print("Camara lista. Esperando a la PC en el puerto %d ..." % a.port, flush=True)
    try:
        while True:
            client = server.accept(BUFFER, 1460)
            print("PC conectada", flush=True)
            period, last = 1.0 / a.send_fps, 0.0
            try:
                while True:
                    frame = cam.read()
                    if frame is None:
                        time.sleep(0.002)
                        continue
                    if time.time() - last < period:
                        continue
                    last = time.time()
                    header, data = encode(frame, a.quality)
                    client.send_byte_array(header, len(header))
                    client.send_byte_array(data, len(data))
                    client.flush()
            except Exception as exc:
                print("PC desconectada: %s" % exc, flush=True)
                try:
                    client.close()
                except Exception:
                    pass
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        server.close()


if __name__ == "__main__":
    sys.exit(main())
