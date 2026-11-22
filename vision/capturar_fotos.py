# -*- coding: utf-8 -*-
"""
Captura de fotos reales para el dataset (laboratorio).

Fuentes:
  --dron IP      video QDF1 del explorador (web/drone/qdrone2_camera_streamer.py en el Jetson, puerto 18800)
  --camara N     webcam/RealSense conectada a la PC (índice de OpenCV)

  python vision/capturar_fotos.py --dron 192.168.2.11 --cada 0.7
  python vision/capturar_fotos.py --camara 0

Teclas en la ventana:  ESPACIO = guardar foto   a = activar/desactivar guardado automático (cada --cada s)
                       q / ESC = salir
Las fotos se guardan en vision/datos_reales/crudas/<sesion>/ con nombre <sesion>_<n>.jpg (n sigue al mayor
número existente; repetir --sesion continúa la misma carpeta sin sobrescribir).
OJO: qdrone2_camera_streamer.py atiende UN cliente a la vez. Cierra la estación web antes de capturar con --dron.
--sin_ventana guarda automáticamente sin mostrar nada (útil por SSH); --max limita el número de fotos.
--prueba levanta un servidor QDF1 local con imágenes sintéticas y captura 3 fotos (autoprueba).
"""
import argparse
import datetime
import glob
import os
import socket
import struct
import threading
import time

import cv2
import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
CABECERA = struct.Struct("<4sBxxxHHI")    # (b"QDF1", formato, alto, ancho, bytes)


def recibir(sock, n):
    buf = bytearray()
    while len(buf) < n:
        k = sock.recv(n - len(buf))
        if not k:
            raise ConnectionError("se cerró el stream")
        buf += k
    return bytes(buf)


class FuenteQDF1:
    def __init__(self, host, puerto=18800):
        try:
            self.s = socket.create_connection((host, puerto), timeout=5)
        except OSError as e:
            raise SystemExit("No me pude conectar a %s:%d (%s). ¿Corre qdrone2_camera_streamer.py en el Jetson y "
                             "la IP es la del explorador?" % (host, puerto, e))
        # el streamer atiende UN cliente a la vez: si la estación web está conectada, aceptamos la conexión
        # TCP (backlog) pero nunca llega un frame. Se espera hasta 8 s el primero y luego 5 s entre frames.
        self.s.settimeout(8.0)
        self.primero = True
        self.aviso_res = False

    def leer(self):
        try:
            f = self._leer()
        except socket.timeout:
            if self.primero:
                raise SystemExit("Conectado, pero el streamer no manda frames. Atiende un solo cliente a la vez: "
                                 "CIERRA la estación web (o desconecta su video) y vuelve a intentar.")
            raise SystemExit("El stream se detuvo (5 s sin frames).")
        if self.primero:
            self.primero = False
            self.s.settimeout(5.0)
        if f is not None and not self.aviso_res and f.shape[1] < 640:
            self.aviso_res = True
            print("AVISO: llegan frames de %dx%d (el Jetson manda RAW reducido cuando no tiene cv2). Para el "
                  "dataset conviene 640x480 JPEG: instala python3-opencv en el Jetson." % (f.shape[1], f.shape[0]))
        return f

    def _leer(self):
        while True:
            magic, fmt, h, w, n = CABECERA.unpack(recibir(self.s, CABECERA.size))
            if magic != b"QDF1":
                raise ValueError("cabecera QDF1 inválida")
            datos = recibir(self.s, n)
            if fmt == 1:
                return cv2.imdecode(np.frombuffer(datos, np.uint8), cv2.IMREAD_COLOR)
            if fmt == 2:
                return np.frombuffer(datos, np.uint8).reshape(h, w, 3).copy()
            # fmt 3 = profundidad: se ignora

    def cerrar(self):
        self.s.close()


class FuenteCamara:
    def __init__(self, idx):
        self.c = cv2.VideoCapture(idx, cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY)
        self.c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def leer(self):
        ok, f = self.c.read()
        return f if ok else None

    def cerrar(self):
        self.c.release()


def servidor_prueba(puerto, imagenes, parar):
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", puerto))
    srv.listen(1)
    con, _ = srv.accept()
    i = 0
    try:
        while not parar.is_set():
            img = cv2.imread(imagenes[i % len(imagenes)])
            ok, jpg = cv2.imencode(".jpg", img)
            con.sendall(CABECERA.pack(b"QDF1", 1, img.shape[0], img.shape[1], len(jpg)) + jpg.tobytes())
            i += 1
            time.sleep(1 / 15)
    except OSError:
        pass
    finally:
        con.close()
        srv.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dron", default=None, help="IP del Jetson del explorador")
    ap.add_argument("--puerto", type=int, default=18800)
    ap.add_argument("--camara", type=int, default=None)
    ap.add_argument("--cada", type=float, default=1.0, help="segundos entre fotos en modo automático")
    ap.add_argument("--sesion", default=None)
    ap.add_argument("--salida", default=os.path.join(AQUI, "datos_reales", "crudas"))
    ap.add_argument("--sin_ventana", action="store_true")
    ap.add_argument("--max", type=int, default=0)
    ap.add_argument("--prueba", action="store_true")
    a = ap.parse_args()

    parar = threading.Event()
    if a.prueba:
        imgs = sorted(glob.glob(os.path.join(AQUI, "datos", "*", "images", "val", "*.jpg")))[:10]
        if not imgs:
            raise SystemExit("no hay imágenes sintéticas para la prueba")
        a.dron, a.puerto, a.sin_ventana, a.max, a.cada = "127.0.0.1", 18899, True, 3, 0.2
        a.sesion = a.sesion or "prueba"
        a.salida = os.path.join(AQUI, "runs", "prueba_captura")
        threading.Thread(target=servidor_prueba, args=(a.puerto, imgs, parar), daemon=True).start()
        time.sleep(0.5)

    sesion = a.sesion or datetime.datetime.now().strftime("lab_%Y%m%d_%H%M")
    dst = os.path.join(a.salida, sesion)
    fuente = FuenteQDF1(a.dron, a.puerto) if a.dron else FuenteCamara(a.camara or 0)
    os.makedirs(dst, exist_ok=True)
    # numeración: siguiente al MAYOR índice existente (borrar una foto no hace que se sobrescriba otra)
    idx = []
    for q in glob.glob(os.path.join(dst, "%s_*.jpg" % sesion)):
        try:
            idx.append(int(os.path.splitext(os.path.basename(q))[0].rsplit("_", 1)[1]))
        except ValueError:
            pass
    k = max(idx) + 1 if idx else 0
    previas = len(idx)
    n = 0               # fotos nuevas de esta corrida (--max cuenta sólo éstas)
    auto = a.sin_ventana
    t_ult = 0.0
    print("Guardando en", dst, "| ESPACIO = foto, a = automático, q = salir")
    try:
        while True:
            f = fuente.leer()
            if f is None:
                break
            guardar = False
            if auto and time.time() - t_ult >= a.cada:
                guardar = True
            if not a.sin_ventana:
                vista = f.copy()
                cv2.putText(vista, "%d fotos  auto=%s" % (previas + n, "SI" if auto else "no"), (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
                cv2.imshow("captura (ESPACIO foto, a auto, q salir)", vista)
                k = cv2.waitKey(1) & 0xFF
                if k in (ord("q"), 27):
                    break
                if k == ord(" "):
                    guardar = True
                if k == ord("a"):
                    auto = not auto
            if guardar:
                ruta = os.path.join(dst, "%s_%05d.jpg" % (sesion, k))
                cv2.imwrite(ruta, f, [cv2.IMWRITE_JPEG_QUALITY, 95])
                n += 1
                k += 1
                t_ult = time.time()
                print("foto", ruta, f.shape)
                if a.max and n >= a.max:
                    break
    finally:
        parar.set()
        fuente.cerrar()
        if not a.sin_ventana:
            cv2.destroyAllWindows()
    print("fotos nuevas: %d | total en la sesión: %d" % (n, previas + n))


if __name__ == "__main__":
    main()
