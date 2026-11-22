# -*- coding: utf-8 -*-
"""
Dibuja las etiquetas YOLO sobre las imágenes de un dataset para revisarlas a ojo.

  python vision/sintetico/ver_etiquetas.py --dataset vision/datos/sintetico_prueba --n 16 --mosaico salida.jpg

Genera un mosaico (o imágenes sueltas con --carpeta) con cajas rojas = fuego, azules = toma.
"""
import argparse
import glob
import os
import random

import cv2
import numpy as np

COLORES = {0: (0, 0, 255), 1: (255, 128, 0)}
NOMBRES = {0: "fuego", 1: "toma"}


def dibujar(ruta_img, ruta_lbl, ancho=640):
    img = cv2.imread(ruta_img)
    h, w = img.shape[:2]
    if os.path.exists(ruta_lbl):
        for linea in open(ruta_lbl):
            p = linea.split()
            if len(p) != 5:
                continue
            c, x, y, bw, bh = int(p[0]), *map(float, p[1:])
            x1, y1 = int((x - bw / 2) * w), int((y - bh / 2) * h)
            x2, y2 = int((x + bw / 2) * w), int((y + bh / 2) * h)
            cv2.rectangle(img, (x1, y1), (x2, y2), COLORES.get(c, (0, 255, 0)), 2)
            cv2.putText(img, NOMBRES.get(c, str(c)), (x1, max(12, y1 - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        COLORES.get(c, (0, 255, 0)), 1, cv2.LINE_AA)
    esc = ancho / w
    img = cv2.resize(img, (ancho, int(h * esc)))
    cv2.putText(img, os.path.basename(ruta_img), (5, img.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (255, 255, 255), 1, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--split", default="*")
    ap.add_argument("--n", type=int, default=16)
    ap.add_argument("--semilla", type=int, default=0)
    ap.add_argument("--mosaico", default=None, help="ruta del mosaico de salida (jpg)")
    ap.add_argument("--columnas", type=int, default=4)
    ap.add_argument("--solo_con_etiquetas", action="store_true")
    ap.add_argument("--ancho", type=int, default=480)
    ap.add_argument("--filtro", default="", help="subcadena que debe contener el nombre")
    a = ap.parse_args()
    imgs = sorted(glob.glob(os.path.join(a.dataset, "images", a.split, "*.jpg")) +
                  glob.glob(os.path.join(a.dataset, "images", a.split, "*.png")))
    pares = []
    for r in imgs:
        lbl = r.replace(os.sep + "images" + os.sep, os.sep + "labels" + os.sep).replace("/images/", "/labels/")
        lbl = os.path.splitext(lbl)[0] + ".txt"
        if a.filtro and a.filtro not in os.path.basename(r):
            continue
        if a.solo_con_etiquetas and (not os.path.exists(lbl) or os.path.getsize(lbl) == 0):
            continue
        pares.append((r, lbl))
    random.Random(a.semilla).shuffle(pares)
    pares = pares[:a.n]
    tiles = [dibujar(r, l, a.ancho) for r, l in pares]
    if not tiles:
        print("sin imágenes")
        return
    h = max(t.shape[0] for t in tiles)
    tiles = [cv2.copyMakeBorder(t, 0, h - t.shape[0], 0, 0, cv2.BORDER_CONSTANT) for t in tiles]
    filas = []
    for i in range(0, len(tiles), a.columnas):
        fila = tiles[i:i + a.columnas]
        while len(fila) < a.columnas:
            fila.append(np.zeros_like(tiles[0]))
        filas.append(np.hstack(fila))
    mos = np.vstack(filas)
    salida = a.mosaico or os.path.join(a.dataset, "mosaico_etiquetas.jpg")
    cv2.imwrite(salida, mos, [cv2.IMWRITE_JPEG_QUALITY, 88])
    print("mosaico:", salida, mos.shape)


if __name__ == "__main__":
    main()
