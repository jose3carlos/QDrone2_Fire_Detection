# -*- coding: utf-8 -*-
"""
Etiquetado rápido de fotos reales (formato YOLO) sin instalar nada extra.

1) Pre-etiquetar con el modelo sintético (copia las fotos a vision/datos_reales/images/ y escribe
   vision/datos_reales/labels/*.txt con las predicciones):

     python vision/etiquetar.py auto --entrada vision/datos_reales/crudas/lab_20261123_1030 --conf 0.25
     python vision/etiquetar.py auto --entrada vision/datos_reales/crudas/lab_val --split val
       (la validación se copia SIN pre-etiquetas: se etiqueta desde cero para no sesgar la métrica)

   Las pre-etiquetas NO cuentan como etiquetas: entrenar.py / evaluar.py sólo usan las fotos que están en
   vision/datos_reales/revisadas.txt, es decir, las que alguien pasó con `revisar` (d / ESPACIO).

2) Revisar / corregir a mano (ventana OpenCV):

     python vision/etiquetar.py revisar

   Ratón:  arrastrar con botón IZQUIERDO = caja nueva de la clase activa
           clic IZQUIERDO dentro de una caja = seleccionarla; clic DERECHO dentro = borrarla
   Teclado: 1 = clase fuego, 2 = clase toma (también cambia la caja seleccionada)
            d / ESPACIO = siguiente (guarda y marca como revisada)   a = anterior
            x = borrar todas las cajas (foto sin anomalías: también sirve, es un negativo)
            z = deshacer última caja    q / ESC = guardar y salir
   Arranca en la primera foto no revisada (progreso en vision/datos_reales/revisadas.txt).

  python vision/etiquetar.py revisar --prueba    (autoprueba sin ventana)
"""
import argparse
import glob
import os
import shutil
import sys

import cv2

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
REALES = os.path.join(AQUI, "datos_reales")
NOMBRES = {0: "fuego", 1: "toma"}
COLORES = {0: (0, 0, 255), 1: (255, 128, 0)}
EXT = (".jpg", ".jpeg", ".png")


def ruta_etiqueta(img):
    """.../images/[sub/]x.jpg -> .../labels/[sub/]x.txt (se cambia el ÚLTIMO componente 'images')."""
    partes = os.path.normpath(img).split(os.sep)
    k = max(i for i, q in enumerate(partes) if q == "images")
    partes[k] = "labels"
    return os.path.splitext(os.sep.join(partes))[0] + ".txt"


def leer(lbl, w, h):
    cajas = []
    if os.path.exists(lbl):
        for ln in open(lbl):
            p = ln.split()
            if len(p) >= 5:
                c, x, y, bw, bh = int(p[0]), *map(float, p[1:5])
                cajas.append([c, (x - bw / 2) * w, (y - bh / 2) * h, (x + bw / 2) * w, (y + bh / 2) * h])
    return cajas


def escribir(lbl, cajas, w, h):
    os.makedirs(os.path.dirname(lbl), exist_ok=True)
    with open(lbl, "w") as f:
        for c, x1, y1, x2, y2 in cajas:
            x1, x2 = sorted((max(0, min(w, x1)), max(0, min(w, x2))))
            y1, y2 = sorted((max(0, min(h, y1)), max(0, min(h, y2))))
            if x2 - x1 < 3 or y2 - y1 < 3:
                continue
            f.write("%d %.6f %.6f %.6f %.6f\n" % (c, (x1 + x2) / 2 / w, (y1 + y2) / 2 / h, (x2 - x1) / w,
                                                   (y2 - y1) / h))


def cmd_auto(a):
    # La validación NO se pre-etiqueta con el modelo: se etiqueta desde cero para que la métrica real no
    # quede sesgada hacia lo que el modelo ya predice (se puede forzar con --prelabel_val).
    prelabel = not (a.split == "val" and not a.prelabel_val)
    if a.split == "val" and a.prelabel_val:
        print("AVISO: pre-etiquetar la validación sesga la métrica real hacia este modelo")
    det = None
    if prelabel:
        sys.path.insert(0, RAIZ)
        from vision.detector import Detector
        det = Detector(a.modelo, conf=a.conf)
    dst_img = os.path.join(REALES, "images", a.split) if a.split else os.path.join(REALES, "images")
    os.makedirs(dst_img, exist_ok=True)
    rutas = sorted(p for p in glob.glob(os.path.join(a.entrada, "**", "*"), recursive=True)
                   if p.lower().endswith(EXT) and not os.path.splitext(p)[0].endswith("_det"))
    n_cajas = n_eti = 0
    for p in rutas:
        dst = os.path.join(dst_img, os.path.basename(p))
        if not os.path.exists(dst):
            shutil.copy2(p, dst)
        if det is None:
            continue
        lbl = ruta_etiqueta(dst)
        if os.path.exists(lbl) and not a.sobrescribir:
            continue
        img = cv2.imread(dst)
        h, w = img.shape[:2]
        dets = det.detect(img)
        escribir(lbl, [[c, x1, y1, x2, y2] for c, _, x1, y1, x2, y2 in dets], w, h)
        n_cajas += len(dets)
        n_eti += 1
    if det is None:
        print("copiadas %d fotos de validación a %s SIN pre-etiquetas (etiquétalas con 'revisar')"
              % (len(rutas), dst_img))
    else:
        print("pre-etiquetadas %d fotos (%d cajas) en %s" % (n_eti, n_cajas, dst_img))
    print("IMPORTANTE: entrenar.py y evaluar.py sólo usan las fotos marcadas como revisadas "
          "(datos_reales/revisadas.txt). Corre: python vision/etiquetar.py revisar")


class Editor:
    def __init__(self, imagenes, progreso):
        self.imgs = imagenes
        self.progreso = progreso
        self.revisadas = set(open(progreso).read().split()) if os.path.exists(progreso) else set()
        self.i = next((k for k, p in enumerate(imagenes) if os.path.basename(p) not in self.revisadas), 0)
        self.clase = 0
        self.sel = -1
        self.ini = None
        self.mov = None
        self.cargar()

    def cargar(self):
        self.img = cv2.imread(self.imgs[self.i])
        self.h, self.w = self.img.shape[:2]
        self.cajas = leer(ruta_etiqueta(self.imgs[self.i]), self.w, self.h)
        self.sel = -1

    def guardar(self, revisada=False):
        escribir(ruta_etiqueta(self.imgs[self.i]), self.cajas, self.w, self.h)
        if revisada:
            self.revisadas.add(os.path.basename(self.imgs[self.i]))
            with open(self.progreso, "w") as f:
                f.write("\n".join(sorted(self.revisadas)))

    def caja_en(self, x, y):
        dentro = [k for k, (c, x1, y1, x2, y2) in enumerate(self.cajas) if x1 <= x <= x2 and y1 <= y <= y2]
        if not dentro:
            return -1
        return min(dentro, key=lambda k: (self.cajas[k][3] - self.cajas[k][1]) * (self.cajas[k][4] - self.cajas[k][2]))

    def raton(self, ev, x, y, flags, _):
        if ev == cv2.EVENT_LBUTTONDOWN:
            self.ini = (x, y)
            self.mov = (x, y)
        elif ev == cv2.EVENT_MOUSEMOVE and self.ini is not None:
            self.mov = (x, y)
        elif ev == cv2.EVENT_LBUTTONUP and self.ini is not None:
            x0, y0 = self.ini
            self.ini = None
            if abs(x - x0) > 4 and abs(y - y0) > 4:
                self.cajas.append([self.clase, min(x0, x), min(y0, y), max(x0, x), max(y0, y)])
                self.sel = len(self.cajas) - 1
            else:
                self.sel = self.caja_en(x, y)
        elif ev == cv2.EVENT_RBUTTONDOWN:
            k = self.caja_en(x, y)
            if k >= 0:
                self.cajas.pop(k)
                self.sel = -1

    def tecla(self, k):
        """Devuelve False para salir."""
        if k in (ord("q"), 27):
            self.guardar()
            return False
        if k in (ord("1"), ord("2")):
            self.clase = k - ord("1")
            if 0 <= self.sel < len(self.cajas):
                self.cajas[self.sel][0] = self.clase
        elif k in (ord("d"), ord(" ")):
            self.guardar(revisada=True)
            if self.i < len(self.imgs) - 1:
                self.i += 1
                self.cargar()
        elif k == ord("a"):
            self.guardar()
            if self.i > 0:
                self.i -= 1
                self.cargar()
        elif k == ord("x"):
            self.cajas = []
        elif k == ord("z") and self.cajas:
            self.cajas.pop()
            self.sel = -1
        return True

    def dibujar(self):
        v = self.img.copy()
        for k, (c, x1, y1, x2, y2) in enumerate(self.cajas):
            cv2.rectangle(v, (int(x1), int(y1)), (int(x2), int(y2)), COLORES[c], 3 if k == self.sel else 1)
            cv2.putText(v, NOMBRES[c], (int(x1), max(12, int(y1) - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        COLORES[c], 1, cv2.LINE_AA)
        if self.ini is not None and self.mov is not None:
            cv2.rectangle(v, self.ini, self.mov, COLORES[self.clase], 1)
        estado = "%d/%d  %s  clase=%s  revisadas=%d" % (
            self.i + 1, len(self.imgs), os.path.basename(self.imgs[self.i]), NOMBRES[self.clase],
            len(self.revisadas))
        cv2.rectangle(v, (0, 0), (v.shape[1], 22), (0, 0, 0), -1)
        cv2.putText(v, estado, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return v


def cmd_revisar(a):
    imgs = sorted(p for p in glob.glob(os.path.join(a.carpeta, "**", "*"), recursive=True)
                  if p.lower().endswith(EXT) and not os.path.splitext(p)[0].endswith("_det"))
    if a.prueba:
        # copia 3 imágenes sintéticas a una carpeta temporal y simula una sesión de edición
        import tempfile
        tmp = tempfile.mkdtemp(prefix="prueba_etiquetar_")
        os.makedirs(os.path.join(tmp, "images"))
        src = sorted(glob.glob(os.path.join(AQUI, "datos", "*", "images", "val", "*.jpg")))[:3]
        for p in src:
            shutil.copy2(p, os.path.join(tmp, "images"))
            lb = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(p))), "labels", "val",
                              os.path.splitext(os.path.basename(p))[0] + ".txt")
            os.makedirs(os.path.join(tmp, "labels"), exist_ok=True)
            if os.path.exists(lb):
                shutil.copy2(lb, os.path.join(tmp, "labels"))
        imgs = sorted(glob.glob(os.path.join(tmp, "images", "*.jpg")))
        ed = Editor(imgs, os.path.join(tmp, "revisadas.txt"))
        n0 = len(ed.cajas)
        ed.raton(cv2.EVENT_LBUTTONDOWN, 10, 10, 0, None)
        ed.raton(cv2.EVENT_MOUSEMOVE, 60, 50, 0, None)
        ed.raton(cv2.EVENT_LBUTTONUP, 60, 50, 0, None)          # caja nueva (fuego)
        ed.tecla(ord("2"))                                      # la cambia a toma
        cv2.imwrite(os.path.join(tmp, "vista_editor.jpg"), ed.dibujar())
        ed.tecla(ord("d"))                                      # guarda y avanza
        txt = open(ruta_etiqueta(imgs[0])).read().strip().splitlines()
        assert len(txt) == n0 + 1 and txt[-1].startswith("1 "), txt
        ed.raton(cv2.EVENT_RBUTTONDOWN, 5, 5, 0, None)
        ed.tecla(ord("x"))
        ed.tecla(ord("d"))
        assert open(ruta_etiqueta(imgs[1])).read() == ""
        ed2 = Editor(imgs, os.path.join(tmp, "revisadas.txt"))
        assert ed2.i == 2, ed2.i                               # retoma en la primera no revisada
        print("PRUEBA OK: cajas %d -> %d en %s; vista en %s" % (n0, len(txt), os.path.basename(imgs[0]),
                                                                os.path.join(tmp, "vista_editor.jpg")))
        return
    if not imgs:
        print("no hay imágenes en", a.carpeta)
        return
    ed = Editor(imgs, os.path.join(REALES, "revisadas.txt"))
    win = "etiquetar (1 fuego, 2 toma, d siguiente, a anterior, x vaciar, z deshacer, q salir)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, ed.raton)
    while True:
        cv2.imshow(win, ed.dibujar())
        k = cv2.waitKey(20) & 0xFF
        if k != 255 and not ed.tecla(k):
            break
    cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p1 = sub.add_parser("auto")
    p1.add_argument("--entrada", required=True)
    p1.add_argument("--modelo", default=None)
    p1.add_argument("--conf", type=float, default=0.25)
    p1.add_argument("--sobrescribir", action="store_true")
    p1.add_argument("--prelabel_val", action="store_true",
                    help="pre-etiquetar también la validación (sesga la métrica; no recomendado)")
    p1.add_argument("--split", default="", choices=["", "train", "val"],
                    help="vacío = carpeta plana (partición 80/20 automática); val = sesión reservada para validar")
    p2 = sub.add_parser("revisar")
    p2.add_argument("--carpeta", default=os.path.join(REALES, "images"))
    p2.add_argument("--prueba", action="store_true")
    a = ap.parse_args()
    cmd_auto(a) if a.cmd == "auto" else cmd_revisar(a)


if __name__ == "__main__":
    main()
