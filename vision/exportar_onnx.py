# -*- coding: utf-8 -*-
"""
Exporta vision/modelos/mejor.pt a vision/modelos/mejor.onnx (entrada fija 480x640 = alto x ancho, opset 12,
sin NMS).

Con el .onnx la estación web detecta fuegos Y tomas sólo con opencv-python (cv2.dnn, CPU), sin torch.
No reentrena nada: sólo convierte los pesos. entrenar.py lo llama solo al terminar.

  python vision/exportar_onnx.py                       (mejor.pt -> mejor.onnx, entrada 480x640)
  python vision/exportar_onnx.py --modelo vision/modelos/otro.pt
  python vision/exportar_onnx.py --imgsz 640           (entrada cuadrada 640x640, la de antes)
  python vision/exportar_onnx.py --imgsz 480 640 --salida vision/modelos/candidato.onnx

480x640 es la forma de las imágenes de la cámara (640x480): cv2.dnn las procesa sin bandas de relleno, igual que
ultralytics con el .pt (ver vision/README.md §3). Alto y ancho deben ser múltiplos de 32.
Junto al .onnx se escribe <nombre>.onnx.json con el md5 del .pt de origen y "imgsz": [alto, ancho]; detector.py
lee de ahí la forma de entrada y avisa si el .onnx quedó desactualizado respecto al .pt.
Requiere (sólo en el venv de visión): pip install onnx onnxslim
"""
import argparse
import hashlib
import json
import os
import shutil
import sys
import tempfile

AQUI = os.path.dirname(os.path.abspath(__file__))


def md5(ruta):
    h = hashlib.md5()
    with open(ruta, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


IMGSZ = (480, 640)   # alto x ancho por defecto (cámara de 640x480)


def forma(imgsz):
    """640 -> (640, 640); [480, 640] -> (480, 640). Alto y ancho múltiplos de 32."""
    if isinstance(imgsz, (int, float)):
        imgsz = (imgsz, imgsz)
    h, w = (int(x) for x in imgsz)
    if h <= 0 or w <= 0 or h % 32 or w % 32:
        raise ValueError("imgsz debe ser alto ancho múltiplos de 32 (p. ej. 480 640), no %s" % (imgsz,))
    return h, w


def exportar(ruta_pt, imgsz=IMGSZ, destino=None):
    from ultralytics import YOLO
    h, w = forma(imgsz)
    ruta_pt = os.path.abspath(ruta_pt)
    destino = os.path.abspath(destino) if destino else os.path.splitext(ruta_pt)[0] + ".onnx"
    # se exporta desde una copia temporal para no dejar basura junto al modelo
    tmp = tempfile.mkdtemp(prefix="exp_onnx_")
    try:
        copia = os.path.join(tmp, "modelo.pt")
        shutil.copy2(ruta_pt, copia)
        res = YOLO(copia).export(format="onnx", imgsz=[h, w], opset=12, simplify=True, dynamic=False,
                                 nms=False, verbose=False)
        shutil.copy2(str(res), destino)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    with open(destino + ".json", "w", encoding="utf-8") as f:
        json.dump({"origen": os.path.basename(ruta_pt), "md5_pt": md5(ruta_pt), "imgsz": [h, w], "opset": 12}, f,
                  indent=2)
    return destino


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--modelo", default=os.path.join(AQUI, "modelos", "mejor.pt"))
    ap.add_argument("--imgsz", type=int, nargs="+", default=list(IMGSZ), metavar="N",
                    help="alto ancho de la entrada (por defecto 480 640); un solo número = cuadrada")
    ap.add_argument("--salida", default=None, help="ruta del .onnx (por defecto junto al .pt, mismo nombre)")
    a = ap.parse_args()
    if not os.path.exists(a.modelo):
        print("no existe", a.modelo)
        return 1
    if len(a.imgsz) > 2:
        ap.error("--imgsz lleva 1 o 2 números (alto ancho)")
    try:
        h, w = forma(a.imgsz if len(a.imgsz) == 2 else a.imgsz[0])
    except ValueError as e:
        ap.error(str(e))
    d = exportar(a.modelo, (h, w), a.salida)
    print("ONNX guardado en", d, "(%.1f MB, entrada %dx%d = alto x ancho)" % (os.path.getsize(d) / 1e6, h, w))
    return 0


if __name__ == "__main__":
    sys.exit(main())
