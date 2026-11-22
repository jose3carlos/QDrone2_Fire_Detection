# -*- coding: utf-8 -*-
"""
Exporta vision/modelos/mejor.pt a vision/modelos/mejor.onnx (entrada fija 640x640, opset 12).

Con el .onnx la estación web detecta fuegos Y tomas sólo con opencv-python (cv2.dnn, CPU), sin torch.
No reentrena nada: sólo convierte los pesos. entrenar.py lo llama solo al terminar.

  python vision/exportar_onnx.py                       (mejor.pt -> mejor.onnx)
  python vision/exportar_onnx.py --modelo vision/modelos/otro.pt

Junto al .onnx se escribe <nombre>.onnx.json con el md5 del .pt de origen; detector.py avisa si el .onnx
quedó desactualizado respecto al .pt.
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


def exportar(ruta_pt, imgsz=640):
    from ultralytics import YOLO
    ruta_pt = os.path.abspath(ruta_pt)
    destino = os.path.splitext(ruta_pt)[0] + ".onnx"
    # se exporta desde una copia temporal para no dejar basura junto al modelo
    tmp = tempfile.mkdtemp(prefix="exp_onnx_")
    try:
        copia = os.path.join(tmp, "modelo.pt")
        shutil.copy2(ruta_pt, copia)
        res = YOLO(copia).export(format="onnx", imgsz=imgsz, opset=12, simplify=True, dynamic=False,
                                 verbose=False)
        shutil.copy2(str(res), destino)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    with open(destino + ".json", "w", encoding="utf-8") as f:
        json.dump({"origen": os.path.basename(ruta_pt), "md5_pt": md5(ruta_pt), "imgsz": imgsz, "opset": 12}, f,
                  indent=2)
    return destino


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--modelo", default=os.path.join(AQUI, "modelos", "mejor.pt"))
    ap.add_argument("--imgsz", type=int, default=640)
    a = ap.parse_args()
    if not os.path.exists(a.modelo):
        print("no existe", a.modelo)
        return 1
    d = exportar(a.modelo, a.imgsz)
    print("ONNX guardado en", d, "(%.1f MB)" % (os.path.getsize(d) / 1e6))
    return 0


if __name__ == "__main__":
    sys.exit(main())
