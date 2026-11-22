# -*- coding: utf-8 -*-
"""
Mueve el modelo y las fotos reales entre la laptop y la PC del laboratorio. Los pesos (*.pt, *.onnx) y las fotos
NO van a git (vision/.gitignore), así que viajan en un .zip (USB, OneDrive, Drive, correo...).

  python vision/paquete.py exportar                         -> vision/runs/paquete_vision_<fecha>.zip
  python vision/paquete.py exportar --salida E:\\            (directo a la USB)
  python vision/paquete.py exportar --sin_fotos             (sólo el modelo, ~17 MB)
  python vision/paquete.py exportar --con_crudas            (incluye también datos_reales/crudas/)
  python vision/paquete.py importar E:\\paquete_vision_20261123_1030.zip

Qué lleva: modelos/mejor.pt, mejor.onnx (+ .json), métricas, y datos_reales/images + labels + revisadas.txt.
Al importar:
  - un modelo distinto al local se respalda antes como <nombre>_local_<fecha>.<ext>;
  - las fotos y etiquetas se copian; si una etiqueta local es distinta, se queda la MÁS RECIENTE;
  - revisadas.txt se une (unión de ambos lados).
"""
import argparse
import datetime
import glob
import os
import shutil
import sys
import time
import zipfile

AQUI = os.path.dirname(os.path.abspath(__file__))
SELLO = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def archivos(con_fotos, con_crudas):
    pat = ["modelos/*.pt", "modelos/*.onnx", "modelos/*.onnx.json", "modelos/metricas*.json",
           "modelos/evaluacion_*.json", "modelos/results_entrenamiento*.csv"]
    if con_fotos:
        pat += ["datos_reales/images/**/*", "datos_reales/labels/**/*", "datos_reales/revisadas.txt"]
    if con_crudas:
        pat += ["datos_reales/crudas/**/*"]
    out = []
    for p in pat:
        out += [f for f in glob.glob(os.path.join(AQUI, p), recursive=True) if os.path.isfile(f)]
    return sorted(set(out))


def exportar(a):
    fs = archivos(not a.sin_fotos, a.con_crudas)
    if not fs:
        print("no hay nada que exportar")
        return 1
    os.makedirs(a.salida, exist_ok=True)
    z = os.path.join(a.salida, "paquete_vision_%s%s.zip" % (SELLO, "_solo_modelo" if a.sin_fotos else ""))
    with zipfile.ZipFile(z, "w", zipfile.ZIP_DEFLATED) as zf:
        for f in fs:
            zf.write(f, os.path.relpath(f, AQUI).replace("\\", "/"))
    n_fotos = sum(1 for f in fs if "/images/" in f.replace("\\", "/"))
    print("paquete %s: %d archivos (%d fotos), %.1f MB" % (z, len(fs), n_fotos, os.path.getsize(z) / 1e6))
    return 0


def iguales(zf, info, ruta):
    if not os.path.exists(ruta) or os.path.getsize(ruta) != info.file_size:
        return False
    with open(ruta, "rb") as f:
        return f.read() == zf.read(info)


def importar(a):
    if not os.path.exists(a.zip):
        print("no existe", a.zip)
        return 1
    nuevos = iguales_n = resp = conservados = 0
    with zipfile.ZipFile(a.zip) as zf:
        for info in zf.infolist():
            rel = info.filename
            if info.is_dir() or rel.startswith("/") or ".." in rel.split("/"):
                continue
            dst = os.path.join(AQUI, *rel.split("/"))
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            if rel == "datos_reales/revisadas.txt":
                loc = set(open(dst, encoding="utf-8").read().split()) if os.path.exists(dst) else set()
                loc |= set(zf.read(info).decode("utf-8").split())
                with open(dst, "w", encoding="utf-8") as f:
                    f.write("\n".join(sorted(loc)))
                continue
            if iguales(zf, info, dst):
                iguales_n += 1
                continue
            t_zip = time.mktime(info.date_time + (0, 0, -1))
            if os.path.exists(dst):
                if rel.startswith("modelos/"):
                    base, ext = os.path.splitext(dst)
                    if dst.endswith(".onnx.json"):
                        base, ext = dst[:-len(".onnx.json")], ".onnx.json"
                    shutil.copy2(dst, "%s_local_%s%s" % (base, SELLO, ext))
                    resp += 1
                elif os.path.getmtime(dst) > t_zip + 2:
                    conservados += 1          # la copia local es más nueva (p. ej. etiqueta corregida aquí)
                    continue
            with open(dst, "wb") as f:
                f.write(zf.read(info))
            os.utime(dst, (t_zip, t_zip))
            nuevos += 1
    print("importado %s: %d copiados/actualizados, %d ya iguales, %d locales más nuevos conservados, "
          "%d modelos locales respaldados" % (a.zip, nuevos, iguales_n, conservados, resp))
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("exportar")
    e.add_argument("--salida", default=os.path.join(AQUI, "runs"))
    e.add_argument("--sin_fotos", action="store_true")
    e.add_argument("--con_crudas", action="store_true")
    i = sub.add_parser("importar")
    i.add_argument("zip")
    a = ap.parse_args()
    return exportar(a) if a.cmd == "exportar" else importar(a)


if __name__ == "__main__":
    sys.exit(main())
