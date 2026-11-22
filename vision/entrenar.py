# -*- coding: utf-8 -*-
"""
Entrena YOLO11n (clases 0 fuego, 1 toma) UNA SOLA VEZ y guarda vision/modelos/mejor.pt + métricas.

  python vision/entrenar.py --datos vision/datos/sintetico_smoke --epocas 30
  python vision/entrenar.py --datos D:/datasets/sintetico_10k vision/datos/sintetico_smoke --epocas 80 --forzar

- Si vision/modelos/mejor.pt ya existe NO reentrena (sale con un aviso). Usa --forzar para reentrenar;
  el modelo anterior se respalda como vision/modelos/mejor_<fecha>.pt.
- --datos acepta una o varias carpetas de dataset (con images/{train,val}) o archivos data.yaml.
- Las fotos reales etiquetadas Y REVISADAS (datos_reales/revisadas.txt) se mezclan automáticamente (ver README);
  las pre-etiquetas de `etiquetar.py auto` que nadie revisó se dejan fuera (--incluir_sin_revisar para forzar):
    * con partición propia: datos_reales/images/{train,val} + datos_reales/labels/{train,val}
    * o planas: datos_reales/images/*.jpg + datos_reales/labels/*.txt (se parten 80/20 de forma fija por nombre)
  --peso_reales K repite las fotos reales K veces en train (son pocas frente al sintético).
- --solo_reales entrena/afina sólo con las fotos reales (p. ej. partiendo de --base vision/modelos/mejor.pt).
- Las métricas van junto al modelo con su nombre: mejor.pt -> metricas.json; otro.pt -> metricas_otro.json.
- Al terminar exporta también <modelo>.onnx (para la estación web sin torch).
"""
import argparse
import datetime
import glob
import hashlib
import json
import os
import shutil
import sys
import time

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
DIR_MODELOS = os.path.join(AQUI, "modelos")
DIR_RUNS = os.path.join(AQUI, "runs")
DIR_REALES = os.path.join(AQUI, "datos_reales")
EXT = (".jpg", ".jpeg", ".png", ".bmp")


def cargar_yaml(ruta):
    """Lee un YAML con PyYAML si está; si no, con el lector que trae ultralytics."""
    try:
        import yaml
        with open(ruta, encoding="utf-8") as f:
            return yaml.safe_load(f)
    except ImportError:
        from ultralytics.utils import YAML
        return YAML.load(ruta)


def ruta_abs(p):
    return p if os.path.isabs(p) else os.path.normpath(os.path.join(os.getcwd(), p))


def partes_de_dataset(p):
    """Devuelve (lista_train, lista_val) de rutas (carpetas de imágenes) de un dataset o data.yaml."""
    p = ruta_abs(p)
    if p.endswith((".yaml", ".yml")):
        d = cargar_yaml(p)
        base = d.get("path") or os.path.dirname(p)
        if not os.path.isabs(base):
            base = os.path.join(os.path.dirname(p), base)

        def lst(x):
            x = x if isinstance(x, list) else [x]
            return [v if os.path.isabs(v) else os.path.join(base, v) for v in x]
        return lst(d["train"]), lst(d["val"])
    tr, va = os.path.join(p, "images", "train"), os.path.join(p, "images", "val")
    if not os.path.isdir(tr):
        raise FileNotFoundError("no encuentro %s" % tr)
    return [tr], ([va] if os.path.isdir(va) and os.listdir(va) else [])


def revisadas(dir_reales=DIR_REALES):
    """Nombres (basename) de las fotos que una persona ya revisó con `etiquetar.py revisar`."""
    ruta = os.path.join(dir_reales, "revisadas.txt")
    if not os.path.exists(ruta):
        return set()
    with open(ruta, encoding="utf-8") as f:
        return set(f.read().split())


def imagenes_reales(dir_reales=DIR_REALES, incluir_sin_revisar=False, avisar=True):
    """Lista (train, val) de fotos reales etiquetadas Y REVISADAS en vision/datos_reales.

    Las pre-etiquetas de `etiquetar.py auto` son predicciones del modelo: una foto sólo entra a train/val cuando
    está en datos_reales/revisadas.txt (salvo incluir_sin_revisar=True). Sin esto se entrenaría y validaría con
    los errores del propio modelo.
    - images/train/, images/val/: partición explícita.
    - images/*.jpg (plana): partición fija 80/20 por md5 del nombre (la misma en entrenar.py y evaluar.py).
    """
    tr, va, sin_rev = [], [], []
    base = os.path.join(dir_reales, "images")
    if not os.path.isdir(base):
        return tr, va
    ok = revisadas(dir_reales)

    def ruta_lbl(img):
        partes = os.path.normpath(img).split(os.sep)
        k = max(i for i, q in enumerate(partes) if q == "images")
        partes[k] = "labels"
        return os.path.splitext(os.sep.join(partes))[0] + ".txt"

    for img in sorted(glob.glob(os.path.join(base, "**", "*"), recursive=True)):
        if not img.lower().endswith(EXT) or os.path.splitext(os.path.basename(img))[0].endswith("_det"):
            continue
        if os.path.basename(img) not in ok and not incluir_sin_revisar:
            sin_rev.append(img)
            continue
        if not os.path.exists(ruta_lbl(img)):
            continue
        rel = os.path.relpath(img, base).replace("\\", "/")
        if rel.startswith("train/"):
            tr.append(img)
        elif rel.startswith("val/"):
            va.append(img)
        else:   # plana: partición fija por hash del nombre
            h = int(hashlib.md5(os.path.basename(img).encode()).hexdigest(), 16) % 100
            (va if h < 20 else tr).append(img)
    if sin_rev and avisar:
        print("AVISO: %d fotos reales SIN REVISAR se dejan fuera (corre 'etiquetar.py revisar'; "
              "--incluir_sin_revisar para usarlas igual)" % len(sin_rev))
    return tr, va


def nombres_metricas(destino):
    """Archivos de métricas propios de cada modelo: mejor.pt -> metricas.json (compatibilidad),
    otro.pt -> metricas_otro.json; igual para results_entrenamiento*.csv."""
    d, stem = os.path.dirname(destino), os.path.splitext(os.path.basename(destino))[0]
    suf = "" if stem == "mejor" else "_" + stem
    return os.path.join(d, "metricas%s.json" % suf), os.path.join(d, "results_entrenamiento%s.csv" % suf)


def main():
    ap = argparse.ArgumentParser(description="Entrenamiento YOLO11n fuego/toma")
    ap.add_argument("--datos", nargs="*", default=[os.path.join(AQUI, "datos", "sintetico_smoke")])
    ap.add_argument("--base", default="yolo11n.pt", help="pesos iniciales (yolo11n.pt preentrenado en COCO)")
    ap.add_argument("--epocas", type=int, default=60)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", default="0", help="0 = primera GPU, cpu = CPU")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--paciencia", type=int, default=25)
    ap.add_argument("--nombre", default=None)
    ap.add_argument("--peso_reales", type=int, default=3)
    ap.add_argument("--solo_reales", action="store_true")
    ap.add_argument("--reales", default=DIR_REALES, help="carpeta de fotos reales (por defecto vision/datos_reales)")
    ap.add_argument("--incluir_sin_revisar", action="store_true",
                    help="usar también fotos reales pre-etiquetadas que nadie revisó (NO recomendado)")
    ap.add_argument("--forzar", action="store_true", help="reentrenar aunque ya exista mejor.pt")
    ap.add_argument("--salida", default=os.path.join(DIR_MODELOS, "mejor.pt"))
    a = ap.parse_args()

    destino = ruta_abs(a.salida)
    if os.path.exists(destino) and not a.forzar:
        print("Ya existe %s: no se reentrena (usa --forzar si de verdad quieres reentrenar)." % destino)
        return 0

    nombre = a.nombre or datetime.datetime.now().strftime("yolo11n_%Y%m%d_%H%M")
    dir_run = os.path.join(DIR_RUNS, nombre)
    os.makedirs(dir_run, exist_ok=True)

    train, val = [], []
    if not a.solo_reales:
        for d in a.datos:
            t, v = partes_de_dataset(d)
            train += t
            val += v
    rtr, rva = imagenes_reales(ruta_abs(a.reales), incluir_sin_revisar=a.incluir_sin_revisar)
    if rtr or rva:
        lt = os.path.join(dir_run, "reales_train.txt")
        lv = os.path.join(dir_run, "reales_val.txt")
        with open(lt, "w", encoding="utf-8") as f:
            for _ in range(1 if a.solo_reales else max(1, a.peso_reales)):
                f.writelines(p.replace("\\", "/") + "\n" for p in rtr)
        with open(lv, "w", encoding="utf-8") as f:
            f.writelines(p.replace("\\", "/") + "\n" for p in rva)
        if rtr:
            train.append(lt)
        if rva:
            val.append(lv)
    print("Fotos reales: %d train, %d val" % (len(rtr), len(rva)))
    if not train:
        print("ERROR: no hay datos de entrenamiento")
        return 1
    if not val:
        print("AVISO: no hay validación; se usa train como val")
        val = list(train)

    yaml_ruta = os.path.join(dir_run, "data_combinada.yaml")
    with open(yaml_ruta, "w", encoding="utf-8") as f:
        f.write("# generado por vision/entrenar.py\n")
        f.write("train:\n" + "".join("  - %s\n" % p.replace("\\", "/") for p in train))
        f.write("val:\n" + "".join("  - %s\n" % p.replace("\\", "/") for p in val))
        f.write("names:\n  0: fuego\n  1: toma\n")
    print(open(yaml_ruta, encoding="utf-8").read())

    import torch
    from ultralytics import YOLO
    print("torch", torch.__version__, "| CUDA:", torch.cuda.is_available(),
          torch.cuda.get_device_name(0) if torch.cuda.is_available() else "")
    base = a.base
    if base == "yolo11n.pt":
        # se descarga una vez a vision/modelos/base/ (no va a git)
        os.makedirs(os.path.join(DIR_MODELOS, "base"), exist_ok=True)
        local = os.path.join(DIR_MODELOS, "base", "yolo11n.pt")
        if os.path.exists(local):
            base = local
        else:
            cwd = os.getcwd()
            os.chdir(os.path.join(DIR_MODELOS, "base"))
            YOLO("yolo11n.pt")
            os.chdir(cwd)
            base = local if os.path.exists(local) else "yolo11n.pt"
    modelo = YOLO(base)
    t0 = time.time()
    modelo.train(
        data=yaml_ruta, epochs=a.epocas, imgsz=a.imgsz, batch=a.batch, device=a.device, workers=a.workers,
        project=DIR_RUNS, name=nombre, exist_ok=True, patience=a.paciencia, seed=0, plots=True,
        # vista cenital: los volteos vertical y horizontal son válidos; la rotación libre ya viene en los datos
        fliplr=0.5, flipud=0.5, degrees=0.0, scale=0.5, translate=0.1, mosaic=1.0, close_mosaic=5,
        hsv_h=0.015, hsv_s=0.6, hsv_v=0.4,
    )
    t_ent = time.time() - t0
    best = os.path.join(dir_run, "weights", "best.pt")
    if not os.path.exists(best):
        print("ERROR: no se generó", best)
        return 1

    # validación final con el mejor modelo
    m = YOLO(best)
    r = m.val(data=yaml_ruta, imgsz=a.imgsz, device=a.device, split="val", plots=False, verbose=False,
              project=DIR_RUNS, name=nombre + "_val", exist_ok=True)
    met = {
        "fecha": datetime.datetime.now().isoformat(timespec="seconds"),
        "run": dir_run,
        "datos_train": train, "datos_val": val,
        "fotos_reales": {"train": len(rtr), "val": len(rva)},
        "epocas": a.epocas, "imgsz": a.imgsz, "batch": a.batch, "base": a.base,
        "minutos_entrenamiento": round(t_ent / 60, 1),
        "mAP50": round(float(r.box.map50), 4), "mAP50_95": round(float(r.box.map), 4),
        "precision": round(float(r.box.mp), 4), "recall": round(float(r.box.mr), 4),
        "por_clase": {},
    }
    for i, c in enumerate(r.box.ap_class_index):
        met["por_clase"][r.names[int(c)]] = {
            "AP50": round(float(r.box.ap50[i]), 4), "AP50_95": round(float(r.box.ap[i]), 4),
            "P": round(float(r.box.p[i]), 4), "R": round(float(r.box.r[i]), 4)}
    os.makedirs(os.path.dirname(destino), exist_ok=True)
    f_met, f_csv = nombres_metricas(destino)
    if os.path.exists(destino):
        sello = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        stem = os.path.splitext(os.path.basename(destino))[0]
        resp = os.path.join(os.path.dirname(destino), "%s_%s.pt" % (stem, sello))
        shutil.move(destino, resp)
        if os.path.exists(f_met):
            shutil.copy2(f_met, os.path.splitext(f_met)[0] + "_%s.json" % sello)
        print("modelo anterior respaldado en", resp)
    shutil.copy2(best, destino)
    with open(f_met, "w", encoding="utf-8") as f:
        json.dump(met, f, indent=2, ensure_ascii=False)
    res_csv = os.path.join(dir_run, "results.csv")
    if os.path.exists(res_csv):
        shutil.copy2(res_csv, f_csv)
    print("METRICAS", json.dumps(met, ensure_ascii=False))
    print("Modelo guardado en", destino, "| métricas en", f_met)
    # .onnx para la estación web sin torch (cv2.dnn); si falla, se puede repetir con exportar_onnx.py
    try:
        sys.path.insert(0, AQUI)
        from exportar_onnx import exportar
        print("ONNX guardado en", exportar(destino, a.imgsz))
    except Exception as e:
        print("AVISO: no se exportó el ONNX (%s: %s); corre python vision/exportar_onnx.py --modelo %s"
              % (type(e).__name__, e, destino))
    return 0


if __name__ == "__main__":
    sys.exit(main())
