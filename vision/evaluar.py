# -*- coding: utf-8 -*-
"""
Evalúa el modelo guardado (por defecto vision/modelos/mejor.pt):
  1) mAP50 / mAP50-95 / P / R por clase sobre un conjunto de validación (sintético o fotos reales).
  2) Velocidad de inferencia (ms/imagen) con Detector.detect(): ultralytics en GPU y CPU, y opencv (ONNX) en CPU.
  3) Opcional: compara los dos motores (--comparar_motores) y guarda N imágenes con las predicciones dibujadas.

  python vision/evaluar.py --datos vision/datos/sintetico_smoke --comparar_motores
  python vision/evaluar.py --datos vision/datos_reales --guardar 20

Con vision/datos_reales se evalúa SÓLO la validación (images/val + el 20 % por md5 de las fotos planas) y sólo
fotos revisadas, igual que en entrenar.py. Una carpeta sin images/val se rechaza.
  python vision/evaluar.py --solo_velocidad --imagenes vision/datos/sintetico_smoke/images/val

Escribe un JSON con los resultados en vision/modelos/evaluacion_<nombre_dataset>.json.
"""
import argparse
import glob
import json
import os
import statistics
import sys
import time

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
sys.path.insert(0, RAIZ)


def es_carpeta_reales(datos):
    """Una carpeta con la estructura de vision/datos_reales (tiene revisadas.txt o crudas/)."""
    return os.path.exists(os.path.join(datos, "revisadas.txt")) or os.path.isdir(os.path.join(datos, "crudas"))


def yaml_de(datos, dir_tmp, incluir_sin_revisar=False):
    """Acepta un data.yaml, un dataset con images/val, o vision/datos_reales.

    Para fotos reales se usa EXACTAMENTE la misma validación que en entrenar.py (images/val + el 20 % por md5 de
    las fotos planas, sólo fotos revisadas). Nunca se evalúa sobre images/ completo: ultralytics lo recorre
    recursivamente y mezclaría las fotos de entrenamiento (métricas infladas).
    """
    datos = os.path.abspath(datos)
    if datos.endswith((".yaml", ".yml")):
        return datos
    os.makedirs(dir_tmp, exist_ok=True)
    nombre = os.path.basename(datos.rstrip("/\\"))
    y = os.path.join(dir_tmp, "eval_%s.yaml" % nombre)
    if es_carpeta_reales(datos):
        from vision.entrenar import imagenes_reales
        tr, va = imagenes_reales(datos, incluir_sin_revisar=incluir_sin_revisar)
        if not va:
            raise SystemExit("ERROR: no hay fotos reales de validación revisadas en %s (images/val o el 20 %% "
                             "por md5 de images/); etiqueta y revisa primero" % datos)
        lista = os.path.join(dir_tmp, "eval_%s_val.txt" % nombre)
        with open(lista, "w", encoding="utf-8") as f:
            f.writelines(p.replace("\\", "/") + "\n" for p in va)
        print("Fotos reales: evaluando %d de validación (no se usan las %d de train)" % (len(va), len(tr)))
        val = lista
    else:
        val = os.path.join(datos, "images", "val")
        if not os.path.isdir(val):
            raise SystemExit("ERROR: %s no tiene images/val; no se evalúa sobre images/ completo porque incluiría "
                             "las imágenes de entrenamiento" % datos)
    with open(y, "w", encoding="utf-8") as f:
        f.write("train: %s\nval: %s\nnames:\n  0: fuego\n  1: toma\n" % (val.replace("\\", "/"),
                                                                       val.replace("\\", "/")))
    return y


def lista_val(y):
    """Imágenes de validación de un yaml (carpeta o .txt), sin las salidas *_det de detector.py."""
    from vision.entrenar import cargar_yaml
    v = cargar_yaml(y)["val"]
    v = v if isinstance(v, list) else [v]
    base = cargar_yaml(y).get("path") or os.path.dirname(y)
    imgs = []
    for c in v:
        c = c if os.path.isabs(c) else os.path.join(base, c)
        if c.endswith(".txt"):
            with open(c, encoding="utf-8") as f:
                imgs += [ln.strip() for ln in f if ln.strip()]
        else:
            imgs += sorted(glob.glob(os.path.join(c, "*.jpg")) + glob.glob(os.path.join(c, "*.png")))
    return [p for p in imgs if not os.path.splitext(os.path.basename(p))[0].endswith("_det")]


def etiquetas_de(img, w, h):
    partes = os.path.normpath(img).split(os.sep)
    k = max(i for i, q in enumerate(partes) if q == "images")
    partes[k] = "labels"
    lbl = os.path.splitext(os.sep.join(partes))[0] + ".txt"
    gt = []
    if os.path.exists(lbl):
        for ln in open(lbl):
            p = ln.split()
            if len(p) >= 5:
                c, x, y, bw, bh = int(p[0]), *map(float, p[1:5])
                gt.append((c, (x - bw / 2) * w, (y - bh / 2) * h, (x + bw / 2) * w, (y + bh / 2) * h))
    return gt


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    i = ix * iy
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i
    return i / u if u > 0 else 0.0


def pr_en_operacion(det, imgs, umbral_iou=0.5):
    """Precisión / recall / F1 por clase con la confianza de operación del detector (lo que ve la estación)."""
    import cv2
    tp = {0: 0, 1: 0}
    fp = {0: 0, 1: 0}
    fn = {0: 0, 1: 0}
    for p in imgs:
        f = cv2.imread(p)
        h, w = f.shape[:2]
        gt = etiquetas_de(p, w, h)
        usado = [False] * len(gt)
        for c, _, x1, y1, x2, y2 in det.detect(f):
            mejor, k = 0.0, -1
            for j, g in enumerate(gt):
                if g[0] == c and not usado[j]:
                    v = iou((x1, y1, x2, y2), g[1:])
                    if v > mejor:
                        mejor, k = v, j
            if mejor >= umbral_iou:
                usado[k] = True
                tp[c] += 1
            else:
                fp[c] += 1
        for j, g in enumerate(gt):
            if not usado[j]:
                fn[g[0]] += 1
    out = {}
    for c, n in ((0, "fuego"), (1, "toma")):
        P = tp[c] / max(1, tp[c] + fp[c])
        R = tp[c] / max(1, tp[c] + fn[c])
        out[n] = {"TP": tp[c], "FP": fp[c], "FN": fn[c], "P": round(P, 4), "R": round(R, 4),
                  "F1": round(2 * P * R / max(1e-9, P + R), 4)}
    return out


def acuerdo(d1, d2, imgs):
    """Fracción de cajas de d1 que d2 reproduce (misma clase, IoU >= 0.8) y viceversa."""
    import cv2
    n1 = n2 = m1 = m2 = 0
    dif_conf = []
    for p in imgs:
        f = cv2.imread(p)
        a, b = d1.detect(f), d2.detect(f)
        n1 += len(a)
        n2 += len(b)
        for x in a:
            ok = [y for y in b if y[0] == x[0] and iou(x[2:], y[2:]) >= 0.8]
            if ok:
                m1 += 1
                dif_conf.append(abs(x[1] - max(ok, key=lambda y: iou(x[2:], y[2:]))[1]))
        m2 += sum(1 for y in b if any(x[0] == y[0] and iou(x[2:], y[2:]) >= 0.8 for x in a))
    import statistics
    return {"cajas_ultralytics": n1, "cajas_opencv": n2, "ultralytics_en_opencv": round(m1 / max(1, n1), 4),
            "opencv_en_ultralytics": round(m2 / max(1, n2), 4),
            "dif_conf_media": round(statistics.mean(dif_conf), 4) if dif_conf else None}


def velocidad(modelo, imagenes, device, n=100, backend=None):
    import cv2
    from vision.detector import Detector
    det = Detector(modelo, conf=0.25, device=device, backend=backend)
    frames = [cv2.imread(p) for p in imagenes[:min(n, len(imagenes))]]
    frames = [f for f in frames if f is not None]
    for f in frames[:5]:
        det.detect(f)
    ts = []
    for f in frames:
        t = time.perf_counter()
        det.detect(f)
        ts.append((time.perf_counter() - t) * 1000)
    return {"motor": det.backend, "dispositivo": det.device, "n": len(ts), "ms_media": round(statistics.mean(ts), 2),
            "ms_mediana": round(statistics.median(ts), 2), "ms_p90": round(sorted(ts)[int(0.9 * (len(ts) - 1))], 2),
            "fps_media": round(1000 / statistics.mean(ts), 1),
            "res_tipica": "%dx%d" % (frames[0].shape[1], frames[0].shape[0])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--modelo", default=os.path.join(AQUI, "modelos", "mejor.pt"))
    ap.add_argument("--datos", default=os.path.join(AQUI, "datos", "sintetico_smoke"))
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.001, help="conf para mAP (0.001 es el estándar)")
    ap.add_argument("--imagenes", default=None, help="carpeta de imágenes para medir velocidad")
    ap.add_argument("--n_vel", type=int, default=100)
    ap.add_argument("--solo_velocidad", action="store_true")
    ap.add_argument("--sin_cpu", action="store_true")
    ap.add_argument("--comparar_motores", action="store_true",
                    help="compara ultralytics (mejor.pt) contra opencv (mejor.onnx): acuerdo de cajas y P/R en "
                         "operación con la confianza de config")
    ap.add_argument("--incluir_sin_revisar", action="store_true")
    ap.add_argument("--guardar", type=int, default=0, help="guardar N imágenes con predicciones")
    a = ap.parse_args()

    import torch
    from ultralytics import YOLO
    res = {"modelo": os.path.abspath(a.modelo), "datos": os.path.abspath(a.datos)}
    dir_tmp = os.path.join(AQUI, "runs", "evaluacion")
    y = yaml_de(a.datos, dir_tmp, a.incluir_sin_revisar)
    imgs_val = lista_val(y)
    res["n_imagenes_val"] = len(imgs_val)
    if not a.solo_velocidad:
        m = YOLO(a.modelo)
        dev = 0 if torch.cuda.is_available() else "cpu"
        r = m.val(data=y, imgsz=a.imgsz, conf=a.conf, device=dev, split="val", plots=True,
                  project=dir_tmp, name="val", exist_ok=True, verbose=False)
        res.update({"mAP50": round(float(r.box.map50), 4), "mAP50_95": round(float(r.box.map), 4),
                    "precision": round(float(r.box.mp), 4), "recall": round(float(r.box.mr), 4),
                    "por_clase": {}})
        for i, c in enumerate(r.box.ap_class_index):
            res["por_clase"][r.names[int(c)]] = {"AP50": round(float(r.box.ap50[i]), 4),
                                                 "AP50_95": round(float(r.box.ap[i]), 4),
                                                 "P": round(float(r.box.p[i]), 4), "R": round(float(r.box.r[i]), 4)}
        print("METRICAS", json.dumps(res, ensure_ascii=False))
    if a.comparar_motores:
        from vision.detector import Detector
        du = Detector(a.modelo, backend="ultralytics")
        do = Detector(a.modelo, backend="opencv")
        res["comparacion_motores"] = {
            "conf_operacion": du.conf,
            "acuerdo_cajas_iou0.8": acuerdo(du, do, imgs_val),
            "ultralytics": pr_en_operacion(du, imgs_val),
            "opencv": pr_en_operacion(do, imgs_val)}
        print("MOTORES", json.dumps(res["comparacion_motores"], ensure_ascii=False))
    # velocidad
    if a.imagenes:
        imgs = sorted(glob.glob(os.path.join(a.imagenes, "*.jpg")) + glob.glob(os.path.join(a.imagenes, "*.png")))
        imgs = [p for p in imgs if not os.path.splitext(os.path.basename(p))[0].endswith("_det")]
    else:
        imgs = imgs_val
    res["velocidad"] = []
    if imgs:
        if torch.cuda.is_available():
            res["velocidad"].append(velocidad(a.modelo, imgs, "cuda:0", a.n_vel, "ultralytics"))
            res["velocidad"][-1]["gpu"] = torch.cuda.get_device_name(0)
        if not a.sin_cpu:
            # opencv ANTES que torch en CPU: los hilos OpenMP de torch siguen ocupando la CPU un rato después de
            # cada inferencia y en el mismo proceso hacían ver a cv2.dnn ~4x más lento de lo que es
            onnx = os.path.splitext(os.path.abspath(a.modelo))[0] + ".onnx"
            if os.path.exists(onnx):
                res["velocidad"].append(velocidad(a.modelo, imgs, "cpu", min(a.n_vel, 50), "opencv"))
            res["velocidad"].append(velocidad(a.modelo, imgs, "cpu", min(a.n_vel, 50), "ultralytics"))
        for v in res["velocidad"]:
            print("VELOCIDAD", json.dumps(v))
    if a.guardar and imgs:
        import cv2
        from vision.detector import Detector
        det = Detector(a.modelo)
        dst = os.path.join(dir_tmp, "predicciones")
        os.makedirs(dst, exist_ok=True)
        for p in imgs[:a.guardar]:
            f = cv2.imread(p)
            cv2.imwrite(os.path.join(dst, os.path.basename(p)), det.dibujar(f, det.detect(f)))
        print("predicciones dibujadas en", dst)
    nombre = os.path.basename(os.path.abspath(a.datos).rstrip("/\\")).replace(".yaml", "")
    out = os.path.join(os.path.dirname(os.path.abspath(a.modelo)), "evaluacion_%s.json" % nombre)
    if os.path.exists(out):     # conserva lo que no se recalculó (p. ej. mAP al usar --solo_velocidad)
        try:
            with open(out, encoding="utf-8") as f:
                prev = json.load(f)
            prev.update(res)
            res = prev
        except Exception:
            pass
    with open(out, "w", encoding="utf-8") as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    print("resultados en", out)


if __name__ == "__main__":
    main()
