# -*- coding: utf-8 -*-
"""
Detector YOLO11n de anomalías para la estación web (clase 0 = fuego, 1 = toma).

Uso desde Python (así lo usa web/app/vision_service.py):

    from vision.detector import Detector
    det = Detector("vision/modelos/mejor.pt", conf=0.5)
    for cls, conf, x1, y1, x2, y2 in det.detect(frame_bgr):
        ...

Dos motores (se elige solo, ver `backend`):
  - "ultralytics": PyTorch + ultralytics con mejor.pt. Usa la GPU (CUDA) si existe. Es el del entrenamiento.
  - "opencv":      cv2.dnn con mejor.onnx (exportado de mejor.pt con `python vision/exportar_onnx.py`).
                   Sólo necesita opencv-python + numpy, así que funciona en el venv de la estación web
                   (qd2_web) SIN instalar torch. Corre en CPU. La forma de entrada (480x640 = alto x ancho desde
                   el 22-nov-2026; antes 640x640) se lee del propio .onnx o de su .onnx.json; `imgsz` sólo
                   se usa si no está en ninguno de los dos.
  backend="auto" (por defecto) intenta ultralytics y, si torch/ultralytics no están instalados, usa opencv
  con el .onnx que está junto al .pt. Se puede forzar con config/estacion.json -> vision.backend.

Uso de prueba desde consola:

    python vision/detector.py --imagen foto.jpg                 (guarda en vision/runs/detector/)
    python vision/detector.py --carpeta vision/datos_reales/images/val
    python vision/detector.py --camara 0                        (webcam; q para salir)
    python vision/detector.py --imagen foto.jpg --backend opencv

Las imágenes con cajas se guardan por defecto en vision/runs/detector/ (nunca dentro de la carpeta de datos)
y los archivos *_det.* se ignoran como entrada.
Si no se indica modelo/confianza se leen de config/estacion.json (vision.modelo, vision.confianza_min).
"""
import json
import os
import time

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
NOMBRES = {0: "fuego", 1: "toma"}
DIR_SALIDA_DET = os.path.join(AQUI, "runs", "detector")


def leer_config():
    ruta = os.path.join(RAIZ, "config", "estacion.json")
    try:
        with open(ruta, encoding="utf-8") as f:
            return json.load(f).get("vision", {})
    except Exception:
        return {}


def _hay_ultralytics():
    try:
        import torch  # noqa: F401
        import ultralytics  # noqa: F401
        return True
    except Exception:
        return False


def forma_entrada_onnx(ruta_onnx, defecto=640):
    """(alto, ancho) de la entrada fija del .onnx exportado por ultralytics.

    Se lee de los metadatos del propio .onnx (ultralytics guarda "imgsz": "[480, 640]"); si no están, del
    <nombre>.onnx.json que escribe exportar_onnx.py ("imgsz": [480, 640], o 640 en los exportados antes);
    si tampoco, `defecto` (cuadrada). No necesita el paquete onnx (la estación sólo tiene opencv + numpy).
    """
    import re

    def par(v):
        if isinstance(v, (int, float)):
            return int(v), int(v)
        if isinstance(v, (list, tuple)) and len(v) == 2:
            return int(v[0]), int(v[1])
        return None
    try:
        with open(ruta_onnx, "rb") as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - (1 << 16)))   # metadata_props va al final del archivo
            cola = f.read()
        m = re.search(rb"\x0a\x05imgsz\x12.\[(\d+), ?(\d+)\]", cola, re.S)
        if m:
            return int(m.group(1)), int(m.group(2))
    except OSError:
        pass
    try:
        with open(ruta_onnx + ".json", encoding="utf-8") as f:
            r = par(json.load(f).get("imgsz"))
        if r:
            return r
    except Exception:
        pass
    return par(defecto)


class _MotorOpenCV:
    """YOLO11 exportado a ONNX (entrada fija 1x3xHxW, salida 1x(4+nc)xN, sin NMS) con cv2.dnn, en CPU.

    La imagen se ajusta a HxW (la forma del .onnx) con letterbox igual que ultralytics: se escala sin deformar
    (r = min(H/h, W/w)) y se rellena con gris 114 centrado. Con el .onnx de 480x640 una imagen de 640x480 entra
    tal cual (r = 1, sin bandas), como en ultralytics con el .pt; con el .onnx viejo de 640x640 lleva bandas de
    80 px arriba y abajo. Una imagen vertical (480 de ancho x 640 de alto) en el .onnx de 480x640 se reduce a
    360x480 (r = 0.75) con bandas de 140 px a los lados: funciona, pero los objetos quedan 25 % más chicos.
    """

    def __init__(self, ruta_onnx, imgsz):
        import cv2
        self.cv2 = cv2
        # motor y destino por defecto de OpenCV = CPU (en OpenCV 5 setPreferableTarget sólo genera avisos)
        self.net = cv2.dnn.readNetFromONNX(ruta_onnx)
        self.h, self.w = forma_entrada_onnx(ruta_onnx, imgsz)
        # anclas de YOLO11 (pasos 8, 16 y 32): 8400 con 640x640, 6300 con 480x640
        self.n_anclas = sum((self.h // e) * (self.w // e) for e in (8, 16, 32))

    def predecir(self, img, conf, iou, max_det=300):
        cv2 = self.cv2
        h, w = img.shape[:2]
        # letterbox igual que ultralytics (relleno gris 114 centrado) a la forma HxW del .onnx
        r = min(self.h / h, self.w / w)
        nw, nh = int(round(w * r)), int(round(h * r))
        dw, dh = (self.w - nw) / 2.0, (self.h - nh) / 2.0
        im = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR) if (nw, nh) != (w, h) else img
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        if top or bottom or left or right:
            im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
        blob = cv2.dnn.blobFromImage(im, 1 / 255.0, (self.w, self.h), swapRB=True, crop=False)
        self.net.setInput(blob)
        # importar ultralytics hace cv2.setNumThreads(0) (un solo hilo): cv2.dnn se vuelve ~4x más lento.
        # Se usan todos los núcleos sólo durante la inferencia y se deja como estaba.
        hilos = cv2.getNumThreads()
        if hilos < 2:
            cv2.setNumThreads(os.cpu_count() or 4)
        try:
            out = np.asarray(self.net.forward())
        finally:
            if hilos < 2:
                cv2.setNumThreads(hilos)
        if out.shape[-1] != self.n_anclas:
            raise ValueError("el .onnx entrega %d anclas y con entrada %dx%d deberían ser %d: revisa \"imgsz\" "
                             "en el .onnx.json" % (out.shape[-1], self.h, self.w, self.n_anclas))
        p = out.reshape(out.shape[-2], out.shape[-1]).T            # (N anclas, 4 + nc)
        sc = p[:, 4:]
        cls = sc.argmax(1)
        cf = sc[np.arange(len(sc)), cls]
        m = cf >= conf
        if not np.any(m):
            return []
        p, cls, cf = p[m], cls[m], cf[m]
        x1 = (p[:, 0] - p[:, 2] / 2 - left) / r
        y1 = (p[:, 1] - p[:, 3] / 2 - top) / r
        bw, bh = p[:, 2] / r, p[:, 3] / r
        cajas = np.stack([x1, y1, bw, bh], 1)
        # NMS por clase (como ultralytics con agnostic=False): se desplazan las cajas por clase
        desp = cajas.copy()
        desp[:, :2] += cls[:, None] * 4096.0
        idx = cv2.dnn.NMSBoxes(desp.tolist(), cf.astype(float).tolist(), float(conf), float(iou), top_k=max_det)
        idx = np.array(idx).reshape(-1)
        return [(int(cls[i]), float(cf[i]), x1[i], y1[i], x1[i] + bw[i], y1[i] + bh[i]) for i in idx]


class Detector:
    """Envuelve el modelo YOLO11n (ultralytics o cv2.dnn).

    detect(frame_bgr) -> lista de (cls, conf, x1, y1, x2, y2) en píxeles del frame (enteros), ordenada por
    confianza descendente.
    """

    name = "YOLO11n (vision/detector.py)"
    classes = dict(NOMBRES)

    def __init__(self, model_path=None, conf=None, device=None, imgsz=640, iou=0.5, backend=None):
        cfg = leer_config()
        if model_path is None:
            model_path = cfg.get("modelo", "vision/modelos/mejor.pt")
        if not os.path.isabs(model_path):
            model_path = os.path.normpath(os.path.join(RAIZ, model_path))
        self.conf = float(cfg.get("confianza_min", 0.5) if conf is None else conf)
        self.min_confidence = self.conf
        self.iou = float(iou)
        self.imgsz = int(imgsz)
        backend = (backend or cfg.get("backend", "auto")).lower()
        ruta_onnx = os.path.splitext(model_path)[0] + ".onnx"
        if backend == "auto":
            if model_path.lower().endswith(".onnx"):
                backend = "opencv"
            elif _hay_ultralytics() and os.path.exists(model_path):
                backend = "ultralytics"
            elif os.path.exists(ruta_onnx):
                backend = "opencv"
            elif not _hay_ultralytics():
                raise ImportError("no hay torch/ultralytics en este Python y no existe %s; exporta el modelo con "
                                  "'python vision/exportar_onnx.py' o instala vision/requirements.txt" % ruta_onnx)
            else:
                backend = "ultralytics"
        self.backend = backend
        if backend == "opencv":
            if not os.path.exists(ruta_onnx):
                raise FileNotFoundError("no existe %s (genéralo con python vision/exportar_onnx.py)" % ruta_onnx)
            self.device = "cpu"
            self._avisar_si_desactualizado(ruta_onnx)
            self._motor = _MotorOpenCV(ruta_onnx, self.imgsz)
            self.model = None
            self.model_path = ruta_onnx
        elif backend == "ultralytics":
            if not os.path.exists(model_path):
                raise FileNotFoundError("no existe el modelo %s (entrena con vision/entrenar.py o copia mejor.pt)"
                                        % model_path)
            import torch
            from ultralytics import YOLO
            if device is None:
                device = "cuda:0" if torch.cuda.is_available() else "cpu"
            self.device = device
            self.model = YOLO(model_path)
            self.model_path = model_path
            nombres = getattr(self.model, "names", None)
            if nombres:
                self.classes = {int(k): v for k, v in dict(nombres).items()}
        else:
            raise ValueError("backend desconocido: %s (auto, ultralytics u opencv)" % backend)
        self.name = "YOLO11n %s/%s (vision/detector.py)" % (self.backend, self.device)
        # calentamiento (la primera inferencia en GPU es lenta)
        self.detect(np.zeros((480, 640, 3), dtype=np.uint8))

    @staticmethod
    def _avisar_si_desactualizado(ruta_onnx):
        """Avisa si el .onnx no se exportó del .pt que está a su lado (se reentrenó y no se exportó)."""
        try:
            import hashlib
            pt = os.path.splitext(ruta_onnx)[0] + ".pt"
            with open(ruta_onnx + ".json", encoding="utf-8") as f:
                esperado = json.load(f).get("md5_pt")
            if os.path.exists(pt) and esperado:
                with open(pt, "rb") as f:
                    if hashlib.md5(f.read()).hexdigest() != esperado:
                        print("AVISO detector: %s no corresponde a %s; corre python vision/exportar_onnx.py"
                              % (os.path.basename(ruta_onnx), os.path.basename(pt)))
        except Exception:
            pass

    def detect(self, frame_bgr):
        if frame_bgr is None:
            return []
        h, w = frame_bgr.shape[:2]
        if self.backend == "opencv":
            crudas = self._motor.predecir(frame_bgr, self.conf, self.iou)
        else:
            # FP32: en la RTX 4060 FP16 no fue más rápido con YOLO11n y 'half' está obsoleto en ultralytics 8.4
            r = self.model.predict(frame_bgr, conf=self.conf, iou=self.iou, imgsz=self.imgsz, device=self.device,
                                   verbose=False)[0]
            if r.boxes is None or len(r.boxes) == 0:
                return []
            xyxy = r.boxes.xyxy.cpu().numpy()
            cls = r.boxes.cls.cpu().numpy().astype(int)
            cf = r.boxes.conf.cpu().numpy()
            crudas = [(int(c), float(p), x1, y1, x2, y2) for (x1, y1, x2, y2), c, p in zip(xyxy, cls, cf)]
        out = [(c, round(p, 3), int(max(0, x1)), int(max(0, y1)), int(min(w - 1, x2)), int(min(h - 1, y2)))
               for c, p, x1, y1, x2, y2 in crudas]
        out.sort(key=lambda d: -d[1])
        return out

    def dibujar(self, frame_bgr, dets):
        import cv2
        img = frame_bgr.copy()
        for c, p, x1, y1, x2, y2 in dets:
            col = (0, 0, 255) if c == 0 else (255, 128, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
            cv2.putText(img, "%s %.2f" % (self.classes.get(c, c), p), (x1, max(14, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
        return img


def es_salida_det(ruta):
    """True para las imágenes con cajas que escribe este script (no son datos)."""
    return os.path.splitext(os.path.basename(ruta))[0].endswith("_det")


def _main():
    import argparse
    import glob

    import cv2
    ap = argparse.ArgumentParser(description="Prueba del detector YOLO11n")
    ap.add_argument("--modelo", default=None)
    ap.add_argument("--conf", type=float, default=None)
    ap.add_argument("--device", default=None, help="cuda:0 o cpu (por defecto automático)")
    ap.add_argument("--backend", default=None, choices=["auto", "ultralytics", "opencv"])
    ap.add_argument("--imagen", default=None)
    ap.add_argument("--carpeta", default=None)
    ap.add_argument("--camara", type=int, default=None)
    ap.add_argument("--salida", default=DIR_SALIDA_DET,
                    help="carpeta para las imágenes con cajas (por defecto vision/runs/detector)")
    ap.add_argument("--guardar_cada", type=float, default=0,
                    help="con --camara: guardar una captura con cajas cada N segundos (0 = solo con la tecla s)")
    ap.add_argument("--duracion", type=float, default=0,
                    help="con --camara: cerrar solo después de N segundos (0 = hasta q)")
    a = ap.parse_args()
    det = Detector(a.modelo, a.conf, a.device, backend=a.backend)
    print("modelo:", det.model_path, "| motor:", det.backend, "| dispositivo:", det.device, "| conf:", det.conf)
    rutas = []
    if a.imagen:
        rutas = [a.imagen]
    elif a.carpeta:
        rutas = sorted(glob.glob(os.path.join(a.carpeta, "*.jpg")) + glob.glob(os.path.join(a.carpeta, "*.png")))
        rutas = [r for r in rutas if not es_salida_det(r)]
    if rutas:
        os.makedirs(a.salida, exist_ok=True)
    for r in rutas:
        img = cv2.imread(r)
        t = time.perf_counter()
        d = det.detect(img)
        ms = (time.perf_counter() - t) * 1000
        print("%s  %.1f ms  %s" % (os.path.basename(r), ms, d))
        cv2.imwrite(os.path.join(a.salida, os.path.splitext(os.path.basename(r))[0] + "_det.jpg"),
                    det.dibujar(img, d))
    if rutas:
        print("imágenes con cajas en", os.path.abspath(a.salida))
    if a.camara is not None:
        _camara_en_vivo(cv2, det, a)


def _camara_en_vivo(cv2, det, a):
    """Webcam con cajas, FPS y resumen. s = guardar captura, q/ESC = salir.
    Todo va a <salida>/webcam_<fecha>/: detecciones.log (una línea por segundo con lo detectado y el resumen)
    y las capturas (tNNNN.N_k.jpg sin cajas + _det.jpg con cajas). Con --guardar_cada S guarda además una
    captura cada S segundos (para revisarlas después)."""
    cap = cv2.VideoCapture(a.camara, cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        print("no se pudo abrir la cámara", a.camara)
        return
    sesion = time.strftime("webcam_%Y%m%d_%H%M%S")
    carpeta = os.path.join(a.salida, sesion)
    os.makedirs(carpeta, exist_ok=True)
    log = open(os.path.join(carpeta, "detecciones.log"), "w", encoding="utf-8")
    print("capturas y registro en", os.path.abspath(carpeta), flush=True)
    t0 = time.time()
    t_guardado = t_log = t0
    n_cuadros = n_con_det = n_guardadas = 0
    max_por_clase = {0: 0, 1: 0}
    conf_max = {0: 0.0, 1: 0.0}
    fps = 0.0
    t_prev = time.perf_counter()
    try:
        while True:
            ok, img = cap.read()
            if not ok:
                print("la cámara dejó de entregar cuadros")
                break
            d = det.detect(img)
            ahora_pc = time.perf_counter()
            fps = 0.9 * fps + 0.1 * (1.0 / max(ahora_pc - t_prev, 1e-6)) if fps else 1.0 / max(ahora_pc - t_prev, 1e-6)
            t_prev = ahora_pc
            n_cuadros += 1
            n_con_det += bool(d)
            for c in (0, 1):
                dc = [x for x in d if int(x[0]) == c]
                max_por_clase[c] = max(max_por_clase[c], len(dc))
                if dc:
                    conf_max[c] = max(conf_max[c], max(float(x[1]) for x in dc))
            vis = det.dibujar(img, d)
            nf = sum(1 for x in d if int(x[0]) == 0)
            nt = sum(1 for x in d if int(x[0]) == 1)
            txt = "%.1f FPS | fuegos: %d | tomas: %d | s=guardar q=salir" % (fps, nf, nt)
            cv2.rectangle(vis, (0, 0), (vis.shape[1], 24), (0, 0, 0), -1)
            cv2.putText(vis, txt, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow("detector (q = salir)", vis)
            ahora = time.time()
            if ahora - t_log >= 1.0:
                t_log = ahora
                desc = ", ".join("%s %.2f" % (NOMBRES.get(int(c), c), cf) for c, cf, *_ in d) or "nada"
                log.write("%7.1f s  %4.1f FPS  %s\n" % (ahora - t0, fps, desc))
                log.flush()
            guardar = a.guardar_cada and ahora - t_guardado >= a.guardar_cada
            k = cv2.waitKey(1) & 0xFF
            if k == ord("s"):
                guardar = True
            if guardar:
                t_guardado = ahora
                n_guardadas += 1   # el contador evita sobrescribir si se pulsa s dos veces en < 0.1 s
                base = os.path.join(carpeta, "t%06.1f_%d" % (ahora - t0, n_guardadas))
                cv2.imwrite(base + "_det.jpg", vis)
                cv2.imwrite(base + ".jpg", img)   # sin cajas: sirve después para etiquetar
            if k in (ord("q"), 27) or cv2.getWindowProperty("detector (q = salir)", cv2.WND_PROP_VISIBLE) < 1:
                break
            if a.duracion and ahora - t0 >= a.duracion:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        dur = time.time() - t0
        resumen = ("RESUMEN: %d cuadros en %.1f s, %.1f FPS promedio (%.1f al final), %d cuadros con detección; "
                   "máx. simultáneos fuego=%d toma=%d; conf. máx fuego=%.2f toma=%.2f; %d capturas"
                   % (n_cuadros, dur, n_cuadros / max(dur, 1e-6), fps, n_con_det, max_por_clase[0],
                      max_por_clase[1], conf_max[0], conf_max[1], n_guardadas))
        log.write(resumen + "\n")
        log.close()
        print(resumen, flush=True)


if __name__ == "__main__":
    _main()
