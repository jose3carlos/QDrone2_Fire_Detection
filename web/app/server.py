"""Servidor web de QDrone2 Fire Detection (sin MATLAB).

Uso:
    python -m app.server                 # usa config.json
    python -m app.server --config otro.json
Luego abre http://127.0.0.1:8080
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import sys
import threading
import time

import numpy as np
from flask import Flask, Response, jsonify, request, send_from_directory

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None

from .cameras import CameraWorker
from .fomo import FomoDetector, camera_to_model
from .mission import MissionController
from .pose import SetpointServer, make_pose_source

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATIC = os.path.join(ROOT, "app", "static")


def load_config(path):
    with open(path, encoding="utf-8") as fh:
        cfg = json.load(fh)
    model = cfg.setdefault("model", {})
    if not os.path.isabs(model.get("path", "")):
        model["path"] = os.path.normpath(os.path.join(
            ROOT, model.get("path", "../models/fire_hazard-cpp-mcu-v7-impulse-#1.zip")))
    cam = cfg.setdefault("camera", {})
    if cam.get("folder") and not os.path.isabs(cam["folder"]):
        cam["folder"] = os.path.join(ROOT, cam["folder"])
    return cfg


def _jpeg(img, quality=80):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return buf.tobytes() if ok else b""


def _draw_box(img, box, src_w, src_h, color=(0, 0, 255), label=None):
    """Dibuja una caja en coordenadas del modelo (src_w x src_h) sobre img."""
    h, w = img.shape[:2]
    x, y, bw, bh = box
    if bw <= 0 or bh <= 0:
        return
    sx, sy = w / src_w, h / src_h
    p1 = (int(x * sx), int(y * sy))
    p2 = (int((x + bw) * sx), int((y + bh) * sy))
    cv2.rectangle(img, p1, p2, color, 2)
    if label:
        cv2.putText(img, label, (p1[0], max(12, p1[1] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def _annotated_model_image(model_img, result, scale=2):
    """Imagen 180x180 (lo que ve el modelo) ampliada con todas las celdas FOMO y la seleccionada."""
    img = model_img if model_img.ndim == 3 else cv2.cvtColor(model_img, cv2.COLOR_GRAY2BGR)
    img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    H, W = model_img.shape[:2]
    for d in result.detections:
        _draw_box(img, [d.x, d.y, d.width, d.height], W, H, (0, 200, 255))
    if result.detected:
        _draw_box(img, result.box, W, H, (0, 0, 255), f"{result.label} {result.confidence:.0f}%")
    return img


def create_app(cfg):
    detector = FomoDetector(cfg["model"]["path"], cfg["model"].get("min_confidence", 0.5))
    camera = CameraWorker(cfg.get("camera", {}))
    camera.start()
    pose = make_pose_source(cfg.get("pose", {}))
    setpoints = SetpointServer(cfg.get("setpoint_output", {}))
    mission = MissionController(camera, detector, pose, setpoints, cfg.get("mission", {}))
    mission.start()

    app = Flask(__name__, static_folder=None)
    live = {"enabled": False, "result": None, "stamp": 0.0, "fps": 0.0}

    def live_loop():
        last_id, n, t0 = 0, 0, time.time()
        while True:
            if not live["enabled"]:
                time.sleep(0.2)
                continue
            got = camera.wait_newer(last_id, 1.0)
            if got is None:
                continue
            frame, last_id, _ = got
            try:
                live["result"] = detector.infer_model_ready(camera_to_model(frame, detector.info))
                live["stamp"] = time.time()
            except Exception as exc:
                live["result"], live["error"] = None, str(exc)
            n += 1
            if time.time() - t0 > 1:
                live["fps"], n, t0 = n / (time.time() - t0), 0, time.time()
            time.sleep(0.1)  # ~ 8-10 inferencias por segundo es suficiente para la vista

    threading.Thread(target=live_loop, daemon=True, name="live-detect").start()

    @app.get("/")
    def index():
        return send_from_directory(STATIC, "index.html")

    @app.get("/video.mjpg")
    def video():
        def gen():
            last_id = 0
            while True:
                got = camera.wait_newer(last_id, 2.0)
                if got is None:
                    frame = np.zeros((240, 320, 3), np.uint8)
                    cv2.putText(frame, "Sin imagen de camara", (40, 125), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (200, 200, 200), 1, cv2.LINE_AA)
                else:
                    frame, last_id, _ = got
                    frame = frame.copy()
                    r = live["result"]
                    if live["enabled"] and r is not None and time.time() - live["stamp"] < 1.0 and r.detected:
                        _draw_box(frame, r.box, detector.info.width, detector.info.height,
                                  (0, 0, 255), f"{r.label} {r.confidence:.0f}%")
                jpg = _jpeg(frame)
                yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + \
                      str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n"
                time.sleep(1 / 25)

        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.get("/api/state")
    def state():
        snap = mission.snapshot()
        r = live["result"]
        snap.update({
            "camera": camera.status(),
            "pose": {"source": getattr(pose, "name", "?"), "connected": pose.connected,
                     "error": pose.error, "raw": list(pose.raw)[:64]},
            "setpointOutput": setpoints.status(),
            "model": detector.describe(),
            "live": {"enabled": live["enabled"], "fps": round(live["fps"], 1),
                     "result": r.to_dict() if (r is not None and live["enabled"]) else None},
        })
        return jsonify(snap)

    @app.post("/api/mission/start")
    def mission_start():
        mission.start_mission()
        return jsonify(ok=True)

    @app.post("/api/mission/home")
    def mission_home():
        mission.abort_to_home()
        return jsonify(ok=True)

    @app.post("/api/mission/reset")
    def mission_reset():
        mission.reset()
        return jsonify(ok=True)

    @app.post("/api/mission/waypoints")
    def mission_waypoints():
        body = request.get_json(force=True)
        try:
            wps = [[float(v) for v in row] for row in body.get("waypoints", [])]
            if any(len(row) != 4 for row in wps):
                raise ValueError("Cada waypoint debe tener 4 valores: X, Y, Z, Yaw")
            mission.set_waypoints(wps, body.get("batchImages"), body.get("captureSeconds"))
        except Exception as exc:
            return jsonify(ok=False, error=str(exc)), 400
        return jsonify(ok=True)

    @app.post("/api/live")
    def set_live():
        live["enabled"] = bool(request.get_json(force=True).get("enabled"))
        return jsonify(ok=True, enabled=live["enabled"])

    def _respond(result, model_img):
        jpg = _jpeg(_annotated_model_image(model_img, result), 90)
        return jsonify(ok=True, result=result.to_dict(),
                       image="data:image/jpeg;base64," + base64.b64encode(jpg).decode())

    @app.post("/api/detect")
    def detect_now():
        frame, fid, _ = camera.latest()
        if frame is None:
            return jsonify(ok=False, error="La cámara todavía no entrega imágenes"), 503
        model_img = camera_to_model(frame, detector.info)
        return _respond(detector.infer_model_ready(model_img), model_img)

    @app.post("/api/infer")
    def infer_upload():
        f = request.files.get("image")
        if f is None:
            return jsonify(ok=False, error="Falta el archivo 'image'"), 400
        data = np.frombuffer(f.read(), np.uint8)
        img = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
        if img is None:
            return jsonify(ok=False, error="No se pudo decodificar la imagen"), 400
        if img.dtype == np.uint16:
            img = np.floor(img.astype(np.float64) / 257 + 0.5).astype(np.uint8)
        if img.ndim == 3 and img.shape[2] == 4:  # como prepare_image.m: alfa sobre negro
            alpha = img[..., 3:4].astype(np.float64) / 255
            img = np.floor(img[..., :3] * alpha + 0.5).astype(np.uint8)
        from .fomo import prepare_image
        model_img = prepare_image(img, detector.info)
        return _respond(detector.infer_model_ready(model_img), model_img)

    @app.get("/api/results/<int:i>.jpg")
    def result_image(i):
        res = mission.results
        if i < 0 or i >= len(res):
            return Response(status=404)
        r = res[i]
        frame = r["_bestFrame"].copy()
        br = r["bestResult"]
        if br["detected"]:
            _draw_box(frame, br["box"], detector.info.width, detector.info.height,
                      (0, 0, 255), f"Fire {br['confidence']:.0f}%")
        return Response(_jpeg(cv2.resize(frame, (320, 240))), mimetype="image/jpeg")

    app.extensions["qdrone2"] = {"mission": mission, "camera": camera, "detector": detector}
    return app


def main(argv=None):
    parser = argparse.ArgumentParser(description="QDrone2 Fire Detection - interfaz web")
    parser.add_argument("--config", default=os.path.join(ROOT, "config.json"))
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    args = parser.parse_args(argv)
    if cv2 is None:
        sys.exit("Falta opencv-python: python -m pip install -r requirements.txt")
    cfg = load_config(args.config)
    host = args.host or cfg.get("server", {}).get("host", "127.0.0.1")
    port = args.port or int(cfg.get("server", {}).get("port", 8080))
    app = create_app(cfg)
    print(f"\n  QDrone2 Fire Detection listo en  http://{'127.0.0.1' if host == '0.0.0.0' else host}:{port}\n")
    app.run(host=host, port=port, threaded=True, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
