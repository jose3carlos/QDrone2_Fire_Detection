"""Mundo sintético para probar sin hardware: anomalías "verdaderas", cámara virtual y
un detector falso por color.

* ``default_anomalies(vertices)``: coloca fuegos y tomas sobre la N (fracciones del recorrido).
* ``render_view(...)``: imagen BGR de lo que vería la RealSense del explorador hacia abajo
  (tapete oscuro, retícula, tubería blanca, fuego = disco rojo-naranja, toma = bloque azul).
* ``ColorDetector``: implementa la misma interfaz que ``vision/detector.py``
  (``detect(frame_bgr) -> [(cls, conf, x1, y1, x2, y2)]``) segmentando esos colores.
  Es un detector *de verdad* sobre la imagen (no hace trampa leyendo la posición real), así la
  cadena imagen → caja → rayo → suelo → cluster → objetivo se prueba completa.
"""
from __future__ import annotations

import math

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None

PIPE_D = 0.0267          # PVC 3/4" SCH40, diámetro exterior (m)
Z_PIPE = PIPE_D / 2.0
FIRE_R = 0.05            # radio de la pieza "fuego" (m)
TOMA_W, TOMA_L = 0.07, 0.10

COL_FLOOR = (38, 36, 34)
COL_GRID = (55, 53, 50)
COL_PIPE = (225, 228, 230)
COL_FIRE = (20, 70, 245)      # BGR rojo-naranja
COL_FIRE_CORE = (40, 200, 255)
COL_TOMA = (205, 95, 25)      # BGR azul


def path_points(vertices, fractions):
    """Puntos (x, y) a las fracciones dadas de la longitud total de la polilínea."""
    V = np.asarray(vertices, dtype=float)
    seg = np.linalg.norm(np.diff(V, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = cum[-1]
    out = []
    for f in fractions:
        s = min(max(f, 0.0), 1.0) * total
        i = min(int(np.searchsorted(cum, s, side="right")) - 1, len(seg) - 1)
        a = (s - cum[i]) / seg[i] if seg[i] > 0 else 0.0
        p = V[i] + a * (V[i + 1] - V[i])
        d = (V[i + 1] - V[i]) / (seg[i] if seg[i] > 0 else 1.0)
        out.append((float(p[0]), float(p[1]), float(d[0]), float(d[1])))
    return out


def default_anomalies(vertices):
    """3 anomalías: fuego, toma, fuego, separadas 8 cm a un lado del tubo."""
    pts = path_points(vertices, [0.22, 0.52, 0.83])
    out = []
    for k, (x, y, dx, dy) in enumerate(pts):
        nx, ny = -dy, dx  # normal izquierda
        side = 0.08 if k % 2 == 0 else -0.08
        out.append({"cls": k % 2, "x": round(x + side * nx, 3), "y": round(y + side * ny, 3)})
    return out


def render_view(cam, pose, vertices, anomalies, arena_half=1.5, rng=None):
    """Imagen BGR (cam.h x cam.w) vista por la cámara del explorador en ``pose`` = (x, y, z, yaw)."""
    x, y, z, yaw = pose
    img = np.empty((cam.h, cam.w, 3), np.uint8)
    img[:] = COL_FLOOR
    if rng is not None:
        noise = rng.integers(-6, 7, size=(cam.h, cam.w, 1), dtype=np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    def proj(P):
        uv, front = cam.world_to_pixel(P, x, y, z, yaw)
        return np.clip(uv, -1e5, 1e5), front

    def draw_poly(P, color, thick, closed=False):
        uv, front = proj(P)
        for i in range(len(P) - (0 if closed else 1)):
            j = (i + 1) % len(P)
            if front[i] and front[j]:
                cv2.line(img, (int(uv[i, 0]), int(uv[i, 1])), (int(uv[j, 0]), int(uv[j, 1])),
                         color, max(1, int(thick)), cv2.LINE_AA)

    depth = max(z, 0.2)
    # retícula del tapete cada 0.5 m y borde de la arena
    for g in np.arange(-arena_half, arena_half + 1e-6, 0.5):
        line_x = np.array([[g, -arena_half, 0.0], [g, arena_half, 0.0]])
        line_y = np.array([[-arena_half, g, 0.0], [arena_half, g, 0.0]])
        draw_poly(_densify(line_x, 0.1), COL_GRID, 2)
        draw_poly(_densify(line_y, 0.1), COL_GRID, 2)
    # tubería
    V = np.asarray(vertices, dtype=float)
    P = np.column_stack([V, np.full(len(V), Z_PIPE)])
    draw_poly(_densify(P, 0.02), COL_PIPE, PIPE_D * cam.fx / depth)
    # anomalías
    for a in anomalies:
        if int(a["cls"]) == 0:
            ang = np.linspace(0, 2 * math.pi, 28, endpoint=False)
            ring = np.column_stack([a["x"] + FIRE_R * np.cos(ang), a["y"] + FIRE_R * np.sin(ang),
                                    np.full(len(ang), 0.03)])
            core = np.column_stack([a["x"] + 0.45 * FIRE_R * np.cos(ang), a["y"] + 0.45 * FIRE_R * np.sin(ang),
                                    np.full(len(ang), 0.03)])
            _fill(img, proj(ring), COL_FIRE)
            _fill(img, proj(core), COL_FIRE_CORE)
        else:
            hw, hl = TOMA_W / 2, TOMA_L / 2
            rect = np.array([[a["x"] - hl, a["y"] - hw, 0.03], [a["x"] + hl, a["y"] - hw, 0.03],
                             [a["x"] + hl, a["y"] + hw, 0.03], [a["x"] - hl, a["y"] + hw, 0.03]])
            _fill(img, proj(rect), COL_TOMA)
    if rng is not None:
        img = cv2.GaussianBlur(img, (3, 3), 0)
    return img


def _densify(P, step):
    out = [P[0]]
    for a, b in zip(P[:-1], P[1:]):
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        for k in range(1, n + 1):
            out.append(a + (b - a) * k / n)
    return np.array(out)


def _fill(img, projected, color):
    uv, front = projected
    if not np.all(front):
        return
    cv2.fillPoly(img, [np.round(uv).astype(np.int32)], color, cv2.LINE_AA)


class ColorDetector:
    """Detector falso por color con la interfaz del detector real.

    ``detect(frame_bgr)`` -> lista de ``(cls, conf, x1, y1, x2, y2)`` en píxeles del frame.
    """
    name = "falso (color)"
    classes = {0: "fuego", 1: "toma"}

    def __init__(self, min_area_px=30):
        self.min_area = int(min_area_px)

    def detect(self, frame_bgr):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        fire = (((h <= 30) | (h >= 170)) & (s >= 120) & (v >= 120)).astype(np.uint8)
        toma = ((h >= 95) & (h <= 130) & (s >= 120) & (v >= 90)).astype(np.uint8)
        out = []
        for cls, mask in ((0, fire), (1, toma)):
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            n, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
            for i in range(1, n):
                x1, y1, w, hh, area = stats[i]
                if area < self.min_area:
                    continue
                fill = area / float(max(1, w * hh))
                # en el borde de la imagen la pieza está cortada: confianza menor
                edge = x1 <= 1 or y1 <= 1 or x1 + w >= frame_bgr.shape[1] - 1 or y1 + hh >= frame_bgr.shape[0] - 1
                conf = min(0.99, 0.55 + 0.4 * fill) * (0.6 if edge else 1.0)
                out.append((cls, round(float(conf), 3), int(x1), int(y1), int(x1 + w), int(y1 + hh)))
        return out
