"""Geometría de la cámara del explorador y geolocalización de detecciones (ARQUITECTURA §7).

Marcos de referencia:
* Mundo = RSF de OptiTrack: +x adelante, +y izquierda, +z arriba; yaw = 0 mirando a +x.
* Cuerpo del dron: mismos ejes que el mundo cuando yaw = 0 (roll y pitch ≈ 0 en hover).
* Cámara (convención OpenCV): +x a la derecha de la imagen, +y hacia abajo de la imagen,
  +z = eje óptico.

Montaje con ``rotacion_montaje = 0``: la cámara mira hacia abajo y la parte de **arriba de la
imagen apunta hacia adelante** del dron (+x cuerpo). Entonces:
    eje óptico  z_cam = -z_cuerpo,   y_cam = -x_cuerpo,   x_cam = -y_cuerpo.
``rotacion_montaje = θ`` (grados) gira la cámara θ alrededor de su eje óptico (visto desde la
cámara, sentido de +x_cam hacia +y_cam). Con θ = 90 la parte de arriba de la imagen apunta a
la derecha del dron (-y cuerpo); con θ = 180 apunta hacia atrás.
"""
from __future__ import annotations

import math

import numpy as np

# Rotación cuerpo <- cámara para montaje hacia abajo con rotación 0 (columnas = ejes de cámara)
R_BODY_CAM0 = np.array([[0.0, -1.0, 0.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, 0.0, -1.0]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def intrinsics_from_config(cfg, width, height):
    """(fx, fy, cx, cy, origen) para una imagen SIN girar de ``width`` x ``height``, o None.

    Prioridad: ``camara.intrinsecos["WxH"]`` (los que imprime el streamer con rs2, medidos en el
    dron) > ``vision.intrinsecos_nominales["WxH"]`` (nominales D435i). ``cfg`` = estacion.json completo.
    """
    key = f"{int(width)}x{int(height)}"
    for sec, sub, origen in (("camara", "intrinsecos", "camara.intrinsecos"),
                             ("vision", "intrinsecos_nominales", "vision.intrinsecos_nominales")):
        tab = (cfg.get(sec) or {}).get(sub) or {}
        d = tab.get(key) if isinstance(tab, dict) else None
        if isinstance(d, dict) and all(k in d for k in ("fx", "fy", "cx", "cy")):
            return float(d["fx"]), float(d["fy"]), float(d["cx"]), float(d["cy"]), f"{origen}[{key}]"
    return None


def rotate_intrinsics(w, h, fx, fy, cx, cy, giro):
    """Intrínsecos de la imagen girada ``giro`` grados en sentido horario (``cv2.rotate``, como
    ``--rotar`` del streamer). Devuelve (w', h', fx', fy', cx', cy')."""
    giro = int(giro) % 360
    if giro == 90:      # u' = h-1-v, v' = u
        return h, w, fy, fx, (h - 1) - cy, cx
    if giro == 180:     # u' = w-1-u, v' = h-1-v
        return w, h, fx, fy, (w - 1) - cx, (h - 1) - cy
    if giro == 270:     # u' = v, v' = w-1-u
        return h, w, fy, fx, cy, (w - 1) - cx
    return w, h, fx, fy, cx, cy


class CameraModel:
    """Intrínsecos (fx, fy, cx, cy; o HFOV/VFOV si no se dan) + extrínsecos del soporte."""

    def __init__(self, width=640, height=480, hfov_deg=69.4, vfov_deg=42.5, mount_rot_deg=0.0,
                 offset_m=(0.20, 0.0, -0.05), fx=None, fy=None, cx=None, cy=None, origen=None):
        self.w, self.h = int(width), int(height)
        self.fx = float(fx) if fx else (self.w / 2.0) / math.tan(math.radians(float(hfov_deg)) / 2.0)
        self.fy = float(fy) if fy else (self.h / 2.0) / math.tan(math.radians(float(vfov_deg)) / 2.0)
        self.cx = float(cx) if cx is not None else (self.w - 1) / 2.0
        self.cy = float(cy) if cy is not None else (self.h - 1) / 2.0
        self.origen = origen or ("intrínsecos dados" if fx else "HFOV/VFOV")
        self.mount_rot_deg = float(mount_rot_deg)
        self.R_bc = R_BODY_CAM0 @ rot_z(math.radians(self.mount_rot_deg))
        self.offset = np.asarray(offset_m, dtype=float).reshape(3)
        self.aviso = ""

    @property
    def hfov_deg(self):
        return 2 * math.degrees(math.atan(self.w / (2.0 * self.fx)))

    @property
    def vfov_deg(self):
        return 2 * math.degrees(math.atan(self.h / (2.0 * self.fy)))

    @classmethod
    def from_config(cls, cam_cfg, cfg=None, image_size=None):
        """Modelo para la imagen TAL COMO LLEGA a la PC.

        ``cam_cfg`` = sección ``camara``; ``cfg`` = estacion.json completo (para los intrínsecos de
        ``camara.intrinsecos`` / ``vision.intrinsecos_nominales``; si falta se usan HFOV/VFOV).
        ``image_size`` = (ancho, alto) del frame recibido (por defecto ``camara.resolucion`` girada).
        ``camara.giro_en_dron`` (0/90/180/270) = ``--rotar`` del streamer: gira también los intrínsecos.
        """
        cfg = cfg if cfg is not None else {"camara": cam_cfg}
        W, H = (int(v) for v in cam_cfg.get("resolucion", [640, 480]))
        giro = int(cam_cfg.get("giro_en_dron", 0) or 0) % 360
        rot = cam_cfg.get("rotacion_montaje", 0)
        off = cam_cfg.get("offset_camara_m", [0.20, 0.0, -0.05])
        aviso = ""
        if image_size is not None:
            iw, ih = int(image_size[0]), int(image_size[1])
            exp_w, exp_h = (H, W) if giro in (90, 270) else (W, H)
            if (iw, ih) != (exp_w, exp_h) and (iw, ih) == (exp_h, exp_w) and iw != ih:
                # llegó con ancho y alto cambiados respecto a lo configurado: el dron la está girando
                giro = 90 if giro in (0, 180) else 0
                aviso = (f"la imagen llega de {iw}x{ih} pero camara.resolucion={W}x{H} y giro_en_dron="
                         f"{cam_cfg.get('giro_en_dron', 0)}: se asume giro de {giro}°; ajusta giro_en_dron")
            # tamaño de la imagen sin girar
            uw, uh = (ih, iw) if giro in (90, 270) else (iw, ih)
        else:
            uw, uh = W, H
        intr = intrinsics_from_config(cfg, uw, uh)
        if intr is None and (uw, uh) != (W, H):
            base = intrinsics_from_config(cfg, W, H)
            if base is not None:  # mismo sensor reescalado
                sx, sy = uw / float(W), uh / float(H)
                intr = (base[0] * sx, base[1] * sy, (base[2] + 0.5) * sx - 0.5, (base[3] + 0.5) * sy - 0.5,
                        base[4] + f" escalado a {uw}x{uh}")
        if intr is None:
            fx = (uw / 2.0) / math.tan(math.radians(float(cam_cfg.get("hfov_deg", 69.4))) / 2.0)
            fy = (uh / 2.0) / math.tan(math.radians(float(cam_cfg.get("vfov_deg", 42.5))) / 2.0)
            intr = (fx, fy, (uw - 1) / 2.0, (uh - 1) / 2.0, "HFOV/VFOV (sin intrínsecos en config)")
        fx, fy, cx, cy, origen = intr
        w2, h2, fx, fy, cx, cy = rotate_intrinsics(uw, uh, fx, fy, cx, cy, giro)
        if giro:
            origen += f", girados {giro}°"
        m = cls(w2, h2, 0, 0, rot, off, fx=fx, fy=fy, cx=cx, cy=cy, origen=origen)
        m.aviso = aviso
        return m

    def scaled(self, width, height):
        """Mismo modelo para la imagen reescalada a otro tamaño (sin girar; centros de píxel conservados)."""
        sx, sy = width / float(self.w), height / float(self.h)
        return CameraModel(width, height, 0, 0, self.mount_rot_deg, self.offset, fx=self.fx * sx,
                           fy=self.fy * sy, cx=(self.cx + 0.5) * sx - 0.5, cy=(self.cy + 0.5) * sy - 0.5, origen=self.origen + " (escalado)")

    # ------------------------------------------------------------------ pose
    def pose(self, x, y, z, yaw):
        """Devuelve (R_mundo<-cámara, posición de la cámara en el mundo)."""
        Rz = rot_z(yaw)
        R_wc = Rz @ self.R_bc
        p_c = np.array([x, y, z], dtype=float) + Rz @ self.offset
        return R_wc, p_c

    # ------------------------------------------------------------------ píxel -> suelo
    def pixel_to_ground(self, u, v, x, y, z, yaw, z_plane=0.02):
        """Rayo del píxel (u, v) intersecado con el plano z = z_plane. None si no corta."""
        R_wc, p_c = self.pose(x, y, z, yaw)
        d_cam = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        d = R_wc @ d_cam
        if d[2] >= -1e-9:
            return None
        t = (z_plane - p_c[2]) / d[2]
        if t <= 0:
            return None
        g = p_c + t * d
        return float(g[0]), float(g[1])

    # ------------------------------------------------------------------ mundo -> píxel
    def world_to_pixel(self, pts, x, y, z, yaw):
        """``pts`` (N,3) en el mundo -> (N,2) píxeles y máscara de puntos delante de la cámara."""
        R_wc, p_c = self.pose(x, y, z, yaw)
        P = np.asarray(pts, dtype=float).reshape(-1, 3)
        d = (P - p_c) @ R_wc  # = (R_wc^T (P - p_c))^T
        front = d[:, 2] > 1e-6
        dz = np.where(front, d[:, 2], 1.0)
        uv = np.stack([self.fx * d[:, 0] / dz + self.cx, self.fy * d[:, 1] / dz + self.cy], axis=1)
        return uv, front

    def footprint(self, x, y, z, yaw, z_plane=0.02):
        """Polígono (4 esquinas) que ve la cámara sobre el suelo, en orden de la imagen."""
        corners = [(0, 0), (self.w - 1, 0), (self.w - 1, self.h - 1), (0, self.h - 1)]
        out = []
        for u, v in corners:
            g = self.pixel_to_ground(u, v, x, y, z, yaw, z_plane)
            if g is None:
                return []
            out.append([round(g[0], 3), round(g[1], 3)])
        return out


class Clusterer:
    """Agrupa observaciones (clase, x, y, conf) por radio; confirma con un mínimo de observaciones."""

    def __init__(self, radius=0.25, min_obs=3, max_targets=32):
        self.radius = float(radius)
        self.min_obs = int(min_obs)
        self.max_targets = int(max_targets)
        self.clusters = []  # dicts: cls, sx, sy, sconf, n, id (0 si aún no se publica)
        self.next_id = 1

    def add(self, cls, x, y, conf):
        """Agrega una observación. Devuelve el cluster afectado."""
        best, best_d = None, None
        for c in self.clusters:
            if c["cls"] != cls:
                continue
            d = math.hypot(c["sx"] / c["n"] - x, c["sy"] / c["n"] - y)
            if d <= self.radius and (best_d is None or d < best_d):
                best, best_d = c, d
        if best is None:
            best = {"cls": int(cls), "sx": 0.0, "sy": 0.0, "sconf": 0.0, "n": 0, "id": 0,
                    "manual": False, "t0": None}
            self.clusters.append(best)
        best["sx"] += x
        best["sy"] += y
        best["sconf"] += conf
        best["n"] += 1
        if best["id"] == 0 and best["n"] >= self.min_obs and self.next_id <= self.max_targets:
            best["id"] = self.next_id
            self.next_id += 1
        return best

    def add_manual(self, cls, x, y):
        """Objetivo puesto a mano desde la UI: se publica de inmediato."""
        if self.next_id > self.max_targets:
            raise ValueError("No quedan ids libres: el protocolo solo admite ids 1..32 (visited_mask). El objetivo NO se publicó.")
        c = {"cls": int(cls), "sx": float(x), "sy": float(y), "sconf": 1.0, "n": 1,
             "id": self.next_id, "manual": True, "t0": None}
        self.next_id += 1
        self.clusters.append(c)
        return c

    def targets(self):
        """Objetivos publicados: lista de dicts con id, cls, x, y, conf, n."""
        out = []
        for c in self.clusters:
            if c["id"]:
                out.append({"id": c["id"], "cls": c["cls"], "x": c["sx"] / c["n"], "y": c["sy"] / c["n"],
                            "conf": c["sconf"] / c["n"], "n": c["n"], "manual": c["manual"]})
        return sorted(out, key=lambda t: t["id"])

    def pending(self):
        return [{"cls": c["cls"], "x": c["sx"] / c["n"], "y": c["sy"] / c["n"], "n": c["n"]}
                for c in self.clusters if not c["id"]]

    def blocked(self):
        """Clusters ya confirmados (≥ min_obs) que NO recibieron id porque se acabaron los ids 1..32
        (visited_mask de Simulink solo representa 32). No se publican."""
        return [{"cls": c["cls"], "x": c["sx"] / c["n"], "y": c["sy"] / c["n"], "n": c["n"]}
                for c in self.clusters if not c["id"] and c["n"] >= self.min_obs]

    def reset(self, next_id=1):
        self.clusters, self.next_id = [], int(max(1, next_id))

    def ensure_next_id(self, floor):
        """Garantiza que los ids nuevos no choquen con los que Simulink ya tiene (``floor`` = primer id libre)."""
        if floor > self.next_id:
            self.next_id = int(floor)

    def to_dict(self):
        return {"next_id": self.next_id, "clusters": [dict(c) for c in self.clusters]}

    def load_dict(self, d):
        self.clusters = [dict(c) for c in d.get("clusters", [])]
        for c in self.clusters:          # un archivo viejo con ids > 32: esos ya no se publican
            if c.get("id") and not (1 <= int(c["id"]) <= self.max_targets):
                c["id"] = 0
        used = [c["id"] for c in self.clusters if c.get("id")]
        self.next_id = max([int(d.get("next_id", 1))] + [i + 1 for i in used])
