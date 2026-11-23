"""Geometría de la cámara del explorador y geolocalización de detecciones (ARQUITECTURA §7).

Marcos de referencia:
* Mundo = RSF de OptiTrack: +x adelante, +y izquierda, +z arriba; yaw = 0 mirando a +x.
* Cuerpo del dron (FLU): +x nariz, +y izquierda, +z arriba; coincide con el mundo con yaw = roll = pitch = 0.
* Cámara (convención OpenCV): +x a la derecha de la imagen, +y hacia abajo de la imagen,
  +z = eje óptico.

Actitud (``R_mundo_cuerpo``, la ÚNICA función que fija la convención): la misma de la
"Corrected Pose Data [x y z roll pitch yaw isTracking]" de OptiTrack en tierra (ARQUITECTURA §3.3,
y(61:63)): Tait-Bryan intrínsecos ZYX en radianes, rotaciones activas y regla de la mano derecha,
    R_mundo<-cuerpo = Rz(yaw) · Ry(pitch) · Rx(roll).
Con roll > 0 sube el lado izquierdo (baja el derecho); con pitch > 0 baja la nariz (el empuje se
inclina hacia +x, el dron acelera hacia adelante). Es la convención de qd2_quad_plant.m y del
simulador 3D (rot_zyx), comprobada con un vuelo real del Equipo2. OJO: con la nariz abajo la cámara
(que mira hacia abajo) ve hacia ATRÁS del dron, así que el centro de la imagen cae detrás del punto
que daría la suposición nivelada (≈ -z·tan(pitch) en x del cuerpo); con roll > 0 la cámara ve hacia
la izquierda (+y). La orientación de la cámara es ``R_mundo_cuerpo · R_cuerpo_cámara``. Con roll =
pitch = 0 (valor por defecto) el resultado es idéntico al de la versión que solo usaba el yaw.

Pivote de Motive (``pivote_m``): la pose que llega NO es el pivote del rigid body sino el "punto
corregido" de Measurement Conditioning, ``pivote + bias_z · ẑ_MUNDO`` (Bias [0 0 bias_z], -0.07 m por
defecto: ``optitrack.bias_z_dronN``). Ese punto no está fijo al cuerpo: al inclinarse el dron se queda
``b = -bias_z`` m justo debajo del pivote. ``camara.offset_camara_m`` se mide con el dron nivelado desde
el punto corregido, así que el vector fijo al cuerpo es pivote -> lente = ``offset - b·ẑ`` y
    p_cámara = p + b·ẑ + R_mundo_cuerpo · (offset - b·ẑ) = p + R·offset + b·(ẑ - R·ẑ).
Con el dron nivelado ``R·ẑ = ẑ`` y queda ``p + R·offset`` (igual que antes). Si se ignorara el pivote, el
error en el piso sería ≈ b·sin(inclinación): ~6 mm a 5°, ~12 mm a 10°.

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
E3 = np.array([0.0, 0.0, 1.0])
PIVOTE_DEFAULT_M = 0.07     # -optitrack.bias_z_dronN por defecto (qd2_cargar_config.m)


def pivote_de_config(cfg):
    """Altura (m) del pivote de Motive sobre el punto corregido de la pose: ``-bias_z`` de
    ``optitrack.bias_z_dron1/2`` (promedio de los dos: el explorador puede ser cualquiera; si difieren
    un centímetro el efecto es < 2 mm a 10°). Sin la sección o con un valor no numérico: 0.07 m."""
    opt = cfg.get("optitrack") if isinstance(cfg, dict) else None
    opt = opt if isinstance(opt, dict) else {}
    vals = []
    for k in ("bias_z_dron1", "bias_z_dron2"):
        v = opt.get(k, -PIVOTE_DEFAULT_M)
        ok = not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(float(v))
        vals.append(-float(v) if ok else PIVOTE_DEFAULT_M)
    return sum(vals) / len(vals)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def R_mundo_cuerpo(yaw, roll=0.0, pitch=0.0):
    """Rotación cuerpo -> mundo del dron: ``Rz(yaw) · Ry(pitch) · Rx(roll)`` (ZYX intrínseca, rad).

    Es la convención de la Corrected Pose Data de OptiTrack (ver el docstring del módulo): roll > 0 =
    lado izquierdo arriba, pitch > 0 = nariz abajo. Si en el laboratorio un signo sale al revés NO se
    cambia aquí: se corrige con ``vision.actitud_signos`` (``corregir_actitud``)."""
    if not roll and not pitch:
        return rot_z(yaw)               # idéntico bit a bit a la versión nivelada
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def corregir_actitud(roll, pitch, signo_roll=1.0, signo_pitch=1.0, offset_roll=0.0, offset_pitch=0.0):
    """Roll/pitch de la telemetría -> roll/pitch en la convención de ``R_mundo_cuerpo``:
    ``signo · (ángulo - offset)``. ``offset`` = lo que marca el dron nivelado en el piso (rigid body de
    Motive creado un poco inclinado); ``signo`` = -1 si la prueba del libro sale al revés."""
    return signo_roll * (roll - offset_roll), signo_pitch * (pitch - offset_pitch)


def inclinacion(roll, pitch):
    """Ángulo (rad) entre el eje z del cuerpo y la vertical: ``acos(cos(roll)·cos(pitch))``.
    No depende del yaw ni del signo de los ángulos."""
    return math.acos(max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch))))


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
                 offset_m=(0.20, 0.0, -0.05), fx=None, fy=None, cx=None, cy=None, origen=None, pivote_m=0.0):
        self.w, self.h = int(width), int(height)
        self.fx = float(fx) if fx else (self.w / 2.0) / math.tan(math.radians(float(hfov_deg)) / 2.0)
        self.fy = float(fy) if fy else (self.h / 2.0) / math.tan(math.radians(float(vfov_deg)) / 2.0)
        self.cx = float(cx) if cx is not None else (self.w - 1) / 2.0
        self.cy = float(cy) if cy is not None else (self.h - 1) / 2.0
        self.origen = origen or ("intrínsecos dados" if fx else "HFOV/VFOV")
        self.mount_rot_deg = float(mount_rot_deg)
        self.R_bc = R_BODY_CAM0 @ rot_z(math.radians(self.mount_rot_deg))
        self.offset = np.asarray(offset_m, dtype=float).reshape(3)
        self.pivote_m = float(pivote_m)      # pivote de Motive sobre el punto de la pose (docstring del módulo)
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
        El pivote de Motive sale de ``optitrack.bias_z_dron1/2`` (``pivote_de_config``).
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
        m = cls(w2, h2, 0, 0, rot, off, fx=fx, fy=fy, cx=cx, cy=cy, origen=origen, pivote_m=pivote_de_config(cfg))
        m.aviso = aviso
        return m

    def scaled(self, width, height):
        """Mismo modelo para la imagen reescalada a otro tamaño (sin girar; centros de píxel conservados)."""
        sx, sy = width / float(self.w), height / float(self.h)
        return CameraModel(width, height, 0, 0, self.mount_rot_deg, self.offset, fx=self.fx * sx,
                           fy=self.fy * sy, cx=(self.cx + 0.5) * sx - 0.5, cy=(self.cy + 0.5) * sy - 0.5, origen=self.origen + " (escalado)",
                           pivote_m=self.pivote_m)

    # ------------------------------------------------------------------ pose
    def pose(self, x, y, z, yaw, roll=0.0, pitch=0.0):
        """Devuelve (R_mundo<-cámara, posición de la cámara en el mundo).

        ``R_wb = R_mundo_cuerpo(yaw, roll, pitch)``; ``R_wc = R_wb · R_bc``;
        ``p_c = p + R_wb · offset + b · (ẑ - R_wb · ẑ)`` con ``b = pivote_m`` (el offset gira alrededor del
        pivote de Motive, no del punto corregido; ver el docstring del módulo). Nivelado, el último término
        es exactamente 0."""
        R_wb = R_mundo_cuerpo(yaw, roll, pitch)
        R_wc = R_wb @ self.R_bc
        p_c = np.array([x, y, z], dtype=float) + R_wb @ self.offset + self.pivote_m * (E3 - R_wb[:, 2])
        return R_wc, p_c

    # ------------------------------------------------------------------ píxel -> suelo
    # roll y pitch van justo después de yaw (con 0 por defecto) y z_plane SOLO por nombre en las llamadas:
    # así ``cam.metodo(u, v, *pose, z_plane=...)`` funciona igual con poses de 4 que de 6 valores.
    def pixel_to_ground(self, u, v, x, y, z, yaw, roll=0.0, pitch=0.0, z_plane=0.02):
        """Rayo del píxel (u, v) intersecado con el plano z = z_plane. None si no corta."""
        R_wc, p_c = self.pose(x, y, z, yaw, roll, pitch)
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
    def world_to_pixel(self, pts, x, y, z, yaw, roll=0.0, pitch=0.0):
        """``pts`` (N,3) en el mundo -> (N,2) píxeles y máscara de puntos delante de la cámara."""
        R_wc, p_c = self.pose(x, y, z, yaw, roll, pitch)
        P = np.asarray(pts, dtype=float).reshape(-1, 3)
        d = (P - p_c) @ R_wc  # = (R_wc^T (P - p_c))^T
        front = d[:, 2] > 1e-6
        dz = np.where(front, d[:, 2], 1.0)
        uv = np.stack([self.fx * d[:, 0] / dz + self.cx, self.fy * d[:, 1] / dz + self.cy], axis=1)
        return uv, front

    def footprint(self, x, y, z, yaw, roll=0.0, pitch=0.0, z_plane=0.02):
        """Polígono (4 esquinas) que ve la cámara sobre el suelo, en orden de la imagen."""
        corners = [(0, 0), (self.w - 1, 0), (self.w - 1, self.h - 1), (0, self.h - 1)]
        out = []
        for u, v in corners:
            g = self.pixel_to_ground(u, v, x, y, z, yaw, roll, pitch, z_plane=z_plane)
            if g is None:
                return []
            out.append([round(g[0], 3), round(g[1], 3)])
        return out


def dist_polilinea(x, y, vertices):
    """Distancia horizontal (m) del punto (x, y) a la polilínea ``vertices`` (en el orden dado,
    segmentos consecutivos, como ``densify`` de mock_sim y ``path_points`` de mock_world).
    None si hay menos de 2 vértices válidos."""
    V = []
    for v in vertices or []:
        try:
            V.append((float(v[0]), float(v[1])))
        except (TypeError, ValueError, IndexError):
            continue
    if len(V) < 2:
        return None
    best = None
    for (ax, ay), (bx, by) in zip(V[:-1], V[1:]):
        dx, dy = bx - ax, by - ay
        L = dx * dx + dy * dy
        t = 0.0 if L < 1e-12 else min(1.0, max(0.0, ((x - ax) * dx + (y - ay) * dy) / L))
        d = math.hypot(ax + t * dx - x, ay + t * dy - y)
        best = d if best is None or d < best else best
    return best


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
