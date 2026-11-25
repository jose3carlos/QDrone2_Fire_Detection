"""Simulador *mock* de la misión de 2 drones (emula al Simulink de tierra, ARQUITECTURA §1 y §3.3).

Sirve para probar la estación sin MATLAB. Es un **Stream Server TCP** que habla exactamente el
protocolo del puente (:18700): manda 64 doubles a 50 Hz y recibe 32 doubles. La estación se
conecta a él con el mismo ``BridgeClient`` que usa con Simulink, así que el empaquetado real
queda probado de punta a punta.

Lo que emula (versión simplificada; el supervisor "de verdad" vive en Simulink / MATLAB):
* roles: menor x = EXPLORADOR, mayor x = RESPONDEDOR; home = posición al arrancar;
* START → despegue → hover listo → misión; E recorre la N a z_E (empezando por el extremo más
  cercano a su home, igual que ``qd2_mission_step``); R atiende primero el objetivo pendiente
  más cercano, a z_R, hace hover ``t_mark`` encima y lo marca como visitado; al terminar la N
  E regresa a su home;
* comandos con las MISMAS reglas que ``simulink_qdrone2/lib/qd2_mission_step.m``: se aplican
  cuando ``cmd_seq`` cambia (cualquier valor, también 0) y hay heartbeat; el primer contacto
  absorbe el ``cmd_seq`` (no re-ejecuta un comando viejo); START solo en ASSIGN_ROLES; PAUSE solo
  en MISSION / RETURN_HOME; RESUME solo en PAUSED (y con rastreo); ABORT y LAND_ALL en el aire (durante la
  reacción a una pérdida de rastreo solo LAND_ALL); en TAKEOFF sin
  ningún dron en el aire cancelan el despegue (sin takeoff aún: desarma y vuelve a ASSIGN_ROLES;
  con takeoff ya dado: bajan en su sitio hasta DONE). Takeoff se da 2 s después de armar;
* separation guard (PID) con las reglas de ``qd2_separation_guard.m`` (simplificadas): el dron
  con prioridad espera si el otro está a < D+m de su camino [pose, consigna] (también con las
  poses predichas a ``t_pred``); el que cede solo avanza si su camino queda a ≥ D+2m del camino
  del otro (y de los próximos waypoints de E); si no, espera o se evade a ≥ D+3m;
* heartbeat perdido > 1 s en MISSION → PAUSED (hay que mandar RESUME);
* fin: E terminó, cola vacía y R libre → RETURN_HOME (+ aterrizaje si ``auto_land``);
* START rechazado con alerta 1 si los homes están a < d_min_h + guard_margen (PID) o < 0.8 m (MPC);
* telemetría y(55) = alerta (se puede forzar cualquier código 0..10 con ``alerta_forzada`` para
  probar la UI) y y(56:60) = un objetivo de la tabla por mensaje, rotando ``[id clase x y estado]``;
  ``add_internal_target`` agrega objetivos "de Simulink" que la estación no publicó (como el
  simulador 3D con su detector interno);
* o+2 (``fsm_state``) como el modelo real: en PID el Flight State de tierra del Equipo2
  (0 Idle, 1 Armed, 2 Takeoff, 3 Land, 9 MS Stop) y en MPC la FSM a bordo (1..9);
* y(61:63) = roll, pitch y válido del EXPLORADOR (como el TELEMETRY PACK nuevo): la inclinación que
  necesita el empuje para dar la aceleración de la planta (convención ``geo.R_mundo_cuerpo``: pitch > 0
  = nariz abajo al acelerar hacia adelante, roll > 0 = lado izquierdo arriba al acelerar hacia la
  derecha); 0 en el piso; y(63) = 0 sin explorador o sin rastreo. ``actitud_en_tel = False`` emula un
  modelo de Simulink viejo (y(61:63) = 0).
* modo SOLO EXPLORADOR (``mision.modo = "solo_explorador"``, ``mision.dron_solo`` = 1 o 2; reglas de
  ``config.mision_modo``): vuela un solo dron. El dron solo sale de ``homes[dron_solo - 1]`` y es el explorador
  desde INIT (y(4) = dron_solo, y(5) = 0, y(64) = dron_solo); el otro (ausente) sale sin rastreo (pose NaN en la
  trama), ``role`` = 0, ``comm_ok`` = 0 y nunca se mueve ni recibe arm/takeoff. Sin respondedor ni separation
  guard: los objetivos se registran y se quedan "pendientes" (y(53) = cuántos); al terminar la N + 2 s de gracia
  -> RETURN_HOME -> (``auto_land`` o LAND_ALL) -> LANDING (E en su sitio) -> DONE. En dual nada cambia.
* pérdida de rastreo (``perder_rastreo(dron, t1, t2)``: OptiTrack no ve a ese dron para t1 <= t < t2; la trama
  lleva tracked = 0 y pose NaN, y la misión solo usa la ÚLTIMA pose vista). Misma lógica que
  ``qd2_mission_step`` (bloques "Pérdida de rastreo sostenida: aterrizar" y "Pérdida de rastreo", con
  ``iniciar_falla`` / ``paso_falla`` / ``planear_aterrizaje`` / ``paso_aterrizaje`` / ``qd2_ruta_desvio``
  portados línea a línea), con el umbral ``mision.t_perdida_aterrizar_s`` leído y validado igual que
  MATLAB (``config.mision_t_perdida_aterrizar``: 0.2 a 2.0 s, 0.2 por defecto):

  - en el aire, más de ``t_perdida_aterrizar_s`` sin rastreo -> ese dron queda EN FALLA 2 (alerta 2) y se le baja
    takeoff en ese mismo paso (aterriza en su sitio, sin PAUSED con el valor por defecto); nunca vuelve a recibir
    takeoff aunque el rastreo regrese o llegue RESUME / START; un timeout de su aterrizaje no pone la alerta 4;
  - solo explorador: MISSION/... -> LANDING -> DONE (7 -> 8); dual: el otro se aparta a SU altura a >= 1.60 m
    (d_min_h + guard_margen + 0.05 + 0.5 de deriva) de la última pose vista del perdido, regresa a su home (o se
    queda si su home está cerca del perdido) y aterriza: ABORT -> LANDING -> DONE (9 -> 7 -> 8); si el perdido
    está ENCIMA del otro y cerca, su takeoff se baja cuando el otro ya salió de debajo;
  - rampa de despegue sin nadie en el aire: despegue cancelado (nadie despega); TAKEOFF antes de dar takeoff:
    WAIT_TRACKING desarmado; LANDING: sigue aterrizando;
  - con ``t_perdida_aterrizar_s`` > 0.2: entre 0.2 s y ese valor PAUSED (alerta 2, RESUME solo con rastreo).
  Diferencias con MATLAB (el mock es simplificado): sin LANDING "R primero" del aterrizaje normal (un LAND_ALL
  normal baja a los dos en su sitio), así que una pérdida en ese LANDING solo marca la falla.

También se puede correr solo, para que otros (p. ej. el simulador MATLAB o pruebas) tengan
un servidor :18700 de referencia:   ``python -m app.mock_sim --puerto 18700``
"""
from __future__ import annotations

import argparse
import math
import select
import socket
import threading
import time

import numpy as np

from . import protocol as P
from .config import T_PERDIDA_TRK, mision_modo, mision_t_perdida_aterrizar

# Estados de la FSM a bordo (aprox. a los del DroneStack del Equipo2)
FSM_IDLE, FSM_ARM, FSM_TO_THROTTLE, FSM_TO_HEIGHT, FSM_FLIGHT, FSM_AUTOLAND = 1, 2, 3, 4, 5, 6
D_HOME_MIN_MPC = 0.8   # = p.d_home_min_mpc de qd2_params.m
T_ARM_A_DESPEGUE = 2.0  # s, = p.t_arm_a_despegue (takeoff se da este tiempo después de armar)
Z_DESPEGADO = 0.15     # m, = p.z_despegado (un dron más alto que esto está en el aire)
# pérdida de rastreo y reacción (= qd2_params.m)
D_DERIVA_SIN_RASTREO = 0.5  # m, p.d_deriva_sin_rastreo (margen alrededor de la última pose del perdido)
Z_ATERRIZADO = 0.08    # m, p.z_aterrizado
T_ATERRIZADO = 1.0     # s, p.t_aterrizado
T_ASENTAR = 1.0        # s, p.t_asentar
T_LAND_MAX = 30.0      # s, p.t_land_max
T_FALLA_SEP_MAX = 8.0  # s, p.t_falla_sep_max
XY_MARGEN = 0.15       # m, p.xy_margen (consignas recortadas a limites_xy -/+ margen)


def flight_state_tierra(fsm, aterrizo):
    """FSM a bordo del mock -> Flight State de tierra del Flight Plan del Equipo2 (versión PID)."""
    if fsm == FSM_IDLE:
        return 3 if aterrizo else 0
    if fsm == FSM_ARM:
        return 1
    if fsm in (FSM_TO_THROTTLE, FSM_TO_HEIGHT, FSM_FLIGHT):
        return 2
    if fsm == FSM_AUTOLAND:
        return 3
    return 9


def densify(vertices, spacing, z):
    """Waypoints [x, y, z, yaw] cada ``spacing`` m sobre la polilínea (incluye los vértices)."""
    V = np.asarray(vertices, dtype=float)
    out = [[V[0, 0], V[0, 1], z, 0.0]]
    for a, b in zip(V[:-1], V[1:]):
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / spacing)))
        for k in range(1, n + 1):
            p = a + (b - a) * k / n
            out.append([p[0], p[1], z, 0.0])
    return np.array(out[:128])


def contar_waypoints(vertices, spacing, max_wp=128):
    """Número de waypoints que da ``qd2_track_waypoints.m`` (y(7) = explorer_wp_total) para esta pista:
    misma regla exacta (``ceil(L/spacing - 1e-9)`` por tramo, tramos de longitud 0 ignorados y espaciado
    agrandado si no caben en 128). La usa el aviso de "pista distinta" de la estación."""
    try:
        V = [(float(a), float(b)) for a, b in vertices]
    except (TypeError, ValueError):
        return 0
    nv = 0
    for v in V:
        if all(math.isfinite(c) for c in v):
            nv += 1
        else:
            break
    if nv < 1:
        return 0
    spacing = float(spacing) if spacing and float(spacing) > 0 else 0.25
    L = [math.hypot(V[k][0] - V[k - 1][0], V[k][1] - V[k - 1][1]) for k in range(1, nv)]
    disponible = max_wp - 1 - (nv - 1)
    if sum(L) / spacing > disponible:
        spacing = sum(L) / disponible
    n = 1
    for seg in L:
        if seg < 1e-9:
            continue
        n += int(math.ceil(seg / spacing - 1e-9))
    return min(n, max_wp)


def hdist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _seg_dist(p, a, b):
    """Distancia horizontal del punto p al segmento [a, b] (escalares: rápido)."""
    px, py, ax, ay, bx, by = float(p[0]), float(p[1]), float(a[0]), float(a[1]), float(b[0]), float(b[1])
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0.0 if L < 1e-12 else min(1.0, max(0.0, ((px - ax) * dx + (py - ay) * dy) / L))
    return math.hypot(ax + t * dx - px, ay + t * dy - py)


def _seg_seg(a1, a2, b1, b2):
    """Distancia mínima entre dos segmentos 2D (0 si se cruzan)."""
    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
    d1, d2 = cross(b1, b2, a1), cross(b1, b2, a2)
    d3, d4 = cross(a1, a2, b1), cross(a1, a2, b2)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return 0.0
    return min(_seg_dist(a1, b1, b2), _seg_dist(a2, b1, b2), _seg_dist(b1, a1, a2), _seg_dist(b2, a1, a2))


def _min_dist(p, segs):
    return min(_seg_dist(p, a, b) for a, b in segs)


def _path_min(p, q, segs):
    return min(_seg_seg(p, q, a, b) for a, b in segs)


def _leg_ok(p, q, groups, dist, hard_min=0.0):
    """El tramo [p, q] no se acerca a ninguna zona más de lo que ya está (ni de ``dist`` si está lejos).
    Al cuerpo de A (``groups[0]``) nunca a menos de ``hard_min`` (evita acercarse poco a poco)."""
    for i, g in enumerate(groups):
        if g:
            now = _min_dist(p, g)
            need = min(now, dist) - 0.05
            if i == 0:
                # ya demasiado cerca de A: solo se aceptan tramos que no lo acercan nada
                need = min(now - 0.01, dist) if now < hard_min else max(need, hard_min)
            if _path_min(p, q, g) < need:
                return False
    return True


G = 9.81


def actitud_de_aceleracion(a_w, yaw):
    """(roll, pitch) que inclinan el empuje hacia ``a_w + g·z`` con R = Rz(yaw)·Ry(pitch)·Rx(roll)
    (``geo.R_mundo_cuerpo``): la dirección del empuje en el cuerpo sin yaw es
    Ry(pitch)·Rx(roll)·e3 = [cos(roll)·sin(pitch), -sin(roll), cos(roll)·cos(pitch)]."""
    c, s = math.cos(yaw), math.sin(yaw)
    ax, ay = c * a_w[0] + s * a_w[1], -s * a_w[0] + c * a_w[1]     # Rz(-yaw)·a
    t = np.array([ax, ay, G + a_w[2]])
    t /= np.linalg.norm(t)
    roll = -math.asin(max(-1.0, min(1.0, t[1])))
    pitch = math.atan2(t[0], t[2])
    return roll, pitch


class Drone:
    def __init__(self, x, y, yaw=0.0):
        self.p = np.array([x, y, 0.0])
        self.v = np.zeros(3)
        self.a = np.zeros(3)       # aceleración del último paso (mundo), para la actitud de y(61:62)
        self.yaw = yaw
        self.sp = np.array([x, y, 0.0])
        self.yawd = yaw
        self.flying = False
        self.fsm = FSM_IDLE
        self.battery = 16.4
        self.tracked = 1
        self.home = np.array([x, y])
        self.role = P.ROLE_NONE
        self.aterrizo = False      # ya aterrizó después de volar (Flight State de tierra = 3 Land)

    def step(self, dt, vmax_xy=0.55, vmax_z=0.5, tau=0.35):
        if not self.flying and self.sp[2] <= 0.0 and self.p[2] <= 0.01:
            self.v[:] = 0.0
            self.a[:] = 0.0
            self.p[2] = 0.0
            return
        err = self.sp - self.p
        vd = 1.6 * err
        n = math.hypot(vd[0], vd[1])
        if n > vmax_xy:
            vd[:2] *= vmax_xy / n
        vd[2] = max(-vmax_z, min(vmax_z, vd[2]))
        dv = (vd - self.v) * min(1.0, dt / tau)
        self.a = dv / dt if dt > 0 else np.zeros(3)
        self.v += dv
        self.p += self.v * dt
        if self.p[2] < 0.0:
            self.p[2], self.v[2] = 0.0, 0.0
            self.a[2] = 0.0
        e = math.atan2(math.sin(self.yawd - self.yaw), math.cos(self.yawd - self.yaw))
        self.yaw += max(-1.0 * dt, min(1.0 * dt, e))
        self.battery = max(13.5, self.battery - (0.0009 if self.flying else 0.00002) * dt)


class MockMission:
    """FSM de misión + planta. ``step(dt)`` avanza el tiempo; no hay hilos aquí (fácil de probar)."""

    def __init__(self, cfg, homes=((0.9, -0.4), (-0.9, 0.4))):
        self.cfg = cfg
        m = cfg.get("mision", {})
        self.z_E = float(m.get("z_explorador", 1.5))
        self.z_R = float(m.get("z_respondedor", 0.5))
        self.t_mark = float(m.get("t_mark", 3.0))
        self.d_min_h = float(m.get("d_min_h", 0.9))
        self.dz_min = float(m.get("dz_min", 0.8))
        self.spacing = float(m.get("espaciado_waypoints", 0.25))
        self.tol_xy = float(m.get("tol_xy", 0.12))
        self.tol_z = float(m.get("tol_z", 0.12))
        self.auto_land = bool(m.get("auto_land", False))
        lim = m.get("limites_xy", [-1.5, 1.5])
        self.lim = (float(lim[0]) + 0.1, float(lim[1]) - 0.1)
        self.version = 2 if str(cfg.get("version_control", "PID")).upper() == "MPC" else 1
        self.vertices = cfg.get("pista", {}).get("vertices", [[-1.2, 1.2], [1.2, 1.2], [-1.2, -1.2], [1.2, -1.2]])
        self.drones = {1: Drone(*homes[0]), 2: Drone(*homes[1])}
        self.actitud_en_tel = True   # False = modelo de Simulink viejo: y(61:63) = 0
        self.t = 0.0
        self.state = P.INIT
        self.prev_state = P.INIT
        self.state_t = 0.0
        self.track_t = 0.0
        self.E = self.R = 0
        # modo solo explorador: un solo dron (ds); el otro (da) sin rastreo, sin rol, sin comm y quieto
        modo = mision_modo(cfg)
        self.solo = modo["modo"] == "solo_explorador"
        self.ds = modo["dron_solo"] if self.solo else 0
        if self.solo:
            self.da = 3 - self.ds
            self.drones[self.da].tracked = 0
            self.E = self.ds          # explorer_id fijo desde INIT (y(4) = y(64) = dron_solo)
        self.wps = densify(self.vertices, self.spacing, self.z_E)
        self.wp_idx = 0
        self.explorer_done = False
        self.targets = {}        # id -> dict(cls, x, y, conf)
        self.alias = {}          # id nuevo a < radio_cluster de uno conocido -> id conocido (como Simulink)
        self.radio_cluster = float(cfg.get("vision", {}).get("radio_cluster_m", 0.25) or 0.25)
        self.visited = set()
        self.queue = []          # ids pendientes, en orden de llegada
        self.current = 0         # objetivo que atiende R
        self.mark_t = 0.0
        self.guard = False
        self.cmd_seq_ack = 0
        self.hb_visto = False
        self.pausa_por_hb = False
        self.guard_margin = 0.15     # p.guard_margen de qd2_params
        self.t_pred = 0.7            # s, horizonte de predicción de las poses en el guard
        self.hold = {}               # punto donde espera cada dron retenido por el guard
        self.wait_pt = {}
        self.via = {}
        self._via_next = {}
        self._swap_until = -1.0
        self._hold_t0 = 0.0
        self.r_idle = None
        self.t_e_done = 0.0
        self.last_hb = None
        self.last_hb_t = 0.0
        self.last_sp_E = None
        self.min_hdist_flying = float("inf")   # para pruebas
        self.events = []
        self.alerta = 0              # y(55) del supervisor (0 nada, 1 homes cerca)
        self.alerta_forzada = None   # para probar la UI: código 0..10 que sustituye a y(55)
        self.home_ok = True
        self._rot_obj = 0            # rotación de y(56:60)
        self.takeoff_dado = False    # en TAKEOFF: ya se dio takeoff (T_ARM_A_DESPEGUE después de armar)
        # ---- pérdida de rastreo (qd2_mission_step): umbral, rastreo visto y falla por dron
        tp = mision_t_perdida_aterrizar(cfg)
        self.t_perdida_aterrizar = tp["valor"]
        self.aviso_t_perdida = tp["aviso"]
        self.xy_lo, self.xy_hi = float(lim[0]) + XY_MARGEN, float(lim[1]) - XY_MARGEN
        self.ventanas_sin_rastreo = []          # (dron, t1, t2): OptiTrack no lo ve para t1 <= t < t2
        self.t_trk = {1: 0.0, 2: 0.0}           # último instante con rastreo válido (s.t_trk)
        self.vista = {k: d.p.copy() for k, d in self.drones.items()}   # última pose vista (s.pose_ok)
        self.tk = {1: 0, 2: 0}                  # takeoff por DRON (como tel.takeoff de qd2_mission_step)
        self.falla = {1: 0, 2: 0}               # por DRON: 2 = pérdida de rastreo sostenida (s.falla)
        self.pausa_por_trk = False
        self.falla_fase = 0                     # reacción: 1 apartarse, 2 regresar, 3 aterriza el sano, 4 espera
        self.falla_sano = 0
        self.falla_land = False
        self.falla_trk = 0
        self.falla_sep = np.zeros(2)
        self.falla_hold = np.zeros(3)
        self.falla_z = 0.0
        self.falla_land_xy = np.zeros(2)
        self.falla_via = np.zeros(2)
        self.falla_usa_via = False
        self.t_falla_fase = 0.0
        self.t_falla_llego = -1.0
        self.t_falla_suelo = -1.0
        # LANDING portado de paso_aterrizaje (solo en los caminos de pérdida de rastreo)
        self.land_matlab = False
        self.land_paso = 0
        self.land_xy = np.zeros((2, 2))         # fila 0 = E, fila 1 = R (en solo: el ausente)
        self.land_via_E = np.zeros(2)
        self.land_usa_via_E = False
        self.land_r_baja = False
        self.t_suelo = [-1.0, -1.0]             # [E R]
        self.t_land_paso = 0.0
        self.t_suelo_v = {1: -1.0, 2: -1.0}     # LANDING simplificado: desde cuándo se VE cada dron en el suelo

    # ------------------------------------------------------------------ entradas
    def apply_command_message(self, msg):
        """``msg`` = dict de ``protocol.unpack_command`` (mismas reglas que qd2_mission_step)."""
        if msg["heartbeat"] != self.last_hb:
            self.last_hb, self.last_hb_t = msg["heartbeat"], self.t
            if not self.hb_visto:
                # primer contacto: no re-ejecutar un comando viejo de la estación
                self.hb_visto = True
                self.cmd_seq_ack = msg["cmd_seq"]
        for s in msg["slots"]:
            if 0 < s["id"] <= P.MAX_TARGETS:
                if s["id"] in self.alias:
                    continue
                new = s["id"] not in self.targets
                if new:
                    # como actualizar_objetivos de qd2_mission_step: un id nuevo a < radio_cluster de un
                    # objetivo conocido (sin mirar la clase) es el mismo objeto -> alias, no se agrega
                    cerca = min(((hdist((t["x"], t["y"]), (s["x"], s["y"])), k) for k, t in self.targets.items()),
                                default=(None, None))
                    if cerca[0] is not None and cerca[0] < self.radio_cluster:
                        self.alias[s["id"]] = cerca[1]
                        self.events.append((round(self.t, 2), f"objetivo {s['id']} = alias del {cerca[1]}"))
                        continue
                if not new and (s["id"] in self.visited or (s["id"] == self.current and self.mark_t > 0)):
                    continue  # Simulink no refina un objetivo visitado ni el que se está marcando
                self.targets[s["id"]] = {"cls": s["cls"], "x": s["x"], "y": s["y"], "conf": s["conf"]}
                if new and s["id"] not in self.visited:
                    self.queue.append(s["id"])
                    self.events.append((round(self.t, 2), f"objetivo {s['id']} recibido"))
        for key, attr in (("z_explorer", "z_E"), ("z_responder", "z_R"), ("t_mark", "t_mark"), ("d_min_h", "d_min_h")):
            if msg.get(key, 0.0) > 0.0:
                setattr(self, attr, float(msg[key]))
        hb_ok = self.hb_visto and (self.t - self.last_hb_t) <= 1.0
        if hb_ok and msg["cmd_seq"] != self.cmd_seq_ack:
            self.cmd_seq_ack = msg["cmd_seq"]
            self.command(msg["cmd"])

    def command(self, cmd):
        if cmd == P.CMD_NONE:
            return
        if self.falla_fase > 0:
            # reacción a una falla en curso (qd2_mission_step): solo LAND_ALL (el sano aterriza en cuanto está
            # apartado); START, ABORT, PAUSE y RESUME se confirman pero no se ejecutan
            if cmd == P.CMD_LAND_ALL:
                self.falla_land = True
                self.events.append((round(self.t, 2), "comando LAND_ALL durante la reacción a la falla"))
            else:
                self.events.append((round(self.t, 2), f"comando {P.COMMAND_NAMES.get(cmd, cmd)} ignorado "
                                    "(reacción a una falla en curso)"))
            return
        s = self.state
        # en el aire: estados de vuelo, o TAKEOFF solo si ya se dio takeoff y algún dron pasó z_despegado
        despegado = any(d.p[2] > Z_DESPEGADO for d in self.drones.values())
        aire = (s in (P.HOVER_READY, P.MISSION, P.RETURN_HOME, P.ABORT, P.PAUSED)
                or (s == P.TAKEOFF and self.takeoff_dado and despegado))
        if cmd in (P.CMD_ABORT, P.CMD_LAND_ALL) and s == P.TAKEOFF and not aire:
            self._cancelar_despegue()
            self.events.append((round(self.t, 2), f"comando {P.COMMAND_NAMES.get(cmd, cmd)}: despegue cancelado"))
            return
        if cmd == P.CMD_START and s == P.ASSIGN_ROLES and self.E and not self.home_ok:
            self.alerta = 1
            self.events.append((round(self.t, 2), "START rechazado: homes demasiado cerca (alerta 1)"))
            return
        if cmd == P.CMD_START and s == P.ASSIGN_ROLES and self.E:
            self._start_mission()
            self._goto(P.TAKEOFF)
        elif cmd == P.CMD_PAUSE and s in (P.MISSION, P.RETURN_HOME):
            self._pause()
            self.pausa_por_hb = False
        elif cmd == P.CMD_RESUME and s == P.PAUSED and not self._perdido():
            # RESUME (solo con el rastreo de vuelta)
            self.pausa_por_hb = False
            self.pausa_por_trk = False
            if self.alerta == 2:
                self.alerta = 0
            self._goto(self.prev_state)
        elif cmd == P.CMD_ABORT and aire:
            self._goto(P.ABORT)
        elif cmd == P.CMD_LAND_ALL and aire:
            self._goto(P.LANDING)
        else:
            self.events.append((round(self.t, 2), f"comando {P.COMMAND_NAMES.get(cmd, cmd)} ignorado en {P.MISSION_STATES[s]}"))
            return
        self.events.append((round(self.t, 2), f"comando {P.COMMAND_NAMES.get(cmd, cmd)}"))

    def _start_mission(self):
        """Como START en qd2_mission_step: la N empieza por el extremo más cercano al home de E."""
        self.wps = densify(self.vertices, self.spacing, self.z_E)
        hE = self.drones[self.E].home
        if len(self.wps) > 1 and hdist(self.wps[-1], hE) < hdist(self.wps[0], hE):
            self.wps = self.wps[::-1].copy()
        self.wp_idx, self.explorer_done = 0, False
        self.takeoff_dado = False
        self.alerta = 0
        self.falla = {1: 0, 2: 0}                # como START en qd2_mission_step (s.falla = [0 0])
        self.land_matlab = False
        self.current, self.mark_t, self.r_idle = 0, 0.0, None
        self.hold, self.wait_pt, self.via = {}, {}, {}

    def _cancelar_despegue(self):
        """ABORT / LAND_ALL en TAKEOFF sin ningún dron en el aire (como cancelar_despegue de qd2_mission_step):
        nunca se da un takeoff nuevo.
        * takeoff aún no dado: se desarma y se vuelve a ASSIGN_ROLES (se puede dar START otra vez);
        * takeoff ya dado (rampa de empuje): se baja takeoff de los dos en su sitio (LANDING) hasta DONE."""
        if not self.takeoff_dado:
            for d in self.drones.values():
                d.flying, d.fsm = False, FSM_IDLE
                d.sp = np.array([d.p[0], d.p[1], 0.0])
            self._goto(P.ASSIGN_ROLES)
        else:
            self.takeoff_dado = False
            for d in self.drones.values():
                d.sp = np.array([d.p[0], d.p[1], 0.0])
            self._goto(P.LANDING)

    def _goto(self, st):
        if st != self.state:
            self.events.append((round(self.t, 2), f"{P.MISSION_STATES[self.state]} -> {P.MISSION_STATES[st]}"))
        self.state, self.state_t = st, 0.0

    def _pause(self):
        self.prev_state = self.state
        for d in self.drones.values():
            d.sp = d.p.copy()  # hover donde está
        self._goto(P.PAUSED)

    # ------------------------------------------------------------------ paso
    def step(self, dt):
        self.t += dt
        self.state_t += dt
        self._actualizar_rastreo()
        dE = self.drones.get(self.E)
        dR = self.drones.get(self.R)
        self.guard = False
        self._perdida_rastreo()          # bloques de pérdida de rastreo de qd2_mission_step (antes de la FSM)
        s = self.state

        if self.falla_fase > 0:
            self._paso_falla()           # reacción: estado 9 mientras el sano se aparta y regresa, 7, luego 8
        elif s == P.INIT and self.t > 0.2:
            self._goto(P.WAIT_TRACKING)
        elif s == P.WAIT_TRACKING and self.solo:
            # solo: basta el dron solo rastreado 1 s; sin chequeo de distancia entre homes (nunca alerta 1)
            self.track_t = self.track_t + dt if dE.tracked else 0.0
            if self.track_t >= 1.0:
                dE.home = dE.p[:2].copy()
                dE.role = P.ROLE_EXPLORER
                self.home_ok, self.alerta = True, 0
                self._goto(P.ASSIGN_ROLES)
        elif s == P.WAIT_TRACKING:
            ok = all(d.tracked for d in self.drones.values())
            self.track_t = self.track_t + dt if ok else 0.0
            if self.track_t >= 1.0:
                d1, d2 = self.drones[1], self.drones[2]
                self.E, self.R = (1, 2) if d1.p[0] < d2.p[0] else (2, 1)
                for k, d in self.drones.items():
                    d.home = d.p[:2].copy()
                    d.role = P.ROLE_EXPLORER if k == self.E else P.ROLE_RESPONDER
                d_min = D_HOME_MIN_MPC if self.version == 2 else self.d_min_h + self.guard_margin
                self.home_ok = hdist(d1.home, d2.home) >= d_min
                self.alerta = 0 if self.home_ok else 1
                self._goto(P.ASSIGN_ROLES)
        elif s == P.TAKEOFF:
            # se arma al entrar y takeoff se da T_ARM_A_DESPEGUE después (p.t_arm_a_despegue)
            if not self.takeoff_dado and self.state_t >= T_ARM_A_DESPEGUE:
                self.takeoff_dado = True
            for d, z in self._volando(((dE, self.z_E), (dR, self.z_R))):
                if self.takeoff_dado:
                    d.flying = True
                    d.fsm = FSM_TO_HEIGHT
                    d.sp = np.array([d.home[0], d.home[1], z])
                else:
                    d.fsm = FSM_ARM
                    d.sp = np.array([d.home[0], d.home[1], 0.0])
            if self.takeoff_dado and all(abs(d.p[2] - d.sp[2]) < self.tol_z for d in self._volando((dE, dR))):
                self._goto(P.HOVER_READY)
        elif s == P.HOVER_READY:
            for d in self._volando((dE, dR)):
                d.fsm = FSM_FLIGHT
            if self.state_t >= 2.0:
                self._goto(P.MISSION)
        elif s == P.MISSION:
            if self.last_hb is None or self.t - self.last_hb_t > 1.0:
                self.events.append((round(self.t, 2), "heartbeat perdido > 1 s"))
                self._pause()
                self.pausa_por_hb = True
            else:
                self._mission_step(dE, dR, dt)
        elif s in (P.RETURN_HOME, P.ABORT) and self.solo:
            # solo: E directo a su home a z_E (sin orden de regreso ni guard)
            dE.sp = np.array([dE.home[0], dE.home[1], self.z_E])
            self.last_sp_E = dE.sp.copy()
            if (hdist(dE.p, dE.home) < self.tol_xy and s == P.RETURN_HOME and self.auto_land
                    and self.state_t > 2.0):
                self._goto(P.LANDING)
        elif s in (P.RETURN_HOME, P.ABORT):
            spE = np.array([dE.home[0], dE.home[1], self.z_E])
            spR = np.array([dR.home[0], dR.home[1], self.z_R])
            if self.version == 1:  # E regresa primero; R cede el paso
                spE, spR = self._guard(dE, dR, spE, spR, False, False)
            dE.sp, dR.sp = spE, spR
            self.last_sp_E = spE.copy()
            at_home = all(hdist(d.p, d.home) < self.tol_xy for d in (dE, dR))
            if at_home and s == P.RETURN_HOME and self.auto_land and self.state_t > 2.0:
                self._goto(P.LANDING)
        elif s == P.LANDING and self.land_matlab:
            self._paso_aterrizaje()      # caminos de pérdida de rastreo: paso_aterrizaje de MATLAB
            if self.land_paso == 3:
                self._goto(P.DONE)
        elif s == P.LANDING:
            for d in self.drones.values():
                if d.flying:
                    d.fsm = FSM_AUTOLAND
                    d.sp = np.array([d.p[0], d.p[1], 0.0])
                    if d.p[2] < 0.05:
                        d.flying, d.fsm = False, FSM_IDLE
                        d.aterrizo = True
                        d.sp[2] = 0.0
            # un dron sin rastreo (falla 2) no se ve en el suelo: se espera a verlo o p.t_land_max (sin alerta 4)
            sin_ver = any(self.falla[k] == 2 and not self._visto_en_suelo(k) for k in self.drones)
            if not any(d.flying for d in self.drones.values()) and not (sin_ver and self.state_t < T_LAND_MAX):
                self._goto(P.DONE)

        self._takeoff_por_estado()
        for d in self.drones.values():
            d.step(dt)
        if dE is not None and dR is not None and dE.flying and dR.flying and dE.p[2] > 0.3 and dR.p[2] > 0.3:
            self.min_hdist_flying = min(self.min_hdist_flying, hdist(dE.p, dR.p))

    def _volando(self, items):
        """En solo, de las parejas (E, R) solo queda la de E (R no existe); en dual, todas."""
        items = list(items)
        return items[:1] if self.solo else items

    def _clamp(self, xy):
        return np.clip(np.asarray(xy, dtype=float), self.lim[0], self.lim[1])

    def _mission_step_solo(self, dE):
        """Solo explorador: E recorre la N y regresa; los objetivos se registran pero nadie los visita."""
        if not self.explorer_done:
            wp = self.wps[self.wp_idx]
            spE = np.array([wp[0], wp[1], self.z_E])
            if hdist(dE.p, wp) < self.tol_xy and abs(dE.p[2] - self.z_E) < self.tol_z:
                if self.wp_idx + 1 < len(self.wps):
                    self.wp_idx += 1
                else:
                    self.explorer_done = True
                    self.t_e_done = self.t
                    self.events.append((round(self.t, 2), "explorador terminó la N"))
        else:
            spE = np.array([dE.home[0], dE.home[1], self.z_E])
        dE.sp = spE
        self.last_sp_E = spE.copy()
        if self.explorer_done and self.t - self.t_e_done >= 2.0:
            self._goto(P.RETURN_HOME)

    def _mission_step(self, dE, dR, dt):
        if self.solo:
            return self._mission_step_solo(dE)
        # --- explorador: recorre la N; al terminar regresa a su home (a su altura)
        if not self.explorer_done:
            wp = self.wps[self.wp_idx]
            spE = np.array([wp[0], wp[1], self.z_E])
            if hdist(dE.p, wp) < self.tol_xy and abs(dE.p[2] - self.z_E) < self.tol_z:
                if self.wp_idx + 1 < len(self.wps):
                    self.wp_idx += 1
                else:
                    self.explorer_done = True
                    self.t_e_done = self.t
                    self.events.append((round(self.t, 2), "explorador terminó la N"))
        else:
            spE = np.array([dE.home[0], dE.home[1], self.z_E])
        # --- respondedor: el objetivo pendiente más cercano primero
        marking = False
        if self.current == 0 and self.queue:
            self.current = min(self.queue, key=lambda i: hdist(dR.p, (self.targets[i]["x"], self.targets[i]["y"])))
            self.queue.remove(self.current)
            self.mark_t = 0.0
            self.events.append((round(self.t, 2), f"R va al objetivo {self.current}"))
        if self.current:
            tg = self.targets[self.current]
            xy = self._clamp((tg["x"], tg["y"]))
            spR = np.array([xy[0], xy[1], self.z_R])
            if self.mark_t > 0 or (hdist(dR.p, spR) < self.tol_xy and abs(dR.p[2] - self.z_R) < self.tol_z):
                marking = True
                if hdist(dR.p, spR) < 1.5 * self.tol_xy:
                    self.mark_t += dt
                if self.mark_t >= self.t_mark:
                    self.visited.add(self.current)
                    self.events.append((round(self.t, 2), f"objetivo {self.current} visitado"))
                    self.r_idle = spR[:2].copy()
                    self.current, self.mark_t, marking = 0, 0.0, False
        if not self.current:
            idle = dR.home if self.r_idle is None else self.r_idle
            spR = np.array([idle[0], idle[1], self.z_R])
        # --- separation guard (solo PID)
        if self.version == 1:
            spE, spR = self._guard(dE, dR, spE, spR, marking, self.explorer_done)
        dE.sp, dR.sp = spE, spR
        self.last_sp_E = spE.copy()
        # --- fin de misión (como Simulink: 2 s de gracia por objetivos tardíos, cola vacía y R libre)
        if self.explorer_done and self.t - self.t_e_done >= 2.0 and not self.queue and self.current == 0:
            self._goto(P.RETURN_HOME)

    # ------------------------------------------------------------------ separation guard
    def _guard(self, dE, dR, spE, spR, marking, prio_R):
        """Filtro de consignas PID (ver docstring del módulo). Devuelve (spE, spR) filtradas."""
        D, m = self.d_min_h, self.guard_margin
        first_R = prio_R and not marking
        if self.t < self._swap_until and not marking:
            first_R = not first_R   # desbloqueo: se invierte la prioridad un rato
        if first_R:
            A, B, spA, spB, b_lock = dR, dE, spR, spE, False
        else:
            A, B, spA, spB, b_lock = dE, dR, spE, spR, marking
        kA = self.E if A is dE else self.R
        kB = self.R if A is dE else self.E
        pA, pB = A.p[:2].copy(), B.p[:2].copy()
        pA_pred = pA + self.t_pred * A.v[:2]
        pB_pred = pB + self.t_pred * B.v[:2]
        guard = False
        # los papeles A/B pueden cambiar (R marcando, E terminó): se olvida lo retenido del otro papel
        self.hold.pop(kB, None)
        self.wait_pt.pop(kA, None)
        self.via.pop(kA, None)
        # 1) el que tiene prioridad (A) espera si B está cerca de su camino o de su predicción
        spA_want = spA[:2].copy()           # adonde QUIERE ir A (aunque espere)
        segA = [(pA, spA_want), (pA, pA_pred)]
        dB_A = min(_seg_dist(q, a, b) for q in (pB, pB_pred) for a, b in segA)
        if dB_A < D + m:
            guard = True
            if kA not in self.hold:
                self.hold[kA] = pA.copy()
                self._hold_t0 = self.t
            spA = np.array([self.hold[kA][0], self.hold[kA][1], spA[2]])
            # bloqueo mutuo (B no tiene por dónde quitarse del camino de A): se invierte la prioridad
            if not b_lock and self.t - self._hold_t0 > 4.0 and math.hypot(B.v[0], B.v[1]) < 0.05:
                self._swap_until = self.t + 8.0
                self.hold, self.wait_pt, self.via = {}, {}, {}
                self.events.append((round(self.t, 2), "guard: bloqueo mutuo, se invierte la prioridad 8 s"))
        else:
            self.hold.pop(kA, None)
        # zona de A que B no debe pisar: su camino, su predicción y (si A = E) los próximos waypoints
        # (se usa el camino deseado, no el retenido: si no, B nunca se quita y A espera para siempre)
        hard = [(pA, spA_want), (pA, pA_pred)]
        danger = list(hard)
        if A is dE and not self.explorer_done and self.state == P.MISSION:
            prev = spA_want
            for w in self.wps[self.wp_idx:self.wp_idx + 6]:
                danger.append((prev, w[:2]))
                prev = w[:2]
        # los TRAMOS de B solo se revisan contra el cuerpo de A (pose y predicción; nunca a < D+m); el camino
        # y los waypoints de A solo cuentan para el punto final (B puede cruzarlos lejos de A)
        groups = [[(pA, pA_pred)]]
        # 2) el que cede (B)
        if b_lock:
            pass  # B está marcando: no se mueve (A ya espera)
        elif _path_min(pB, spB[:2], danger) >= D + 2 * m:
            self.wait_pt.pop(kB, None)
            self.via.pop(kB, None)
        else:
            guard = True
            # ¿hay un rodeo (un punto intermedio) seguro hasta su consigna?
            via = self.via.get(kB)
            if via is not None and (hdist(via, pB) < self.tol_xy or _min_dist(via, danger) < D + m
                                    or _path_min(via, spB[:2], danger) < D + m
                                    or not _leg_ok(pB, via, groups, D + m, D + m)):
                via = None
            if via is None and _min_dist(spB[:2], danger) >= D + m and self.t >= self._via_next.get(kB, 0.0):
                via = self._via(pB, spB[:2], danger, groups, D + 2 * m)
                if via is None:
                    self._via_next[kB] = self.t + 0.25   # búsqueda costosa: no repetir en cada paso
            if via is not None:
                self.via[kB] = via
                self.wait_pt.pop(kB, None)
                spB = np.array([via[0], via[1], spB[2]])
                self.guard = True
                return self._final(A, dE, kA, pA, spA, spB, D)
            self.via.pop(kB, None)
            w = self.wait_pt.get(kB)
            if w is None or _min_dist(w, danger) < D + 2 * m or not _leg_ok(pB, w, groups, D + 2 * m, D + m):
                w = pB.copy() if _min_dist(pB, danger) >= D + 2 * m else self._evade(B, pB, danger, groups, D + 3 * m)
                self.wait_pt[kB] = w
            spB = np.array([w[0], w[1], spB[2]])
        # freno: si la predicción de B se mete a < D+m del cuerpo de A y se está acercando, B se queda en hover
        if not b_lock:
            body = groups[0]
            d_now, d_pred = _min_dist(pB, body), _min_dist(pB_pred, body)
            d_sp = _path_min(pB, spB[:2], body) if hdist(pB, spB) > 1e-6 else d_now
            if d_pred < D + m and d_pred < d_now and d_sp < d_now:
                guard = True
                spB = np.array([pB[0], pB[1], spB[2]])
                self.wait_pt[kB] = pB.copy()
        self.guard = self.guard or guard
        return self._final(A, dE, kA, pA, spA, spB, D)

    def _final(self, A, dE, kA, pA, spA, spB, D):
        """Respaldo final: consignas a < D -> A espera donde está."""
        if hdist(spA, spB) < D:
            self.guard = True
            self.hold.setdefault(kA, pA.copy())
            spA = np.array([self.hold[kA][0], self.hold[kA][1], spA[2]])
        return (spA, spB) if A is dE else (spB, spA)

    def _via(self, pB, target, danger, groups, dist):
        """Punto intermedio para rodear la zona de A: ambos tramos a ≥ ``dist`` (el primero sin acercarse).
        Si no hay, se intenta con el margen mínimo D+m (arena chica: pasar junto a la pared)."""
        for d in (dist, self.d_min_h + self.guard_margin):
            c = self._via_grid(pB, target, danger, groups, d)
            if c is not None:
                return c
        return None

    def _via_grid(self, pB, target, danger, groups, dist):
        best, best_len = None, None
        for x in np.arange(self.lim[0], self.lim[1] + 1e-6, 0.25):
            for y in np.arange(self.lim[0], self.lim[1] + 1e-6, 0.25):
                c = np.array([x, y])
                if _min_dist(c, danger) < dist:
                    continue
                L = hdist(pB, c) + hdist(c, target)
                if best_len is not None and L >= best_len:
                    continue
                if (_min_dist(c, danger) >= dist and _path_min(c, target, danger) >= dist
                        and _leg_ok(pB, c, groups, dist, self.d_min_h + self.guard_margin)):
                    best, best_len = c, L
        return best

    def _evade(self, B, pB, danger, groups, dist):
        """Punto a ≥ ``dist`` de la zona de A, dentro de la arena, el más cercano a B, sin acercarse en el camino."""
        cands = [self._clamp(B.home)]
        for r in (0.3, 0.6, 0.9, 1.2, 1.6, 2.0):
            for a in np.linspace(0, 2 * math.pi, 16, endpoint=False):
                cands.append(self._clamp(pB + r * np.array([math.cos(a), math.sin(a)])))
        ok = [c for c in cands if _min_dist(c, danger) >= dist and _leg_ok(pB, c, groups, dist, self.d_min_h + self.guard_margin)]
        if ok:
            return min(ok, key=lambda c: hdist(c, pB))
        # sin punto ideal: el que deja más lejos sin que el camino pase cerca de A (o quedarse en hover)
        cands.append(pB.copy())
        return max(cands, key=lambda c: min(_min_dist(c, danger), _path_min(pB, c, groups[0]) if hdist(c, pB) > 1e-6
                                            else _min_dist(pB, groups[0])))

    # ------------------------------------------------------------------ pérdida de rastreo (qd2_mission_step)
    def perder_rastreo(self, dron, t1, t2=math.inf):
        """OptiTrack deja de ver al dron ``dron`` (1 o 2) para ``t1 <= t < t2`` (s de misión, ``self.t``): la trama
        lleva tracked = 0 y pose NaN y la misión solo usa la última pose vista. El dron (la planta) sigue su
        consigna: en el laboratorio el binario PID ya cae a los 0.2 s (FSM a bordo 7), aquí baja con takeoff 0."""
        if int(dron) not in (1, 2):
            raise ValueError("dron debe ser 1 o 2")
        self.ventanas_sin_rastreo.append((int(dron), float(t1), float(t2)))
        self.events.append((round(self.t, 2), f"rastreo del dron {int(dron)} perdido de {float(t1):.2f} s a "
                            f"{float(t2):.2f} s"))

    def _actualizar_rastreo(self):
        """Como el inicio de qd2_mission_step: rastreo válido -> pose vista y t_trk; en 0-2 t_trk siempre."""
        t = self.t
        for k, d in self.drones.items():
            if self.solo and k == self.da:
                d.tracked = 0
            else:
                d.tracked = 0 if any(dr == k and t1 <= t < t2 for dr, t1, t2 in self.ventanas_sin_rastreo) else 1
            if d.tracked:
                self.vista[k] = d.p.copy()
            if d.tracked or self.state <= P.ASSIGN_ROLES:
                self.t_trk[k] = t
            if self.vista[k][2] <= Z_ATERRIZADO:
                if self.t_suelo_v[k] < 0:
                    self.t_suelo_v[k] = t
            else:
                self.t_suelo_v[k] = -1.0

    def _visto_en_suelo(self, k):
        return self.t_suelo_v[k] >= 0 and self.t - self.t_suelo_v[k] >= T_ATERRIZADO

    def _ar(self):
        """Disparos por DRON en los estados de vuelo (``ar`` de qd2_mission_step): en solo, solo el dron ds."""
        return {k: (k == self.ds) if self.solo else True for k in self.drones}

    def _dron_R(self):
        """Dron del rol R de qd2_mission_step: en solo explorador es el ausente."""
        return self.da if self.solo else self.R

    def _perdido(self):
        """Algún dron (en solo: el dron ds) sin rastreo válido desde hace > t_perdida_trk."""
        if self.solo:
            return self.t - self.t_trk[self.ds] > T_PERDIDA_TRK
        return any(self.t - self.t_trk[k] > T_PERDIDA_TRK for k in (1, 2))

    def _hay_despegado(self):
        zE = self.vista[self.E][2] if self.E else 0.0
        if self.solo:
            return zE > Z_DESPEGADO
        R = self._dron_R()
        zR = self.vista[R][2] if R else 0.0
        return max(zE, zR) > Z_DESPEGADO

    def _perdida_rastreo(self):
        """Bloques "Pérdida de rastreo sostenida: aterrizar" y "Pérdida de rastreo" de qd2_mission_step."""
        t = self.t
        Dm = self.d_min_h + self.guard_margin
        sin_trk = {k: t - self.t_trk[k] > T_PERDIDA_TRK for k in (1, 2)}
        t_at = max(self.t_perdida_aterrizar, T_PERDIDA_TRK)
        perd_at = {k: 2 if ((not self.solo or k == self.ds) and t - self.t_trk[k] > t_at and self.falla[k] == 0)
                   else 0 for k in (1, 2)}
        if any(perd_at.values()):
            est = self.state
            if self.falla_fase > 0:
                # reacción en curso: si el que se pierde es el SANO, aterriza ya en su sitio; al dron en falla se
                # le baja takeoff salvo que esté ENCIMA del sano y cerca (espera en paso_falla)
                h = self.falla_sano
                if h in (1, 2) and perd_at[h]:
                    self._marcar_falla(perd_at)
                    if self.falla_fase < 3:
                        f = 3 - h
                        self.tk[h] = 0
                        r_sep = Dm + 0.05 + D_DERIVA_SIN_RASTREO
                        Ph, Pf = self.vista[h], self.vista[f]
                        encima = Pf[2] > Ph[2] and hdist(Pf, Ph) < r_sep
                        if not encima:
                            self.tk[f] = 0
                        self.falla_land_xy = Ph[:2].copy()
                        self.falla_usa_via = False
                        self.falla_fase = 3
                        self.t_falla_fase = t
                        self.t_falla_suelo = -1.0
                        self._goto(P.LANDING)
            else:
                despegado = self._hay_despegado()
                # en el aire (PAUSED durante la rampa de despegue sin nadie despegado NO cuenta)
                en_aire = (est in (P.HOVER_READY, P.MISSION, P.RETURN_HOME, P.ABORT)
                           or (est == P.PAUSED and (self.prev_state != P.TAKEOFF or despegado))
                           or (est == P.TAKEOFF and self.takeoff_dado and despegado))
                if est in (P.TAKEOFF, P.PAUSED) and self.takeoff_dado and not en_aire:
                    # rampa de despegue sin nadie en el aire: nadie despega
                    self._cancelar_despegue_rastreo()
                    self._marcar_falla(perd_at)
                    self.pausa_por_trk = self.pausa_por_hb = False
                elif en_aire or (est == P.LANDING and self.land_matlab and self.land_paso == 1
                                 and not self.land_r_baja):
                    # en vuelo (también desde PAUSED): el perdido aterriza; en dual el sano se aparta y aterriza
                    self.pausa_por_trk = self.pausa_por_hb = False
                    self._iniciar_falla(perd_at, sin_trk)
                elif est == P.LANDING:
                    self._marcar_falla(perd_at)     # sigue aterrizando; takeoff 0 al perdido
        # PAUSED por rastreo (con el valor por defecto el bloque de arriba actúa antes y no ocurre)
        if self._perdido() and self.falla_fase == 0:
            est = self.state
            if est == P.TAKEOFF and not self.takeoff_dado:
                for d in self.drones.values():
                    d.flying, d.fsm = False, FSM_IDLE
                    d.sp = np.array([d.p[0], d.p[1], 0.0])
                self.tk = {1: 0, 2: 0}
                self.track_t = 0.0
                self.events.append((round(self.t, 2), "pérdida de rastreo antes de dar takeoff: se desarma"))
                self._goto(P.WAIT_TRACKING)
            elif est in (P.TAKEOFF, P.HOVER_READY, P.MISSION, P.RETURN_HOME, P.ABORT):
                self.prev_state = est
                self._pausar_vista()
                self.pausa_por_hb = False
                self.pausa_por_trk = True
                self.alerta = 2
                self.events.append((round(self.t, 2), "pérdida de rastreo: PAUSED (alerta 2)"))
                self._goto(P.PAUSED)

    def _pausar_vista(self):
        """``pausar`` de qd2_mission_step: hover en la última pose VISTA, a la altura de su consigna."""
        ar = self._ar()
        for k, d in self.drones.items():
            if not ar[k]:
                continue
            z = d.sp[2] if d.sp[2] > 0 else (self.z_E if k == self.E else self.z_R)
            d.sp = np.array([self.vista[k][0], self.vista[k][1], z])

    def _marcar_falla(self, nueva):
        """``marcar_falla``: recuerda la falla por dron; alerta 9 > 10 > 2."""
        for k in (1, 2):
            if nueva[k] > self.falla[k]:
                self.falla[k] = nueva[k]
                if nueva[k] == 2:
                    self.events.append((round(self.t, 2), f"dron {k}: pérdida de rastreo sostenida -> aterriza "
                                        "(alerta 2, nunca más takeoff)"))
        vals = set(self.falla.values())
        self.alerta = 9 if 9 in vals else (10 if 10 in vals else 2)

    def _cancelar_despegue_rastreo(self):
        """``cancelar_despegue`` con takeoff ya dado: takeoff 0 a los dos en su sitio (LANDING paso 4)."""
        self.takeoff_dado = False
        self.tk = {1: 0, 2: 0}
        E, R = self.E, self._dron_R()
        self.land_matlab = True
        self.land_paso = 4
        self.t_suelo = [-1.0, -1.0]
        self.t_land_paso = self.t
        self.land_xy = np.array([self.vista[E][:2], self.vista[R][:2]], dtype=float)
        self.land_via_E = self.vista[E][:2].copy()
        self.land_usa_via_E = False
        self._goto(P.LANDING)

    def _iniciar_falla(self, nueva, sin_trk):
        """``iniciar_falla``: solo explorador (o los dos en falla) -> aterrizaje; dual -> reacción (fase 1)."""
        self._marcar_falla(nueva)
        t = self.t
        if self.solo or (self.falla[1] and self.falla[2]):
            self._planear_aterrizaje()
            self._goto(P.LANDING)
            return
        f = 1 if self.falla[1] else 2
        h = 3 - f
        self.falla_sano = h
        self.falla_fase = 1
        self.falla_land = False
        self.t_falla_fase = t
        self.t_falla_llego = -1.0
        self.t_falla_suelo = -1.0
        zf = float(self.drones[f].sp[2])
        if zf <= 0:
            zf = float(self.vista[f][2])
        self.falla_hold = np.array([self.vista[f][0], self.vista[f][1], zf])
        self.falla_z = self.z_E if h == self.E else self.z_R
        hh = self.drones[h].home
        self.falla_trk = int(self.falla[f] == 2 or sin_trk[f])
        r_sep = self.d_min_h + self.guard_margin + 0.05 + self.falla_trk * D_DERIVA_SIN_RASTREO
        self.falla_sep = self._punto_separacion(self.vista[h], self.vista[f], hh, r_sep)
        if self.falla_trk:
            # sin pose no sostiene el hover: takeoff 0 YA, salvo que esté ENCIMA del sano y cerca
            encima = self.vista[f][2] > self.vista[h][2] and hdist(self.vista[f], self.vista[h]) < r_sep
            if not encima:
                self.tk[f] = 0
        self.events.append((round(t, 2), f"reacción: el dron {h} se aparta del dron {f}"))
        self._goto(P.ABORT)

    def _punto_separacion(self, ph, pf, hh, r):
        """``punto_separacion``: punto a >= r (horizontal) de pf, lo más cerca de ph, dentro de la arena."""
        lo, hi = self.xy_lo, self.xy_hi
        ph = np.array(ph[:2], dtype=float)
        pf = np.array(pf[:2], dtype=float)
        hh = np.array(hh[:2], dtype=float)
        c_ok = np.clip(ph, lo, hi)
        if hdist(c_ok, pf) >= r:
            return c_ok
        mejor = math.inf
        d = hdist(ph, pf)
        for k in range(33):
            if k == 0:
                if d < 1e-3:
                    continue
                u = (ph - pf) / d                 # dirección radial: alejarse en recta
            else:
                ang = 2 * math.pi * (k - 1) / 32
                u = np.array([math.cos(ang), math.sin(ang)])
            c = np.clip(pf + r * u, lo, hi)
            if hdist(c, pf) >= r - 1e-6:
                costo = hdist(c, ph) + 0.05 * hdist(c, hh)
                if costo < mejor:
                    mejor, c_ok = costo, c
        if not math.isfinite(mejor):
            esquinas = [np.array(e, dtype=float) for e in ((lo, lo), (lo, hi), (hi, lo), (hi, hi))]
            c_ok = max(esquinas, key=lambda e: hdist(e, pf))
        return c_ok

    def _ruta_desvio(self, A, B, O, r):
        """``qd2_ruta_desvio``: (via, n_via, ok) para ir de A a B a >= r del obstáculo puntual O."""
        a, b, o = (np.array(v[:2], dtype=float) for v in (A, B, O))
        via, n_via, ok = b.copy(), 0, True
        dA, dB = hdist(a, o), hdist(b, o)
        if _seg_dist(o, a, b) >= min(r, dA, dB) - 1e-9:
            return via, n_via, ok
        mejor, ok = math.inf, False
        for rr in (r + 0.10, r + 0.30, r + 0.50):
            for k in range(24):
                ang = 2 * math.pi * k / 24
                c = np.clip(o + rr * np.array([math.cos(ang), math.sin(ang)]), self.xy_lo, self.xy_hi)
                if hdist(c, o) < r - 1e-9:
                    continue
                if (_seg_dist(o, a, c) >= min(r, dA) - 1e-9 and _seg_dist(o, c, b) >= min(r, dB) - 1e-9):
                    L = hdist(c, a) + hdist(b, c)
                    if L < mejor:
                        mejor, via, ok = L, c, True
        return via, (1 if ok else 0), ok

    def _paso_falla(self):
        """``paso_falla``: 1 apartarse, 2 regresar (o quedarse), 3 aterriza el sano, 4 espera al otro -> DONE."""
        t = self.t
        h = self.falla_sano
        f = 3 - h
        ph, pf = self.vista[h], self.vista[f]
        r = self.d_min_h + self.guard_margin + 0.05 + self.falla_trk * D_DERIVA_SIN_RASTREO
        sp = {f: np.array(self.falla_hold, dtype=float), h: None}
        espera_f = self.falla_fase >= 3 and self.tk[f]
        if espera_f and (self.falla_fase >= 4 or ph[2] <= Z_ATERRIZADO or t - self.t_falla_fase >= T_FALLA_SEP_MAX):
            self.tk[f] = 0
            espera_f = False
        if (self.falla_fase >= 2 and not espera_f) or (self.falla_trk and not self.tk[f]):
            sp[f][2] = 0.0                       # (sin rastreo: ya aterriza en su sitio)
        hh = self.drones[h].home
        separado = hdist(ph, pf) >= self.d_min_h
        if self.falla_fase == 1:
            if hdist(self.falla_sep, pf) < r - 0.05:
                self.falla_sep = self._punto_separacion(ph, pf, hh, r)
            sp[h] = np.array([self.falla_sep[0], self.falla_sep[1], self.falla_z])
            llego = hdist(ph, self.falla_sep) <= 1.5 * self.tol_xy
            if (separado and llego) or t - self.t_falla_fase >= T_FALLA_SEP_MAX:
                self.tk[f] = 0
                sp[f][2] = 0.0
                if self.falla_land and separado:
                    land = ph[:2]
                elif hdist(hh, pf) >= r:
                    land = hh
                else:
                    land = ph[:2]
                self.falla_land_xy = np.array(land[:2], dtype=float)
                via, nv, ok = self._ruta_desvio(ph, self.falla_land_xy, pf, r)
                self.falla_via = np.array(via, dtype=float)
                self.falla_usa_via = bool(nv > 0 and ok)
                self.falla_fase = 2
                self.t_falla_fase = t
                self.t_falla_llego = -1.0
                if self.falla_land and separado:
                    self.falla_usa_via = False
                    self.falla_fase = 3
                    self._goto(P.LANDING)
        if self.falla_fase == 2:
            if self.falla_land and separado:
                self.falla_land_xy = ph[:2].copy()
                self.falla_usa_via = False
                self.falla_fase = 3
                self.t_falla_fase = t
                self._goto(P.LANDING)
            else:
                if self.falla_usa_via:
                    sp[h] = np.array([self.falla_via[0], self.falla_via[1], self.falla_z])
                    if hdist(ph, self.falla_via) <= 1.5 * self.tol_xy:
                        self.falla_usa_via = False
                else:
                    sp[h] = np.array([self.falla_land_xy[0], self.falla_land_xy[1], self.falla_z])
                en_punto = not self.falla_usa_via and hdist(ph, self.falla_land_xy) <= self.tol_xy
                if en_punto:
                    if self.t_falla_llego < 0:
                        self.t_falla_llego = t
                else:
                    self.t_falla_llego = -1.0
                if (en_punto and t - self.t_falla_llego >= T_ASENTAR) or t - self.t_falla_fase >= 2 * T_LAND_MAX:
                    self.falla_fase = 3
                    self.t_falla_fase = t
                    self._goto(P.LANDING)
        if self.falla_fase == 3:
            self.tk[h] = 0                       # autoland del sano
            sp[h] = np.array([self.falla_land_xy[0], self.falla_land_xy[1], 0.0])
            if ph[2] <= Z_ATERRIZADO:
                if self.t_falla_suelo < 0:
                    self.t_falla_suelo = t
            else:
                self.t_falla_suelo = -1.0
            if ((self.t_falla_suelo >= 0 and t - self.t_falla_suelo >= T_ATERRIZADO)
                    or t - self.t_falla_fase >= T_LAND_MAX):
                self.falla_fase = 4
                self.t_falla_fase = t
                self.t_falla_suelo = -1.0
        elif self.falla_fase == 4:
            sp[h] = np.array([self.falla_land_xy[0], self.falla_land_xy[1], 0.0])
            if pf[2] <= Z_ATERRIZADO:
                if self.t_falla_suelo < 0:
                    self.t_falla_suelo = t
            else:
                self.t_falla_suelo = -1.0
            if ((self.t_falla_suelo >= 0 and t - self.t_falla_suelo >= T_ATERRIZADO)
                    or t - self.t_falla_fase >= T_LAND_MAX):
                self.tk = {1: 0, 2: 0}
                self.falla_fase = 0
                self._goto(P.DONE)
        if self.falla_fase in (1, 2):
            nuevo = P.LANDING if self.falla_land else P.ABORT   # LAND_ALL aceptado: la UI lo ve como LANDING
            if nuevo != self.state:
                self.events.append((round(t, 2), f"{P.MISSION_STATES[self.state]} -> {P.MISSION_STATES[nuevo]}"))
                self.state = nuevo
        for k in (h, f):
            if sp[k] is not None:
                xy = np.clip(sp[k][:2], self.xy_lo, self.xy_hi)
                self.drones[k].sp = np.array([xy[0], xy[1], sp[k][2]])

    def _planear_aterrizaje(self):
        """``planear_aterrizaje``: R en su sitio; E en su sitio si queda a >= Dm + 0.05 del punto de R."""
        t = self.t
        E, R = self.E, self._dron_R()
        pE, pR = self.vista[E], self.vista[R]
        hE = self.drones[E].home
        r = self.d_min_h + self.guard_margin + 0.05
        self.land_matlab = True
        self.land_paso = 1
        self.t_land_paso = t
        self.t_suelo = [-1.0, -1.0]
        self.land_r_baja = False
        if self.solo:
            self.land_r_baja = True
            self.land_xy = np.array([pE[:2], pR[:2]], dtype=float)
            self.land_via_E = pE[:2].copy()
            self.land_usa_via_E = False
            return
        xyR = np.array(pR[:2], dtype=float)
        if hdist(pE, xyR) >= r:
            xyE = np.array(pE[:2], dtype=float)
        elif hdist(hE, xyR) >= r:
            xyE = np.array(hE[:2], dtype=float)
        else:
            xyE = np.array(pE[:2], dtype=float)
            mejor = math.inf
            for k in range(16):
                ang = 2 * math.pi * k / 16
                c = np.clip(xyR + (r + 0.1) * np.array([math.cos(ang), math.sin(ang)]), self.xy_lo, self.xy_hi)
                if hdist(c, xyR) >= r and hdist(c, pE) < mejor:
                    mejor, xyE = hdist(c, pE), c
        self.land_xy = np.array([xyE, xyR], dtype=float)
        via, nv, ok = self._ruta_desvio(pE, xyE, xyR, r)
        self.land_via_E = np.array(via, dtype=float)
        self.land_usa_via_E = bool(nv > 0 and ok)

    def _paso_aterrizaje(self):
        """``paso_aterrizaje``: 1 baja R (E a su punto a su altura), 2 baja E, 3 listo, 4 los dos a la vez.
        Un dron en falla nunca recibe takeoff; un dron sin rastreo (falla 2) no pone la alerta 4."""
        t = self.t
        E, R = self.E, self._dron_R()
        pE, pR = self.vista[E], self.vista[R]
        xyE, xyR = self.land_xy[0], self.land_xy[1]
        for i, pp in ((0, pE), (1, pR)):
            if pp[2] <= Z_ATERRIZADO:
                if self.t_suelo[i] < 0:
                    self.t_suelo[i] = t
            else:
                self.t_suelo[i] = -1.0
        r_en_suelo = self.solo or (self.t_suelo[1] >= 0 and t - self.t_suelo[1] >= T_ATERRIZADO)
        e_en_suelo = self.t_suelo[0] >= 0 and t - self.t_suelo[0] >= T_ATERRIZADO
        vencido = t - self.t_land_paso >= T_LAND_MAX
        spR = np.array([xyR[0], xyR[1], 0.0])
        fE, fR = self.falla[E] > 0, self.falla[R] > 0
        trkE, trkR = self.falla[E] == 2, self.falla[R] == 2
        if self.land_paso == 4:
            self.tk[E] = self.tk[R] = 0
            spE = np.array([xyE[0], xyE[1], 0.0])
            if (r_en_suelo and e_en_suelo) or vencido:
                if (not r_en_suelo and not trkR) or (not e_en_suelo and not trkE):
                    self.alerta = 4
                self.land_paso = 3
        else:
            if self.land_usa_via_E:
                spE = np.array([self.land_via_E[0], self.land_via_E[1], self.z_E])
                if hdist(pE, self.land_via_E) <= 1.5 * self.tol_xy:
                    self.land_usa_via_E = False
            else:
                spE = np.array([xyE[0], xyE[1], self.z_E])
            if self.land_paso == 1 and not self.land_r_baja:
                if hdist(pE, xyR) >= self.d_min_h or fR or fE:
                    self.land_r_baja = True
            if self.land_paso == 1 and not self.land_r_baja:
                self.tk[R] = 1                   # R espera a su altura (E todavía cerca en horizontal)
                spR = np.array([xyR[0], xyR[1], self.z_R])
            else:
                self.tk[R] = 0
            e_en_punto = (not self.land_usa_via_E and hdist(pE, xyE) <= self.tol_xy) or fE
            if self.land_paso == 1:
                if fE:
                    self.tk[E] = 0               # E en falla: nunca takeoff (baja en su sitio)
                    spE = np.array([pE[0], pE[1], 0.0])
                else:
                    self.tk[E] = 1
                if ((r_en_suelo or vencido) and e_en_punto) or t - self.t_land_paso >= 2 * T_LAND_MAX:
                    if (not r_en_suelo and not trkR) or not e_en_punto:
                        self.alerta = 4
                    self.land_paso = 2
                    self.t_land_paso = t
                    self.land_usa_via_E = False
            if self.land_paso == 2:
                self.tk[E] = 0
                spE = np.array([xyE[0], xyE[1], 0.0])
                if e_en_suelo or t - self.t_land_paso >= T_LAND_MAX:
                    if not e_en_suelo and not trkE:
                        self.alerta = 4
                    self.land_paso = 3
        self.drones[E].sp = spE
        if not self.solo:
            self.drones[R].sp = spR

    def _takeoff_por_estado(self):
        """takeoff por dron fuera de los caminos portados (como ``s.takeoff = ar`` en 4/5/6/9/10) y la planta:
        un dron con takeoff 0 en esos caminos baja en su sitio (autoland a bordo)."""
        if self.falla_fase == 0 and not self.land_matlab:
            st = self.state
            v = self.takeoff_dado if st == P.TAKEOFF else st in (P.HOVER_READY, P.MISSION, P.RETURN_HOME,
                                                                    P.ABORT, P.PAUSED)
            ar = self._ar()
            for k in self.drones:
                self.tk[k] = int(bool(v) and ar[k] and not self.falla[k])
        if self.state == P.DONE:
            self.tk = {1: 0, 2: 0}
        if self.solo:
            self.tk[self.da] = 0
        for k, d in self.drones.items():
            if not (self.falla_fase or self.land_matlab or self.falla[k]):
                continue
            if self.tk[k]:
                if d.flying:
                    d.fsm = FSM_FLIGHT
            else:
                d.sp = np.array([d.sp[0], d.sp[1], 0.0])
                if d.flying:
                    d.fsm = FSM_AUTOLAND
                    if d.p[2] < 0.05:
                        d.flying, d.fsm, d.aterrizo = False, FSM_IDLE, True

    # ------------------------------------------------------------------ objetivos internos
    def add_internal_target(self, cls, x, y):
        """Objetivo que "detecta Simulink" sin que la estación lo publique (como el simulador 3D con su
        detector interno). Toma el siguiente id libre de la tabla (1..32); None si no hay."""
        tid = max(list(self.targets) + list(self.alias), default=0) + 1
        if tid > P.MAX_TARGETS:
            return None
        self.targets[tid] = {"cls": int(cls), "x": float(x), "y": float(y), "conf": 1.0}
        self.queue.append(tid)
        self.events.append((round(self.t, 2), f"objetivo interno {tid} detectado por el mock"))
        return tid

    def _objetivo_rotando(self):
        """y(56:60): un objetivo de la tabla por mensaje, rotando (1 pendiente, 2 asignado, 3 visitado)."""
        ids = sorted(self.targets)
        if not ids:
            return None
        tid = ids[self._rot_obj % len(ids)]
        self._rot_obj = (self._rot_obj + 1) % len(ids)
        tg = self.targets[tid]
        est = 3 if tid in self.visited else (2 if tid == self.current else 1)
        return {"id": tid, "cls": tg["cls"], "x": tg["x"], "y": tg["y"], "estado": est}

    # ------------------------------------------------------------------ salida
    def telemetry(self):
        dE = self.drones.get(self.E)
        dR = self.drones.get(self.R)
        if self.solo:
            dh = dv = 0.0          # sin respondedor: y(12) = y(13) = 0 (como qd2_telemetry_pack)
        else:
            dh = hdist(dE.p, dR.p) if dE is not None else hdist(self.drones[1].p, self.drones[2].p)
            dv = (dE.p[2] - dR.p[2]) if dE is not None else self.drones[1].p[2] - self.drones[2].p[2]
        mask = 0
        for k in self.visited:
            mask |= 1 << (k - 1)
        for a, k in self.alias.items():      # mascara_visitados: también el id alias de un visitado
            if k in self.visited:
                mask |= 1 << (a - 1)
        tel = {
            "t": self.t, "mission_state": self.state, "version": self.version,
            "explorer_id": self.E, "responder_id": self.R,
            "explorer_wp_idx": (self.wp_idx + 1 if self.E and not (self.solo and self.state < P.ASSIGN_ROLES)
                                else 0),
            "explorer_wp_total": len(self.wps),
            "responder_target_id": self.current, "n_targets_known": len(self.targets),
            "n_targets_visited": len(self.visited), "guard_active": int(self.guard),
            "dist_horizontal": dh, "dist_vertical": dv, "explorer_done": int(self.explorer_done),
            "visited_mask": mask, "marking_progress": min(1.0, self.mark_t / self.t_mark) if self.t_mark > 0 else 0.0,
            "queue_length": len(self.queue), "cmd_seq_ack": self.cmd_seq_ack, "drones": {},
            "alerta": self.alerta if self.alerta_forzada is None else int(self.alerta_forzada),
            "objetivo_sim": self._objetivo_rotando(),
        }
        if self.actitud_en_tel and dE is not None and dE.tracked:
            tel["explorer_roll"], tel["explorer_pitch"] = actitud_de_aceleracion(dE.a, dE.yaw)
            tel["explorer_att_ok"] = 1
        for k, d in self.drones.items():
            fsm = d.fsm if self.version == 2 else flight_state_tierra(d.fsm, d.aterrizo)
            tel["drones"][k] = {
                "tracked": d.tracked, "fsm_state": fsm, "x": d.p[0], "y": d.p[1], "z": d.p[2], "yaw": d.yaw,
                "xd": d.sp[0], "yd": d.sp[1], "zd": d.sp[2], "yawd": d.yawd,
                "vx": d.v[0], "vy": d.v[1], "vz": d.v[2], "battery_V": d.battery,
                "home_x": d.home[0], "home_y": d.home[1], "role": d.role, "comm_ok": 1,
            }
            if not d.tracked:
                # OptiTrack no lo ve: pose NaN en la trama (como Motive/QUARC con el rigid body perdido)
                tel["drones"][k].update(x=None, y=None, z=None, yaw=None)
        if self.solo:
            tel["dron_solo"] = self.ds                     # y(64)
            # dron ausente: sin rastreo (pose NaN en la trama), sin rol, sin enlace, sin consigna
            blk = tel["drones"][self.da]
            for k in ("x", "y", "z", "yaw", "xd", "yd", "zd", "yawd", "vx", "vy", "vz", "battery_V",
                      "home_x", "home_y"):
                blk[k] = None
            blk.update(tracked=0, role=P.ROLE_NONE, comm_ok=0)
        return tel


class MockServer:
    """Stream Server TCP que corre ``MockMission`` en tiempo real (50 Hz) y atiende 1 cliente."""

    def __init__(self, cfg, host="127.0.0.1", port=0, rate_hz=50.0, speed=1.0, cfg_fn=None):
        self.mission = MockMission(cfg)
        self.cfg_fn = cfg_fn       # opcional: devuelve la config actual (reset() la vuelve a leer: modo, pista)
        self.host = host
        self.rate = float(rate_hz)
        self.speed = float(speed)
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((host, int(port)))
        self._srv.listen(1)
        self.port = self._srv.getsockname()[1]
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.clients = 0
        self._drop = False
        self._thread = threading.Thread(target=self._run, daemon=True, name="mock-simulink")

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        try:
            self._srv.close()
        except OSError:
            pass
        self._thread.join(timeout=2)

    def reset(self):
        """Como reiniciar el modelo de Simulink: misión nueva (t = 0, tabla vacía) y se cierra la conexión
        (el Stream Server real se vuelve a abrir); los comandos que venían en camino se descartan. Con
        ``cfg_fn`` se vuelve a leer la config (como Simulink al recompilar: modo de misión y pista)."""
        cfg = self.mission.cfg
        if self.cfg_fn is not None:
            try:
                cfg = self.cfg_fn() or cfg
            except Exception:  # pragma: no cover - una config ilegible deja la anterior
                pass
        with self._lock:
            forzada = self.mission.alerta_forzada
            self.mission = MockMission(cfg)
            self.mission.alerta_forzada = forzada
            self._drop = True

    def forzar_alerta(self, code):
        """y(55) = ``code`` (0..10) hasta nueva orden; None vuelve a la alerta real del mock."""
        if code is not None and not (0 <= int(code) <= P.ALERTA_MAX):
            raise ValueError(f"la alerta debe estar entre 0 y {P.ALERTA_MAX}")
        with self._lock:
            self.mission.alerta_forzada = None if code is None else int(code)

    def estado_pruebas(self):
        """Lo que la tarjeta "Pruebas con el mock" de la UI debe reflejar (alerta forzada actual)."""
        with self._lock:
            return {"alerta_forzada": self.mission.alerta_forzada}

    def perder_rastreo(self, dron, duracion_s=math.inf, desde_s=0.0):
        """OptiTrack deja de ver al dron ``dron`` desde ``desde_s`` s a partir de ahora durante ``duracion_s`` s
        (``MockMission.perder_rastreo`` con el tiempo de la misión actual)."""
        with self._lock:
            t1 = self.mission.t + float(desde_s)
            self.mission.perder_rastreo(dron, t1, t1 + float(duracion_s))

    def objetivo_interno(self, cls, x, y):
        """Objetivo que 'detecta Simulink' (sale en y(56:60) aunque la estación no lo haya publicado)."""
        with self._lock:
            return self.mission.add_internal_target(cls, x, y)

    def _run(self):
        self._srv.setblocking(False)
        conn = None
        buf = bytearray()
        period = 1.0 / self.rate
        next_t = time.time()
        while not self._stop.is_set():
            if self._drop:
                self._drop = False
                if conn is not None:
                    try:
                        conn.close()
                    except OSError:
                        pass
                    conn = None
                buf = bytearray()
            if conn is None:
                try:
                    conn, _ = self._srv.accept()
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    conn.setblocking(False)
                    self.clients += 1
                    buf = bytearray()
                except (socket.timeout, OSError):
                    pass
            # la simulación sigue aunque no haya cliente (como Simulink)
            now = time.time()
            if now < next_t:
                wait = next_t - now
                if conn is not None:
                    try:
                        r, _, _ = select.select([conn], [], [], wait)
                    except (OSError, ValueError):
                        r = []
                    if r:
                        try:
                            chunk = conn.recv(65536)
                        except OSError:
                            chunk = b""
                        if not chunk:
                            conn.close()
                            conn = None
                            continue
                        buf += chunk
                        n = len(buf) // P.COMMAND_BYTES
                        with self._lock:
                            for k in range(n):
                                self.mission.apply_command_message(
                                    P.unpack_command(bytes(buf[k * P.COMMAND_BYTES:(k + 1) * P.COMMAND_BYTES])))
                        del buf[:n * P.COMMAND_BYTES]
                    continue
                elif conn is None:
                    time.sleep(min(wait, 0.02))
                    continue
            next_t += period
            if next_t < time.time() - 0.5:
                next_t = time.time() + period
            with self._lock:
                self.mission.step(period * self.speed)
                data = P.pack_telemetry(self.mission.telemetry())
            if conn is not None:
                try:
                    conn.setblocking(True)
                    conn.settimeout(0.5)
                    conn.sendall(data)
                    conn.setblocking(False)
                except OSError:
                    try:
                        conn.close()
                    except OSError:
                        pass
                    conn = None


def main(argv=None):
    from .config import Config
    ap = argparse.ArgumentParser(description="Servidor :18700 de prueba (emula al Simulink de tierra)")
    ap.add_argument("--puerto", type=int, default=18700)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--velocidad", type=float, default=1.0, help="factor de tiempo (2 = el doble de rápido)")
    ap.add_argument("--alerta", type=int, choices=range(0, P.ALERTA_MAX + 1), metavar="0..10",
                    help="fuerza y(55) = este código de alerta (para probar la UI)")
    ap.add_argument("--objetivo-interno", nargs=3, type=float, action="append", default=[],
                    metavar=("X", "Y", "CLS"), help="objetivo que 'detecta Simulink' (sale en y(56:60)); se repite")
    ap.add_argument("--rastreo", nargs=3, type=float, action="append", default=[], metavar=("DRON", "T1", "T2"),
                    help="OptiTrack no ve al DRON (1 o 2) de T1 a T2 s de misión (tracked = 0, pose NaN); se repite")
    a = ap.parse_args(argv)
    cfg = Config()
    srv = MockServer(cfg.data, a.host, a.puerto, speed=a.velocidad, cfg_fn=cfg.reload_if_changed).start()
    with srv._lock:
        srv.mission.alerta_forzada = a.alerta
        for x, y, c in a.objetivo_interno:
            srv.mission.add_internal_target(int(c), x, y)
        for dron, t1, t2 in a.rastreo:
            srv.mission.perder_rastreo(int(dron), t1, t2)
    print(f"Mock de Simulink escuchando en tcpip://{a.host}:{srv.port}  (Ctrl+C para salir)")
    try:
        while True:
            time.sleep(1)
            m = srv.mission
            print(f"t={m.t:6.1f}  {P.MISSION_STATES[m.state]:<13} wp={m.wp_idx + 1}/{len(m.wps)}  "
                  f"objetivos={len(m.targets)} visitados={len(m.visited)}", flush=True)
    except KeyboardInterrupt:
        srv.stop()


if __name__ == "__main__":
    main()
