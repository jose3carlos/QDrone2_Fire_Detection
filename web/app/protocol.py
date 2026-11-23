"""Empaquetado exacto del puente Simulink ⇄ Python (ARQUITECTURA §3.3).

Todos los mensajes son doubles IEEE-754 little-endian de tamaño fijo.
* Telemetría Simulink → Python: 64 doubles (512 bytes) a 50 Hz.
* Comandos Python → Simulink: 32 doubles (256 bytes) a 20 Hz.

En este archivo los índices se escriben **base 1** (igual que en el contrato y en MATLAB) y se
convierten a base 0 solo en ``_i()``, para que se puedan comparar línea por línea con la tabla.
"""
from __future__ import annotations

import math
import struct

N_TELEMETRY = 64
N_COMMAND = 32
TELEMETRY_BYTES = N_TELEMETRY * 8
COMMAND_BYTES = N_COMMAND * 8
_TEL = struct.Struct("<%dd" % N_TELEMETRY)
_CMD = struct.Struct("<%dd" % N_COMMAND)

MAX_TARGETS = 32
N_SLOTS = 3

# Estados de misión (índice 2 de la telemetría)
MISSION_STATES = {
    0: "INIT", 1: "WAIT_TRACKING", 2: "ASSIGN_ROLES", 3: "TAKEOFF", 4: "HOVER_READY",
    5: "MISSION", 6: "RETURN_HOME", 7: "LANDING", 8: "DONE", 9: "ABORT", 10: "PAUSED",
}
MISSION_STATES_ES = {
    0: "Inicio", 1: "Esperando OptiTrack", 2: "Roles asignados", 3: "Despegue", 4: "Hover listo",
    5: "Misión", 6: "Regreso a home", 7: "Aterrizando", 8: "Terminado", 9: "Abortado", 10: "En pausa",
}
INIT, WAIT_TRACKING, ASSIGN_ROLES, TAKEOFF, HOVER_READY, MISSION, RETURN_HOME, LANDING, DONE, ABORT, PAUSED = range(11)

# Comandos (índice 2 del mensaje Python → Simulink)
CMD_NONE, CMD_START, CMD_ABORT, CMD_LAND_ALL, CMD_PAUSE, CMD_RESUME = range(6)
COMMANDS = {"NADA": CMD_NONE, "START": CMD_START, "ABORT": CMD_ABORT, "LAND_ALL": CMD_LAND_ALL,
            "PAUSE": CMD_PAUSE, "RESUME": CMD_RESUME}
COMMAND_NAMES = {v: k for k, v in COMMANDS.items()}

CLASSES = {0: "fuego", 1: "toma"}
CLASS_IDS = {v: k for k, v in CLASSES.items()}

ROLE_NONE, ROLE_EXPLORER, ROLE_RESPONDER = 0, 1, 2

# y(55) = alerta (ARQUITECTURA §3.3). Códigos acordados por los dos equipos; tiene prioridad la del
# operador (5-8) sobre la del supervisor (1-4). ``nivel``: "aviso" (amarillo) o "grave" (rojo).
ALERTAS = {
    0: ("nada", ""),
    1: ("Homes demasiado cerca: START rechazado. Separa los drones y vuelve a dar START.", "aviso"),
    2: ("Pérdida de rastreo (OptiTrack): misión en PAUSA, los dos drones en hover.", "grave"),
    3: ("Timeout de despegue: se canceló o se abortó el despegue.", "grave"),
    4: ("Timeout de aterrizaje: un dron no se vio en el suelo (revisa el pivot del rigid body).", "aviso"),
    5: ("E-STOP del operador: motores cortados en los dos drones.", "grave"),
    6: ("Switch Takeoff/Land abajo (operador): LAND ALL ordenado.", "aviso"),
    7: ("Switch Arm abajo (operador): drones desarmados y LAND ALL.", "grave"),
    8: ("Respaldo del operador: el supervisor no aceptó el LAND ALL (2 s / 90 s); se soltó el takeoff.", "grave"),
    9: ("Falla a bordo de un dron (FSM a bordo 7 u 8, o 9 / enlace perdido debiendo volar).", "grave"),
    10: ("Protocolo MPC incorrecto: un dron no responde con la etiqueta y(8) = 2 o su FSM está fuera de 1..9.",
         "grave"),
}
ALERTA_MAX = 10
ALERTA_NO_FINITA = -1   # y(55) llegó como NaN o inf (no es un código acordado: se muestra como grave)


def alerta_info(code):
    """(texto, nivel) de un código y(55). Un código desconocido se trata como grave."""
    if code in ALERTAS:
        return ALERTAS[code]
    if code == ALERTA_NO_FINITA:
        return ("y(55) no es un número (NaN o inf): telemetría corrupta o TELEMETRY PACK mal cableado. "
                "Revisa el modelo de Simulink.", "grave")
    return (f"Alerta desconocida (código {code}): revisa el modelo de Simulink.", "grave")


# Estado del dron en o+2 (``fsm_state``). Depende de la versión y(3):
# * PID (1): Flight State de TIERRA del Flight Plan del Equipo2 (el dron PID no reporta su FSM).
# * MPC (2): FSM A BORDO reportada por el dron (índice 2 de su respuesta de 8 doubles, §3.2).
FLIGHT_STATE_TIERRA = {0: "Reposo (Idle)", 1: "Armado", 2: "Despegue / en vuelo", 3: "Aterrizaje (Land)",
                       9: "Paro (MS Stop)"}
FSM_A_BORDO = {1: "IDLE", 2: "ARMADO", 3: "DESPEGUE (empuje)", 4: "SUBIENDO", 5: "VUELO", 6: "AUTOLAND",
               7: "EMERGENCIA: rampa (error)", 8: "EMERGENCIA: aborto", 9: "DESHABILITADO"}
FSM_A_BORDO_FALLA = (7, 8)          # siempre son falla
# 9 (DISABLE a bordo / MS Stop de tierra) es el FIN NORMAL de un vuelo (sim/README: 1→2→3→4→5→6→9; el Flight
# Plan PID se queda en 3/9). Solo es falla si el dron debería estar volando: en estos estados de misión
# (como qd2_mission_step: 9 con takeoff = 1, o en TAKEOFF armado) o si tierra ya publicó la alerta 9.
ESTADOS_EN_VUELO = (TAKEOFF, HOVER_READY, MISSION, RETURN_HOME, ABORT, PAUSED)


def fsm_info(version, code, mission_state=None, alerta=0, falla_vista=None):
    """Texto del estado del dron según la versión: (tipo, texto, falla).

    ``mission_state`` y ``alerta`` (y(2), y(55)) dan el contexto para el código 9, que es el fin normal de
    un vuelo y solo es falla si el dron debería estar en el aire (o si tierra MPC publicó la alerta 9).

    ``falla_vista`` (solo MPC): None si la estación no sabe QUÉ dron falló; True/False si sí lo sabe (vio a
    ESTE dron en 7/8, o en 9 debiendo volar, en esta corrida de Simulink). La alerta 9 es global: sin esto,
    el dron SANO que aterriza después de la falla del otro también saldría en rojo con su 9 normal."""
    en_vuelo = mission_state in ESTADOS_EN_VUELO
    if version == 2:
        if code == 0:
            return "FSM a bordo", "sin respuesta del dron todavía", False
        txt = FSM_A_BORDO.get(code, f"fuera de rango ({code})")
        if code == 9:
            falla = en_vuelo or bool(falla_vista) or (alerta == 9 and falla_vista is None)
            return "FSM a bordo", txt + (" (falla)" if falla else " (fin de vuelo)"), falla
        return "FSM a bordo", txt, code in FSM_A_BORDO_FALLA or code not in FSM_A_BORDO
    txt = FLIGHT_STATE_TIERRA.get(code, f"estado {code}")
    if code == 9:
        return "Flight State (tierra)", txt + (" en vuelo: revisa el dron" if en_vuelo else " (fin de vuelo)"), en_vuelo
    return "Flight State (tierra)", txt, False


# y(56:60): un objetivo de la tabla de Simulink por mensaje, rotando: [id clase x y estado]
OBJ_SIM_BASE = 56

# y(61:63): actitud del EXPLORADOR (misma Corrected Pose Data de OptiTrack que x, y, z, yaw en o+3..o+6):
# y(61) = roll (rad), y(62) = pitch (rad), y(63) = 1 si son válidos (actitud medida y explorador rastreado).
# Convención R = Rz(yaw)·Ry(pitch)·Rx(roll) (geo.R_mundo_cuerpo). Modelos viejos mandan 0, 0, 0 -> sin actitud.
# y(64) sigue reservado (0).
EXPLORER_ROLL_IDX, EXPLORER_PITCH_IDX, EXPLORER_ATT_OK_IDX = 61, 62, 63
OBJ_ESTADOS = {1: "pendiente", 2: "asignado", 3: "visitado"}

# Campos globales de la telemetría: nombre -> índice base 1
TEL_FIELDS = {
    "t": 1, "mission_state": 2, "version": 3, "explorer_id": 4, "responder_id": 5,
    "explorer_wp_idx": 6, "explorer_wp_total": 7, "responder_target_id": 8,
    "n_targets_known": 9, "n_targets_visited": 10, "guard_active": 11,
    "dist_horizontal": 12, "dist_vertical": 13, "explorer_done": 14,
    "visited_mask": 51, "marking_progress": 52, "queue_length": 53, "cmd_seq_ack": 54,
}
DRONE_OFFSETS = {1: 14, 2: 32}
# Campos del bloque de dron: nombre -> desplazamiento k (índice = o + k)
DRONE_FIELDS = {
    "tracked": 1, "fsm_state": 2, "x": 3, "y": 4, "z": 5, "yaw": 6,
    "xd": 7, "yd": 8, "zd": 9, "yawd": 10, "vx": 11, "vy": 12, "vz": 13,
    "battery_V": 14, "home_x": 15, "home_y": 16, "role": 17, "comm_ok": 18,
}
_INT_FIELDS = {"mission_state", "version", "explorer_id", "responder_id", "explorer_wp_idx",
               "explorer_wp_total", "responder_target_id", "n_targets_known", "n_targets_visited",
               "guard_active", "explorer_done", "visited_mask", "queue_length"}
# cmd_seq_ack NO se redondea: el OPERATOR COMMAND MERGE de tierra inyecta cmd_seq + 0.5 durante un
# paso (1 ms) tras una petición del operador; redondearlo podría confirmar por error otro cmd_seq.
_INT_DRONE = {"tracked", "fsm_state", "role", "comm_ok"}

# Comandos: nombre -> índice base 1
CMD_FIELDS = {"heartbeat": 1, "cmd": 2, "cmd_seq": 3, "n_targets": 4,
              "z_explorer": 20, "z_responder": 21, "t_mark": 22, "d_min_h": 23}
SLOT_BASE = 5  # ranura s (0..2) ocupa los índices 5+5s .. 9+5s: [id, clase, x, y, confianza]


def _i(idx1):
    return idx1 - 1


def _num(v):
    """Convierte a número JSON (NaN/inf -> None)."""
    v = float(v)
    return None if (math.isnan(v) or math.isinf(v)) else v


def _int(v):
    v = float(v)
    return 0 if (math.isnan(v) or math.isinf(v)) else int(round(v))


# ---------------------------------------------------------------------------- telemetría
def unpack_telemetry(data):
    """512 bytes -> dict con campos globales y ``drones`` {1: {...}, 2: {...}}."""
    if len(data) != TELEMETRY_BYTES:
        raise ValueError(f"La telemetría debe medir {TELEMETRY_BYTES} bytes (llegaron {len(data)})")
    v = _TEL.unpack(data)
    out = {}
    for name, idx in TEL_FIELDS.items():
        out[name] = _int(v[_i(idx)]) if name in _INT_FIELDS else _num(v[_i(idx)])
    ack = v[_i(TEL_FIELDS["cmd_seq_ack"])]
    if not math.isfinite(ack):
        ack = 0.0
    out["cmd_seq_ack"] = int(ack) if float(ack).is_integer() else float(ack)
    out["ack_operador"] = not float(ack).is_integer()   # cmd_seq + 0.5: petición del operador (1 ms)
    out["mission_state_name"] = MISSION_STATES.get(out["mission_state"], "?")
    out["mission_state_es"] = MISSION_STATES_ES.get(out["mission_state"], "?")
    out["version_name"] = {1: "PID", 2: "MPC"}.get(out["version"], "?")
    # y(55): alerta. Un valor no finito NO es "nada": se reporta como ALERTA_NO_FINITA (grave).
    raw_alerta = float(v[_i(55)])
    out["alerta"] = int(round(raw_alerta)) if math.isfinite(raw_alerta) else ALERTA_NO_FINITA
    out["alerta_texto"], out["alerta_nivel"] = alerta_info(out["alerta"])
    drones = {}
    for d, o in DRONE_OFFSETS.items():
        blk = {}
        for name, k in DRONE_FIELDS.items():
            raw = v[_i(o + k)]
            blk[name] = _int(raw) if name in _INT_DRONE else _num(raw)
        blk["fsm_tipo"], blk["fsm_texto"], blk["fsm_falla"] = fsm_info(
            out["version"], blk["fsm_state"], out["mission_state"], out["alerta"])
        drones[d] = blk
    out["drones"] = drones
    out["visited_ids"] = [k for k in range(1, MAX_TARGETS + 1) if (out["visited_mask"] >> (k - 1)) & 1]
    # y(56:60): objetivo de la tabla de Simulink (rotando); None si la ranura está vacía o es inválida
    out["objetivo_sim"] = _objetivo_sim(v[_i(OBJ_SIM_BASE):_i(OBJ_SIM_BASE) + 5])
    # y(61:63): actitud del explorador. Sin y(63) = 1 o con un ángulo no finito: sin actitud (None, False)
    roll, pitch = float(v[_i(EXPLORER_ROLL_IDX)]), float(v[_i(EXPLORER_PITCH_IDX)])
    ok = float(v[_i(EXPLORER_ATT_OK_IDX)])
    att_ok = math.isfinite(ok) and int(round(ok)) == 1 and math.isfinite(roll) and math.isfinite(pitch)
    out["explorer_roll"] = roll if att_ok else None
    out["explorer_pitch"] = pitch if att_ok else None
    out["explorer_att_ok"] = bool(att_ok)
    # y(64): el único reservado que queda
    out["reserved"] = [_num(x) for x in v[_i(64):_i(64) + 1]]
    return out


def _objetivo_sim(vals):
    """[id clase x y estado] -> dict, o None. Solo ids 1..32 (visited_mask), clase 0/1 y estado 1..3."""
    tid, cls, x, y, est = (float(a) for a in vals)
    if not all(math.isfinite(a) for a in (tid, cls, x, y, est)):
        return None
    tid, cls, est = int(round(tid)), int(round(cls)), int(round(est))
    if not (1 <= tid <= MAX_TARGETS) or cls not in CLASSES or est not in OBJ_ESTADOS:
        return None
    return {"id": tid, "cls": cls, "x": x, "y": y, "estado": est}


def pack_telemetry(tel):
    """Inverso de ``unpack_telemetry`` (lo usan el simulador mock y las pruebas).
    Acepta además ``alerta`` (y(55)), ``objetivo_sim`` = dict(id, cls, x, y, estado) (y(56:60)) y
    ``explorer_roll``, ``explorer_pitch``, ``explorer_att_ok`` (y(61:63); sin att_ok verdadero o con un
    ángulo no finito se escribe 0, 0, 0, igual que el TELEMETRY PACK de Simulink)."""
    v = [0.0] * N_TELEMETRY
    for name, idx in TEL_FIELDS.items():
        if name in tel and tel[name] is not None:
            v[_i(idx)] = float(tel[name])
    for d, o in DRONE_OFFSETS.items():
        blk = tel.get("drones", {}).get(d, {})
        for name, k in DRONE_FIELDS.items():
            val = blk.get(name)
            v[_i(o + k)] = float("nan") if val is None else float(val)
    v[_i(55)] = float(tel.get("alerta") or 0)
    ob = tel.get("objetivo_sim")
    if ob:
        for k, key in enumerate(("id", "cls", "x", "y", "estado")):
            v[_i(OBJ_SIM_BASE + k)] = float(ob[key])
    if tel.get("explorer_att_ok"):
        try:
            r, p = float(tel.get("explorer_roll")), float(tel.get("explorer_pitch"))
        except (TypeError, ValueError):
            r = p = float("nan")
        if math.isfinite(r) and math.isfinite(p):
            v[_i(EXPLORER_ROLL_IDX)], v[_i(EXPLORER_PITCH_IDX)], v[_i(EXPLORER_ATT_OK_IDX)] = r, p, 1.0
    return _TEL.pack(*v)


# ---------------------------------------------------------------------------- comandos
def pack_command(heartbeat, cmd, cmd_seq, n_targets, slots, z_explorer=0.0, z_responder=0.0,
                 t_mark=0.0, d_min_h=0.0):
    """Arma los 32 doubles. ``slots``: hasta 3 tuplas ``(id, clase, x, y, conf)``; el resto queda en 0."""
    v = [0.0] * N_COMMAND
    v[_i(1)] = float(heartbeat)
    v[_i(2)] = float(cmd)
    v[_i(3)] = float(cmd_seq)
    v[_i(4)] = float(min(int(n_targets), MAX_TARGETS))
    for s, slot in enumerate(list(slots)[:N_SLOTS]):
        base = SLOT_BASE + 5 * s
        for k in range(5):
            v[_i(base + k)] = float(slot[k])
    v[_i(20)] = float(z_explorer)
    v[_i(21)] = float(z_responder)
    v[_i(22)] = float(t_mark)
    v[_i(23)] = float(d_min_h)
    return _CMD.pack(*v)


def unpack_command(data):
    """256 bytes -> dict (lo usa el simulador mock y las pruebas)."""
    if len(data) != COMMAND_BYTES:
        raise ValueError(f"El comando debe medir {COMMAND_BYTES} bytes (llegaron {len(data)})")
    v = _CMD.unpack(data)
    out = {name: v[_i(idx)] for name, idx in CMD_FIELDS.items()}
    for k in ("heartbeat", "cmd", "cmd_seq", "n_targets"):
        out[k] = int(round(out[k]))
    slots = []
    for s in range(N_SLOTS):
        base = SLOT_BASE + 5 * s
        tid, cls, x, y, conf = (v[_i(base + k)] for k in range(5))
        slots.append({"id": int(round(tid)), "cls": int(round(cls)), "x": x, "y": y, "conf": conf})
    out["slots"] = slots
    return out


class TargetRotator:
    """Reparte todos los objetivos conocidos en las 3 ranuras, rotando en cada mensaje."""

    def __init__(self):
        self.pos = 0

    def next_slots(self, targets):
        """``targets``: lista de tuplas (id, clase, x, y, conf) ordenadas por id."""
        n = len(targets)
        if n == 0:
            return []
        if n <= N_SLOTS:
            self.pos = 0
            return list(targets)
        start = self.pos % n
        out = [targets[(start + k) % n] for k in range(N_SLOTS)]
        self.pos = (start + N_SLOTS) % n
        return out
