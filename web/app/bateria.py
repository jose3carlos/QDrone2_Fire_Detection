"""Batería de cada dron (``battery_V``, índice o+14 de la telemetría) para la UI y el registro de vuelo.

SOLO VISUALIZACIÓN Y REGISTRO. Esta estación NO actúa por la batería: no manda comandos, no levanta alertas, no
cambia la misión ni bloquea nada. Los únicos límites que existen viven fuera de aquí y NO se tocan:

* A bordo, en TODOS los modelos del dron (Equipo2 PID y los MPC): "Check Battery level", V <= 14.0 V durante
  >= 0.3 s (bloque Threshold de QUARC) enclava low_battery; en vuelo (FSM 4/5) pasa a AUTOLAND (6) y en
  IDLE/ARMADO/DESPEGUE da Error (no arma). Es automático y es del dron, no de la estación.
* ``docs/CHECKLIST_LABORATORIO.md``: no despegar con menos de ~15.5 V. Es una RECOMENDACIÓN para las personas, no
  hay nada automático detrás.

``V_ATERRIZAJE_A_BORDO`` y ``V_RECOMENDADA_DESPEGUE`` solo se usan para DIBUJAR esas dos líneas de referencia y para
la estimación informativa "≈ N min hasta 14.0 V a este ritmo". (14.8 V es el voltaje NOMINAL de una LiPo 4S,
3.7 V/celda; no es un corte.)

Sin dato: el modelo de tierra PID pone battery_V = NaN porque el dron PID del Equipo2 solo devuelve el eco del
timestamp. En esos vuelos la UI lo dice y no dibuja nada; la batería se revisa después del vuelo con los logs a
bordo (``simulink_qdrone2/bateria/qd2_bateria.m``). En MPC sí llega en vivo; el mock la simula.

Muestreo: la telemetría (~50 Hz) se promedia por segundo de reloj de la PC (media de 1 s = el suavizado). Cada
muestra cerrada va a la historia de la UI (``VENTANA_S``) y al registro de vuelo (``RegistroVuelos.bateria``).
Solo cuentan los valores finitos: un NaN no es "0 V" ni rompe la media.
"""
from __future__ import annotations

import collections
import math
import threading

V_ATERRIZAJE_A_BORDO = 14.0    # V: corte A BORDO (Threshold de QUARC, >= 0.3 s) -> AUTOLAND. Solo referencia
T_ATERRIZAJE_A_BORDO = 0.3     # s que debe durar V <= 14.0 a bordo
V_RECOMENDADA_DESPEGUE = 15.5  # V: mínimo recomendado para despegar (checklist, no automático)
VENTANA_S = 300                # s de historia que se muestran en la UI (últimos 5 min)
PASO_UI_S = 2                  # s entre muestras que viajan a la UI (en el registro van todas, ~1 Hz)
Z_AIRE = 0.15                  # m: sobre esto el dron está en el aire ("caída desde el despegue" y estimación)
AIRE_MIN_S = 20.0              # s en el aire con dato antes de estimar la pendiente
TRANSITORIO_S = 5.0            # s después del despegue que no entran a la pendiente (la caída por carga es inmediata)
VENTANA_PENDIENTE_S = 60.0     # s de vuelo más recientes con los que se ajusta la recta
FSM_MPC_AIRE = (4, 5, 6)       # FSM a bordo (MPC) en el aire, si no hay z (sin rastreo)
FSM_REPOSO = {2: (0, 1, 9), 1: (0, 3, 9)}   # motores apagados: MPC = FSM a bordo; PID = Flight State de tierra
T_REF_DESPEGUE_S = 60.0        # s: el dato en reposo / en tierra de referencia debe ser así de reciente al despegar


def _finito(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def estado_texto(version, fsm, aire):
    """Texto corto del estado de carga de la batería según la FSM (MPC: a bordo; PID: Flight State de tierra).
    ``aire``: True / False con el z rastreado (o la serie); None si no se sabe (PID sin rastreo)."""
    if version == 2:
        return {0: "sin respuesta del dron", 1: "en reposo (IDLE)", 2: "armado: motores en ralentí",
                3: "bajo carga: despegando", 4: "bajo carga: subiendo", 5: "bajo carga: en vuelo",
                6: "bajo carga: aterrizando (AUTOLAND)", 7: "emergencia a bordo", 8: "emergencia a bordo",
                9: "en reposo (motores deshabilitados)"}.get(fsm, f"FSM {fsm}")
    if aire:
        return "bajo carga: en el aire"
    if fsm == 1:
        return "armado: motores girando"
    if fsm == 2:
        return "bajo carga: despegando" if aire is False else "bajo carga: despegue / en vuelo"
    if fsm == 3 and aire is None:
        return "aterrizaje (Land): bajando o ya en tierra"
    return "en reposo (en tierra)"


class _Dron:
    def __init__(self):
        self.k = None                  # segundo (int(stamp)) que se está promediando
        self.sv = self.nv = 0.0        # suma y número de voltajes finitos del segundo
        self.sz = self.nz = 0.0
        self.fsm = 0
        self.estado = None
        self.version = 0
        self.serie = collections.deque(maxlen=VENTANA_S + 5)   # (t, V, aire) a ~1 Hz
        self.v_despegue = None         # referencia del último despegue: V en reposo (o en tierra) justo antes
        self.ref_despegue = None       # "reposo" (motores apagados) o "tierra" (p. ej. ya armado)
        self.t_despegue = None
        self.ultimo_tierra = None      # (t, V) del último segundo en tierra
        self.ultimo_reposo = None      # (t, V) del último segundo en tierra con los motores apagados


class HistorialBateria:
    """Promedia la batería de los dos drones por segundo y guarda los últimos ``VENTANA_S`` s. Thread-safe:
    ``agregar`` lo llama el hilo del puente y ``estado`` el del WebSocket."""

    def __init__(self):
        self._lock = threading.Lock()
        self._d = {1: _Dron(), 2: _Dron()}

    def reiniciar(self):
        """Olvida todo (cambio de fuente, mock reiniciado): la historia de otra fuente no es de esta batería."""
        with self._lock:
            self._d = {1: _Dron(), 2: _Dron()}

    # -------------------------------------------------------------- entrada
    def agregar(self, tel, stamp):
        """Una telemetría. Devuelve las muestras de 1 s que se cerraron con ella (solo con dato finito):
        ``{"t", "dron", "V", "fsm", "estado_mision", "z", "aire"}`` (``t`` = mitad del segundo, reloj de la PC)."""
        out = []
        k = int(math.floor(stamp))
        version = tel.get("version")
        est = tel.get("mission_state")
        with self._lock:
            for dron, d in self._d.items():
                blk = (tel.get("drones") or {}).get(dron) or {}
                if d.k is not None and k != d.k:
                    m = self._cerrar(dron, d)
                    if m is not None:
                        out.append(m)
                if d.k != k:
                    d.k, d.sv, d.nv, d.sz, d.nz = k, 0.0, 0, 0.0, 0
                v, z = blk.get("battery_V"), blk.get("z")
                if _finito(v):
                    d.sv += float(v)
                    d.nv += 1
                if _finito(z) and blk.get("tracked", 1):
                    d.sz += float(z)
                    d.nz += 1
                d.fsm = blk.get("fsm_state") or 0
                d.estado = est
                d.version = version
        return out

    def _cerrar(self, dron, d):
        """(con ``_lock``) Cierra el segundo ``d.k``: muestra a la serie; None si no hubo voltaje finito."""
        if not d.nv:
            return None
        t = d.k + 0.5
        V = d.sv / d.nv
        z = d.sz / d.nz if d.nz else None
        aire = (z > Z_AIRE) if z is not None else (d.version == 2 and d.fsm in FSM_MPC_AIRE)
        prev_aire = d.serie[-1][2] if d.serie else False
        if aire and not prev_aire:
            # despegue: referencia = último dato en reposo (motores apagados) o, si no hay, el último en tierra
            d.t_despegue, d.v_despegue, d.ref_despegue = t, None, None
            for nombre, ref in (("reposo", d.ultimo_reposo), ("tierra", d.ultimo_tierra)):
                if ref is not None and t - ref[0] <= T_REF_DESPEGUE_S:
                    d.v_despegue, d.ref_despegue = ref[1], nombre
                    break
        if not aire:
            d.ultimo_tierra = (t, V)
            if d.fsm in FSM_REPOSO.get(d.version, ()):
                d.ultimo_reposo = (t, V)
        d.serie.append((t, V, aire))
        return {"t": t, "dron": dron, "V": round(V, 3), "fsm": d.fsm, "estado_mision": d.estado,
                "z": None if z is None else round(z, 3), "aire": bool(aire)}

    # -------------------------------------------------------------- salida (UI)
    def estado(self, tel, activos, ahora):
        """Bloque ``bateria`` del snapshot para los drones ``activos`` (modo solo: solo el dron solo)."""
        drones = {}
        with self._lock:
            for dron in activos:
                d = self._d.get(dron)
                if d is None:
                    continue
                blk = ((tel or {}).get("drones") or {}).get(dron) or {}
                v = blk.get("battery_V")
                v = float(v) if _finito(v) else None
                version = (tel or {}).get("version")
                serie = [s for s in d.serie if ahora - s[0] <= VENTANA_S]
                aire_serie = bool(serie and serie[-1][2] and ahora - serie[-1][0] <= 5.0)
                # en el aire AHORA: z rastreado de esta telemetría (en PID la serie está vacía: battery_V es NaN);
                # sin rastreo, la serie reciente; si tampoco hay, no se sabe (None)
                z = blk.get("z")
                if _finito(z) and blk.get("tracked", 1):
                    aire = float(z) > Z_AIRE
                elif serie and ahora - serie[-1][0] <= 5.0:
                    aire = aire_serie
                else:
                    aire = None
                fila = {"v": None if v is None else round(v, 2), "version": version,
                        "estado": estado_texto(version, blk.get("fsm_state") or 0, aire) if tel else "sin telemetría",
                        "en_aire": bool(aire), "v_despegue": None, "ref_despegue": None, "caida": None, "aire_s": None,
                        "pendiente_V_s": None, "min_hasta_14": None, "serie": []}
                if v is None:
                    # sin dato en vivo: nada que dibujar (ni la historia vieja: no es la batería de ahora)
                    fila["sin_dato"] = "pid" if version == 1 else "nan" if tel else "sin_tel"
                    drones[str(dron)] = fila
                    continue
                fila["sin_dato"] = None
                ult = len(serie) - 1             # [s antes de ahora, V, en el aire]; la última siempre va
                fila["serie"] = [[round(s[0] - ahora, 1), round(s[1], 3), int(s[2])] for i, s in enumerate(serie)
                                 if int(s[0]) % PASO_UI_S == 0 or i == ult]
                if d.v_despegue is not None and d.t_despegue is not None and ahora - d.t_despegue <= VENTANA_S * 2:
                    fila["v_despegue"] = round(d.v_despegue, 2)
                    fila["ref_despegue"] = d.ref_despegue
                    fila["caida"] = round(d.v_despegue - v, 2)
                if aire and aire_serie and d.t_despegue is not None:
                    fila.update(self._estimacion(d, serie, ahora))
                drones[str(dron)] = fila
        return {"drones": drones, "ventana_s": VENTANA_S,
                "ref": {"v_aterrizaje_a_bordo": V_ATERRIZAJE_A_BORDO, "t_aterrizaje_a_bordo_s": T_ATERRIZAJE_A_BORDO,
                        "v_recomendada_despegue": V_RECOMENDADA_DESPEGUE}}

    @staticmethod
    def _estimacion(d, serie, ahora):
        """Recta (mínimos cuadrados) del voltaje en vuelo de los últimos ``VENTANA_PENDIENTE_S`` s, sin los primeros
        ``TRANSITORIO_S`` s tras el despegue. Informativa: "≈ N min hasta 14.0 V a este ritmo"."""
        aire = [(t, V) for t, V, a in serie if a and t >= d.t_despegue]
        aire_s = (aire[-1][0] - aire[0][0] + 1.0) if aire else 0.0
        out = {"aire_s": round(aire_s, 1)}
        if aire_s <= AIRE_MIN_S:
            return out
        pts = [(t, V) for t, V in aire if t >= d.t_despegue + TRANSITORIO_S and t >= ahora - VENTANA_PENDIENTE_S]
        if len(pts) < 10:
            return out
        n = len(pts)
        mt = sum(t for t, _ in pts) / n
        mv = sum(V for _, V in pts) / n
        sxx = sum((t - mt) ** 2 for t, _ in pts)
        if sxx <= 0:
            return out
        b = sum((t - mt) * (V - mv) for t, V in pts) / sxx
        out["pendiente_V_s"] = round(b, 5)
        v_fit = mv + b * (pts[-1][0] - mt)               # voltaje de la recta ahora (suavizado)
        if b < -1e-4:
            # 0.0 solo si la recta ya está en 14.0 V o abajo; si no, al menos 0.1 (el redondeo no la lleva a 0)
            m = (v_fit - V_ATERRIZAJE_A_BORDO) / (-b) / 60.0
            out["min_hasta_14"] = 0.0 if m <= 0 else max(0.1, round(m, 1))
        return out
