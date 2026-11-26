"""Caja negra del vuelo: todo lo que entra y sale de la estación, para reconstruir qué pasó (y repetirlo).

Si algo falla en un vuelo real, este archivo permite ver después, paquete por paquete, lo que vio la estación:
``web/datos/vuelos/AAAA-MM-DD/vuelo_HHMMSS_<fuente>.tel.jsonl.gz`` (fecha/hora local de la PC), junto al registro de
observaciones del mismo vuelo (``registro_vuelo.py``: con la caja abierta el registro usa el MISMO nombre base,
``vuelo_HHMMSS_<fuente>.jsonl``). Se sube a git como el registro. Se analiza con ``web/herramientas/caja_negra.py``
(``resumen`` / ``csv``) y se reproduce en la UI con ``python -m app.server --repetir <archivo>`` (``repeticion.py``).

Formato: JSON Lines dentro de gzip (``FORMATO`` = 1). ``t`` = segundos desde ``t0`` de la cabecera (hora de la PC
al abrir); negativo = antes de abrir (búfer circular de ``pre_s`` s: lo que pasó antes del START).
* ``{"tipo": "cabecera", ...}``: formato, fecha, t0 (epoch), fuente, motivo_apertura, pre_s, decimacion (1 = todos
  los paquetes), hz_nominal, ``campos`` = nombre de cada índice y(1..64) (base 1, = ``protocol.py``), config (ruta),
  config_datos (estacion.json en memoria al abrir), modo de misión, puente, eventos_previos (los últimos de la
  estación antes de abrir).
* ``{"tipo": "tel", "t", "y": [64]}``: cada paquete del puente TAL CUAL llegó (los 64 doubles; null = NaN,
  "Inf"/"-Inf" = infinito). ``t`` = hora de llegada (los paquetes de una misma ráfaga TCP comparten la hora).
* ``{"tipo": "cmd", "t", "cmd", "origen", "resultado", "cmd_seq", "detalle"}``: cada comando pedido a la estación
  (/api/cmd); ``resultado`` = enviado | en cola | 409 (rechazado: sin telemetría, modo o cámara distintos) | 400.
* ``{"tipo": "ack", "t", "cmd", "cmd_seq", "estado", "detalle", "latencia_s"}``: final de cada comando en el puente
  (recibido = ack de Simulink, caducado, descartado, reemplazado); ``latencia_s`` = desde el clic.
* ``{"tipo": "evento", "t", "msg"}``: cada línea de los eventos de la estación.
* ``{"tipo": "aviso", "t", "accion", "nivel", "msg"}``: un aviso de la barra roja/amarilla que aparece o desaparece
  (``accion`` = "activo" en t = 0: los que ya estaban al abrir).
* ``{"tipo": "muestra", "t", "camara", "vision", "puente", "avisos", "objetivos", "candidatos", ...}``: ~1 Hz, estado
  de la cámara (fps, edad del último frame), de la visión, del puente y los objetivos de la estación.
* ``{"tipo": "fin", "t", "motivo", "duracion_s", "n_tel", "lineas"}``: al cerrar.

Cuándo: habilitada (``web.caja_negra`` / ``--caja-negra``: auto = solo la fuente simulink), la estación guarda
siempre los últimos ``pre_s`` s en memoria. El archivo se abre cuando la estación manda un START o cuando la misión
entra a un estado de vuelo (``EN_VUELO``: TAKEOFF..PAUSED, LANDING y ABORT incluidos), y se cierra ``post_s`` s
después de salir de ellos (DONE, START rechazado, Simulink reiniciado en INIT), tras ``timeout_s`` sin telemetría, al
cambiar de fuente o al detener la estación. Solo graba: no actúa sobre nada.

Escritura: hereda de ``RegistroVuelos`` el hilo escritor, los reintentos y los avisos de error de disco (a lo más
uno por minuto); el puente solo agrega a una cola en memoria. Cada vaciado (cada ``flush_s`` s) agrega al archivo un
MIEMBRO gzip completo: un corte de luz o un cierre a la fuerza deja el archivo legible hasta el último vaciado
(``repeticion.leer_lineas`` tolera un último miembro cortado; ``gzip -dc`` también lo lee, avisando al final).
Cada línea se serializa y se comprime UNA sola vez (``_comprimir``): si el disco falla, se reintentan los mismos
miembros ya comprimidos (sin volver a gastar CPU) y, si la falla dura más de ``max_cola`` líneas, se tiran los miembros
más viejos salvo los que traen la cabecera (sin ella el archivo no se puede leer). Un archivo dura a lo más ``max_s``
s (``MAX_S``): si la misión sigue en un estado de vuelo, se cierra y sigue en otro (ningún archivo llega a los 100 MB
que GitHub rechaza).
"""
from __future__ import annotations

import collections
import gzip
import math
import os
import struct
import threading
import time

from . import protocol as P
from .registro_vuelo import RegistroVuelos, _json, _limpio

FORMATO = 1
EXT = ".tel.jsonl.gz"
HZ_NOMINAL = 50.0          # telemetría de Simulink
# Se graban TODOS los paquetes (decimación 1; ``web.caja_negra_decimacion`` = 2 graba 25 Hz). Medido con 10 min de
# vuelo dual a 50 Hz: mock tal cual 0.95 MB; mock con ruido tipo OptiTrack en pose/velocidad/batería (todos los
# dígitos del double ocupados, como un vuelo real) 6.3 MB, y 3.2 MB a 25 Hz (web/README.md, "Caja negra").
DECIMACION = 1
PRE_S = 10.0               # s de búfer antes de abrir (lo que pasó antes del START)
POST_S = 10.0              # s que se sigue grabando después de salir de los estados de vuelo
MUESTRA_S = 1.0            # periodo de las líneas "muestra" (cámara, visión, avisos, objetivos)
MAX_S = 1800.0             # s por archivo (30 min ≈ 19 MB en el peor caso medido): luego sigue en otro archivo
EN_VUELO = (P.TAKEOFF, P.HOVER_READY, P.MISSION, P.RETURN_HOME, P.LANDING, P.ABORT, P.PAUSED)
_VEC = struct.Struct("<%dd" % P.N_TELEMETRY)


class _Bloque:
    """Miembro gzip ya comprimido de ``n`` líneas de un archivo (se reintenta tal cual si el disco falla)."""
    __slots__ = ("datos", "n", "cabecera")

    def __init__(self, datos, n, cabecera):
        self.datos, self.n, self.cabecera = datos, n, cabecera


def _es_cabecera(entrada):
    return isinstance(entrada, dict) and entrada.get("tipo") == "cabecera"


def nombres_y():
    """Nombre de cada índice de la telemetría (lista de 64; posición k = y(k + 1)), igual que ``protocol.py``."""
    n = [f"y{k}" for k in range(1, P.N_TELEMETRY + 1)]
    for nombre, idx in P.TEL_FIELDS.items():
        n[idx - 1] = nombre
    for d, o in P.DRONE_OFFSETS.items():
        for nombre, k in P.DRONE_FIELDS.items():
            n[o + k - 1] = f"d{d}_{nombre}"
    n[55 - 1] = "alerta"
    for k, nombre in enumerate(("obj_id", "obj_cls", "obj_x", "obj_y", "obj_estado")):
        n[P.OBJ_SIM_BASE + k - 1] = nombre
    n[P.EXPLORER_ROLL_IDX - 1] = "explorer_roll"
    n[P.EXPLORER_PITCH_IDX - 1] = "explorer_pitch"
    n[P.EXPLORER_ATT_OK_IDX - 1] = "explorer_att_ok"
    n[P.DRON_SOLO_IDX - 1] = "dron_solo"
    return n


def campos():
    """Bloque ``campos`` de la cabecera: nombres por índice (base 1) y cómo se codifican los valores."""
    return {"n": P.N_TELEMETRY, "base": 1, "nombres": nombres_y(),
            "codificacion": "double tal cual llegó; null = NaN, \"Inf\" / \"-Inf\" = infinito",
            "ver": "web/app/protocol.py (TEL_FIELDS, DRONE_OFFSETS, DRONE_FIELDS) y ARQUITECTURA §3.3"}


def vector(raw):
    """512 bytes -> lista de 64 valores JSON (NaN -> None, ±inf -> "Inf"/"-Inf")."""
    out = []
    for v in _VEC.unpack(raw):
        if v != v:
            out.append(None)
        elif math.isinf(v):
            out.append("Inf" if v > 0 else "-Inf")
        else:
            out.append(v)
    return out


def empacar(y):
    """Inverso de ``vector``: lista de 64 valores -> 512 bytes (para repetir el paquete por el puente)."""
    vals = []
    for v in y:
        if v is None:
            vals.append(math.nan)
        elif isinstance(v, str):
            vals.append(math.inf if v == "Inf" else -math.inf if v == "-Inf" else math.nan)
        else:
            vals.append(float(v))
    if len(vals) != P.N_TELEMETRY:
        raise ValueError(f"el vector tiene {len(vals)} valores (se esperan {P.N_TELEMETRY})")
    return _VEC.pack(*vals)


def _estado(raw):
    """y(2) del paquete crudo (-1 si no es finito)."""
    try:
        v = struct.unpack_from("<d", raw, 8)[0]
        return int(round(v)) if math.isfinite(v) else -1
    except (struct.error, TypeError):
        return -1


class CajaNegra(RegistroVuelos):
    """Graba la telemetría cruda, los comandos, sus acks, los eventos, los avisos y una muestra por segundo.

    ``habilitado_fn()`` = si se graba (se revisa a lo más una vez por segundo); ``cabecera_fn()`` = dict que se
    agrega a la cabecera al abrir; ``muestra_fn()`` = dict de la línea "muestra" (con ``avisos``: lista de
    {"nivel", "msg"}). ``hilo=False`` (pruebas): sin hilo; se escribe con ``vaciar()`` y se muestrea con
    ``muestrear()``."""

    EXT = EXT
    NOMBRE = "Caja negra"
    max_cola = 90000                    # ~30 min de telemetría a 50 Hz si el disco falla mucho tiempo

    def __init__(self, carpeta, fuente_fn, habilitado_fn, log_fn=None, cabecera_fn=None, muestra_fn=None,
                 pre_s=PRE_S, post_s=POST_S, decimacion=DECIMACION, flush_s=2.0, timeout_s=60.0, reloj=time.time,
                 hilo=True, base_rel=None, max_s=MAX_S):
        super().__init__(carpeta, fuente_fn, habilitado_fn, log_fn=log_fn, flush_s=flush_s, flush_n=10 ** 9,
                         timeout_s=timeout_s, reloj=reloj, hilo=hilo, base_rel=base_rel)
        self.cabecera_fn = cabecera_fn or dict
        self.muestra_fn = muestra_fn
        self.pre_s, self.post_s, self.max_s = float(pre_s), float(post_s), float(max_s)
        self.decimacion = max(1, int(decimacion))
        # la cola del registro (self._cola) guarda aquí miembros ya comprimidos ([(ruta, _Bloque)]); las líneas nuevas
        # esperan en _crudas hasta el siguiente vaciado. vaciar() comprime y luego escribe: RLock, una cosa a la vez
        self._lock_disco = threading.RLock()
        self._crudas = []                   # [(ruta, entrada)] aún sin serializar (el puente solo agrega aquí)
        self._t_comp = reloj()
        self._buf = collections.deque()     # [(stamp, entrada)] de los últimos pre_s s, sin archivo abierto
        self._t0 = {}                       # ruta -> t0 (para escribir los t relativos)
        self._habil, self._t_habil = False, -math.inf
        self._n_pkt = 0
        self._avisos_prev = None
        self._t_muestra = -math.inf
        self.ultimo_n_tel = 0

    # -------------------------------------------------------------- habilitación y búfer
    def _habilitado(self, now):
        if now - self._t_habil >= 1.0 or now < self._t_habil:
            try:
                self._habil = bool(self.habilitado_fn())
            except Exception:
                self._habil = False
            self._t_habil = now
            if not self._habil:
                with self._lock:
                    self._buf.clear()
        return self._habil

    def _al_buffer(self, stamp, entrada):
        """(con ``_lock``) Agrega al búfer circular y tira lo más viejo que ``pre_s``."""
        self._buf.append((stamp, entrada))
        lim = stamp - self.pre_s
        while self._buf and (self._buf[0][0] < lim or len(self._buf) > self.pre_s * HZ_NOMINAL * 4):
            self._buf.popleft()

    def descartar_buffer(self):
        """Tira el búfer previo (al cambiar de fuente: lo de la fuente anterior no entra al archivo de la nueva)."""
        with self._lock:
            self._buf.clear()

    def _agregar(self, v, obj):
        """(con ``_lock``) Como el registro, pero se guarda el objeto: se serializa en el hilo escritor."""
        v["n_lineas"] += 1
        self._crudas.append((v["ruta"], obj))
        if len(self._crudas) > self.max_cola:     # el escritor no avanza (p. ej. un disco colgado): no crece sin límite
            self._recortar()

    def _recortar(self):
        """(con ``_lock``) Si hay más de ``max_cola`` líneas pendientes (el disco falla mucho tiempo), tira las más
        viejas (miembros enteros y luego líneas sueltas), salvo las cabeceras: sin ella el archivo no se puede leer."""
        exceso = sum(b.n for _, b in self._cola) + len(self._crudas) - self.max_cola
        if exceso <= 0:
            return
        quedan = []
        for ruta, b in self._cola:
            if exceso > 0 and not b.cabecera:
                exceso -= b.n
                self.n_perdidas += b.n
            else:
                quedan.append((ruta, b))
        self._cola[:] = quedan
        if exceso > 0:
            viejas = self._crudas[:exceso]
            cabeceras = [x for x in viejas if _es_cabecera(x[1])]
            self._crudas[:exceso] = cabeceras
            self.n_perdidas += len(viejas) - len(cabeceras)

    def _registrar(self, d, stamp=None):
        """Línea con hora (``_t``): al archivo abierto o, si no hay, al búfer (solo si la caja está habilitada)."""
        d["_t"] = self.reloj() if stamp is None else float(stamp)
        with self._lock:
            v = self._vuelo
            if v is not None:
                self._agregar(v, d)
            elif self._habil:
                self._al_buffer(d["_t"], d)

    # -------------------------------------------------------------- entradas
    def telemetria(self, raw, stamp):
        """Cada paquete crudo del puente (``BridgeClient.raw_listeners``): lo graba y abre / cierra el archivo."""
        now = float(stamp)
        estado = _estado(raw)
        self._n_pkt += 1
        grabar = self.decimacion == 1 or self._n_pkt % self.decimacion == 0
        habil = self._habilitado(now)
        abrir = cerrar = None
        with self._lock:
            v = self._vuelo
            if v is not None:
                v["t_tick"] = now
                if grabar:
                    self._agregar(v, ("tel", now, raw))
                    v["n_tel"] += 1
                if estado in EN_VUELO:
                    v["t_vuelo"], v["volo"] = now, True
                elif now - v["t_vuelo"] > self.post_s:
                    nombre = P.MISSION_STATES.get(estado, estado)
                    cerrar = f"misión en {nombre}" + ("" if v["volo"] else " (sin despegar)")
                if cerrar is None and now - v["t0"] > self.max_s:     # sigue en otro archivo (si sigue en vuelo)
                    cerrar = f"tope de {self.max_s:g} s por archivo"
            elif habil:
                if grabar:
                    self._al_buffer(now, ("tel", now, raw))
                if estado in EN_VUELO:
                    abrir = f"misión en {P.MISSION_STATES.get(estado, estado)}"
        if habil:
            self._arrancar_hilo()
        if cerrar:
            self.cerrar(cerrar)
        elif abrir:
            self.abrir(abrir)

    def comando(self, cmd, origen, resultado, cmd_seq=None, detalle=""):
        """Un comando pedido a la estación y lo que respondió. Un START que salió abre el archivo."""
        self._registrar({"tipo": "cmd", "cmd": cmd, "origen": origen, "resultado": resultado, "cmd_seq": cmd_seq,
                         "detalle": detalle})
        if cmd == "START" and resultado in ("enviado", "en cola") and self._habilitado(self.reloj()):
            self.abrir("START")

    def ack(self, r):
        """Final de un comando en el puente (``BridgeClient.on_cmd_result``)."""
        t, tc = r.get("t"), r.get("t_click")
        lat = round(t - tc, 3) if isinstance(t, (int, float)) and isinstance(tc, (int, float)) else None
        self._registrar({"tipo": "ack", "cmd": r.get("cmd"), "cmd_seq": r.get("cmd_seq"), "estado": r.get("estado"),
                         "detalle": r.get("detalle"), "latencia_s": lat}, t)

    def evento(self, msg):
        self._registrar({"tipo": "evento", "msg": msg})

    def muestrear(self):
        """Línea "muestra" (~1 Hz) y una línea "aviso" por cada aviso que aparece o desaparece."""
        if self.muestra_fn is None:
            return
        try:
            m = _limpio(dict(self.muestra_fn() or {}))     # copia JSON: se serializa después, en el hilo escritor
        except Exception as exc:            # la muestra es informativa: nunca tumba al hilo
            m = {"error_muestra": f"{type(exc).__name__}: {exc}"}
        now = self.reloj()
        avisos = m.get("avisos")
        if isinstance(avisos, list):
            actuales = [(a.get("nivel"), a.get("msg")) for a in avisos if isinstance(a, dict)]
            prev = self._avisos_prev or []
            for nivel, msg in actuales:
                if (nivel, msg) not in prev:
                    self._registrar({"tipo": "aviso", "accion": "aparece", "nivel": nivel, "msg": msg}, now)
            for nivel, msg in prev:
                if (nivel, msg) not in actuales:
                    self._registrar({"tipo": "aviso", "accion": "desaparece", "nivel": nivel, "msg": msg}, now)
            self._avisos_prev = actuales
        self._registrar(dict({"tipo": "muestra"}, **m), now)

    # -------------------------------------------------------------- ciclo del archivo
    def abrir(self, motivo="", cabecera=None):
        """Abre el archivo (si no hay otro abierto) con la cabecera y el búfer de los últimos ``pre_s`` s."""
        now = self.reloj()
        lt = time.localtime(now)
        try:
            extra = _limpio(dict(self.cabecera_fn() or {}))
        except Exception as exc:            # la cabecera es informativa: nunca impide grabar
            extra = {"error_cabecera": f"{type(exc).__name__}: {exc}"}
        with self._lock:
            if self._vuelo is not None or self._parar.is_set():     # detenida: no se abre otro archivo
                return False
            try:
                ruta = self._ruta_nueva(lt)
            except OSError as exc:
                self.error = f"{type(exc).__name__}: {exc}"
                return False
            self._t0[ruta] = now
            cab = {"tipo": "cabecera", "formato": FORMATO, "fecha": time.strftime("%Y-%m-%dT%H:%M:%S", lt),
                   "t0": round(now, 4), "fuente": self.fuente_fn(), "motivo_apertura": motivo, "pre_s": self.pre_s,
                   "post_s": self.post_s, "decimacion": self.decimacion, "hz_nominal": HZ_NOMINAL, "campos": campos()}
            cab.update(extra)
            cab.update(cabecera or {})
            v = {"ruta": ruta, "t0": now, "t_tick": now, "t_vuelo": now, "volo": False, "materializado": True,
                 "lineas": [], "n_lineas": 0, "n_tel": 0}
            self._vuelo = v
            self._agregar(v, cab)
            for _, entrada in self._buf:
                self._agregar(v, entrada)
                if isinstance(entrada, tuple):
                    v["n_tel"] += 1
            self._buf.clear()
            # los avisos que ya estaban (aparecieron antes del búfer): el archivo se explica solo
            for nivel, msg in self._avisos_prev or []:
                self._agregar(v, {"tipo": "aviso", "_t": now, "accion": "activo", "nivel": nivel, "msg": msg})
        self._arrancar_hilo()
        self._evento.set()
        self.log_fn(f"Caja negra abierta ({motivo}): {self._rel(ruta)}")
        return True

    def cerrar(self, motivo="cerrado"):
        """Cierra el archivo abierto con la línea "fin" y pide escribir ya."""
        now = self.reloj()
        with self._lock:
            v = self._vuelo
            if v is None:
                return None
            self._vuelo = None
            fin = {"tipo": "fin", "_t": now, "motivo": motivo, "duracion_s": round(now - v["t0"], 2),
                   "n_tel": v["n_tel"], "lineas": v["n_lineas"] + 1}
            self._agregar(v, fin)
            self.ultimo, self.ultimo_lineas, self.ultimo_n_tel = v["ruta"], v["n_lineas"], v["n_tel"]
            self._urgente = True
        self._evento.set()
        self.log_fn(f"Caja negra cerrada ({motivo}): {self._rel(v['ruta'])}, {v['n_tel']} paquetes de telemetría")
        return v["ruta"]

    def base_actual(self):
        """Ruta sin extensión del archivo abierto (el registro de observaciones usa el mismo nombre) o None."""
        with self._lock:
            v = self._vuelo
            return v["ruta"][:-len(self.EXT)] if v is not None else None

    def archivo_actual(self):
        with self._lock:
            v = self._vuelo
            return self._rel(v["ruta"]) if v is not None else None

    # -------------------------------------------------------------- disco
    def _linea(self, ruta, entrada):
        t0 = self._t0.get(ruta, 0.0)
        try:
            if isinstance(entrada, tuple):              # ("tel", stamp, raw)
                return _json({"tipo": "tel", "t": round(entrada[1] - t0, 4), "y": vector(entrada[2])})
            d = dict(entrada)
            if "_t" not in d:                           # cabecera
                return _json(d)
            t = d.pop("_t")
            return _json(dict({"tipo": d.pop("tipo", "?"), "t": round(t - t0, 4)}, **d))
        except Exception as exc:                        # una línea rara no detiene la grabación
            return _json({"tipo": "error_linea", "error": f"{type(exc).__name__}: {exc}"})

    def vaciar(self, forzar=True):
        """Comprime lo pendiente (cada ``flush_s`` s) y escribe los miembros que falten (``RegistroVuelos.vaciar``)."""
        with self._lock_disco:
            hecho = self._comprimir(forzar)
            return super().vaciar(forzar or hecho)

    def _comprimir(self, forzar):
        """(con ``_lock_disco``) Pasa las líneas de ``_crudas`` a la cola como UN miembro gzip por archivo. Cada línea se
        serializa y se comprime una sola vez: los reintentos por un error de disco no vuelven a gastar CPU."""
        now = self.reloj()
        with self._lock:
            if not self._crudas or not (forzar or self._urgente or now - self._t_comp >= self.flush_s):
                return False
            lote, self._crudas, self._t_comp = self._crudas, [], now
        bloques, i = [], 0
        while i < len(lote):
            ruta, j = lote[i][0], i
            while j < len(lote) and lote[j][0] == ruta:
                j += 1
            entradas = [e for _, e in lote[i:j]]
            datos = "".join(self._linea(ruta, e) + "\n" for e in entradas).encode("utf-8")
            bloques.append((ruta, _Bloque(gzip.compress(datos, compresslevel=6, mtime=0), len(entradas),
                                          any(_es_cabecera(e) for e in entradas))))
            i = j
        with self._lock:
            self._cola.extend(bloques)          # después de los que esperan reintento: el orden se conserva
            self._recortar()
        return True

    def _escribir(self, ruta, bloques):
        """(con ``_lock_disco``) Agrega los miembros gzip ya comprimidos. Todo o nada: si falla a medias, recorta el
        archivo a como estaba (si tampoco se puede, ``repeticion.leer_lineas`` salta la basura hasta el siguiente
        miembro) y se reintentan los mismos miembros en el siguiente ciclo."""
        p0 = None
        try:
            comp = b"".join(b.datos for b in bloques)
            os.makedirs(os.path.dirname(ruta), exist_ok=True)
            p0 = os.path.getsize(ruta) if os.path.exists(ruta) else 0
            with open(ruta, "ab") as fh:
                fh.write(comp)
        except Exception as exc:
            if p0 is not None:
                try:
                    if os.path.getsize(ruta) > p0:
                        with open(ruta, "r+b") as fh:
                            fh.truncate(p0)
                except OSError:
                    pass
            return 0, exc
        return len(bloques), None

    def _bucle(self):
        while not self._parar.is_set():
            self._evento.wait(0.25)
            self._evento.clear()
            try:
                now = self.reloj()
                if (self._habil or self.abierto()) and now - self._t_muestra >= MUESTRA_S:
                    self._t_muestra = now
                    self.muestrear()
                self.vaciar(forzar=False)
                with self._lock:
                    v = self._vuelo
                    viejo = v is not None and self.reloj() - v["t_tick"] > self.timeout_s
                if viejo:
                    self.cerrar(f"sin telemetría {self.timeout_s:g} s")
            except Exception as exc:            # pragma: no cover - el hilo nunca debe morir
                self.error = f"{type(exc).__name__}: {exc}"

    # -------------------------------------------------------------- estado para la UI
    def estado(self):
        with self._lock:
            v = self._vuelo
            out = {"carpeta": self._rel(self.carpeta), "abierto": v is not None,
                   "archivo": self._rel(v["ruta"]) if v is not None else None,
                   "lineas": v["n_lineas"] if v is not None else 0, "n_tel": v["n_tel"] if v is not None else 0,
                   "ultimo": self._rel(self.ultimo), "ultimo_lineas": self.ultimo_lineas,
                   "ultimo_n_tel": self.ultimo_n_tel, "error": self.error, "errores": self.n_errores,
                   "perdidas": self.n_perdidas,
                   "en_cola": sum(b.n for _, b in self._cola) + len(self._crudas),
                   "buffer_s": round(self._buf[-1][0] - self._buf[0][0], 1) if len(self._buf) > 1 else 0.0}
        try:
            out["habilitado"] = bool(self.habilitado_fn())
        except Exception:
            out["habilitado"] = False
        return out
