"""Registro en disco de las observaciones de visión de cada vuelo (JSON Lines), para corregir ``pista.tubos``.

El espacio del laboratorio es compartido y los tubos se mueven: una toma real puede quedar "lejos del tubo"
configurado y la estación la descarta (``vision_service.MOTIVO_TUBO``). Cada vuelo deja un archivo
``web/datos/vuelos/AAAA-MM-DD/vuelo_HHMMSS_<fuente>.jsonl`` (fecha/hora local de la PC) que se sube a git para que
cualquier sesión (laptop o PC del laboratorio) lo lea y ``web/herramientas/ajustar_tubos.py`` proponga los tubos
corregidos. NO se muestra en la UI (solo la ruta y las líneas en la pestaña Anomalías).

Formato (una línea JSON por renglón, ``FORMATO`` = 1):
* ``{"tipo": "cabecera", ...}``: fecha, fuente, modo de misión, explorador, pista (ruta, tubos efectivos, tramos sin
  tubo), filtros, confianza mínima, cámara. La arma la estación (``Station._cabecera_vuelo``).
* ``{"tipo": "obs", "t", "stamp", "cls", "conf", "xy", "motivo", "dist_tubo_m", "tubo", "tam_m", "pose", ...}``: una
  por observación con conf >= ``vision.confianza_min`` y punto en el piso, aceptada (``motivo`` = null) o descartada
  por tamaño / lejos del tubo / inclinación. ``tubo`` = tubo más cercano (``geo.tubo_cercano``: pieza, tramo, punto,
  desplazamiento con signo, + = a la izquierda del sentido del tramo). Las de conf < confianza_min solo se cuentan.
* ``{"tipo": "resumen", ...}``: al cerrar, conteos por clase y motivo, n bajo umbral, duración y los objetivos y
  candidatos finales de la estación.

Cuándo: el vuelo se abre cuando el gate de la visión (``Station._vision_gate``: MISSION/PAUSED, explorador
rastreado sobre ``web.z_min_deteccion``) pasa a activo, y se cierra cuando la misión sale de MISSION/PAUSED
(regreso, aterrizaje, abortar, Simulink reiniciado), al reiniciar los objetivos, al cambiar de fuente, al detener la
estación o tras ``timeout_s`` sin telemetría. Si el gate se apaga un momento (pausa, explorador bajo) el vuelo
sigue. Un vuelo sin observaciones solo deja archivo si tuvo más de ``min_gate_s`` s de gate activo.

Escritura: las líneas se acumulan en memoria y un hilo propio las escribe (cada ``flush_s`` s o cada ``flush_n``
líneas, y al cerrar). Ni el hilo de visión ni el del puente tocan el disco. Un error de disco nunca se propaga:
queda en ``estado()["error"]`` y en los eventos de la estación (un error intermitente, a lo más una vez por
minuto), y las líneas que no quedaron en disco se reintentan en el siguiente ciclo (sin duplicar las ya escritas).
"""
from __future__ import annotations

import json
import math
import os
import threading
import time

from . import protocol as P

FORMATO = 1
ESTADOS_VUELO = (P.MISSION, P.PAUSED)   # mismos estados que el gate de la visión
MAX_COLA = 20000                         # líneas en memoria si el disco falla mucho tiempo (después se pierden)
AVISO_S = 60.0                           # un mismo error de disco intermitente se avisa a lo más una vez por minuto


def _limpio(o):
    """JSON estricto: NaN/inf -> null, tuplas -> listas, escalares de numpy -> Python."""
    if isinstance(o, float):
        return float(o) if math.isfinite(o) else None
    if isinstance(o, dict):
        return {str(k): _limpio(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_limpio(v) for v in o]
    if hasattr(o, "item") and not isinstance(o, (str, bytes)):
        try:
            return _limpio(o.item())
        except (TypeError, ValueError):
            return str(o)
    return o


def _json(o):
    return json.dumps(_limpio(o), ensure_ascii=False, separators=(",", ":"))


class RegistroVuelos:
    """Abre/cierra vuelos según el gate de la visión y escribe sus observaciones en ``carpeta``.

    ``habilitado_fn()`` decide al ABRIR un vuelo si se registra (fuente / ``web.registro_vuelos`` / CLI);
    ``fuente_fn()`` da el nombre de la fuente para el archivo; ``resumen_fn()`` (opcional) da un dict que se agrega
    a la línea de resumen (objetivos finales); ``log_fn(texto)`` manda avisos a los eventos de la estación.
    ``hilo=False`` (pruebas): no arranca el hilo escritor; se escribe llamando ``vaciar()``."""

    def __init__(self, carpeta, fuente_fn, habilitado_fn, log_fn=None, resumen_fn=None, flush_s=1.0, flush_n=50,
                 min_gate_s=5.0, timeout_s=60.0, reloj=time.time, hilo=True, base_rel=None):
        self.carpeta = os.path.abspath(carpeta)
        self.fuente_fn, self.habilitado_fn = fuente_fn, habilitado_fn
        self.log_fn = log_fn or (lambda texto: None)
        self.resumen_fn = resumen_fn
        self.flush_s, self.flush_n = float(flush_s), int(flush_n)
        self.min_gate_s, self.timeout_s = float(min_gate_s), float(timeout_s)
        self.reloj = reloj
        self.base_rel = base_rel
        self._usar_hilo = hilo
        self._lock = threading.Lock()          # estado del vuelo y cola (rápido: nunca se hace I/O con él tomado)
        self._lock_disco = threading.Lock()    # una sola escritura a la vez (hilo escritor / detener)
        self._vuelo = None
        self._cola = []                        # [(ruta, texto)] listas para el disco, en orden
        self._urgente = False
        self._t_flush = reloj()
        self._evento = threading.Event()
        self._parar = threading.Event()
        self._hilo = None
        self.error = ""
        self._aviso_err, self._t_aviso, self._aviso_pend = "", -math.inf, False   # último error avisado (eventos)
        self._cortadas = set()                 # rutas que quedaron con una línea a medias sin poder recortarla
        self.n_errores = self.n_perdidas = 0
        self.ultimo = None                     # ruta del último vuelo cerrado que dejó archivo
        self.ultimo_lineas = 0
        self._rutas = set()                    # rutas usadas en esta sesión (dos vuelos en el mismo segundo)

    # -------------------------------------------------------------- utilidades
    def _rel(self, ruta):
        if not ruta:
            return None
        if self.base_rel:
            try:
                r = os.path.relpath(ruta, self.base_rel)
                if not r.startswith(".."):
                    return r.replace(os.sep, "/")
            except ValueError:                 # otra unidad en Windows
                pass
        return ruta.replace(os.sep, "/")

    def abierto(self):
        with self._lock:
            return self._vuelo is not None

    def _ruta_nueva(self, lt):
        dia = os.path.join(self.carpeta, time.strftime("%Y-%m-%d", lt))
        base = f"vuelo_{time.strftime('%H%M%S', lt)}_{self.fuente_fn()}"
        ruta, n = os.path.join(dia, base + ".jsonl"), 1
        while ruta in self._rutas or os.path.exists(ruta):
            n += 1
            ruta = os.path.join(dia, f"{base}_{n}.jsonl")
        self._rutas.add(ruta)
        return ruta

    # -------------------------------------------------------------- ciclo del vuelo
    def tick(self, gate, estado_mision, cabecera_fn=dict):
        """Llamar con cada telemetría: ``gate`` = gate de la visión, ``estado_mision`` = y(2).
        ``cabecera_fn()`` solo se llama al abrir un vuelo."""
        now = self.reloj()
        cerrar = abrir = None
        with self._lock:
            v = self._vuelo
            if v is not None:
                dt = max(0.0, now - v["t_tick"])
                v["t_tick"] = now
                if v["gate_prev"]:
                    v["gate_s"] += min(dt, 1.0)          # un hueco largo de telemetría no cuenta como vuelo
                v["gate_prev"] = bool(gate)
                if estado_mision not in ESTADOS_VUELO:
                    cerrar = f"misión en {P.MISSION_STATES.get(estado_mision, estado_mision)}"
                else:
                    self._quizas_materializar(v)
            elif gate and estado_mision in ESTADOS_VUELO:
                abrir = True
        if cerrar:
            self.cerrar(cerrar)
        elif abrir and self.habilitado_fn():
            self.abrir(cabecera_fn())

    def abrir(self, cabecera=None):
        """Abre un vuelo (si no hay otro abierto). El archivo no se crea hasta que hay algo que guardar."""
        now = self.reloj()
        lt = time.localtime(now)
        with self._lock:
            if self._vuelo is not None:
                return False
            try:
                ruta = self._ruta_nueva(lt)
            except OSError as exc:              # os.path.exists no debería fallar, pero la visión no se detiene
                self.error = f"{type(exc).__name__}: {exc}"
                return False
            fuente = self.fuente_fn()
            cab = {"tipo": "cabecera", "formato": FORMATO, "fecha": time.strftime("%Y-%m-%dT%H:%M:%S", lt),
                   "t0": round(now, 3), "fuente": fuente}
            cab.update(cabecera or {})
            v = {"ruta": ruta, "t0": now, "t_tick": now, "gate_s": 0.0, "gate_prev": True, "lineas": [],
                 "materializado": False, "n_lineas": 0, "n_obs": 0, "n_bajo": 0, "conteos": {}}
            self._vuelo = v
            self._agregar(v, cab)
        self._arrancar_hilo()
        return True

    def observaciones(self, stamp, lineas, n_bajo=0):
        """Observaciones de un frame (dicts sin ``tipo`` ni ``t``). Sin vuelo abierto se ignoran (False)."""
        with self._lock:
            v = self._vuelo
            if v is None:
                return False
            v["n_bajo"] += int(n_bajo)
            for o in lineas:
                clase = P.CLASSES.get(o.get("cls"), str(o.get("cls")))
                motivo = o.get("motivo") or "aceptada"
                cnt = v["conteos"].setdefault(clase, {})
                cnt[motivo] = cnt.get(motivo, 0) + 1
                v["n_obs"] += 1
                self._agregar(v, {"tipo": "obs", "t": round(float(stamp) - v["t0"], 3), **o})
            self._quizas_materializar(v)
        return True

    def cerrar(self, motivo="cerrado"):
        """Cierra el vuelo abierto: agrega el resumen (si el vuelo deja archivo) y pide escribir ya."""
        with self._lock:
            v = self._vuelo
            if v is None:
                return None
            self._vuelo = None
        extra = {}
        if self.resumen_fn is not None:
            try:
                extra = self.resumen_fn() or {}
            except Exception as exc:            # el resumen es informativo: nunca impide cerrar
                extra = {"error_resumen": f"{type(exc).__name__}: {exc}"}
        now = self.reloj()
        res = {"tipo": "resumen", "motivo_cierre": motivo, "duracion_s": round(now - v["t0"], 2),
               "duracion_gate_s": round(v["gate_s"], 2), "n_obs": v["n_obs"], "n_bajo_umbral": v["n_bajo"],
               "conteos": v["conteos"]}
        res.update(extra)
        with self._lock:
            self._quizas_materializar(v)
            if not v["materializado"]:
                guardado = None
            else:
                res["lineas"] = v["n_lineas"] + 1
                self._agregar(v, res)
                self.ultimo, self.ultimo_lineas = v["ruta"], v["n_lineas"]
                self._urgente = True
                guardado = v["ruta"]
        self._evento.set()
        if guardado:
            self.log_fn(f"Registro de vuelo cerrado ({motivo}): {self._rel(guardado)}, {v['n_obs']} observaciones")
        return guardado

    def _quizas_materializar(self, v):
        """(con ``_lock``) El vuelo pasa a tener archivo: con alguna observación o > min_gate_s de gate activo."""
        if v["materializado"] or not (v["n_obs"] > 0 or v["gate_s"] > self.min_gate_s):
            return
        v["materializado"] = True
        self._cola.extend((v["ruta"], t) for t in v["lineas"])
        v["lineas"] = []
        self._recortar()
        self._evento.set()

    def _agregar(self, v, obj):
        """(con ``_lock``) Serializa una línea del vuelo ``v``."""
        texto = _json(obj)
        v["n_lineas"] += 1
        if v["materializado"]:
            self._cola.append((v["ruta"], texto))
            self._recortar()
        else:
            v["lineas"].append(texto)

    def _recortar(self):
        if len(self._cola) > MAX_COLA:
            n = len(self._cola) - MAX_COLA
            del self._cola[:n]
            self.n_perdidas += n

    # -------------------------------------------------------------- disco
    def vaciar(self, forzar=True):
        """Escribe lo pendiente (``forzar`` = sin esperar ``flush_s`` / ``flush_n``). True si quedó todo escrito."""
        with self._lock_disco:
            now = self.reloj()
            with self._lock:
                if not self._cola:
                    return True
                if not (forzar or self._urgente or len(self._cola) >= self.flush_n
                        or now - self._t_flush >= self.flush_s):
                    return True
                lote, self._cola, self._urgente = self._cola, [], False
            i, exc = 0, None
            while i < len(lote):
                ruta = lote[i][0]
                j = i
                while j < len(lote) and lote[j][0] == ruta:
                    j += 1
                hechas, exc = self._escribir(ruta, [t for _, t in lote[i:j]])
                i += hechas
                if exc is not None:
                    break
            if exc is not None:                 # disco lleno, OneDrive bloqueando, carpeta inválida...
                err = f"{type(exc).__name__}: {exc}"
                with self._lock:
                    self._cola[:0] = lote[i:]       # lo que no quedó en disco se reintenta en el siguiente ciclo, en orden
                    self._recortar()
                    self.n_errores += 1
                    self.error = err
                    # un error intermitente (p. ej. OneDrive bloqueando el archivo un ciclo sí y otro no) se avisa a lo
                    # más una vez cada AVISO_S s, para no llenar los eventos de la estación
                    nuevo = err != self._aviso_err or (not self._aviso_pend and now - self._t_aviso >= AVISO_S)
                    if nuevo:
                        self._aviso_err, self._t_aviso, self._aviso_pend = err, now, True
                if nuevo:
                    self.log_fn(f"Registro de vuelo: no se pudo escribir ({err}); se reintenta")
                return False
            self._t_flush = now
            if self._aviso_pend:
                self._aviso_pend = False
                self.log_fn("Registro de vuelo: la escritura volvió a funcionar")
            self.error = ""
            return True

    def _escribir(self, ruta, textos):
        """(con ``_lock_disco``) Agrega las líneas ``textos`` a ``ruta``. Devuelve (líneas completas en disco, error
        o None). Si la escritura falla a medias (disco lleno, cierre fallido) cuenta por tamaño las líneas que sí
        quedaron, para no duplicarlas al reintentar, y recorta la línea cortada; si no se puede recortar, el
        siguiente intento empieza con un salto de línea para no pegarle la siguiente (queda como una línea rota)."""
        pre = b"\n" if ruta in self._cortadas else b""
        datos = [(t + "\n").encode("utf-8") for t in textos]
        p0 = None
        try:
            os.makedirs(os.path.dirname(ruta), exist_ok=True)
            p0 = os.path.getsize(ruta) if os.path.exists(ruta) else 0
            with open(ruta, "ab") as fh:
                fh.write(pre + b"".join(datos))
        except Exception as exc:
            if p0 is None:
                return 0, exc
            try:
                w = os.path.getsize(ruta) - p0
            except OSError:
                return 0, exc               # no se sabe qué quedó: se reintenta todo
            if w <= 0:
                return 0, exc
            self._cortadas.discard(ruta)    # w >= 1 >= len(pre): la línea cortada anterior ya quedó terminada
            w -= len(pre)
            base = p0 + len(pre)
            hechas = acum = 0
            while hechas < len(datos) and acum + len(datos[hechas]) <= w:
                acum += len(datos[hechas])
                hechas += 1
            if w > acum:                    # línea cortada a la mitad
                try:
                    with open(ruta, "r+b") as fh:
                        fh.truncate(base + acum)
                except OSError:
                    self._cortadas.add(ruta)
            return hechas, exc
        self._cortadas.discard(ruta)
        return len(textos), None

    def _arrancar_hilo(self):
        if not self._usar_hilo or (self._hilo is not None and self._hilo.is_alive()) or self._parar.is_set():
            return
        self._hilo = threading.Thread(target=self._bucle, daemon=True, name="registro_vuelo")
        self._hilo.start()

    def _bucle(self):
        while not self._parar.is_set():
            self._evento.wait(0.25)
            self._evento.clear()
            try:
                self.vaciar(forzar=False)
                with self._lock:
                    v = self._vuelo
                    viejo = v is not None and self.reloj() - v["t_tick"] > self.timeout_s
                if viejo:
                    self.cerrar(f"sin telemetría {self.timeout_s:g} s")
            except Exception as exc:            # pragma: no cover - el hilo nunca debe morir
                self.error = f"{type(exc).__name__}: {exc}"

    def detener(self, motivo="estación detenida"):
        """Cierra el vuelo abierto y escribe todo (lo llama ``Station.stop``)."""
        self._parar.set()
        self._evento.set()
        self.cerrar(motivo)
        if self._hilo is not None:
            self._hilo.join(timeout=2.0)
        self.vaciar(forzar=True)

    # -------------------------------------------------------------- estado para la UI
    def estado(self):
        with self._lock:
            v = self._vuelo
            out = {"carpeta": self._rel(self.carpeta), "abierto": v is not None,
                   "archivo": self._rel(v["ruta"]) if v is not None and v["materializado"] else None,
                   "pendiente": v is not None and not v["materializado"],
                   "lineas": v["n_lineas"] if v is not None else 0, "obs": v["n_obs"] if v is not None else 0,
                   "ultimo": self._rel(self.ultimo), "ultimo_lineas": self.ultimo_lineas,
                   "error": self.error, "errores": self.n_errores, "perdidas": self.n_perdidas,
                   "en_cola": len(self._cola)}
        try:
            out["habilitado"] = bool(self.habilitado_fn())
        except Exception:
            out["habilitado"] = False
        return out
