"""Lectura de la caja negra (``caja_negra.py``) y repetición de un vuelo por el MISMO camino que la telemetría real.

``python -m app.server --repetir <archivo.tel.jsonl.gz> [--velocidad N]`` arranca la estación con un
``ServidorRepeticion`` en lugar de Simulink / el mock: es un Stream Server TCP en 127.0.0.1 (puerto libre) que manda
los paquetes crudos grabados con el ritmo original (× velocidad). El ``BridgeClient`` de siempre se conecta a él, así
que la UI muestra todo igual que en el vuelo (vista 3D, mapa 2D, tarjetas, batería, alertas). Lo que no viaja en la
telemetría sale de la grabación: avisos, cámara, visión y objetivos de la estación (líneas "muestra", ~1 Hz) y
los eventos. La estación queda en solo lectura (``Station.repeticion``): comandos y cambios responden 409.

Con la repetición en pausa (o al final) el servidor sigue mandando el último paquete (rotando los objetivos de
y(56:60) que ya se vieron) para que la telemetría no se vea vieja.
"""
from __future__ import annotations

import bisect
import json
import math
import os
import select
import socket
import struct
import tempfile
import threading
import time
import zlib

from . import protocol as P
from .caja_negra import empacar

GZ_MAGIA = b"\x1f\x8b\x08"
VELOCIDADES = (0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0)
KEEPALIVE_S = 0.05      # en pausa: el último paquete a 20 Hz


def leer_lineas(ruta):
    """Líneas JSON de un ``.tel.jsonl.gz`` (o ``.jsonl``) y un dict ``info`` {"miembros", "truncado",
    "basura_bytes", "danadas"}. Tolera un último miembro gzip cortado (corte de luz) y basura entre miembros (una
    escritura fallida que no se pudo recortar): salta hasta el siguiente encabezado gzip. Una última línea a medias
    o una línea que no es JSON se cuentan en ``danadas`` y se omiten."""
    with open(ruta, "rb") as fh:
        datos = fh.read()
    info = {"miembros": 0, "truncado": False, "basura_bytes": 0, "danadas": 0}
    if not datos.startswith(GZ_MAGIA):
        texto = datos
    else:
        partes, pos = [], 0
        while pos < len(datos):
            d = zlib.decompressobj(wbits=31)
            try:
                partes.append(d.decompress(datos[pos:]))
            except zlib.error:
                sig = datos.find(GZ_MAGIA, pos + 1)
                salto = (sig if sig >= 0 else len(datos)) - pos
                info["basura_bytes"] += salto
                pos += salto
                continue
            if not d.eof:                       # último miembro cortado: lo que se alcanzó a descomprimir
                try:
                    partes.append(d.flush())
                except zlib.error:
                    pass
                info["truncado"] = True
                info["miembros"] += 1
                break
            info["miembros"] += 1
            pos = len(datos) - len(d.unused_data)
        texto = b"".join(partes)
    lineas = []
    trozos = texto.split(b"\n")
    if trozos and trozos[-1].strip():           # última línea sin salto: a medias
        info["danadas"] += 1
    for t in trozos[:-1]:
        t = t.strip()
        if not t:
            continue
        try:
            o = json.loads(t.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            info["danadas"] += 1
            continue
        if isinstance(o, dict):
            lineas.append(o)
        else:
            info["danadas"] += 1
    return lineas, info


class Grabacion:
    """Contenido de una caja negra listo para repetir o analizar.

    ``tel`` = [(t, bytes de 512)] en orden; ``cmds``, ``acks``, ``eventos``, ``avisos``, ``muestras`` = líneas de
    ese tipo (dicts con ``t``); ``cabecera`` / ``fin`` (None si el archivo quedó cortado)."""

    def __init__(self, ruta):
        self.ruta = os.path.abspath(ruta)
        lineas, self.info = leer_lineas(ruta)
        self.cabecera, self.fin = {}, None
        self.tel, self.cmds, self.acks, self.eventos, self.avisos, self.muestras, self.otras = [], [], [], [], [], [], []
        for o in lineas:
            tipo = o.get("tipo")
            if tipo == "cabecera" and not self.cabecera:
                self.cabecera = o
            elif tipo == "tel":
                try:
                    self.tel.append((float(o["t"]), empacar(o["y"])))
                except (KeyError, TypeError, ValueError):
                    self.info["danadas"] += 1
            elif tipo == "cmd":
                self.cmds.append(o)
            elif tipo == "ack":
                self.acks.append(o)
            elif tipo == "evento":
                self.eventos.append(o)
            elif tipo == "aviso":
                self.avisos.append(o)
            elif tipo == "muestra":
                self.muestras.append(o)
            elif tipo == "fin":
                self.fin = o
            else:
                self.otras.append(o)
        if not self.cabecera:
            raise ValueError(f"{ruta}: no es una caja negra (sin cabecera)")
        # el hilo escritor vacía la cola en orden, pero por si acaso (y el búfer previo mezcla fuentes)
        self.tel.sort(key=lambda p: p[0])
        for lista in (self.cmds, self.acks, self.eventos, self.avisos, self.muestras):
            lista.sort(key=lambda o: o.get("t", 0.0))
        self.t_tel = [t for t, _ in self.tel]
        self.t_muestras = [m.get("t", 0.0) for m in self.muestras]
        self.t_eventos = [e.get("t", 0.0) for e in self.eventos]
        self.t0 = float(self.cabecera.get("t0") or 0.0)
        self.t_ini = self.t_tel[0] if self.t_tel else 0.0
        self.t_fin = self.t_tel[-1] if self.t_tel else 0.0

    @property
    def fuente(self):
        return self.cabecera.get("fuente") or "?"

    def config_temporal(self):
        """Escribe la config grabada (``config_datos``) en un archivo temporal y devuelve su ruta (None si la
        cabecera no la trae). La estación de la repetición lee ESA config (pista, tubos, modo del vuelo)."""
        datos = self.cabecera.get("config_datos")
        if not isinstance(datos, dict):
            return None
        carpeta = tempfile.mkdtemp(prefix="qd2_repeticion_")
        ruta = os.path.join(carpeta, "estacion.json")
        with open(ruta, "w", encoding="utf-8") as fh:
            json.dump(datos, fh, ensure_ascii=False, indent=2)
        return ruta

    def muestra_en(self, t):
        """Última línea "muestra" con t <= ``t`` (None antes de la primera)."""
        i = bisect.bisect_right(self.t_muestras, t) - 1
        return self.muestras[i] if i >= 0 else None

    def hora(self, t):
        """Hora de la PC (HH:MM:SS) de un t relativo."""
        return time.strftime("%H:%M:%S", time.localtime(self.t0 + t))

    def eventos_en(self, t, n=15):
        """Eventos de la estación hasta ``t`` (el más nuevo primero), como los muestra la UI, y antes los de la
        cabecera (``eventos_previos``: los que ya había al abrir el archivo)."""
        i = bisect.bisect_right(self.t_eventos, t)
        out = [{"t": self.hora(e.get("t", 0.0)), "msg": e.get("msg", "")} for e in reversed(self.eventos[max(0, i - n):i])]
        if len(out) < n:
            previos = [e for e in (self.cabecera.get("eventos_previos") or []) if isinstance(e, dict)]
            t_buf = self.t0 + (self.t_eventos[0] if self.t_eventos else 0.0)
            out += [{"t": e.get("t"), "msg": e.get("msg")} for e in previos
                    if not isinstance(e.get("ts"), (int, float)) or e["ts"] < t_buf - 1e-3][:n - len(out)]
        return out


class ServidorRepeticion:
    """Stream Server TCP (1 cliente) que manda los paquetes de una ``Grabacion`` con su ritmo original."""

    def __init__(self, grab, velocidad=1.0, host="127.0.0.1", port=0):
        self.grab = grab
        self.host = host
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((host, int(port)))
        self._srv.listen(1)
        self.port = self._srv.getsockname()[1]
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="repeticion")
        self.velocidad = _velocidad(velocidad)
        self.t = grab.t_ini
        self.i = 0
        self.pausado = False
        self.fin = not grab.tel
        self.saltos = 0
        self.clientes = 0
        self.n_tx = 0
        self._ultimo = None
        self._pendiente = None          # paquete del instante al que se saltó (sale en seguida, aun en pausa)
        self._objs = {}                 # id -> 5 doubles de y(56:60) vistos (para el paquete de pausa)
        self._rot = 0

    # -------------------------------------------------------------- control
    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        try:
            self._srv.close()
        except OSError:
            pass
        if self._thread.is_alive():
            self._thread.join(timeout=2)

    def pausa(self, pausado=True):
        with self._lock:
            if not pausado and self.fin:      # al final: "seguir" vuelve a empezar
                self._ir(self.grab.t_ini)
            self.pausado = bool(pausado)

    def set_velocidad(self, v):
        with self._lock:
            self.velocidad = _velocidad(v)

    def saltar(self, t):
        """Lleva la repetición a ``t`` (s, relativo a t0): el siguiente paquete es el grabado en ``t``."""
        t = float(t)
        if not math.isfinite(t):        # NaN dejaría self.t = NaN y estado() (snapshot) fallaría en cada vuelta
            raise ValueError("t debe ser un número finito (s)")
        with self._lock:
            self._ir(t)
            self.saltos += 1

    def _ir(self, t):
        g = self.grab
        t = min(max(float(t), g.t_ini), g.t_fin)
        self.t = t
        self.i = max(0, bisect.bisect_right(g.t_tel, t) - 1)
        self.fin = False
        # los objetivos de y(56:60) que ya se habían visto hasta ese momento (para el paquete de pausa)
        self._objs = {}
        for _, raw in g.tel[max(0, self.i - 2000):self.i + 1]:
            self._nota_obj(raw)
        if g.tel:                       # el paquete grabado en t sale ya (y es el que se repite en pausa)
            self._pendiente = self._ultimo = g.tel[self.i][1]
            self.i += 1

    def estado(self):
        g = self.grab
        with self._lock:
            return {"archivo": g.ruta, "fecha": g.cabecera.get("fecha"), "fuente": g.fuente,
                    "t": round(self.t, 2), "t_ini": round(g.t_ini, 2), "t_fin": round(g.t_fin, 2),
                    "hora": g.hora(self.t), "velocidad": self.velocidad, "velocidades": list(VELOCIDADES),
                    "pausado": self.pausado, "fin": self.fin, "saltos": self.saltos, "n_tel": len(g.tel),
                    "conectado": self.clientes > 0, "truncado": bool(g.info.get("truncado")) or g.fin is None}

    # -------------------------------------------------------------- hilo
    def _nota_obj(self, raw):
        vals = struct.unpack_from("<5d", raw, (P.OBJ_SIM_BASE - 1) * 8)
        if all(math.isfinite(v) for v in vals) and 1 <= round(vals[0]) <= P.MAX_TARGETS:
            self._objs[int(round(vals[0]))] = vals

    def _pausa_raw(self):
        """El último paquete, rotando en y(56:60) los objetivos que ya se vieron (la tabla de la UI no se vacía)."""
        raw = bytearray(self._ultimo)
        if self._objs:
            ids = sorted(self._objs)
            self._rot = (self._rot + 1) % len(ids)
            struct.pack_into("<5d", raw, (P.OBJ_SIM_BASE - 1) * 8, *self._objs[ids[self._rot]])
        return bytes(raw)

    def _run(self):
        self._srv.setblocking(False)
        conn = None
        t_prev = time.monotonic()
        t_keep = 0.0
        while not self._stop.is_set():
            if conn is None:
                try:
                    conn, _ = self._srv.accept()
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    conn.setblocking(False)
                    self.clientes += 1
                except (BlockingIOError, socket.timeout, OSError):
                    time.sleep(0.02)
                    t_prev = time.monotonic()
                    continue
            ahora = time.monotonic()
            dt, t_prev = ahora - t_prev, ahora
            lote = []
            with self._lock:
                g = self.grab
                if self._pendiente is not None:
                    lote.append(self._pendiente)
                    self._pendiente = None
                if not self.pausado and not self.fin:
                    self.t = min(self.t + dt * self.velocidad, g.t_fin)
                    while self.i < len(g.tel) and g.tel[self.i][0] <= self.t + 1e-9:
                        lote.append(g.tel[self.i][1])
                        self.i += 1
                    if self.i >= len(g.tel):
                        self.fin = self.pausado = True
                for raw in lote:
                    self._nota_obj(raw)
                if lote:
                    self._ultimo = lote[-1]
                    t_keep = ahora
                elif (self.pausado or self.fin) and self._ultimo is not None and ahora - t_keep >= KEEPALIVE_S:
                    lote = [self._pausa_raw()]
                    t_keep = ahora
            try:
                if lote:
                    conn.setblocking(True)
                    conn.settimeout(1.0)
                    conn.sendall(b"".join(lote))
                    conn.setblocking(False)
                    self.n_tx += len(lote)
                # los comandos del puente (32 doubles a 20 Hz) se leen y se tiran: la repetición no obedece nada
                r, _, _ = select.select([conn], [], [], 0.004)
                if r and not conn.recv(65536):
                    raise ConnectionError("el puente cerró la conexión")
            except (OSError, ValueError, ConnectionError):
                try:
                    conn.close()
                except OSError:
                    pass
                conn = None
                self.clientes = max(0, self.clientes - 1)


def _velocidad(v):
    v = float(v)
    if not (math.isfinite(v) and 0.05 <= v <= 64):
        raise ValueError("la velocidad debe estar entre 0.05 y 64")
    return v
