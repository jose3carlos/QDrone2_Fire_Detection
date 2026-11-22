"""Cliente TCP del puente Simulink ⇄ Python (ARQUITECTURA §3.3).

Simulink (o el simulador MATLAB 3D, o el mock interno) es el *Stream Server* en ``:18700``.
Este cliente:
* recibe telemetría de 64 doubles (se queda con el mensaje más reciente);
* envía 32 doubles a 20 Hz con ``heartbeat`` creciente, el comando en curso y su ``cmd_seq``,
  y 3 ranuras de objetivos que rotan entre todos los objetivos conocidos;
* comandos con cola y caducidad (seguridad): solo se aceptan con telemetría fresca; se manda
  uno a la vez y el siguiente sale cuando Simulink devuelve el ``cmd_seq_ack``. Si no llega el
  ack en ``cmd_timeout_s`` el comando **caduca**: se incrementa ``cmd_seq`` con ``cmd = 0``
  para que un comando viejo nunca se ejecute al reconectar. ABORT y LAND_ALL tienen
  prioridad: descartan los comandos normales que esperan en cola y reemplazan al que está
  en curso si es normal;
* se reconecta solo si el servidor no está o se cae (QUARC usa TCP sin encabezados, así que
  basta un socket normal; no hace falta el SDK de Quanser).
"""
from __future__ import annotations

import collections
import select
import socket
import threading
import time

from . import protocol as P

EMERGENCIA = (P.CMD_ABORT, P.CMD_LAND_ALL)


class PuenteNoListo(RuntimeError):
    """El puente no tiene telemetría fresca: el comando no se acepta (no se guarda para después)."""


class BridgeClient:
    def __init__(self, host="127.0.0.1", port=18700, params_fn=None, send_hz=20.0,
                 reconnect_s=1.0, stale_s=2.0, cmd_timeout_s=1.5):
        self.host, self.port = host, int(port)
        self.params_fn = params_fn or (lambda: {})
        self.period = 1.0 / float(send_hz)
        self.reconnect_s = float(reconnect_s)
        self.stale_s = float(stale_s)
        self.cmd_timeout_s = float(cmd_timeout_s)

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._targets = []           # [(id, clase, x, y, conf)] ordenados por id
        self._rot = P.TargetRotator()
        self.heartbeat = 0
        self.cmd_seq = 0             # último cmd_seq usado (comando o cancelación)
        self._inflight = None        # comando en curso: dict(code, name, seq, t_click)
        self._queue = collections.deque()   # comandos en espera: dict(code, name, t_click)
        self.cmd_results = collections.deque(maxlen=20)  # historial: recibido/caducado/descartado/reemplazado
        self._n_results = 0
        self.on_cmd_result = None    # f(dict) opcional (la estación lo usa para su bitácora)
        self.telemetry = None
        self.stamp = 0.0
        self.connected = False
        self.error = ""
        self.n_rx = self.n_tx = self.n_reconnects = 0
        self._rx_times = collections.deque(maxlen=100)
        self._tx_times = collections.deque(maxlen=100)
        self.listeners = []          # funciones f(tel, stamp) llamadas en CADA mensaje de telemetría (en orden)
        self.last_tx = None          # últimos bytes enviados (para diagnóstico/pruebas)
        # Objetivos retenidos hasta la primera telemetría de cada conexión: si Simulink se reinició,
        # la estación lo detecta con esa telemetría (t vuelve a 0) y vacía su lista ANTES de mandar
        # objetivos viejos a un Simulink con la tabla vacía.
        self._sesion_con_rx = True
        self.n_sesion = 0               # cuenta las conexiones (la estación revisa la 1.ª telemetría de cada una)
        self.n_targets_rechazados = 0   # objetivos con id fuera de 1..32 que no se mandaron

    # ------------------------------------------------------------------ API pública
    @property
    def uri(self):
        return f"tcpip://{self.host}:{self.port}"

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, daemon=True, name="puente-18700")
            self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3)

    @property
    def cmd(self):
        """Código del comando que se está mandando ahora (0 si no hay ninguno en curso)."""
        inf = self._inflight
        return inf["code"] if inf else P.CMD_NONE

    def send_command(self, name_or_code, require_fresh=True):
        """Pone un comando en la cola. Devuelve el ``cmd_seq`` si salió de inmediato, o None si quedó
        en cola detrás de otro. Lanza ``PuenteNoListo`` si no hay telemetría fresca."""
        code = P.COMMANDS[name_or_code] if isinstance(name_or_code, str) else int(name_or_code)
        if code not in P.COMMAND_NAMES or code == P.CMD_NONE:
            raise ValueError(f"Comando desconocido: {name_or_code}")
        if require_fresh and not self.fresh():
            raise PuenteNoListo("sin telemetría reciente del puente: el comando NO se envió")
        now = time.time()
        item = {"code": code, "name": P.COMMAND_NAMES[code], "t_click": now}
        with self._lock:
            if code in EMERGENCIA:
                # descarta los comandos normales que esperaban y reemplaza al normal en curso
                for q in [q for q in self._queue if q["code"] not in EMERGENCIA]:
                    self._queue.remove(q)
                    self._result(q, "descartado", f"lo reemplazó {item['name']}")
                if self._inflight and self._inflight["code"] not in EMERGENCIA:
                    self._result(self._inflight, "reemplazado", f"lo reemplazó {item['name']}")
                    self._inflight = None
            self._queue.append(item)
            self._advance(now)
            return item.get("seq")

    def _ack(self):
        return self.telemetry.get("cmd_seq_ack") if self.telemetry else None

    def _result(self, item, estado, detalle=""):
        """Registra el final de un comando (llamar con el candado tomado)."""
        self._n_results += 1
        r = {"n": self._n_results, "cmd": item["name"], "cmd_seq": item.get("seq"), "estado": estado,
             "detalle": detalle, "t": time.time()}
        self.cmd_results.append(r)
        if self.on_cmd_result:
            try:
                self.on_cmd_result(r)
            except Exception:
                pass

    def _advance(self, now):
        """Ack, caducidad y salida del siguiente comando de la cola (con el candado tomado)."""
        ack = self._ack()
        inf = self._inflight
        if inf is not None:
            if ack is not None and ack == inf["seq"] and self.stamp >= inf["t_tx"]:
                self._result(inf, "recibido", f"Simulink devolvió ack {ack}")
                self._inflight = inf = None
            elif now - inf["t_click"] > self.cmd_timeout_s:
                # caduca: se cambia cmd_seq con cmd = 0 para que Simulink nunca lo ejecute tarde
                self.cmd_seq += 1
                self._result(inf, "caducado", f"sin ack en {self.cmd_timeout_s:.1f} s; se mandó cmd_seq "
                                              f"{self.cmd_seq} con cmd=0")
                self._inflight = inf = None
        while self._queue and now - self._queue[0]["t_click"] > self.cmd_timeout_s:
            self._result(self._queue.popleft(), "caducado", "no alcanzó a salir de la cola")
        if inf is None and self._queue and self.fresh():
            item = self._queue.popleft()
            # el cmd_seq nuevo siempre es distinto del ack actual de Simulink (p. ej. tras reiniciar
            # la estación, cuando Simulink conserva el ack de la sesión anterior)
            self.cmd_seq = max(self.cmd_seq, int(ack or 0)) + 1
            item["seq"] = self.cmd_seq
            item["t_tx"] = now
            self._inflight = item

    def set_targets(self, targets):
        """``targets``: iterable de (id, clase, x, y, conf). Se envían como máximo 32."""
        t = sorted((tuple(float(v) for v in tt) for tt in targets), key=lambda r: r[0])
        ok = [tt for tt in t if 1 <= tt[0] <= P.MAX_TARGETS][:P.MAX_TARGETS]
        with self._lock:
            self._targets = ok
            self.n_targets_rechazados = len(t) - len(ok)

    def latest(self):
        with self._lock:
            return self.telemetry, self.stamp

    def fresh(self):
        return self.telemetry is not None and (time.time() - self.stamp) < self.stale_s

    def status(self):
        now = time.time()
        with self._lock:
            ack = self.telemetry.get("cmd_seq_ack") if self.telemetry else None
            return {
                "uri": self.uri, "connected": self.connected, "fresh": self.fresh(),
                "error": self.error, "rx_hz": _rate(self._rx_times, now), "tx_hz": _rate(self._tx_times, now),
                "rx_age_s": round(now - self.stamp, 2) if self.stamp else None,
                "heartbeat": self.heartbeat, "cmd": P.COMMAND_NAMES.get(self.cmd, "?"),
                "cmd_seq": self.cmd_seq, "cmd_seq_ack": ack, "n_targets_tx": len(self._targets),
                "n_targets_rechazados": self.n_targets_rechazados,
                "reconnects": self.n_reconnects, "cola": [q["name"] for q in self._queue],
                "cmd_resultados": list(self.cmd_results)[-8:],
            }

    # ------------------------------------------------------------------ mensaje saliente
    def build_message(self):
        """Arma el siguiente mensaje de 32 doubles (incrementa el heartbeat y rota ranuras)."""
        p = self.params_fn() or {}
        with self._lock:
            self.heartbeat += 1
            self._advance(time.time())
            # sin comando en curso (ya aplicado, caducado o nunca hubo) se manda cmd = 0
            cmd = self.cmd
            targets = self._targets if self._sesion_con_rx else []
            slots = self._rot.next_slots(targets)
            msg = P.pack_command(self.heartbeat, cmd, self.cmd_seq, len(targets), slots,
                                 p.get("z_explorer", 0.0), p.get("z_responder", 0.0),
                                 p.get("t_mark", 0.0), p.get("d_min_h", 0.0))
        self.last_tx = msg
        return msg

    # ------------------------------------------------------------------ hilo
    def _run(self):
        while not self._stop.is_set():
            sock = None
            try:
                sock = socket.create_connection((self.host, self.port), timeout=2.0)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.settimeout(None)
                sock.setblocking(False)
                with self._lock:
                    self.connected, self.error = True, ""
                self._session(sock)
            except (OSError, ConnectionError, ValueError) as exc:
                self.error = f"{type(exc).__name__}: {exc}"
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
                if self.connected:
                    self.n_reconnects += 1
                self.connected = False
            self._stop.wait(self.reconnect_s)

    def _session(self, sock):
        buf = bytearray()
        next_tx = time.time()
        last_rx = time.time()
        self._sesion_con_rx = False
        self.n_sesion += 1
        while not self._stop.is_set():
            now = time.time()
            timeout = max(0.0, next_tx - now)
            r, _, _ = select.select([sock], [], [], timeout)
            if r:
                chunk = sock.recv(65536)
                if not chunk:
                    raise ConnectionError("el servidor cerró la conexión")
                buf += chunk
                n = len(buf) // P.TELEMETRY_BYTES
                if n:
                    # Se desempaquetan TODOS los mensajes completos, no solo el último: y(56:60) trae un
                    # objetivo distinto en cada mensaje (rotando), y Simulink / el simulador 3D pueden mandar
                    # varios juntos (ráfaga). Si solo se leyera el último, con ráfagas regulares algunos
                    # objetivos no se verían nunca. ``telemetry`` sigue siendo el más reciente.
                    tels = [P.unpack_telemetry(bytes(buf[k * P.TELEMETRY_BYTES:(k + 1) * P.TELEMETRY_BYTES]))
                            for k in range(n)]
                    del buf[:n * P.TELEMETRY_BYTES]
                    stamp = last_rx = time.time()
                    with self._lock:
                        self.telemetry, self.stamp = tels[-1], stamp
                        self.n_rx += n
                        self._rx_times.extend([stamp] * min(n, 5))
                    # los oyentes reciben cada mensaje en orden (el último al final, con la misma hora)
                    for tel in tels:
                        for f in list(self.listeners):
                            try:
                                f(tel, stamp)
                            except Exception:  # un oyente con error no debe tumbar el puente
                                pass
                    self._sesion_con_rx = True
            if time.time() >= next_tx:
                msg = self.build_message()
                sock.setblocking(True)
                sock.settimeout(1.0)
                sock.sendall(msg)
                sock.setblocking(False)
                self.n_tx += 1
                self._tx_times.append(time.time())
                next_tx += self.period
                if next_tx < time.time() - 1.0:  # si nos atrasamos mucho, no se mandan ráfagas
                    next_tx = time.time() + self.period
            if time.time() - last_rx > 5 * self.stale_s:
                raise ConnectionError("sin telemetría; reconectando")


def _rate(times, now, window=2.0):
    n = sum(1 for t in times if now - t <= window)
    return round(n / window, 1)
