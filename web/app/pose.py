"""Retroalimentacion de posicion [X, Y, Z, Yaw] y publicacion del setpoint.

En ``Trayectory_Model.slx`` la posicion es un bloque Constant ``[0;0;1;0]``
("Position Simulation") y el setpoint ``PoseD`` solo va a un Display: el modelo
no esta conectado al QDrone2. Aqui la fuente de pose es intercambiable:

* ``sim``           : dron simulado de primer orden que sigue el setpoint.
* ``qdrone2_stack`` : protocolo de ``pal.products.qdrone2.QDrone2StreamStack``
                      (Quanser Academic Resources): se envia 1 double y se reciben
                      47 doubles de ``tcpip://<ip>:18491``. Hay que indicar en que
                      indices vienen X, Y, Z, Yaw (ver la guia).
* ``stream``        : cualquier modelo QUARC/Python que publique 4 doubles
                      [X, Y, Z, Yaw] con un bloque *Stream Server* (p. ej. desde OptiTrack).
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

NAN4 = np.full(4, np.nan)


class SimPose:
    name = "sim"

    def __init__(self, cfg):
        self.pos = np.array(cfg.get("sim_start", [0.0, 0.0, 1.0, 0.0]), dtype=float)
        self.speed = float(cfg.get("sim_speed", 0.5))       # m/s
        self.yaw_rate = float(cfg.get("sim_yaw_rate", 1.0))  # rad/s
        self.noise = float(cfg.get("sim_noise", 0.005))      # m
        self.connected, self.error, self.raw = True, "", []

    def update(self, setpoint, dt):
        err = np.asarray(setpoint, float) - self.pos
        err[3] = math.atan2(math.sin(err[3]), math.cos(err[3]))
        step = err[:3] * min(1.0, 2.0 * dt)  # primer orden, tau = 0.5 s
        n = np.linalg.norm(step)
        if n > self.speed * dt:
            step *= self.speed * dt / n
        self.pos[:3] += step
        self.pos[3] += float(np.clip(err[3], -self.yaw_rate * dt, self.yaw_rate * dt))

    def read(self):
        out = self.pos.copy()
        out[:3] += np.random.default_rng().normal(0, self.noise, 3)
        return out

    def close(self):
        pass


class _ThreadedPose:
    """Base: un hilo mantiene la ultima lectura; read() nunca bloquea el lazo de mision."""

    def __init__(self):
        self._latest = NAN4.copy()
        self._stamp = 0.0
        self._stop = threading.Event()
        self.connected, self.error, self.raw = False, "", []
        self.timeout = 0.5
        threading.Thread(target=self._loop, daemon=True, name=self.name).start()

    def update(self, setpoint, dt):
        pass

    def read(self):
        if time.time() - self._stamp > self.timeout:
            return NAN4.copy()  # dato viejo => feedback invalido (la FSM se detiene)
        return self._latest.copy()

    def close(self):
        self._stop.set()

    def _loop(self):
        while not self._stop.is_set():
            stream = None
            try:
                stream = self._connect()
                self.connected, self.error = True, ""
                while not self._stop.is_set():
                    self._receive_once(stream)
            except Exception as exc:
                self.connected, self.error = False, f"{type(exc).__name__}: {exc}"
                self._stop.wait(1.0)
            finally:
                if stream is not None:
                    try:
                        stream.shutdown()
                    except Exception:
                        pass
                    try:
                        stream.close()
                    except Exception:
                        pass


class QDrone2StackPose(_ThreadedPose):
    name = "qdrone2_stack"
    N_RX = 47

    def __init__(self, cfg):
        self.uri = "tcpip://%s:%d" % (cfg.get("qdrone2_ip", "192.168.2.15"), int(cfg.get("qdrone2_port", 18491)))
        idx = cfg.get("qdrone2_indices")
        self.indices = [int(i) for i in idx] if idx else None
        self.yaw_scale = float(cfg.get("qdrone2_yaw_scale", 1.0))
        self.tx = np.zeros(1, dtype=np.float64)
        self.rx = np.zeros(self.N_RX, dtype=np.float64)
        self.t0 = time.time()
        super().__init__()

    def _connect(self):
        from quanser.communications import Stream
        s = Stream()
        s.connect(self.uri, False, 8, self.N_RX * 8)
        return s

    def _receive_once(self, stream):
        # Igual que QDrone2StreamStack.read(): manda la marca de tiempo y recibe 47 doubles.
        self.tx[0] = time.time() - self.t0
        stream.send_byte_array(self.tx.tobytes(), 8)
        stream.flush()
        buf = bytearray(self.N_RX * 8)
        if stream.receive_byte_array(buf, len(buf)) != 1:
            raise RuntimeError("El QDrone2 cerro la conexion")
        self.rx = np.frombuffer(bytes(buf), dtype=np.float64).copy()
        self.raw = [round(float(v), 4) for v in self.rx]
        if self.indices:
            p = self.rx[self.indices].astype(float)
            p[3] *= self.yaw_scale
            self._latest = p
        self._stamp = time.time()
        time.sleep(0.01)

    def read(self):
        if not self.indices:
            self.error = "Configura pose.qdrone2_indices ([iX,iY,iZ,iYaw]) usando el vector crudo"
            return NAN4.copy()
        return super().read()


class StreamPose(_ThreadedPose):
    name = "stream"

    def __init__(self, cfg):
        self.uri = cfg.get("stream_uri", "tcpip://localhost:18700")
        super().__init__()

    def _connect(self):
        from quanser.communications import Stream
        s = Stream()
        s.connect(self.uri, False, 1460, 4 * 8 * 64)
        return s

    def _receive_once(self, stream):
        buf = np.zeros(4, dtype=np.float64)
        if stream.receive_double_array(buf, 4) != 1:
            raise RuntimeError("El servidor de pose cerro la conexion")
        self._latest, self._stamp = buf.copy(), time.time()
        self.raw = [round(float(v), 4) for v in buf]


POSE_SOURCES = {c.name: c for c in (SimPose, QDrone2StackPose, StreamPose)}


def make_pose_source(cfg):
    name = cfg.get("source", "sim")
    if name not in POSE_SOURCES:
        raise ValueError(f"Fuente de pose desconocida: {name}. Opciones: {list(POSE_SOURCES)}")
    return POSE_SOURCES[name](cfg)


class SetpointServer:
    """Publica cada ciclo 6 doubles: [X, Y, Z, Yaw, startCamera, fuegoDetectado].

    Un modelo QUARC con un bloque *Stream Client* (o cualquier script Python) puede
    conectarse a ``uri`` y usar el setpoint para mandar al QDrone2 a cada waypoint.
    """

    def __init__(self, cfg):
        self.enabled = bool(cfg.get("enabled", False))
        self.uri = cfg.get("uri", "tcpip://localhost:18600")
        self.clients = []
        self.error = ""
        self._lock = threading.Lock()
        if self.enabled:
            threading.Thread(target=self._accept_loop, daemon=True, name="setpoint-server").start()

    def _accept_loop(self):
        try:
            from quanser.communications import Stream
            server = Stream()
            # No bloqueante: si un cliente deja de leer, el lazo de mision nunca se detiene.
            server.listen(self.uri, True)
            while True:
                client = server.accept(6 * 8 * 16, 1460)
                if client is None:
                    time.sleep(0.2)
                    continue
                with self._lock:
                    self.clients.append(client)
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"

    def publish(self, setpoint, start_camera, fire):
        if not self.enabled:
            return
        data = np.array(list(setpoint) + [float(start_camera), float(fire)], dtype=np.float64)
        with self._lock:
            alive = []
            for c in self.clients:
                try:
                    c.send_double_array(data, 6)
                    c.flush()
                    alive.append(c)
                except Exception:
                    try:
                        c.close()
                    except Exception:
                        pass
            self.clients = alive

    def status(self):
        return {"enabled": self.enabled, "uri": self.uri, "clients": len(self.clients), "error": self.error}
