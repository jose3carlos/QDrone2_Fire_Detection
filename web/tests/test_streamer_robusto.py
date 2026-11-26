"""Video que se recupera solo (el Wi-Fi del dron es intermitente).

Lado dron: ``drone/qdrone2_camera_streamer.py`` (el real del repo) con fuentes falsas en 127.0.0.1: una fuente que
falla N veces (al abrir, al leer, frames vacíos) y luego se recupera; un cliente que desaparece sin leer (y otro que
cierra con RST) no tumba el streamer ni frena a la estación; un cliente nuevo no se queda atascado en el viejo.
Lado estación: ``cameras.CameraHub`` + ``QDF1Client`` contra el streamer real (matarlo y relanzarlo) y contra
servidores falsos (corte a mitad de frame, servidor que acepta y no manda nada, corte que deja el socket colgado):
recupera el video en < ~3 s, los contadores son correctos y los Eventos tienen antirrebote. Por último la estación
completa: ``snapshot.camara``, Eventos, caídas en el vuelo y la muestra de ~1 Hz de la caja negra.

Todo en 127.0.0.1 (nunca la red de los drones).
"""
import os
import queue
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
import types

import numpy as np
import pytest

from app import protocol as P
from app.cameras import CAIDA_MIN_S, ESPERA_MAX_S, CameraHub, QDF1Client
from test_solo_explorador import _set, _station, _tel_modo

DRONE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "drone")
STREAMER = os.path.join(DRONE, "qdrone2_camera_streamer.py")
sys.path.insert(0, DRONE)
import qdrone2_camera_streamer as streamer  # noqa: E402


# =============================================================================== utilidades
def _puerto_libre():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _esperar(cond, timeout, paso=0.02):
    fin = time.time() + timeout
    while time.time() < fin:
        if cond():
            return True
        time.sleep(paso)
    return bool(cond())


def _hub(port, eventos, **kw):
    h = CameraHub(lambda: ("dron", "127.0.0.1", port), lambda k: QDF1Client(k[1], k[2]), evento_fn=eventos.append, **kw)
    h.start()
    return h


class _Proceso:
    """El streamer real en un subproceso (``--camara sintetica``); guarda su salida en ``lineas``."""

    def __init__(self, port):
        env = dict(os.environ, PYTHONIOENCODING="utf-8")
        self.p = subprocess.Popen([sys.executable, "-u", STREAMER, "--camara", "sintetica", "--host", "127.0.0.1",
                                   "--puerto", str(port), "--fps", "30", "--fps-envio", "30", "--cada-estado", "0"],
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env)
        self.lineas, self._q = [], queue.Queue()
        threading.Thread(target=self._leer, daemon=True).start()
        fin = time.time() + 20
        while time.time() < fin:
            try:
                linea = self._q.get(timeout=0.5)
            except queue.Empty:
                if self.p.poll() is not None:
                    break
                continue
            if "Esperando a la PC" in linea:
                self.listo = time.time()
                return
        self.matar()
        raise AssertionError("el streamer no arrancó:\n" + "".join(self.lineas))

    def _leer(self):
        for b in iter(self.p.stdout.readline, b""):
            linea = b.decode("utf-8", "replace")
            self.lineas.append(linea)
            self._q.put(linea)

    def matar(self):
        if self.p.poll() is None:
            self.p.kill()
        self.p.wait(timeout=10)


FRAME = streamer.empaquetar(np.full((48, 64, 3), 90, np.uint8), 80)


class _ServidorFalso:
    """Servidor QDF1 falso: la conexión k-ésima sigue ``planes[k]`` (el último se repite).

    Planes: ("bien",) manda frames a ~50 fps; ("corte", n) n frames y medio frame y cierra; ("cuelga", n) n frames y
    medio frame y deja el socket abierto sin mandar nada; ("mudo",) acepta y no manda nada. ``pausa()`` cierra todo
    (como un dron fuera del Wi-Fi o un streamer muerto) y ``reanudar()`` vuelve a escuchar en el mismo puerto."""

    def __init__(self, planes):
        self.planes, self.n, self.parar = list(planes), 0, threading.Event()
        self.conns, self.cortes, self._lock = [], [], threading.Lock()
        self.port = None
        self._escuchar()

    def _escuchar(self, port=0):
        srv = socket.socket()
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", port))
        srv.listen(4)
        srv.settimeout(0.2)
        self.srv, self.port, self._activo = srv, srv.getsockname()[1], threading.Event()
        self._activo.set()
        threading.Thread(target=self._aceptar, args=(srv, self._activo), daemon=True).start()

    def _aceptar(self, srv, activo):
        while activo.is_set() and not self.parar.is_set():
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with self._lock:
                plan = self.planes[min(self.n, len(self.planes) - 1)]
                self.n += 1
                self.conns.append(conn)
            threading.Thread(target=self._servir, args=(conn, plan, activo), daemon=True).start()

    def _servir(self, conn, plan, activo):
        try:
            n = 0
            while activo.is_set() and not self.parar.is_set():
                if plan[0] == "mudo" or (plan[0] == "cuelga" and n > plan[1]):
                    time.sleep(0.05)
                    continue
                if plan[0] in ("corte", "cuelga") and n == plan[1]:
                    conn.sendall(FRAME[:HEADER_Y_MEDIO])
                    self.cortes.append(time.time())
                    n += 1
                    if plan[0] == "corte":
                        conn.close()
                        return
                    continue
                conn.sendall(FRAME)
                n += 1
                time.sleep(0.02)
        except OSError:
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def pausa(self):
        self._activo.clear()
        self.srv.close()
        with self._lock:
            conns, self.conns = self.conns, []
        for c in conns:
            try:
                c.close()
            except OSError:
                pass

    def reanudar(self):
        self._escuchar(self.port)

    def cerrar(self):
        self.parar.set()
        self.pausa()


HEADER_Y_MEDIO = streamer.HEADER.size + (len(FRAME) - streamer.HEADER.size) // 2


# =============================================================================== lado estación
def test_estacion_recupera_video_al_matar_y_relanzar_el_streamer():
    port, eventos = _puerto_libre(), []
    p1 = _Proceso(port)
    p2 = None
    hub = _hub(port, eventos)
    try:
        assert _esperar(lambda: hub.frame_id >= 5, 10)
        s = hub.status()
        assert (s["conexiones"], s["reconexiones"], s["caidas"], s["atascos"]) == (1, 0, 0, 0)
        assert s["conectada"] and s["caida_s"] is None and s["edad_s"] < 1.0
        p1.matar()
        t_muerto = time.time()
        assert _esperar(lambda: any("Video caído" in e for e in eventos), 5)
        assert time.time() - t_muerto >= CAIDA_MIN_S - 0.2        # antirrebote: no antes de ~1 s
        time.sleep(2.5)                                            # sin streamer: la espera crece hasta el tope
        s = hub.status()
        assert s["caidas"] == 1 and s["fallos"] >= 1 and not s["conectada"] and s["caida_s"] >= 2.5
        assert s["espera_s"] <= ESPERA_MAX_S and s["frames"] == hub.frame_id
        n0 = hub.frame_id
        p2 = _Proceso(port)
        assert _esperar(lambda: hub.frame_id > n0, 6)
        dt = time.time() - p2.listo
        assert dt < 3.0, f"tardó {dt:.2f} s en recuperar el video"
        assert _esperar(lambda: any("Video de vuelta" in e for e in eventos), 5)
        s = hub.status()
        assert s["caidas"] == 1 and s["reconexiones"] == 1 and s["conexiones"] == 2 and s["caida_s"] is None
        assert s["ultima_caida"]["dur_s"] > 2.5 and s["espera_s"] == hub.espera_min
        assert sum("Video caído" in e for e in eventos) == 1 and sum("Video de vuelta" in e for e in eventos) == 1
        assert "1 reconexión" in [e for e in eventos if "Video de vuelta" in e][0]
    finally:
        hub.stop()
        p1.matar()
        if p2 is not None:
            p2.matar()


def test_corte_a_mitad_de_frame_reconecta_sin_llenar_eventos():
    srv, eventos = _ServidorFalso([("corte", 8), ("bien",)]), []
    hub = _hub(srv.port, eventos)
    hub.iniciar_vuelo()
    try:
        assert _esperar(lambda: srv.cortes, 5)
        n0 = hub.frame_id
        assert _esperar(lambda: hub.frame_id > n0 + 3, 3)
        assert time.time() - srv.cortes[0] < 1.0                  # cerró: reconecta en la primera espera (0.25 s)
        time.sleep(1.5)
        s = hub.status()
        assert (s["conexiones"], s["reconexiones"], s["caidas"], s["caidas_vuelo"], s["atascos"]) == (2, 1, 1, 1, 0)
        assert s["ultima_caida"]["dur_s"] < CAIDA_MIN_S and "cerró" in s["ultima_caida"]["motivo"]
        assert eventos == []                                       # corte breve: solo contadores (antirrebote)
        hub.iniciar_vuelo()                                        # vuelo nuevo con video: vuelve a 0
        assert hub.status()["caidas_vuelo"] == 0 and hub.status()["caidas"] == 1
    finally:
        hub.stop()
        srv.cerrar()


def test_servidor_que_acepta_y_no_manda_nada():
    srv, eventos = _ServidorFalso([("mudo",), ("bien",)]), []
    t0 = time.time()
    hub = _hub(srv.port, eventos)
    try:
        assert _esperar(lambda: hub.frame_id > 0, 5)
        dt = time.time() - t0
        assert 1.8 < dt < 3.0, dt                                  # ATASCO_S = 2 s y reconecta
        s = hub.status()
        assert (s["conexiones"], s["atascos"], s["caidas"], s["fallos"]) == (2, 1, 0, 0)   # nunca hubo video: no es caída
        assert eventos == []
    finally:
        hub.stop()
        srv.cerrar()


def test_corte_que_deja_el_socket_colgado():
    srv, eventos = _ServidorFalso([("cuelga", 8), ("bien",)]), []
    hub = _hub(srv.port, eventos)
    try:
        assert _esperar(lambda: srv.cortes, 5)
        n0 = hub.frame_id
        assert _esperar(lambda: hub.frame_id > n0, 5)
        dt = time.time() - srv.cortes[0]
        assert dt < 3.0, dt
        assert _esperar(lambda: any("Video de vuelta" in e for e in eventos), 4)
        s = hub.status()
        assert (s["conexiones"], s["caidas"], s["atascos"]) == (2, 1, 1)
        assert [e.split(" (")[0] for e in eventos] == ["Video caído", "Video de vuelta"]
        assert "FlujoAtascado" in eventos[0]
    finally:
        hub.stop()
        srv.cerrar()


def test_enlace_lento_sigue_dando_video():
    """Wi-Fi degradado: cada frame tarda ~3x el plazo de atasco en llegar, pero los bytes no paran. El atasco se mide
    por progreso (no por frame completo): hay video lento, sin atascos ni reconexiones."""
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    parar, trozo = threading.Event(), len(FRAME) // 9 + 1

    def servir():
        conn, _ = srv.accept()
        try:
            while not parar.is_set():
                for i in range(0, len(FRAME), trozo):
                    conn.sendall(FRAME[i:i + trozo])
                    time.sleep(0.1)                                  # ~0.9 s por frame, atasco_s = 0.3 s
        except OSError:
            pass
        finally:
            conn.close()
    threading.Thread(target=servir, daemon=True).start()
    port, eventos = srv.getsockname()[1], []
    hub = CameraHub(lambda: ("dron", "127.0.0.1", port), lambda k: QDF1Client(k[1], k[2], atasco_s=0.3),
                    evento_fn=eventos.append)
    hub.start()
    try:
        assert _esperar(lambda: hub.frame_id >= 3, 6), hub.status()
        s = hub.status()
        assert (s["conexiones"], s["atascos"], s["caidas"]) == (1, 0, 0) and eventos == []
    finally:
        parar.set()
        hub.stop()
        srv.close()


# =============================================================================== lado dron
class _CamFalsa:
    def __init__(self, n_buenos, luego):
        self.n_buenos, self.luego, self.n, self.cerrada = n_buenos, luego, 0, False

    def read(self):
        time.sleep(0.02)
        self.n += 1
        if self.n_buenos is None or self.n <= self.n_buenos:
            return np.full((48, 64, 3), self.n % 250, np.uint8)
        if self.luego == "error":
            raise OSError("se desconectó la cámara")
        return None if self.luego == "none" else np.zeros((0, 0, 3), np.uint8)

    def close(self):
        self.cerrada = True


def _args(*extra):
    return streamer.argumentos(["--host", "127.0.0.1", "--puerto", "0", "--fps-envio", "50", "--cada-estado", "0",
                                "--espera-max", "1"] + list(extra))


def test_streamer_reabre_la_camara_que_falla_y_la_estacion_ni_se_entera():
    aperturas, camaras = [], []
    plan = [None, None, (15, "error"), (15, "none"), (15, "vacio"), (None, None)]   # None = no abre

    def abrir(preferida):
        k = len(aperturas)
        aperturas.append(preferida)
        p = plan[min(k, len(plan) - 1)]
        if p is None:
            raise RuntimeError("cámara ocupada")
        cam = _CamFalsa(*p)
        cam.clave = "falsa"
        camaras.append(cam)
        return cam

    st = streamer.Streamer(_args("--vacios-s", "0.5"), abrir_fn=abrir).iniciar()
    th = threading.Thread(target=st.correr, daemon=True)
    th.start()
    eventos = []
    hub = None
    try:
        assert _esperar(lambda: st.captura.frames > 0, 5)
        hub = _hub(st.puerto, eventos)
        assert _esperar(lambda: len(camaras) >= 4 and hub.frame_id > 0, 10)
        n0 = hub.frame_id
        assert _esperar(lambda: hub.frame_id > n0 + 20, 5)        # la última cámara entrega frames
        e = st.estado()
        assert e["fallos_apertura"] == 2 and e["reaperturas"] == 3 and e["fallos_lectura"] == 3
        assert e["camara_abierta"] and all(c.cerrada for c in camaras[:-1])
        assert aperturas[3:] == ["falsa"] * (len(aperturas) - 3)    # reabre primero la que funcionó
        s = hub.status()
        assert s["conexiones"] == 1 and s["caidas"] == 0 and s["atascos"] == 0   # huecos < 2 s: la PC no reconecta
        assert th.is_alive()
    finally:
        if hub is not None:
            hub.stop()
        st.detener()
        th.join(timeout=5)


def test_streamer_sobrevive_a_un_cliente_que_desaparece(capsys):
    rng = np.random.default_rng(1)
    ruido = [rng.integers(0, 255, (480, 640, 3), dtype=np.uint8) for _ in range(4)]   # JPEG grandes (~400 kB)

    class Ruido:
        clave, n = "ruido", 0

        def read(self):
            time.sleep(0.03)
            self.n += 1
            return ruido[self.n % 4]

        def close(self):
            pass

    st = streamer.Streamer(_args("--timeout-envio", "0.5", "--max-clientes", "3"), abrir_fn=lambda p: Ruido()).iniciar()
    th = threading.Thread(target=st.correr, daemon=True)
    th.start()
    hub, eventos = None, []
    muerto = socket.socket()
    try:
        # cliente A: buffer mínimo, lee un encabezado y deja de leer sin cerrar (PC que se fue del Wi-Fi)
        muerto.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
        muerto.settimeout(5)
        muerto.connect(("127.0.0.1", st.puerto))
        muerto.recv(streamer.HEADER.size)
        # cliente C: se conecta y cierra con RST
        rst = socket.create_connection(("127.0.0.1", st.puerto))
        rst.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
        rst.close()
        # la estación (cliente B) recibe video mientras tanto
        hub = _hub(st.puerto, eventos)
        assert _esperar(lambda: st.estado()["desconexiones"] >= 2, 15)
        e = st.estado()
        assert e["clientes_activos"] == 1 and e["clientes_total"] == 3 and th.is_alive()
        n0 = hub.frame_id
        time.sleep(1.0)
        assert hub.frame_id - n0 >= 10                              # ~30 fps: el cliente muerto no la frena
        s = hub.status()
        assert s["conexiones"] == 1 and s["caidas"] == 0
        out = capsys.readouterr().out
        assert "la PC no recibe" in out
    finally:
        muerto.close()
        if hub is not None:
            hub.stop()
        st.detener()
        th.join(timeout=5)


def test_cliente_nuevo_no_se_atasca_en_el_viejo():
    st = streamer.Streamer(_args("--max-clientes", "1"), abrir_fn=lambda p: _CamFalsa(None, None)).iniciar()
    th = threading.Thread(target=st.correr, daemon=True)
    th.start()
    viejo = socket.create_connection(("127.0.0.1", st.puerto))
    hub, eventos = None, []
    try:
        viejo.settimeout(3)
        assert viejo.recv(streamer.HEADER.size)                    # el viejo recibe video
        hub = _hub(st.puerto, eventos)
        assert _esperar(lambda: hub.frame_id >= 5, 3)              # el nuevo entra de inmediato
        datos = b"x"
        fin = time.time() + 3
        while datos and time.time() < fin:                          # al viejo se le cierra el socket
            try:
                datos = viejo.recv(65536)
            except OSError:
                datos = b""
        assert datos == b""
        assert st.estado()["clientes_activos"] == 1
    finally:
        viejo.close()
        if hub is not None:
            hub.stop()
        st.detener()
        th.join(timeout=5)


def test_vigia_sale_con_codigo_3_si_la_camara_se_cuelga():
    bloqueo = threading.Event()

    class Colgada:
        clave = "colgada"

        def read(self):
            bloqueo.wait(10)
            return None

        def close(self):
            pass

    st = streamer.Streamer(_args("--vigia-s", "0.5"), abrir_fn=lambda p: Colgada()).iniciar()
    try:
        t0 = time.time()
        assert st.correr() == 3 and time.time() - t0 < 4
    finally:
        bloqueo.set()
        st.detener()


def test_vigia_cubre_el_cierre_colgado_y_el_log_deja_la_causa(monkeypatch):
    """read() falla (USB) y close() se cuelga en la librería (stop_streaming): también sale con código 3, y la causa
    queda en el log antes del cierre."""
    bloqueo, lineas = threading.Event(), []
    monkeypatch.setattr(streamer, "log", lineas.append)

    class CierreColgado:
        clave = "colgada"

        def __init__(self):
            self.n = 0

        def read(self):
            self.n += 1
            if self.n > 5:
                raise OSError("USB desconectada")
            time.sleep(0.01)
            return np.zeros((48, 64, 3), np.uint8)

        def close(self):
            bloqueo.wait(10)

    st = streamer.Streamer(_args("--vigia-s", "0.5"), abrir_fn=lambda p: CierreColgado()).iniciar()
    try:
        t0 = time.time()
        assert st.correr() == 3 and time.time() - t0 < 4
        assert st.captura.ocupado_en == "cierre"
        causa = [i for i, m in enumerate(lineas) if "falló la lectura" in m and "USB desconectada" in m]
        vigia = [i for i, m in enumerate(lineas) if "colgada en cierre" in m and "código 3" in m]
        assert causa and vigia and causa[0] < vigia[0], lineas
    finally:
        bloqueo.set()
        st.detener()


def test_sin_camara_sale_con_codigo_4():
    """La cámara no vuelve (p. ej. 'device busy' por handles que la librería dejó en el proceso): tras
    ``--sin-camara-s`` sale con código 4 para que correr_streamer.sh relance un proceso limpio."""
    def no_abre(pref):
        raise RuntimeError("device busy")

    st = streamer.Streamer(_args("--sin-camara-s", "1"), abrir_fn=no_abre).iniciar()
    try:
        t0 = time.time()
        assert st.correr() == 4 and 1.0 <= time.time() - t0 < 4
        assert st.captura.fallos_apertura >= 2
    finally:
        st.detener()


def test_con_frames_no_sale_por_sin_camara():
    st = streamer.Streamer(_args("--sin-camara-s", "0.5", "--max-frames", "60"),
                           abrir_fn=lambda p: _CamFalsa(None, None)).iniciar()
    hub = _hub(st.puerto, [])
    try:
        assert st.correr() == 0                                       # termina por --max-frames (~2 s), no por el 4
    finally:
        hub.stop()
        st.detener()


def test_senal_detiene_el_streamer():
    """El manejador de SIGTERM solo pone ``st.senal`` (sin log() ni locks, que podrían estar tomados por el mismo
    hilo principal): correr() lo ve y sale en <= 0.5 s cerrando la cámara."""
    cams = []

    def abrir(pref):
        cams.append(_CamFalsa(None, None))
        return cams[-1]
    st = streamer.Streamer(_args(), abrir_fn=abrir).iniciar()
    threading.Timer(0.5, lambda: setattr(st, "senal", 15)).start()
    t0 = time.time()
    assert st.correr() == 0 and time.time() - t0 < 2.0
    assert not st.captura.is_alive() and cams and cams[-1].cerrada


def test_quanser_no_deja_handles_si_falla_abrir_o_cerrar(monkeypatch):
    """RealSenseQuanser: si stream_open() falla al reabrir, cierra el Video3D; si stop_streaming() lanza al cerrar,
    igual cierra el stream y el Video3D (si no, las reaperturas del mismo proceso dan 'device busy')."""
    llamadas, falla = [], {"abrir": True}

    class Stream(object):
        def close(self):
            llamadas.append("s.close")

    class Video3D(object):
        def __init__(self, serie):
            llamadas.append("abre")

        def stream_open(self, *args):
            if falla["abrir"]:
                raise RuntimeError("el dispositivo se está reiniciando")
            return Stream()

        def start_streaming(self):
            llamadas.append("start")

        def stop_streaming(self):
            llamadas.append("stop")
            raise RuntimeError("USB")

        def close(self):
            llamadas.append("v.close")

    mod = types.ModuleType("quanser.multimedia")
    mod.Video3D = Video3D
    mod.Video3DStreamType = types.SimpleNamespace(COLOR=0)
    mod.ImageFormat = types.SimpleNamespace(ROW_MAJOR_INTERLEAVED_BGR=0)
    mod.ImageDataType = types.SimpleNamespace(UINT8=0)
    monkeypatch.setitem(sys.modules, "quanser", types.ModuleType("quanser"))
    monkeypatch.setitem(sys.modules, "quanser.multimedia", mod)
    a = _args("--camara", "quanser")
    with pytest.raises(RuntimeError, match="reiniciando"):
        streamer.RealSenseQuanser(a)
    assert llamadas[0] == "abre" and llamadas[-1] == "v.close"
    del llamadas[:]
    falla["abrir"] = False
    cam = streamer.RealSenseQuanser(a)
    with pytest.raises(RuntimeError, match="USB"):
        cam.close()
    assert llamadas == ["abre", "start", "stop", "s.close", "v.close"]


def _bash():
    """bash para probar correr_streamer.sh (en Windows, el de Git: el de System32 es WSL)."""
    if os.name == "nt":
        for c in (r"C:\Program Files\Git\bin\bash.exe", r"C:\Program Files\Git\usr\bin\bash.exe"):
            if os.path.exists(c):
                return c
        return None
    return shutil.which("bash")


@pytest.mark.skipif(_bash() is None, reason="sin bash")
def test_pid_bucle_ignora_un_pid_ajeno(tmp_path):
    """~/.streamer_camara.pid sobrevive a un apagado: si el PID guardado ya es otro proceso, --estado dice que no
    corre, el arranque no responde 'Ya corre' y --detener no lo mata. Uno que sí es el script se reconoce."""
    script = os.path.join(DRONE, "correr_streamer.sh").replace(os.sep, "/")
    env = dict(os.environ, STREAMER_PID=(tmp_path / "pid").as_posix(), STREAMER_LOG=(tmp_path / "log").as_posix())
    cmd = ('[ -r /proc/$$/cmdline ] || { echo SIN_PROC; exit 0; }; '
           'sleep 5 & echo $! > "$STREAMER_PID"; '
           'bash "$1" --estado; bash "$1" --detener; kill -0 $! && echo AJENO_VIVO; kill $!; '
           'bash -c "sleep 5; :" correr_streamer.sh & echo $! > "$STREAMER_PID"; sleep 0.3; '
           'bash "$1" --estado; kill $!')
    r = subprocess.run([_bash(), "-c", cmd, "prueba", script], env=env, stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, timeout=30)
    out = r.stdout.decode("utf-8", "replace")
    if "SIN_PROC" in out:
        pytest.skip("bash sin /proc")
    assert "Streamer NO corre" in out and "No hay streamer corriendo" in out and "AJENO_VIVO" in out, out
    assert "Streamer corriendo (bucle PID" in out, out


# =============================================================================== estación completa
def test_estacion_snapshot_eventos_y_caidas_en_el_vuelo(cfg_path):
    srv = _ServidorFalso([("bien",)])

    def fn(c):
        c["camara"]["fuente"], c["camara"]["dron"] = "dron", 1
        c["red"]["ip_dron1"], c["red"]["puerto_video"] = "127.0.0.1", srv.port
    _set(cfg_path, fn)
    st = _station(cfg_path, _tel_modo(ms=P.ASSIGN_ROLES))
    st.camera.start()
    msgs = lambda: [e["msg"] for e in st.events]   # noqa: E731
    try:
        assert _esperar(lambda: st.camera.frame_id >= 5, 5)
        c = st.snapshot()["camara"]
        assert c["fuente"] == ["dron", "127.0.0.1", srv.port] or tuple(c["fuente"]) == ("dron", "127.0.0.1", srv.port)
        assert (c["conexiones"], c["reconexiones"], c["caidas"], c["caidas_vuelo"]) == (1, 0, 0, 0)
        assert c["edad_s"] is not None and c["edad_s"] < 1.0 and not c["en_vuelo"]
        st._on_telemetry(_tel_modo(ms=P.MISSION), time.time())
        assert st.camera.en_vuelo
        srv.pausa()                                                  # el dron se va del Wi-Fi
        assert _esperar(lambda: any(m.startswith("Video caído (dron 127.0.0.1:") for m in msgs()), 5)
        c = st.snapshot()["camara"]
        assert c["caidas"] == 1 and c["caidas_vuelo"] == 1 and c["caida_s"] is not None
        assert st._muestra_caja()["camara"]["caidas_vuelo"] == 1   # la muestra de ~1 Hz de la caja negra
        srv.reanudar()
        assert _esperar(lambda: any(m.startswith("Video de vuelta") for m in msgs()), 6)
        c = st.snapshot()["camara"]
        assert (c["caidas"], c["caidas_vuelo"], c["reconexiones"]) == (1, 1, 1) and c["caida_s"] is None
        st._on_telemetry(_tel_modo(ms=P.DONE), time.time())          # fin del vuelo: se conserva el número
        assert not st.camera.en_vuelo and st.snapshot()["camara"]["caidas_vuelo"] == 1
        st._on_telemetry(_tel_modo(ms=P.TAKEOFF), time.time())       # vuelo nuevo: vuelve a contar
        assert st.snapshot()["camara"]["caidas_vuelo"] == 0
        assert sum(m.startswith("Video caído") for m in msgs()) == 1
    finally:
        st.camera.stop()
        srv.cerrar()


def test_caja_negra_resume_las_caidas_del_video(tmp_path, capsys):
    """Los contadores de snapshot.camara viajan en la muestra de ~1 Hz y herramientas/caja_negra.py los resume."""
    from app.caja_negra import CajaNegra
    from app.repeticion import Grabacion
    from test_caja_negra import _Reloj, _alimentar, _paquete, herr

    r, cam = _Reloj(), {"fps": 11.0, "edad_s": 0.1, "caidas": 2, "reconexiones": 3, "atascos": 0, "caidas_vuelo": 0,
                        "caida_s": None}
    caja = CajaNegra(tmp_path / "vuelos", lambda: "simulink", lambda: True, muestra_fn=lambda: {"camara": dict(cam)},
                     reloj=r, hilo=False)
    _alimentar(caja, r, [_paquete(k * 0.02, P.ASSIGN_ROLES) for k in range(100)])
    caja.comando("START", "web 127.0.0.1", "enviado", 1)
    _alimentar(caja, r, [_paquete(2 + k * 0.02, P.MISSION, d1=(-0.9, 0.4, 1.0)) for k in range(100)])
    cam.update(caidas=3, reconexiones=5, atascos=1, caidas_vuelo=1, caida_s=2.4, fps=0.0)   # corte en el vuelo
    _alimentar(caja, r, [_paquete(4 + k * 0.02, P.MISSION, d1=(-0.9, 0.4, 1.0)) for k in range(60)])
    cam.update(caida_s=None, fps=11.0)
    _alimentar(caja, r, [_paquete(5.2 + k * 0.02, P.DONE) for k in range(600)])
    caja.vaciar()
    (f,) = [os.path.join(d, n) for d, _, ns in os.walk(tmp_path / "vuelos") for n in ns if n.endswith(".tel.jsonl.gz")]
    v = herr.analizar(Grabacion(f))["video"]
    assert (v["caidas"], v["reconexiones"], v["atascos"], v["caidas_vuelo"], v["caida_max_s"]) == (1, 2, 1, 1, 2.4)
    assert herr.main(["resumen", f]) == 0
    assert "enlace: 1 caídas en la grabación (1 en el vuelo) · 2 reconexiones · 1 atascos" in capsys.readouterr().out
