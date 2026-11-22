"""Cliente del puente contra un servidor TCP falso (heartbeat 20 Hz, cmd_seq, ranuras, reconexión)."""
import socket
import threading
import time

from app import protocol as P
from app.bridge import BridgeClient


class FakeSimulink:
    """Stream Server mínimo: manda telemetría a 50 Hz y guarda todo lo que recibe."""

    def __init__(self, port=0):
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", port))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.rx = []
        self.ack = 0
        self.acking = True       # False = Simulink que no contesta (para probar la caducidad)
        self.stop_ev = threading.Event()
        self.conn = None
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        self.srv.settimeout(0.2)
        while not self.stop_ev.is_set():
            try:
                conn, _ = self.srv.accept()
            except (socket.timeout, OSError):
                continue
            self.conn = conn
            conn.settimeout(0.02)
            buf = b""
            t_next = time.time()
            try:
                while not self.stop_ev.is_set():
                    try:
                        chunk = conn.recv(4096)
                        if not chunk:
                            break
                        buf += chunk
                        while len(buf) >= 256:
                            m = P.unpack_command(buf[:256])
                            buf = buf[256:]
                            self.rx.append((time.time(), m))
                            if self.acking and m["cmd_seq"] != self.ack:   # como qd2_mission_step
                                self.ack = m["cmd_seq"]
                    except socket.timeout:
                        pass
                    if time.time() >= t_next:
                        t_next += 0.02
                        conn.sendall(P.pack_telemetry({"t": time.time(), "mission_state": 5, "cmd_seq_ack": self.ack,
                                                       "explorer_id": 1, "responder_id": 2, "drones": {}}))
            except OSError:
                pass
            finally:
                conn.close()

    def kill_connection(self):
        if self.conn:
            self.conn.close()

    def close(self):
        self.stop_ev.set()
        self.srv.close()


def wait(cond, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_heartbeat_comandos_y_ranuras():
    sim = FakeSimulink()
    br = BridgeClient("127.0.0.1", sim.port, params_fn=lambda: {"z_explorer": 1.5, "d_min_h": 0.9}).start()
    try:
        assert wait(lambda: br.fresh() and len(sim.rx) > 5)
        tel, _ = br.latest()
        assert tel["mission_state_name"] == "MISSION"
        n0, t0 = len(sim.rx), time.time()
        time.sleep(1.0)
        rate = (len(sim.rx) - n0) / (time.time() - t0)
        assert 17 <= rate <= 23, rate                         # 20 Hz
        hbs = [m["heartbeat"] for _, m in sim.rx]
        assert all(b == a + 1 for a, b in zip(hbs, hbs[1:]))   # contador creciente sin saltos
        assert sim.rx[-1][1]["z_explorer"] == 1.5 and sim.rx[-1][1]["d_min_h"] == 0.9

        seq = br.send_command("START")
        assert seq == 1
        assert wait(lambda: sim.ack == 1)
        assert wait(lambda: br.status()["cmd_seq_ack"] == 1)
        # tras el ack se sigue mandando el mismo cmd_seq pero con cmd = 0
        assert wait(lambda: sim.rx[-1][1]["cmd"] == 0 and sim.rx[-1][1]["cmd_seq"] == 1)
        assert any(m["cmd"] == P.CMD_START and m["cmd_seq"] == 1 for _, m in sim.rx)
        assert br.send_command("PAUSE") == 2
        assert wait(lambda: sim.ack == 2)

        # 5 objetivos: en pocos mensajes deben aparecer todos en las ranuras
        br.set_targets([(i, i % 2, 0.1 * i, -0.1 * i, 0.8) for i in range(5, 0, -1)])
        k0 = len(sim.rx)
        assert wait(lambda: len(sim.rx) > k0 + 6)
        ids = {s["id"] for _, m in sim.rx[k0 + 1:] for s in m["slots"] if s["id"]}
        assert ids == {1, 2, 3, 4, 5}
        assert all(m["n_targets"] == 5 for _, m in sim.rx[k0 + 1:])
        slot = [s for _, m in sim.rx[k0 + 1:] for s in m["slots"] if s["id"] == 3][0]
        assert slot["cls"] == 1 and abs(slot["x"] - 0.3) < 1e-12 and abs(slot["y"] + 0.3) < 1e-12
    finally:
        br.stop()
        sim.close()


def test_reconexion():
    # sin servidor: el cliente reintenta sin morir
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    br = BridgeClient("127.0.0.1", port, reconnect_s=0.2).start()
    try:
        # en Windows un connect rechazado tarda ~2 s en fallar
        assert wait(lambda: br.status()["error"] != "", 6) and not br.status()["connected"]
        sim = FakeSimulink(port)
        assert wait(lambda: br.fresh(), 5)
        sim.kill_connection()                       # Simulink se cae / reinicia
        assert wait(lambda: br.status()["reconnects"] >= 1, 5)
        assert wait(lambda: br.fresh() and br.status()["connected"], 5)
        hb_before = br.heartbeat
        assert wait(lambda: br.heartbeat > hb_before + 5)
        sim.close()
    finally:
        br.stop()
