"""y(55) alerta, y(56:60) objetivos de Simulink, etiqueta de fsm según la versión, ids 1..32,
cmd_seq_ack con +0.5 del operador y objetivos retenidos hasta la 1.ª telemetría de cada conexión."""
import math
import socket
import threading
import time

import pytest
from fastapi.testclient import TestClient

from app import protocol as P
from app.bridge import BridgeClient
from app.geo import Clusterer
from app.mock_sim import MockMission, flight_state_tierra, FSM_AUTOLAND, FSM_FLIGHT, FSM_IDLE
from app.server import SIM_TTL_S, Station, create_app
from test_mock_mision import Driver
from test_puente import wait
from test_revision import _cfg, _free_port, _patch_cfg


def _tel(**kw):
    """Telemetría empaquetada y desempaquetada (pasa por los 512 bytes reales)."""
    base = {"t": 10.0, "mission_state": P.MISSION, "version": 1, "explorer_id": 1, "responder_id": 2,
            "drones": {1: {"tracked": 1, "fsm_state": 2}, 2: {"tracked": 1, "fsm_state": 2}}}
    base.update(kw)
    return P.unpack_telemetry(P.pack_telemetry(base))


# =============================================================================== protocolo
def test_alerta_etiquetas_0_a_10():
    assert set(P.ALERTAS) == set(range(11)) and P.ALERTA_MAX == 10
    esperado = {1: "Homes", 2: "rastreo", 3: "despegue", 4: "aterrizaje", 5: "E-STOP", 6: "Takeoff",
                7: "Arm", 8: "Respaldo", 9: "a bordo", 10: "Protocolo MPC"}
    for code, palabra in esperado.items():
        tel = _tel(alerta=code)
        assert tel["alerta"] == code
        assert palabra.lower() in tel["alerta_texto"].lower()
        assert tel["alerta_nivel"] in ("aviso", "grave")
    assert _tel(alerta=0)["alerta"] == 0 and _tel()["alerta"] == 0
    assert _tel(alerta=5)["alerta_nivel"] == "grave" and _tel(alerta=1)["alerta_nivel"] == "aviso"
    desconocida = _tel(alerta=42)
    assert desconocida["alerta_nivel"] == "grave" and "42" in desconocida["alerta_texto"]


def test_objetivo_sim_ida_y_vuelta_y_validacion():
    ob = {"id": 7, "cls": 1, "x": -0.42, "y": 1.1, "estado": 2}
    tel = _tel(objetivo_sim=ob)
    assert tel["objetivo_sim"] == pytest.approx(ob)
    # mismos índices base 1 que el contrato: 56 id, 57 clase, 58 x, 59 y, 60 estado
    raw = P._TEL.unpack(P.pack_telemetry({"objetivo_sim": ob}))
    assert raw[55:60] == (7.0, 1.0, -0.42, 1.1, 2.0)
    assert _tel()["objetivo_sim"] is None                                  # ranura vacía (id 0)
    for malo in ({"id": 33}, {"id": 0}, {"cls": 2}, {"estado": 0}, {"estado": 4}, {"x": float("nan")}):
        assert _tel(objetivo_sim={**ob, **malo})["objetivo_sim"] is None, malo


def test_cmd_seq_ack_no_se_redondea():
    tel = _tel(cmd_seq_ack=4)
    assert tel["cmd_seq_ack"] == 4 and isinstance(tel["cmd_seq_ack"], int) and not tel["ack_operador"]
    op = _tel(cmd_seq_ack=4.5)      # OPERATOR COMMAND MERGE: cmd_seq + 0.5 durante 1 ms
    assert op["cmd_seq_ack"] == 4.5 and op["ack_operador"]


def test_cmd_seq_ack_mas_medio_no_confirma_otro_comando():
    """Con ack = seq_previo + 0.5 la estación no debe dar por recibido el comando en curso."""
    br = BridgeClient("127.0.0.1", _free_port())
    br.telemetry, br.stamp = _tel(cmd_seq_ack=0), time.time()
    seq = br.send_command("START")
    assert seq == 1
    br.telemetry, br.stamp = _tel(cmd_seq_ack=0.5), time.time() + 0.01   # round(0.5) = 0, int = 0: no es 1
    br.build_message()
    assert br.status()["cmd"] == "START" and not br.cmd_results
    br.telemetry, br.stamp = _tel(cmd_seq_ack=1.5), time.time() + 0.02   # round(1.5) = 2 ≠ 1, pero 1.5 ≠ 1
    br.build_message()
    assert br.status()["cmd"] == "START" and not br.cmd_results
    br.telemetry, br.stamp = _tel(cmd_seq_ack=1), time.time() + 0.03
    br.build_message()
    assert br.cmd_results[-1]["estado"] == "recibido"


def test_fsm_segun_version():
    # PID: Flight State de tierra del Equipo2
    pid = _tel(version=1, mission_state=P.DONE, drones={1: {"fsm_state": 0}, 2: {"fsm_state": 9}})
    assert pid["drones"][1]["fsm_tipo"] == "Flight State (tierra)" and "Idle" in pid["drones"][1]["fsm_texto"]
    # 9 MS Stop después de aterrizar es el fin normal (pid/README: "se queda en 3/9 después de un vuelo")
    assert "MS Stop" in pid["drones"][2]["fsm_texto"] and not pid["drones"][2]["fsm_falla"]
    assert P.fsm_info(1, 2)[1] == "Despegue / en vuelo" and P.fsm_info(1, 3)[1] == "Aterrizaje (Land)"
    assert not P.fsm_info(1, 2)[2]
    # MPC: FSM a bordo; 7 y 8 siempre son falla, fuera de 1..9 también
    mpc = _tel(version=2, drones={1: {"fsm_state": 5}, 2: {"fsm_state": 7}})
    assert mpc["drones"][1]["fsm_tipo"] == "FSM a bordo" and mpc["drones"][1]["fsm_texto"] == "VUELO"
    assert not mpc["drones"][1]["fsm_falla"] and mpc["drones"][2]["fsm_falla"]
    assert "fuera de rango" in P.fsm_info(2, 12)[1] and P.fsm_info(2, 12)[2]
    assert all(P.fsm_info(2, k)[2] for k in (7, 8)) and not any(P.fsm_info(2, k)[2] for k in range(1, 7))
    # 0 = el dron todavía no responde: no se pinta como falla (para eso están comm_ok y la alerta 10)
    assert not P.fsm_info(2, 0)[2] and "sin respuesta" in P.fsm_info(2, 0)[1]


def test_fsm_9_es_fin_normal_salvo_en_vuelo_o_con_alerta_9():
    """sim/README: en un vuelo normal la FSM a bordo pasa por 1→2→3→4→5→6→9; qd2_mission_step solo trata el 9
    como falla con takeoff = 1 (o en TAKEOFF armado)."""
    for st in (P.ASSIGN_ROLES, P.LANDING, P.DONE, P.INIT):
        for version in (1, 2):
            tipo, txt, falla = P.fsm_info(version, 9, st, 0)
            assert not falla and "fin de vuelo" in txt, (version, st)
    for st in (P.TAKEOFF, P.HOVER_READY, P.MISSION, P.RETURN_HOME, P.ABORT, P.PAUSED):
        assert P.fsm_info(2, 9, st, 0)[2] and P.fsm_info(1, 9, st, 0)[2], st
    # tierra MPC ya decidió que es una falla a bordo (alerta 9): también después de aterrizar
    assert P.fsm_info(2, 9, P.DONE, 9)[2]
    # de punta a punta (512 bytes): misión terminada con los dos en 9 -> nada en rojo
    tel = _tel(version=2, mission_state=P.DONE, drones={1: {"fsm_state": 9}, 2: {"fsm_state": 9}})
    assert not tel["drones"][1]["fsm_falla"] and not tel["drones"][2]["fsm_falla"]
    tel = _tel(version=2, mission_state=P.MISSION, drones={1: {"fsm_state": 5}, 2: {"fsm_state": 9}})
    assert not tel["drones"][1]["fsm_falla"] and tel["drones"][2]["fsm_falla"]


def test_alerta_no_finita_es_grave_y_no_nada():
    for malo in (float("nan"), float("inf"), float("-inf")):
        raw = list(P._TEL.unpack(P.pack_telemetry({"mission_state": P.MISSION})))
        raw[54] = malo                                   # y(55)
        tel = P.unpack_telemetry(P._TEL.pack(*raw))
        assert tel["alerta"] == P.ALERTA_NO_FINITA and tel["alerta"] != 0
        assert tel["alerta_nivel"] == "grave" and "NaN" in tel["alerta_texto"]
    # y(61:64) son los únicos reservados
    assert len(_tel()["reserved"]) == 4


# =============================================================================== mock
def test_mock_rechaza_start_con_homes_cerca(cfg_path):
    d = Driver(_cfg(cfg_path), homes=((0.3, 0.0), (-0.3, 0.2)))      # 0.63 m < 1.05 m (PID)
    assert d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    assert d.m.telemetry()["alerta"] == 1
    d.send("START")
    d.step(20)
    tel = d.m.telemetry()
    assert d.m.state == P.ASSIGN_ROLES and tel["cmd_seq_ack"] == 1 and tel["alerta"] == 1
    assert all(not dr.flying for dr in d.m.drones.values())
    # MPC: el mínimo es 0.8 m -> 0.85 m se acepta, pero en PID no
    cfg = _cfg(cfg_path)
    for version, ok in (("MPC", True), ("PID", False)):
        cfg["version_control"] = version
        d = Driver(cfg, homes=((0.45, 0.0), (-0.4, 0.0)))
        d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
        d.send("START")
        d.step(10)
        assert (d.m.state == P.TAKEOFF) == ok and (d.m.telemetry()["alerta"] == 0) == ok, version


def test_mock_y56_60_rota_por_toda_la_tabla(cfg_path):
    d = Driver(_cfg(cfg_path))
    d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    d.slots = [(1, 0, 0.1, 0.2, 0.9), (2, 1, -0.3, 0.4, 0.9)]
    d.n = 2
    d.step(2)
    assert d.m.add_internal_target(1, 0.5, -0.5) == 3           # siguiente id libre de la tabla
    vistos = {}
    for _ in range(9):
        ob = P.unpack_telemetry(P.pack_telemetry(d.m.telemetry()))["objetivo_sim"]
        vistos.setdefault(ob["id"], ob)
    assert set(vistos) == {1, 2, 3}
    assert vistos[3]["cls"] == 1 and vistos[3]["x"] == pytest.approx(0.5) and vistos[3]["estado"] == 1
    d.m.visited.add(2)
    d.m.current = 1
    est = {}
    for _ in range(3):
        ob = d.m.telemetry()["objetivo_sim"]
        est[ob["id"]] = ob["estado"]
    assert est == {1: 2, 2: 3, 3: 1}


def test_mock_alerta_forzada_y_fsm_por_version(cfg_path):
    m = MockMission(_cfg(cfg_path))
    m.alerta_forzada = 9
    assert m.telemetry()["alerta"] == 9
    m.alerta_forzada = None
    assert m.telemetry()["alerta"] == 0
    # PID: Flight State de tierra (0 Idle, 1 Armed, 2 Takeoff, 3 Land, 9 MS Stop)
    assert flight_state_tierra(FSM_IDLE, False) == 0 and flight_state_tierra(FSM_FLIGHT, False) == 2
    assert flight_state_tierra(FSM_AUTOLAND, False) == 3 and flight_state_tierra(FSM_IDLE, True) == 3
    assert {k: v["fsm_state"] for k, v in m.telemetry()["drones"].items()} == {1: 0, 2: 0}
    cfg = _cfg(cfg_path)
    cfg["version_control"] = "MPC"
    assert {k: v["fsm_state"] for k, v in MockMission(cfg).telemetry()["drones"].items()} == {1: 1, 2: 1}


def test_mock_mision_pid_flight_state_completo(cfg_path):
    """Misión mock PID completa: el Flight State de tierra pasa por 0 -> 2 -> 3 (nunca códigos a bordo 4-6)."""
    cfg = _cfg(cfg_path)
    cfg["mision"]["auto_land"] = True
    d = Driver(cfg)
    vistos = set()
    d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    vistos.update(v["fsm_state"] for v in d.m.telemetry()["drones"].values())     # en tierra: 0 Idle
    d.send("START")
    ok = d.run_until(lambda: (vistos.update(v["fsm_state"] for v in d.m.telemetry()["drones"].values())
                              or d.m.state == P.DONE), 400)
    assert ok and {0, 2, 3} <= vistos and not (vistos & {4, 5, 6})


# =============================================================================== estación: y(56:60)
def _station(cfg_path):
    _patch_cfg(cfg_path, lambda dd: dd["red"].update(puerto_puente=_free_port()))
    return Station(cfg_path, "simulink")      # sin start(): se alimenta con telemetría a mano


def test_estacion_sincroniza_objetivos_de_simulink_sin_duplicar(cfg_path):
    st = _station(cfg_path)
    assert st.vision.add_manual(0, 0.10, 0.20) == 1                       # publicado por la estación
    now = time.time()
    st._on_telemetry(_tel(n_targets_known=3, objetivo_sim={"id": 1, "cls": 0, "x": 0.11, "y": 0.2, "estado": 1}), now)
    st._on_telemetry(_tel(n_targets_known=3, objetivo_sim={"id": 2, "cls": 1, "x": -0.6, "y": 0.9, "estado": 1}), now)
    st._on_telemetry(_tel(n_targets_known=3, objetivo_sim={"id": 3, "cls": 0, "x": 0.7, "y": -1.0, "estado": 3}), now)
    tel = _tel(n_targets_known=3, responder_target_id=2, visited_mask=0)
    tg = st.merged_targets(tel)
    assert [t["id"] for t in tg] == [1, 2, 3]                              # sin duplicar el 1
    por_id = {t["id"]: t for t in tg}
    assert por_id[1]["origen"] == "estacion+simulink" and por_id[1]["n"] == 1
    assert por_id[2]["origen"] == "simulink" and por_id[2]["clase"] == "toma" and por_id[2]["conf"] is None
    assert por_id[2]["x"] == pytest.approx(-0.6) and por_id[2]["estado"] == "asignada"
    assert por_id[3]["estado"] == "visitada"                               # estado 3 de y(56:60)
    assert por_id[1]["estado"] == "detectada"
    # un id nuevo de la estación nunca choca con los que reporta Simulink
    assert st.vision.add_manual(1, -1.0, -1.0) == 4
    # Simulink reinicia su tabla (n_targets_known = 0): se olvidan sus objetivos
    st._on_telemetry(_tel(n_targets_known=0), now)
    assert [t["origen"] for t in st.merged_targets(_tel())] == ["estacion", "estacion"]


def test_estacion_olvida_objetivos_de_simulink_viejos(cfg_path):
    st = _station(cfg_path)
    st._on_telemetry(_tel(n_targets_known=1, objetivo_sim={"id": 5, "cls": 1, "x": 0, "y": 0, "estado": 1}),
                     time.time() - SIM_TTL_S - 1)

    class _Puente:              # telemetría fresca, pero el objetivo 5 dejó de aparecer en y(56:60)
        def fresh(self):
            return True

        def status(self):
            return {}
    assert [t["id"] for t in st.merged_targets(_tel())] == [5]         # sin puente fresco se conserva
    st.bridge = _Puente()
    assert st.merged_targets(_tel()) == []


def test_choque_de_ids_avisa(cfg_path):
    st = _station(cfg_path)
    st.vision.add_manual(0, 0.0, 0.0)
    st._on_telemetry(_tel(n_targets_known=1, objetivo_sim={"id": 1, "cls": 0, "x": 1.0, "y": 1.0, "estado": 1}),
                     time.time())
    assert any(a["nivel"] == "rojo" and "Choque de ids" in a["msg"] for a in st.avisos())


def test_alerta_queda_en_eventos(cfg_path):
    st = _station(cfg_path)
    st._on_telemetry(_tel(alerta=7), time.time())
    st._on_telemetry(_tel(alerta=0), time.time())
    msgs = [e["msg"] for e in st.events]
    assert any(m.startswith("ALERTA 7") and "Arm" in m for m in msgs) and any("despejada" in m for m in msgs)


# =============================================================================== ids 1..32
def test_no_se_asignan_ids_mayores_a_32(cfg_path):
    c = Clusterer(radius=0.25, min_obs=3)
    c.ensure_next_id(33)                    # Simulink ya conoce 32
    for _ in range(4):
        c.add(0, 0.5, 0.5, 0.9)
    assert c.targets() == [] and len(c.blocked()) == 1
    with pytest.raises(ValueError, match="1..32"):
        c.add_manual(1, 0.0, 0.0)
    c.load_dict({"next_id": 1, "clusters": [{"cls": 0, "sx": 1.0, "sy": 1.0, "sconf": 1.0, "n": 1, "id": 40,
                                            "manual": True, "t0": None}]})
    assert c.targets() == [] and c.next_id == 1
    # puente: un id fuera de 1..32 nunca sale
    br = BridgeClient("127.0.0.1", _free_port())
    br.set_targets([(1, 0, 0.0, 0.0, 0.9), (33, 1, 0.5, 0.5, 0.9)])
    m = P.unpack_command(br.build_message())
    assert m["n_targets"] == 1 and [s["id"] for s in m["slots"]] == [1, 0, 0]
    assert br.status()["n_targets_rechazados"] == 1
    # estación: aviso rojo y el objetivo manual se rechaza con 400
    st = _station(cfg_path)
    st.vision.ensure_next_id(33)
    for _ in range(3):
        st.vision.clusters.add(1, -0.5, 0.5, 0.9)
    assert any(a["nivel"] == "rojo" and "1..32" in a["msg"] for a in st.avisos())
    assert any("Sin ids libres" in e["msg"] for e in st.events)


# =============================================================================== puente: objetivos retenidos
def test_objetivos_retenidos_hasta_la_primera_telemetria():
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    got = {}

    def server():
        conn, _ = srv.accept()
        conn.settimeout(2.0)
        buf = b""
        while len(buf) < 256:
            buf += conn.recv(4096)
        got["primero"] = P.unpack_command(buf[:256])
        conn.sendall(P.pack_telemetry({"t": 1.0, "mission_state": 2}))
        buf = buf[256:]
        end = time.time() + 2.0
        while time.time() < end and "despues" not in got:
            buf += conn.recv(4096)
            while len(buf) >= 256:
                m = P.unpack_command(buf[:256])
                buf = buf[256:]
                if m["n_targets"]:
                    got["despues"] = m
        conn.close()

    th = threading.Thread(target=server, daemon=True)
    th.start()
    br = BridgeClient("127.0.0.1", port, reconnect_s=0.2)
    br.set_targets([(1, 0, 0.1, 0.1, 0.9), (2, 1, 0.2, 0.2, 0.9)])
    br.start()
    try:
        th.join(5)
        assert got["primero"]["n_targets"] == 0 and all(s["id"] == 0 for s in got["primero"]["slots"])
        assert got["despues"]["n_targets"] == 2
    finally:
        br.stop()
        srv.close()


# =============================================================================== de punta a punta con el mock
@pytest.fixture
def client(cfg_path):
    st = Station(cfg_path, "mock", mock_speed=4.0)
    with TestClient(create_app(st)) as c:
        c.station = st
        yield c


def _estado(c):
    return c.get("/api/estado").json()


def test_mock_alerta_y_objetivos_de_simulink_en_la_ui(client):
    assert wait(lambda: (_estado(client)["tel"] or {}).get("mission_state_name") == "ASSIGN_ROLES", 10)
    assert client.post("/api/mock/alerta", json={"codigo": 11}).status_code == 400
    assert client.post("/api/mock/alerta", json={"codigo": 5}).json()["ok"]
    assert wait(lambda: _estado(client)["tel"]["alerta"] == 5, 5)
    s = _estado(client)
    assert "E-STOP" in s["tel"]["alerta_texto"] and s["tel"]["alerta_nivel"] == "grave"
    assert any(e["msg"].startswith("ALERTA 5") for e in s["eventos"])
    # objetivo que "detecta Simulink" (no lo publicó la estación) -> llega por y(56:60)
    r = client.post("/api/mock/objetivo_interno", json={"cls": "toma", "x": 0.4, "y": -0.8}).json()
    assert r["id"] == 1
    assert wait(lambda: any(t["id"] == 1 and t["origen"] == "simulink" for t in _estado(client)["objetivos"]), 5)
    # objetivo manual de la estación: id 2 (no choca) y Simulink lo confirma -> "ambos", sin duplicados
    assert client.post("/api/objetivos/manual", json={"cls": 0, "x": -0.5, "y": 0.5}).json()["id"] == 2
    assert wait(lambda: {t["id"]: t["origen"] for t in _estado(client)["objetivos"]}
                == {1: "simulink", 2: "estacion+simulink"}, 5)
    tg = {t["id"]: t for t in _estado(client)["objetivos"]}
    assert tg[1]["clase"] == "toma" and tg[1]["x"] == pytest.approx(0.4)
    # PID: el fsm es el Flight State de tierra
    d = _estado(client)["tel"]["drones"]["1"]
    assert d["fsm_tipo"] == "Flight State (tierra)" and d["fsm_state"] == 0
    assert client.post("/api/mock/alerta", json={"codigo": None}).json()["ok"]
    assert wait(lambda: _estado(client)["tel"]["alerta"] == 0, 5)


@pytest.mark.parametrize("fuente", ["simulink", "sim3d"])
def test_y56_60_y_alerta_con_fuentes_simulink_y_sim3d(cfg_path, fuente):
    """Las fuentes simulink y sim3d usan el mismo puente: un Stream Server (aquí el mock) que reporta objetivos
    propios en y(56:60) y una alerta en y(55) se ve en la tabla de la estación."""
    from app.mock_sim import MockServer
    mock = MockServer(_cfg(cfg_path), "127.0.0.1", 0, speed=2.0).start()
    mock.objetivo_interno(0, -0.7, 0.3)
    mock.objetivo_interno(1, 0.9, -0.2)
    mock.forzar_alerta(2)
    _patch_cfg(cfg_path, lambda dd: dd["red"].update(puerto_puente=mock.port))
    st = Station(cfg_path, fuente).start()
    try:
        assert wait(lambda: {t["id"]: t["origen"] for t in st.snapshot()["objetivos"]} == {1: "simulink", 2: "simulink"}, 5)
        s = st.snapshot()
        assert s["tel"]["alerta"] == 2 and "rastreo" in s["tel"]["alerta_texto"]
        assert {t["id"]: t["clase"] for t in s["objetivos"]} == {1: "fuego", 2: "toma"}
        assert st.vision.add_manual(0, 0.0, 1.0) == 3            # no choca con los ids de Simulink
    finally:
        st.stop()
        mock.stop()


# =============================================================================== revisión: ráfagas, ABORT, alias
def test_puente_procesa_todas_las_telemetrias_de_una_rafaga():
    """Si Simulink / sim3d manda varias telemetrías juntas (un solo recv), el objetivo rotando y(56:60) de cada
    una debe llegar a los oyentes; antes solo se procesaba la última y, con ráfagas regulares de 2 y 4
    objetivos, los ids 1 y 3 no se veían nunca."""
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    alto = threading.Event()

    def tel(tid, t):
        return P.pack_telemetry({"t": t, "mission_state": P.MISSION, "version": 1, "n_targets_known": 4,
                                 "drones": {1: {}, 2: {}},
                                 "objetivo_sim": {"id": tid, "cls": 0, "x": 0.1 * tid, "y": 0.0, "estado": 1}})

    def server():
        conn, _ = srv.accept()
        conn.setblocking(False)
        k = 0
        while not alto.is_set():
            try:
                conn.sendall(tel(k % 4 + 1, k * 0.02) + tel((k + 1) % 4 + 1, k * 0.02 + 0.02))   # ráfaga de 2
            except (BlockingIOError, OSError):
                pass
            try:
                conn.recv(65536)
            except (BlockingIOError, OSError):
                pass
            k += 2
            time.sleep(0.04)
        conn.close()

    th = threading.Thread(target=server, daemon=True)
    th.start()
    vistos, ts = {}, []
    br = BridgeClient("127.0.0.1", port)
    br.listeners.append(lambda t, s: (ts.append(t["t"]), t["objetivo_sim"] and vistos.__setitem__(
        t["objetivo_sim"]["id"], vistos.get(t["objetivo_sim"]["id"], 0) + 1)))
    br.start()
    try:
        assert wait(lambda: len(vistos) == 4 and min(vistos.values()) >= 5, 5), vistos
        assert ts == sorted(ts)                                   # en orden de llegada
        assert br.latest()[0]["t"] == ts[-1]                      # ``telemetry`` = la más reciente
    finally:
        alto.set()
        br.stop()
        srv.close()


def test_pose_de_una_rafaga_gana_la_mas_reciente():
    from app.vision_service import PoseHistory
    h = PoseHistory()
    h.add(10.0, 0.0, 0.0, 1.5, 0.0)
    h.add(10.0, 0.3, 0.0, 1.5, 0.0)          # mismo stamp (2.º mensaje de la misma ráfaga)
    h.add(9.0, 9.9, 9.9, 9.9, 0.0)           # más vieja: se ignora
    assert h.at(10.0)[0] == pytest.approx(0.3) and len(h.t) == 1


def _hasta_takeoff(cfg_path):
    d = Driver(_cfg(cfg_path))
    assert d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    d.send("START")
    assert d.run_until(lambda: d.m.state == P.TAKEOFF, 1)
    return d


@pytest.mark.parametrize("cmd", ["ABORT", "LAND_ALL"])
def test_mock_abort_antes_del_takeoff_desarma_y_vuelve_a_assign_roles(cfg_path, cmd):
    """§4: ABORT / LAND_ALL en TAKEOFF sin takeoff dado -> desarmar y volver a ASSIGN_ROLES (sin despegar)."""
    d = _hasta_takeoff(cfg_path)
    d.step(20)                                           # 0.2 s: armados, takeoff aún no (2 s)
    assert not d.m.takeoff_dado and d.m.telemetry()["drones"][1]["fsm_state"] == 1     # PID: 1 Armed
    d.send(cmd)
    zmax = 0.0
    for _ in range(600):                                 # 6 s
        d.step()
        zmax = max(zmax, *(dr.p[2] for dr in d.m.drones.values()))
    assert d.m.state == P.ASSIGN_ROLES and zmax < 0.01 and all(not dr.flying for dr in d.m.drones.values())
    # se puede volver a dar START
    d.send("START")
    assert d.run_until(lambda: d.m.state == P.HOVER_READY, 10)


@pytest.mark.parametrize("cmd", ["ABORT", "LAND_ALL"])
def test_mock_abort_con_takeoff_dado_y_en_tierra_baja_hasta_done(cfg_path, cmd):
    """§4: takeoff ya dado pero nadie por encima de z_despegado -> bajan en su sitio y DONE (nunca un takeoff
    nuevo). Antes el mock pasaba a ABORT y los subía a z_E / z_R."""
    d = _hasta_takeoff(cfg_path)
    assert d.run_until(lambda: d.m.takeoff_dado, 3)
    assert max(dr.p[2] for dr in d.m.drones.values()) < 0.15
    d.send(cmd)
    zmax = 0.0
    for _ in range(600):
        d.step()
        zmax = max(zmax, *(dr.p[2] for dr in d.m.drones.values()))
    assert d.m.state == P.DONE and zmax < 0.15


def test_mock_abort_con_un_dron_en_el_aire_es_abort_normal(cfg_path):
    d = _hasta_takeoff(cfg_path)
    assert d.run_until(lambda: max(dr.p[2] for dr in d.m.drones.values()) > 0.3, 5)
    d.send("ABORT")
    d.step(2)
    assert d.m.state == P.ABORT


def test_alias_de_simulink_no_se_duplica(cfg_path):
    """Simulink guarda como alias (sin agregarlo) un id nuevo a < radio_cluster de un objetivo que ya tiene:
    la UI muestra una sola fila (la de la estación) con ``alias_de``."""
    st = _station(cfg_path)
    now = time.time()
    st._on_telemetry(_tel(n_targets_known=1, objetivo_sim={"id": 1, "cls": 0, "x": 0.50, "y": 0.50, "estado": 1}), now)
    assert st.vision.add_manual(0, 0.55, 0.50) == 2       # mismo objeto, otro id (1 ya lo conoce Simulink)
    tg = st.merged_targets(_tel(n_targets_known=1, responder_target_id=1))
    assert len(tg) == 1
    assert tg[0]["id"] == 2 and tg[0]["origen"] == "estacion+simulink" and tg[0]["alias_de"] == 1
    assert tg[0]["estado"] == "asignada"                   # R va al #1 de Simulink = este objeto
    # visited_mask marca también el id alias cuando se visita (mascara_visitados)
    assert st.merged_targets(_tel(n_targets_known=1, visited_mask=0b11))[0]["estado"] == "visitada"
    # orden inverso: la estación publicó primero y Simulink reporta el mismo objeto con otro id
    st2 = _station(cfg_path)
    assert st2.vision.add_manual(1, -0.3, 0.2) == 1
    st2._on_telemetry(_tel(n_targets_known=1, objetivo_sim={"id": 7, "cls": 1, "x": -0.32, "y": 0.2, "estado": 1}), now)
    assert [(t["id"], t.get("alias_de")) for t in st2.merged_targets(_tel(n_targets_known=1))] == [(1, 7)]
    assert any("mismo lugar" in e["msg"] for e in st2.events)
    # lejos (> radio_cluster): son dos objetos distintos
    assert st.vision.add_manual(1, -0.8, -0.8) == 3
    st._on_telemetry(_tel(n_targets_known=2, objetivo_sim={"id": 4, "cls": 1, "x": 0.8, "y": -0.8, "estado": 1}), now)
    assert {t["id"]: t["origen"] for t in st.merged_targets(_tel(n_targets_known=2))} == \
        {2: "estacion+simulink", 3: "estacion", 4: "simulink"}


def test_snapshot_refleja_la_alerta_forzada_del_mock(client):
    assert wait(lambda: _estado(client)["mock"] is not None, 5)
    assert _estado(client)["mock"]["alerta_forzada"] is None
    client.post("/api/mock/alerta", json={"codigo": 4})
    assert _estado(client)["mock"]["alerta_forzada"] == 4
    client.post("/api/mock/alerta", json={"codigo": None})
    assert _estado(client)["mock"]["alerta_forzada"] is None


def test_mock_guarda_alias_como_simulink_y_la_ui_no_duplica(client):
    """El mock emula el alias de qd2_mission_step: la estación publica el mismo objeto que ya tenía "Simulink"
    con otro id -> no se agrega a la tabla del mock y la UI muestra una sola fila."""
    assert wait(lambda: (_estado(client)["tel"] or {}).get("mission_state_name") == "ASSIGN_ROLES", 10)
    assert client.post("/api/mock/objetivo_interno", json={"cls": "toma", "x": 0.6, "y": -0.9}).json()["id"] == 1
    assert wait(lambda: [t["id"] for t in _estado(client)["objetivos"]] == [1], 5)
    assert client.post("/api/objetivos/manual", json={"cls": 1, "x": 0.65, "y": -0.9}).json()["id"] == 2
    assert wait(lambda: [(t["id"], t["origen"], t.get("alias_de")) for t in _estado(client)["objetivos"]]
                == [(2, "estacion+simulink", 1)], 5)
    m = client.station.mock.mission
    assert wait(lambda: m.alias == {2: 1}, 5), (m.alias, m.targets)       # el puente ya se lo mandó al mock
    assert set(m.targets) == {1} and _estado(client)["tel"]["n_targets_known"] == 1
    m.visited.add(1)                                     # visitado: la máscara marca también el alias
    assert wait(lambda: _estado(client)["tel"]["visited_ids"] == [1, 2], 5)
    assert [t["estado"] for t in _estado(client)["objetivos"]] == ["visitada"]
