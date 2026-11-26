"""Modo SOLO EXPLORADOR (un solo QDrone 2): config, y(64), mock hasta DONE sin tocar al otro dron, avisos,
START 409, --host-puente, comandos de Conexiones, preflight remoto, snapshot y compatibilidad con lo viejo.

Ninguna prueba abre conexiones a 192.168.2.x: los hosts remotos solo se validan o pasan a un puente falso.
"""
import contextlib
import importlib.util
import io
import json
import math
import os
import socket
import subprocess
import sys
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from app import netinfo
from app import protocol as P
from app import server as server_mod
from app.config import Config, mision_modo
from app.mock_sim import MockMission, contar_waypoints, densify, hdist
from app.mock_world import default_anomalies
from app.server import ModoDistinto, Station, build_parser, create_app
from conftest import N_DUAL, PISTA_SOLO, WEB
from test_mock_mision import DT, Driver
from test_puente import wait

PC = "192.168.2.11"
HOMES_LAB = ((1.25, 0.0), (1.45, -1.45))


def _cfg(path):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def _set(path, fn):
    """Cambia la copia de la config con el escritor de la estación (mismo estilo y finales de línea)."""
    c = Config(path)
    d = json.loads(json.dumps(c.data))
    fn(d)
    c.update(d)
    return path


# =============================================================================== 1. tabla de §1.2
TABLA = [
    ({}, ("dual", 0, False)),
    ({"modo": "dual", "dron_solo": 2}, ("dual", 0, False)),
    ({"modo": "solo_explorador"}, ("solo_explorador", 1, False)),
    ({"modo": "solo_explorador", "dron_solo": 1}, ("solo_explorador", 1, False)),
    ({"modo": " SOLO_Explorador ", "dron_solo": 2}, ("solo_explorador", 2, False)),
    ({"modo": "solo_explorador", "dron_solo": 2.0}, ("solo_explorador", 2, False)),
    ({"modo": "solo_explorador", "dron_solo": None}, ("solo_explorador", 1, False)),
    ({"modo": "solo_explorador", "dron_solo": 3}, ("dual", 0, True)),
    ({"modo": "solo_explorador", "dron_solo": 0}, ("dual", 0, True)),
    ({"modo": "solo_explorador", "dron_solo": 1.5}, ("dual", 0, True)),
    ({"modo": "solo_explorador", "dron_solo": "1"}, ("dual", 0, True)),
    ({"modo": "solo_explorador", "dron_solo": True}, ("dual", 0, True)),
    ({"modo": "solo"}, ("dual", 0, True)),
    ({"modo": 1}, ("dual", 0, True)),
    ({"modo": None}, ("dual", 0, False)),
    ({"modo": ""}, ("dual", 0, False)),
]


def test_mision_modo_tabla(tmp_path):
    for k, (mision, (modo, ds, aviso)) in enumerate(TABLA, 1):
        # por JSON (como llega del archivo), también con los números como los guarda Config (_Num)
        cfg = json.loads(json.dumps({"mision": mision}))
        r = mision_modo(cfg)
        assert (r["modo"], r["dron_solo"], bool(r["aviso"])) == (modo, ds, aviso), (k, mision, r)
        p = tmp_path / f"c{k}.json"
        p.write_text(json.dumps({"mision": mision}), encoding="utf-8")
        r2 = mision_modo(Config(str(p)).data)
        assert (r2["modo"], r2["dron_solo"], bool(r2["aviso"])) == (modo, ds, aviso), (k, mision, r2)
    # NaN y lista no vacía también son inválidos; sin sección mision -> dual
    assert mision_modo({"mision": {"modo": "solo_explorador", "dron_solo": float("nan")}})["modo"] == "dual"
    assert mision_modo({"mision": {"modo": "solo_explorador", "dron_solo": [1, 2]}})["aviso"]
    assert mision_modo({"mision": {"modo": ["solo_explorador"]}})["aviso"]
    assert mision_modo({}) == {"modo": "dual", "dron_solo": 0, "aviso": None}
    assert "'solo'" in mision_modo({"mision": {"modo": "solo"}})["aviso"]


# =============================================================================== 2. y(64)
def test_protocolo_y64():
    base = {"mission_state": P.MISSION, "explorer_id": 1, "responder_id": 0}
    casos = {0.0: (0, "dual", False), 1.0: (1, "solo_explorador", False), 2.0: (2, "solo_explorador", False),
             3.0: (0, "dual", True), float("nan"): (0, "dual", True), -1.0: (0, "dual", True),
             1.5: (0, "dual", True)}
    for raw, (ds, modo, inval) in casos.items():
        v = list(P._TEL.unpack(P.pack_telemetry(base)))
        v[63] = raw
        tel = P.unpack_telemetry(P._TEL.pack(*v))
        assert (tel["dron_solo"], tel["modo_mision"], tel["y64_invalido"]) == (ds, modo, inval), raw
        # "reserved" (y(64) crudo) se conserva por compatibilidad
        assert len(tel["reserved"]) == 1
        assert tel["reserved"][0] == (None if math.isnan(raw) else raw)
    # ida y vuelta con pack_telemetry
    for ds in (1, 2):
        raw = P.pack_telemetry(dict(base, explorer_id=ds, dron_solo=ds))
        assert P._TEL.unpack(raw)[63] == float(ds)
        tel = P.unpack_telemetry(raw)
        assert tel["dron_solo"] == ds and tel["modo_mision"] == "solo_explorador" and tel["responder_id"] == 0
    # sin la clave: y(64) = 0 (dual), igual que antes
    assert P._TEL.unpack(P.pack_telemetry(base))[63] == 0.0


def test_compatibilidad_telemetria_vieja(cfg_path):
    """Modelo de tierra viejo (y(64) = 0 'reservado') con la estación nueva: dual, sin avisos de modo."""
    v = [0.0] * P.N_TELEMETRY
    v[1], v[2], v[3], v[4] = P.MISSION, 1, 2, 1           # estado, PID, E = 2, R = 1
    for o in (14, 32):
        v[o] = 1.0                                         # tracked
    tel = P.unpack_telemetry(P._TEL.pack(*v))
    assert tel["dron_solo"] == 0 and tel["modo_mision"] == "dual" and not tel["y64_invalido"]
    assert tel["reserved"] == [0.0]
    st = Station(cfg_path, "simulink")
    st.bridge = _FakeBridge(tel)
    m = st.modo_info()
    assert m["efectivo"] == "dual" and m["coincide"] and m["tel"] == {"modo": "dual", "dron_solo": 0}
    assert not [a for a in st.avisos() if "y(64)" in a["msg"] or "SOLO" in a["msg"] or "dron 1 (no usado)" in a["msg"]]


# =============================================================================== 3-5. mock
def _driver_solo(cfg, homes=HOMES_LAB):
    d = Driver(cfg, homes)
    assert d.m.solo and d.m.E == d.m.ds and d.m.R == 0
    return d


def _mision_solo(cfg, ds):
    da = 3 - ds
    d = _driver_solo(cfg)
    p_aus = d.m.drones[da].p.copy()
    estados, tels = [], []

    def paso():
        tel = P.unpack_telemetry(P.pack_telemetry(d.m.telemetry()))
        tels.append(tel)
        if not estados or estados[-1] != tel["mission_state"]:
            estados.append(tel["mission_state"])
        return tel

    def run_until(cond, t_max):
        for _ in range(int(t_max / DT)):
            d.step()
            paso()
            if cond():
                return True
        return False

    paso()
    assert tels[0]["explorer_id"] == ds and tels[0]["responder_id"] == 0 and tels[0]["dron_solo"] == ds
    assert run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    assert d.m.alerta == 0
    d.send("START")
    assert run_until(lambda: d.m.state == P.MISSION, 15)
    assert d.m.drones[ds].p[2] == pytest.approx(1.5, abs=0.13)
    run_until(lambda: False, 4)
    anom = default_anomalies(cfg["pista"]["vertices"])
    d.set_targets(anom)
    assert run_until(lambda: d.m.state == P.RETURN_HOME, 200)
    t = d.m.telemetry()
    assert t["explorer_done"] == 1 and t["explorer_wp_idx"] == t["explorer_wp_total"] == len(d.m.wps)
    assert t["n_targets_known"] == len(anom) and t["n_targets_visited"] == 0 and t["visited_mask"] == 0
    assert t["queue_length"] == len(anom)                 # y(53) = objetivos pendientes ("detectados")
    E = d.m.drones[ds]
    assert run_until(lambda: hdist(E.p, E.home) < 0.12, 40)
    d.send("LAND_ALL")
    assert run_until(lambda: d.m.state == P.DONE, 20)
    assert E.p[2] < 0.05 and not E.flying and hdist(E.p, E.home) < 0.15
    # secuencia: INIT -> WAIT_TRACKING -> ASSIGN_ROLES -> TAKEOFF -> HOVER_READY -> MISSION -> RETURN_HOME
    #            -> LANDING -> DONE
    assert estados == [P.INIT, P.WAIT_TRACKING, P.ASSIGN_ROLES, P.TAKEOFF, P.HOVER_READY, P.MISSION,
                       P.RETURN_HOME, P.LANDING, P.DONE], estados
    # el dron ausente nunca se movió, nunca voló ni se rastreó, sin rol ni enlace
    A = d.m.drones[da]
    assert (A.p == p_aus).all() and not A.flying and A.p[2] == 0.0
    for tel in tels:
        a = tel["drones"][da]
        assert a["tracked"] == 0 and a["role"] == 0 and a["comm_ok"] == 0 and a["x"] is None and a["zd"] is None
        assert tel["dron_solo"] == ds and tel["explorer_id"] == ds and tel["responder_id"] == 0
        assert tel["guard_active"] == 0 and tel["dist_horizontal"] == 0.0 and tel["dist_vertical"] == 0.0
        assert tel["responder_target_id"] == 0 and tel["marking_progress"] == 0.0 and tel["alerta"] == 0
        ob = tel["objetivo_sim"]
        assert ob is None or ob["estado"] == 1                 # nunca asignado ni visitado
    assert {tel["drones"][ds]["role"] for tel in tels[-50:]} == {P.ROLE_EXPLORER}
    return d


def test_mock_solo_mision_completa(cfg_path_solo):
    cfg = _cfg(cfg_path_solo)
    d = _mision_solo(cfg, 1)
    assert len(d.m.wps) == 35 == contar_waypoints(PISTA_SOLO, 0.25)
    # el recorrido sigue el orden del dibujo con el home en (1.25, 0): empieza en el vértice 1
    assert tuple(d.m.wps[0][:2]) == pytest.approx(tuple(PISTA_SOLO[0]))


def test_mock_solo_dron2(cfg_path_solo):
    cfg = _cfg(_set(cfg_path_solo, lambda c: c["mision"].update(dron_solo=2)))
    assert mision_modo(cfg)["dron_solo"] == 2
    d = _mision_solo(cfg, 2)
    # home del dron 2 = HOMES_LAB[1] = (1.45, -1.45): el vértice 1 (1.91 m) queda más cerca que el 7 (2.48 m)
    assert tuple(d.m.wps[0][:2]) == pytest.approx(tuple(PISTA_SOLO[0]))


def test_mock_solo_auto_land_abort_y_cancelar(cfg_path_solo):
    cfg = _cfg(cfg_path_solo)
    cfg["mision"]["auto_land"] = True
    d = _driver_solo(cfg)
    d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    # ABORT antes del takeoff: desarma y vuelve a ASSIGN_ROLES
    d.send("START")
    assert d.run_until(lambda: d.m.state == P.TAKEOFF, 1)
    d.send("ABORT")
    assert d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 1)
    d.send("START")
    assert d.run_until(lambda: d.m.state == P.MISSION, 15)
    d.run_until(lambda: False, 5)
    d.send("ABORT")
    assert d.run_until(lambda: d.m.state == P.ABORT, 0.5)
    E = d.m.drones[1]
    assert d.run_until(lambda: hdist(E.p, E.home) < 0.12, 40) and d.m.state == P.ABORT   # hover en home
    # pérdida de heartbeat en MISSION -> PAUSED; con auto_land el fin es solo (sin LAND_ALL)
    d2 = _driver_solo(cfg)
    d2.run_until(lambda: d2.m.state == P.ASSIGN_ROLES, 5)
    d2.send("START")
    d2.run_until(lambda: d2.m.state == P.MISSION, 15)
    d2.heartbeat_on = False
    assert d2.run_until(lambda: d2.m.state == P.PAUSED, 1.5)
    d2.heartbeat_on = True
    d2.send("RESUME")
    assert d2.run_until(lambda: d2.m.state == P.MISSION, 1)
    assert d2.run_until(lambda: d2.m.state == P.DONE, 260)
    assert not d2.m.drones[2].flying and d2.m.drones[2].p[2] == 0.0


def _mock_head():
    """mock_sim.py de la última versión confirmada en git (antes del modo solo), como módulo de ``app``."""
    try:
        src = subprocess.run(["git", "show", "HEAD:web/app/mock_sim.py"], cwd=WEB, capture_output=True,
                             timeout=20).stdout.decode("utf-8")
    except (OSError, subprocess.SubprocessError):
        src = ""
    if "class MockMission" not in src:
        pytest.skip("sin git o sin web/app/mock_sim.py en HEAD")
    spec = importlib.util.spec_from_loader("app._mock_sim_head", loader=None)
    mod = importlib.util.module_from_spec(spec)
    mod.__package__ = "app"
    exec(compile(src, "HEAD:web/app/mock_sim.py", "exec"), mod.__dict__)
    return mod


@pytest.mark.parametrize("version", ["PID", "MPC"])
def test_mock_dual_igual(cfg_path, version):
    """En dual el mock nuevo da EXACTAMENTE la misma telemetría (512 bytes en cada paso) que el de HEAD."""
    head = _mock_head()
    cfg = dict(_cfg(cfg_path), version_control=version)
    # sin la envolvente por eje (mision.limites_x / limites_y): el mock de HEAD no la conoce (sin las claves el
    # mock nuevo es idéntico; con ellas recorta distinto a propósito, test_envolvente.py)
    cfg["mision"] = {k: v for k, v in cfg["mision"].items() if k not in ("limites_x", "limites_y")}
    assert mision_modo(cfg)["modo"] == "dual"
    nuevo, viejo = Driver(cfg), Driver(cfg)
    viejo.m = head.MockMission(cfg)
    assert not nuevo.m.solo
    anom = default_anomalies(cfg["pista"]["vertices"])
    n_ok = 0
    for k in range(int(140 / DT)):
        if k == int(3 / DT):
            nuevo.send("START"), viejo.send("START")
        if k == int(20 / DT):
            nuevo.set_targets(anom), viejo.set_targets(anom)
        nuevo.step(), viejo.step()
        tn, tv = nuevo.m.telemetry(), viejo.m.telemetry()
        # battery_V (o+14) es lo único que cambia a propósito: el mock simula la caída por carga y la descarga en
        # vuelo (test_bateria.py); el resto de los 512 bytes debe ser idéntico
        for t_ in (tn, tv):
            for dd in t_["drones"].values():
                dd["battery_V"] = 0.0
        a, b = P.pack_telemetry(tn), P.pack_telemetry(tv)
        assert a == b, f"paso {k}: la telemetría dual cambió"
        n_ok += 1
    assert nuevo.m.state in (P.MISSION, P.RETURN_HOME) and nuevo.m.visited
    assert n_ok == int(140 / DT)


# =============================================================================== 6-7. avisos y START 409
class _FakeBridge:
    """Puente falso (no abre sockets): telemetría fija, siempre fresca y registra los comandos."""

    def __init__(self, tel, host="127.0.0.1", fresh=True):
        self.telemetry, self.host, self._fresh, self.sent = tel, host, fresh, []
        self.port, self.stamp = 18700, 1.0

    def fresh(self):
        return self._fresh

    def status(self):
        return {}

    def latest(self):
        return self.telemetry, 1.0

    def send_command(self, name):
        self.sent.append(name)
        return len(self.sent)

    def set_targets(self, t):
        pass

    def stop(self):
        pass


def _tel_modo(ds=0, explorer=1, responder=2, ms=P.ASSIGN_ROLES, aus_tracked=0, aus_comm=0, y64=None, wp_total=0):
    t = {"mission_state": ms, "version": 1, "explorer_id": explorer, "responder_id": responder,
         "explorer_wp_total": wp_total,
         "drones": {1: {"tracked": 1, "x": 1.25, "y": 0.0, "z": 0.0, "yaw": 0.0, "comm_ok": 1},
                    2: {"tracked": 1, "x": -0.9, "y": 0.4, "z": 0.0, "yaw": 0.0, "comm_ok": 1}}}
    if ds:
        t["dron_solo"] = ds
        t["drones"][3 - ds].update(tracked=aus_tracked, comm_ok=aus_comm)
    raw = P.pack_telemetry(t)
    if y64 is not None:
        v = list(P._TEL.unpack(raw))
        v[63] = y64
        raw = P._TEL.pack(*v)
    return P.unpack_telemetry(raw)


def _station(path, tel, fuente="simulink"):
    st = Station(path, fuente)
    st.bridge = _FakeBridge(tel)
    return st


MODO_KW = ("y(64)", "DUAL", "Dron solo", "incoherente", "SOLO", "no usado", "waypoints")


def _hay(av, nivel, texto):
    return any(a["nivel"] == nivel and texto in a["msg"] for a in av)


def test_avisos_modo(cfg_path, cfg_path_solo):
    # config solo + modelo dual -> rojo y START 409 (los demás comandos nunca se bloquean)
    st = _station(cfg_path_solo, _tel_modo())
    assert _hay(st.avisos(), "rojo", "El modelo de tierra está en DUAL")
    m = st.modo_info()
    assert not m["coincide"] and m["efectivo"] == "dual" and m["config"]["modo"] == "solo_explorador"
    with pytest.raises(ModoDistinto):
        st.command("START")
    for c in ("PAUSE", "RESUME", "ABORT", "LAND_ALL"):
        st.command(c)
    assert st.bridge.sent == ["PAUSE", "RESUME", "ABORT", "LAND_ALL"]
    with TestClient(create_app(st, start=False)) as c:
        r = c.post("/api/cmd", json={"cmd": "START"})
        assert r.status_code == 409 and "no coincide" in r.json()["detail"]
        assert c.post("/api/cmd", json={"cmd": "LAND_ALL"}).status_code == 200
    # config dual + modelo solo -> amarillo y START 409
    st = _station(cfg_path, _tel_modo(ds=1, responder=0))
    assert _hay(st.avisos(), "amarillo", "SOLO EXPLORADOR")
    with pytest.raises(ModoDistinto):
        st.command("START")
    # dron_solo distinto -> rojo
    st = _station(cfg_path_solo, _tel_modo(ds=2, explorer=2, responder=0))
    assert _hay(st.avisos(), "rojo", "Dron solo distinto")
    # y(64) = 5 -> rojo; incoherente (y(64) = 1 pero explorador 2) -> rojo
    assert _hay(_station(cfg_path, _tel_modo(y64=5.0)).avisos(), "rojo", "y(64) = 5.0")
    assert _hay(_station(cfg_path_solo, _tel_modo(ds=1, explorer=2, responder=0)).avisos(), "rojo", "incoherente")
    # coinciden: sin avisos de modo y START sí sale
    st = _station(cfg_path_solo, _tel_modo(ds=1, responder=0))
    av = st.avisos()
    assert not [a for a in av if a["nivel"] == "rojo" and any(k in a["msg"] for k in MODO_KW)], av
    assert not _hay(av, "amarillo", "no usado") and not _hay(av, "amarillo", "conectado al modelo")
    assert st.command("START") == 1 and st.bridge.sent == ["START"]
    assert st.modo_info()["coincide"] and st.modo_info()["efectivo"] == "solo_explorador"
    # ausente rastreado o con comm -> amarillo
    av = _station(cfg_path_solo, _tel_modo(ds=1, responder=0, aus_tracked=1, aus_comm=1)).avisos()
    assert _hay(av, "amarillo", "El dron 2 (no usado) está a la vista") and _hay(av, "amarillo", "conectado al modelo")
    # texto basura en la config -> amarillo; mock: nunca se bloquea START por el modo
    _set(cfg_path, lambda c: c["mision"].update(modo="solo"))
    assert _hay(_station(cfg_path, _tel_modo()).avisos(), "amarillo", "mision.modo desconocido")
    st = _station(cfg_path_solo, _tel_modo(), fuente="mock")
    assert st.command("START") == 1
    # sin telemetría fresca no se compara (coincide) y no hay avisos de modo
    st = _station(cfg_path_solo, _tel_modo())
    st.bridge._fresh = False
    assert st.modo_info()["coincide"] and not _hay(st.avisos(), "rojo", "DUAL")


def test_aviso_pista_distinta(cfg_path_solo):
    assert contar_waypoints(PISTA_SOLO, 0.25) == len(densify(PISTA_SOLO, 0.25, 1.5)) == 35
    assert contar_waypoints(N_DUAL, 0.25) == len(densify(N_DUAL, 0.25, 1.5)) == 35
    assert contar_waypoints([[0, 0], [2.5, 0]], 0.25) == 11            # L/espaciado exacto (como ceil(L/s - 1e-9))
    assert contar_waypoints([[0, 0], [0, 0], [1, 0]], 0.25) == 5       # tramo de longitud 0 ignorado
    # tope de 128: el espaciado se agranda (16 vértices: 1 + 15 * ceil(112/15) = 121)
    assert contar_waypoints([[-1.5, -1.5], [1.5, 1.5]] * 8, 0.01) == 121
    ok = _station(cfg_path_solo, _tel_modo(ds=1, responder=0, ms=P.MISSION, wp_total=35))
    assert not _hay(ok.avisos(), "amarillo", "waypoints")
    malo = _station(cfg_path_solo, _tel_modo(ds=1, responder=0, ms=P.MISSION, wp_total=20))
    assert _hay(malo.avisos(), "amarillo", "La pista compilada en tierra tiene 20 waypoints y la de esta estación 35")
    antes = _station(cfg_path_solo, _tel_modo(ds=1, responder=0, ms=P.ASSIGN_ROLES, wp_total=20))
    assert not _hay(antes.avisos(), "amarillo", "waypoints")          # solo con mission_state >= 3


# =============================================================================== 8. --host-puente
class _BridgeSinRed:
    """Sustituye a BridgeClient: guarda host/puerto y NUNCA abre un socket."""
    creados = []

    def __init__(self, host, port, **kw):
        self.host, self.port, self.listeners, self.telemetry = host, port, [], None
        self.on_cmd_result = None
        _BridgeSinRed.creados.append(self)

    @property
    def uri(self):
        return f"tcpip://{self.host}:{self.port}"

    def start(self):
        return self

    def stop(self):
        pass

    def set_targets(self, t):
        pass

    def fresh(self):
        return False

    def status(self):
        return {}


def test_host_puente_cli(cfg_path, monkeypatch):
    monkeypatch.setattr(server_mod, "BridgeClient", _BridgeSinRed)

    def no_conectar(*a, **k):
        raise AssertionError("una prueba intentó abrir una conexión de red")
    monkeypatch.setattr(socket, "create_connection", no_conectar)
    monkeypatch.setattr(socket.socket, "connect", no_conectar)

    st = Station(cfg_path, "simulink", host_puente=PC)
    assert st.host_puente() == PC
    st._start_source()
    assert st.bridge.host == PC and st.bridge.port == 18700 and st.puente_remoto() == PC
    m = st.modo_info()
    assert m["host_puente"] == PC and m["remoto"] is True
    # cambiar a sim3d desde la UI: siempre loopback (y queda en Eventos)
    st.set_fuente("sim3d")
    assert st.bridge.host == "127.0.0.1" and st.puente_remoto() is None
    assert any("127.0.0.1" in e["msg"] and PC in e["msg"] for e in st.events)
    st.set_fuente("simulink")
    assert st.bridge.host == PC
    # sin --host-puente: web.host_puente de la config (defecto 127.0.0.1)
    assert Station(cfg_path, "simulink").host_puente() == "127.0.0.1"
    assert Station(cfg_path, "sim3d", host_puente="127.0.0.1").host_puente() == "127.0.0.1"
    # host remoto con sim3d o mock -> error; IP inválida -> error
    for fuente in ("sim3d", "mock"):
        with pytest.raises(ValueError, match="solo se permite con --fuente simulink"):
            Station(cfg_path, fuente, host_puente=PC)
    with pytest.raises(ValueError, match="inválido"):
        Station(cfg_path, "simulink", host_puente="192.168.2.300")
    # CLI: el parser valida la IP; --imprimir-puerto acepta --host-puente (lo usa iniciar_web.bat con %*)
    with pytest.raises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
        build_parser().parse_args(["--host-puente", "pc-lab"])
    a = build_parser().parse_args(["--fuente", "simulink", "--host-puente", PC, "--imprimir-puerto"])
    assert a.host_puente == PC and a.imprimir_puerto
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        server_mod.main(["--config", cfg_path, "--fuente", "simulink", "--host-puente", PC, "--imprimir-puerto"])
    assert out.getvalue().strip() == "8080"
    # la config nunca guarda el host del puente
    assert "host_puente" not in (_cfg(cfg_path).get("web") or {})


# =============================================================================== 9. comandos de Conexiones
def test_comandos_ip_pc_remoto(cfg_path_solo, cfg_path):
    cfg = _cfg(cfg_path_solo)
    cfg["red"]["ip_pc"] = ""
    cfg["red"]["ip_dron1"] = "192.168.2.16"
    r = netinfo.comandos(cfg, None, mision_modo(cfg), host_puente=PC)
    d1, d2 = r["drones"][1], r["drones"][2]
    assert r["ip_pc"] == PC and d1["uri_host"] == f"-URI_Host tcpip://{PC}:18001" and not d1["pendiente"]
    assert d1["no_usado"] is False and d2["no_usado"] is True and r["modo"] == "solo_explorador"
    assert r["dron_solo"] == 1
    # red.ip_pc tiene prioridad; con host loopback y sin ip_pc se usa la interfaz (no la del puente)
    cfg["red"]["ip_pc"] = "192.168.2.50"
    assert netinfo.ip_recomendada(cfg, PC) == "192.168.2.50"
    cfg["red"]["ip_pc"] = ""
    assert netinfo.ip_recomendada(cfg, PC) == PC
    # dual: los dos drones con comandos
    cd = _cfg(cfg_path)
    r = netinfo.comandos(cd, "10.0.0.5", mision_modo(cd))
    assert r["drones"][1]["no_usado"] is False and r["drones"][2]["no_usado"] is False
    # por la API, con el puente remoto
    st = Station(cfg_path_solo, "simulink", host_puente=PC)
    st.bridge = _FakeBridge(None, host=PC)
    with TestClient(create_app(st, start=False)) as c:
        j = c.get("/api/comandos").json()
    assert j["drones"]["2"]["no_usado"] and j["drones"]["1"]["uri_host"] == f"-URI_Host tcpip://{PC}:18001"


# =============================================================================== 10. preflight
def _preflight():
    spec = importlib.util.spec_from_file_location("preflight_lab", os.path.join(WEB, "preflight_lab.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_preflight_solo_remoto(cfg_path_solo, monkeypatch, capsys):
    def no_conectar(*a, **k):
        raise AssertionError("preflight --remoto abrió una conexión")
    monkeypatch.setattr(socket, "create_connection", no_conectar)
    monkeypatch.setattr(socket.socket, "connect", no_conectar)
    _set(cfg_path_solo, lambda c: c["red"].update(ip_dron1="192.168.2.16", ip_dron2="", ip_pc=PC))
    pf = _preflight()
    pf.main(["--config", cfg_path_solo, "--remoto", PC])
    out = capsys.readouterr().out
    fallas = [ln for ln in out.splitlines() if ln.startswith("[FALLA")]
    assert not [ln for ln in fallas if any(k in ln for k in ("ip_dron", "ip_pc", "quarc_run", "Motive", "-URI_Host",
                                                                "red ", "puerto"))], fallas
    assert "[INFO ] ip_dron2 vacía: no se usa en modo solo explorador" in out
    assert "modo SOLO EXPLORADOR con el dron 1" in out
    assert "puerto 18700 (puente Simulink-Python): está en la PC 192.168.2.11" in out
    assert f"IP de la PC para -URI_Host: {PC}" in out
    # red.ip_pc distinta de la PC remota -> FALLA
    _set(cfg_path_solo, lambda c: c["red"].update(ip_pc="192.168.2.50"))
    pf.main(["--config", cfg_path_solo, "--remoto", PC])
    out = capsys.readouterr().out
    assert any(ln.startswith("[FALLA") and "red.ip_pc = 192.168.2.50" in ln for ln in out.splitlines())
    # sin la IP del dron solo -> FALLA
    _set(cfg_path_solo, lambda c: c["red"].update(ip_dron1="", ip_pc=""))
    pf.main(["--config", cfg_path_solo, "--remoto", PC])
    out = capsys.readouterr().out
    assert "[FALLA] falta ip_dron1 (dron solo" in out


# =============================================================================== 11. snapshot y UI con el mock
@pytest.fixture
def client_solo(cfg_path_solo):
    st = Station(cfg_path_solo, "mock", mock_speed=6.0)
    with TestClient(create_app(st)) as c:
        c.station = st
        yield c


def test_snapshot_modo(client_solo):
    c = client_solo
    s = None
    assert wait(lambda: (c.get("/api/estado").json().get("tel") or {}).get("mission_state") == P.ASSIGN_ROLES, 10)
    s = c.get("/api/estado").json()
    m = s["modo"]
    assert set(m) >= {"config", "tel", "fresca", "efectivo", "dron_solo", "coincide", "host_puente", "remoto"}
    assert m["config"] == {"modo": "solo_explorador", "dron_solo": 1, "aviso": None}
    assert m["tel"] == {"modo": "solo_explorador", "dron_solo": 1} and m["fresca"]
    assert m["efectivo"] == "solo_explorador" and m["dron_solo"] == 1 and m["coincide"]
    assert m["host_puente"] == "127.0.0.1" and m["remoto"] is False
    tel = s["tel"]
    assert tel["explorer_id"] == 1 and tel["responder_id"] == 0 and tel["dron_solo"] == 1
    assert tel["drones"]["2"]["tracked"] == 0 and tel["drones"]["2"]["x"] is None and tel["drones"]["2"]["role"] == 0
    assert len(s["pista"]) == 7
    assert not [a for a in s["avisos"] if a["nivel"] == "rojo" and any(k in a["msg"] for k in MODO_KW)], s["avisos"]
    # la página trae el chip del modo
    html = c.get("/").text
    assert 'id="chip-modo"' in html and 'id="pista-remoto"' in html
    # START por la API y la huella del explorador en MISSION (aunque no haya respondedor)
    assert c.post("/api/cmd", json={"cmd": "START"}).status_code == 200
    assert wait(lambda: (c.get("/api/estado").json()["tel"] or {}).get("mission_state") == P.MISSION, 15)
    assert wait(lambda: len(c.get("/api/estado").json()["huella"]) == 4, 5)
    j = c.get("/api/comandos").json()
    assert j["drones"]["2"]["no_usado"] and not j["drones"]["1"]["no_usado"]
