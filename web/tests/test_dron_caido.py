"""Dron caído en dual PID (qd2_mission_step, bloque "Dron caído (dual PID)"; p.t_caido_s = mision.t_caido_s).

Con el binario PID del Equipo2 tierra nunca recibe la FSM a bordo: un dron que pierde el enlace (FSM 7 a los 0.5 s,
motores cortados a ~1 s), que entra en error a bordo o en AUTOLAND por batería (14.0 V) acaba en el piso, rastreado.
La misión lo detecta con la pose (takeoff = 1 en un estado de vuelo, ya estuvo despegado, en el piso más de
t_caido_s) y hace la reacción de falla a bordo: alerta 9, takeoff 0 al caído, el otro se aparta, regresa y aterriza.

* ``config.mision_t_caido``: misma validación que ``qd2_params_from_config`` (casos evaluados en MATLAB, en
  ``referencia_rastreo/referencia_rastreo_matlab.json``);
* el mock (``MockMission.apagar_dron``): reacción con R y con E apagados, sin falsos positivos en misiones normales,
  nada en solo explorador, en MPC ni con ``t_caido_s = 0`` (la paridad con MATLAB la comparan los escenarios
  ``dual_caido_*`` de ``test_rastreo_perdida.test_mock_igual_que_matlab``);
* estación: texto de la alerta 9 con el binario PID ("Dron N en el piso sin que la misión lo pidiera ..."), Eventos,
  aviso de enlace y avisos de la config; de punta a punta con el mock (``POST /api/mock/apagar``).
"""
import copy
import json
import math
import os
import time

import numpy as np
import pytest
from fastapi.testclient import TestClient

from app import protocol as P
from app.config import mision_t_caido
from app.mock_sim import D_DERIVA_SIN_RASTREO, MockMission
from app.server import Station, create_app
from test_alertas_objetivos_sim import SANOS, _PuenteFresco, _dar, _station, _tel
from test_mock_mision import DT, Driver
from test_revision import _cfg, _patch_cfg

AQUI = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(AQUI, "referencia_rastreo", "referencia_rastreo_matlab.json"), encoding="utf-8") as _fh:
    REF = json.load(_fh)
TEXTO = ("en el piso sin que la misión lo pidiera (pérdida de enlace, error a bordo o batería): el otro se aparta, "
         "regresa y aterriza")
HOMES_CERCA = ((0.55, -0.25), (-0.55, 0.25))      # 1.21 m: el sano tiene que apartarse para quedar a 1.60 m


# =============================================================================== config y textos
def test_t_caido_igual_que_matlab():
    assert REF["casos_config_caido"]
    for caso in REF["casos_config_caido"]:
        cfg = {"mision": {"t_caido_s": json.loads(caso["json"])}}
        r = mision_t_caido(cfg)
        assert r["valor"] == pytest.approx(caso["valor"]), caso
        assert bool(r["aviso"]) == bool(caso["aviso"]), caso
    assert mision_t_caido({"mision": {}}) == {"valor": 1.0, "aviso": None}
    assert mision_t_caido({}) == {"valor": 1.0, "aviso": None}
    assert mision_t_caido({"mision": {"t_caido_s": 0}}) == {"valor": 0.0, "aviso": None}
    assert "inválido" in mision_t_caido({"mision": {"t_caido_s": 0.2}})["aviso"]
    # las configs del repo no traen la clave (valor por defecto 1.0 s)
    raiz = os.path.dirname(os.path.dirname(AQUI))
    for nombre in ("estacion.json", "estacion_dual.json"):
        with open(os.path.join(raiz, "config", nombre), encoding="utf-8-sig") as fh:
            c = json.load(fh)
        assert "t_caido_s" not in c["mision"] and mision_t_caido(c) == {"valor": 1.0, "aviso": None}


def test_textos_de_la_alerta_9():
    txt, nivel = P.ALERTAS[9]
    assert nivel == "grave" and "a bordo" in txt and "en el piso sin que la misión lo pidiera" in txt
    assert P.texto_alerta_caido([2]) == "Dron 2 " + TEXTO + "."
    assert P.texto_alerta_caido([]).startswith("Un dron en el piso")
    assert P.texto_alerta_caido([1, 2]).startswith("Un dron en el piso")
    tel = _tel(mission_state=P.ABORT, alerta=9,
               drones={1: {"tracked": 1, "z": 1.5}, 2: {"tracked": 1, "z": 0.02}})
    assert P.drones_en_piso(tel) == [2]
    tel = _tel(mission_state=P.ABORT, alerta=9, drones={1: {"tracked": 1, "z": 1.5}, 2: {"tracked": 0}})
    assert P.drones_en_piso(tel) == []
    tel = _tel(dron_solo=1, responder_id=0, drones={1: {"tracked": 1, "z": 0.0}, 2: {"tracked": 0}})
    assert P.drones_en_piso(tel) == []


# =============================================================================== mock
class Registro:
    """Corre un Driver y guarda por paso: estado, alerta, takeoff y falla por dron, fase y poses."""

    def __init__(self, d):
        self.d = d
        self.t, self.estado, self.alerta, self.tk, self.falla, self.fase, self.pos, self.sep = ([] for _ in range(8))

    def paso(self, n=1):
        m = self.d.m
        for _ in range(n):
            self.d.step()
            self.t.append(m.t)
            self.estado.append(m.state)
            self.alerta.append(m.alerta)
            self.tk.append((m.tk[1], m.tk[2]))
            self.falla.append((m.falla[1], m.falla[2]))
            self.fase.append(m.falla_fase)
            self.pos.append((m.drones[1].p.copy(), m.drones[2].p.copy()))
            self.sep.append(m.falla_sep.copy())

    def hasta(self, cond, t_max):
        for _ in range(int(t_max / DT)):
            self.paso()
            if cond():
                return True
        return False

    def estados(self, desde=0):
        e = self.estado[desde:]
        return [x for i, x in enumerate(e) if i == 0 or e[i - 1] != x]


def _cfg_dual(cfg_path, **mision):
    cfg = copy.deepcopy(_cfg(cfg_path))
    cfg["version_control"] = "PID"
    cfg["mision"].update(mision)
    return cfg


def _a(reg, estado, t_max=40.0):
    d = reg.d
    assert reg.hasta(lambda: d.m.state == P.ASSIGN_ROLES, 10)
    reg.paso(25)
    d.send("START")
    assert reg.hasta(lambda: d.m.state == estado, t_max)


@pytest.mark.parametrize("quien", ["R", "E"])
def test_mock_dron_apagado_reaccion(cfg_path, quien):
    """Dual PID con los homes a 1.21 m: se apaga R al entrar a HOVER_READY (o E al entrar a MISSION), con el otro a
    < 1.60 m. A los t_caido_s en el piso: alerta 9,
    falla 9, takeoff 0 al caído (nunca vuelve aunque llegue START / RESUME); el sano se aparta a >= 1.60 m (consigna)
    de él, regresa y aterriza: 9 -> 7 -> 8."""
    reg = Registro(Driver(_cfg_dual(cfg_path), homes=HOMES_CERCA))
    m = reg.d.m
    assert m.t_caido == 1.0 and m.version == 1 and not m.solo
    _a(reg, P.HOVER_READY if quien == "R" else P.MISSION)
    dr = m.R if quien == "R" else m.E
    h = 3 - dr
    k0 = len(reg.t)
    m.apagar_dron(dr)
    assert reg.hasta(lambda: m.alerta == 9, 15)
    k9 = len(reg.t) - 1
    z_rol = m.z_R if dr == m.R else m.z_E
    z_piso = max(0.15, min(0.25, z_rol - 0.30))
    kp = next(k for k in range(k0, k9 + 1) if reg.pos[k][dr - 1][2] < z_piso)
    assert reg.t[k9] - reg.t[kp] == pytest.approx(m.t_caido + DT, abs=DT + 1e-9)
    assert m.falla == {dr: 9, h: 0} and m.tk[dr] == 0 and m.falla_trk == 1
    assert math.hypot(*(reg.pos[k9][0][:2] - reg.pos[k9][1][:2])) < 1.5       # el sano tiene que apartarse
    r_sep = m.d_min_h + m.guard_margin + 0.05 + D_DERIVA_SIN_RASTREO
    assert r_sep == pytest.approx(1.60)
    assert math.hypot(*(m.falla_sep - m.vista[dr][:2])) >= r_sep - 1e-6
    # comandos que no deben dar takeoff de nuevo durante la reacción
    reg.d.send("RESUME")
    reg.paso(5)
    reg.d.send("START")
    assert reg.hasta(lambda: m.state == P.DONE, 60)
    reg.paso(50)
    assert reg.estados(k9) == [P.ABORT, P.LANDING, P.DONE]
    assert all(tk[dr - 1] == 0 for tk in reg.tk[k9:])
    assert not any(b[0] == 0 and a[0] == 1 for a, b in zip(reg.tk[k9 + 1:], reg.tk[k9:])) and \
        not any(b[1] == 0 and a[1] == 1 for a, b in zip(reg.tk[k9 + 1:], reg.tk[k9:]))
    k2 = next(k for k in range(k9, len(reg.t)) if reg.fase[k] == 2)
    d_f1 = math.hypot(*(reg.pos[k2][0][:2] - reg.pos[k2][1][:2]))
    assert d_f1 >= r_sep - 1.5 * m.tol_xy - 0.02
    assert m.alerta == 9 and all(d.p[2] < 0.01 for d in m.drones.values())
    assert math.hypot(*(m.drones[1].p[:2] - m.drones[2].p[:2])) >= m.d_min_h
    assert any(f"dron {dr} en el piso sin que la misión lo pidiera" in e for _, e in m.events)
    assert max(reg.alerta[:kp]) == 0


def _mision_normal(cfg, homes, objetivos, cmds, t_fin=200.0, hb_off=None):
    """Misión dual PID del mock con comandos [(t, nombre)] desde START; devuelve el Registro."""
    d = Driver(cfg, homes=homes)
    reg = Registro(d)
    d.set_targets([{"cls": 1, "x": x, "y": y} for x, y in objetivos])
    _a(reg, P.TAKEOFF)
    t0 = d.m.t
    cmds = sorted(cmds)
    while d.m.t - t0 < t_fin and d.m.state != P.DONE:
        while cmds and d.m.t - t0 >= cmds[0][0]:
            d.send(cmds.pop(0)[1])
        d.heartbeat_on = not (hb_off and hb_off[0] <= d.m.t - t0 < hb_off[1])
        reg.paso()
    reg.paso(50)
    return reg


def test_mock_sin_falsos_positivos(cfg_path):
    """Misiones dual PID normales del mock (homes y objetivos al azar, 0 a 8 objetivos, auto_land, LAND_ALL a media
    misión, ABORT + LAND_ALL, PAUSE / RESUME, pausa por heartbeat): nunca alerta 9 ni falla, y la misma corrida con
    la detección desactivada (t_caido_s = 0) da exactamente los mismos estados y posiciones."""
    rng = np.random.default_rng(7)
    casos = [
        (3, True, []),
        (6, False, [(30.0, "LAND_ALL")]),
        (0, False, [(25.0, "ABORT"), (40.0, "LAND_ALL")]),
        (4, True, [(30.0, "PAUSE"), (36.0, "RESUME")]),
        (2, True, [(24.0, "RESUME")]),          # heartbeat cortado 20-22 s: PAUSED por heartbeat y RESUME
        (8, True, []),
    ]
    for j, (n, auto, cmds) in enumerate(casos):
        h1 = (float(rng.uniform(0.6, 1.1)), float(rng.uniform(-1.2, 1.2)))
        h2 = (float(rng.uniform(-1.2, -0.7)), float(rng.uniform(-1.2, 1.2)))
        obj = [(float(rng.uniform(-1.3, 1.3)), float(rng.uniform(-1.3, 1.3))) for _ in range(n)]
        hb = (20.0, 22.0) if j == 4 else None
        regs = []
        for t_caido in (None, 0):
            mision = {"auto_land": auto}
            if t_caido is not None:
                mision["t_caido_s"] = t_caido
            regs.append(_mision_normal(_cfg_dual(cfg_path, **mision), (h1, h2), obj, list(cmds), hb_off=hb))
        a, b = regs
        assert a.d.m.t_caido == 1.0 and b.d.m.t_caido == 0.0
        assert 9 not in a.alerta and max(max(f) for f in a.falla) == 0, j
        assert a.estado == b.estado and a.alerta == b.alerta and a.tk == b.tk, j
        assert all(np.array_equal(p[0], q[0]) and np.array_equal(p[1], q[1]) for p, q in zip(a.pos, b.pos)), j
        if auto or cmds:
            assert a.d.m.state == P.DONE, (j, a.estados())


def test_mock_solo_mpc_y_desactivado_no_detectan(cfg_path, cfg_path_solo):
    """Sin detección: solo explorador (el dron solo apagado: la misión sigue como antes), MPC (el supervisor real usa
    la FSM a bordo, que el mock no simula) y dual PID con t_caido_s = 0."""
    # solo explorador
    reg = Registro(Driver(_cfg(cfg_path_solo)))
    _a(reg, P.MISSION)
    reg.d.m.apagar_dron(1)
    reg.paso(int(10 / DT))
    assert reg.d.m.alerta == 0 and reg.d.m.state == P.MISSION and reg.d.m.falla == {1: 0, 2: 0}
    assert reg.d.m.drones[1].p[2] < 0.01
    # MPC y t_caido_s = 0
    for cfg in (_cfg_dual(cfg_path), _cfg_dual(cfg_path, t_caido_s=0)):
        if cfg["mision"].get("t_caido_s") is None:
            cfg["version_control"] = "MPC"
        reg = Registro(Driver(cfg))
        _a(reg, P.MISSION)
        reg.d.m.apagar_dron(reg.d.m.R)
        reg.paso(int(10 / DT))
        assert reg.d.m.alerta == 0 and reg.d.m.state == P.MISSION and reg.d.m.falla == {1: 0, 2: 0}, cfg["version_control"]


def test_mock_dron_que_no_despego_no_cuenta(cfg_path):
    """Latch: un dron apagado ANTES de despegar (en TAKEOFF, armado con takeoff) nunca estuvo arriba: no es un dron
    caído (sin alerta 9); el despegue sigue hasta el timeout de MATLAB (el mock no lo simula) o un ABORT."""
    reg = Registro(Driver(_cfg_dual(cfg_path)))
    m = reg.d.m
    _a(reg, P.TAKEOFF)
    m.apagar_dron(m.R)
    reg.paso(int(8 / DT))
    assert m.state == P.TAKEOFF and m.alerta == 0 and not m.caido_alto[m.R]
    assert m.caido_alto[m.E]


@pytest.mark.parametrize("periodo,dur", [(0.3, 0.04), (0.5, 0.06), (0.9, 0.1)])
def test_mock_caido_con_rastreo_intermitente(cfg_path, periodo, dur):
    """Un dron tirado en el piso (volteado, marcadores tapados) parpadea en Motive: huecos de rastreo cortos (<= 0.2 s,
    no es pérdida sostenida) más seguido que t_caido_s. Los huecos no reinician la cuenta "en el piso": alerta 9,
    falla 9, takeoff 0 y DONE (antes la misión seguía en MISSION sin alerta). El mismo parpadeo con R volando no
    dispara nada."""
    for apagar in (True, False):
        reg = Registro(Driver(_cfg_dual(cfg_path)))
        m = reg.d.m
        _a(reg, P.MISSION)
        dr = m.R
        if apagar:
            m.apagar_dron(dr)
        t = m.t + 0.5
        while t < m.t + 60:
            m.perder_rastreo(dr, t, t + dur)
            t += periodo
        k0 = len(reg.t)
        if apagar:
            assert reg.hasta(lambda: m.alerta == 9, 15), (periodo, dur)
            assert m.falla[dr] == 9 and m.falla[3 - dr] == 0 and m.tk[dr] == 0
            assert reg.hasta(lambda: m.state == P.DONE, 60)
        else:
            reg.paso(int(15 / DT))
            assert 9 not in reg.alerta[k0:] and max(max(f) for f in reg.falla) == 0


def test_mock_caido_en_home_durante_regreso(cfg_path):
    """RETURN_HOME con auto_land: el primero que llega sobre su home se apaga (batería / enlace) y el otro llega al
    suyo antes de que pasen la caída y t_caido_s. No se pasa a LANDING con un dron más de tol_z por debajo de su
    altura: en el piso, alerta 9 y falla 9 del caído (antes: LANDING al llegar el otro y DONE con alerta 0, como un
    aterrizaje normal)."""
    reg = Registro(Driver(_cfg_dual(cfg_path, auto_land=True)))
    m = reg.d.m
    reg.d.set_targets([{"cls": 1, "x": x, "y": y} for x, y in ((0.4, 1.0), (0.6, -0.6))])
    _a(reg, P.RETURN_HOME, t_max=200)

    def en_home(k):
        return math.hypot(*(m.drones[k].p[:2] - m.drones[k].home)) < m.tol_xy

    assert reg.hasta(lambda: en_home(1) or en_home(2), 20)
    dr = 1 if en_home(1) else 2
    assert m.state == P.RETURN_HOME and not en_home(3 - dr)
    m.apagar_dron(dr)
    k0 = len(reg.t)
    assert reg.hasta(lambda: en_home(3 - dr), 20)
    t_llega = m.t
    assert m.drones[dr].p[2] > 0.25                       # el otro llega antes de que el caído toque el piso
    assert reg.hasta(lambda: m.state == P.DONE, 60)
    assert m.alerta == 9 and m.falla == {dr: 9, 3 - dr: 0}
    assert all(e == P.RETURN_HOME for e, t in zip(reg.estado[k0:], reg.t[k0:]) if t <= t_llega + 2.0)
    assert any(f"dron {dr} en el piso sin que la misión lo pidiera" in e for _, e in m.events)


# =============================================================================== estación
def test_estacion_alerta_9_pid_dice_que_dron_cayo(cfg_path):
    st = _station(cfg_path)
    st.bridge = _PuenteFresco()
    now = time.time()
    # aparece la 9 con el dron 2 en el piso y el 1 todavía en el aire
    tel = _tel(mission_state=P.ABORT, alerta=9,
               drones={1: {"tracked": 1, "z": 1.48, "fsm_state": 2, "comm_ok": 1},
                       2: {"tracked": 1, "z": 0.01, "fsm_state": 2, "comm_ok": 1}})
    _dar(st, tel, now)
    ui = st._tel_ui(tel)["alerta_texto"]
    assert ui == "Dron 2 " + TEXTO + "."
    msgs = [e["msg"] for e in st.events]
    assert any(m.startswith("ALERTA 9") and "Dron 2 en el piso" in m for m in msgs)
    assert sum("Dron 2 en el piso sin que la misión lo pidiera" in m for m in msgs) == 2   # Eventos + ALERTA
    # el sano aterriza después: se recuerda cuál cayó
    tel = _tel(mission_state=P.DONE, alerta=9, drones={1: {"tracked": 1, "z": 0.0}, 2: {"tracked": 1, "z": 0.0}})
    _dar(st, tel, now)
    assert st._tel_ui(tel)["alerta_texto"] == "Dron 2 " + TEXTO + "."
    # otra alerta: se olvida
    _dar(st, _tel(alerta=0), now)
    assert st._caido_visto == frozenset()
    # MPC: la 9 conserva su texto (FSM a bordo)
    tel = _tel(version=2, mission_state=P.ABORT, alerta=9,
               drones={1: {"tracked": 1, "z": 1.5, "fsm_state": 5}, 2: {"tracked": 1, "z": 0.0, "fsm_state": 7}})
    _dar(st, tel, now)
    assert st._tel_ui(tel)["alerta_texto"] == P.ALERTAS[9][0]


def test_estacion_avisos_de_enlace_y_config(cfg_path):
    st = _station(cfg_path)
    st.bridge = _PuenteFresco()
    caido = {1: SANOS[1], 2: {"tracked": 1, "fsm_state": 2, "comm_ok": 0}}
    _dar(st, _tel(drones=caido))
    av = [a for a in st.avisos() if "sin enlace con tierra" in a["msg"]]
    assert len(av) == 1 and "binario PID" in av[0]["msg"] and "LAND ALL" in av[0]["msg"]
    assert "alerta 9" in av[0]["msg"] and "se aparta, regresa y aterriza solo" in av[0]["msg"]
    # detección desactivada: aviso amarillo y el texto de antes
    _patch_cfg(cfg_path, lambda d: d["mision"].update(t_caido_s=0))
    st = _station(cfg_path)
    st.bridge = _PuenteFresco()
    _dar(st, _tel(drones=caido))
    msgs = [a["msg"] for a in st.avisos()]
    assert any("mision.t_caido_s = 0" in m and "DESACTIVADA" in m for m in msgs)
    av = [m for m in msgs if "sin enlace con tierra" in m]
    assert len(av) == 1 and "NO lo sabe" in av[0] and "t_caido_s = 0" in av[0]
    # valor inválido
    _patch_cfg(cfg_path, lambda d: d["mision"].update(t_caido_s="1"))
    msgs = [a["msg"] for a in _station(cfg_path).avisos()]
    assert any("mision.t_caido_s inválido" in m for m in msgs)
    _patch_cfg(cfg_path, lambda d: d["mision"].pop("t_caido_s"))
    assert not any("t_caido_s" in a["msg"] for a in _station(cfg_path).avisos())


def test_ui_con_el_mock_dual_apagar_dron(cfg_path):
    """De punta a punta: estación con el mock dual PID, START, ``POST /api/mock/apagar`` del dron R en MISSION ->
    alerta 9 con "Dron N en el piso sin que la misión lo pidiera ..." en /api/estado, Eventos, y el resto de la
    reacción hasta DONE."""
    _patch_cfg(cfg_path, lambda d: d.update(version_control="PID"))
    st = Station(cfg_path, "mock", mock_speed=4.0)
    with TestClient(create_app(st)) as c:
        def est():
            return c.get("/api/estado").json().get("tel") or {}

        def esperar(fn, t=20.0):
            fin = time.time() + t
            while time.time() < fin:
                v = fn()
                if v:
                    return v
                time.sleep(0.05)
            return None

        assert c.post("/api/mock/apagar", json={"dron": 3}).status_code == 400
        assert esperar(lambda: est().get("mission_state") == P.ASSIGN_ROLES)
        time.sleep(0.3)
        assert c.post("/api/cmd", json={"cmd": "START"}).json()["ok"]
        tel = esperar(lambda: (lambda t: t if t.get("mission_state") == P.MISSION else None)(est()), 30)
        assert tel
        dr = tel["responder_id"]
        assert c.post("/api/mock/apagar", json={"dron": dr}).json() == {"ok": True, "dron": dr}
        tel = esperar(lambda: (lambda t: t if t.get("alerta") == 9 else None)(est()), 15)
        assert tel and tel["alerta_texto"] == f"Dron {dr} " + TEXTO + "."
        assert tel["drones"][str(dr)]["tracked"] == 1 and tel["drones"][str(dr)]["z"] < 0.25
        assert esperar(lambda: est().get("mission_state") == P.DONE, 40)
        assert est()["alerta_texto"] == f"Dron {dr} " + TEXTO + "."
        msgs = [e["msg"] for e in st.events]
        assert any(f"Dron {dr} en el piso sin que la misión lo pidiera" in m for m in msgs)
        assert any(f"dron {dr} apagado a bordo" in m for m in msgs)
