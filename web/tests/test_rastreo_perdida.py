"""Pérdida de rastreo sostenida en el aire -> ese dron aterriza (qd2_mission_step, 24-nov).

* texto de la alerta 2 y su contexto en la UI ("aterrizaje por pérdida de rastreo (dron N)", N de tracked = 0);
* ``config.mision_t_perdida_aterrizar``: misma validación que ``qd2_params_from_config`` (casos evaluados en MATLAB);
* el mock (``MockMission``): sin PAUSED con el valor por defecto, solo 7 -> 8, dual 9 -> 7 -> 8, nunca vuelve a dar
  takeoff (RESUME / START), sin alerta 4 por el perdido, PAUSED entre 0.2 s y un umbral mayor;
* comparación con trazas de la función pura ``qd2_mission_step`` (MATLAB R2025a) de
  ``referencia_rastreo/referencia_rastreo_matlab.json`` (la generan ``generar_referencia_rastreo.m`` a partir de
  ``escenarios_rastreo.json``; volver a generarla si cambia la lógica de MATLAB).

Tolerancias de la comparación (las dos simulaciones usan la MISMA planta, ver generar_referencia_rastreo.m):
* secuencia de valores de estado, alerta, takeoff por dron, falla por dron y fase de la reacción: idéntica;
* tiempos medidos desde t0 (entrada al estado de disparo + retraso; INIT dura 0.2 s en el mock y 1.0 s en MATLAB,
  así que los eventos de antes de t0 - 1 s solo se comparan en valor): +-0.025 s (un paso de 20 ms) para los
  eventos del primer segundo después de que empieza cada pérdida (los decide la pérdida, no la planta) y para el
  resto en los escenarios sin MISSION larga; en ``dual_mision_umbral_05`` +-0.25 s para los que dependen de la
  planta (el mock no hace el dwell de 0.3 s por waypoint de MATLAB, así que el explorador va un poco adelantado);
* posiciones cada 0.1 s: <= 0.01 m (en ``dual_mision_umbral_05`` <= 0.35 m por lo mismo, y <= 0.02 m al final).

Limitación conocida (no la cubren estas tolerancias): por ese mismo dwell que falta, si la pérdida llega tras una
MISSION larga el explorador del mock va más adelantado y aterriza en otro sitio. Con la pista real y la pérdida a
6 s de MISSION, los estados y los tiempos de los eventos son idénticos a MATLAB, pero la posición difiere hasta
~0.5 m (0.45 m al final). La lógica de la pérdida es la misma; solo cambia dónde estaba el dron al perderlo.
"""
import copy
import json
import math
import os
import time

import pytest
from fastapi.testclient import TestClient

from app import protocol as P
from app.config import mision_t_perdida_aterrizar
from app.mock_sim import T_LAND_MAX, MockMission
from app.server import Station, create_app
from test_alertas_objetivos_sim import _station, _tel
from test_mock_mision import DT, Driver
from test_revision import _cfg, _patch_cfg

AQUI = os.path.dirname(os.path.abspath(__file__))
REF_DIR = os.path.join(AQUI, "referencia_rastreo")


def _cargar(nombre):
    with open(os.path.join(REF_DIR, nombre), encoding="utf-8") as fh:
        return json.load(fh)


ESC = _cargar("escenarios_rastreo.json")
REF = _cargar("referencia_rastreo_matlab.json")
CAMPOS = {1: "estado", 2: "alerta", 3: "takeoff1", 4: "takeoff2", 5: "falla1", 6: "falla2", 7: "fase"}
TOL_T = 0.025
TOL_T_PLANTA = {"dual_mision_umbral_05": 0.25}
TOL_POS = {"dual_mision_umbral_05": 0.35}


# =============================================================================== texto de la alerta 2
def test_alerta_2_ya_no_dice_pausa():
    txt, nivel = P.ALERTAS[2]
    assert nivel == "grave" and P.ALERTA_MAX == 10
    assert "PAUSA" not in txt.upper() and "aterriza" in txt and "0.2 s" in txt and "no vuelve a despegar" in txt


def test_texto_de_la_alerta_2_segun_el_estado():
    for st in (P.ABORT, P.LANDING, P.DONE):
        assert "aterrizaje por pérdida de rastreo (dron 2)" in P.texto_alerta_rastreo(st, [2]).lower()
    assert "(drones 1 y 2)" in P.texto_alerta_rastreo(P.LANDING, {2, 1})
    assert "(dron ?)" in P.texto_alerta_rastreo(P.DONE, [])
    assert "PAUSA" in P.texto_alerta_rastreo(P.PAUSED, [1]) and "(dron 1)" in P.texto_alerta_rastreo(P.PAUSED, [1])
    assert P.texto_alerta_rastreo(P.MISSION, [1]) is None
    # tracked = 0 en la trama; en solo explorador el ausente no cuenta
    tel = _tel(mission_state=P.ABORT, alerta=2, drones={1: {"tracked": 1}, 2: {"tracked": 0}})
    assert P.drones_sin_rastreo(tel) == [2]
    tel = _tel(mission_state=P.LANDING, alerta=2, dron_solo=1, responder_id=0,
               drones={1: {"tracked": 0}, 2: {"tracked": 0}})
    assert P.drones_sin_rastreo(tel) == [1]


# =============================================================================== config
def test_t_perdida_aterrizar_igual_que_matlab():
    """Los casos de ``casos_config`` evaluados con qd2_params_from_config (jsondecode) en MATLAB."""
    assert REF["casos_config"]
    for caso in REF["casos_config"]:
        cfg = {"mision": {"t_perdida_aterrizar_s": json.loads(caso["json"])}}
        r = mision_t_perdida_aterrizar(cfg)
        assert r["valor"] == pytest.approx(caso["valor"]), caso
        assert bool(r["aviso"]) == bool(caso["aviso"]), caso
    assert mision_t_perdida_aterrizar({"mision": {}}) == {"valor": 0.2, "aviso": None}
    assert mision_t_perdida_aterrizar({}) == {"valor": 0.2, "aviso": None}
    assert "inválido" in mision_t_perdida_aterrizar({"mision": {"t_perdida_aterrizar_s": 3}})["aviso"]


def test_aviso_de_la_estacion_con_un_umbral_mayor(cfg_path):
    _patch_cfg(cfg_path, lambda d: d["mision"].update(t_perdida_aterrizar_s=0.5))
    msgs = [a["msg"] for a in _station(cfg_path).avisos()]
    assert any("t_perdida_aterrizar_s = 0.5 s" in m and "PAUSED" in m for m in msgs)
    _patch_cfg(cfg_path, lambda d: d["mision"].update(t_perdida_aterrizar_s="0.5"))
    msgs = [a["msg"] for a in _station(cfg_path).avisos()]
    assert any("t_perdida_aterrizar_s inválido" in m for m in msgs)
    _patch_cfg(cfg_path, lambda d: d["mision"].update(t_perdida_aterrizar_s=0.2))
    assert not any("t_perdida_aterrizar_s" in a["msg"] for a in _station(cfg_path).avisos())


# =============================================================================== mock
def _con(cfg, **mision):
    cfg = copy.deepcopy(cfg)
    cfg["mision"].pop("t_perdida_aterrizar_s", None)
    cfg["mision"].update(mision)
    return cfg


def _a_mision(d, t_extra=1.0):
    assert d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 10)
    d.step(25)
    d.send("START")
    assert d.run_until(lambda: d.m.state == P.MISSION, 30)
    d.step(int(t_extra / DT))


def _correr(d, t, cada=None):
    """Avanza ``t`` s y devuelve los estados vistos (en orden, sin repetir seguidos)."""
    vistos = []
    for _ in range(int(round(t / DT))):
        d.step()
        if cada:
            cada()
        if not vistos or vistos[-1] != d.m.state:
            vistos.append(d.m.state)
    return vistos


def test_solo_aterriza_sin_pausa_y_nunca_vuelve_a_despegar(cfg_path_solo):
    d = Driver(_con(_cfg(cfg_path_solo)))
    assert d.m.t_perdida_aterrizar == 0.2
    _a_mision(d)
    t1 = d.m.t + DT / 2
    d.m.perder_rastreo(1, t1)
    d.step(int(0.18 / DT))
    assert d.m.state == P.MISSION and d.m.tk[1] == 1 and d.m.alerta == 0      # 0.18 s: nada todavía
    assert d.run_until(lambda: d.m.state != P.MISSION, 0.1)
    assert d.m.state == P.LANDING and 0.2 < d.m.t - t1 <= 0.2 + 2 * DT       # 5 -> 7 directo, sin PAUSED
    assert d.m.tk[1] == 0 and d.m.falla[1] == 2 and d.m.alerta == 2
    tel = P.unpack_telemetry(P.pack_telemetry(d.m.telemetry()))
    assert tel["drones"][1]["tracked"] == 0 and tel["drones"][1]["x"] is None and tel["alerta"] == 2
    assert P.drones_sin_rastreo(tel) == [1]
    tk = []
    d.send("RESUME")
    d.step()
    d.send("START")
    vistos = _correr(d, T_LAND_MAX + 1.0, lambda: tk.append(d.m.tk[1]))
    assert vistos == [P.LANDING, P.DONE] and not any(tk)
    assert d.m.alerta == 2                                  # sin alerta 4 por el dron sin rastreo
    assert not d.m.drones[1].flying and d.m.drones[1].p[2] == 0.0


def test_solo_vuelve_el_rastreo_done_al_verlo_en_el_suelo(cfg_path_solo):
    d = Driver(_con(_cfg(cfg_path_solo)))
    _a_mision(d)
    t1 = d.m.t
    d.m.perder_rastreo(1, t1, t1 + 1.0)
    vistos = _correr(d, 8.0)
    assert vistos == [P.MISSION, P.LANDING, P.DONE] and d.m.alerta == 2
    d.send("START")
    _correr(d, 2.0)
    assert d.m.state == P.DONE and d.m.tk[1] == 0 and not d.m.drones[1].flying


def test_dual_el_otro_se_aparta_y_aterrizan_9_7_8(cfg_path):
    homes = ((0.55, -0.25), (-0.55, 0.25))              # E = dron 2 (arriba, 1.2 m del R)
    d = Driver(_con(_cfg(cfg_path)), homes)
    _a_mision(d, 0.5)
    assert (d.m.E, d.m.R) == (2, 1)
    t1 = d.m.t
    d.m.perder_rastreo(2, t1)
    ultima = d.m.vista[2][:2].copy()
    tk2, z1 = [], []
    d.step(int(0.5 / DT))
    ultima = d.m.vista[2][:2].copy()                     # última pose vista del perdido
    d.send("RESUME")
    vistos = _correr(d, T_LAND_MAX + 8.0, lambda: (tk2.append(d.m.tk[2]), z1.append(d.m.drones[1].p[2])))
    assert vistos[-3:] == [P.ABORT, P.LANDING, P.DONE] and P.PAUSED not in vistos
    assert d.m.falla == {1: 0, 2: 2} and d.m.alerta == 2 and not any(tk2[-100:])
    sano = d.m.drones[1]
    assert not sano.flying and math.hypot(sano.p[0] - ultima[0], sano.p[1] - ultima[1]) >= 1.4
    assert d.m.min_hdist_flying >= 1.0


def test_umbral_mayor_pausa_entre_0_2_y_ese_valor(cfg_path):
    d = Driver(_con(_cfg(cfg_path), t_perdida_aterrizar_s=0.5))
    assert d.m.t_perdida_aterrizar == 0.5
    _a_mision(d)
    t1 = d.m.t
    d.m.perder_rastreo(1, t1, t1 + 0.3)                  # 0.3 s: PAUSED, no aterriza
    d.step(int(0.25 / DT))
    assert d.m.state == P.PAUSED and d.m.alerta == 2 and d.m.falla == {1: 0, 2: 0}
    d.send("RESUME")                                     # todavía sin rastreo: no se reanuda
    d.step()
    assert d.m.state == P.PAUSED
    d.step(int(0.3 / DT))
    d.send("RESUME")
    d.step()
    assert d.m.state == P.MISSION and d.m.alerta == 0
    t2 = d.m.t
    d.m.perder_rastreo(1, t2, t2 + 0.15)                 # < 0.2 s: nada
    vistos = _correr(d, 0.5)
    assert vistos == [P.MISSION] and d.m.alerta == 0
    t3 = d.m.t
    d.m.perder_rastreo(1, t3)
    assert d.run_until(lambda: d.m.state == P.PAUSED, 0.3) and 0.2 < d.m.t - t3 <= 0.2 + 2 * DT
    assert d.run_until(lambda: d.m.state == P.ABORT, 0.5) and 0.5 < d.m.t - t3 <= 0.5 + 2 * DT
    assert d.m.falla[1] == 2 and d.m.tk[1] == 0 and d.m.tk[2] == 1


def test_rampa_de_despegue_se_cancela_y_nadie_despega(cfg_path_solo):
    d = Driver(_con(_cfg(cfg_path_solo)))
    assert d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 10)
    d.step(25)
    d.send("START")
    assert d.run_until(lambda: d.m.takeoff_dado, 5)
    t1 = d.m.t
    d.m.perder_rastreo(1, t1, t1 + 1.0)
    assert d.run_until(lambda: d.m.state != P.TAKEOFF, 0.3)
    assert d.m.state == P.LANDING and d.m.falla[1] == 2 and d.m.tk[1] == 0 and d.m.alerta == 2
    assert d.m.vista[1][2] < 0.15                         # nadie en el aire todavía
    zmax = []
    vistos = _correr(d, 5.0, lambda: zmax.append(d.m.drones[1].p[2]))
    assert vistos[-1] == P.DONE and max(zmax) < 0.2 and d.m.tk[1] == 0


def test_takeoff_sin_dar_vuelve_a_wait_tracking(cfg_path):
    d = Driver(_con(_cfg(cfg_path)))
    assert d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 10)
    d.step(25)
    d.send("START")
    d.step(int(0.5 / DT))
    assert d.m.state == P.TAKEOFF and not d.m.takeoff_dado
    t1 = d.m.t
    d.m.perder_rastreo(2, t1, t1 + 0.5)
    assert d.run_until(lambda: d.m.state == P.WAIT_TRACKING, 0.3)
    assert d.m.falla == {1: 0, 2: 0} and d.m.alerta == 0 and d.m.tk == {1: 0, 2: 0}
    assert all(d.m.drones[k].fsm == 1 for k in (1, 2))  # desarmados
    assert d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 3)
    d.step(25)
    d.send("START")                                      # nunca voló: se puede volver a despegar
    assert d.run_until(lambda: d.m.state == P.HOVER_READY, 15)


def test_en_landing_sigue_aterrizando_sin_alerta_4(cfg_path):
    d = Driver(_con(_cfg(cfg_path)))
    _a_mision(d)
    d.send("LAND_ALL")
    d.step(int(0.5 / DT))
    assert d.m.state == P.LANDING
    t1 = d.m.t
    d.m.perder_rastreo(1, t1)
    d.step(int(0.3 / DT))
    assert d.m.state == P.LANDING and d.m.falla[1] == 2 and d.m.alerta == 2 and d.m.tk[1] == 0
    vistos = _correr(d, T_LAND_MAX + 1.0)
    assert vistos[-1] == P.DONE and d.m.alerta == 2 and not d.m.drones[1].flying


def test_perdida_corta_no_hace_nada(cfg_path):
    d = Driver(_con(_cfg(cfg_path)))
    _a_mision(d)
    t1 = d.m.t
    d.m.perder_rastreo(2, t1, t1 + 0.19)
    vistos = _correr(d, 1.0)
    assert vistos == [P.MISSION] and d.m.alerta == 0 and d.m.falla == {1: 0, 2: 0}


# =============================================================================== mock contra MATLAB
def _correr_escenario(esc, base, dt):
    """Mismo arnés que generar_referencia_rastreo.m: START 0.5 s después de ASSIGN_ROLES, pérdidas y comandos
    relativos a t0, eventos [t campo valor] y posiciones cada 0.1 s desde t0 - 1 s."""
    cfg = copy.deepcopy(base)
    cfg["mision"].update(esc["mision"])
    m = MockMission(cfg, homes=tuple(tuple(h) for h in esc["homes"]))
    hb = seq = cmd = kc = 0
    t0 = t_start = None
    ev, mu, prev = [], [], [None] * 8
    for _ in range(100000):
        t = m.t + dt
        if t_start is not None and seq == 0 and t >= t_start:
            seq, cmd = 1, P.CMD_START
        if t0 is not None and kc < len(esc["comandos"]) and t >= t0 + esc["comandos"][kc][0]:
            seq, cmd, kc = seq + 1, P.COMMANDS[esc["comandos"][kc][1]], kc + 1
        hb += 1
        m.apply_command_message(P.unpack_command(P.pack_command(hb, cmd, seq, 0, [])))
        m.step(dt)
        val = [None, m.state, m.alerta, m.tk[1], m.tk[2], m.falla[1], m.falla[2], m.falla_fase]
        ev.extend([m.t, c, val[c]] for c in range(1, 8) if val[c] != prev[c])
        prev = val
        if t_start is None and m.state == P.ASSIGN_ROLES:
            t_start = m.t + 0.5
        if t0 is None and m.state == esc["disparo"]["estado"]:
            t0 = m.t + esc["disparo"]["retraso_s"]
            for dron, t1, t2 in esc["perdidas"]:
                m.perder_rastreo(dron, t0 + t1, t0 + (math.inf if t2 is None else t2))
        if t0 is not None and round((m.t - t0) / dt) % 5 == 0 and m.t >= t0 - 1:
            mu.append([m.t] + list(m.drones[1].p) + list(m.drones[2].p))
        if t0 is not None and m.t >= t0 + esc["t_fin"]:
            break
    return {"t0": t0, "eventos": ev, "muestras": mu}


@pytest.mark.parametrize("nombre", [e["nombre"] for e in ESC["escenarios"]])
def test_mock_igual_que_matlab(nombre):
    esc = next(e for e in ESC["escenarios"] if e["nombre"] == nombre)
    ref = next(r for r in REF["escenarios"] if r["nombre"] == nombre)
    assert REF["dt"] == ESC["dt"]
    r = _correr_escenario(esc, ESC["base"], ESC["dt"])
    assert r["t0"] is not None
    inicios = [p[1] for p in esc["perdidas"]]
    tol_planta = TOL_T_PLANTA.get(nombre, TOL_T)
    for c, campo in CAMPOS.items():
        a = [(t - r["t0"], v) for t, cc, v in r["eventos"] if cc == c]
        b = [(t - ref["t0"], v) for t, cc, v in ref["eventos"] if cc == c]
        assert [v for _, v in a] == [v for _, v in b], f"{nombre}: {campo} mock {a} / MATLAB {b}"
        for (ta, va), (tb, _) in zip(a, b):
            if tb < -1.0:
                continue                                # antes de t0 - 1 s: INIT dura distinto (solo el valor)
            de_perdida = any(0.0 <= tb - t1 <= 1.0 for t1 in inicios)
            tol = TOL_T if de_perdida else tol_planta
            assert abs(ta - tb) <= tol, f"{nombre}: {campo} = {va} a {ta:.3f} s (MATLAB {tb:.3f} s)"
    # la secuencia de estados es la de la referencia (resumen legible)
    assert [v for _, c, v in r["eventos"] if c == 1] == ref["estados"]
    mm = {round(x[0] - r["t0"], 2): x[1:] for x in r["muestras"]}
    dmax, n = 0.0, 0
    for x in ref["muestras"]:
        k = round(x[0] - ref["t0"], 2)
        if k in mm:
            n += 1
            a = mm[k]
            dmax = max(dmax, math.hypot(a[0] - x[1], a[1] - x[2]), abs(a[2] - x[3]),
                       math.hypot(a[3] - x[4], a[4] - x[5]), abs(a[5] - x[6]))
    assert n >= 0.95 * len(ref["muestras"]) and dmax <= TOL_POS.get(nombre, 0.01), (nombre, n, dmax)
    fin_m, fin_r = r["muestras"][-1][1:], ref["muestras"][-1][1:]
    assert max(abs(p - q) for p, q in zip(fin_m, fin_r)) <= 0.02


def test_referencia_cubre_solo_y_dual():
    est = {r["nombre"]: r["estados"] for r in REF["escenarios"]}
    assert est["solo_mision_permanente"][-3:] == [P.MISSION, P.LANDING, P.DONE]
    assert est["dual_hover_permanente"][-4:] == [P.HOVER_READY, P.ABORT, P.LANDING, P.DONE]
    assert est["dual_mision_umbral_05"][-6:] == [P.PAUSED, P.MISSION, P.PAUSED, P.ABORT, P.LANDING, P.DONE]
    assert all(P.PAUSED not in v for k, v in est.items() if k != "dual_mision_umbral_05")


# =============================================================================== estación / UI
def test_ui_dice_que_dron_aterriza_por_perdida_de_rastreo(cfg_path):
    st = _station(cfg_path)
    now = time.time()
    tel = _tel(mission_state=P.ABORT, alerta=2, drones={1: {"tracked": 1, "fsm_state": 2}, 2: {"tracked": 0}})
    st._on_telemetry(tel, now)
    assert "aterrizaje por pérdida de rastreo (dron 2)" in st._tel_ui(tel)["alerta_texto"].lower()
    msgs = [e["msg"] for e in st.events]
    assert any("Dron 2: aterrizaje por pérdida de rastreo" in m for m in msgs)
    assert any(m.startswith("ALERTA 2") and "(dron 2)" in m for m in msgs)
    # el rastreo vuelve pero la alerta sigue: se recuerda qué dron fue
    tel = _tel(mission_state=P.LANDING, alerta=2, drones={1: {"tracked": 1}, 2: {"tracked": 1}})
    st._on_telemetry(tel, now)
    assert "(dron 2)" in st._tel_ui(tel)["alerta_texto"]
    # otra alerta: se olvida
    st._on_telemetry(_tel(alerta=0), now)
    tel = _tel(mission_state=P.PAUSED, alerta=2, drones={1: {"tracked": 0}, 2: {"tracked": 1}})
    st._on_telemetry(tel, now)
    ui = st._tel_ui(tel)["alerta_texto"]
    assert "PAUSA" in ui and "(dron 1)" in ui and "dron 2" not in ui
    # sin alerta 2 la telemetría pasa tal cual
    tel = _tel(alerta=4)
    assert st._tel_ui(tel) is tel


def test_ui_con_el_mock_solo_explorador(cfg_path_solo):
    st = Station(cfg_path_solo, "mock", mock_speed=4.0)
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

        assert esperar(lambda: est().get("mission_state") == P.ASSIGN_ROLES)
        time.sleep(0.3)
        assert c.post("/api/cmd", json={"cmd": "START"}).json()["ok"]
        assert esperar(lambda: est().get("mission_state") == P.MISSION, 30)
        st.mock.perder_rastreo(1)
        tel = esperar(lambda: (lambda t: t if t.get("alerta") == 2 and t.get("mission_state") == P.LANDING
                               else None)(est()), 10)
        assert tel and tel["drones"]["1"]["tracked"] == 0 and tel["drones"]["1"]["x"] is None
        assert "aterrizaje por pérdida de rastreo (dron 1)" in tel["alerta_texto"].lower()
