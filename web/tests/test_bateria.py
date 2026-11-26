"""Batería (app/bateria.py): SOLO visualización y registro; nada actúa por ella.

* ``HistorialBateria``: media de 1 s de battery_V solo con valores finitos; "caída desde el despegue" contra el último
  dato en reposo; estimación informativa con > 20 s en el aire; payload de la UI acotado (5 min, una cada 2 s);
* panel con NaN: en PID (modelo del Equipo2) "sin dato en vivo" y nada que dibujar; en modo solo solo el dron solo;
* registro de vuelo: líneas ``bateria`` solo con V finito y del dron activo, resumen por dron (null sin dato), las
  líneas de batería no hacen que un vuelo deje archivo y ``ajustar_tubos.py`` las ignora;
* mock: ~16.4 V en reposo, ~0.9 V de caída con carga y descarga en vuelo, sin reaccionar a la batería;
* ninguna orden, alerta, aviso ni cambio de misión depende de battery_V (estático y con la estación).
"""
import ast
import glob
import importlib.util
import json
import math
import os
import time

import pytest
from fastapi.testclient import TestClient

from app import bateria as B
from app import protocol as P
from app.bateria import HistorialBateria
from app.mock_sim import MockMission
from app.registro_vuelo import RegistroVuelos
from app.server import Station, create_app
from test_alertas_objetivos_sim import _tel
from test_mock_mision import Driver
from test_registro_vuelo import _Puente, _Reloj, _leer, _obs

WEB = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
T0 = 1_700_000_000.0


def _t(version=2, estado=P.MISSION, ds=0, **drones):
    """Telemetría como dict (sin empaquetar). drones: d1=(V, z, fsm), d2=(V, z, fsm)."""
    out = {"version": version, "mission_state": estado, "dron_solo": ds, "drones": {}}
    for k in (1, 2):
        V, z, fsm = drones.get(f"d{k}", (None, 0.0, 1))
        out["drones"][k] = {"battery_V": V, "z": z, "fsm_state": fsm, "tracked": 1}
    return out


def _vuelo_sintetico(h, t0=T0, hz=50, reposo_s=10, armado_s=2, aire_s=60, pendiente=-0.01, version=2):
    """Reposo a 16.4 V (FSM 1), armado a 15.7 V (FSM 2) y en el aire 15.5 V + pendiente·t (FSM 5, z = 1 m)."""
    muestras, t = [], t0
    for _ in range(int(reposo_s * hz)):
        muestras += h.agregar(_t(version, P.ASSIGN_ROLES, d1=(16.4, 0.0, 1)), t)
        t += 1 / hz
    for _ in range(int(armado_s * hz)):
        muestras += h.agregar(_t(version, P.TAKEOFF, d1=(15.7, 0.0, 2)), t)
        t += 1 / hz
    ta = t
    for _ in range(int(aire_s * hz)):
        muestras += h.agregar(_t(version, P.MISSION, d1=(15.5 + pendiente * (t - ta), 1.0, 5)), t)
        t += 1 / hz
    return muestras, t


# =============================================================================== HistorialBateria
def test_media_de_1_s_solo_con_valores_finitos():
    h = HistorialBateria()
    assert h.agregar(_t(d1=(16.0, 0.0, 1), d2=(15.0, 0.0, 1)), T0 + 0.1) == []
    h.agregar(_t(d1=(float("nan"), 0.0, 1), d2=(None, 0.0, 1)), T0 + 0.4)     # NaN / None no cuentan
    h.agregar(_t(d1=(16.2, 0.0, 1), d2=(float("inf"), 0.0, 1)), T0 + 0.7)
    m = h.agregar(_t(d1=(None, 0.0, 1), d2=(None, 0.0, 1)), T0 + 1.2)         # cierra el segundo T0
    assert sorted((x["dron"], x["V"]) for x in m) == [(1, 16.1), (2, 15.0)]
    assert all(x["t"] == T0 + 0.5 and x["estado_mision"] == P.MISSION and x["fsm"] == 1 and x["z"] == 0.0 for x in m)
    # un segundo sin ningún valor finito no deja muestra
    assert h.agregar(_t(d1=(None, 0.0, 1)), T0 + 2.1) == []
    assert all(math.isfinite(s[1]) for d in (1, 2) for s in h._d[d].serie)


def test_caida_desde_reposo_y_estimacion_con_mas_de_20_s_en_el_aire():
    h = HistorialBateria()
    _, t = _vuelo_sintetico(h, aire_s=15)
    e = h.estado(_t(d1=(15.35, 1.0, 5)), (1,), t)["drones"]["1"]
    assert e["en_aire"] and e["estado"] == "bajo carga: en vuelo" and e["sin_dato"] is None
    assert e["v_despegue"] == 16.4 and e["ref_despegue"] == "reposo"         # no el dato ya armado (15.7 V)
    assert e["caida"] == pytest.approx(16.4 - 15.35)
    assert e["aire_s"] <= B.AIRE_MIN_S and e["pendiente_V_s"] is None and e["min_hasta_14"] is None
    h = HistorialBateria()
    _, t = _vuelo_sintetico(h, aire_s=60)
    v_ahora = 15.5 - 0.01 * 60
    e = h.estado(_t(d1=(v_ahora, 1.0, 5)), (1,), t)["drones"]["1"]
    assert e["aire_s"] > B.AIRE_MIN_S and e["pendiente_V_s"] == pytest.approx(-0.01, abs=2e-4)
    assert e["min_hasta_14"] == pytest.approx((v_ahora - 14.0) / 0.01 / 60, abs=0.15)
    # en tierra no hay estimación (la caída sigue a la vista)
    h.agregar(_t(estado=P.DONE, d1=(14.9, 0.0, 9)), t + 1.0)
    h.agregar(_t(estado=P.DONE, d1=(14.9, 0.0, 9)), t + 2.0)
    e = h.estado(_t(estado=P.DONE, d1=(14.9, 0.0, 9)), (1,), t + 2.0)["drones"]["1"]
    assert not e["en_aire"] and e["min_hasta_14"] is None and e["caida"] == pytest.approx(1.5)
    assert e["estado"] == "en reposo (motores deshabilitados)"
    # subiendo el voltaje (no baja): sin estimación
    h = HistorialBateria()
    _, t = _vuelo_sintetico(h, aire_s=40, pendiente=+0.001)
    assert h.estado(_t(d1=(15.6, 1.0, 5)), (1,), t)["drones"]["1"]["min_hasta_14"] is None


def test_en_aire_con_el_z_rastreado_y_estimacion_sin_falso_cero():
    # PID real: battery_V siempre NaN (serie vacía); el estado sale del z rastreado de la telemetría actual
    h = HistorialBateria()
    for k in range(200):
        h.agregar(_t(version=1, d1=(None, 1.5, 2)), T0 + 0.02 * k)
    e = h.estado(_t(version=1, d1=(None, 1.5, 2)), (1,), T0 + 4)["drones"]["1"]
    assert e["sin_dato"] == "pid" and e["en_aire"] and e["estado"] == "bajo carga: en el aire"
    e = h.estado(_t(version=1, d1=(None, 0.8, 3)), (1,), T0 + 4)["drones"]["1"]       # Land, todavía bajando
    assert e["en_aire"] and e["estado"] == "bajo carga: en el aire"
    e = h.estado(_t(version=1, d1=(None, 0.0, 3)), (1,), T0 + 4)["drones"]["1"]
    assert not e["en_aire"] and e["estado"] == "en reposo (en tierra)"
    sin_z = _t(version=1, d1=(None, 1.5, 3))
    sin_z["drones"][1]["tracked"] = 0                                                  # sin rastreo: no se sabe
    e = h.estado(sin_z, (1,), T0 + 4)["drones"]["1"]
    assert not e["en_aire"] and e["estado"] == "aterrizaje (Land): bajando o ya en tierra"
    sin_z["drones"][1]["fsm_state"] = 2
    assert h.estado(sin_z, (1,), T0 + 4)["drones"]["1"]["estado"] == "bajo carga: despegue / en vuelo"
    # el redondeo a 0.1 min no da 0.0 si la recta sigue arriba de 14.0 V (14.02 V a -0.008 V/s ≈ 2.5 s)
    h = HistorialBateria()
    d = h._d[1]
    d.t_despegue = T0
    serie = [(T0 + 0.5 + i, 14.02 + 0.008 * (39 - i), True) for i in range(40)]
    assert h._estimacion(d, serie, T0 + 40)["min_hasta_14"] == 0.1
    serie = [(t, V - 0.05, a) for t, V, a in serie]
    assert h._estimacion(d, serie, T0 + 40)["min_hasta_14"] == 0.0


def test_payload_de_la_ui_acotado():
    h = HistorialBateria()
    _, t = _vuelo_sintetico(h, hz=20, reposo_s=200, aire_s=400, pendiente=-0.002)
    e = h.estado(_t(d1=(14.7, 1.0, 5)), (1,), t)["drones"]["1"]
    assert len(e["serie"]) <= B.VENTANA_S // B.PASO_UI_S + 2
    assert all(-B.VENTANA_S <= s[0] <= 0 and s[2] in (0, 1) for s in e["serie"])
    assert len(json.dumps(e)) < 6000


def test_sin_dato_en_vivo_pid_nan_y_sin_telemetria():
    h = HistorialBateria()
    _vuelo_sintetico(h, aire_s=5)          # historia vieja con dato: no se dibuja si ahora llega NaN
    e = h.estado(_t(version=1, d1=(None, 1.0, 2), d2=(None, 0.0, 0)), (1, 2), T0 + 20)["drones"]
    assert set(e) == {"1", "2"}
    assert all(f["v"] is None and f["sin_dato"] == "pid" and f["serie"] == [] and f["caida"] is None
               and f["min_hasta_14"] is None for f in e.values())
    e = h.estado(_t(version=2, d1=(None, 1.0, 5)), (1,), T0 + 20)["drones"]["1"]
    assert e["sin_dato"] == "nan" and e["serie"] == []
    e = h.estado(None, (1,), T0 + 20)["drones"]["1"]
    assert e["sin_dato"] == "sin_tel" and e["estado"] == "sin telemetría"
    ref = h.estado(None, (1,), T0)["ref"]
    assert ref == {"v_aterrizaje_a_bordo": 14.0, "t_aterrizaje_a_bordo_s": 0.3, "v_recomendada_despegue": 15.5}


def test_panel_de_la_estacion_pid_sin_dato_y_modo_solo(cfg_path, cfg_path_solo):
    for path, esperado in ((cfg_path, {"1", "2"}), (cfg_path_solo, {"1"})):
        st = Station(path, "simulink")
        st.bridge = _Puente()
        # el modelo de tierra PID manda NaN en o+14 (dron del Equipo2: solo eco del timestamp)
        tel = _tel(version=1, dron_solo=1 if esperado == {"1"} else 0,
                   drones={1: {"tracked": 1, "fsm_state": 2, "z": 1.0, "battery_V": float("nan")},
                           2: {"tracked": 1, "fsm_state": 2, "z": 0.0, "battery_V": float("nan")}})
        assert tel["drones"][1]["battery_V"] is None
        for k in range(30):
            st.bridge.telemetry = tel
            st._on_telemetry(tel, time.time() + 0.1 * k)
        with TestClient(create_app(st, start=False)) as c:
            b = c.get("/api/estado").json()["bateria"]
        assert set(b["drones"]) == esperado
        assert all(f["sin_dato"] == "pid" and f["v"] is None and f["serie"] == [] for f in b["drones"].values())
    # el texto que ve el operador
    js = open(os.path.join(WEB, "app", "static", "app.js"), encoding="utf-8").read()
    assert ("Sin dato en vivo: el dron PID del Equipo2 no envía la batería a tierra. Revísala después del vuelo con "
            "simulink_qdrone2/bateria/qd2_bateria.m") in js
    assert "14.0 V: aterrizaje automático a bordo" in js.replace("${f2(v14, 1)}", "14.0")
    assert "V: mínimo recomendado para despegar (checklist)" in js


# =============================================================================== registro de vuelo
def _registro(tmp_path, reloj):
    return RegistroVuelos(tmp_path / "vuelos", lambda: "simulink", lambda: True, reloj=reloj, hilo=False)


def test_registro_solo_valores_finitos_y_resumen(tmp_path):
    r = _Reloj()
    reg = _registro(tmp_path, r)
    assert reg.bateria([{"t": r.t, "dron": 1, "V": 16.0}]) is False          # sin vuelo abierto
    reg.tick(True, P.MISSION)
    m = lambda dt, d, V: {"t": r.t + dt, "dron": d, "V": V, "fsm": 5, "estado_mision": P.MISSION, "z": 1.5}
    reg.bateria([m(0.5, 1, 15.6), m(0.5, 2, float("nan")), m(1.5, 1, None), m(1.5, 2, float("inf")),
                 m(2.5, 1, 15.4), m(3.5, 1, 15.5), m(3.5, 2, True)])
    reg.observaciones(r.t + 1, [_obs()])
    r.t += 10
    reg.tick(True, P.LANDING)
    reg.vaciar()
    (f,) = glob.glob(str(tmp_path / "vuelos" / "*" / "*.jsonl"))
    L = _leer(f)
    bat = [l for l in L if l["tipo"] == "bateria"]
    assert [(l["dron"], l["V"], l["t"]) for l in bat] == [(1, 15.6, 0.5), (1, 15.4, 2.5), (1, 15.5, 3.5)]
    assert set(bat[0]) == {"tipo", "t", "dron", "V", "fsm", "estado_mision", "z"} and bat[0]["z"] == 1.5
    res = L[-1]
    assert res["tipo"] == "resumen" and res["lineas"] == len(L)
    assert res["bateria"] == {"1": {"v_inicial": 15.6, "v_final": 15.5, "v_min_suavizada": 15.4, "n_muestras": 3},
                              "2": None}


def test_lineas_de_bateria_no_hacen_que_un_vuelo_deje_archivo(tmp_path):
    r = _Reloj()
    reg = _registro(tmp_path, r)
    reg.tick(True, P.MISSION)
    for k in range(4):                     # 2 s de gate (< min_gate_s) y solo batería
        r.t += 0.5
        reg.tick(True, P.MISSION)
        reg.bateria([{"t": r.t, "dron": 1, "V": 16.0 - 0.01 * k}])
    reg.tick(True, P.DONE)
    reg.vaciar()
    assert glob.glob(str(tmp_path / "vuelos" / "**" / "*.jsonl"), recursive=True) == []


def test_estacion_registra_la_bateria_del_dron_activo_y_ajustar_tubos_la_ignora(cfg_path_lab):
    st = Station(cfg_path_lab, "simulink")          # solo explorador con el dron 1
    st.registro._usar_hilo = False
    st.bridge = _Puente()

    def paso(estado, V1, z, k):
        tel = _tel(mission_state=estado, explorer_id=1, dron_solo=1,
                   drones={1: {"tracked": 1, "fsm_state": 2, "x": 0.5, "y": -1.0, "z": z, "yaw": 0.0,
                               "battery_V": V1},
                           2: {"tracked": 0, "battery_V": 16.1}})        # el no usado también manda: no se registra
        st.bridge.telemetry = tel
        st._on_telemetry(tel, t0 + 0.1 * k)

    t0 = time.time()
    for k in range(80):                      # 8 s (de stamps) en MISSION a 1.5 m: el vuelo se abre
        paso(P.MISSION, 15.5 - 0.001 * k if k % 7 else float("nan"), 1.5, k)
        if k == 40:
            st.registro.observaciones(time.time(), [_obs()])   # con una observación el vuelo deja archivo
    paso(P.LANDING, 15.4, 0.2, 81)
    st.registro.vaciar()
    (f,) = glob.glob(os.path.join(os.path.dirname(cfg_path_lab), "vuelos", "*", "*.jsonl"))
    L = _leer(f)
    bat = [l for l in L if l["tipo"] == "bateria"]
    assert len(bat) >= 6 and {l["dron"] for l in bat} == {1}
    assert all(math.isfinite(l["V"]) and 15.4 < l["V"] <= 15.5 for l in bat)
    assert all(b["t"] > a["t"] for a, b in zip(bat, bat[1:]))
    res = L[-1]
    assert res["bateria"]["2"] is None and res["bateria"]["1"]["n_muestras"] == len(bat)
    assert res["bateria"]["1"]["v_inicial"] >= res["bateria"]["1"]["v_final"]
    # web/herramientas/ajustar_tubos.py lee el mismo archivo sin romperse (las líneas nuevas no son "obs")
    spec = importlib.util.spec_from_file_location("ajustar_tubos", os.path.join(WEB, "herramientas", "ajustar_tubos.py"))
    at = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(at)
    v = at.leer_vuelo(f)
    assert len(v["obs"]) == 1 and v["malas"] == 0 and v["cabecera"] and v["resumen"]["bateria"]["2"] is None


# =============================================================================== mock
def test_mock_bateria_desciende_de_forma_realista(cfg):
    d = Driver(cfg)
    m = d.m
    assert {k: dd.battery for k, dd in m.drones.items()} == {1: 16.4, 2: 16.4}
    assert d.run_until(lambda: m.state == P.ASSIGN_ROLES, 5)
    reposo = {k: dd.battery for k, dd in m.drones.items()}
    assert all(16.39 < v <= 16.4 for v in reposo.values())               # en reposo casi no baja
    d.send("START")
    assert d.run_until(lambda: m.state == P.MISSION, 30)
    serie = []
    for _ in range(int(40 / 0.02)):
        d.step()
        serie.append(m.drones[m.E].battery)
    caida = reposo[m.E] - serie[0]
    assert 0.85 < caida < 1.1                                           # caída por carga (~0.9 V) + algo de vuelo
    assert all(b <= a + 1e-12 for a, b in zip(serie, serie[1:]))        # en vuelo solo baja
    assert serie[0] - serie[-1] == pytest.approx(0.008 * 40, rel=0.05)  # ~0.008 V/s
    # telemetría por el empaquetado real: en PID NaN (como el modelo de tierra del Equipo2), en MPC el voltaje
    version = m.version
    m.version = 1
    assert P.unpack_telemetry(P.pack_telemetry(m.telemetry()))["drones"][m.E]["battery_V"] is None
    m.version = 2
    assert P.unpack_telemetry(P.pack_telemetry(m.telemetry()))["drones"][m.E]["battery_V"] == pytest.approx(serie[-1])
    m.version = version
    # al aterrizar se recupera la caída por carga, pero queda abajo del inicio
    d.send("LAND_ALL")
    assert d.run_until(lambda: m.state == P.DONE, 60)
    d.step(int(3 / 0.02))
    final = m.drones[m.E].battery
    assert serie[-1] + 0.8 < final < reposo[m.E] - 0.3


def test_mock_no_reacciona_a_la_bateria(cfg):
    """Con la batería simulada en el piso (13.5 V, abajo de los 14.0 V del corte a bordo) el mock vuela igual."""
    normal, baja = Driver(cfg), Driver(cfg)
    for dr in (normal, baja):
        assert dr.run_until(lambda: dr.m.state == P.ASSIGN_ROLES, 5)
    for dd in baja.m.drones.values():
        dd.bat_reposo = 13.0
    for dr in (normal, baja):
        dr.send("START")
    for _ in range(int(40 / 0.02)):
        normal.step()
        baja.step()
        assert baja.m.state == normal.m.state and baja.m.alerta == normal.m.alerta
    assert baja.m.state in (P.MISSION, P.RETURN_HOME)
    assert all(dd.battery == B.V_ATERRIZAJE_A_BORDO - 0.5 for dd in baja.m.drones.values())
    for k in (1, 2):
        assert (baja.m.drones[k].p == normal.m.drones[k].p).all() and baja.m.drones[k].fsm == normal.m.drones[k].fsm


@pytest.fixture
def cfg(cfg_path):
    with open(cfg_path, encoding="utf-8") as fh:
        return json.load(fh)


# =============================================================================== nada actúa por la batería
ACCIONES = {"send_command", "command", "_goto", "_pause", "forzar_alerta", "set_targets", "_publish_targets",
            "log", "avisos", "_avisos_modo", "_check_alerta", "cerrar", "abrir", "tick", "_aterrizar", "reset"}
MARCAS = ("battery", "bateria", "bat_")
SEMILLAS = ("battery", "bat_")          # el VALOR de la batería (para seguirlo por variables locales)
DESTINOS = {"state", "alerta", "alerta_forzada", "cmd", "sp", "flying", "fsm", "guard", "falla"}


def _nombre(n):
    return n.attr if isinstance(n, ast.Attribute) else n.id if isinstance(n, ast.Name) else ""


def _menciona_bateria(nodo, locales=frozenset(), marcas=MARCAS):
    """True si ``nodo`` usa la batería: battery / battery_V / bat_* / bateria, o una variable local que la lleva."""
    for n in ast.walk(nodo):
        nom = _nombre(n)
        if nom and (any(m in nom.lower() for m in marcas) or (isinstance(n, ast.Name) and nom in locales)):
            return True
        if isinstance(n, ast.Constant) and isinstance(n.value, str) and n.value == "battery_V":
            return True
    return False


def _locales_con_bateria(fn):
    """Variables locales de ``fn`` asignadas (directa o indirectamente) desde el valor de la batería."""
    locales, cambio = set(), True
    while cambio:
        cambio = False
        for n in ast.walk(fn):
            if isinstance(n, (ast.Assign, ast.AugAssign, ast.AnnAssign, ast.NamedExpr)) and n.value is not None                     and _menciona_bateria(n.value, frozenset(locales), SEMILLAS):
                tgts = n.targets if isinstance(n, ast.Assign) else [n.target]
                for t in tgts:            # solo variables (no ``self.x = ...`` ni ``d[k] = ...``)
                    for x in (t.elts if isinstance(t, (ast.Tuple, ast.List)) else [t]):
                        if isinstance(x, ast.Name) and x.id not in locales:
                            locales.add(x.id)
                            cambio = True
    return frozenset(locales)


def _revisar(archivo, nodo, locales):
    for n in ast.walk(nodo):
        # 1) ninguna condición ni comparación usa la batería (bateria.py solo calcula lo que se muestra: ver 4)
        if archivo != "bateria.py":
            if isinstance(n, (ast.If, ast.While, ast.IfExp, ast.Assert)) and _menciona_bateria(n.test, locales):
                pytest.fail(f"{archivo}:{n.lineno}: una condición depende de la batería")
            if isinstance(n, ast.Compare) and _menciona_bateria(n, locales):
                pytest.fail(f"{archivo}:{n.lineno}: comparación con la batería")
        # 2) ningún comando / alerta / aviso / evento / cambio de misión recibe algo de la batería
        if isinstance(n, ast.Call) and _nombre(n.func) in ACCIONES:
            if any(_menciona_bateria(a, locales) for a in list(n.args) + [k.value for k in n.keywords]):
                pytest.fail(f"{archivo}:{n.lineno}: {_nombre(n.func)}() recibe la batería")
        # 3) la batería no se asigna al estado de la misión, a alertas, consignas ni comandos
        if isinstance(n, (ast.Assign, ast.AugAssign)) and _menciona_bateria(n.value, locales):
            for tgt in (n.targets if isinstance(n, ast.Assign) else [n.target]):
                for x in ast.walk(tgt):
                    assert _nombre(x).lower() not in DESTINOS, f"{archivo}:{n.lineno}: {_nombre(x)} = batería"


@pytest.mark.parametrize("archivo", ["server.py", "mock_sim.py", "protocol.py", "bateria.py", "registro_vuelo.py",
                                     "bridge.py"])
def test_ningun_comando_ni_alerta_depende_de_battery_V(archivo):
    src = open(os.path.join(WEB, "app", archivo), encoding="utf-8").read()
    arbol = ast.parse(src)
    _revisar(archivo, arbol, frozenset())
    for fn in ast.walk(arbol):
        if isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
            _revisar(archivo, fn, _locales_con_bateria(fn))
    # 4) protocol.py: battery_V solo es un campo del bloque del dron; bateria.py no importa nada que pueda actuar
    if archivo == "protocol.py":
        assert src.count("battery") == 1 and '"battery_V": 14' in src
    if archivo == "bateria.py":
        imports = {a.name for n in ast.walk(arbol) if isinstance(n, ast.Import) for a in n.names}
        imports |= {n.module or "" for n in ast.walk(arbol) if isinstance(n, ast.ImportFrom)}
        assert imports <= {"__future__", "collections", "math", "threading"}


def test_estacion_no_actua_con_bateria_baja(cfg_path):
    """La misma telemetría con 16.0 V y con 13.0 V (abajo del corte a bordo): mismos avisos, eventos y comandos."""
    res = []
    for V in (16.0, 13.0):
        st = Station(cfg_path, "simulink", registro_vuelos="no")
        st.bridge = _Puente()
        enviados = []
        st.bridge.send_command = lambda name: enviados.append(name)
        for k in range(40):
            tel = _tel(mission_state=P.MISSION, version=2,
                       drones={1: {"tracked": 1, "fsm_state": 5, "z": 1.5, "battery_V": V},
                               2: {"tracked": 1, "fsm_state": 5, "z": 0.5, "battery_V": V}})
            st.bridge.telemetry = tel
            st._on_telemetry(tel, time.time() + 0.1 * k)
        snap = st.snapshot()
        assert snap["bateria"]["drones"]["1"]["v"] == V
        res.append((snap["avisos"], [e["msg"] for e in snap["eventos"]], enviados, snap["tel"]["alerta"]))
    assert res[0] == res[1]
    assert not any("bater" in e.lower() for e in res[1][1])
