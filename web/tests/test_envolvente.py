"""Envolvente por eje de las consignas xy (mision.limites_x / mision.limites_y).

Decisión del usuario (25-nov): el respondedor llega en y+ hasta 1.2 (OptiTrack pierde el dron en y >= ~1.0 cerca de
x ~1.3, vuelos 2-4 del 24-nov) y en y- hasta -1.4 (hacia y- rastrea más lejos). Una toma fuera se marca desde el
punto más cercano dentro. 26-nov: en DUAL (solo ``config/estacion_dual.json``) también ``limites_x`` = [-1.35, 1.2],
para que ninguna consigna de los dos drones vaya a la esquina x 1.2-1.35, y > 0.95 (OptiTrack pierde el dron);
``estacion.json`` (solo explorador, que ya vuela bien) no la lleva y no cambia. Mismas reglas que ``qd2_params_from_config.m`` / ``qd2_limites_xy.m`` (``config.py``) y el
mock (``mock_sim.py``):

1. reglas de la config (ausente / null / [] / inválido / recortado a limites_xy);
2. mock dual con la ruta y los tubos REALES de ``config/estacion_dual.json``: las consignas de R y de E nunca pasan
   de x [-1.35, 1.2] ni de y [-1.4, 1.2], la posición simulada de R tampoco (más una tolerancia), las 4 anomalías
   quedan visitadas y se reporta la separación horizontal mínima en el aire;
3. sin las claves el mock recorta como MATLAB: limites_xy -/+ 0.15 (±1.35, = qd2_limites_xy; antes ±1.4); con una
   sola clave solo cambia ese eje;
4. la reacción a la pérdida de rastreo (apartarse >= 1.60 m) sigue siendo posible dentro de la envolvente;
5. estación: ``envolvente`` en el estado y en la info del vuelo, avisos (clave inválida, ruta fuera).
"""
import copy
import json
import math
import os

import numpy as np
import pytest

from app import protocol as P
from app.config import XY_MARGEN, mision_envolvente, vertices_fuera_envolvente
from app.mock_sim import MockMission, hdist
from app.mock_world import anomalias_mock
from app.server import Station

WEB = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG_DUAL = os.path.join(os.path.dirname(WEB), "config", "estacion_dual.json")
DT = 0.02
Y_LO, Y_HI = -1.4, 1.2          # decisión del usuario (config/estacion.json y estacion_dual.json)
X_LO, X_HI = -1.35, 1.2         # decisión del usuario, 26-nov: solo en dual (config/estacion_dual.json)
TOL_POS = 0.06                  # m, lo que la planta del mock puede pasarse de la consigna recortada
D_SEP = 0.9 + 0.15 + 0.05 + 0.5  # m, d_min_h + guard_margen + 0.05 + d_deriva_sin_rastreo (= 1.60)


def _dual():
    with open(CFG_DUAL, encoding="utf-8") as fh:
        return json.load(fh)


def _sin_env(cfg):
    cfg = copy.deepcopy(cfg)
    for k in ("limites_x", "limites_y"):
        cfg["mision"].pop(k, None)
    return cfg


# =============================================================================== 1. reglas de la config
@pytest.mark.parametrize("mision, x, y, aviso", [
    ({}, None, None, None),
    ({"limites_y": None}, None, None, None),
    ({"limites_y": []}, None, None, None),
    ({"limites_y": [-1.4, 1.2]}, None, [-1.4, 1.2], None),
    ({"limites_y": [-1.6, 1.2], "limites_x": [-1, 1]}, [-1.0, 1.0], [-1.5, 1.2], "se sale de mision.limites_xy"),
    ({"limites_y": [1.2, -1.4]}, None, None, "inválido"),
    ({"limites_y": [1]}, None, None, "inválido"),
    ({"limites_y": "a"}, None, None, "inválido"),
    ({"limites_y": [True, False]}, None, None, "inválido"),
    ({"limites_y": [-1.4, float("nan")]}, None, None, "inválido"),
    ({"limites_y": [[-1.4, 1.2]]}, None, None, "inválido"),
    ({"limites_y": [2, 3]}, None, None, "queda fuera"),
])
def test_reglas_config(mision, x, y, aviso):
    """Mismos casos que test_envolvente_config de simulink_qdrone2/tests/test_qd2_lib.m."""
    e = mision_envolvente({"mision": dict(mision, limites_xy=[-1.5, 1.5])})
    assert e["x"] == x and e["y"] == y
    assert e["definida"] == (x is not None or y is not None)
    assert (e["aviso"] is None) if aviso is None else (aviso in e["aviso"])
    # límite efectivo: el eje con envolvente la usa tal cual (sin margen); el otro limites_xy -/+ 0.15
    for i, v in enumerate((x, y)):
        exp = v if v is not None else [-1.5 + XY_MARGEN, 1.5 - XY_MARGEN]
        assert [e["lo"][i], e["hi"][i]] == exp


def test_configs_del_repo():
    """Las dos configs del repo traen la envolvente del usuario y su ruta queda dentro: y [-1.4, 1.2] en las dos;
    x [-1.35, 1.2] solo en estacion_dual.json (26-nov; el solo explorador sigue en x ±1.35 de limites_xy)."""
    raiz = os.path.dirname(WEB)
    for nombre, x in (("estacion.json", None), ("estacion_dual.json", [X_LO, X_HI])):
        with open(os.path.join(raiz, "config", nombre), "rb") as fh:
            raw = fh.read()
        assert b'    "limites_y": [-1.4, 1.2],\r\n' in raw          # CRLF, solo se agregó la clave
        assert (b'    "limites_x": [-1.35, 1.2],\r\n' in raw) == (x is not None)
        assert b"\n" not in raw.replace(b"\r\n", b"")                 # todo CRLF
        cfg = json.loads(raw.decode("utf-8"))
        e = mision_envolvente(cfg)
        assert e["y"] == [Y_LO, Y_HI] and e["x"] == x and e["aviso"] is None
        assert [e["lo"][0], e["hi"][0]] == pytest.approx(x or [-1.35, 1.35])
        assert vertices_fuera_envolvente(cfg) == []
    cfg = _dual()
    cfg["pista"]["vertices"][1] = [0.99, 1.3]
    assert vertices_fuera_envolvente(cfg) == [[0.99, 1.3]]


# =============================================================================== 2. mock dual con la ruta real
def _correr(cfg, homes=((0.9, -0.4), (-0.9, 0.4)), perder=None, t_max=400):
    """Misión dual completa del mock con las anomalías de ``anomalias_mock`` (sobre los tubos reales).
    Devuelve (mock, anomalías, registro): por paso con R en el aire [t, sp_x, sp_y, x, y, z, marcando, spE_x, spE_y]
    (spE = consigna del explorador en ese paso, NaN si está en el piso)."""
    m = MockMission(cfg, homes)
    if perder:
        m.perder_rastreo(*perder)
    anom = anomalias_mock(cfg)
    slots = [(i + 1, a["cls"], a["x"], a["y"], 0.9) for i, a in enumerate(anom)]
    hb = seq = cmd = n = 0
    reg = []
    for k in range(int(t_max / DT)):
        if m.state == P.ASSIGN_ROLES and seq == 0 and k * DT > 1.5:
            seq, cmd = 1, P.COMMANDS["START"]
        if m.state == P.MISSION and m.state_t > 5:
            n = len(slots)
        if m.state == P.RETURN_HOME and m.state_t > 6 and seq == 1:
            seq, cmd = 2, P.COMMANDS["LAND_ALL"]
        hb += 1
        msg = P.unpack_command(P.pack_command(hb, cmd, seq, n, slots[:3]))
        slots = slots[3:] + slots[:3]
        m.apply_command_message(msg)
        m.step(DT)
        if m.R:
            d = m.drones[m.R]
            if d.flying:
                e = m.drones[m.E]
                spE = (e.sp[0], e.sp[1]) if e.flying else (math.nan, math.nan)
                reg.append((m.t, d.sp[0], d.sp[1], d.p[0], d.p[1], d.p[2], float(m.mark_t > 0), *spE))
        if m.state == P.DONE:
            break
    return m, anom, np.array(reg)


@pytest.mark.parametrize("version", ["PID", "MPC"])
def test_mock_dual_ruta_real(version, capsys):
    cfg = dict(_dual(), version_control=version)
    m, anom, reg = _correr(cfg)
    assert m.state == P.DONE
    # anomalías de la pista real: dos quedan fuera de la envolvente (y = 1.40 y -1.476)
    fuera = [a for a in anom if not (Y_LO <= a["y"] <= Y_HI)]
    assert len(anom) == 4 and len(fuera) == 2
    assert len(m.visited) == 4, f"visitadas {sorted(m.visited)}"
    sp_y, py = reg[:, 2], reg[:, 4]
    assert sp_y.max() <= Y_HI + 1e-12 and sp_y.min() >= Y_LO - 1e-12
    assert py.max() <= Y_HI + TOL_POS and py.min() >= Y_LO - TOL_POS
    # sí llegó a las orillas (se marca desde el punto más cercano dentro)
    assert sp_y.max() == pytest.approx(Y_HI) and sp_y.min() == pytest.approx(Y_LO)
    # eje x con envolvente (26-nov, solo dual): R nunca a la esquina x 1.2-1.35 (antes ±1.35 de limites_xy)
    sp_x, px = reg[:, 1], reg[:, 3]
    assert sp_x.max() <= X_HI + 1e-12 and sp_x.min() >= X_LO - 1e-12
    assert px.max() <= X_HI + TOL_POS and px.min() >= X_LO - TOL_POS
    # el explorador tampoco sale (su ruta queda dentro)
    e = reg[~np.isnan(reg[:, 7])]
    assert len(e) and e[:, 7].max() <= X_HI + 1e-12 and e[:, 7].min() >= X_LO - 1e-12
    assert e[:, 8].max() <= Y_HI + 1e-12 and e[:, 8].min() >= Y_LO - 1e-12
    with capsys.disabled():
        print(f"\n[envolvente mock {version}] R: consigna x {sp_x.min():+.3f}..{sp_x.max():+.3f}, "
              f"y {sp_y.min():+.3f}..{sp_y.max():+.3f}, posición y {py.min():+.3f}..{py.max():+.3f}; E: consigna x "
              f"{e[:, 7].min():+.3f}..{e[:, 7].max():+.3f}; visitadas {len(m.visited)}/4; separación horizontal mínima "
              f"en el aire {m.min_hdist_flying:.3f} m")
    if version == "PID":
        assert m.min_hdist_flying > cfg["mision"]["d_min_h"] - 0.05


def test_mock_sin_envolvente_recorta_como_matlab():
    """Sin las claves: misión simplificada y caminos portados de MATLAB a limites_xy -/+ 0.15 (±1.35, = qd2_limites_xy;
    antes la simplificada iba a ±1.4), y la consigna final no se recorta en esos ejes (como el guard de MATLAB, que
    solo recorta su salida en los ejes con envolvente). Con una clave solo cambia el eje que la trae (estacion.json:
    solo limites_y); estacion_dual.json trae las dos (26-nov: limites_x [-1.35, 1.2])."""
    m = MockMission(_sin_env(_dual()))
    assert list(m.lim_lo) == [-1.5 + 0.15] * 2 and list(m.lim_hi) == [1.5 - 0.15] * 2
    assert list(m.xy_lo) == [-1.5 + 0.15] * 2 and list(m.xy_hi) == [1.5 - 0.15] * 2
    assert list(m.sp_lo) == [-math.inf] * 2 and list(m.sp_hi) == [math.inf] * 2
    solo_y = _dual()
    solo_y["mision"].pop("limites_x")
    m = MockMission(solo_y)
    assert list(m.lim_lo) == [-1.5 + 0.15, Y_LO] and list(m.lim_hi) == [1.5 - 0.15, Y_HI]
    assert list(m.xy_lo) == [-1.5 + 0.15, Y_LO] and list(m.xy_hi) == [1.5 - 0.15, Y_HI]
    assert list(m.sp_lo) == [-math.inf, Y_LO] and list(m.sp_hi) == [math.inf, Y_HI]
    m = MockMission(_dual())
    assert list(m.lim_lo) == [X_LO, Y_LO] and list(m.lim_hi) == [X_HI, Y_HI]
    assert list(m.xy_lo) == [X_LO, Y_LO] and list(m.xy_hi) == [X_HI, Y_HI]
    assert list(m.sp_lo) == [X_LO, Y_LO] and list(m.sp_hi) == [X_HI, Y_HI]


def test_mock_home_fuera_de_la_envolvente():
    """Home fuera de la envolvente (limites_x hasta 1.1, R en el piso en x = 1.25): se recorta al capturarlo (como
    qd2_mission_step), así que el regreso llega "en home" y auto_land aterriza (antes se quedaba en RETURN_HOME)."""
    cfg = _dual()
    cfg["mision"] = dict(cfg["mision"], limites_x=[-1.35, 1.1], auto_land=True)
    m = MockMission(cfg, ((1.25, -0.4), (-0.9, 0.4)))
    hb = seq = cmd = 0
    homes = None
    for k in range(int(300 / DT)):
        if m.state == P.ASSIGN_ROLES and seq == 0 and k * DT > 1.5:
            seq, cmd = 1, P.COMMANDS["START"]
            homes = {i: tuple(d.home) for i, d in m.drones.items()}
        hb += 1
        m.apply_command_message(P.unpack_command(P.pack_command(hb, cmd, seq, 0, [])))
        m.step(DT)
        if m.state == P.DONE:
            break
    assert homes[1] == pytest.approx((1.1, -0.4)) and homes[2] == pytest.approx((-0.9, 0.4))
    assert m.state == P.DONE


def test_mock_pausa_con_R_pasado_de_la_envolvente():
    """PAUSE justo cuando R, por sobretiro, pasó de x 1.2 marcando una toma de la orilla x+ (1.45, 0): la consigna
    de hover se recorta a 1.2 (recortar_sp de MATLAB corre en todos los estados; antes el mock mandaba la pose,
    x ~1.201)."""
    m = MockMission(_dual(), ((0.9, -0.4), (-0.9, 0.4)))
    slots = [(1, 1, 1.45, 0.0, 0.95)]
    hb = seq = cmd = n = 0
    sp_pausa, x_pausa = [], None
    for k in range(int(200 / DT)):
        if m.state == P.ASSIGN_ROLES and seq == 0 and k * DT > 1.5:
            seq, cmd = 1, P.COMMANDS["START"]
        if m.state == P.MISSION and m.state_t > 5:
            n = 1
        R = m.R
        if (x_pausa is None and R and m.state == P.MISSION and m.drones[R].flying
                and m.drones[R].p[0] > X_HI + 1e-3):
            seq, cmd, x_pausa = seq + 1, P.COMMANDS["PAUSE"], m.drones[R].p[0]
        hb += 1
        m.apply_command_message(P.unpack_command(P.pack_command(hb, cmd, seq, n, slots)))
        m.step(DT)
        if m.state == P.PAUSED:
            sp_pausa.append(m.drones[m.R].sp[0])
            if len(sp_pausa) > 100:
                break
    assert x_pausa is not None and x_pausa > X_HI, "R nunca pasó de x 1.2 (cambió la planta del mock)"
    assert sp_pausa and max(sp_pausa) == pytest.approx(X_HI)


def test_mock_consignas_de_pose_recortadas():
    """Los caminos que ponen la consigna en la pose (pausa, pausa por rastreo con la pose vista, cancelar despegue,
    aterrizaje de los caminos de pérdida de rastreo) la recortan a la envolvente, como recortar_sp de MATLAB."""
    def nuevo():
        m = MockMission(_dual(), ((0.9, -0.4), (-0.9, 0.4)))
        m.E, m.R = 2, 1
        m.drones[1].home, m.drones[2].home = np.array([0.9, -0.4]), np.array([-0.9, 0.4])
        for k, p in ((1, (1.22, 0.0, 0.5)), (2, (-1.0, 1.25, 1.5))):     # R pasado en x, E pasado en y
            m.drones[k].p = np.array(p)
            m.vista[k] = np.array(p)
        return m

    def dentro(m):
        for d in m.drones.values():
            assert X_LO - 1e-12 <= d.sp[0] <= X_HI + 1e-12 and Y_LO - 1e-12 <= d.sp[1] <= Y_HI + 1e-12, d.sp
        assert m.drones[1].sp[0] == pytest.approx(X_HI) and m.drones[2].sp[1] == pytest.approx(Y_HI)

    m = nuevo()
    m._pause()
    dentro(m)
    assert m.drones[1].sp[2] == pytest.approx(0.5)            # z: la pose (sin cambio)
    m = nuevo()
    m._pausar_vista()
    dentro(m)
    for dado in (False, True):
        m = nuevo()
        m.takeoff_dado = dado
        m._cancelar_despegue()
        dentro(m)
    m = nuevo()
    m._planear_aterrizaje()
    for _ in range(3):
        m._paso_aterrizaje()
        dentro(m)


# =============================================================================== 4. reacción a falla posible
def test_apartarse_posible_dentro_de_la_envolvente():
    """punto_separacion (portado de MATLAB): desde cualquier punto de la envolvente del dron perdido el sano tiene un
    punto a >= 1.60 m dentro de la envolvente (x -1.35..1.2, y -1.4..1.2 de estacion_dual.json: la esquina más
    lejana queda a >= la media diagonal, 1.82 m)."""
    m = MockMission(_dual())
    lo, hi = m.xy_lo, m.xy_hi
    peor = math.inf
    for fx in np.linspace(lo[0], hi[0], 15):
        for fy in np.linspace(lo[1], hi[1], 15):
            for ph in ((fx + 0.3, fy), (fx, fy - 0.2), (0.0, 0.0)):
                c = m._punto_separacion(np.array([*ph, 0.5]), np.array([fx, fy, 1.5]), np.array([0.0, 0.0]), D_SEP)
                assert np.all(c >= lo - 1e-12) and np.all(c <= hi + 1e-12)
                peor = min(peor, hdist(c, (fx, fy)))
    assert peor >= D_SEP - 1e-6


def test_mock_R_pierde_rastreo_en_la_orilla(capsys):
    """R pierde el rastreo marcando la toma de la orilla y+ (marca desde y = 1.2): aterriza ahí; E se aparta a
    >= 1.60 m dentro de la envolvente, regresa y aterriza (9 -> 7 -> 8)."""
    cfg = _dual()
    m0, _, reg = _correr(cfg)
    # R marcando la toma de la orilla (0.46, 1.40) desde (0.46, 1.2): a la mitad del hover de marca
    k = next(i for i, r in enumerate(reg) if r[6] > 0 and r[2] == pytest.approx(Y_HI))
    t_perd = float(reg[k, 0]) + 1.5
    m, _, _ = _correr(cfg, perder=(m0.R, t_perd))
    assert m.state == P.DONE and m.falla[m0.R] == 2
    assert m.vista[m0.R][1] == pytest.approx(Y_HI, abs=TOL_POS)     # se perdió en la orilla y+
    ev = " | ".join(e for _, e in m.events)
    assert "se aparta" in ev
    assert np.all(m.falla_sep >= m.xy_lo - 1e-12) and np.all(m.falla_sep <= m.xy_hi + 1e-12)
    d = hdist(m.falla_sep, m.vista[m0.R])
    assert d >= D_SEP - 1e-6
    with capsys.disabled():
        print(f"\n[envolvente falla] R sin rastreo en t={t_perd:.1f} s en ({m.vista[m0.R][0]:+.2f}, "
              f"{m.vista[m0.R][1]:+.2f}); E se aparta a ({m.falla_sep[0]:+.2f}, {m.falla_sep[1]:+.2f}), a {d:.2f} m")


# =============================================================================== 5. estación
def test_estacion_estado_y_avisos(tmp_path):
    f = tmp_path / "estacion.json"
    cfg = _dual()
    f.write_text(json.dumps(cfg), encoding="utf-8")
    st = Station(str(f), "simulink")
    assert st.envolvente_ui() == {"x": [X_LO, X_HI], "y": [Y_LO, Y_HI], "ejes": ["x", "y"]}
    assert not any("envolvente" in a["msg"] or "limites_y" in a["msg"] for a in st.avisos())
    # sin claves: sin envolvente (nada se dibuja ni se avisa)
    f.write_text(json.dumps(_sin_env(cfg)), encoding="utf-8")
    assert Station(str(f), "simulink").envolvente_ui() is None
    # clave inválida y ruta fuera: avisos amarillos (no bloquean nada)
    c2 = copy.deepcopy(cfg)
    c2["mision"]["limites_x"] = [1, -1]
    c2["pista"]["vertices"][1] = [0.99, 1.35]
    f.write_text(json.dumps(c2), encoding="utf-8")
    av = Station(str(f), "simulink").avisos()
    assert any(a["nivel"] == "amarillo" and "mision.limites_x inválido" in a["msg"] for a in av)
    assert any(a["nivel"] == "amarillo" and "Ruta fuera de la envolvente" in a["msg"] and "+1.35" in a["msg"]
               for a in av)
