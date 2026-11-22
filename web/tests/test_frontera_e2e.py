"""Defectos de frontera encontrados en la prueba punta a punta (docs/PRUEBA_E2E.md).

* MPC con la alerta 9: solo sale en rojo el dron que falló; el 9 del dron sano al aterrizar es su fin de
  vuelo (antes los dos salían "DESHABILITADO (falla)" porque la alerta 9 es global).
* Sin historial (la estación se conectó después de la falla) se conserva la regla del contrato: con la
  alerta 9, cualquier 9 es falla.
* Un reinicio de Simulink olvida qué dron falló.
"""
import time

from app import protocol as P
from test_alertas_objetivos_sim import _station, _tel


def _mpc(ms, f1, f2, alerta=0, t=50.0):
    return _tel(t=t, version=2, mission_state=ms, alerta=alerta,
                drones={1: {"tracked": 1, "fsm_state": f1}, 2: {"tracked": 1, "fsm_state": f2}})


def test_fsm_info_con_historial_de_falla():
    # sin historial: regla del contrato (alerta 9 -> cualquier 9 es falla)
    assert P.fsm_info(2, 9, P.LANDING, 9)[2] is True
    # la estación sabe que ESTE dron no falló: su 9 al aterrizar es el fin de vuelo
    tipo, txt, falla = P.fsm_info(2, 9, P.LANDING, 9, falla_vista=False)
    assert not falla and "fin de vuelo" in txt
    # el que falló sigue en rojo aunque ya no esté "en vuelo"
    assert P.fsm_info(2, 9, P.DONE, 9, falla_vista=True)[2] is True
    assert P.fsm_info(2, 9, P.DONE, 0, falla_vista=True)[2] is True
    # 9 debiendo volar siempre es falla; 7/8 siempre
    assert P.fsm_info(2, 9, P.MISSION, 0, falla_vista=False)[2] is True
    assert P.fsm_info(2, 7, P.LANDING, 9, falla_vista=False)[2] is True


def test_estacion_marca_solo_al_dron_que_fallo(cfg_path):
    st = _station(cfg_path)

    class _Puente:
        n_sesion = 1

        def __init__(self):
            self.tel, self.stamp = None, 0.0

        @property
        def telemetry(self):
            return self.tel

        def latest(self):
            return self.tel, self.stamp

        def fresh(self):
            return True

        def status(self):
            return {}

    pu = _Puente()
    st.bridge = pu

    def paso(tel):
        st._on_telemetry(tel, time.time())
        pu.tel, pu.stamp = tel, time.time()
        return st.snapshot()["tel"]["drones"]

    paso(_mpc(P.MISSION, 5, 5, t=40.0))
    d = paso(_mpc(P.MISSION, 5, 7, alerta=9, t=45.0))              # falla a bordo del dron 2 (como la E2E)
    assert d[2]["fsm_falla"] and not d[1]["fsm_falla"]
    d = paso(_mpc(P.LANDING, 9, 9, alerta=9, t=52.0))             # los dos aterrizados, alerta 9 activa
    assert d[2]["fsm_falla"] and "(falla)" in d[2]["fsm_texto"]
    assert not d[1]["fsm_falla"] and "fin de vuelo" in d[1]["fsm_texto"]
    assert any("Dron 2: falla a bordo" in e["msg"] for e in st.events)
    # Simulink se reinicia (t vuelve a empezar): se olvida el historial
    d = paso(_mpc(P.LANDING, 9, 9, alerta=9, t=1.0))
    assert d[1]["fsm_falla"] and d[2]["fsm_falla"]                # sin historial: regla del contrato


def test_estacion_sin_historial_conserva_la_regla_del_contrato(cfg_path):
    st = _station(cfg_path)
    tel = _mpc(P.DONE, 9, 9, alerta=9)
    st._on_telemetry(tel, time.time())
    ui = st._tel_ui(tel)
    assert ui["drones"][1]["fsm_falla"] and ui["drones"][2]["fsm_falla"]


def test_pid_no_usa_el_historial(cfg_path):
    st = _station(cfg_path)
    tel = _tel(version=1, mission_state=P.LANDING, alerta=0,
               drones={1: {"tracked": 1, "fsm_state": 9}, 2: {"tracked": 1, "fsm_state": 3}})
    st._on_telemetry(tel, time.time())
    assert st._tel_ui(tel) is tel and st._falla_vista == {1: False, 2: False}


def test_aviso_si_la_version_del_modelo_no_es_la_de_la_config(cfg_path):
    """La E2E corrió tierra MPC con version_control = PID: Conexiones generaba quarc_run del binario PID."""
    st = _station(cfg_path)

    class _Puente:
        def __init__(self, tel):
            self.telemetry = tel

        def fresh(self):
            return True

        def status(self):
            return {}

    st.cfg.data["version_control"] = "PID"
    st.bridge = _Puente(_tel(version=2))
    assert any(a["nivel"] == "amarillo" and "version_control = PID" in a["msg"] for a in st.avisos())
    st.bridge = _Puente(_tel(version=1))
    assert not any("version_control" in a["msg"] for a in st.avisos())
