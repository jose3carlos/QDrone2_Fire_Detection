"""Registro de observaciones por vuelo (app/registro_vuelo.py) y ``geo.tubo_cercano``.

* ``tubo_cercano`` da exactamente la distancia de ``dist_tubos`` (y ``dist_polilinea`` no cambió);
* formato JSON Lines: cabecera, una línea por observación (conf >= confianza_min con punto en el piso, aceptadas y
  descartadas con su motivo y el tubo más cercano) y resumen con conteos, n bajo umbral y objetivos finales;
* apertura y cierre con el gate de la visión y el estado de la misión (una pausa no parte el vuelo);
* vuelos cortos sin observaciones no dejan archivo; con > 5 s de gate sí (cabecera + resumen);
* habilitación: auto (solo simulink), web.registro_vuelos y --registro-vuelos; pytest nunca escribe en web/datos;
* un error de disco no tumba la visión y las líneas se reintentan en orden.
"""
import glob
import json
import math
import os
import re
import time

import numpy as np
import pytest

from app import protocol as P
from app.geo import CameraModel, dist_polilinea, dist_tubos, tubo_cercano, tubos_pista
from app.mock_world import ColorDetector, anomalias_mock, render_view
from app.registro_vuelo import RegistroVuelos
from app.server import DATA_DIR, Station, build_parser
from app.vision_service import MOTIVO_TUBO, PoseHistory, VisionService
from conftest import PISTA_LAB
from test_alertas_objetivos_sim import _tel


# =============================================================================== geo.tubo_cercano
def _dist_polilinea_original(x, y, vertices, sin_tubo=None):
    """Copia literal de geo.dist_polilinea antes de este cambio (referencia)."""
    V = []
    for v in vertices or []:
        try:
            V.append((float(v[0]), float(v[1])))
        except (TypeError, ValueError, IndexError):
            continue
    if len(V) < 2:
        return None
    sin_tubo = sin_tubo or ()
    best = None
    for k, ((ax, ay), (bx, by)) in enumerate(zip(V[:-1], V[1:]), start=1):
        if k in sin_tubo:
            continue
        dx, dy = bx - ax, by - ay
        L = dx * dx + dy * dy
        t = 0.0 if L < 1e-12 else min(1.0, max(0.0, ((x - ax) * dx + (y - ay) * dy) / L))
        d = math.hypot(ax + t * dx - x, ay + t * dy - y)
        best = d if best is None or d < best else best
    return best


def _dist_tubos_original(x, y, tubos):
    best = None
    for P_ in tubos or []:
        d = _dist_polilinea_original(x, y, P_)
        if d is not None and (best is None or d < best):
            best = d
    return best


def test_tubo_cercano_da_la_misma_distancia_que_dist_tubos():
    rng = np.random.default_rng(3)
    juegos = [PISTA_LAB["tubos"], tubos_pista({"vertices": PISTA_LAB["vertices"], "tramos_sin_tubo": [4]}),
              [[[0, 0], [0, 0], [1, 0]]], [[[0.5, 0.5]]], []]
    for tubos in juegos:
        for x, y in rng.uniform(-2.0, 2.0, size=(1500, 2)):
            ref = _dist_tubos_original(x, y, tubos)
            assert dist_tubos(x, y, tubos) == ref                    # bit a bit
            c = tubo_cercano(x, y, tubos)
            if ref is None:
                assert c is None
                continue
            assert c["dist"] == ref
            assert math.hypot(c["punto"][0] - x, c["punto"][1] - y) == pytest.approx(ref, abs=1e-12)
            P_ = tubos[c["pieza"]]
            a, b = P_[c["tramo"]], P_[c["tramo"] + 1]
            # desplazamiento = distancia con signo a la RECTA del tramo: igual a la distancia dentro del tramo
            if 0.0 < c["u"] < 1.0:
                assert abs(c["desplazamiento"]) == pytest.approx(ref, abs=1e-9)
            assert 0.0 <= c["t"] <= 1.0 and (c["t"] == c["u"] or not 0.0 <= c["u"] <= 1.0)
            assert c["punto"] == pytest.approx([a[0] + c["t"] * (b[0] - a[0]), a[1] + c["t"] * (b[1] - a[1])])
    # dist_polilinea (con huecos) sigue igual
    for x, y in rng.uniform(-2.0, 2.0, size=(500, 2)):
        assert dist_polilinea(x, y, PISTA_LAB["vertices"], {4}) == _dist_polilinea_original(x, y, PISTA_LAB["vertices"], {4})


def test_tubo_cercano_signo_pieza_y_tramo():
    tubos = [[[0, 0], [1, 0], [1, 1]], [[-1, -1], [-1, -2]]]
    c = tubo_cercano(0.5, 0.2, tubos)                 # a la izquierda de (0,0)->(1,0)
    assert (c["pieza"], c["tramo"]) == (0, 0) and c["desplazamiento"] == pytest.approx(0.2) and c["u"] == pytest.approx(0.5)
    c = tubo_cercano(1.3, 0.5, tubos)                 # a la derecha de (1,0)->(1,1) (sentido +y)
    assert (c["pieza"], c["tramo"]) == (0, 1) and c["desplazamiento"] == pytest.approx(-0.3)
    c = tubo_cercano(-1.0, -2.4, tubos)               # más allá del final de la pieza 2
    assert (c["pieza"], c["tramo"]) == (1, 0) and c["u"] == pytest.approx(1.4) and c["t"] == 1.0
    assert c["dist"] == pytest.approx(0.4) and c["punto"] == pytest.approx([-1.0, -2.0])


# =============================================================================== RegistroVuelos directo
class _Reloj:
    def __init__(self, t=1_700_000_000.0):
        self.t = t

    def __call__(self):
        return self.t


def _leer(ruta):
    with open(ruta, encoding="utf-8") as fh:
        return [json.loads(l) for l in fh]


def _obs(cls=1, xy=(0.5, -1.0), motivo=None, conf=0.95):
    return {"frame": 1, "stamp": 0.0, "cls": cls, "conf": conf, "xy": list(xy), "motivo": motivo, "dist_tubo_m": 0.1,
            "tubo": None, "tam_m": 0.1, "pose": [0, 0, 1.5, 0], "roll_deg": None, "pitch_deg": None,
            "inclinacion_deg": None, "box": [1, 2, 3, 4]}


def _registro(tmp_path, reloj, **kw):
    logs = []
    reg = RegistroVuelos(tmp_path / "vuelos", lambda: "simulink", lambda: True, log_fn=logs.append,
                         resumen_fn=lambda: {"objetivos": [{"id": 1, "cls": 1, "x": 0.5, "y": -1.0}]},
                         reloj=reloj, hilo=False, **kw)
    reg.logs = logs
    return reg


def test_formato_cabecera_obs_y_resumen(tmp_path):
    r = _Reloj()
    reg = _registro(tmp_path, r)
    reg.tick(True, P.MISSION, lambda: {"pista": {"tubos": PISTA_LAB["tubos"]}, "confianza_min": 0.9})
    assert reg.abierto() and reg.estado()["pendiente"] and reg.estado()["archivo"] is None
    r.t += 0.5
    reg.observaciones(r.t, [_obs(1, motivo=MOTIVO_TUBO), _obs(0, xy=(0.1, 1.2))], n_bajo=3)
    r.t += 0.5
    reg.observaciones(r.t, [_obs(1)], n_bajo=1)
    est = reg.estado()
    assert est["archivo"] and est["lineas"] == 4 and est["obs"] == 3
    r.t += 9.0
    reg.tick(True, P.LANDING)
    assert not reg.abierto()
    assert reg.vaciar()
    archivos = glob.glob(str(tmp_path / "vuelos" / "*" / "*.jsonl"))
    assert len(archivos) == 1
    lt = time.localtime(1_700_000_000.0)
    assert re.search(r"[\\/]%s[\\/]vuelo_%s_simulink\.jsonl$" % (time.strftime("%Y-%m-%d", lt), time.strftime("%H%M%S", lt)),
                     archivos[0])
    L = _leer(archivos[0])
    assert [l["tipo"] for l in L] == ["cabecera", "obs", "obs", "obs", "resumen"]
    cab = L[0]
    assert cab["formato"] == 1 and cab["fuente"] == "simulink" and cab["confianza_min"] == 0.9
    assert cab["pista"]["tubos"] == PISTA_LAB["tubos"] and cab["fecha"] == time.strftime("%Y-%m-%dT%H:%M:%S", lt)
    assert [l["t"] for l in L[1:4]] == [0.5, 0.5, 1.0]
    assert L[1]["motivo"] == MOTIVO_TUBO and L[2]["motivo"] is None and L[1]["box"] == [1, 2, 3, 4]
    res = L[-1]
    assert res["conteos"] == {"toma": {MOTIVO_TUBO: 1, "aceptada": 1}, "fuego": {"aceptada": 1}}
    assert res["n_obs"] == 3 and res["n_bajo_umbral"] == 4 and res["duracion_s"] == 10.0
    assert res["objetivos"] == [{"id": 1, "cls": 1, "x": 0.5, "y": -1.0}] and res["lineas"] == 5
    assert res["motivo_cierre"].startswith("misión en LANDING")
    assert reg.estado()["ultimo"].endswith(".jsonl") and any("cerrado" in m for m in reg.logs)


def test_vuelo_sin_observaciones_solo_si_dura_mas_de_5_s(tmp_path):
    r = _Reloj()
    reg = _registro(tmp_path, r)
    reg.tick(True, P.MISSION)
    for _ in range(8):                       # 4 s de gate activo
        r.t += 0.5
        reg.tick(True, P.MISSION)
    reg.tick(True, P.DONE)
    reg.vaciar()
    assert glob.glob(str(tmp_path / "vuelos" / "**" / "*.jsonl"), recursive=True) == []
    # gate apagado (p. ej. explorador bajo) no cuenta como tiempo de vuelo
    reg.tick(True, P.MISSION)
    for _ in range(20):
        r.t += 0.5
        reg.tick(False, P.MISSION)
    reg.tick(True, P.DONE)
    reg.vaciar()
    assert glob.glob(str(tmp_path / "vuelos" / "**" / "*.jsonl"), recursive=True) == []
    # > 5 s de gate: cabecera + resumen
    r.t += 100
    reg.tick(True, P.MISSION)
    for _ in range(12):
        r.t += 0.5
        reg.tick(True, P.PAUSED)
    reg.tick(True, P.RETURN_HOME)
    reg.vaciar()
    (f,) = glob.glob(str(tmp_path / "vuelos" / "**" / "*.jsonl"), recursive=True)
    L = _leer(f)
    assert [l["tipo"] for l in L] == ["cabecera", "resumen"] and L[1]["duracion_gate_s"] == pytest.approx(6.0)


def test_flush_periodico_y_por_cantidad(tmp_path):
    r = _Reloj()
    reg = _registro(tmp_path, r, flush_s=1.0, flush_n=50)
    reg.tick(True, P.MISSION)
    reg.observaciones(r.t, [_obs()])
    assert reg.vaciar(forzar=False) and not glob.glob(str(tmp_path / "vuelos" / "*" / "*.jsonl"))   # < 1 s, < 50
    r.t += 1.1
    reg.vaciar(forzar=False)
    (f,) = glob.glob(str(tmp_path / "vuelos" / "*" / "*.jsonl"))
    assert len(_leer(f)) == 2
    reg.observaciones(r.t, [_obs()] * 60)
    reg.vaciar(forzar=False)                                          # >= 50 líneas: sin esperar
    assert len(_leer(f)) == 62
    assert reg.estado()["en_cola"] == 0


def test_error_de_escritura_no_tumba_nada_y_se_reintenta(tmp_path):
    r = _Reloj()
    bloqueo = tmp_path / "vuelos"
    bloqueo.write_text("soy un archivo, no una carpeta")             # makedirs va a fallar
    reg = _registro(tmp_path, r)
    reg.tick(True, P.MISSION, lambda: {"x": 1})
    reg.observaciones(r.t, [_obs(), _obs(0)])
    assert reg.vaciar() is False
    est = reg.estado()
    assert est["error"] and est["errores"] == 1 and est["en_cola"] == 3
    assert any("no se pudo escribir" in m for m in reg.logs)
    reg.observaciones(r.t + 1, [_obs()])
    reg.tick(True, P.ABORT)
    assert reg.vaciar() is False and reg.estado()["errores"] == 2
    bloqueo.unlink()
    assert reg.vaciar() is True and reg.estado()["error"] == ""
    (f,) = glob.glob(str(tmp_path / "vuelos" / "*" / "*.jsonl"))
    assert [l["tipo"] for l in _leer(f)] == ["cabecera", "obs", "obs", "obs", "resumen"]


class _ArchivoMitad:
    """Archivo que escribe la mitad de lo que recibe y lanza OSError 28 (disco lleno)."""

    def __init__(self, fh):
        self.fh = fh

    def write(self, datos):
        self.fh.write(datos[: len(datos) // 2])
        self.fh.flush()
        raise OSError(28, "No space left on device")

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.fh.close()


@pytest.mark.parametrize("recorte_falla", [False, True])
def test_escritura_a_medias_no_duplica_ni_pega_lineas(tmp_path, monkeypatch, recorte_falla):
    import builtins
    import app.registro_vuelo as RV
    real = builtins.open
    fallas = {"ab": 1}

    def falso_open(ruta, modo="r", *a, **kw):
        if modo == "ab" and fallas["ab"] > 0:
            fallas["ab"] -= 1
            return _ArchivoMitad(real(ruta, modo, *a, **kw))
        if modo == "r+b" and recorte_falla:
            raise PermissionError(13, "bloqueado")
        return real(ruta, modo, *a, **kw)

    monkeypatch.setattr(RV, "open", falso_open, raising=False)
    r = _Reloj()
    reg = _registro(tmp_path, r)
    reg.tick(True, P.MISSION)
    reg.observaciones(r.t, [_obs(xy=(0.1 * k, -1.0)) for k in range(1, 9)])      # 8 obs. reales
    assert reg.vaciar() is False and 0 < reg.estado()["en_cola"] < 9                # lo ya escrito no se re-encola
    assert reg.vaciar() is True
    (f,) = glob.glob(str(tmp_path / "vuelos" / "*" / "*.jsonl"))
    buenas, rotas = [], 0
    with open(f, encoding="utf-8") as fh:
        for l in fh:
            try:
                buenas.append(json.loads(l))
            except ValueError:
                rotas += 1
    xs = [o["xy"][0] for o in buenas if o["tipo"] == "obs"]
    assert [l["tipo"] for l in buenas][0] == "cabecera"
    assert xs == pytest.approx([0.1 * k for k in range(1, 9)])                     # cada una exactamente una vez
    assert rotas == (1 if recorte_falla else 0)                                     # sin recorte: 1 línea rota suelta


def test_error_intermitente_no_llena_los_eventos(tmp_path, monkeypatch):
    import builtins
    import app.registro_vuelo as RV
    real = builtins.open
    llamadas = {"n": 0}

    def falso_open(ruta, modo="r", *a, **kw):        # OneDrive bloqueando el archivo una vez sí y otra no
        if modo == "ab":
            llamadas["n"] += 1
            if llamadas["n"] % 2 == 1:
                raise PermissionError(13, "Permission denied", str(ruta))
        return real(ruta, modo, *a, **kw)

    monkeypatch.setattr(RV, "open", falso_open, raising=False)
    r = _Reloj()
    reg = _registro(tmp_path, r)
    reg.tick(True, P.MISSION)
    for k in range(8):                               # 8 ciclos de 1 s
        reg.observaciones(r.t, [_obs(xy=(0.1 * k, -1.0))])
        reg.vaciar()
        r.t += 1.0
    avisos = [m for m in reg.logs if m.startswith("Registro de vuelo:")]
    assert len(avisos) == 2 and "no se pudo escribir" in avisos[0] and "volvió a funcionar" in avisos[1]
    assert reg.estado()["errores"] == 4
    r.t += 60.0                                      # pasado un minuto se vuelve a avisar
    for k in range(2):
        reg.observaciones(r.t, [_obs()])
        reg.vaciar()
        r.t += 1.0
    avisos = [m for m in reg.logs if m.startswith("Registro de vuelo:")]
    assert len(avisos) == 4
    # un error persistente se avisa una sola vez
    monkeypatch.setattr(RV, "open", lambda *a, **kw: (_ for _ in ()).throw(PermissionError(13, "fijo")), raising=False)
    for k in range(5):
        reg.observaciones(r.t, [_obs()])
        reg.vaciar()
        r.t += 1.0
    assert len([m for m in reg.logs if m.startswith("Registro de vuelo:")]) == 5


def test_un_registro_que_falla_no_tumba_la_vision(tmp_path, cfg_path_lab):
    with open(cfg_path_lab, encoding="utf-8") as fh:
        cfg = json.load(fh)
    cfg["vision"]["confianza_min"] = 0.5

    class _Cam:
        def wait_newer(self, *a):
            return None

    vs = VisionService(_Cam(), PoseHistory(), lambda: True, lambda t: None, lambda: cfg, ColorDetector(), "falso")

    def roto(*a):
        raise OSError("disco lleno")
    vs.obs_fn = roto
    cam = CameraModel.from_config(cfg["camara"], cfg, image_size=(640, 480))
    toma = anomalias_mock(cfg)[2]
    pose = (toma["x"] - 0.2, toma["y"], 1.5, 0.0)
    img = render_view(cam, pose, PISTA_LAB["vertices"], [toma], rng=np.random.default_rng(1), tubos=PISTA_LAB["tubos"])
    dets = vs.process(img, 10.0, pose=pose)
    assert any(d["cls"] == 1 and d["descartada"] is None for d in dets)
    assert "disco lleno" in vs.status()["registro_error"] and vs.n_obs == 1


# =============================================================================== estación: gate y estado de la misión
class _Puente:
    def __init__(self):
        self.telemetry = None
        self.n_sesion = 0

    def fresh(self):
        return self.telemetry is not None

    def stop(self):
        pass

    def status(self):
        return {"n_targets_rechazados": 0}

    def latest(self):
        return self.telemetry, time.time()

    def set_targets(self, targets):
        self.targets = targets


def _tel_explorador(estado, x, y, z, yaw=0.0):
    return _tel(mission_state=estado, explorer_id=1, n_targets_known=0, dron_solo=1,
                drones={1: {"tracked": 1, "fsm_state": 2, "x": x, "y": y, "z": z, "yaw": yaw}, 2: {"tracked": 0}})


def _paso(st, estado, pose):
    tel = _tel_explorador(estado, *pose)
    st.bridge.telemetry = tel
    st._on_telemetry(tel, time.time())
    return tel


def test_estacion_abre_y_cierra_con_el_gate_y_registra_las_rechazadas(cfg_path_lab):
    st = Station(cfg_path_lab, "simulink")              # auto: la fuente simulink (vuelo real) se registra
    assert st.registro_modo()[0] and st.registro.carpeta == os.path.join(os.path.dirname(cfg_path_lab), "vuelos")
    st.registro._usar_hilo = False                     # escritura a mano: prueba determinista
    st.bridge = _Puente()
    st.vision.detector = ColorDetector()
    st.cfg.data["vision"]["confianza_min"] = 0.5
    # el tubo 2 de la config está 0.35 m más al norte que el real: la toma real queda "lejos del tubo"
    reales = PISTA_LAB["tubos"]
    st.cfg.data["pista"]["tubos"] = [reales[0], [[p[0], p[1] + 0.35] for p in reales[1]]]
    toma = anomalias_mock({"pista": PISTA_LAB})[2]      # sobre el tubo 2 real
    pose = (toma["x"] - 0.2, toma["y"], 1.5, 0.0)
    cam = CameraModel.from_config(st.cfg.data["camara"], st.cfg.data, image_size=(640, 480))
    img = render_view(cam, pose, PISTA_LAB["vertices"], [toma], rng=np.random.default_rng(4), tubos=reales)

    _paso(st, P.HOVER_READY, pose)
    assert not st.registro.abierto()                   # sin misión: no hay vuelo
    _paso(st, P.MISSION, pose)
    assert st.registro.abierto()
    for k in range(3):
        st.vision.process(img, time.time() + 0.1 * k, pose=pose)
    est = st.snapshot()["registro_vuelo"]
    assert est["archivo"] and est["lineas"] == 4 and est["obs"] == 3 and est["modo"].startswith("auto")
    ruta = est["archivo"]
    _paso(st, P.MISSION, (pose[0], pose[1], 0.2, 0.0))  # explorador bajo: gate apagado, el vuelo sigue
    assert st.registro.abierto() and not st._vision_gate()
    _paso(st, P.PAUSED, pose)                          # pausa: el mismo vuelo
    assert st.registro.estado()["archivo"] == ruta
    st.vision.process(img, time.time(), pose=pose)
    _paso(st, P.LANDING, pose)
    assert not st.registro.abierto()
    st.registro.vaciar()
    assert os.path.isabs(ruta)                         # fuera del repo (tmp): ruta absoluta
    L = _leer(ruta)
    assert [l["tipo"] for l in L] == ["cabecera"] + ["obs"] * 4 + ["resumen"]
    cab = L[0]
    assert cab["fuente"] == "simulink" and cab["explorador"] == 1 and cab["modo_mision"] == "solo_explorador"
    assert cab["pista"]["vertices"] == PISTA_LAB["vertices"] and cab["pista"]["tramos_sin_tubo"] == [4]
    assert cab["pista"]["tubos"][1][0] == pytest.approx([0.47, 0.10]) and cab["pista"]["tubos_de"] == "pista.tubos"
    assert cab["filtros"]["dist_max_tubo_toma_m"] == 0.25 and cab["confianza_min"] == 0.5
    assert cab["camara"]["offset_camara_m"] == [0.2, 0.0, -0.05] and cab["camara"]["modelo"]["fx"] > 0
    o = L[1]
    assert o["cls"] == 1 and o["motivo"] == MOTIVO_TUBO and o["conf"] >= 0.5
    assert math.hypot(o["xy"][0] - toma["x"], o["xy"][1] - toma["y"]) < 0.05
    assert o["tubo"]["pieza"] == 1 and 0.25 < o["dist_tubo_m"] == pytest.approx(0.29, abs=0.03)
    assert o["pose"][:3] == pytest.approx(list(pose[:3])) and o["roll_deg"] is None and o["tam_m"] > 0
    res = L[-1]
    assert res["conteos"] == {"toma": {MOTIVO_TUBO: 4}} and res["n_obs"] == 4
    assert res["objetivos"] == [] and res["motivo_cierre"] == "misión en LANDING"
    # un vuelo nuevo empieza otro archivo
    _paso(st, P.MISSION, pose)
    st.vision.process(img, time.time(), pose=pose)
    assert st.registro.estado()["archivo"] != ruta
    st.stop()                                          # cierra y escribe
    assert len(glob.glob(os.path.join(os.path.dirname(cfg_path_lab), "vuelos", "*", "*.jsonl"))) == 2


def test_reiniciar_objetivos_cierra_el_vuelo(cfg_path_lab):
    from fastapi.testclient import TestClient
    from app.server import create_app
    st = Station(cfg_path_lab, "simulink")
    st.registro._usar_hilo = False
    st.bridge = _Puente()
    pose = (0.5, -1.0, 1.5, 0.0)
    _paso(st, P.MISSION, pose)
    st.registro.observaciones(time.time(), [_obs()])
    c = TestClient(create_app(st, start=False))
    assert c.post("/api/objetivos/reiniciar").status_code == 200
    assert not st.registro.abierto()
    assert st.registro.estado()["ultimo"].endswith("_simulink.jsonl")


# =============================================================================== habilitación
def test_por_defecto_solo_simulink_y_opciones(cfg_path, tmp_path):
    from test_revision import _patch_cfg
    assert Station(cfg_path, "mock").registro_modo()[0] is False
    assert Station(cfg_path, "sim3d").registro_modo()[0] is False
    assert Station(cfg_path, "simulink").registro_modo()[0] is True
    assert Station(cfg_path, "mock", registro_vuelos="si").registro_modo()[0] is True
    assert Station(cfg_path, "simulink", registro_vuelos="no").registro_modo()[0] is False
    _patch_cfg(cfg_path, lambda d: d.setdefault("web", {}).update(registro_vuelos=True))
    assert Station(cfg_path, "mock").registro_modo()[0] is True
    assert Station(cfg_path, "mock", registro_vuelos="auto").registro_modo()[0] is False   # la CLI pisa la config
    _patch_cfg(cfg_path, lambda d: d["web"].update(registro_vuelos=False))
    assert Station(cfg_path, "simulink").registro_modo()[0] is False
    with pytest.raises(ValueError):
        Station(cfg_path, "mock", registro_vuelos="quizas")
    assert build_parser().parse_args(["--registro-vuelos", "si"]).registro_vuelos == "si"
    with pytest.raises(SystemExit):
        build_parser().parse_args(["--registro-vuelos", "tal-vez"])
    # sin registro habilitado no se abre nada
    st = Station(cfg_path, "mock")
    st.bridge = _Puente()
    _paso(st, P.MISSION, (0.0, 0.0, 1.5))
    assert not st.registro.abierto() and st.snapshot()["registro_vuelo"]["habilitado"] is False


def test_pytest_nunca_escribe_en_web_datos():
    from app.config import DEFAULT_PATH
    carpeta = os.path.join(DATA_DIR, "vuelos")
    antes = sorted(glob.glob(os.path.join(carpeta, "**", "*"), recursive=True))
    existia = os.path.isdir(carpeta)
    st = Station(DEFAULT_PATH, "mock", registro_vuelos="si")      # config REAL (no se escribe) y registro forzado
    assert os.path.normcase(st.registro.carpeta) == os.path.normcase(carpeta)
    hab, texto = st.registro_modo()
    assert hab is False and "pytest" in texto
    st.registro._usar_hilo = False
    st.bridge = _Puente()
    _paso(st, P.MISSION, (0.0, 0.0, 1.5))
    st.registro.observaciones(time.time(), [_obs()])
    _paso(st, P.LANDING, (0.0, 0.0, 1.5))
    st.registro.vaciar()
    assert sorted(glob.glob(os.path.join(carpeta, "**", "*"), recursive=True)) == antes
    assert os.path.isdir(carpeta) == existia
