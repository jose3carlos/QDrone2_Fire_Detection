"""Caja negra del vuelo (app/caja_negra.py), repetición (app/repeticion.py, --repetir) y herramienta
web/herramientas/caja_negra.py.

* formato y ciclo: búfer previo de pre_s s, abre con START o con un estado de vuelo, cierra post_s s después de
  salir de ellos; vector crudo tal cual (NaN/inf), comandos, acks, eventos, avisos y muestras; decimación;
* robustez: archivo cortado (corte de luz) o con basura legible; errores de disco no tumban nada y no duplican;
* estación: grabación con el mock forzado (--caja-negra si) de punta a punta; el registro usa el mismo nombre;
  habilitación (auto = solo simulink), CLI y pytest nunca escribe en web/datos;
* repetición: reproduce por el puente los mismos paquetes (estados y posiciones), solo lectura (409), control;
* herramienta: resumen (estados, comandos con latencia, rastreo, máximos, separación, aire, huecos) y csv.
"""
import csv
import glob
import gzip
import importlib.util
import json
import math
import os
import time

import pytest

from app import protocol as P
from app.caja_negra import CajaNegra, empacar, nombres_y, vector
from app.registro_vuelo import RegistroVuelos
from app.repeticion import Grabacion, leer_lineas
from app.server import DATA_DIR, Station, build_parser, create_app

WEB = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_spec = importlib.util.spec_from_file_location("herr_caja_negra", os.path.join(WEB, "herramientas", "caja_negra.py"))
herr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(herr)


class _Reloj:
    def __init__(self, t=1_700_000_000.0):
        self.t = t

    def __call__(self):
        return self.t


def _paquete(t, estado, d1=(-0.9, 0.4, 0.0), d2=(0.9, -0.4, 0.0), tr=(1, 1), ack=0, alerta=0, fsm=(2, 2), ds=0):
    drones = {}
    for d, pos, ok, f in ((1, d1, tr[0], fsm[0]), (2, d2, tr[1], fsm[1])):
        drones[d] = {"tracked": ok, "fsm_state": f, "x": pos[0] if ok else None, "y": pos[1] if ok else None,
                     "z": pos[2] if ok else None, "yaw": 0.0 if ok else None, "battery_V": None if d == 2 else 11.7,
                     "role": d, "comm_ok": 1}
    return P.pack_telemetry({"t": t, "mission_state": estado, "version": 1, "explorer_id": 1 if not ds else ds,
                             "responder_id": 0 if ds else 2, "cmd_seq_ack": ack, "alerta": alerta, "dron_solo": ds,
                             "explorer_wp_total": 36, "drones": drones})


def _caja(tmp_path, reloj, avisos=None, **kw):
    logs = []
    av = avisos if avisos is not None else []
    caja = CajaNegra(tmp_path / "vuelos", lambda: "simulink", lambda: True, log_fn=logs.append,
                     cabecera_fn=lambda: {"modo_mision": "dual", "config": "config/estacion.json"},
                     muestra_fn=lambda: {"camara": {"fps": 11.2, "edad_s": 0.09}, "avisos": list(av),
                                         "objetivos": [{"id": 1, "cls": 1, "x": 0.5, "y": -1.0, "conf": 0.9}]},
                     reloj=reloj, hilo=False, **kw)
    caja.logs = logs
    return caja


def _alimentar(caja, r, paquetes, dt=0.02, muestras=True):
    """Manda los paquetes con dt entre ellos (y una muestra por segundo, como el hilo de la caja)."""
    for raw in paquetes:
        r.t += dt
        caja.telemetria(raw, r.t)
        if muestras and r.t - getattr(caja, "_t_m", -1e9) >= 1.0:
            caja._t_m = r.t
            caja.muestrear()


def _lineas(ruta):
    L, info = leer_lineas(ruta)
    assert info["danadas"] == 0 and not info["truncado"] and info["basura_bytes"] == 0
    return L


def _vuelo_simple(tmp_path, r, **kw):
    """15 s en ASSIGN_ROLES, START, 2 s de TAKEOFF, 3 s de MISSION subiendo en x, 12 s en DONE (cierra)."""
    avisos = [{"nivel": "amarillo", "msg": "aviso viejo"}]
    caja = _caja(tmp_path, r, avisos=avisos, **kw)
    _alimentar(caja, r, [_paquete(k * 0.02, P.ASSIGN_ROLES) for k in range(750)])
    assert not caja.abierto() and 9.9 <= caja.estado()["buffer_s"] <= 10.0
    caja.evento("Comando START enviado (cmd_seq 1)")
    caja.comando("START", "web 127.0.0.1", "enviado", 1)
    assert caja.abierto()
    _alimentar(caja, r, [_paquete(15 + k * 0.02, P.TAKEOFF, ack=1, fsm=(2, 2)) for k in range(100)])
    caja.ack({"cmd": "START", "cmd_seq": 1, "estado": "recibido", "detalle": "Simulink devolvió ack 1",
              "t": r.t - 1.9, "t_click": r.t - 2.0})
    avisos.append({"nivel": "rojo", "msg": "Sin video del explorador"})
    _alimentar(caja, r, [_paquete(17 + k * 0.02, P.MISSION, d1=(-0.9 + k * 0.01, 0.4, 1.0), ack=1)
                         for k in range(150)])
    avisos.pop()
    _alimentar(caja, r, [_paquete(20 + k * 0.02, P.DONE, ack=1, fsm=(9, 9)) for k in range(600)])
    return caja


# =============================================================================== formato y ciclo
def test_formato_buffer_previo_y_cierre(tmp_path):
    r = _Reloj()
    caja = _vuelo_simple(tmp_path, r)
    assert not caja.abierto()                       # 10 s después de salir de los estados de vuelo
    assert caja.vaciar()
    (f,) = glob.glob(str(tmp_path / "vuelos" / "*" / "*.tel.jsonl.gz"))
    L = _lineas(f)
    cab = L[0]
    lt = time.localtime(caja._t0[f])                # hora del START (15 s después del primer paquete)
    assert cab["t0"] == pytest.approx(1_700_000_000.0 + 15.0, abs=0.03)
    assert os.path.basename(f) == f"vuelo_{time.strftime('%H%M%S', lt)}_simulink.tel.jsonl.gz"
    assert cab["tipo"] == "cabecera" and cab["formato"] == 1 and cab["fuente"] == "simulink"
    assert cab["motivo_apertura"] == "START" and cab["pre_s"] == 10.0 and cab["modo_mision"] == "dual"
    assert cab["campos"]["nombres"] == nombres_y() and len(nombres_y()) == 64
    assert nombres_y()[1] == "mission_state" and nombres_y()[14 + 3 - 1] == "d1_x" and nombres_y()[63] == "dron_solo"
    tel = [l for l in L if l["tipo"] == "tel"]
    ts = [l["t"] for l in tel]
    assert ts == sorted(ts) and -10.0 <= ts[0] < -9.9   # búfer previo de 10 s
    assert all(len(l["y"]) == 64 for l in tel)
    # el vector es el crudo, exacto: NaN (sin batería del dron 2) -> null, y vuelve a los mismos bytes
    raw = _paquete(17 + 0.02, P.MISSION, d1=(-0.89, 0.4, 1.0), ack=1)
    assert empacar(vector(raw)) == raw and vector(raw)[32 + 14 - 1] is None
    assert vector(b"".join([struct_inf()] + [b"\0" * 8] * 63))[0] == "Inf" and empacar(["-Inf"] + [0] * 63)[:8] == \
        b"\x00\x00\x00\x00\x00\x00\xf0\xff"
    mision = [l for l in tel if l["y"][1] == P.MISSION]
    assert len(mision) == 150 and mision[1]["y"][14 + 3 - 1] == pytest.approx(-0.89)
    tipos = [l["tipo"] for l in L]
    assert tipos.count("cmd") == 1 and tipos.count("ack") == 1 and tipos[-1] == "fin"
    cmd = next(l for l in L if l["tipo"] == "cmd")
    assert cmd == {"tipo": "cmd", "t": 0.0, "cmd": "START", "origen": "web 127.0.0.1", "resultado": "enviado",
                   "cmd_seq": 1, "detalle": ""}
    assert next(l for l in L if l["tipo"] == "ack")["latencia_s"] == pytest.approx(0.1)
    assert any(l["tipo"] == "evento" and "START" in l["msg"] for l in L)
    av = [(l["accion"], l["msg"]) for l in L if l["tipo"] == "aviso"]
    assert ("aparece", "Sin video del explorador") in av and ("desaparece", "Sin video del explorador") in av
    assert ("activo", "aviso viejo") in av         # apareció antes del búfer: se anota al abrir
    muestras = [l for l in L if l["tipo"] == "muestra"]
    assert len(muestras) >= 20 and muestras[0]["camara"]["fps"] == 11.2 and muestras[0]["objetivos"][0]["id"] == 1
    fin = L[-1]
    assert fin["motivo"] == "misión en DONE" and fin["n_tel"] == len(tel) and fin["lineas"] == len(L)
    assert caja.estado()["ultimo"].endswith(".tel.jsonl.gz") and any("Caja negra cerrada" in m for m in caja.logs)


def struct_inf():
    import struct
    return struct.pack("<d", math.inf)


def test_abre_por_estado_de_vuelo_y_start_sin_despegue_cierra(tmp_path):
    r = _Reloj()
    caja = _caja(tmp_path, r)
    _alimentar(caja, r, [_paquete(k * 0.02, P.HOVER_READY) for k in range(10)])   # sin START visto (otra estación)
    assert caja.abierto() and caja.archivo_actual().endswith(".tel.jsonl.gz")
    caja.cerrar("prueba")
    # START que Simulink no aceptó (alerta 1): se graba y se cierra post_s s después
    _alimentar(caja, r, [_paquete(1 + k * 0.02, P.ASSIGN_ROLES) for k in range(100)])
    caja.comando("START", "web", "enviado", 2)
    _alimentar(caja, r, [_paquete(3 + k * 0.02, P.ASSIGN_ROLES, alerta=1) for k in range(520)])
    assert not caja.abierto()
    caja.vaciar()
    rutas = sorted(glob.glob(str(tmp_path / "vuelos" / "*" / "*.tel.jsonl.gz")))
    assert len(rutas) == 2
    motivos = {_lineas(x)[-1]["motivo"] for x in rutas}
    assert motivos == {"prueba", "misión en ASSIGN_ROLES (sin despegar)"}
    # un 409 no abre nada
    caja.comando("START", "web", "409", detalle="sin telemetría")
    assert not caja.abierto()


def test_decimacion_y_deshabilitada(tmp_path):
    r = _Reloj()
    caja = _vuelo_simple(tmp_path, r, decimacion=2)
    caja.vaciar()
    (f,) = glob.glob(str(tmp_path / "vuelos" / "*" / "*.tel.jsonl.gz"))
    L = _lineas(f)
    assert L[0]["decimacion"] == 2
    assert len([l for l in L if l["tipo"] == "tel" and l["y"][1] == P.MISSION]) == 75
    # deshabilitada: ni búfer ni archivo
    r2 = _Reloj()
    nada = CajaNegra(tmp_path / "otra", lambda: "mock", lambda: False, reloj=r2, hilo=False)
    _alimentar(nada, r2, [_paquete(k * 0.02, P.MISSION) for k in range(100)], muestras=False)
    nada.comando("START", "web", "enviado", 1)
    nada.evento("x")
    assert not nada.abierto() and nada.estado()["buffer_s"] == 0.0 and not os.path.exists(tmp_path / "otra")


# =============================================================================== robustez
def test_archivo_cortado_y_con_basura_se_lee(tmp_path):
    r = _Reloj()
    caja = _caja(tmp_path, r)
    caja.comando("START", "web", "enviado", 1)
    _alimentar(caja, r, [_paquete(k * 0.02, P.TAKEOFF) for k in range(100)])
    caja.vaciar()                                   # miembro 1
    _alimentar(caja, r, [_paquete(2 + k * 0.02, P.MISSION) for k in range(100)])
    caja.vaciar()                                   # miembro 2 (el vuelo sigue abierto: sin línea "fin")
    (f,) = glob.glob(str(tmp_path / "vuelos" / "*" / "*.tel.jsonl.gz"))
    completo, info = leer_lineas(f)
    assert info["miembros"] == 2 and not info["truncado"]
    n_tel = len([l for l in completo if l["tipo"] == "tel"])
    datos = open(f, "rb").read()
    # corte de luz a la mitad del último miembro
    cortado = tmp_path / "cortado.tel.jsonl.gz"
    cortado.write_bytes(datos[:-300])
    L, info = leer_lineas(cortado)
    assert info["truncado"] and info["miembros"] == 2
    tel = [l for l in L if l["tipo"] == "tel"]
    assert 100 < len(tel) < n_tel and L[0]["tipo"] == "cabecera"
    g = Grabacion(str(cortado))
    assert g.fin is None and len(g.tel) == len(tel)
    res = herr.analizar(g)
    assert res["cortado"] and res["n_tel"] == len(tel)
    # gzip estándar también lo lee hasta el corte (avisando al final)
    with pytest.raises(EOFError):
        with gzip.open(cortado) as fh:
            fh.read()
    # basura entre miembros (una escritura fallida que no se pudo recortar): se salta
    p1 = datos.find(b"\x1f\x8b\x08", 10)          # inicio del miembro 2
    con_basura = tmp_path / "basura.tel.jsonl.gz"
    con_basura.write_bytes(datos[:p1] + b"\x1f\x8b\x08 basura de un vaciado a medias" + datos[p1:])
    L, info = leer_lineas(con_basura)
    assert info["basura_bytes"] > 0 and len([l for l in L if l["tipo"] == "tel"]) == n_tel


def test_errores_de_disco_no_tumban_ni_duplican(tmp_path, monkeypatch):
    from app import caja_negra as cn
    r = _Reloj()
    bloqueo = tmp_path / "vuelos"
    bloqueo.write_text("soy un archivo, no una carpeta")       # makedirs falla
    caja = _caja(tmp_path, r)
    caja.comando("START", "web", "enviado", 1)
    _alimentar(caja, r, [_paquete(k * 0.02, P.TAKEOFF) for k in range(50)])
    assert caja.vaciar() is False
    est = caja.estado()
    assert est["error"] and est["errores"] == 1 and est["en_cola"] > 50 and est["abierto"]
    assert sum("Caja negra: no se pudo escribir" in m for m in caja.logs) == 1
    _alimentar(caja, r, [_paquete(1 + k * 0.02, P.MISSION) for k in range(50)])   # sigue grabando en memoria
    assert caja.vaciar() is False and sum("no se pudo escribir" in m for m in caja.logs) == 1
    bloqueo.unlink()
    # disco lleno a la mitad de un miembro: se recorta y se reintenta todo el lote (sin duplicar)
    real_open = open

    class _Mitad:
        def __init__(self, fh):
            self.fh = fh

        def __enter__(self):
            return self

        def __exit__(self, *a):
            self.fh.close()

        def write(self, datos):
            self.fh.write(datos[:len(datos) // 2])
            self.fh.flush()
            raise OSError(28, "No space left on device")

    fallar = {"n": 1}

    def open_falla(ruta, modo="r", *a, **k):
        fh = real_open(ruta, modo, *a, **k)
        if modo == "ab" and fallar["n"]:
            fallar["n"] -= 1
            return _Mitad(fh)
        return fh
    monkeypatch.setattr(cn, "open", open_falla, raising=False)
    assert caja.vaciar() is False
    (f,) = glob.glob(str(tmp_path / "vuelos" / "*" / "*.tel.jsonl.gz"))
    assert os.path.getsize(f) == 0                              # recortado a como estaba
    assert caja.vaciar() is True and caja.estado()["error"] == ""
    assert any("volvió a funcionar" in m for m in caja.logs)
    caja.cerrar("prueba")
    caja.vaciar()
    L = _lineas(f)
    tel = [l for l in L if l["tipo"] == "tel"]
    assert len(tel) == 100 and [l["t"] for l in tel] == sorted(l["t"] for l in tel)
    assert L[-1]["tipo"] == "fin" and L[-1]["n_tel"] == 100


def test_reintentos_no_recomprimen_y_el_desborde_conserva_la_cabecera(tmp_path, monkeypatch):
    from app import caja_negra as cn
    llamadas = {"n": 0}
    real = gzip.compress

    def contar(*a, **k):
        llamadas["n"] += 1
        return real(*a, **k)
    monkeypatch.setattr(cn.gzip, "compress", contar)
    r = _Reloj()
    bloqueo = tmp_path / "vuelos"
    bloqueo.write_text("x")                                     # makedirs falla desde la apertura
    caja = _caja(tmp_path, r)
    caja.max_cola = 100
    caja.comando("START", "web", "enviado", 1)
    _alimentar(caja, r, [_paquete(k * 0.02, P.TAKEOFF) for k in range(40)], muestras=False)
    assert caja.vaciar() is False and llamadas["n"] == 1
    for _ in range(20):                                         # reintentos sin líneas nuevas: nada se recomprime
        r.t += 2.5
        assert caja.vaciar(forzar=False) is False
    assert llamadas["n"] == 1 and caja.estado()["en_cola"] == 41
    # la falla dura más que max_cola: se tiran las líneas más viejas, nunca la cabecera
    for k in range(10):
        _alimentar(caja, r, [_paquete(1 + k + j * 0.02, P.MISSION) for j in range(30)], muestras=False)
        assert caja.vaciar() is False
    est = caja.estado()
    assert est["perdidas"] > 200 and est["en_cola"] <= 100
    bloqueo.unlink()
    caja.cerrar("prueba")
    assert caja.vaciar() is True
    (f,) = glob.glob(str(tmp_path / "vuelos" / "*" / "*.tel.jsonl.gz"))
    g = Grabacion(f)                                            # legible: conserva la cabecera
    assert g.cabecera["motivo_apertura"] == "START" and g.fin["motivo"] == "prueba"
    estados = [P.unpack_telemetry(raw)["mission_state"] for _, raw in g.tel]
    assert estados[:40] == [P.TAKEOFF] * 40 and estados[-1] == P.MISSION and len(g.tel) < 100   # lo más nuevo queda


def test_tope_de_duracion_por_archivo(tmp_path):
    r = _Reloj()
    caja = _caja(tmp_path, r, max_s=5.0)
    caja.comando("START", "web", "enviado", 1)
    _alimentar(caja, r, [_paquete(k * 0.02, P.PAUSED) for k in range(600)], muestras=False)   # 12 s en PAUSED
    assert caja.abierto()
    caja.cerrar("prueba")
    caja.vaciar()
    rutas = sorted(glob.glob(str(tmp_path / "vuelos" / "*" / "*.tel.jsonl.gz")))
    assert len(rutas) == 3
    gs = [Grabacion(x) for x in rutas]
    assert [g.fin["motivo"] for g in gs] == ["tope de 5 s por archivo"] * 2 + ["prueba"]
    assert all(g.t_fin - g.t_ini <= 5.1 for g in gs)
    assert [g.cabecera["motivo_apertura"] for g in gs] == ["START", "misión en PAUSED", "misión en PAUSED"]
    assert sum(len(g.tel) for g in gs) == 600                  # sin huecos ni duplicados entre archivos


def test_cambio_de_fuente_descarta_el_buffer(cfg_path, monkeypatch):
    from app import server as server_mod
    from test_solo_explorador import _BridgeSinRed
    monkeypatch.setattr(server_mod, "BridgeClient", _BridgeSinRed)
    st = Station(cfg_path, "mock", caja_negra="si")
    t = time.time()
    for k in range(200):                                        # 4 s de la fuente mock en el búfer previo
        st.caja.telemetria(_paquete(k * 0.02, P.ASSIGN_ROLES, d1=(9.9, 0.0, 0.0)), t + k * 0.02)
    assert st.caja.estado()["buffer_s"] > 3
    st.set_fuente("simulink")
    assert not any(isinstance(e, tuple) for _, e in st.caja._buf)    # solo el evento de la fuente nueva
    st.caja.detener()


def test_registro_usa_el_nombre_de_la_caja(tmp_path):
    r = _Reloj()
    caja = _caja(tmp_path, r)
    reg = RegistroVuelos(tmp_path / "vuelos", lambda: "simulink", lambda: True, reloj=r, hilo=False,
                         base_fn=caja.base_actual)
    caja.comando("START", "web", "enviado", 1)
    r.t += 20                                       # el registro abre después (gate de la visión)
    reg.tick(True, P.MISSION)
    reg.observaciones(r.t, [{"cls": 1, "conf": 0.9, "xy": [0, 0], "motivo": None}])
    assert reg.estado()["archivo"][:-len(".jsonl")] == caja.archivo_actual()[:-len(".tel.jsonl.gz")]
    reg.cerrar("x")
    caja.cerrar("x")
    # sin caja abierta: el nombre por la hora de apertura
    r.t += 100
    reg.tick(True, P.MISSION)
    reg.observaciones(r.t, [{"cls": 1, "conf": 0.9, "xy": [0, 0], "motivo": None}])
    lt = time.localtime(r.t)
    assert reg.estado()["archivo"].endswith(f"vuelo_{time.strftime('%H%M%S', lt)}_simulink.jsonl")


# =============================================================================== estación
def _esperar(cond, timeout=15.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.05)
    return False


def test_estacion_graba_el_mock_forzado_de_punta_a_punta(cfg_path):
    from fastapi.testclient import TestClient
    st = Station(cfg_path, "mock", mock_speed=4.0, caja_negra="si")
    assert st.caja_modo() == (True, "siempre")
    with TestClient(create_app(st)) as c:
        assert _esperar(lambda: (st.bridge.telemetry or {}).get("mission_state") == P.ASSIGN_ROLES)
        time.sleep(1.2)                              # algo de búfer previo y una muestra
        r = c.post("/api/cmd", json={"cmd": "START"})
        assert r.status_code == 200
        assert _esperar(lambda: (st.bridge.telemetry or {}).get("mission_state") == P.MISSION)
        assert _esperar(lambda: any(x["estado"] == "recibido" for x in st.bridge.cmd_results))
        time.sleep(0.5)
        snap = c.get("/api/estado").json()
        assert snap["caja_negra"]["abierto"] and snap["caja_negra"]["n_tel"] > 50
        assert snap["caja_negra"]["modo"] == "siempre" and snap["repeticion"] is None
    (f,) = glob.glob(os.path.join(os.path.dirname(cfg_path), "vuelos", "*", "*_mock.tel.jsonl.gz"))
    g = Grabacion(f)
    assert g.fin is not None and g.fin["motivo"] == "estación detenida"
    cab = g.cabecera
    assert cab["fuente"] == "mock" and cab["motivo_apertura"] == "START" and cab["config_datos"]["pista"]
    assert cab["puente"].startswith("tcpip://127.0.0.1:")
    assert g.t_ini < -1.0 and len(g.tel) > 100
    estados = [P.unpack_telemetry(raw)["mission_state"] for _, raw in g.tel]
    assert P.ASSIGN_ROLES in estados and P.TAKEOFF in estados and P.MISSION in estados
    (cmd,) = g.cmds
    assert cmd["cmd"] == "START" and cmd["resultado"] == "enviado" and cmd["origen"].startswith("web ")
    assert any(a["cmd"] == "START" and a["estado"] == "recibido" and a["latencia_s"] < 1.5 for a in g.acks)
    assert any("Caja negra abierta" in e["msg"] for e in g.eventos)
    assert any("Estado de misión" in e["msg"] for e in g.eventos)
    assert g.muestras and "fps" in g.muestras[-1]["camara"]
    res = herr.analizar(g)
    assert res["comandos"][0]["ack"] == "recibido" and res["comandos"][0]["latencia_tel_s"] is not None


def test_estacion_con_disco_roto_sigue_funcionando(cfg_path):
    from fastapi.testclient import TestClient
    carpeta = os.path.join(os.path.dirname(cfg_path), "vuelos")
    with open(carpeta, "w") as fh:                  # la carpeta de vuelos es un ARCHIVO: nunca se puede escribir
        fh.write("x")
    st = Station(cfg_path, "mock", mock_speed=4.0, caja_negra="si")
    with TestClient(create_app(st)) as c:
        assert _esperar(lambda: (st.bridge.telemetry or {}).get("mission_state") == P.ASSIGN_ROLES)
        assert c.post("/api/cmd", json={"cmd": "START"}).status_code == 200
        assert _esperar(lambda: (st.bridge.telemetry or {}).get("mission_state") == P.MISSION)
        assert _esperar(lambda: st.caja.estado()["errores"] > 0)
        n = st.bridge.n_rx
        time.sleep(0.3)
        assert st.bridge.n_rx > n and st.bridge.fresh()              # el puente sigue recibiendo
        snap = c.get("/api/estado").json()
        assert snap["caja_negra"]["error"] and snap["tel"]["mission_state"] >= P.TAKEOFF
    assert any("Caja negra: no se pudo escribir" in e["msg"] for e in st.events)


def test_habilitacion_cli_y_pytest(cfg_path):
    from test_revision import _patch_cfg
    assert Station(cfg_path, "mock").caja_modo()[0] is False
    assert Station(cfg_path, "sim3d").caja_modo()[0] is False
    assert Station(cfg_path, "simulink").caja_modo()[0] is True
    assert Station(cfg_path, "simulink", caja_negra="no").caja_modo()[0] is False
    _patch_cfg(cfg_path, lambda d: d.setdefault("web", {}).update(caja_negra=True))
    assert Station(cfg_path, "mock").caja_modo()[0] is True
    assert Station(cfg_path, "mock", caja_negra="auto").caja_modo()[0] is False
    _patch_cfg(cfg_path, lambda d: d["web"].update(caja_negra=False, caja_negra_decimacion=2))
    st = Station(cfg_path, "simulink")
    assert st.caja_modo()[0] is False and st.caja.decimacion == 2
    with pytest.raises(ValueError):
        Station(cfg_path, "mock", caja_negra="quizas")
    a = build_parser().parse_args(["--caja-negra", "si", "--repetir", "x.tel.jsonl.gz", "--velocidad", "4"])
    assert (a.caja_negra, a.repetir, a.velocidad) == ("si", "x.tel.jsonl.gz", 4.0)
    with pytest.raises(SystemExit):
        build_parser().parse_args(["--caja-negra", "tal-vez"])
    # pytest nunca escribe en web/datos, ni con la caja forzada
    from app.config import DEFAULT_PATH
    st = Station(DEFAULT_PATH, "mock", caja_negra="si")
    assert st.caja_modo()[0] is False and "pytest" in st.caja_modo()[1]
    antes = sorted(glob.glob(os.path.join(DATA_DIR, "vuelos", "**", "*"), recursive=True))
    st.caja.telemetria(_paquete(0, P.MISSION), time.time())
    st.caja.comando("START", "web", "enviado", 1)
    assert not st.caja.abierto()
    assert sorted(glob.glob(os.path.join(DATA_DIR, "vuelos", "**", "*"), recursive=True)) == antes


# =============================================================================== repetición
def _grabacion_para_repetir(tmp_path, cfg_path):
    """Vuelo corto (6 s): x del dron 1 distinta en cada paquete, estados ASSIGN_ROLES -> TAKEOFF -> MISSION."""
    with open(cfg_path, encoding="utf-8") as fh:
        cfg = json.load(fh)
    r = _Reloj()
    avisos = []
    caja = CajaNegra(tmp_path / "grabado", lambda: "simulink", lambda: True, reloj=r, hilo=False,
                     cabecera_fn=lambda: {"config_datos": cfg, "modo_mision": "dual",
                                          "eventos_previos": [{"t": "10:00:00", "msg": "evento viejo", "ts": 1.0}]},
                     muestra_fn=lambda: {"camara": {"fuente": ["dron", "192.168.2.15", 18800], "conectada": True,
                                                    "fps": 10.5, "edad_s": 0.1},
                                         "avisos": list(avisos), "vision": {"detector": "vision/detector.py (grabado)"},
                                         "objetivos": [{"id": 3, "cls": 1, "x": 0.25, "y": -0.5, "conf": 0.8, "n": 4,
                                                        "manual": False}], "candidatos": []})
    paquetes = []
    for k in range(300):
        estado = P.ASSIGN_ROLES if k < 50 else P.TAKEOFF if k < 120 else P.MISSION
        paquetes.append(_paquete(k * 0.02, estado, d1=(-1.0 + k * 0.001, 0.4, min(1.0, k * 0.01)), ack=1 if k > 60 else 0))
    _alimentar(caja, r, paquetes[:50])
    caja.comando("START", "web 127.0.0.1", "enviado", 1)
    caja.evento("Comando START enviado (cmd_seq 1)")
    avisos.append({"nivel": "rojo", "msg": "Aviso grabado"})
    _alimentar(caja, r, paquetes[50:])
    caja.cerrar("prueba")
    caja.vaciar()
    (f,) = glob.glob(str(tmp_path / "grabado" / "*" / "*.tel.jsonl.gz"))
    return f, paquetes


def test_repeticion_reproduce_los_mismos_paquetes_y_es_solo_lectura(tmp_path, cfg_path):
    from fastapi.testclient import TestClient
    f, paquetes = _grabacion_para_repetir(tmp_path, cfg_path)
    st = Station(cfg_path, repetir=f, velocidad_repeticion=4.0)
    assert st.fuente == "simulink" and st.caja_modo()[0] is False and st.registro_modo()[0] is False
    assert st._camera_key() == ("ninguna",) and st.cfg.path != os.path.abspath(cfg_path)   # config grabada
    vistos = []
    with TestClient(create_app(st)) as c:
        st.bridge.listeners.append(lambda tel, stamp: vistos.append(
            (tel["t"], tel["mission_state"], tel["drones"][1]["x"], tel["drones"][1]["z"])))
        assert _esperar(lambda: st.repeticion.estado()["fin"], timeout=10)
        snap = c.get("/api/estado").json()
        rep = snap["repeticion"]
        assert rep["fin"] and rep["pausado"] and rep["n_tel"] == 300 and rep["velocidad"] == 4.0
        # en pausa / al final sigue llegando el último paquete: la telemetría no se ve vieja
        assert snap["puente"]["fresh"] and snap["tel"]["mission_state"] == P.MISSION
        assert snap["tel"]["drones"]["1"]["x"] == pytest.approx(-1.0 + 299 * 0.001)
        # lo que no viaja en la telemetría sale de la grabación
        assert snap["avisos"] == [{"nivel": "rojo", "msg": "Aviso grabado"}]
        assert snap["camara"]["fps"] == 10.5 and snap["vision"]["detector"] == "vision/detector.py (grabado)"
        assert [t["id"] for t in snap["objetivos"]] == [3] and snap["objetivos"][0]["origen"] == "estacion"
        msgs = [e["msg"] for e in snap["eventos"]]
        assert "Comando START enviado (cmd_seq 1)" in msgs and "evento viejo" in msgs
        assert snap["registro_vuelo"]["habilitado"] is False and snap["caja_negra"]["modo"] == "desactivada (repetición)"
        # solo lectura: comandos y cambios -> 409; el control de la repetición sí responde
        r = c.post("/api/cmd", json={"cmd": "START"})
        assert r.status_code == 409 and "repetición" in r.json()["detail"]
        for ruta, body in (("/api/cmd", {"cmd": "LAND_ALL"}), ("/api/fuente", {"fuente": "mock"}),
                           ("/api/config/pista", {"vertices": [[0, 0], [1, 1]]}), ("/api/objetivos/reiniciar", {})):
            assert c.post(ruta, json=body).status_code == 409
        with pytest.raises(Exception):
            st.command("START")
        with pytest.raises(ValueError):
            st.set_fuente("mock")
        e = c.post("/api/repeticion", json={"t": 1.0, "accion": "pausa"}).json()
        assert e["t"] == pytest.approx(1.0) and e["pausado"] and not e["fin"] and e["saltos"] == 1
        # en pausa tras el salto llega el paquete grabado en t = 1.0 (y(1) = 1.98: START fue el paquete 50, t 0.98)
        assert _esperar(lambda: abs((st.bridge.telemetry or {}).get("t", -9) - 1.98) < 0.03, timeout=3)
        time.sleep(0.3)
        assert abs(st.bridge.telemetry["t"] - 1.98) < 0.03 and st.bridge.fresh()
        assert c.post("/api/repeticion", json={"velocidad": 0}).status_code == 400
        assert c.post("/api/repeticion", json={"accion": "bailar"}).status_code == 400
        # t no finito: 400 y la repetición sigue intacta (antes dejaba t = NaN y /api/estado respondía 500)
        for t_malo in ("nan", "inf", "-Infinity"):
            assert c.post("/api/repeticion", json={"t": t_malo}).status_code == 400
        assert c.get("/api/estado").status_code == 200 and st.repeticion.estado()["t"] == pytest.approx(1.0)
    # los paquetes llegaron por el puente en el mismo orden y con los mismos valores (hasta el final)
    unicos = []
    for v in vistos:
        if not unicos or v != unicos[-1]:
            unicos.append(v)
    esperado = [(P.unpack_telemetry(p)["t"], P.unpack_telemetry(p)["mission_state"],
                 P.unpack_telemetry(p)["drones"][1]["x"], P.unpack_telemetry(p)["drones"][1]["z"]) for p in paquetes]
    k = esperado.index(unicos[0])
    assert unicos[:len(esperado) - k] == esperado[k:]
    assert len(esperado) - k > 250
    # la repetición no escribió nada (ni caja negra ni registro ni objetivos)
    assert glob.glob(str(tmp_path / "**" / "*.tel.jsonl.gz"), recursive=True) == [f]
    assert not os.path.exists(os.path.join(os.path.dirname(cfg_path), "objetivos_estacion.json"))


# =============================================================================== herramienta
def _vuelo_para_resumen(tmp_path):
    """Dual: despegue, misión con pérdida de rastreo del dron 2 en el aire (0.5 s), hueco de 0.6 s y aterrizaje."""
    r = _Reloj()
    caja = _caja(tmp_path, r)
    p = []
    for k in range(100):
        p.append(_paquete(k * 0.02, P.ASSIGN_ROLES, fsm=(0, 0)))
    _alimentar(caja, r, p)
    caja.comando("START", "web 127.0.0.1", "enviado", 1)
    caja.comando("PAUSE", "web 127.0.0.1", "409", detalle="sin telemetría reciente")
    seq = []
    for k in range(100):            # TAKEOFF: el ack llega a los 0.2 s (y(54) = 1)
        seq.append(_paquete(2 + k * 0.02, P.TAKEOFF, d1=(-0.9, 0.4, k * 0.01), d2=(0.9, -0.4, k * 0.01),
                            ack=1 if k >= 10 else 0, fsm=(2, 2)))
    for k in range(200):            # MISSION: el dron 1 se acerca al 2; el dron 2 se pierde 0.5 s en el aire
        tr2 = 0 if 50 <= k < 75 else 1
        seq.append(_paquete(4 + k * 0.02, P.MISSION, d1=(-0.9 + k * 0.006, 0.4 - k * 0.002, 1.0),
                            d2=(0.9, -0.4, 1.5), tr=(1, tr2), ack=1))
    _alimentar(caja, r, seq[:100])
    caja.ack({"cmd": "START", "cmd_seq": 1, "estado": "recibido", "detalle": "", "t": r.t - 1.8, "t_click": r.t - 2.0})
    _alimentar(caja, r, seq[100:250])
    r.t += 0.6                       # hueco de telemetría
    _alimentar(caja, r, seq[250:])
    _alimentar(caja, r, [_paquete(8 + k * 0.02, P.LANDING, d1=(0.3, 0.0, max(0.0, 1.0 - k * 0.02)),
                                  d2=(0.9, -0.4, max(0.0, 1.5 - k * 0.02)), ack=1, fsm=(3, 3)) for k in range(100)])
    _alimentar(caja, r, [_paquete(10 + k * 0.02, P.DONE, d1=(0.3, 0.0, 0.0), d2=(0.9, -0.4, 0.0), ack=1, fsm=(9, 9))
                         for k in range(520)])
    assert not caja.abierto()
    caja.vaciar()
    (f,) = glob.glob(str(tmp_path / "vuelos" / "*" / "*.tel.jsonl.gz"))
    return f


def test_herramienta_resumen(tmp_path, capsys):
    f = _vuelo_para_resumen(tmp_path)
    assert herr.buscar(str(tmp_path / "vuelos")) == os.path.abspath(f)
    res = herr.analizar(Grabacion(f))
    assert [e["estado"] for e in res["estados"]] == ["ASSIGN_ROLES", "TAKEOFF", "MISSION", "LANDING", "DONE"]
    assert res["estados"][1]["t"] == pytest.approx(0.0, abs=0.03) and res["estados"][0]["t"] < -1.9
    assert [f_["fsm"] for f_ in res["flight_states"]["1"]] == [0, 2, 3, 9]
    start, pause = res["comandos"]
    assert start["ack"] == "recibido" and start["latencia_ack_s"] == pytest.approx(0.2)
    assert start["latencia_tel_s"] == pytest.approx(0.2, abs=0.03)
    assert pause["resultado"] == "409" and "ack" not in pause
    (perd,) = res["drones"]["2"]["perdidas_rastreo"]
    assert perd["dur_s"] == pytest.approx(0.5, abs=0.03) and perd["en_el_aire"] and perd["estado"] == "MISSION"
    assert res["drones"]["1"]["perdidas_rastreo"] == []
    assert res["drones"]["2"]["max_z"]["valor"] == pytest.approx(1.5)
    assert res["drones"]["1"]["max_abs_x"]["valor"] == pytest.approx(0.9)
    sep = res["separacion_min_aire"]
    assert sep["m"] == pytest.approx(math.hypot(0.6, 0.4), abs=1e-3) and sep["estado"] == "LANDING"   # bajando
    assert res["huecos"]["n"] == 1 and res["huecos"]["lista"][0]["dur_s"] == pytest.approx(0.62, abs=0.01)
    assert res["drones"]["1"]["tiempo_en_el_aire_s"] == pytest.approx(7.3, abs=0.4)
    assert res["motivo_cierre"] == "misión en DONE" and not res["cortado"] and res["hz_medio"] > 40
    assert herr.main(["resumen", f]) == 0
    out = capsys.readouterr().out
    assert "PÉRDIDA DE RASTREO" in out and "EN EL AIRE" in out and "Huecos de telemetría > 0.2 s: 1" in out
    assert herr.main(["resumen", f, "--json"]) == 0
    assert json.loads(capsys.readouterr().out)["n_tel"] == res["n_tel"]
    assert herr.main(["resumen", str(tmp_path / "no_existe")]) == 2


def test_herramienta_csv(tmp_path):
    f = _vuelo_para_resumen(tmp_path)
    salida = tmp_path / "vuelo.csv"
    assert herr.main(["csv", f, "-o", str(salida)]) == 0
    with open(salida, encoding="utf-8") as fh:
        filas = list(csv.reader(fh))
    assert filas[0] == ["t_estacion"] + nombres_y()
    g = Grabacion(f)
    assert len(filas) - 1 == len(g.tel)
    col = filas[0].index("d2_x")
    perdida = [r for r in filas[1:] if r[filas[0].index("d2_tracked")] == "0.0"]
    assert perdida and all(r[col] == "NaN" for r in perdida)
    assert float(filas[1][0]) < 0
    with open(tmp_path / "vuelo_eventos.csv", encoding="utf-8") as fh:
        ev = list(csv.DictReader(fh))
    assert [e["cmd"] for e in ev if e["tipo"] == "cmd"] == ["START", "PAUSE"]
    assert any(e["tipo"] == "ack" and e["estado"] == "recibido" for e in ev)
