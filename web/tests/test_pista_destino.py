"""Pestaña Pista: "Capturar vértice" con destino (ruta de vuelo o tubos reales), ``POST /api/config/pista``.

* ``destino = "vertices"`` (por defecto, compatible con la UI vieja): escribe solo ``pista.vertices`` (la ruta que
  compila Simulink) y responde ``fuera_envolvente`` (vértices fuera del límite de consignas: aviso, no bloquea);
* ``destino = "tubos"``: escribe solo ``pista.tubos`` (tubos REALES en el piso; Simulink no lo lee); cada pieza con
  2 o más puntos;
* el resto de ``estacion.json`` queda byte a byte (mismo texto y CRLF), siempre sobre COPIAS de la config.
"""
import json
import math
import os
import shutil

import pytest
from fastapi.testclient import TestClient

from app.config import puntos_fuera_limite_consignas
from app.geo import tubos_pista
from app.server import Station, create_app

WEB = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(os.path.dirname(WEB), "config")
TUBOS_NUEVOS = [[[1.0, 0.4], [1.01, 1.42], [-1.06, 1.38], [-0.28, 0.3]],
                [[0.48, -0.26], [0.98, -1.43], [-1.18, -1.53]],
                [[-1.3, 0.0], [-1.3, 0.5]]]


def _cliente(path):
    st = Station(str(path), "mock", mock_speed=4.0)
    c = TestClient(create_app(st))
    c.station = st
    c.cfg_path = str(path)
    return c


@pytest.fixture(params=["estacion.json", "estacion_dual.json"])
def copia_real(request, tmp_path):
    """Copia BYTE A BYTE de config/estacion.json o estacion_dual.json (CRLF, con pista.tubos y limites_y)."""
    dst = tmp_path / request.param
    shutil.copy(os.path.join(CONFIG, request.param), dst)
    return dst


def _lineas(path):
    raw = open(path, "rb").read()
    assert b"\r\n" in raw and raw.count(b"\n") == raw.count(b"\r\n"), "solo CRLF"
    return raw.decode("utf-8").split("\r\n")


def _bloque(lineas, clave):
    """(inicio, fin) del valor multilínea de ``clave`` dentro de pista (fin = línea del corchete de cierre)."""
    i = next(k for k, l in enumerate(lineas) if l.startswith(f'    "{clave}": '))
    if lineas[i].rstrip(",").endswith("]") and not lineas[i].rstrip(",").endswith("["):
        return i, i
    f = next(k for k in range(i + 1, len(lineas)) if lineas[k].startswith("    ]"))
    return i, f


def test_tubos_solo_cambia_ese_arreglo(copia_real):
    antes = _lineas(copia_real)
    d0 = json.loads("\r\n".join(antes))
    with _cliente(copia_real) as c:
        r = c.post("/api/config/pista", json={"destino": "tubos", "tubos": TUBOS_NUEVOS})
        assert r.status_code == 200, r.text
        assert r.json() == {"ok": True, "destino": "tubos", "tubos": TUBOS_NUEVOS}
    despues = _lineas(copia_real)
    i0, f0 = _bloque(antes, "tubos")
    i1, f1 = _bloque(despues, "tubos")
    assert i0 == i1 and despues[:i0] == antes[:i0]            # todo lo anterior, idéntico
    assert despues[f1 + 1:] == antes[f0 + 1:]                 # todo lo posterior, idéntico
    assert despues[i1:f1 + 1] == ['    "tubos": [',
                                  "      [[1.0, 0.4], [1.01, 1.42], [-1.06, 1.38], [-0.28, 0.3]],",
                                  "      [[0.48, -0.26], [0.98, -1.43], [-1.18, -1.53]],",
                                  "      [[-1.3, 0.0], [-1.3, 0.5]]",
                                  "    ]"]
    d1 = json.loads("\r\n".join(despues))
    assert d1["pista"]["tubos"] == TUBOS_NUEVOS
    d1["pista"]["tubos"] = d0["pista"]["tubos"]
    assert d1 == d0                                           # la ruta y todo lo demás, intactos


def test_ruta_solo_cambia_vertices_y_avisa_fuera_de_envolvente(copia_real):
    antes = _lineas(copia_real)
    d0 = json.loads("\r\n".join(antes))
    assert d0["mision"]["limites_y"] == [-1.4, 1.2]
    # el 2.º vértice en y = 1.3 (como si se hubiera capturado sobre el tubo): se guarda, con aviso
    v = [[1.02, 0.41], [0.99, 1.3], [-1.05, 1.1], [-0.27, 0.29], [0.47, -0.25], [0.99, -1.01], [-1.17, -1.2]]
    with _cliente(copia_real) as c:
        r = c.post("/api/config/pista", json={"destino": "vertices", "vertices": v}).json()
        assert r["ok"] and r["destino"] == "vertices" and r["vertices"] == v
        assert r["fuera_envolvente"] == [[0.99, 1.3]]
        # sin destino = ruta (compatibilidad), dentro de la envolvente: sin aviso
        r = c.post("/api/config/pista", json={"vertices": d0["pista"]["vertices"]}).json()
        assert r["destino"] == "vertices" and r["fuera_envolvente"] == []
    assert _lineas(copia_real) == antes                       # volvió a la ruta original: archivo idéntico
    with _cliente(copia_real) as c:
        c.post("/api/config/pista", json={"vertices": v})
    despues = _lineas(copia_real)
    i0, _ = _bloque(antes, "vertices")
    assert [k for k, (a, b) in enumerate(zip(antes, despues)) if a != b] == [i0] and len(antes) == len(despues)
    assert json.loads("\r\n".join(despues))["pista"]["tubos"] == d0["pista"]["tubos"]


def test_tubos_valida_piezas(copia_real):
    raw0 = open(copia_real, "rb").read()
    malos = [
        {"destino": "tubos"},                                                      # falta la clave
        {"destino": "tubos", "tubos": [[[0, 0], [1, 0]], [[0.5, 0.5]]]},           # pieza de 1 punto
        {"destino": "tubos", "tubos": [[[0, 0], [1, 0]], []]},                     # pieza vacía
        {"destino": "tubos", "tubos": [[[0, 0], [1, 0, 2]]]},                      # punto de 3 coordenadas
        {"destino": "tubos", "tubos": [[[0, 0], ["a", 0]]]},                       # texto
        {"destino": "tubos", "tubos": [[0, 0, 1, 0]]},                             # pieza sin pares
        {"destino": "tubos", "tubos": {"a": [[0, 0], [1, 0]]}},                    # no es lista
        {"destino": "tubos", "tubos": [[[0, 0], [3.5, 0]]]},                       # fuera de la arena
        {"destino": "tubos", "tubos": [[[0, 0], [1, 0]]] * 17},                    # demasiadas piezas
        {"destino": "tubos", "tubos": [[[0.01 * k, 0] for k in range(33)]]},       # demasiados puntos
        {"destino": "ruta", "vertices": [[0, 0], [1, 0]]},                         # destino desconocido
        {"destino": "vertices", "vertices": [[0, 0]]},                             # ruta de 1 vértice
        {"destino": "vertices", "vertices": [[0, 0], [1.6, 0]]},                   # ruta fuera de limites_xy
        {"destino": "vertices", "tubos": TUBOS_NUEVOS},                            # ruta sin vertices
    ]
    with _cliente(copia_real) as c:
        for body in malos:
            r = c.post("/api/config/pista", json=body)
            assert r.status_code == 400, (body, r.text)
        msg = c.post("/api/config/pista", json=malos[1]).json()["detail"]
        assert "pieza 2" in msg and "al menos 2" in msg
    assert open(copia_real, "rb").read() == raw0              # ningún intento fallido tocó el archivo


def test_tubos_vacios_vuelven_a_la_ruta(copia_real):
    """``tubos = []`` (la UI con todas las piezas vaciadas) quita los tubos reales: escribe ``"tubos": []`` y el
    filtro/dibujo vuelven a la ruta partida en tramos_sin_tubo, como sin la clave; nada más cambia."""
    d0 = json.loads("\r\n".join(_lineas(copia_real)))
    sin_clave = {k: v for k, v in d0["pista"].items() if k != "tubos"}
    with _cliente(copia_real) as c:
        r = c.post("/api/config/pista", json={"destino": "tubos", "tubos": []})
        assert r.status_code == 200, r.text
        assert r.json() == {"ok": True, "destino": "tubos", "tubos": []}
        assert c.station.snapshot()["tubos"] == tubos_pista(sin_clave) != tubos_pista(d0["pista"])
    d1 = json.loads("\r\n".join(_lineas(copia_real)))
    assert d1["pista"]["tubos"] == []
    d1["pista"]["tubos"] = d0["pista"]["tubos"]
    assert d1 == d0


def test_tubos_nuevos_sin_clave_y_estado(cfg_path):
    """Config sin pista.tubos (fixture ``cfg_path``: N de 4 vértices): guardar tubos agrega la clave dentro de pista,
    los tubos del estado (filtro de tomas y dibujo) pasan a ser esos y la ruta no cambia."""
    d0 = json.load(open(cfg_path, encoding="utf-8"))
    assert "tubos" not in d0["pista"]
    with _cliente(cfg_path) as c:
        assert c.post("/api/config/pista", json={"destino": "tubos", "tubos": TUBOS_NUEVOS[:2]}).status_code == 200
        st = c.station.snapshot()
        assert st["tubos"] == TUBOS_NUEVOS[:2] and st["pista"] == d0["pista"]["vertices"]
    d1 = json.load(open(cfg_path, encoding="utf-8"))
    assert d1["pista"]["tubos"] == TUBOS_NUEVOS[:2]
    d1["pista"].pop("tubos")
    assert d1 == d0


def test_puntos_fuera_limite_consignas():
    pista = [[0.0, 1.3], [1.36, 0.0], [0.0, -1.38], [1.0, 1.0]]
    # con envolvente en y: x usa limites_xy -/+ 0.15 (±1.35), y la envolvente tal cual
    cfg = {"mision": {"limites_xy": [-1.5, 1.5], "limites_y": [-1.4, 1.2]}}
    assert puntos_fuera_limite_consignas(cfg, pista) == [[0.0, 1.3], [1.36, 0.0]]
    # sin envolvente: limites_xy con su margen (±1.35)
    cfg = {"mision": {"limites_xy": [-1.5, 1.5]}}
    assert puntos_fuera_limite_consignas(cfg, pista) == [[1.36, 0.0], [0.0, -1.38]]
    assert puntos_fuera_limite_consignas({}, [[0, 1.35], [math.nan, 0], "x"]) == []
