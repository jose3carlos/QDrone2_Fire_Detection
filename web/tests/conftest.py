"""Configuración común de pytest: importa ``app`` desde web/ y da una copia temporal de estacion.json.

Las pruebas no dependen del modo ni de la pista del día que tenga ``config/estacion.json``:
* ``cfg_path``: copia con ``mision.modo = "dual"`` (sin ``dron_solo``) y la N de 4 vértices de siempre;
* ``cfg_path_solo``: copia con ``mision.modo = "solo_explorador"``, ``dron_solo = 1`` y la pista de 7 vértices
  del dibujo del 23-nov;
* ``cfg_path_lab``: copia en modo solo explorador (dron 1) con la pista del laboratorio del 24-nov
  (``PISTA_LAB``: ruta de vuelo de 7 vértices, hueco en el tramo 4 y los tubos REALES en ``pista.tubos``).
Si el archivo real ya es dual con la N de siempre (y sin ``pista.tubos``), ``cfg_path`` es una copia byte a byte
(sin reescribir). Al cambiar la ruta se quitan ``pista.tubos`` / ``pista.nota_tubos`` del día (son de otra ruta).
"""
import json
import os
import shutil
import sys

import pytest

WEB = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT = os.path.dirname(WEB)
sys.path.insert(0, WEB)

REAL_CFG = os.path.join(PROJECT, "config", "estacion.json")
N_DUAL = [[-1.2, 1.2], [1.2, 1.2], [-1.2, -1.2], [1.2, -1.2]]
PISTA_SOLO = [[1.02, 0.41], [0.99, 1.20], [-0.90, 1.10], [-0.27, 0.29], [0.47, -0.25], [0.99, -1.01], [-1.02, -1.20]]
# Pista del laboratorio del 24-nov (config/estacion.json de b65603f): vertices = RUTA DE VUELO (Simulink, 8.0 m,
# 36 waypoints), tramo 4 sin tubo y tubos = posición REAL de las 2 piezas, más afuera que la ruta.
PISTA_LAB = {
    "vertices": [[1.02, 0.41], [0.99, 1.2], [-1.05, 1.1], [-0.27, 0.29], [0.47, -0.25], [0.99, -1.01], [-1.17, -1.2]],
    "tramos_sin_tubo": [4],
    "tubos": [[[1.02, 0.41], [0.99, 1.41], [-1.05, 1.37], [-0.27, 0.29]],
              [[0.47, -0.25], [0.99, -1.41], [-1.17, -1.52]]],
}


def _copia(dst, modo, dron_solo, vertices, pista=None):
    """Copia de la config real con ese modo y esa pista, con el mismo estilo y finales de línea del original.
    ``pista`` (dict) sustituye la sección completa; si no, solo se cambian los vértices y se quitan los tubos."""
    import copy
    from app.config import _Num, dumps
    raw = open(REAL_CFG, "rb").read()
    d = json.loads(raw.decode("utf-8-sig"), parse_float=_Num)
    m = d.setdefault("mision", {})
    same = (m.get("modo") in (None, modo) if modo == "dual" else m.get("modo") == modo) and \
        m.get("dron_solo") == dron_solo and d.get("pista", {}).get("vertices") == vertices and \
        pista is None and "tubos" not in d.get("pista", {})
    if same:
        shutil.copy(REAL_CFG, dst)
        return str(dst)
    m.pop("modo", None)
    m.pop("dron_solo", None)
    m["modo"] = modo
    if dron_solo is not None:
        m["dron_solo"] = dron_solo
    if pista is not None:
        d["pista"] = copy.deepcopy(pista)
    else:
        d.setdefault("pista", {})["vertices"] = [list(v) for v in vertices]
        for k in ("tubos", "nota_tubos"):        # tubos reales del día: son de otra ruta
            d["pista"].pop(k, None)
    nl = "\r\n" if b"\r\n" in raw else "\n"
    with open(dst, "w", encoding="utf-8", newline=nl) as fh:
        fh.write(dumps(d) + "\n")
    return str(dst)


@pytest.fixture
def cfg_path(tmp_path):
    """Copia de config/estacion.json (dual, N de 4 vértices): las pruebas nunca escriben el archivo real."""
    return _copia(tmp_path / "estacion.json", "dual", None, N_DUAL)


@pytest.fixture
def cfg_path_solo(tmp_path):
    """Copia de config/estacion.json en modo solo explorador (dron 1) con la pista de 7 vértices."""
    return _copia(tmp_path / "estacion_solo.json", "solo_explorador", 1, PISTA_SOLO)


@pytest.fixture
def cfg_path_lab(tmp_path):
    """Copia de config/estacion.json en modo solo explorador (dron 1) con la pista del lab del 24-nov
    (ruta de vuelo + tubos reales separados, ``PISTA_LAB``)."""
    return _copia(tmp_path / "estacion_lab.json", "solo_explorador", 1, PISTA_LAB["vertices"], pista=PISTA_LAB)
