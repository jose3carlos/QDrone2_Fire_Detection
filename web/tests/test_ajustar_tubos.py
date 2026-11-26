"""web/herramientas/ajustar_tubos.py: propone pista.tubos a partir de los registros de vuelo.

Registro sintético: tubo 1 real = el de la config; tubo 2 real desplazado 0.2 m hacia y-. Las tomas sobre el tubo 2
real que quedan a > 0.15 m del configurado (filtro de ese vuelo) se registran como "lejos del tubo" (igual que la estación). La
herramienta debe mover el tubo 2 (±3 cm) sin tocar el tubo 1, y ``--aplicar`` (sobre una COPIA de la config) debe
cambiar solo el arreglo pista.tubos conservando CRLF y el resto byte a byte.
"""
import importlib.util
import json
import math
import os
import shutil

import numpy as np
import pytest

from app.geo import dist_tubos, tubo_cercano
from conftest import PISTA_LAB

WEB = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_spec = importlib.util.spec_from_file_location("ajustar_tubos", os.path.join(WEB, "herramientas", "ajustar_tubos.py"))
AT = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(AT)

T1, T2 = PISTA_LAB["tubos"]
DY = -0.20
T2_REAL = [[p[0], p[1] + DY] for p in T2]
LIM_TOMA = 0.15     # filtro del vuelo sintético (con 0.25 las tomas a 0.2 m se habrían aceptado: también cuentan)


def _sobre(P, k, f):
    a, b = P[k], P[k + 1]
    return [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])]


# tomas reales: 4 sobre el tubo 1, 5 sobre el tubo 2 real (2 en el tramo 1, 3 en el tramo 2)
TOMAS = ([_sobre(T1, 0, 0.5), _sobre(T1, 1, 0.3), _sobre(T1, 1, 0.7), _sobre(T1, 2, 0.5)]
         + [_sobre(T2_REAL, 0, 0.4), _sobre(T2_REAL, 0, 0.7), _sobre(T2_REAL, 1, 0.25), _sobre(T2_REAL, 1, 0.5),
            _sobre(T2_REAL, 1, 0.75)])


def _escribir_registro(ruta, tomas, tubos_cfg, n=6, ruido=0.01, seed=0, fecha="2026-11-25T10:00:00"):
    rng = np.random.default_rng(seed)
    os.makedirs(os.path.dirname(ruta), exist_ok=True)
    lineas = [{"tipo": "cabecera", "formato": 1, "fecha": fecha, "fuente": "simulink",
               "pista": {"vertices": PISTA_LAB["vertices"], "tubos": tubos_cfg, "tramos_sin_tubo": [4]},
               "filtros": {"dist_max_tubo_toma_m": LIM_TOMA}}]
    t = 0.0
    for x, y in tomas:
        for _ in range(n):
            t += 0.1
            ox, oy = x + rng.normal(0, ruido), y + rng.normal(0, ruido)
            tc = tubo_cercano(ox, oy, tubos_cfg)
            lineas.append({"tipo": "obs", "t": round(t, 3), "cls": 1, "conf": 0.95, "xy": [round(ox, 3), round(oy, 3)],
                           "motivo": "lejos del tubo" if tc["dist"] > LIM_TOMA else None,
                           "dist_tubo_m": round(tc["dist"], 3)})
    # ruido que la herramienta debe ignorar: fuego lejos, toma descartada por tamaño, una toma suelta
    lineas.append({"tipo": "obs", "t": t, "cls": 0, "conf": 0.95, "xy": [0.0, -0.9], "motivo": None})
    lineas.append({"tipo": "obs", "t": t, "cls": 1, "conf": 0.95, "xy": [0.0, 0.0], "motivo": "tamaño"})
    lineas.append({"tipo": "obs", "t": t, "cls": 1, "conf": 0.95, "xy": [-0.5, -0.5], "motivo": "lejos del tubo"})
    lineas.append({"tipo": "resumen", "n_obs": len(lineas) - 1})
    with open(ruta, "w", encoding="utf-8", newline="\n") as fh:
        for l in lineas:
            fh.write(json.dumps(l) + "\n")
        fh.write('{"tipo": "obs", "roto\n')                       # vuelo cortado: línea rota al final
    return ruta


def _correr(argv):
    salida = []
    code = AT.ejecutar(AT.build_parser().parse_args(argv), out=salida.append)
    return code, "\n".join(salida)


def _propuesta(cfg_path, registro, *extra):
    args = AT.build_parser().parse_args([registro, "--config", cfg_path, *extra])
    with open(cfg_path, encoding="utf-8") as fh:
        cfg = json.load(fh)
    tubos = cfg["pista"]["tubos"]
    vuelos = [AT.leer_vuelo(r) for r in AT.archivos_registro([registro])]
    clusters = []
    for v in vuelos:
        for c in AT.agrupar(AT.obs_utiles(v), 0.25, 3):
            if c["confirmado"]:
                c["tc"] = tubo_cercano(c["x"], c["y"], tubos)
                clusters.append(c)
    return AT.proponer(tubos, clusters, umbral=args.umbral, min_clusters=args.min_clusters, n_fuerte=args.n_fuerte,
                       max_desp=args.max_desp, margen=args.margen)


def test_el_registro_sintetico_tiene_tomas_lejos_del_tubo(tmp_path):
    reg = _escribir_registro(str(tmp_path / "v" / "vuelo_1.jsonl"), TOMAS, PISTA_LAB["tubos"])
    v = AT.leer_vuelo(reg)
    assert v["malas"] == 1 and v["cabecera"]["fuente"] == "simulink" and v["resumen"] is not None
    lejos = [o for o in v["obs"] if o.get("motivo") == "lejos del tubo"]
    assert len(lejos) >= 15                                     # tramo 2 del tubo 2 (a 0.2 m): casi todo lejos
    assert len(AT.obs_utiles(v)) == 6 * len(TOMAS) + 1           # sin el fuego ni la de tamaño


def test_propone_el_corrimiento_del_tubo_2_y_no_mueve_el_tubo_1(tmp_path, cfg_path_lab):
    reg = _escribir_registro(str(tmp_path / "v" / "vuelo_1.jsonl"), TOMAS, PISTA_LAB["tubos"])
    nuevos, dec, ext, ign = _propuesta(cfg_path_lab, reg)
    assert nuevos[0] == T1                                       # el tubo 1 no se mueve
    assert len(nuevos[1]) == 3
    # el vértice compartido del tubo 2 = intersección de los tramos movidos = el real
    assert math.hypot(nuevos[1][1][0] - T2_REAL[1][0], nuevos[1][1][1] - T2_REAL[1][1]) <= 0.03
    por_tramo = {(d["pieza"], d["tramo"]): d for d in dec}
    assert all(not d["aplica"] for (i, _), d in por_tramo.items() if i == 0)
    # tramo 2 del tubo 2 (casi paralelo al eje x, sentido -x): +0.2 a la izquierda = hacia y-
    assert por_tramo[(1, 1)]["aplica"] and por_tramo[(1, 1)]["mediana"] == pytest.approx(0.2, abs=0.03)
    d0 = por_tramo[(1, 0)]
    n0 = AT._normal_izq(T2[0], T2[1])
    assert d0["aplica"] and d0["mediana"] == pytest.approx(DY * n0[1], abs=0.03)
    # cada toma real del tubo 2 queda sobre el tubo propuesto (±3 cm); la ruta de vuelo no se toca
    for x, y in TOMAS[4:]:
        assert dist_tubos(x, y, [nuevos[1]]) <= 0.03
    assert ext == [] and ign == []


def test_reporte_svg_y_sin_aplicar_no_escribe(tmp_path, cfg_path_lab):
    reg = _escribir_registro(str(tmp_path / "v" / "2026-11-25" / "vuelo_1.jsonl"), TOMAS, PISTA_LAB["tubos"])
    antes = open(cfg_path_lab, "rb").read()
    svg = str(tmp_path / "tubos.svg")
    code, txt = _correr([str(tmp_path / "v"), "--config", cfg_path_lab, "--svg", svg])
    assert code == 0
    assert open(cfg_path_lab, "rb").read() == antes and not os.path.exists(cfg_path_lab + ".bak")
    assert "LEJOS" in txt and "tubo 2 tramo 2" in txt and "se mueve" in txt and "pista.tubos propuesto" in txt
    assert "pista.vertices (ruta de vuelo) NO se toca" in txt and "1 líneas rotas" in txt
    assert "AVISO" not in txt                                    # la cabecera tiene los tubos de la config
    s = open(svg, encoding="utf-8").read()
    assert s.startswith("<svg") and s.count("<polyline") >= 6 and "toma rechazada" in s and "#d62728" in s
    # la lista JSON propuesta (2 decimales, formato de estacion.json) está en el reporte
    bloque = txt.split('"tubos": ', 1)[1].split("\n    ]", 1)[0] + "]"
    nuevos = _propuesta(cfg_path_lab, reg)[0]
    assert json.loads(bloque) == nuevos and nuevos[1][1] == [0.99, -1.61]


def test_aplicar_solo_cambia_pista_tubos_y_conserva_crlf(tmp_path, cfg_path_lab):
    copia = str(tmp_path / "copia" / "estacion.json")
    os.makedirs(os.path.dirname(copia))
    shutil.copy(cfg_path_lab, copia)
    orig = open(copia, "rb").read()
    assert b"\r\n" in orig                                       # la config real (y su copia) usa CRLF
    reg = _escribir_registro(str(tmp_path / "v" / "vuelo_1.jsonl"), TOMAS, PISTA_LAB["tubos"])
    code, txt = _correr([reg, "--config", copia, "--aplicar"])
    assert code == 0 and "--aplicar: pista.tubos reescrito" in txt
    nuevo = open(copia, "rb").read()
    assert open(copia + ".bak", "rb").read() == orig
    assert nuevo.count(b"\r\n") == nuevo.count(b"\n")           # sin LF sueltos
    lo, ln = orig.decode("utf-8").split("\r\n"), nuevo.decode("utf-8").split("\r\n")
    assert len(lo) == len(ln)
    distintas = [i for i, (a, b) in enumerate(zip(lo, ln)) if a != b]
    assert len(distintas) == 1 and lo[distintas[0]].strip().startswith("[[0.47, -0.25]")   # solo la línea del tubo 2
    a, b = json.loads(orig), json.loads(nuevo)
    assert b["pista"]["tubos"][0] == T1 and b["pista"]["tubos"][1] != T2
    assert b["pista"]["vertices"] == a["pista"]["vertices"]
    a["pista"]["tubos"] = b["pista"]["tubos"]
    assert a == b
    # con la config ya corregida, volver a correr no propone nada (los desplazamientos quedan bajo el umbral)
    code, txt = _correr([reg, "--config", copia, "--aplicar"])
    assert code == 0 and "no hay cambios que escribir" in txt


def test_reemplazar_tubos_agrega_la_clave_si_no_existe(cfg_path):
    texto = open(cfg_path, "rb").read().decode("utf-8")
    assert '"tubos"' not in texto
    nuevo = AT.reemplazar_tubos(texto, [[[0.0, 0.0], [1.0, 0.5]]])
    assert json.loads(nuevo)["pista"]["tubos"] == [[[0.0, 0.0], [1.0, 0.5]]]
    assert ("\r\n" in texto) == ("\r\n" in nuevo) and nuevo.replace("\r\n", "\n").count("\n") == \
        texto.replace("\r\n", "\n").count("\n") + 3


def test_alarga_un_extremo_y_avisa_tubos_distintos_en_las_cabeceras(tmp_path, cfg_path_lab):
    # una toma real 0.3 m más allá del final del tubo 2 (en la prolongación del último tramo)
    a, b = T2[1], T2[2]
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    mas_alla = [b[0] + 0.3 * (b[0] - a[0]) / L, b[1] + 0.3 * (b[1] - a[1]) / L]
    otros = [[p[0], p[1] + 0.1] for p in T2]
    r1 = _escribir_registro(str(tmp_path / "v" / "vuelo_1.jsonl"), [mas_alla], PISTA_LAB["tubos"], n=10)
    _escribir_registro(str(tmp_path / "v" / "vuelo_2.jsonl"), [], [T1, otros], fecha="2026-11-25T11:00:00")
    nuevos, dec, ext, ign = _propuesta(cfg_path_lab, r1)
    assert len(ext) == 1 and ext[0]["pieza"] == 1 and ext[0]["extremo"] == 2
    assert ext[0]["alargue_m"] == pytest.approx(0.35, abs=0.03)
    assert dist_tubos(mas_alla[0], mas_alla[1], [nuevos[1]]) <= 0.03 and nuevos[1][:2] == T2[:2]
    code, txt = _correr([str(tmp_path / "v"), "--config", cfg_path_lab])
    assert "se alarga el extremo 3" in txt and "tubos distintos entre sí" in txt
    code, txt = _correr([str(tmp_path / "v"), "--config", cfg_path_lab, "--ultimos", "1"])
    assert "vuelo_2" in txt and "vuelo_1" not in txt


def test_cluster_junto_a_un_vertice_compartido_no_mueve_el_tramo_vecino():
    # tramo 2 del tubo 2 real 0.2 m hacia y-; un cluster fuerte (n=10) en la cuña exterior del vértice compartido
    # empata entre los dos tramos y queda en el tramo 1 con u > 1: NO debe mover el tramo 1 (que no se movió)
    tubos = [T1, T2]
    cl = [{"x": 1.05, "y": -1.68, "n": 10}, {"x": 0.2, "y": -1.65, "n": 6}, {"x": -0.7, "y": -1.70, "n": 6}]
    for c in cl:
        c["tc"] = tubo_cercano(c["x"], c["y"], tubos)
    assert (cl[0]["tc"]["pieza"], cl[0]["tc"]["tramo"]) == (1, 0) and cl[0]["tc"]["u"] > 1
    nuevos, dec, ext, ign = AT.proponer(tubos, cl)
    por_tramo = {(d["pieza"], d["tramo"]): d for d in dec}
    assert not por_tramo[(1, 0)]["aplica"] and por_tramo[(1, 0)]["mediana"] is None
    assert por_tramo[(1, 1)]["aplica"] and por_tramo[(1, 1)]["mediana"] == pytest.approx(0.2, abs=0.01)
    assert nuevos[0] == T1 and nuevos[1][0] == T2[0]                 # el extremo libre del tramo 1 no se mueve
    a, b = nuevos[1][0], nuevos[1][1]                                # vértice compartido sobre el tramo 1 original
    assert abs((b[0] - a[0]) * (T2[1][1] - a[1]) - (b[1] - a[1]) * (T2[1][0] - a[0])) < 0.01
    assert ext == []


def test_sin_registros(tmp_path, cfg_path_lab):
    code, txt = _correr([str(tmp_path / "vacia"), "--config", cfg_path_lab])
    assert code == 1 and "No hay registros" in txt


def test_avisa_si_el_vuelo_es_de_otra_config(tmp_path, cfg_path_lab):
    """A9: los vuelos duales quedan en web/datos/vuelos junto a los de solo explorador; si la cabecera dice otra config
    que --config, se avisa (--aplicar solo reescribe --config)."""
    reg = _escribir_registro(str(tmp_path / "v" / "vuelo_100000_simulink.jsonl"), TOMAS, PISTA_LAB["tubos"])
    with open(reg, encoding="utf-8") as fh:
        lineas = fh.read().splitlines()

    def con_config(cfg_cabecera):
        cab = dict(json.loads(lineas[0]), config=cfg_cabecera)
        with open(reg, "w", encoding="utf-8") as fh:
            fh.writelines(l + chr(10) for l in [json.dumps(cab)] + lineas[1:])
    con_config("config/estacion_dual.json")
    code, txt = _correr([reg, "--config", cfg_path_lab])
    assert code == 0 and "otra config (config/estacion_dual.json)" in txt
    con_config(AT._rel(os.path.abspath(cfg_path_lab)))              # la misma config: sin aviso
    assert "otra config" not in _correr([reg, "--config", cfg_path_lab])[1]
