"""Propone ``pista.tubos`` corregidos a partir de los registros de vuelo (``web/datos/vuelos/**/*.jsonl``).

El espacio del laboratorio es compartido y los tubos se mueven. La estación descarta una toma "lejos del tubo"
(``vision.dist_max_tubo_toma_m``) si el tubo configurado no está donde está el real; cada vuelo deja en el registro
(``app/registro_vuelo.py``) esas observaciones junto con las aceptadas. Esta herramienta:

1. lee los registros (archivos o carpetas; por defecto todos los de ``web/datos/vuelos``) y la config;
2. por vuelo, agrupa las tomas (cls 1) aceptadas y rechazadas "lejos del tubo" con el mismo radio y mínimo de
   observaciones que la estación (``vision.radio_cluster_m`` / ``min_observaciones``);
3. para cada cluster busca el tubo/tramo más cercano de la config ACTUAL (``geo.tubo_cercano``, con las xy
   registradas, no con los tubos de la cabecera) y su desplazamiento perpendicular con signo;
4. por tramo, propone moverlo en perpendicular la mediana ponderada (por observaciones) de los desplazamientos de
   los clusters FRENTE al tramo (un cluster más allá de un extremo no cuenta: su punto más cercano es un vértice) si
   supera ``--umbral`` y hay ``--min-clusters`` clusters (o uno con ``--n-fuerte`` observaciones); los vértices
   compartidos se recalculan como la intersección de los dos tramos desplazados (casi paralelos: el promedio);
   un cluster más allá de un extremo libre de una pieza alarga ese extremo hasta cubrirlo (+ ``--margen``).

Nunca toca ``pista.vertices`` (la ruta de vuelo que compila Simulink). Sin ``--aplicar`` no escribe nada.

Uso (desde la raíz del repo; python del venv de la estación):
    python web/herramientas/ajustar_tubos.py                              # todos los vuelos
    python web/herramientas/ajustar_tubos.py web/datos/vuelos/2026-11-25 --ultimos 2 --svg tubos.svg
    python web/herramientas/ajustar_tubos.py --aplicar                    # reescribe SOLO pista.tubos (+ .bak)
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import shutil
import sys

WEB = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT = os.path.dirname(WEB)
if WEB not in sys.path:
    sys.path.insert(0, WEB)

from app.config import dumps  # noqa: E402  (mismo formato que estacion.json)
from app.geo import tubo_cercano, tubos_pista  # noqa: E402

CARPETA_VUELOS = os.path.join(WEB, "datos", "vuelos")
CONFIG = os.path.join(PROJECT, "config", "estacion.json")
MOTIVO_TUBO = "lejos del tubo"           # = vision_service.MOTIVO_TUBO (sin importar cv2)
SIN_PARALELO = math.sin(math.radians(10.0))   # tramos a menos de 10° entre sí: sin intersección fiable


# =============================================================================== lectura
def archivos_registro(rutas):
    """Lista ordenada de .jsonl a partir de archivos o carpetas (recursivo)."""
    out = []
    for r in rutas:
        if os.path.isdir(r):
            out += glob.glob(os.path.join(r, "**", "*.jsonl"), recursive=True)
        elif os.path.isfile(r):
            out.append(r)
        else:
            print(f"AVISO: no existe {r}", file=sys.stderr)
    return sorted(set(os.path.abspath(p) for p in out))


def leer_vuelo(ruta):
    """{"ruta", "cabecera", "obs", "resumen", "malas"} de un registro JSON Lines (líneas rotas se cuentan)."""
    v = {"ruta": ruta, "cabecera": None, "obs": [], "resumen": None, "malas": 0}
    with open(ruta, encoding="utf-8") as fh:
        for linea in fh:
            linea = linea.strip()
            if not linea:
                continue
            try:
                o = json.loads(linea)
            except ValueError:
                v["malas"] += 1           # p. ej. la última línea de un vuelo cortado a medias
                continue
            tipo = o.get("tipo") if isinstance(o, dict) else None
            if tipo == "cabecera" and v["cabecera"] is None:
                v["cabecera"] = o
            elif tipo == "obs":
                v["obs"].append(o)
            elif tipo == "resumen":
                v["resumen"] = o
    return v


def _clave_orden(v):
    cab = v["cabecera"] or {}
    return (cab.get("fecha") or "", os.path.basename(v["ruta"]))


# =============================================================================== clusters
def agrupar(obs, radio, min_obs):
    """Mismo criterio que ``geo.Clusterer`` (centroide más cercano de la misma clase a <= radio, en orden de
    llegada). Devuelve todos los clusters: {"cls", "x", "y", "n", "n_acept", "n_lejos", "conf", "obs"}."""
    cl = []
    for o in obs:
        x, y = float(o["xy"][0]), float(o["xy"][1])
        best, best_d = None, None
        for c in cl:
            if c["cls"] != o["cls"]:
                continue
            d = math.hypot(c["sx"] / c["n"] - x, c["sy"] / c["n"] - y)
            if d <= radio and (best_d is None or d < best_d):
                best, best_d = c, d
        if best is None:
            best = {"cls": o["cls"], "sx": 0.0, "sy": 0.0, "sconf": 0.0, "n": 0, "n_acept": 0, "n_lejos": 0, "obs": []}
            cl.append(best)
        best["sx"] += x
        best["sy"] += y
        best["sconf"] += float(o.get("conf") or 0.0)
        best["n"] += 1
        best["n_lejos" if o.get("motivo") == MOTIVO_TUBO else "n_acept"] += 1
        best["obs"].append(o)
    out = []
    for c in cl:
        out.append({"cls": c["cls"], "x": c["sx"] / c["n"], "y": c["sy"] / c["n"], "n": c["n"],
                    "n_acept": c["n_acept"], "n_lejos": c["n_lejos"], "conf": c["sconf"] / c["n"],
                    "confirmado": c["n"] >= min_obs, "obs": c["obs"]})
    return out


def obs_utiles(v, clases=(1,)):
    """Tomas aceptadas y rechazadas por lejanía al tubo (con xy), en orden de llegada."""
    return [o for o in v["obs"] if o.get("cls") in clases and o.get("xy") and o.get("motivo") in (None, MOTIVO_TUBO)]


# =============================================================================== geometría
def _unit(dx, dy):
    L = math.hypot(dx, dy)
    return (dx / L, dy / L) if L > 1e-12 else (0.0, 0.0)


def _normal_izq(a, b):
    ux, uy = _unit(b[0] - a[0], b[1] - a[1])
    return -uy, ux


def _interseccion(p1, d1, p2, d2):
    """Intersección de las rectas p1 + s·d1 y p2 + t·d2 (d unitarios). None si son casi paralelas."""
    cruz = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(cruz) < SIN_PARALELO:
        return None
    s = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / cruz
    return [p1[0] + s * d1[0], p1[1] + s * d1[1]]


def mediana_ponderada(valores, pesos):
    pares = sorted(zip(valores, pesos))
    total = float(sum(p for _, p in pares))
    if not pares or total <= 0:
        return None
    acum = 0.0
    for i, (v, p) in enumerate(pares):
        acum += p
        if abs(acum - total / 2.0) < 1e-9 and i + 1 < len(pares):
            return (v + pares[i + 1][0]) / 2.0      # justo a la mitad: promedio de los dos centrales
        if acum > total / 2.0:
            return v
    return pares[-1][0]


def desplazar_pieza(P, offs):
    """Pieza ``P`` (puntos) con cada tramo k movido ``offs[k]`` m en perpendicular (+ = izquierda).
    Vértices compartidos: intersección de los tramos desplazados; casi paralelos (o intersección a más de
    0.5 m + 3·|desplazamiento| del vértice original): promedio de los dos puntos desplazados."""
    n = len(P) - 1
    N = [_normal_izq(P[k], P[k + 1]) for k in range(n)]
    D = [_unit(P[k + 1][0] - P[k][0], P[k + 1][1] - P[k][1]) for k in range(n)]
    out = []
    for i in range(n + 1):
        if i == 0:
            o, nn = offs[0], N[0]
            out.append([P[0][0] + o * nn[0], P[0][1] + o * nn[1]])
            continue
        if i == n:
            o, nn = offs[n - 1], N[n - 1]
            out.append([P[n][0] + o * nn[0], P[n][1] + o * nn[1]])
            continue
        oa, ob = offs[i - 1], offs[i]
        if oa == 0 and ob == 0:
            out.append(list(P[i]))
            continue
        qa = [P[i][0] + oa * N[i - 1][0], P[i][1] + oa * N[i - 1][1]]
        qb = [P[i][0] + ob * N[i][0], P[i][1] + ob * N[i][1]]
        q = _interseccion(qa, D[i - 1], qb, D[i])
        if q is None or math.hypot(q[0] - P[i][0], q[1] - P[i][1]) > 0.5 + 3 * max(abs(oa), abs(ob)):
            q = [(qa[0] + qb[0]) / 2.0, (qa[1] + qb[1]) / 2.0]
        out.append(q)
    return out


def proponer(tubos, clusters, umbral=0.05, min_clusters=2, n_fuerte=8, max_desp=0.6, margen=0.05):
    """Tubos propuestos a partir de los clusters confirmados de tomas (con su ``tc`` = ``tubo_cercano`` contra
    ``tubos``). Devuelve (tubos nuevos, decisiones por tramo, extensiones, clusters ignorados)."""
    por_tramo, ignorados = {}, []
    for c in clusters:
        tc = c["tc"]
        if tc is None or tc["dist"] > max_desp:
            ignorados.append(c)
            continue
        por_tramo.setdefault((tc["pieza"], tc["tramo"]), []).append(c)
    decisiones = []
    offs = [[0.0] * (len(P) - 1) for P in tubos]
    for (i, k), cs in sorted(por_tramo.items()):
        # solo cuentan para el desplazamiento lateral los clusters frente al tramo (0 <= u <= 1). Uno más allá de
        # un extremo tiene como punto más cercano un VÉRTICE: junto a un vértice compartido empata con el tramo
        # vecino (gana el primero) y su "desplazamiento" (respecto a la recta infinita) no mide este tramo; junto
        # a un extremo libre solo alarga el tubo (abajo).
        lat = [c for c in cs if 0.0 <= c["tc"]["u"] <= 1.0]
        med = mediana_ponderada([c["tc"]["desplazamiento"] for c in lat], [c["n"] for c in lat]) if lat else None
        evidencia = bool(lat) and (len(lat) >= min_clusters or max(c["n"] for c in lat) >= n_fuerte)
        aplica = med is not None and abs(med) >= umbral and evidencia
        if aplica:
            offs[i][k] = med
        motivo = ("se mueve" if aplica else
                  "solo clusters más allá de un extremo del tramo (no cuentan para moverlo)" if med is None else
                  f"|{med:+.3f}| < umbral {umbral:g} m" if abs(med) < umbral else
                  f"poca evidencia ({len(lat)} cluster(s) < {min_clusters} y ninguno con >= {n_fuerte} obs.)")
        decisiones.append({"pieza": i, "tramo": k, "clusters": cs, "mediana": med, "aplica": aplica, "motivo": motivo})
    nuevos = [desplazar_pieza(P, offs[i]) for i, P in enumerate(tubos)]
    # extremos libres: un cluster más allá del principio del primer tramo o del final del último
    extensiones = []
    for (i, k), cs in sorted(por_tramo.items()):
        P = nuevos[i]
        n = len(P) - 1
        for c in cs:
            u = c["tc"]["u"]
            if k == 0 and u < 0:
                a, b, extremo = P[1], P[0], 0          # se alarga hacia atrás desde el vértice 1
            elif k == n - 1 and u > 1:
                a, b, extremo = P[n - 1], P[n], n
            else:
                continue
            dx, dy = _unit(b[0] - a[0], b[1] - a[1])
            largo = math.hypot(b[0] - a[0], b[1] - a[1])
            proy = (c["x"] - a[0]) * dx + (c["y"] - a[1]) * dy     # a lo largo del tramo ya desplazado
            if proy + margen > largo:
                P[extremo] = [a[0] + (proy + margen) * dx, a[1] + (proy + margen) * dy]
                extensiones.append({"pieza": i, "extremo": extremo, "cluster": c, "alargue_m": proy + margen - largo})
    nuevos = [[[round(v[0], 2) + 0.0, round(v[1], 2) + 0.0] for v in P] for P in nuevos]
    return nuevos, decisiones, extensiones, ignorados


# =============================================================================== config
def _ws(t, i):
    while i < len(t) and t[i] in " \t\r\n":
        i += 1
    return i


def _fin_cadena(t, i):
    i += 1
    while t[i] != '"':
        i += 2 if t[i] == "\\" else 1
    return i + 1


def _fin_valor(t, i):
    if t[i] == '"':
        return _fin_cadena(t, i)
    if t[i] in "[{":
        prof = 0
        while True:
            c = t[i]
            if c == '"':
                i = _fin_cadena(t, i)
                continue
            if c in "[{":
                prof += 1
            elif c in "]}":
                prof -= 1
                if prof == 0:
                    return i + 1
            i += 1
    while i < len(t) and t[i] not in ",]} \t\r\n":
        i += 1
    return i


def _miembros(t, i):
    """Miembros del objeto JSON que empieza en ``t[i] == '{'``: [(clave, ini_clave, ini_valor, fin_valor)]."""
    out = []
    i = _ws(t, i + 1)
    while t[i] != "}":
        k0 = i
        k1 = _fin_cadena(t, i)
        i = _ws(t, k1)
        if t[i] != ":":
            raise ValueError(f"se esperaba ':' en la posición {i}")
        v0 = _ws(t, i + 1)
        v1 = _fin_valor(t, v0)
        out.append((json.loads(t[k0:k1]), k0, v0, v1))
        i = _ws(t, v1)
        if t[i] == ",":
            i = _ws(t, i + 1)
    return out


def reemplazar_tubos(texto, tubos):
    """``texto`` de estacion.json con SOLO el arreglo ``pista.tubos`` cambiado (o agregado al final de ``pista``),
    con el formato de ``config.dumps`` (una línea por pieza) y los finales de línea del archivo."""
    nl = "\r\n" if "\r\n" in texto else "\n"
    raiz = texto.index("{")
    pista = [m for m in _miembros(texto, raiz) if m[0] == "pista"]
    if not pista or texto[pista[0][2]] != "{":
        raise ValueError("estacion.json no tiene la sección pista")
    miembros = _miembros(texto, pista[0][2])
    if not miembros:
        raise ValueError("la sección pista está vacía")
    tub = [m for m in miembros if m[0] == "tubos"]
    ref = tub[0] if tub else miembros[-1]
    col = ref[1] - (texto.rfind("\n", 0, ref[1]) + 1)          # columna de la clave (sangría)
    valor = dumps(tubos, indent=col // 2).replace("\n", nl)
    if tub:
        _, _, v0, v1 = tub[0]
        nuevo = texto[:v0] + valor + texto[v1:]
    else:
        v1 = miembros[-1][3]
        nuevo = texto[:v1] + "," + nl + " " * col + '"tubos": ' + valor + texto[v1:]
    a, b = json.loads(texto.lstrip("﻿")), json.loads(nuevo.lstrip("﻿"))
    a["pista"]["tubos"] = b["pista"].get("tubos")
    if a != b or b["pista"]["tubos"] != json.loads(json.dumps(tubos)):
        raise ValueError("el reemplazo cambiaría algo más que pista.tubos: no se escribe")
    return nuevo


def aplicar(ruta_cfg, tubos):
    """Reescribe SOLO pista.tubos de ``ruta_cfg`` (respaldo ``.bak`` con el original byte a byte)."""
    with open(ruta_cfg, "rb") as fh:
        crudo = fh.read()
    nuevo = reemplazar_tubos(crudo.decode("utf-8"), tubos).encode("utf-8")
    shutil.copyfile(ruta_cfg, ruta_cfg + ".bak")
    tmp = ruta_cfg + ".tmp"
    with open(tmp, "wb") as fh:
        fh.write(nuevo)
    os.replace(tmp, ruta_cfg)
    return ruta_cfg + ".bak"


# =============================================================================== SVG
def svg(ruta, cfg, tubos, nuevos, vuelos_obs, clusters):
    """Dibujo cenital (x a la derecha, y hacia arriba, como el mapa 2D): ruta, tubos actuales y propuestos,
    tomas aceptadas / rechazadas por lejanía y clusters."""
    pista = cfg.get("pista") or {}
    lim = (cfg.get("mision") or {}).get("limites_xy") or [-1.5, 1.5]
    xs = [lim[0], lim[1]] + [p[0] for P in tubos + nuevos for p in P] + [o["xy"][0] for o in vuelos_obs]
    ys = [lim[0], lim[1]] + [p[1] for P in tubos + nuevos for p in P] + [o["xy"][1] for o in vuelos_obs]
    x0, x1 = math.floor(min(xs) * 2) / 2 - 0.25, math.ceil(max(xs) * 2) / 2 + 0.25
    y0, y1 = math.floor(min(ys) * 2) / 2 - 0.25, math.ceil(max(ys) * 2) / 2 + 0.25
    S, M = 200.0, 30.0
    W, H = (x1 - x0) * S + 2 * M, (y1 - y0) * S + 2 * M + 60

    def px(x, y):
        return f"{(x - x0) * S + M:.1f},{(y1 - y) * S + M:.1f}"

    def linea(P, attrs):
        return f'<polyline points="{" ".join(px(*p) for p in P)}" fill="none" {attrs}/>'

    e = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{H:.0f}" viewBox="0 0 {W:.0f} {H:.0f}" '
         'font-family="sans-serif" font-size="11">', f'<rect width="{W:.0f}" height="{H:.0f}" fill="#fff"/>']
    g = math.ceil(x0 * 2) / 2
    while g <= x1 + 1e-9:
        e.append(f'<line x1="{(g - x0) * S + M:.1f}" y1="{M:.1f}" x2="{(g - x0) * S + M:.1f}" y2="{(y1 - y0) * S + M:.1f}" '
                 f'stroke="{"#bbb" if abs(g) < 1e-9 else "#eee"}"/>')
        e.append(f'<text x="{(g - x0) * S + M:.1f}" y="{(y1 - y0) * S + M + 14:.1f}" text-anchor="middle" fill="#888">{g:g}</text>')
        g += 0.5
    g = math.ceil(y0 * 2) / 2
    while g <= y1 + 1e-9:
        e.append(f'<line x1="{M:.1f}" y1="{(y1 - g) * S + M:.1f}" x2="{(x1 - x0) * S + M:.1f}" y2="{(y1 - g) * S + M:.1f}" '
                 f'stroke="{"#bbb" if abs(g) < 1e-9 else "#eee"}"/>')
        e.append(f'<text x="{M - 4:.1f}" y="{(y1 - g) * S + M + 4:.1f}" text-anchor="end" fill="#888">{g:g}</text>')
        g += 0.5
    e.append(linea([[lim[0], lim[0]], [lim[1], lim[0]], [lim[1], lim[1]], [lim[0], lim[1]], [lim[0], lim[0]]],
                   'stroke="#999" stroke-dasharray="6 4"'))
    if pista.get("vertices"):
        e.append(linea(pista["vertices"], 'stroke="#7a7a7a" stroke-width="1.5" stroke-dasharray="3 3"'))
    for P in tubos:
        e.append(linea(P, 'stroke="#9aa3ad" stroke-width="8" stroke-linecap="round" stroke-linejoin="round" opacity="0.6"'))
    for P in nuevos:
        e.append(linea(P, 'stroke="#1a9b4b" stroke-width="3" stroke-linejoin="round"'))
    for o in vuelos_obs:
        color = "#d62728" if o.get("motivo") == MOTIVO_TUBO else "#1f77b4"
        x, y = px(*o["xy"]).split(",")
        e.append(f'<circle cx="{x}" cy="{y}" r="2.2" fill="{color}" opacity="0.7"/>')
    for c in clusters:
        x, y = px(c["x"], c["y"]).split(",")
        e.append(f'<circle cx="{x}" cy="{y}" r="9" fill="none" stroke="{"#d62728" if c["n_lejos"] else "#1f77b4"}" '
                 f'stroke-width="1.5"/><text x="{float(x) + 11:.1f}" y="{float(y) - 6:.1f}">{c["n"]}</text>')
    ly = (y1 - y0) * S + M + 36
    ley = [("#9aa3ad", "tubos actuales"), ("#1a9b4b", "tubos propuestos"), ("#7a7a7a", "ruta de vuelo"),
           ("#1f77b4", "toma aceptada"), ("#d62728", "toma rechazada: lejos del tubo")]
    xl = M
    for color, txt in ley:
        e.append(f'<rect x="{xl:.0f}" y="{ly - 9:.0f}" width="14" height="10" fill="{color}"/>'
                 f'<text x="{xl + 18:.0f}" y="{ly:.0f}">{txt}</text>')
        xl += 30 + 6.2 * len(txt)
    e.append("</svg>")
    with open(ruta, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(e) + "\n")


# =============================================================================== reporte
def _tubos_iguales(a, b, tol=0.005):
    if a is None or b is None or len(a) != len(b):
        return False
    for P, Q in zip(a, b):
        if len(P) != len(Q) or any(abs(p[0] - q[0]) > tol or abs(p[1] - q[1]) > tol for p, q in zip(P, Q)):
            return False
    return True


def _rel(ruta):
    try:
        r = os.path.relpath(ruta, PROJECT)
        return r.replace(os.sep, "/") if not r.startswith("..") else ruta
    except ValueError:
        return ruta


def ejecutar(args, out=print):
    with open(args.config, encoding="utf-8-sig") as fh:
        cfg = json.load(fh)
    vis = cfg.get("vision") or {}
    radio = float(args.radio if args.radio is not None else vis.get("radio_cluster_m", 0.25) or 0.25)
    min_obs = int(args.min_obs if args.min_obs is not None else vis.get("min_observaciones", 3) or 3)
    tubos = tubos_pista(cfg.get("pista"))
    if not tubos:
        out("ERROR: la config no tiene tubos (ni pista.tubos ni pista.vertices).")
        return 2
    rutas = archivos_registro(args.registros or [CARPETA_VUELOS])
    vuelos = sorted((leer_vuelo(r) for r in rutas), key=_clave_orden)
    if args.ultimos:
        vuelos = vuelos[-args.ultimos:]
    if not vuelos:
        out(f"No hay registros de vuelo en {', '.join(_rel(r) for r in (args.registros or [CARPETA_VUELOS]))}.")
        return 1
    out(f"Config: {_rel(args.config)}  ·  radio de cluster {radio:g} m, mínimo {min_obs} obs.  ·  {len(vuelos)} vuelo(s)")
    # ---------------------------------------------------------------- tubos de las cabeceras
    cabs = [(v, (v["cabecera"] or {}).get("pista", {}).get("tubos")) for v in vuelos]
    distintos = {json.dumps(t) for _, t in cabs if t is not None}
    if len(distintos) > 1:
        out("AVISO: los vuelos se registraron con tubos distintos entre sí (la config cambió entre vuelos):")
        for v, t in cabs:
            out(f"    {os.path.basename(v['ruta'])}: {json.dumps(t)}")
    if any(t is not None and not _tubos_iguales(t, tubos) for _, t in cabs):
        out("AVISO: hay vuelos registrados con tubos distintos de los de la config actual; los desplazamientos se "
            "calculan contra la config actual con las xy registradas.")
    # ---------------------------------------------------------------- por vuelo
    todos, todas_obs = [], []
    for v in vuelos:
        cab = v["cabecera"] or {}
        obs = obs_utiles(v)
        todas_obs += obs
        cl = agrupar(obs, radio, min_obs)
        conf = [c for c in cl if c["confirmado"]]
        for c in conf:
            c["tc"] = tubo_cercano(c["x"], c["y"], tubos)
            c["vuelo"] = os.path.basename(v["ruta"])
        todos += conf
        n_lejos = sum(1 for o in obs if o.get("motivo") == MOTIVO_TUBO)
        out("")
        out(f"== {_rel(v['ruta'])}  ({cab.get('fecha', '?')}, fuente {cab.get('fuente', '?')}, "
            f"{len(v['obs'])} obs. registradas{', %d líneas rotas' % v['malas'] if v['malas'] else ''}"
            f"{'' if v['resumen'] else ', SIN resumen (vuelo cortado)'})")
        out(f"   tomas: {len(obs) - n_lejos} aceptadas, {n_lejos} lejos del tubo -> {len(conf)} cluster(s) confirmados"
            f" ({len(cl) - len(conf)} con < {min_obs} obs. no cuentan)")
        for c in conf:
            tc = c["tc"]
            marca = "LEJOS" if c["n_lejos"] else "ok   "
            out(f"   {marca} ({c['x']:+.2f}, {c['y']:+.2f})  n={c['n']} ({c['n_acept']} acept., {c['n_lejos']} lejos)"
                f"  tubo {tc['pieza'] + 1} tramo {tc['tramo'] + 1}: dist {tc['dist']:.2f} m, desplazamiento "
                f"{tc['desplazamiento']:+.2f} m{'' if 0 <= tc['u'] <= 1 else ' (más allá del extremo, u = %.2f)' % tc['u']}")
    # ---------------------------------------------------------------- propuesta
    nuevos, decisiones, extensiones, ignorados = proponer(
        tubos, todos, umbral=args.umbral, min_clusters=args.min_clusters, n_fuerte=args.n_fuerte,
        max_desp=args.max_desp, margen=args.margen)
    out("")
    out("== Por tubo / tramo (desplazamiento + = a la izquierda del sentido del tramo)")
    if not decisiones:
        out("   sin clusters de tomas confirmados: no hay nada que proponer")
    for d in decisiones:
        P = tubos[d["pieza"]]
        a, b = P[d["tramo"]], P[d["tramo"] + 1]
        des = ", ".join(f"{c['tc']['desplazamiento']:+.2f}({c['n']}"
                        f"{'' if 0 <= c['tc']['u'] <= 1 else ', extremo: no cuenta'})" for c in d["clusters"])
        med = "-" if d["mediana"] is None else f"{d['mediana']:+.3f} m"
        out(f"   tubo {d['pieza'] + 1} tramo {d['tramo'] + 1} ({a[0]:+.2f},{a[1]:+.2f})->({b[0]:+.2f},{b[1]:+.2f}): "
            f"{len(d['clusters'])} cluster(s) [{des}] -> mediana {med}: {d['motivo']}")
    for x in extensiones:
        c = x["cluster"]
        out(f"   tubo {x['pieza'] + 1}: se alarga el extremo {x['extremo'] + 1} {x['alargue_m']:.2f} m para cubrir el "
            f"cluster ({c['x']:+.2f}, {c['y']:+.2f})")
    for c in ignorados:
        out(f"   ignorado: cluster ({c['x']:+.2f}, {c['y']:+.2f}) n={c['n']} a "
            f"{c['tc']['dist'] if c['tc'] else float('nan'):.2f} m de todo tubo (> {args.max_desp:g} m: ¿otro objeto?)")
    cambia = not _tubos_iguales(nuevos, tubos, tol=1e-9)
    out("")
    out("pista.tubos actual:    " + json.dumps(tubos))
    if not cambia:
        out("pista.tubos propuesto: (sin cambios)")
    else:
        out("pista.tubos propuesto (para la sección pista de estacion.json, o usa --aplicar):")
        out('    "tubos": ' + dumps(nuevos, indent=2))
    lim = (cfg.get("mision") or {}).get("limites_xy") or [-1.5, 1.5]
    fuera = [p for P in nuevos for p in P if not (lim[0] - 0.3 <= p[0] <= lim[1] + 0.3 and lim[0] - 0.3 <= p[1] <= lim[1] + 0.3)]
    if fuera:
        out(f"AVISO: {len(fuera)} punto(s) propuesto(s) a más de 0.3 m fuera de mision.limites_xy {lim}: revísalos.")
    out("pista.vertices (ruta de vuelo) NO se toca.")
    if args.svg:
        svg(args.svg, cfg, tubos, nuevos, todas_obs, todos)
        out(f"SVG: {args.svg}")
    if args.aplicar:
        if not cambia:
            out("--aplicar: no hay cambios que escribir.")
        else:
            bak = aplicar(args.config, nuevos)
            out(f"--aplicar: pista.tubos reescrito en {_rel(args.config)} (respaldo {_rel(bak)}).")
    return 0


def build_parser():
    ap = argparse.ArgumentParser(description="Propone pista.tubos a partir de los registros de vuelo (web/datos/vuelos)")
    ap.add_argument("registros", nargs="*", help="archivos .jsonl o carpetas (por defecto web/datos/vuelos)")
    ap.add_argument("--config", default=CONFIG, help="estacion.json (por defecto config/estacion.json)")
    ap.add_argument("--ultimos", type=int, default=0, help="solo los N vuelos más recientes")
    ap.add_argument("--radio", type=float, default=None, help="radio de cluster (m; por defecto vision.radio_cluster_m)")
    ap.add_argument("--min-obs", type=int, default=None, help="obs. por cluster (por defecto vision.min_observaciones)")
    ap.add_argument("--umbral", type=float, default=0.05, help="desplazamiento mínimo para mover un tramo (m)")
    ap.add_argument("--min-clusters", type=int, default=2, help="clusters por tramo para moverlo")
    ap.add_argument("--n-fuerte", type=int, default=8, help="...o un cluster con al menos estas observaciones")
    ap.add_argument("--max-desp", type=float, default=0.6, help="clusters más lejos que esto de todo tubo se ignoran (m)")
    ap.add_argument("--margen", type=float, default=0.05, help="al alargar un extremo, cuánto pasar del cluster (m)")
    ap.add_argument("--svg", help="dibuja tubos actuales vs propuestos y las tomas en este SVG")
    ap.add_argument("--aplicar", action="store_true", help="reescribe SOLO pista.tubos de --config (+ respaldo .bak)")
    return ap


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8")     # consola de Windows (cp1252) con acentos
        except (AttributeError, ValueError):
            pass
    return ejecutar(build_parser().parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
