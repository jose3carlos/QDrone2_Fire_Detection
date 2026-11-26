"""Analiza la caja negra de un vuelo (``web/datos/vuelos/**/*.tel.jsonl.gz``, ``app/caja_negra.py``).

Pensada para correr después de CADA vuelo (en la PC del laboratorio o en otra sesión, tras ``git pull``):

* ``resumen``: línea de tiempo de los estados de misión, de los Flight States / FSM de cada dron y de las alertas;
  comandos con su resultado y latencia hasta el ack (del puente y de la telemetría, y(54)); pérdidas de rastreo;
  máximos |x|, |y| y z por dron; separación horizontal mínima en el aire (dual); tiempo en el aire; huecos de
  telemetría > ``--hueco`` s; avisos que aparecieron; video (fps, edad del último frame, caídas y reconexiones del
  enlace de video). ``--json`` = lo mismo en JSON.
* ``csv``: exporta la telemetría (una fila por paquete: ``t`` + los 64 campos con su nombre de ``protocol.py``) y los
  comandos / acks / eventos / avisos a CSV para MATLAB (``readtable``; NaN = sin dato).

Uso (desde la raíz del repo; python del venv de la estación):
    python web/herramientas/caja_negra.py resumen                       # la caja negra más reciente
    python web/herramientas/caja_negra.py resumen web/datos/vuelos/2026-11-25          # la más reciente de ese día
    python web/herramientas/caja_negra.py resumen <archivo.tel.jsonl.gz> --json
    python web/herramientas/caja_negra.py csv <archivo.tel.jsonl.gz>    # <nombre>.tel.csv + <nombre>.tel_eventos.csv
    (en la carpeta actual; git los ignora por el nombre. Con ``-o`` usa una ruta fuera del repo, p. ej. %TEMP%\\vuelo.csv)

Para VER el vuelo en la UI: ``cd web && python -m app.server --repetir <archivo> --puerto 8081``.
Solo lee: no escribe nada salvo los CSV que se piden.
"""
from __future__ import annotations

import argparse
import csv
import glob
import io
import json
import math
import os
import sys

WEB = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if WEB not in sys.path:
    sys.path.insert(0, WEB)

from app import protocol as P  # noqa: E402
from app.caja_negra import EXT, _VEC, nombres_y  # noqa: E402
from app.repeticion import Grabacion  # noqa: E402

CARPETA_VUELOS = os.path.join(WEB, "datos", "vuelos")
Z_AIRE = 0.2        # m: un dron rastreado por encima de esto está "en el aire"
DT_MAX = 0.5        # s: un hueco de telemetría más largo no suma tiempo en el aire


# =============================================================================== lectura
def buscar(ruta=None):
    """Archivo de caja negra: ``ruta`` si es un archivo; si es carpeta (o None = web/datos/vuelos), el más reciente
    dentro (por fecha y hora del nombre)."""
    if ruta and os.path.isfile(ruta):
        return os.path.abspath(ruta)
    carpeta = ruta or CARPETA_VUELOS
    if not os.path.isdir(carpeta):
        raise FileNotFoundError(f"no existe {carpeta}")
    cands = glob.glob(os.path.join(carpeta, "**", "*" + EXT), recursive=True)
    if not cands:
        raise FileNotFoundError(f"no hay cajas negras (*{EXT}) en {carpeta}")
    return os.path.abspath(max(cands, key=lambda p: (os.path.basename(os.path.dirname(p)), os.path.basename(p))))


def _fin(v):
    return v is not None and isinstance(v, (int, float)) and math.isfinite(v)


# =============================================================================== análisis
def analizar(grab, z_aire=Z_AIRE, hueco_s=0.2):
    """Resumen de una ``repeticion.Grabacion`` (dict listo para JSON)."""
    cab = grab.cabecera
    tels = [(t, P.unpack_telemetry(raw)) for t, raw in grab.tel]
    dur = (grab.t_fin - grab.t_ini) if tels else 0.0
    tam = os.path.getsize(grab.ruta)
    res = {"archivo": grab.ruta, "fecha": cab.get("fecha"), "fuente": cab.get("fuente"),
           "modo_mision": cab.get("modo_mision"), "dron_solo": cab.get("dron_solo"), "config": cab.get("config"),
           "motivo_apertura": cab.get("motivo_apertura"), "motivo_cierre": (grab.fin or {}).get("motivo"),
           "cortado": grab.fin is None or bool(grab.info.get("truncado")), "lectura": dict(grab.info),
           "decimacion": cab.get("decimacion", 1), "n_tel": len(tels), "t_ini": round(grab.t_ini, 2),
           "t_fin": round(grab.t_fin, 2), "duracion_s": round(dur, 2),
           "hz_medio": round((len(tels) - 1) / dur, 1) if dur > 0 else None,
           "mb": round(tam / 1e6, 3), "mb_por_10_min": round(tam / 1e6 * 600 / dur, 2) if dur > 0 else None}
    ds = cab.get("dron_solo") if cab.get("dron_solo") in (1, 2) else 0
    if tels and tels[-1][1].get("dron_solo") in (1, 2):
        ds = tels[-1][1]["dron_solo"]
    activos = (ds,) if ds else (1, 2)
    res["drones_activos"] = list(activos)

    # ---- líneas de tiempo (cambios)
    estados, fsm, alertas, explorador = [], {1: [], 2: []}, [], []
    prev_e = prev_a = prev_x = None
    prev_f = {1: None, 2: None}
    for t, tel in tels:
        e = tel["mission_state"]
        if e != prev_e:
            estados.append({"t": round(t, 2), "hora": grab.hora(t), "estado": tel["mission_state_name"],
                            "es": tel["mission_state_es"]})
            prev_e = e
        a = tel["alerta"]
        if a != prev_a:
            alertas.append({"t": round(t, 2), "alerta": a, "texto": tel["alerta_texto"] if a else "despejada"})
            prev_a = a
        x = tel["explorer_id"]
        if x != prev_x:
            explorador.append({"t": round(t, 2), "explorador": x})
            prev_x = x
        for d in (1, 2):
            dd = tel["drones"][d]
            if dd["fsm_state"] != prev_f[d]:
                fsm[d].append({"t": round(t, 2), "fsm": dd["fsm_state"], "texto": dd["fsm_texto"],
                               "falla": bool(dd["fsm_falla"])})
                prev_f[d] = dd["fsm_state"]
    res["estados"], res["alertas"], res["explorador"] = estados, alertas, explorador
    res["flight_states"] = {str(d): fsm[d] for d in activos}

    # ---- comandos: resultado en la estación, ack del puente y ack en la telemetría (y(54))
    cmds = []
    acks = list(grab.acks)
    for c in grab.cmds:
        fila = {"t": round(c.get("t", 0.0), 2), "hora": grab.hora(c.get("t", 0.0)), "cmd": c.get("cmd"),
                "origen": c.get("origen"), "resultado": c.get("resultado"), "cmd_seq": c.get("cmd_seq"),
                "detalle": c.get("detalle") or ""}
        if c.get("resultado") in ("enviado", "en cola"):
            ack = next((a for a in acks if a.get("cmd") == c.get("cmd") and a.get("t", 0.0) >= c.get("t", 0.0) - 1e-3
                        and (c.get("cmd_seq") is None or a.get("cmd_seq") == c.get("cmd_seq"))), None)
            if ack is not None:
                acks.remove(ack)
                fila["ack"] = ack.get("estado")
                fila["cmd_seq"] = fila["cmd_seq"] or ack.get("cmd_seq")
                fila["latencia_ack_s"] = ack.get("latencia_s") if ack.get("latencia_s") is not None else \
                    round(ack.get("t", 0.0) - c.get("t", 0.0), 3)
                if ack.get("estado") != "recibido":
                    fila["detalle"] = ack.get("detalle") or fila["detalle"]
            else:
                fila["ack"] = None
            seq = fila["cmd_seq"]
            if seq:
                t_tel = next((t for t, tel in tels if t >= c.get("t", 0.0) and tel["cmd_seq_ack"] == seq), None)
                fila["latencia_tel_s"] = round(t_tel - c.get("t", 0.0), 3) if t_tel is not None else None
        cmds.append(fila)
    res["comandos"] = cmds
    res["acks_sin_comando"] = [{"t": round(a.get("t", 0.0), 2), "cmd": a.get("cmd"), "estado": a.get("estado"),
                                "detalle": a.get("detalle")} for a in acks]

    # ---- por dron: rastreo, máximos, tiempo en el aire
    por_dron = {}
    for d in activos:
        perdidas, ini, visto, z_prev = [], None, False, None
        mx = {"max_abs_x": None, "max_abs_y": None, "max_z": None}
        aire = 0.0
        t_prev = None
        for t, tel in tels:
            dd = tel["drones"][d]
            ok = bool(dd["tracked"]) and all(_fin(dd[k]) for k in ("x", "y", "z"))
            if ok:
                visto = True
                for k, v in (("max_abs_x", abs(dd["x"])), ("max_abs_y", abs(dd["y"])), ("max_z", dd["z"])):
                    if mx[k] is None or v > mx[k][0]:
                        mx[k] = (v, t)
                if ini is not None:
                    perdidas[-1].update(t_fin=round(t, 2), dur_s=round(t - ini, 2))
                    ini = None
                if dd["z"] > z_aire and t_prev is not None:
                    aire += min(t - t_prev, DT_MAX)
                z_prev = dd["z"]
            elif visto and ini is None:
                ini = t
                perdidas.append({"t": round(t, 2), "hora": grab.hora(t), "t_fin": None, "dur_s": None,
                                 "en_el_aire": bool(z_prev is not None and z_prev > z_aire),
                                 "estado": tel["mission_state_name"]})
            t_prev = t
        if ini is not None and tels:
            perdidas[-1]["dur_s"] = round(tels[-1][0] - ini, 2)
            perdidas[-1]["hasta_el_final"] = True
        por_dron[str(d)] = {"perdidas_rastreo": perdidas, "tiempo_en_el_aire_s": round(aire, 1),
                            **{k: ({"valor": round(v[0], 3), "t": round(v[1], 2)} if v else None) for k, v in mx.items()}}
    res["drones"] = por_dron

    # ---- separación horizontal mínima con los dos en el aire (solo dual)
    sep = None
    if not ds:
        for t, tel in tels:
            a, b = tel["drones"][1], tel["drones"][2]
            if not (a["tracked"] and b["tracked"]):
                continue
            if not all(_fin(v) for v in (a["x"], a["y"], a["z"], b["x"], b["y"], b["z"])):
                continue
            if a["z"] > z_aire and b["z"] > z_aire:
                dh = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
                if sep is None or dh < sep["m"]:
                    sep = {"m": round(dh, 3), "t": round(t, 2), "hora": grab.hora(t), "estado": tel["mission_state_name"]}
    res["separacion_min_aire"] = sep if not ds else "no aplica (solo explorador)"

    # ---- huecos de telemetría
    huecos = []
    for (t0, _), (t1, _) in zip(tels, tels[1:]):
        if t1 - t0 > hueco_s:
            huecos.append({"t": round(t0, 2), "hora": grab.hora(t0), "dur_s": round(t1 - t0, 3)})
    res["huecos"] = {"umbral_s": hueco_s, "n": len(huecos), "max_s": max((h["dur_s"] for h in huecos), default=0.0),
                     "lista": huecos[:50]}

    # ---- avisos, eventos y video (líneas "muestra")
    res["avisos"] = [{"t": round(a.get("t", 0.0), 2), "accion": a.get("accion"), "nivel": a.get("nivel"),
                      "msg": a.get("msg")} for a in grab.avisos]
    res["n_eventos"] = len(grab.eventos)
    fps = [m["camara"].get("fps") for m in grab.muestras if isinstance(m.get("camara"), dict)
           and _fin(m["camara"].get("fps"))]
    edad = [m["camara"].get("edad_s") for m in grab.muestras if isinstance(m.get("camara"), dict)
            and _fin(m["camara"].get("edad_s"))]
    res["video"] = {"muestras": len(grab.muestras), "fps_min": min(fps, default=None), "fps_max": max(fps, default=None),
                    "edad_max_s": max(edad, default=None)}
    # contadores del enlace de video (cameras.CameraHub; grabaciones anteriores no los traen -> None). Son acumulados
    # desde que la estación abrió la fuente: se reporta el aumento dentro de la grabación y el último valor del vuelo.
    cams = [m["camara"] for m in grab.muestras if isinstance(m.get("camara"), dict) and "caidas" in m["camara"]]
    if cams:
        def _delta(k):
            vals = [c.get(k) for c in cams if _fin(c.get(k))]
            return (max(vals) - vals[0]) if vals else None
        res["video"].update({"caidas": _delta("caidas"), "reconexiones": _delta("reconexiones"),
                             "atascos": _delta("atascos"), "caidas_vuelo": cams[-1].get("caidas_vuelo"),
                             "caida_max_s": max((c["caida_s"] for c in cams if _fin(c.get("caida_s"))), default=None)})
    return res


# =============================================================================== salida
def _f(v, d=2, u=""):
    return "—" if v is None else f"{v:.{d}f}{u}"


def imprimir(r, out=None):
    out = out or sys.stdout
    w = lambda s="": print(s, file=out)   # noqa: E731
    w(f"Caja negra: {r['archivo']}")
    w(f"  {r['fecha']} · fuente {r['fuente']} · {r['modo_mision']}"
      + (f" (dron {r['dron_solo']})" if r.get("dron_solo") else "") + f" · config {r['config']}")
    w(f"  abierta por: {r['motivo_apertura']} · cerrada: {r['motivo_cierre'] or 'NO (archivo cortado)'}"
      + (" · ARCHIVO CORTADO (se leyó hasta el último vaciado)" if r["cortado"] else ""))
    w(f"  {r['n_tel']} paquetes en {_f(r['duracion_s'], 1, ' s')} (t {r['t_ini']} .. {r['t_fin']}) · "
      f"{_f(r['hz_medio'], 1, ' Hz')} · decimación {r['decimacion']} · {r['mb']} MB "
      f"(≈ {_f(r['mb_por_10_min'], 1, ' MB')} por 10 min)")
    if r["lectura"].get("danadas") or r["lectura"].get("basura_bytes"):
        w(f"  lectura: {r['lectura']['danadas']} líneas dañadas, {r['lectura']['basura_bytes']} bytes ilegibles")
    w()
    w("Estados de misión (t = s desde la apertura; negativo = antes del START):")
    for e in r["estados"]:
        w(f"  {e['t']:>8.2f}  {e['hora']}  {e['estado']:<13} {e['es']}")
    for d, lista in r["flight_states"].items():
        w(f"Dron {d}: Flight State / FSM")
        for f in lista:
            w(f"  {f['t']:>8.2f}  {f['fsm']:>2}  {f['texto']}" + ("  <-- FALLA" if f["falla"] else ""))
    if len(r["alertas"]) > 1 or (r["alertas"] and r["alertas"][0]["alerta"]):
        w("Alertas y(55):")
        for a in r["alertas"]:
            w(f"  {a['t']:>8.2f}  {a['alerta']:>2}  {a['texto']}")
    if len(r["explorador"]) > 1:
        w("Explorador y(4): " + ", ".join(f"{x['explorador']} desde {x['t']}" for x in r["explorador"]))
    w()
    w("Comandos:")
    if not r["comandos"]:
        w("  (ninguno)")
    for c in r["comandos"]:
        txt = f"  {c['t']:>8.2f}  {c['hora']}  {c['cmd']:<8} {c['resultado']:<8} cmd_seq {c['cmd_seq'] or '—'}"
        if "ack" in c:
            txt += (f" · ack {c['ack'] or 'SIN ACK'} {_f(c.get('latencia_ack_s'), 3, ' s')}"
                    f" · y(54) {_f(c.get('latencia_tel_s'), 3, ' s')}")
        txt += f" · {c['origen']}"
        w(txt)
        if c["detalle"] and (c["resultado"] not in ("enviado", "en cola") or c.get("ack") != "recibido"):
            w(f"            {c['detalle']}")
    for a in r["acks_sin_comando"]:
        w(f"  {a['t']:>8.2f}  {a['cmd']:<8} {a['estado']} (sin pedido grabado: {a['detalle']})")
    w()
    for d, info in r["drones"].items():
        w(f"Dron {d}: en el aire {info['tiempo_en_el_aire_s']} s · máx |x| {_f((info['max_abs_x'] or {}).get('valor'), 3, ' m')}"
          f" · máx |y| {_f((info['max_abs_y'] or {}).get('valor'), 3, ' m')} · máx z {_f((info['max_z'] or {}).get('valor'), 3, ' m')}")
        if not info["perdidas_rastreo"]:
            w("  sin pérdidas de rastreo")
        for p in info["perdidas_rastreo"]:
            w(f"  PÉRDIDA DE RASTREO {p['t']:>8.2f} {p['hora']} durante {_f(p['dur_s'], 2, ' s')}"
              f"{' (hasta el final)' if p.get('hasta_el_final') else ''} · {p['estado']}"
              f"{' · EN EL AIRE' if p['en_el_aire'] else ''}")
    sep = r["separacion_min_aire"]
    if isinstance(sep, dict):
        w(f"Separación horizontal mínima con los dos en el aire: {sep['m']} m (t {sep['t']}, {sep['hora']}, {sep['estado']})")
    else:
        w(f"Separación horizontal mínima en el aire: {sep or 'los dos drones nunca estuvieron en el aire a la vez'}")
    h = r["huecos"]
    w(f"Huecos de telemetría > {h['umbral_s']} s: {h['n']}" + (f" (máx {h['max_s']} s)" if h["n"] else ""))
    for x in h["lista"][:15]:
        w(f"  {x['t']:>8.2f}  {x['hora']}  {x['dur_s']} s")
    v = r["video"]
    w(f"Video: {v['muestras']} muestras · fps {_f(v['fps_min'], 1)}..{_f(v['fps_max'], 1)} · "
      f"edad máx del último frame {_f(v['edad_max_s'], 2, ' s')}")
    if "caidas" in v:
        w(f"  enlace: {v['caidas']} caídas en la grabación ({v['caidas_vuelo']} en el vuelo) · {v['reconexiones']} "
          f"reconexiones · {v['atascos']} atascos · corte más largo visto {_f(v['caida_max_s'], 1, ' s')}")
    if r["avisos"]:
        w("Avisos:")
        for a in r["avisos"]:
            w(f"  {a['t']:>8.2f}  {a['accion']:<10} {a['nivel']:<8} {a['msg']}")
    w(f"Eventos de la estación: {r['n_eventos']} (se ven en la repetición: python -m app.server --repetir ...)")


def _num_csv(v):
    if v != v:
        return "NaN"
    if math.isinf(v):
        return "Inf" if v > 0 else "-Inf"
    return repr(v)


def exportar_csv(grab, salida):
    """Telemetría -> ``salida`` (t_estacion + 64 columnas; y(1) de Simulink sigue llamándose t) y comandos/acks/eventos/avisos -> ``<salida>_eventos.csv``."""
    base = salida[:-4] if salida.lower().endswith(".csv") else salida
    salida = base + ".csv"
    eventos = base + "_eventos.csv"
    with open(salida, "w", newline="", encoding="utf-8") as fh:
        wr = csv.writer(fh)
        wr.writerow(["t_estacion"] + nombres_y())
        for t, raw in grab.tel:
            wr.writerow([repr(round(t, 4))] + [_num_csv(v) for v in _VEC.unpack(raw)])
    filas = []
    for c in grab.cmds:
        filas.append((c.get("t", 0.0), "cmd", c.get("cmd"), c.get("resultado"), c.get("cmd_seq"),
                      f"{c.get('origen') or ''} {c.get('detalle') or ''}".strip()))
    for a in grab.acks:
        filas.append((a.get("t", 0.0), "ack", a.get("cmd"), a.get("estado"), a.get("cmd_seq"),
                      f"latencia {a.get('latencia_s')} s; {a.get('detalle') or ''}"))
    for e in grab.eventos:
        filas.append((e.get("t", 0.0), "evento", "", "", "", e.get("msg")))
    for a in grab.avisos:
        filas.append((a.get("t", 0.0), "aviso", a.get("accion"), a.get("nivel"), "", a.get("msg")))
    filas.sort(key=lambda f: f[0])
    with open(eventos, "w", newline="", encoding="utf-8") as fh:
        wr = csv.writer(fh)
        wr.writerow(["t", "tipo", "cmd", "estado", "cmd_seq", "texto"])
        for f in filas:
            wr.writerow([repr(round(f[0], 4))] + ["" if v is None else v for v in f[1:]])
    return salida, eventos


def build_parser():
    ap = argparse.ArgumentParser(description="Analiza la caja negra de un vuelo (*.tel.jsonl.gz)")
    sub = ap.add_subparsers(dest="accion", required=True)
    r = sub.add_parser("resumen", help="línea de tiempo, comandos, rastreo, máximos, separación y huecos")
    r.add_argument("archivo", nargs="?", help="archivo o carpeta (por defecto la más reciente de web/datos/vuelos)")
    r.add_argument("--json", action="store_true", help="imprime el resumen en JSON")
    r.add_argument("--hueco", type=float, default=0.2, help="huecos de telemetría a reportar (s)")
    r.add_argument("--z-aire", type=float, default=Z_AIRE, help="z para considerar un dron en el aire (m)")
    c = sub.add_parser("csv", help="exporta telemetría y eventos a CSV (MATLAB: readtable)")
    c.add_argument("archivo", nargs="?", help="archivo o carpeta (por defecto la más reciente de web/datos/vuelos)")
    c.add_argument("-o", "--salida", help="CSV de telemetría (por defecto <nombre>.tel.csv en la carpeta actual, que "
                                          "git ignora); los eventos van a <salida>_eventos.csv")
    return ap


def main(argv=None):
    for f in (sys.stdout, sys.stderr):       # consola de Windows (cp1252) o salida a un archivo / tubería
        if type(f) is io.TextIOWrapper and (f.encoding or "").lower().replace("-", "") != "utf8":
            try:
                f.reconfigure(encoding="utf-8", errors="replace")
            except ValueError:
                pass
    a = build_parser().parse_args(argv)
    try:
        ruta = buscar(a.archivo)
        grab = Grabacion(ruta)
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    if a.accion == "resumen":
        res = analizar(grab, z_aire=a.z_aire, hueco_s=a.hueco)
        if a.json:
            print(json.dumps(res, ensure_ascii=False, indent=1))
        else:
            imprimir(res)
        return 0
    salida = a.salida or os.path.basename(ruta)[:-len(EXT)] + ".tel.csv"
    tel, ev = exportar_csv(grab, salida)
    print(f"Telemetría: {os.path.abspath(tel)} ({len(grab.tel)} filas)\nEventos:    {os.path.abspath(ev)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
