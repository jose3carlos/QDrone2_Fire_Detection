"""Revisión previa al día de laboratorio (solo lee; nunca escribe estacion.json).

Uso (desde la raíz del proyecto, con el venv de la estación):
    %USERPROFILE%\\.venvs\\qd2_web\\Scripts\\python web\\preflight_lab.py
    ... --version MPC          revisa también lo de la versión MPC (por defecto: version_control de la config)
    ... --ping                 hace ping a los dos drones (en modo solo explorador, solo al dron solo)
    ... --config otra.json     revisa otra config (por defecto config/estacion.json)
    ... --remoto 192.168.2.11  la estación corre en ESTA laptop y el modelo de tierra / Motive / quarc_run en la
                               PC 192.168.2.11: no revisa Motive ni los puertos 18700/18001/18002 locales (están
                               en la PC), exige red.ip_pc vacía o igual a esa IP y NO abre ninguna conexión a ella

Modo de misión (mision.modo / mision.dron_solo, mismas reglas que la estación y qd2_params_from_config): en
"solo_explorador" solo se exige la IP y el rigid body del dron solo; la falta de la IP del otro es INFO.

Además (26-nov):
* .motive (optitrack.archivo_motive): lo interpreta (XML de Motive) y exige los IDs rigid_body_dron1 y _dron2 (el
  modelo pide los dos también en solo explorador). FALLA si un dron que vuela tiene el rigid body AUXILIAR
  (UserData 2, triángulo 0.60/0.75/0.90 m de config/motive/RigidBody_dron1_aux2.motive): en dual el dron 2 nunca
  se rastrearía. Si no lo puede interpretar, AVISO con el motivo.
* Build vs config: el modelo de tierra del modo (solo -> QD2_MissionCtrl_Dual_PID, dual -> _PID_2D, como
  laboratorio/iniciar_tierra_lab.m) debe tener <modelo>.rt-win64 en versiones/<release> y no ser más viejo que la
  config, simulink_qdrone2/lib/*.m ni qd2_cargar_config.m. Si hay sello (<modelo>.config_usada.jsonl, lo escribe
  qd2_cargar_config en el InitFcn) compara CONTENIDOS (sha256): de la config solo las claves que lee el modelo
  (CLAVES_MODELO; con el texto que guarda el sello) y de lib/*.m las que usa tierra (sin LIB_NO_TIERRA); si no,
  fechas. Ver revisar_build.

Imprime [OK] / [INFO] / [AVISO] / [FALLA] por punto y termina con código 1 si hay alguna FALLA.
Ver docs/CHECKLIST_LABORATORIO.md.
"""
from __future__ import annotations

import argparse
import datetime
import glob
import hashlib
import ipaddress
import itertools
import json
import math
import ntpath
import os
import socket
import subprocess
import sys
import xml.etree.ElementTree as ET

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SQ = os.path.join(RAIZ, "simulink_qdrone2")
MD5_PID = "00d4d46e7a2e07158690fe98a9b756af"

res = {"OK": 0, "INFO": 0, "AVISO": 0, "FALLA": 0}


def rep(nivel, texto):
    res[nivel] += 1
    print(f"[{nivel:5s}] {texto}", flush=True)


def md5(ruta):
    h = hashlib.md5()
    with open(ruta, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def get(d, *ks, default=None):
    for k in ks:
        if not isinstance(d, dict) or k not in d:
            return default
        d = d[k]
    return default if d is None else d


def puerto_libre(p):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("0.0.0.0", p))
        return True
    except OSError:
        return False
    finally:
        s.close()


def ipv4(v):
    try:
        return str(ipaddress.IPv4Address(str(v).strip()))
    except ValueError:
        raise argparse.ArgumentTypeError(f"IP inválida: '{v}'")


def sha256(ruta):
    h = hashlib.sha256()
    with open(ruta, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def _fecha(t):
    return datetime.datetime.fromtimestamp(t).strftime("%Y-%m-%d %H:%M:%S")


# ==================================================================================== .motive (OptiTrack)
# Formato (Motive 2.x; ejemplo real: config/motive/RigidBody_dron1_aux2.motive): XML
#   <Profile><NodeAssets><rigid_body id="...">
#       <markers><marker><position>x,y,z</position>...</marker>...</markers>
#       <properties><property><name>NodeName|UserData|MarkerLocationN|...</name><value>...</value></property>
# UserData es el ID de streaming (el que piden optitrack.rigid_body_dronN y el bloque OptiTrack Trackables). Sin
# UserData el ID es su posición en el archivo (1, 2...): así funcionó el RigidBody.motive del Equipo2 (un solo
# rigid body sin UserData = ID 1) y el rigid body 1 del archivo de ejemplo.
# Auxiliar (HALLAZGOS_PC_LAB_23NOV.md #6): rigid body con UserData 2 y geometría imposible para que Motive nunca
# lo confunda con un dron (triángulo de 0.60/0.75/0.90 m; nombre AUX_ID2_NO_USAR). Sirve para que el modelo
# arranque en solo explorador (pide los dos IDs), pero con él el dron 2 nunca se rastrea.
AUX_TRIANGULO = (0.60, 0.75, 0.90)      # m, lados del triángulo auxiliar
TOL_AUX = 0.02                          # m
ENVERGADURA_MAX = 0.60                  # m: el rigid body real del QDrone 2 mide ~0.42 m entre marcadores extremos
TOL_GEOM_IGUAL = 0.005                  # m: dos rigid bodies con estas distancias entre marcadores son indistinguibles


def _local(tag):
    return str(tag).rsplit("}", 1)[-1].lower().replace("_", "")


def _xyz(txt):
    v = [float(q) for q in str(txt).replace(";", ",").split(",") if q.strip()]
    if len(v) != 3 or not all(math.isfinite(q) for q in v):
        raise ValueError(f"posición inválida '{txt}'")
    return tuple(v)


def leer_motive(ruta):
    """Rigid bodies de un .motive: lista de dicts {id, id_implicito, nombre, marcadores, lados, envergadura,
    auxiliar (motivo o None)}. ValueError con el motivo si no se puede interpretar."""
    try:
        raiz = ET.parse(ruta).getroot()
    except ET.ParseError as e:
        raise ValueError(f"no es XML de Motive ({e})")
    except OSError as e:
        raise ValueError(str(e))
    rbs = [el for el in raiz.iter() if _local(el.tag) == "rigidbody"]
    if not rbs:
        raise ValueError(f"no trae ningún <rigid_body> (raíz <{_local(raiz.tag)}>)")
    out = []
    for i, rb in enumerate(rbs, 1):
        props = {}
        for p in rb.iter():
            if _local(p.tag) == "property":
                n = next((h.text for h in p if _local(h.tag) == "name"), None)
                v = next((h.text for h in p if _local(h.tag) == "value"), None)
                if n:
                    props[n.strip().lower()] = (v or "").strip()
        marc = [_xyz(m.text) for m in rb.iter() if _local(m.tag) == "position" and m.text]
        if not marc:   # sin <markers>: MarkerLocation0..N de las propiedades
            marc = [_xyz(props[k]) for k in sorted((k for k in props if k.startswith("markerlocation")),
                                                   key=lambda k: int(k[14:] or 0) if k[14:].isdigit() else 0)]
        ud = props.get("userdata", props.get("streamingid", ""))
        if ud:
            try:
                rid, impl = int(float(ud)), False
            except ValueError:
                raise ValueError(f"rigid body {i}: UserData '{ud}' no es un número")
        else:
            rid, impl = i, True
        lados = sorted(math.dist(a, b) for a, b in itertools.combinations(marc, 2))
        d = {"id": rid, "id_implicito": impl, "nombre": props.get("nodename", f"rigid body {i}"),
             "marcadores": marc, "lados": lados, "envergadura": lados[-1] if lados else 0.0}
        d["auxiliar"] = motivo_auxiliar(d)
        out.append(d)
    return out


def motivo_auxiliar(rb):
    """Por qué ``rb`` es el rigid body AUXILIAR (None si no lo es): triángulo 0.60/0.75/0.90 m o el nombre."""
    if len(rb["marcadores"]) == 3 and all(abs(a - b) <= TOL_AUX for a, b in zip(rb["lados"], AUX_TRIANGULO)):
        return "triángulo de %.2f/%.2f/%.2f m" % tuple(rb["lados"])
    n = rb["nombre"].upper()
    if "AUX" in n or "NO_USAR" in n:
        return f"nombre '{rb['nombre']}'"
    return None


def _desc_rb(r):
    t = f"ID {r['id']}{' (sin UserData: por su posición)' if r['id_implicito'] else ''} '{r['nombre']}', " \
        f"{len(r['marcadores'])} marcadores, {r['envergadura']:.2f} m"
    return t + (f", AUXILIAR ({r['auxiliar']})" if r["auxiliar"] else "")


def revisar_motive(ruta, o, solo, ds, rep=None, copia=False):
    """Revisa el contenido del .motive contra optitrack.rigid_body_dron1/2 y el modo. ``copia``: es la copia del
    repo (no el archivo que lee el modelo): las FALLAs se bajan a AVISO."""
    rep = rep or globals()["rep"]
    pre = f"[copia del repo {os.path.relpath(ruta, RAIZ)}, no el archivo del modelo] " if copia else ""
    nom = os.path.basename(ruta)

    def r(nivel, texto):
        rep("AVISO" if copia and nivel == "FALLA" else nivel, pre + texto)

    try:
        rbs = leer_motive(ruta)
    except ValueError as e:
        r("AVISO", f"no pude interpretar {nom}: {e}. Revisa a mano en Motive que tenga los rigid bodies "
                   f"{o.get('rigid_body_dron1')} y {o.get('rigid_body_dron2')}")
        return
    r("INFO", f"{nom}: {len(rbs)} rigid bodies: " + "; ".join(_desc_rb(x) for x in rbs))
    ids = [x["id"] for x in rbs]
    rep_ids = sorted({i for i in ids if ids.count(i) > 1})
    if rep_ids:
        r("AVISO", f"{nom}: ID repetido {rep_ids} (¿uno sin UserData coincide con otro que sí lo tiene?): no se sabe "
                   "cuál toma QUARC")
    usados = []
    for i in (1, 2):
        try:
            rid = int(o.get(f"rigid_body_dron{i}", i))
        except (TypeError, ValueError):
            continue                      # el resto del preflight ya lo reporta
        vuela = (not solo) or i == ds
        cand = [q for q in rbs if q["id"] == rid]
        x = cand[0] if cand else None
        aux_rep = next((q for q in cand if q["auxiliar"]), None) if len(cand) > 1 else None
        if aux_rep is not None and vuela:
            r("FALLA", f"el rigid body {rid} del dron {i} está {len(cand)} veces en {nom} y uno es el AUXILIAR "
                       f"('{aux_rep['nombre']}', {aux_rep['auxiliar']}): QUARC puede tomar el auxiliar y el dron {i} "
                       "nunca se rastrearía (WAIT_TRACKING). BORRA en Motive el rigid body auxiliar y vuelve a "
                       "exportar el .motive")
        elif x is None:
            r("FALLA", f"el rigid body {rid} del dron {i} (optitrack.rigid_body_dron{i}) NO está en {nom}: QUARC da "
                       "'The rigid body ID is invalid or not found' al conectar (el modelo pide los dos IDs, también "
                       "en solo explorador). Exporta de Motive un .motive con los dos")
        elif x["auxiliar"] and vuela:
            if solo:
                r("FALLA", f"el dron solo {i} usa el rigid body {rid} = AUXILIAR ({x['auxiliar']}): nunca se rastrearía. "
                           f"Pon mision.dron_solo / rigid_body_dron{i} al rigid body real")
            else:
                r("FALLA", f"modo DUAL y el rigid body {rid} del dron {i} es el AUXILIAR de {nom} ({x['auxiliar']}): el "
                           f"dron {i} nunca se rastrearía y la misión se queda en WAIT_TRACKING. Crea en Motive el "
                           f"rigid body REAL del dron {i} (UserData = {rid}), BORRA el auxiliar, exporta el .motive y apunta "
                           "optitrack.archivo_motive a él (o usa el modo solo explorador)")
        elif x["auxiliar"]:
            r("INFO", f"rigid body {rid} del dron {i} = auxiliar ({x['auxiliar']}): correcto en solo explorador (el "
                      f"dron {i} no vuela; el modelo solo necesita que el ID exista)")
        elif len(x["marcadores"]) < 3 and vuela:
            r("AVISO", f"el rigid body {rid} del dron {i} tiene {len(x['marcadores'])} marcadores: Motive necesita 3 o "
                       "más para dar orientación")
        elif x["envergadura"] > ENVERGADURA_MAX and vuela:
            r("AVISO", f"el rigid body {rid} del dron {i} mide {x['envergadura']:.2f} m entre marcadores (un QDrone 2 "
                       f"~0.42 m): ¿es el del dron? Revísalo en Motive")
        else:
            r("OK", f"rigid body {rid} del dron {i} en {nom}: '{x['nombre']}', {len(x['marcadores'])} marcadores, "
                    f"{x['envergadura']:.2f} m" + ("" if vuela else " (no vuela en solo explorador)"))
            if vuela:
                usados.append(x)
    if len(usados) == 2:
        a, b = usados
        if len(a["lados"]) == len(b["lados"]) and all(abs(p - q) <= TOL_GEOM_IGUAL for p, q in zip(a["lados"], b["lados"])):
            r("AVISO", f"los rigid bodies {a['id']} y {b['id']} tienen la misma geometría de marcadores: Motive puede "
                       "confundir los drones. Mueve un marcador en uno de los dos y vuelve a crear su rigid body")


def procesos_abiertos():
    """Nombres de los procesos de esta PC (psutil o, si no está, tasklist de Windows); None si no se puede."""
    try:
        import psutil  # noqa: WPS433
        return [pr.info["name"] or "" for pr in psutil.process_iter(["name"])]
    except Exception:
        pass
    if os.name != "nt":
        return None
    try:
        r = subprocess.run(["tasklist", "/FO", "CSV", "/NH"], capture_output=True, text=True, timeout=15)
        return nombres_tasklist(r.stdout) if r.returncode == 0 else None
    except Exception:
        return None


def nombres_tasklist(texto):
    """Primera columna de ``tasklist /FO CSV /NH``."""
    return [ln.split('","', 1)[0].strip().strip('"') for ln in texto.splitlines() if ln.strip().startswith('"')]


def motive_abierto(nombres):
    return sorted({n for n in nombres if n.lower().startswith("motive")})


# ==================================================================================== build del modelo de tierra
VENTANA_SELLO = 1800.0   # s: la línea del sello debe ser de, como mucho, 30 min antes del .rt-win64 (build ~20 s)
TOL_FECHA = 2.0          # s
# Claves de la config que lee el modelo de tierra al compilar (qd2_cargar_config + qd2_params_from_config). Las
# demás (red.ip_*, red.interfaz_preferida, camara, pista.tubos, vision.* salvo radio_cluster_m, web...) son solo de
# la estación: cambiarlas NO obliga a recompilar.
CLAVES_MODELO = ("version_control", "red.puerto_dron1", "red.puerto_dron2", "red.puerto_puente", "optitrack",
                 "mision", "pista.vertices", "vision.radio_cluster_m")
# lib/*.m que el modelo de tierra no usa (a bordo: qd2_onboard_*, PID/MPC de posición; simulación: planta y
# downwash). tests/test_preflight_lab.py comprueba que QD2_Mission_Lib.slx y qd2_cargar_config.m no las alcanzan.
LIB_NO_TIERRA = frozenset("lib/" + n for n in (
    "qd2_downwash.m", "qd2_mpc_init.m", "qd2_mpc_position.m", "qd2_onboard_init.m", "qd2_onboard_step.m",
    "qd2_pid_position.m", "qd2_quad_plant.m", "qd2_wrap_pi.m"))
_FALTA = object()


def modelo_tierra(ver, solo):
    """Modelo de tierra que se compila para este modo (laboratorio/iniciar_tierra_lab.m, HALLAZGOS_PC_LAB_23NOV)."""
    if str(ver).upper() == "MPC":
        return "QD2_MissionCtrl_Dual_MPC"
    return "QD2_MissionCtrl_Dual_PID" if solo else "QD2_MissionCtrl_Dual_PID_2D"


def config_del_modelo(mdl, raiz=None):
    """JSON que lee el InitFcn del modelo: _PID_2D -> config/estacion_dual.json; los demás, config/estacion.json."""
    raiz = raiz or RAIZ
    return os.path.join(raiz, "config", "estacion_dual.json" if mdl.endswith("_2D") else "estacion.json")


def fuentes_build(sq):
    """{ruta relativa a simulink_qdrone2 (con /): ruta} de lo que entra al build además de la config (sin
    LIB_NO_TIERRA)."""
    f = {"qd2_cargar_config.m": os.path.join(sq, "qd2_cargar_config.m")}
    for p in sorted(glob.glob(os.path.join(sq, "lib", "*.m"))):
        if "lib/" + os.path.basename(p) not in LIB_NO_TIERRA:
            f["lib/" + os.path.basename(p)] = p
    return f


def claves_distintas(a, b, pre=""):
    """Rutas 'x.y.z' que difieren entre dos JSON (los objetos se recorren; listas y valores son hojas)."""
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in list(a) + [k for k in b if k not in a]:
            out += claves_distintas(a.get(k, _FALTA), b.get(k, _FALTA), f"{pre}.{k}" if pre else str(k))
        return out
    return [] if _FALTA not in (a, b) and a == b else [pre]


def la_lee_el_modelo(clave):
    return not clave or any(clave == k or clave.startswith(k + ".") or k.startswith(clave + ".") for k in CLAVES_MODELO)


def dif_config(texto, ruta):
    """Claves distintas entre la config que guardó el sello (``texto``) y la de ``ruta``; None si no se puede saber
    (sello sin texto, de antes del 26-nov, o JSON ilegible)."""
    if not isinstance(texto, str):
        return None
    try:
        with open(ruta, encoding="utf-8-sig") as f:
            ahora = json.load(f)
        antes = json.loads(texto.lstrip("﻿"))
    except (OSError, ValueError):
        return None
    return claves_distintas(antes, ahora)


def leer_sello(ruta):
    """Líneas válidas del sello (JSONL); las ilegibles se cuentan y se ignoran. ([], 0) si no existe."""
    if not os.path.exists(ruta):
        return [], 0
    ent, malas = [], 0
    with open(ruta, encoding="utf-8", errors="replace") as f:
        for ln in f:
            if not ln.strip():
                continue
            try:
                e = json.loads(ln)
                float(e["t_posix"]), str(e["config"]["sha256"])
                ent.append(e)
            except Exception:
                malas += 1
    return ent, malas


def revisar_build(ruta_config, ver, solo, carpeta, sq, rep=None, raiz=None, release="R2022a"):
    """FALLA si falta <modelo>.rt-win64 o si es más viejo que lo que lo produce.

    Con sello (<modelo>.config_usada.jsonl junto al .slx; una línea por llamada a qd2_cargar_config con el modelo
    cargado: InitFcn de Update Diagram / Build / Connect e iniciar_tierra_lab) se toma la última línea ANTERIOR a
    la fecha del .rt-win64 (= la del build; las de un Connect posterior no cuentan) y se comparan los sha256 de la
    config, lib/*.m y qd2_cargar_config.m: así un git pull o una copia que solo cambian FECHAS no da FALLA. Si la
    config cambió, con el texto que guarda el sello se dice qué claves: solo dan FALLA las de CLAVES_MODELO (red.ip_*,
    camara, pista.tubos, web... son INFO); las lib/*.m de LIB_NO_TIERRA no cuentan. Límites:
    el sello dice qué config tenía el modelo al empezar el último build anterior al .rt-win64, no lee el binario;
    si esa línea es de más de 30 min antes del .rt-win64 (build hecho con una qd2_cargar_config sin sello), o no hay
    sello, se comparan fechas (git pull / copia sin conservar fechas pueden dar FALLA de más: recompilar la quita).
    Los .slx solo se comparan por fecha y dan AVISO (guardarlos cambia la fecha sin cambiar nada)."""
    rep = rep or globals()["rep"]
    raiz = raiz or RAIZ
    mdl = modelo_tierra(ver, solo)
    rel = lambda p: os.path.relpath(p, raiz) if os.path.splitdrive(p)[0].lower() == os.path.splitdrive(raiz)[0].lower() else p  # noqa: E731
    cfg_mdl = config_del_modelo(mdl, raiz)
    if os.path.normcase(os.path.abspath(cfg_mdl)) != os.path.normcase(os.path.abspath(ruta_config)):
        if os.path.exists(cfg_mdl) and sha256(cfg_mdl) == sha256(ruta_config):
            rep("INFO", f"{mdl} lee {rel(cfg_mdl)} (InitFcn); tiene el mismo contenido que {rel(ruta_config)}")
        else:
            rep("AVISO", f"{mdl} lee {rel(cfg_mdl)} (InitFcn), no {rel(ruta_config)}: el modelo y la estación usarían "
                         f"configs distintas. Corre la estación y este preflight con --config {rel(cfg_mdl)}")
    rt = os.path.join(carpeta, mdl + ".rt-win64")
    if not os.path.exists(rt):
        rep("FALLA", f"no hay versiones/{release}/{mdl}.rt-win64 (modo {'solo explorador' if solo else 'dual'}): "
                     f"compílalo en esta PC (laboratorio\\INICIAR_{'EXPLORADOR' if solo else 'DUAL'}.bat o "
                     f"HARDWARE -> Monitor & Tune con la carpeta actual en versiones\\{release})")
        return
    t_rt = os.path.getmtime(rt)
    fuentes = fuentes_build(sq)
    sello = os.path.join(carpeta, mdl + ".config_usada.jsonl")
    ent, malas = leer_sello(sello)
    if malas:
        rep("AVISO", f"{os.path.basename(sello)}: {malas} línea(s) ilegibles, se ignoran")
    previas = [e for e in ent if float(e["t_posix"]) <= t_rt + TOL_FECHA and e.get("modelo", mdl) == mdl]
    e = max(previas, key=lambda q: float(q["t_posix"])) if previas else None
    if e is not None and t_rt - float(e["t_posix"]) > VENTANA_SELLO:
        rep("INFO", f"el sello más cercano es del {_fecha(float(e['t_posix']))}, {(t_rt - float(e['t_posix'])) / 60:.0f} "
                    f"min antes del build ({_fecha(t_rt)}): ese build no dejó sello; comparo fechas")
        e = None
    elif e is None and ent:
        rep("INFO", f"el sello {os.path.basename(sello)} solo tiene líneas posteriores al build ({_fecha(t_rt)}): "
                    "comparo fechas")
    elif e is None:
        rep("INFO", f"sin sello {os.path.basename(sello)} (lo escribe qd2_cargar_config al compilar desde el 26-nov): "
                    "comparo fechas; git pull o copiar sin conservar fechas puede dar FALLA de más (recompilar la quita)")
    cambios, nota = [], ""
    if e is not None:
        if e["config"]["sha256"] != sha256(ruta_config):
            dif = dif_config(e["config"].get("texto"), ruta_config)
            usadas = [k for k in (dif or []) if la_lee_el_modelo(k)]
            if dif is None:
                cambios.append(f"la config {rel(ruta_config)} (el build usó {e['config'].get('ruta', '?')}, "
                               f"sha256 {e['config']['sha256'][:12]}; ahora {sha256(ruta_config)[:12]}; sello sin el "
                               "texto de la config o JSON ilegible: no sé qué clave)")
            elif usadas:
                cambios.append(f"la config {rel(ruta_config)} ({', '.join(usadas[:6])}"
                               f"{' y %d más' % (len(usadas) - 6) if len(usadas) > 6 else ''})")
            else:
                rep("INFO", f"la config {rel(ruta_config)} cambió después del build solo en claves que el modelo no "
                            f"lee ({', '.join(dif[:6]) or 'formato'}{' y %d más' % (len(dif) - 6) if len(dif) > 6 else ''}): "
                            "no hace falta recompilar")
        arch = {a.get("ruta"): a.get("sha256") for a in (e.get("archivos") or []) if isinstance(a, dict)}
        if arch:
            for r_, p in fuentes.items():
                if r_ not in arch:
                    cambios.append(f"{r_} (nuevo)")
                elif arch[r_] != sha256(p):
                    cambios.append(r_)
            faltan = sorted(set(arch) - set(fuentes) - LIB_NO_TIERRA)
            if faltan:
                rep("AVISO", f"el build usó {', '.join(faltan)}, que ya no existen: recompila si fue a propósito")
        else:
            cambios += [r_ for r_, p in fuentes.items() if os.path.getmtime(p) > t_rt + TOL_FECHA]
        nuevos_fecha = [p for p in [ruta_config] + list(fuentes.values()) if os.path.getmtime(p) > t_rt + TOL_FECHA]
        if nuevos_fecha and not cambios:
            nota = f" ({len(nuevos_fecha)} archivo(s) con fecha posterior pero mismo contenido para el modelo: git pull / copia)"
        base = f"build {mdl}.rt-win64 del {_fecha(t_rt)}; sello del {e.get('fecha') or _fecha(float(e['t_posix']))}"
    else:
        for r_, p in [(rel(ruta_config), ruta_config)] + list(fuentes.items()):
            if os.path.getmtime(p) > t_rt + TOL_FECHA:
                cambios.append(f"{r_} ({_fecha(os.path.getmtime(p))})")
        base = f"build {mdl}.rt-win64 del {_fecha(t_rt)} (por fechas)"
    if cambios:
        rep("FALLA", f"recompila {mdl}: cambió {', '.join(cambios[:6])}{' y %d más' % (len(cambios) - 6) if len(cambios) > 6 else ''} "
                     f"después del build ({base})")
    else:
        rep("OK", f"{base}: misma config, lib/*.m y qd2_cargar_config.m{nota}")
    slx = [p for p in (os.path.join(carpeta, mdl + ".slx"), os.path.join(carpeta, "QD2_Mission_Lib.slx"))
           if os.path.exists(p) and os.path.getmtime(p) > t_rt + TOL_FECHA]
    if slx:
        rep("AVISO", f"{', '.join(os.path.basename(p) for p in slx)} con fecha posterior al build: si cambiaste el "
                     "modelo, recompila (guardar sin cambios también mueve la fecha)")


def main(argv=None):
    for k in res:
        res[k] = 0
    ap = argparse.ArgumentParser(description="Revisión previa al laboratorio (solo lectura)")
    ap.add_argument("--config", default=os.path.join(RAIZ, "config", "estacion.json"))
    ap.add_argument("--version", choices=["PID", "MPC"], default=None)
    ap.add_argument("--release", default="R2022a", help="carpeta de versiones/ que se va a abrir (R2022a, R2025a...)")
    ap.add_argument("--ping", action="store_true")
    ap.add_argument("--remoto", type=ipv4, default=None, metavar="IP_PC",
                    help="IP de la PC del modelo de tierra cuando la estación corre en esta laptop (no se conecta a ella)")
    a = ap.parse_args(argv)
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass

    print(f"Proyecto: {RAIZ}")
    # ---------------------------------------------------------------- ruta
    if len(RAIZ) > 90:
        rep("AVISO", f"ruta del proyecto de {len(RAIZ)} caracteres: el build QUARC puede pasar de 260. Copia a una ruta corta (p. ej. C:\\QD2)")
    else:
        rep("OK", f"ruta corta ({len(RAIZ)} caracteres)")
    if "onedrive" in RAIZ.lower():
        rep("AVISO", "el proyecto está dentro de OneDrive (autosave y sincronización fallan a veces): mejor C:\\QD2")
    if sys.version_info < (3, 10):
        rep("FALLA", f"Python {sys.version.split()[0]}: la estación necesita 3.10 o más")

    # ---------------------------------------------------------------- config
    try:
        with open(a.config, encoding="utf-8") as f:
            c = json.load(f)
        rep("OK", f"config leída: {a.config}")
    except Exception as e:  # pragma: no cover
        rep("FALLA", f"no pude leer {a.config}: {e}")
        return 1
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from app.config import mision_modo  # noqa: WPS433
    modo = mision_modo(c)
    solo = modo["modo"] == "solo_explorador"
    ds = modo["dron_solo"]
    if modo["aviso"]:
        rep("AVISO", f"{modo['aviso']} (con un solo dron, dual se queda en WAIT_TRACKING y no despega)")
    if solo:
        rep("OK", f"modo SOLO EXPLORADOR con el dron {ds} (el dron {3 - ds} no recibe arm/takeoff)")
    else:
        rep("OK", "modo dual (2 drones)")
    if a.remoto:
        print(f"Estación remota: el modelo de tierra corre en {a.remoto} (no se abre ninguna conexión hacia ella)")
    ver = (a.version or str(get(c, "version_control", default="PID"))).upper()
    print(f"Versión revisada: {ver}")
    if str(get(c, "version_control", default="")).upper() != ver:
        rep("FALLA", f"version_control = {get(c, 'version_control')} en la config, pero vas a correr {ver}: cámbialo en la pestaña Misión")
    else:
        rep("OK", f"version_control = {ver}")

    # ---------------------------------------------------------------- modelos y binarios
    carpeta = os.path.join(SQ, "versiones", a.release)
    tierra = modelo_tierra(ver, solo) + ".slx"     # dual PID -> _PID_2D (lee config/estacion_dual.json)
    for f in [tierra, "QD2_Mission_Lib.slx"] + (["QD2_DroneStack_MPC.slx"] if ver == "MPC" else []):
        p = os.path.join(carpeta, f)
        rep("OK" if os.path.exists(p) else "FALLA", f"{'existe' if os.path.exists(p) else 'FALTA'} versiones/{a.release}/{f}")
    bdir = os.path.join(SQ, "binarios_dron")
    if ver == "PID":
        p = os.path.join(bdir, "QD2_DroneStack_PID_2021a.rt-linux_qdrone2")
        if not os.path.exists(p):
            rep("FALLA", "falta simulink_qdrone2/binarios_dron/QD2_DroneStack_PID_2021a.rt-linux_qdrone2 (sale de Equipo2.zip)")
        elif md5(p) != MD5_PID:
            rep("AVISO", f"el binario PID no es el original del Equipo2 (md5 {md5(p)})")
        else:
            rep("OK", "binario PID original del Equipo2 (md5 correcto)")
    else:
        p = os.path.join(bdir, "QD2_DroneStack_MPC.rt-linux_qdrone2")
        if not os.path.exists(p):
            rep("AVISO", "aún no hay binarios_dron/QD2_DroneStack_MPC.rt-linux_qdrone2: compílalo (misma construcción que tierra MPC)")
        else:
            t_bin = os.path.getmtime(p)
            t_slx = os.path.getmtime(os.path.join(carpeta, "QD2_DroneStack_MPC.slx")) if os.path.exists(os.path.join(carpeta, "QD2_DroneStack_MPC.slx")) else 0
            rep("OK" if t_bin >= t_slx else "AVISO",
                "binario MPC presente" + ("" if t_bin >= t_slx else ", pero es MÁS VIEJO que el .slx: recompílalo"))

    # ---------------------------------------------------------------- build de tierra vs config / lib
    if a.remoto:
        rep("INFO", f"build de tierra ({modelo_tierra(ver, solo)}.rt-win64): está en la PC {a.remoto}; corre allá "
                    "este preflight (sin --remoto) para compararlo con la config y la lib")
    else:
        revisar_build(os.path.abspath(a.config), ver, solo, carpeta, SQ, release=a.release)

    # ---------------------------------------------------------------- visión
    modelo = get(c, "vision", "modelo", default="vision/modelos/mejor.pt")
    mp = modelo if os.path.isabs(modelo) else os.path.join(RAIZ, modelo)
    onnx = os.path.splitext(mp)[0] + ".onnx"
    if not os.path.exists(onnx):
        rep("FALLA", f"falta {os.path.relpath(onnx, RAIZ)}: la estación caería al FOMO (solo fuego)")
    else:
        rep("OK", f"modelo de visión {os.path.relpath(onnx, RAIZ)}")
        j = onnx + ".json"
        if os.path.exists(j) and os.path.exists(mp):
            try:
                with open(j, encoding="utf-8") as f:
                    meta = json.load(f)
                guardado = next((v for k, v in meta.items() if "md5" in k.lower()), None)
                if guardado and guardado != md5(mp):
                    rep("AVISO", "el .onnx no corresponde al .pt de al lado: corre vision/exportar_onnx.py")
                # .onnx de entrada no cuadrada (480x640): necesita el detector.py que lee la forma del .onnx
                imgsz = meta.get("imgsz")
                if isinstance(imgsz, list) and len(imgsz) == 2 and imgsz[0] != imgsz[1]:
                    with open(os.path.join(RAIZ, "vision", "detector.py"), encoding="utf-8") as f:
                        if "forma_entrada_onnx" not in f.read():
                            rep("FALLA", f"el .onnx tiene entrada {imgsz[0]}x{imgsz[1]} y vision/detector.py es una "
                                         "versión vieja (sólo 640x640): actualiza el repositorio")
            except Exception:
                pass

    # ---------------------------------------------------------------- cámara
    cam = get(c, "camara", default={})
    if cam.get("fuente") != "dron":
        rep("FALLA", f"camara.fuente = {cam.get('fuente')!r}: con drones reales debe ser 'dron'")
    else:
        rep("OK", "camara.fuente = dron")
    # camara.dron: dron con la RealSense; el video sale SIEMPRE de ip_dron{camara.dron} y la estación bloquea START
    # si el explorador es el otro (mismas reglas que la estación: app.config.camara_dron)
    from app.config import camara_dron  # noqa: WPS433
    cd = camara_dron(c)
    if cd["aviso"]:
        rep("AVISO", "estacion.json: " + cd["aviso"])
    if solo and ds in (1, 2) and ds != cd["dron"]:
        rep("FALLA", f"camara.dron = {cd['dron']} pero el dron solo es el {ds}: la estación bloquea START (el "
                     "explorador no lleva la cámara). Cambia mision.dron_solo o camara.dron")
    elif solo:
        rep("OK", f"camara.dron = {cd['dron']} = dron solo")
    else:
        rep("INFO", f"camara.dron = {cd['dron']}: coloca ese dron con MENOR x que el otro (ASSIGN_ROLES lo hace "
                    "explorador), al menos 5 cm más atrás; si no, la estación bloquea START")
    if get(c, "web", "permitir_sintetica_en_simulink", default=False):
        rep("FALLA", "web.permitir_sintetica_en_simulink = true: mandaría objetivos inventados a los drones reales")
    res_cam = "%dx%d" % tuple(cam.get("resolucion", [640, 480]))
    if res_cam in (cam.get("intrinsecos") or {}):
        rep("OK", f"camara.intrinsecos[{res_cam}] reales presentes")
    else:
        rep("AVISO", f"sin camara.intrinsecos[{res_cam}]: se usan los nominales; copia los que imprime el streamer")

    # ---------------------------------------------------------------- OptiTrack
    o = get(c, "optitrack", default={})
    if o.get("rigid_body_dron1") == o.get("rigid_body_dron2"):
        rep("FALLA", "rigid_body_dron1 = rigid_body_dron2: qd2_cargar_config no deja arrancar el modelo")
    else:
        rep("OK", f"rigid bodies {o.get('rigid_body_dron1')} / {o.get('rigid_body_dron2')}")
    for k in ("archivo_cal", "archivo_motive"):
        v = o.get(k) or ""
        if not v and k == "archivo_motive":
            rep("FALLA", "optitrack.archivo_motive vacío: el modelo usa el RigidBody.motive del Equipo2 del 20-nov (un "
                         "solo rigid body, ID 1) y QUARC da 'The rigid body ID is invalid or not found' al conectar (pide "
                         "los dos IDs). Vincula el .motive del día en la pestaña Motive")
            continue
        if not v:
            rep("AVISO", f"optitrack.{k} vacío: el modelo usa la calibración del Equipo2 del 20-nov. Vincula la del día en la pestaña Motive")
            continue
        p = v if (len(v) > 2 and (v[1] == ":" or v.startswith("\\\\"))) else os.path.join(o.get("carpeta", ""), v)
        existe = (not a.remoto) and os.path.isfile(p)
        if a.remoto:
            rep("INFO", f"optitrack.{k}: {p} (ruta de la PC {a.remoto}: no se verifica desde aquí)")
        else:
            rep("OK" if existe else "FALLA", f"optitrack.{k}: {p} {'existe' if existe else 'NO EXISTE en esta PC'}")
        if k != "archivo_motive":
            continue
        if existe:
            revisar_motive(p, o, solo, ds)
        else:
            # no se puede leer el de la PC: si el repo trae uno con el mismo nombre, se revisa esa copia (sin FALLAs)
            copia = os.path.join(RAIZ, "config", "motive", ntpath.basename(p))
            if os.path.isfile(copia):
                revisar_motive(copia, o, solo, ds, copia=True)
    if solo:
        rep("INFO", f"rigid body del explorador (dron {ds}) = {o.get(f'rigid_body_dron{ds}')}; el del dron {3 - ds} "
                    "debe ser distinto y ese dron debe quedar fuera de la arena")
    if a.remoto:
        rep("INFO", f"Motive corre en la PC {a.remoto}: ciérralo allá antes de compilar y de correr el modelo de tierra")
    else:
        nombres = procesos_abiertos()
        if nombres is None:
            rep("INFO", "no pude listar los procesos (ni psutil ni tasklist): revisa a mano que Motive esté cerrado")
        elif motive_abierto(nombres):
            rep("AVISO", f"Motive está abierto ({', '.join(motive_abierto(nombres))}): ciérralo antes de compilar y de "
                         "correr el modelo de tierra (la guía lo pide; el bloque OptiTrack usa la API de Motive)")
        else:
            rep("OK", "Motive cerrado")

    # ---------------------------------------------------------------- pista
    v = get(c, "pista", "vertices", default=[])
    lim = get(c, "mision", "limites_xy", default=[-1.5, 1.5])
    if not (2 <= len(v) <= 16):
        rep("FALLA", f"pista con {len(v)} vértices (Simulink acepta 2..16)")
    elif any(not (lim[0] <= q <= lim[1]) for p in v for q in p):
        rep("FALLA", f"algún vértice de la pista fuera de {lim}")
    else:
        rep("OK", f"pista: {len(v)} vértices dentro de {lim}")
    # envolvente de consignas (mision.limites_x / limites_y; Simulink la toma al COMPILAR)
    from app.config import mision_envolvente, vertices_fuera_envolvente  # noqa: WPS433
    env = mision_envolvente(c)
    if env["aviso"]:
        rep("AVISO", "estacion.json: " + env["aviso"])
    if env["definida"]:
        fuera = vertices_fuera_envolvente(c)
        txt = f"x {env['lo'][0]:g}..{env['hi'][0]:g}, y {env['lo'][1]:g}..{env['hi'][1]:g} m"
        if fuera:
            rep("AVISO", f"vértice(s) de la ruta fuera de la envolvente de consignas ({txt}): {fuera}; el explorador "
                "no alcanza ese waypoint (consigna recortada) y se queda en hover ahí")
        else:
            rep("OK", f"envolvente de consignas {txt}: la ruta queda dentro (recompila tierra para que la tome)")

    # ---------------------------------------------------------------- red
    red = get(c, "red", default={})
    ips = [red.get("ip_dron1") or "", red.get("ip_dron2") or ""]
    usados = [ds] if solo else [1, 2]          # drones que se van a lanzar
    ips_usadas = [ips[i - 1] for i in usados]
    if solo:
        if not ips[ds - 1]:
            rep("FALLA", f"falta ip_dron{ds} (dron solo; pestaña Conexiones)")
        else:
            rep("OK", f"dron solo {ds}: {ips[ds - 1]}")
        if not ips[2 - ds]:
            rep("INFO", f"ip_dron{3 - ds} vacía: no se usa en modo solo explorador")
        elif ips[0] == ips[1]:
            rep("FALLA", "ip_dron1 = ip_dron2")
    elif not all(ips):
        rep("FALLA", "faltan ip_dron1 / ip_dron2 (pestaña Conexiones)")
    elif ips[0] == ips[1]:
        rep("FALLA", "ip_dron1 = ip_dron2")
    else:
        rep("OK", f"drones {ips[0]} / {ips[1]}")
    if a.remoto:
        ip_cfg = red.get("ip_pc") or ""
        if ip_cfg and ip_cfg != a.remoto:
            rep("FALLA", f"red.ip_pc = {ip_cfg} pero el modelo de tierra corre en {a.remoto}: el -URI_Host de los drones "
                         "apuntaría a otra máquina. Pon red.ip_pc vacía o igual a la PC")
        else:
            rep("OK", f"red.ip_pc = {ip_cfg or '(vacía)'}: el -URI_Host apunta a la PC {a.remoto}")
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from app import netinfo  # noqa: WPS433
        ifs = netinfo.interfaces(red.get("interfaz_preferida", "Wi-Fi 2"))
        ip_pc = netinfo.ip_recomendada(c, a.remoto)
        for r in ifs:
            marca = " <- Alfa/recomendada" if r["recomendada"] else ""
            print(f"         interfaz {r['nombre']}: {r['ip']} ({r['descripcion']}){marca}")
        if not ip_pc:
            rep("FALLA", "no encuentro la IP de la PC en la red de los drones (antena Alfa 'Wi-Fi 2' sin IP)")
        else:
            rep("OK", f"IP de la PC para -URI_Host: {ip_pc}")
            if all(ips_usadas):
                try:
                    red_pc = ipaddress.ip_network(ip_pc + "/24", strict=False)
                    fuera = [ip for ip in ips_usadas if ipaddress.ip_address(ip) not in red_pc]
                    if fuera:
                        rep("FALLA", f"{fuera} no están en la red {red_pc} de la PC: revisa a qué Wi-Fi está conectada la Alfa")
                except ValueError as e:
                    rep("FALLA", f"IP inválida: {e}")
    except Exception as e:
        rep("AVISO", f"no pude listar interfaces: {e}")

    for p, quien in ((int(red.get("puerto_puente", 18700)), "puente Simulink-Python"),
                     (int(red.get("puerto_dron1", 18001)), "Stream Server dron 1"),
                     (int(red.get("puerto_dron2", 18002)), "Stream Server dron 2"),
                     (int(red.get("puerto_web", 8080)), "web")):
        if a.remoto and quien != "web":
            rep("INFO", f"puerto {p} ({quien}): está en la PC {a.remoto}, no se revisa aquí")
            continue
        if puerto_libre(p):
            rep("OK", f"puerto {p} libre ({quien})")
        else:
            rep("AVISO", f"puerto {p} ocupado ({quien}): normal si el modelo de tierra o la estación ya corren; si no, ciérralo (netstat -ano | findstr :{p})")

    if a.ping and all(ips_usadas):
        for i, ip in zip(usados, ips_usadas):
            ok = subprocess.run(["ping", "-n", "2", "-w", "1000", ip], capture_output=True).returncode == 0
            rep("OK" if ok else "FALLA", f"ping dron {i} ({ip}) {'responde' if ok else 'NO responde'}")

    # ---------------------------------------------------------------- QUARC
    qd = os.environ.get("QUARC_DIR", "")
    qr = os.path.join(qd, "quarc_run.exe") if qd else ""
    if qr and os.path.exists(qr):
        rep("OK", f"quarc_run: {qr}")
    elif a.remoto:
        rep("INFO", f"quarc_run se corre en la PC {a.remoto} (cmd en C:\\QD2\\simulink_qdrone2\\binarios_dron)")
    else:
        rep("FALLA", "no encuentro quarc_run.exe (QUARC_DIR): instala/activa QUARC en esta PC")

    print(f"\nResumen: {res['OK']} OK, {res['INFO']} info, {res['AVISO']} avisos, {res['FALLA']} fallas")
    return 1 if res["FALLA"] else 0


if __name__ == "__main__":
    sys.exit(main())
