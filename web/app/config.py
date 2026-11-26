"""Lectura y escritura de ``config/estacion.json`` (única fuente de verdad, ARQUITECTURA §5-§6).

Reglas:
* Nunca se renombran ni se borran claves existentes; solo se modifican valores o se agregan claves.
* Cada guardado vuelve a leer el archivo del disco y aplica solo el *parche* pedido. Así no se
  pierden cambios que otra herramienta (Simulink, MATLAB, otro agente) haya hecho mientras tanto.
* La escritura es atómica (archivo temporal + ``os.replace``) y conserva el estilo del archivo
  original: diccionarios con sangría de 2 espacios, listas de números en una sola línea, el
  texto original de cada número (``0.20`` sigue siendo ``0.20``) y los finales de línea (CRLF o LF),
  para no ensuciar el ``git diff``.
* Al arrancar NO se escribe nada: los valores por defecto de la sección ``web`` viven en memoria
  (``WEB_DEFAULTS``) y solo se guardan las claves que el usuario cambia desde la página.
"""
from __future__ import annotations

import copy
import ipaddress
import json
import math
import os
import threading
import time

WEB_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_DIR = os.path.dirname(WEB_DIR)
DEFAULT_PATH = os.path.join(PROJECT_DIR, "config", "estacion.json")

_LOCK = threading.RLock()

# Valores por defecto que la estación agrega si faltan (solo claves nuevas, nunca renombres).
WEB_DEFAULTS = {
    "fuente": "mock",            # mock | simulink | sim3d
    "host": "127.0.0.1",         # por seguridad solo escucha en la propia PC
    "host_puente": "127.0.0.1",  # dónde corre el Stream Server de Simulink / simulador
    "detector": "auto",          # auto | vision | fomo | falso | ninguno
    "camara_latencia_s": 0.12,   # retraso estimado del video para sincronizar con la pose
    "z_min_deteccion": 0.4,      # no se geolocaliza con el explorador por debajo de esta altura
    "cmd_timeout_s": 1.5,        # un comando sin ack de Simulink en este tiempo caduca (no se ejecuta tarde)
    "registro_vuelos": "auto",   # registro por vuelo en web/datos/vuelos: auto (solo fuente simulink) | true | false
    "caja_negra": "auto",        # caja negra por vuelo (*.tel.jsonl.gz, caja_negra.py): auto (solo simulink) | true | false
    "caja_negra_decimacion": 1,  # 1 = graba todos los paquetes (50 Hz); 2 = uno de cada 2 (25 Hz), ver web/README.md
    "puerto_quarc_target": 17000,
    "puerto_ext_mode": 17001,
}

# Filtros físicos de la estación sobre las detecciones (vision_service.VisionService.process).
# Clave ausente -> este valor; clave presente con null (o <= 0) -> filtro desactivado.
VISION_FILTROS_DEFAULTS = {
    "tam_max_m": 0.35,              # caja proyectada al piso más grande que esto: no es una pieza (cara, persona)
    "tam_min_m": 0.0,               # 0 = desactivado
    "dist_max_tubo_toma_m": 0.25,   # una toma va abrazada al tubo: más lejos de la polilínea pista.vertices se descarta
    "dist_max_tubo_fuego_m": None,  # null = desactivado (un fuego puede estar fuera del tubo)
    "inclinacion_max_deg": 15.0,    # explorador más inclinado que esto (con actitud y(61:63) válida): se descarta
}

# Actitud del explorador (roll/pitch de y(61:63)) en la geolocalización (vision_service, geo.R_mundo_cuerpo).
ACTITUD_DEFAULTS = {
    "usar_actitud": True,            # false = suponer el dron nivelado (comportamiento anterior) y no descartar por inclinación
    "actitud_signos": [1, 1],        # [roll, pitch]: -1 invierte ese ángulo si la prueba del libro sale al revés
    "actitud_offset_deg": [0.0, 0.0],  # [roll, pitch] que marca el dron nivelado en el piso (se resta)
}


def actitud_vision(cfg):
    """Cómo usar la actitud del explorador, leído de ``cfg['vision']`` (estacion.json completo).

    Devuelve ``{"usar": bool, "signo_roll", "signo_pitch": ±1.0, "offset_roll", "offset_pitch": rad}``.
    ``usar_actitud`` solo acepta true/false (cualquier otro valor -> true); un signo que no sea ±1 o un
    offset no numérico se ignoran (valor por defecto), igual que en ``filtros_vision``."""
    vis = cfg.get("vision") if isinstance(cfg, dict) else None
    vis = vis if isinstance(vis, dict) else {}
    usar = vis.get("usar_actitud", ACTITUD_DEFAULTS["usar_actitud"])
    usar = usar if isinstance(usar, bool) else ACTITUD_DEFAULTS["usar_actitud"]

    def par(key, ok):
        val = vis.get(key)
        dflt = ACTITUD_DEFAULTS[key]
        if not isinstance(val, (list, tuple)) or len(val) != 2:
            return [float(a) for a in dflt]
        out = []
        for a, d in zip(val, dflt):
            good = not isinstance(a, bool) and isinstance(a, (int, float)) and math.isfinite(float(a)) and ok(float(a))
            out.append(float(a) if good else float(d))
        return out

    sr, sp = par("actitud_signos", lambda a: a in (1.0, -1.0))
    orr, op = par("actitud_offset_deg", lambda a: abs(a) <= 30.0)
    return {"usar": usar, "signo_roll": sr, "signo_pitch": sp,
            "offset_roll": math.radians(orr), "offset_pitch": math.radians(op)}


def filtros_vision(cfg):
    """Umbrales de los filtros físicos leídos de ``cfg['vision']`` (estacion.json completo, ya decodificado).

    Devuelve ``{clave: float o None}``; ``None`` = filtro desactivado. Un valor que no es número
    (texto, lista, bool) se ignora y se usa el valor por defecto, para que un error de captura en el
    JSON no apague un filtro sin querer ni tumbe el hilo de visión.
    """
    vis = cfg.get("vision") if isinstance(cfg, dict) else None
    vis = vis if isinstance(vis, dict) else {}
    out = {}
    for k, dflt in VISION_FILTROS_DEFAULTS.items():
        v = vis.get(k, dflt)
        if v is not None:
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                v = dflt
            elif not math.isfinite(float(v)) or float(v) <= 0:
                v = None
        out[k] = float(v) if v is not None else None
    return out


# Modo de misión (mision.modo / mision.dron_solo). Mismas reglas que qd2_params_from_config.m (§1.2 del
# contrato "modo solo explorador"): cualquier valor inválido cae a "dual", que con un solo dron se queda en
# WAIT_TRACKING (exige los dos drones rastreados) y nunca despega.
MODOS = ("dual", "solo_explorador")


def mision_modo(cfg):
    """``{"modo": "dual"|"solo_explorador", "dron_solo": 0|1|2, "aviso": str|None}`` leído de ``cfg['mision']``.

    * ``modo`` ausente, null, lista vacía o "" -> dual sin aviso; texto -> ``lower(strip())``: "dual" o
      "solo_explorador"; cualquier otra cosa (texto desconocido, número, bool, lista, objeto) -> dual con aviso.
    * En solo, ``dron_solo`` ausente/null/lista vacía -> 1; un número real finito (no bool) igual a 1 o 2 (1.0
      vale) -> ese dron; cualquier otro valor (0, 3, 1.5, "1", true, NaN, [1, 2]) -> dual con aviso.
    * En dual ``dron_solo`` = 0 (no se mira)."""
    m = cfg.get("mision") if isinstance(cfg, dict) else None
    m = m if isinstance(m, dict) else {}
    dual = {"modo": "dual", "dron_solo": 0, "aviso": None}
    raw = m.get("modo")
    if raw is None or (isinstance(raw, list) and not raw) or (isinstance(raw, str) and raw == ""):
        return dual
    if not isinstance(raw, str):
        return dict(dual, aviso=f"mision.modo desconocido ('{_txt(raw)}'): se usa dual")
    s = raw.strip().lower()
    if s == "dual":
        return dual
    if s != "solo_explorador":
        return dict(dual, aviso=f"mision.modo desconocido ('{raw}'): se usa dual")
    d = m.get("dron_solo")
    if d is None or (isinstance(d, list) and not d):
        return {"modo": "solo_explorador", "dron_solo": 1, "aviso": None}
    if (not isinstance(d, bool) and isinstance(d, (int, float)) and math.isfinite(float(d))
            and float(d) in (1.0, 2.0)):
        return {"modo": "solo_explorador", "dron_solo": int(float(d)), "aviso": None}
    return dict(dual, aviso=f"mision.dron_solo inválido ('{_txt(d)}'): se usa dual")


def camara_dron(cfg):
    """``{"dron": 1|2, "aviso": str|None}``: dron que lleva la RealSense abajo (``cfg['camara']['dron']``). El video
    QDF1 se conecta SIEMPRE a ``red.ip_dron{dron}`` (no al explorador de turno, que ASSIGN_ROLES reevalúa antes del
    START) y START se bloquea si el explorador no es este dron (``Station.command``).

    * ausente, null o [] -> 1 sin aviso;
    * número real finito (no bool) igual a 1 o 2 (1.0 vale) -> ese dron;
    * cualquier otro valor (0, 3, 1.5, "2", true, NaN, [1, 2]) -> 1 con aviso."""
    c = cfg.get("camara") if isinstance(cfg, dict) else None
    c = c if isinstance(c, dict) else {}
    d = c.get("dron")
    if d is None or (isinstance(d, list) and not d):
        return {"dron": 1, "aviso": None}
    if _es_num(d) and float(d) in (1.0, 2.0):
        return {"dron": int(float(d)), "aviso": None}
    return {"dron": 1, "aviso": f"camara.dron inválido ('{_txt(d)}'; 1 o 2): se usa el dron 1"}


# Pérdida de rastreo sostenida en el aire -> ese dron aterriza (qd2_params.m / qd2_params_from_config.m).
T_PERDIDA_TRK = 0.2              # s, = p.t_perdida_trk (umbral de la FSM a bordo del Equipo2 y de PAUSED)
T_PERDIDA_ATERRIZAR_DEF = 0.2    # s, = p.t_perdida_aterrizar_s por defecto
T_PERDIDA_ATERRIZAR_MAX = 2.0    # s, = T_MAX de qd2_params_from_config (sin pose el dron cae en ~1-2 s)


def mision_t_perdida_aterrizar(cfg):
    """``{"valor": s, "aviso": str|None}`` de ``cfg['mision']['t_perdida_aterrizar_s']`` con las mismas reglas que
    ``t_aterrizar`` de ``qd2_params_from_config.m`` (sobre lo que da ``jsondecode``):

    * ausente, null o [] -> 0.2 s sin aviso;
    * número real finito (no bool) en [0.2, 2.0] -> ese valor (``[0.3]`` también: jsondecode lo vuelve escalar);
    * cualquier otra cosa (fuera de rango, texto, true, lista de 2, objeto) -> 0.2 s con aviso.

    Con el binario PID del Equipo2 debe quedar en 0.2 s; un valor mayor deja PAUSED entre 0.2 s y ese valor."""
    m = cfg.get("mision") if isinstance(cfg, dict) else None
    m = m if isinstance(m, dict) else {}
    dflt = {"valor": T_PERDIDA_ATERRIZAR_DEF, "aviso": None}
    if "t_perdida_aterrizar_s" not in m:
        return dflt
    raw = m["t_perdida_aterrizar_s"]
    x = raw
    if x is None or (isinstance(x, list) and not x):
        return dflt                                   # null o [] = no definido
    if isinstance(x, list) and len(x) == 1 and not isinstance(x[0], (list, dict, str)) and x[0] is not None:
        x = x[0]                                      # jsondecode('[0.3]') = 0.3 (escalar)
    if (not isinstance(x, bool) and isinstance(x, (int, float)) and math.isfinite(float(x))
            and T_PERDIDA_TRK <= float(x) <= T_PERDIDA_ATERRIZAR_MAX):
        return {"valor": float(x), "aviso": None}
    return dict(dflt, aviso=(f"mision.t_perdida_aterrizar_s inválido ('{_txt(raw)}'; válido "
                             f"{T_PERDIDA_TRK:.1f} a {T_PERDIDA_ATERRIZAR_MAX:.1f} s): se usa "
                             f"{T_PERDIDA_ATERRIZAR_DEF:.2f} s"))


# Envolvente por eje de las consignas xy (mision.limites_x / mision.limites_y; qd2_params_from_config.m y
# qd2_limites_xy.m). Sin la clave, ese eje usa limites_xy -/+ XY_MARGEN (±1.35 m con ±1.5).
XY_MARGEN = 0.15                 # m, = p.xy_margen
LIMITES_XY_DEF = (-1.5, 1.5)     # m, = p.xy_lim


def _es_num(v):
    return not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(float(v))


def mision_limites_xy(cfg):
    """``(lo, hi)`` de ``mision.limites_xy`` como lo toma ``qd2_params_from_config`` (2 números) o ±1.5."""
    m = cfg.get("mision") if isinstance(cfg, dict) else None
    lim = (m if isinstance(m, dict) else {}).get("limites_xy")
    if isinstance(lim, list) and len(lim) == 2 and all(_es_num(v) for v in lim):
        return float(lim[0]), float(lim[1])
    return LIMITES_XY_DEF


def mision_envolvente(cfg):
    """Envolvente de las consignas xy, mismas reglas que ``envolvente`` de ``qd2_params_from_config.m`` y el
    recorte de ``qd2_limites_xy.m``. Devuelve ``{"x": [lo, hi]|None, "y": [lo, hi]|None, "lo": [x, y],
    "hi": [x, y], "definida": bool, "aviso": str|None}``:

    * ``x`` / ``y``: ``mision.limites_x`` / ``mision.limites_y`` aceptados (ausente, null o [] -> None sin aviso;
      dos números reales finitos (no bool) con lo < hi -> recortados a ``limites_xy`` SIN margen (aviso si se
      recortó; None con aviso si no queda nada); cualquier otra cosa -> None con aviso);
    * ``lo`` / ``hi``: límite EFECTIVO de las consignas por eje: la envolvente de ese eje TAL CUAL (el margen no se
      le aplica) o, sin ella, ``limites_xy`` -/+ ``XY_MARGEN`` (mismo cálculo que MATLAB: ±1.35 con ±1.5)."""
    m = cfg.get("mision") if isinstance(cfg, dict) else None
    m = m if isinstance(m, dict) else {}
    l0, l1 = mision_limites_xy(cfg)
    lo0, hi0 = l0 + XY_MARGEN, l1 - XY_MARGEN
    out = {"x": None, "y": None, "lo": [lo0, lo0], "hi": [hi0, hi0], "definida": False, "aviso": None}
    avisos = []
    for i, eje in enumerate(("x", "y")):
        clave = f"limites_{eje}"
        if clave not in m:
            continue
        raw = m[clave]
        if raw is None or (isinstance(raw, list) and not raw):
            continue                                  # null o [] = no definido
        if not (isinstance(raw, list) and len(raw) == 2 and all(_es_num(v) for v in raw)
                and float(raw[0]) < float(raw[1])):
            avisos.append(f"mision.{clave} inválido ('{_txt(raw)}'; se espera [lo, hi] con lo < hi): "
                          f"sin envolvente en ese eje")
            continue
        x = [float(raw[0]), float(raw[1])]
        r = [max(x[0], l0), min(x[1], l1)]
        if r[0] >= r[1]:
            avisos.append(f"mision.{clave} {_txt(raw)} queda fuera de mision.limites_xy [{l0:g}, {l1:g}]: "
                          f"sin envolvente en ese eje")
            continue
        if r != x:
            avisos.append(f"mision.{clave} {_txt(raw)} se sale de mision.limites_xy [{l0:g}, {l1:g}]: "
                          f"se usa [{r[0]:g}, {r[1]:g}]")
        out[eje] = r
        out["lo"][i], out["hi"][i] = r
        out["definida"] = True
    out["aviso"] = "; ".join(avisos) or None
    return out


def vertices_fuera_envolvente(cfg):
    """Vértices de la ruta (``pista.vertices``) fuera de la envolvente DEFINIDA (``mision.limites_x`` / ``_y``):
    la consigna del explorador se recorta y, si queda a más de ``tol_xy`` del waypoint, E nunca lo alcanza y se queda
    en hover ahí (MISSION no avanza). Lista de ``[x, y]`` (vacía sin envolvente)."""
    env = mision_envolvente(cfg)
    if not env["definida"]:
        return []
    pista = cfg.get("pista") if isinstance(cfg, dict) else None
    verts = (pista if isinstance(pista, dict) else {}).get("vertices") or []
    fuera = []
    for v in verts:
        if isinstance(v, list) and len(v) == 2 and all(_es_num(q) for q in v) and any(
                not (env["lo"][i] - 1e-9 <= float(v[i]) <= env["hi"][i] + 1e-9) for i in (0, 1)):
            fuera.append([float(v[0]), float(v[1])])
    return fuera


def puntos_fuera_limite_consignas(cfg, puntos):
    """Puntos ``[x, y]`` de ``puntos`` fuera del límite EFECTIVO de las consignas (``mision_envolvente`` lo/hi: la
    envolvente si existe; si no, ``limites_xy`` -/+ ``XY_MARGEN``). Para el aviso de la pestaña Pista al guardar la
    ruta (no bloquea): la ruta debe quedar dentro de la zona con rastreo."""
    env = mision_envolvente(cfg)
    fuera = []
    for v in puntos or []:
        if isinstance(v, (list, tuple)) and len(v) == 2 and all(_es_num(q) for q in v) and any(
                not (env["lo"][i] - 1e-9 <= float(v[i]) <= env["hi"][i] + 1e-9) for i in (0, 1)):
            fuera.append([float(v[0]), float(v[1])])
    return fuera


def _txt(v):
    """Valor de la config como se escribió en el JSON (para los avisos)."""
    if isinstance(v, _Num):
        return v.text
    try:
        return json.dumps(v, ensure_ascii=False)
    except (TypeError, ValueError):
        return str(v)


LOOPBACK = ("127.0.0.1", "localhost")


def es_loopback(host):
    return str(host or "").strip().lower() in LOOPBACK


def validar_host_puente(host):
    """Host del Stream Server :18700 pasado por ``--host-puente``: ``localhost``/``127.0.0.1`` o una IPv4.
    Lanza ``ValueError`` si no es válido."""
    h = str(host or "").strip()
    if h.lower() == "localhost":
        return "localhost"
    try:
        return str(ipaddress.IPv4Address(h))
    except ValueError:
        raise ValueError(f"--host-puente inválido: '{host}' (usa una IPv4, p. ej. 192.168.2.11, o 127.0.0.1)")


class _Num(float):
    """float que recuerda cómo estaba escrito en el archivo (para no cambiar 0.20 por 0.2)."""

    def __new__(cls, text):
        obj = float.__new__(cls, text)
        obj.text = text
        return obj

    def __deepcopy__(self, memo):
        return self

    def __reduce__(self):
        return (_Num, (self.text,))


def _fmt_scalar(v):
    if isinstance(v, _Num):
        return v.text
    if isinstance(v, bool):
        return "true" if v else "false"
    if v is None:
        return "null"
    if isinstance(v, float):
        if math.isnan(v) or math.isinf(v):
            return "null"
        r = repr(v)
        return r
    return json.dumps(v, ensure_ascii=False)


def _inline(v):
    """True si la lista se puede escribir en una sola línea (escalares o listas de escalares)."""
    if not isinstance(v, list):
        return False
    for x in v:
        if isinstance(x, dict):
            return False
        if isinstance(x, list) and any(isinstance(y, (list, dict)) for y in x):
            return False
    return True


def dumps(obj, indent=0):
    """Serializa como el ``estacion.json`` original (sangría 2, listas numéricas en línea)."""
    pad = "  " * indent
    if isinstance(obj, dict):
        if not obj:
            return "{}"
        if indent >= 3 and not any(isinstance(v, (dict, list)) for v in obj.values()):
            # diccionarios pequeños anidados (p. ej. intrínsecos) en una sola línea, como el original
            return "{" + ", ".join(f"{json.dumps(k, ensure_ascii=False)}: {_fmt_scalar(v)}"
                                   for k, v in obj.items()) + "}"
        items = [f'{pad}  {json.dumps(k, ensure_ascii=False)}: {dumps(v, indent + 1)}' for k, v in obj.items()]
        return "{\n" + ",\n".join(items) + "\n" + pad + "}"
    if isinstance(obj, list):
        if _inline(obj):
            return "[" + ", ".join(dumps(x, indent + 1) if isinstance(x, list) else _fmt_scalar(x)
                                   for x in obj) + "]"
        items = [pad + "  " + dumps(x, indent + 1) for x in obj]
        return "[\n" + ",\n".join(items) + "\n" + pad + "]"
    return _fmt_scalar(obj)


def deep_merge(base, patch):
    """Mezcla ``patch`` dentro de una copia de ``base`` (los diccionarios se mezclan, el resto se reemplaza)."""
    out = copy.deepcopy(base)
    for k, v in patch.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def validar_ip(ip):
    """'' (pendiente) o una IPv4 válida."""
    ip = (ip or "").strip()
    if ip == "":
        return ""
    try:
        return str(ipaddress.IPv4Address(ip))
    except ValueError:
        raise ValueError(f"IP inválida: '{ip}'")


class Config:
    """Acceso con candado a ``estacion.json``. ``data`` es una copia en memoria."""

    def __init__(self, path=DEFAULT_PATH):
        self.path = os.path.abspath(path)
        self.data = {}
        self.mtime = 0.0
        self.reload()

    # ------------------------------------------------------------------ lectura
    def _read_disk(self):
        with open(self.path, "rb") as fh:
            raw = fh.read()
        self._newline = "\r\n" if b"\r\n" in raw else "\n"
        return json.loads(raw.decode("utf-8-sig"), parse_float=_Num)

    def reload(self):
        with _LOCK:
            self.data = self._read_disk()
            self.mtime = os.path.getmtime(self.path)
            return self.data

    def reload_if_changed(self):
        try:
            if os.path.getmtime(self.path) != self.mtime:
                self.reload()
        except OSError:
            pass
        return self.data

    def get(self, *keys, default=None):
        d = self.data
        for k in keys:
            if not isinstance(d, dict) or k not in d:
                return default
            d = d[k]
        return d

    def web(self, key):
        return self.get("web", key, default=WEB_DEFAULTS.get(key))

    # ------------------------------------------------------------------ escritura
    def update(self, patch):
        """Aplica ``patch`` sobre la versión del disco y guarda de forma atómica."""
        with _LOCK:
            try:
                current = self._read_disk()
            except FileNotFoundError:
                current = {}
            merged = deep_merge(current, patch)
            text = dumps(merged) + "\n"
            tmp = self.path + ".tmp"
            with open(tmp, "w", encoding="utf-8", newline=getattr(self, "_newline", "\n")) as fh:
                fh.write(text)
            for intento in range(10):  # OneDrive o un editor pueden bloquear el archivo un instante
                try:
                    os.replace(tmp, self.path)
                    break
                except PermissionError:
                    if intento == 9:
                        raise
                    time.sleep(0.1)
            self.data = merged
            self.mtime = os.path.getmtime(self.path)
            return merged

    def ensure_web_defaults(self):
        """Agrega la sección ``web`` (o las claves que falten) sin tocar nada más.
        Ya no se llama al arrancar (para no reescribir el archivo); queda para uso manual."""
        web = self.data.get("web", {}) if isinstance(self.data.get("web"), dict) else {}
        faltan = {k: v for k, v in WEB_DEFAULTS.items() if k not in web}
        if faltan:
            self.update({"web": faltan})
