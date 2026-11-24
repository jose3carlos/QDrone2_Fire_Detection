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
