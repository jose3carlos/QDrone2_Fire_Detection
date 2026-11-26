"""Interfaces de red de la PC y comandos de lanzamiento de los drones (ARQUITECTURA §6)."""
from __future__ import annotations

import json
import os
import socket
import subprocess
import time

try:
    import psutil
except Exception:  # pragma: no cover
    psutil = None

_CACHE = {"t": 0.0, "desc": {}}
ALFA_HINTS = ("8811", "alfa", "awus")


def _descriptions():
    """Nombre -> descripción del adaptador (Windows, vía Get-NetAdapter). Se cachea 60 s."""
    if os.name != "nt":
        return {}
    if time.time() - _CACHE["t"] < 60:
        return _CACHE["desc"]
    desc = {}
    try:
        out = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command",
             "Get-NetAdapter | Select-Object Name,InterfaceDescription,Status | ConvertTo-Json -Compress"],
            capture_output=True, text=True, timeout=8, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        data = json.loads(out.stdout or "[]")
        if isinstance(data, dict):
            data = [data]
        for a in data:
            desc[a.get("Name", "")] = {"descripcion": a.get("InterfaceDescription", ""), "estado": a.get("Status", "")}
    except Exception:
        pass
    _CACHE.update(t=time.time(), desc=desc)
    return desc


def interfaces(preferida="Wi-Fi 2"):
    """Lista de interfaces IPv4 con marca de la recomendada (antena Alfa / 'Wi-Fi 2')."""
    out = []
    if psutil is None:
        try:
            ip = socket.gethostbyname(socket.gethostname())
            return [{"nombre": "?", "ip": ip, "descripcion": "", "activa": True, "recomendada": False, "alfa": False}]
        except OSError:
            return []
    stats = psutil.net_if_stats()
    desc = _descriptions()
    for name, addrs in psutil.net_if_addrs().items():
        for a in addrs:
            if a.family != socket.AF_INET or a.address.startswith("127."):
                continue
            d = desc.get(name, {})
            text = (name + " " + d.get("descripcion", "")).lower()
            alfa = any(h in text for h in ALFA_HINTS)
            out.append({
                "nombre": name, "ip": a.address, "mascara": a.netmask,
                "descripcion": d.get("descripcion", ""),
                "activa": bool(stats.get(name).isup) if stats.get(name) else False,
                "alfa": alfa, "recomendada": alfa or name == preferida,
                "link_local": a.address.startswith("169.254."),
            })
    out.sort(key=lambda r: (not r["recomendada"], not r["activa"], r["link_local"], r["nombre"]))
    return out


def _loopback(host):
    return str(host or "").strip().lower() in ("127.0.0.1", "localhost")


def ip_recomendada(cfg, host_puente=None):
    """IP de la PC del modelo de tierra para ``-URI_Host`` de los drones:
    1) ``red.ip_pc`` si existe; 2) ``host_puente`` si no es loopback (la estación corre en otra máquina y el
    modelo de tierra en esa PC: nunca la IP de esta laptop); 3) la interfaz recomendada de esta PC."""
    red = cfg.get("red", {})
    if red.get("ip_pc"):
        return red["ip_pc"]
    if host_puente and not _loopback(host_puente):
        return str(host_puente).strip()
    for r in interfaces(red.get("interfaz_preferida", "Wi-Fi 2")):
        if r["recomendada"] and r["activa"] and not r["link_local"]:
            return r["ip"]
    return ""


def modelo_dron(cfg):
    return "QD2_DroneStack_MPC" if str(cfg.get("version_control", "PID")).upper() == "MPC" else "QD2_DroneStack_PID_2021a"


def comandos(cfg, ip_pc=None, modo=None, host_puente=None):
    """Comandos listos para copiar, por dron. Las IPs vacías quedan como ``<IP_DRONi>`` (pendiente).

    ``modo`` = ``config.mision_modo(cfg)`` (opcional): en solo explorador el dron que no se usa lleva
    ``no_usado = True`` (la UI no muestra su ``quarc_run``). ``host_puente`` = host del puente :18700 (con
    un host remoto, ``-URI_Host`` apunta a esa PC si ``red.ip_pc`` está vacía)."""
    red = cfg.get("red", {})
    web = cfg.get("web", {})
    ip_pc = ip_pc if ip_pc is not None else ip_recomendada(cfg, host_puente)
    modo = modo or {"modo": "dual", "dron_solo": 0}
    solo = modo.get("modo") == "solo_explorador" and modo.get("dron_solo") in (1, 2)
    pc = ip_pc or "<IP_PC>"
    tport = int(web.get("puerto_quarc_target", 17000))
    eport = int(web.get("puerto_ext_mode", 17001))
    model = modelo_dron(cfg)
    exe = model + ".rt-linux_qdrone2"
    out = {"ip_pc": ip_pc, "modelo": model, "drones": {},
           "modo": modo.get("modo", "dual"), "dron_solo": modo.get("dron_solo", 0) if solo else 0}
    for i in (1, 2):
        ip = red.get(f"ip_dron{i}", "") or ""
        host = ip or f"<IP_DRON{i}>"
        port = int(red.get(f"puerto_dron{i}", 18000 + i))
        uri_host = f"tcpip://{pc}:{port}"
        out["drones"][i] = {
            "ip": ip, "pendiente": not ip or not ip_pc,
            "no_usado": bool(solo and i != modo.get("dron_solo")),
            "uri_host": f"-URI_Host {uri_host}",
            "quarc_run": f"quarc_run -r -t tcpip://{host}:{tport} {exe} -uri tcpip://{host}:{eport}?retries=10 -URI_Host {uri_host}",  # -r = descargar y ejecutar (-D y -r son alternativas en QUARC); ?retries=10 = default del _main.c original
            "quarc_stop": f"quarc_run -q -t tcpip://{host}:{tport} {exe}",
            # Mismo formato que el modelo del dron del Equipo2: lista de cadenas de MATLAB entre comillas
            # ('<args>','<URI del modelo>'); dentro del set_param las comillas van dobladas.
            "ext_mode_args": (f"set_param('{model}', 'ExtModeMexArgs', '''-w -d /tmp -uri %u -URI_Host {uri_host}'','"
                              f"'tcpip://{host}:{eport}?retries=10''')"),
            "ping": f"ping {host}",
            "scp_camara": f"scp web/drone/qdrone2_camera_streamer.py web/drone/correr_streamer.sh nvidia@{host}:~/",
            # correr_streamer.sh: en segundo plano (sobrevive al cierre de ssh), lo relanza si muere, log en el dron
            "ssh_camara": f"ssh nvidia@{host} \"bash ~/correr_streamer.sh --camara auto --puerto {int(red.get('puerto_video', 18800))}\"",
            "ssh_camara_estado": f"ssh nvidia@{host} \"bash ~/correr_streamer.sh --estado\"",
            "ssh_camara_detener": f"ssh nvidia@{host} \"bash ~/correr_streamer.sh --detener\"",
        }
    return out
