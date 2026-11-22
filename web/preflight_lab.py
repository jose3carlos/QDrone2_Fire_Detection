"""Revisión previa al día de laboratorio (solo lee; nunca escribe estacion.json).

Uso (desde la raíz del proyecto, con el venv de la estación):
    %USERPROFILE%\\.venvs\\qd2_web\\Scripts\\python web\\preflight_lab.py
    ... --version MPC          revisa también lo de la versión MPC (por defecto: version_control de la config)
    ... --ping                 hace ping a los dos drones
    ... --config otra.json     revisa otra config (por defecto config/estacion.json)

Imprime [OK] / [AVISO] / [FALLA] por punto y termina con código 1 si hay alguna FALLA.
Ver docs/CHECKLIST_LABORATORIO.md.
"""
from __future__ import annotations

import argparse
import hashlib
import ipaddress
import json
import os
import socket
import subprocess
import sys

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SQ = os.path.join(RAIZ, "simulink_qdrone2")
MD5_PID = "00d4d46e7a2e07158690fe98a9b756af"

res = {"OK": 0, "AVISO": 0, "FALLA": 0}


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


def main(argv=None):
    ap = argparse.ArgumentParser(description="Revisión previa al laboratorio (solo lectura)")
    ap.add_argument("--config", default=os.path.join(RAIZ, "config", "estacion.json"))
    ap.add_argument("--version", choices=["PID", "MPC"], default=None)
    ap.add_argument("--release", default="R2022a", help="carpeta de versiones/ que se va a abrir (R2022a, R2025a...)")
    ap.add_argument("--ping", action="store_true")
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
    ver = (a.version or str(get(c, "version_control", default="PID"))).upper()
    print(f"Versión revisada: {ver}")
    if str(get(c, "version_control", default="")).upper() != ver:
        rep("FALLA", f"version_control = {get(c, 'version_control')} en la config, pero vas a correr {ver}: cámbialo en la pestaña Misión")
    else:
        rep("OK", f"version_control = {ver}")

    # ---------------------------------------------------------------- modelos y binarios
    carpeta = os.path.join(SQ, "versiones", a.release)
    tierra = "QD2_MissionCtrl_Dual_%s.slx" % ver
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
            except Exception:
                pass

    # ---------------------------------------------------------------- cámara
    cam = get(c, "camara", default={})
    if cam.get("fuente") != "dron":
        rep("FALLA", f"camara.fuente = {cam.get('fuente')!r}: con drones reales debe ser 'dron'")
    else:
        rep("OK", "camara.fuente = dron")
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
        if not v:
            rep("AVISO", f"optitrack.{k} vacío: el modelo usa la ruta del Equipo2 del 20-nov (un solo rigid body). Vincula los del día en la pestaña Motive")
            continue
        p = v if (len(v) > 2 and (v[1] == ":" or v.startswith("\\\\"))) else os.path.join(o.get("carpeta", ""), v)
        rep("OK" if os.path.exists(p) else "FALLA", f"optitrack.{k}: {p} {'existe' if os.path.exists(p) else 'NO EXISTE en esta PC'}")
    try:
        import psutil  # noqa: WPS433
        motive = [pr.info["name"] for pr in psutil.process_iter(["name"]) if (pr.info["name"] or "").lower().startswith("motive")]
        if motive:
            rep("AVISO", "Motive está abierto: ciérralo antes de correr el modelo de tierra (el bloque OptiTrack usa la API de Motive)")
        else:
            rep("OK", "Motive cerrado")
    except Exception:
        pass

    # ---------------------------------------------------------------- pista
    v = get(c, "pista", "vertices", default=[])
    lim = get(c, "mision", "limites_xy", default=[-1.5, 1.5])
    if not (2 <= len(v) <= 16):
        rep("FALLA", f"pista con {len(v)} vértices (Simulink acepta 2..16)")
    elif any(not (lim[0] <= q <= lim[1]) for p in v for q in p):
        rep("FALLA", f"algún vértice de la pista fuera de {lim}")
    else:
        rep("OK", f"pista: {len(v)} vértices dentro de {lim}")

    # ---------------------------------------------------------------- red
    red = get(c, "red", default={})
    ips = [red.get("ip_dron1") or "", red.get("ip_dron2") or ""]
    if not all(ips):
        rep("FALLA", "faltan ip_dron1 / ip_dron2 (pestaña Conexiones)")
    elif ips[0] == ips[1]:
        rep("FALLA", "ip_dron1 = ip_dron2")
    else:
        rep("OK", f"drones {ips[0]} / {ips[1]}")
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from app import netinfo  # noqa: WPS433
        ifs = netinfo.interfaces(red.get("interfaz_preferida", "Wi-Fi 2"))
        ip_pc = netinfo.ip_recomendada(c)
        for r in ifs:
            marca = " <- Alfa/recomendada" if r["recomendada"] else ""
            print(f"         interfaz {r['nombre']}: {r['ip']} ({r['descripcion']}){marca}")
        if not ip_pc:
            rep("FALLA", "no encuentro la IP de la PC en la red de los drones (antena Alfa 'Wi-Fi 2' sin IP)")
        else:
            rep("OK", f"IP de la PC para -URI_Host: {ip_pc}")
            if all(ips):
                try:
                    red_pc = ipaddress.ip_network(ip_pc + "/24", strict=False)
                    fuera = [ip for ip in ips if ipaddress.ip_address(ip) not in red_pc]
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
        if puerto_libre(p):
            rep("OK", f"puerto {p} libre ({quien})")
        else:
            rep("AVISO", f"puerto {p} ocupado ({quien}): normal si el modelo de tierra o la estación ya corren; si no, ciérralo (netstat -ano | findstr :{p})")

    if a.ping and all(ips):
        for i, ip in enumerate(ips, 1):
            ok = subprocess.run(["ping", "-n", "2", "-w", "1000", ip], capture_output=True).returncode == 0
            rep("OK" if ok else "FALLA", f"ping dron {i} ({ip}) {'responde' if ok else 'NO responde'}")

    # ---------------------------------------------------------------- QUARC
    qd = os.environ.get("QUARC_DIR", "")
    qr = os.path.join(qd, "quarc_run.exe") if qd else ""
    if qr and os.path.exists(qr):
        rep("OK", f"quarc_run: {qr}")
    else:
        rep("FALLA", "no encuentro quarc_run.exe (QUARC_DIR): instala/activa QUARC en esta PC")

    print(f"\nResumen: {res['OK']} OK, {res['AVISO']} avisos, {res['FALLA']} fallas")
    return 1 if res["FALLA"] else 0


if __name__ == "__main__":
    sys.exit(main())
