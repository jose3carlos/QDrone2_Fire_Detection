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

Imprime [OK] / [INFO] / [AVISO] / [FALLA] por punto y termina con código 1 si hay alguna FALLA.
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
        if not v:
            rep("AVISO", f"optitrack.{k} vacío: el modelo usa la ruta del Equipo2 del 20-nov (un solo rigid body). Vincula los del día en la pestaña Motive")
            continue
        p = v if (len(v) > 2 and (v[1] == ":" or v.startswith("\\\\"))) else os.path.join(o.get("carpeta", ""), v)
        if a.remoto:
            rep("INFO", f"optitrack.{k}: {p} (ruta de la PC {a.remoto}: no se verifica desde aquí)")
            continue
        rep("OK" if os.path.exists(p) else "FALLA", f"optitrack.{k}: {p} {'existe' if os.path.exists(p) else 'NO EXISTE en esta PC'}")
    if solo:
        rep("INFO", f"rigid body del explorador (dron {ds}) = {o.get(f'rigid_body_dron{ds}')}; el del dron {3 - ds} "
                    "debe ser distinto y ese dron debe quedar fuera de la arena")
    try:
        if a.remoto:
            raise RuntimeError("remoto")
        import psutil  # noqa: WPS433
        motive = [pr.info["name"] for pr in psutil.process_iter(["name"]) if (pr.info["name"] or "").lower().startswith("motive")]
        if motive:
            rep("AVISO", "Motive está abierto: ciérralo antes de correr el modelo de tierra (el bloque OptiTrack usa la API de Motive)")
        else:
            rep("OK", "Motive cerrado")
    except Exception:
        if a.remoto:
            rep("INFO", f"Motive corre en la PC {a.remoto}: ciérralo allá antes de correr el modelo de tierra")

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
