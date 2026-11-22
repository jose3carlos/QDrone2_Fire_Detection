"""Prueba punta a punta (E2E) de la estación web REAL contra QD2_Sim_Dual o el simulador 3D.

Levanta ``app.server`` (el mismo proceso que abre el operador), lee el estado por el WebSocket ``/ws``
(lo mismo que dibuja la página) y maneja la misión con la API HTTP de los botones:
START → objetivo manual → PAUSE → RESUME → LAND ALL en home. Toma capturas de la UI con Chrome
headless (misión en curso, objetivo visitado, alerta, DONE) y al final revisa cada corrida.

Lo de MATLAB se corre aparte, en otra terminal (ver docs/PRUEBA_E2E.md):
  * ``simulink_qdrone2/sim/qd2_e2e_web.m``  (QD2_Sim_Dual en tiempo real, COMMAND SOURCE = 2)
  * ``simulador3d/sim3d_e2e_web.m``         (simulador 3D, escenario 'web', use_vision_python)

Uso (desde la carpeta web, con el venv qd2_web):
  python e2e\\e2e_estacion.py --fuente simulink --corridas pid,mpc,falla
  python e2e\\e2e_estacion.py --fuente sim3d --corridas PID,MPC

La configuración de la prueba es una COPIA de config/estacion.json en la carpeta de salida: la real no se
toca. En la copia la cámara es la sintética (anomalías inventadas de mock_world); con la fuente simulink se
habilita ``web.permitir_sintetica_en_simulink`` SOLO en esa copia, porque QD2_Sim_Dual es una simulación.
Nunca uses esa copia con los drones reales.

Código de salida: 0 si todas las corridas pasan las comprobaciones, 1 si alguna falla.
"""
import argparse
import asyncio
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request

try:
    import websockets
except ImportError:  # pragma: no cover
    sys.exit("Falta el paquete 'websockets' (está en el venv qd2_web: web/requirements.txt).")

AQUI = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.dirname(AQUI)
PROJECT = os.path.dirname(WEB)
CHROMES = [r"C:\Program Files\Google\Chrome\Application\chrome.exe",
           r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
           os.path.expandvars(r"%LOCALAPPDATA%\Google\Chrome\Application\chrome.exe"),
           r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"]
# Alerta y(55) esperada por corrida (las demás corridas deben terminar sin alertas)
ALERTA_ESPERADA = {"falla": 9}
TOL_HOME = 0.15   # m: los drones deben aterrizar a menos de esto de su home (salvo una corrida con falla)

ap = argparse.ArgumentParser(description="Prueba E2E de la estación web real")
ap.add_argument("--fuente", choices=("simulink", "sim3d"), default="simulink")
ap.add_argument("--corridas", default="pid,mpc,falla",
                help="nombres de las corridas en el orden en que las corre MATLAB (uno por reinicio de t)")
ap.add_argument("--puerto-web", type=int, default=8090, help="puerto HTTP de la estación de prueba")
ap.add_argument("--salida", default=os.path.join(tempfile.gettempdir(), "qd2_e2e"),
                help="carpeta para la config de prueba, bitácoras, resumen y capturas")
ap.add_argument("--timeout", type=float, default=1800, help="s de reloj como máximo")
ap.add_argument("--pausa", default="pid,mpc,PID,MPC", help="corridas con PAUSE/RESUME")
ap.add_argument("--manual", default="0.8,-0.5,1", help="objetivo manual x,y,clase ('' = ninguno)")
ap.add_argument("--sin-capturas", action="store_true")
ap.add_argument("--ventana", default="1680,1050", help="tamaño de la ventana de Chrome headless")
a = ap.parse_args()

OUT = os.path.abspath(a.salida)
CAP = os.path.join(OUT, "capturas")
os.makedirs(CAP, exist_ok=True)
BASE = f"http://127.0.0.1:{a.puerto_web}"
CORRIDAS = [c for c in a.corridas.split(",") if c]
PAUSA = set(a.pausa.split(",")) if a.pausa else set()
CHROME = next((c for c in CHROMES if os.path.exists(c)), None)

# ---------------------------------------------------------------- config de prueba (copia)
cfg_path = os.path.join(OUT, "estacion_e2e.json")
with open(os.path.join(PROJECT, "config", "estacion.json"), encoding="utf-8") as fh:
    cfg = json.load(fh)
web = cfg.setdefault("web", {})
cfg["camara"]["fuente"] = "sintetica"
if a.fuente == "simulink":
    web["permitir_sintetica_en_simulink"] = True
web["fuente"] = a.fuente
with open(cfg_path, "w", encoding="utf-8") as fh:
    json.dump(cfg, fh, indent=2, ensure_ascii=False)
TOL_XY = float(cfg.get("mision", {}).get("tol_xy", 0.12))
p_obj = os.path.join(OUT, "objetivos_estacion.json")
if os.path.exists(p_obj):
    os.remove(p_obj)

LOG = open(os.path.join(OUT, f"e2e_{a.fuente}.log"), "w", encoding="utf-8")
T0 = time.time()


def log(msg):
    line = f"[{time.time() - T0:7.1f}s] {msg}"
    print(line, flush=True)
    LOG.write(line + "\n")
    LOG.flush()


def post(path, body):
    req = urllib.request.Request(BASE + path, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.load(r)
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode(errors="replace")


hilos_captura = []


def captura(nombre, hash_=""):
    """Chrome headless sobre la página real, en un hilo (el lazo del WebSocket no se detiene)."""
    if a.sin_capturas or not CHROME:
        return
    destino = os.path.join(CAP, nombre + ".png")
    perfil = os.path.join(OUT, "chrome_perfil_" + nombre)
    cmd = [CHROME, "--headless=new", "--hide-scrollbars", f"--window-size={a.ventana}",
           "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist",
           f"--user-data-dir={perfil}", "--virtual-time-budget=9000", f"--screenshot={destino}",
           f"{BASE}/{('#' + hash_) if hash_ else ''}"]

    def correr():
        t = time.time()
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
        except subprocess.TimeoutExpired:
            log(f"captura {nombre}: Chrome no terminó en 60 s")
        shutil.rmtree(perfil, ignore_errors=True)
        log(f"captura {nombre} -> {destino} ({'ok' if os.path.exists(destino) else 'FALLÓ'}, {time.time() - t:.1f} s)")

    th = threading.Thread(target=correr, daemon=True)
    th.start()
    hilos_captura.append(th)


# ---------------------------------------------------------------- estación web real
srv_log = open(os.path.join(OUT, f"servidor_{a.fuente}.log"), "w", encoding="utf-8")
srv = subprocess.Popen([sys.executable, "-m", "app.server", "--config", cfg_path, "--fuente", a.fuente,
                        "--puerto", str(a.puerto_web)], cwd=WEB, stdout=srv_log, stderr=subprocess.STDOUT,
                       env=dict(os.environ, QD2_NO_BROWSER="1", PYTHONIOENCODING="utf-8"))
log(f"estación web (pid {srv.pid}) en {BASE}, fuente {a.fuente}, config de prueba {cfg_path}")
log("Chrome: " + (CHROME or "no encontrado (sin capturas)"))
for _ in range(60):
    try:
        urllib.request.urlopen(BASE + "/api/estado", timeout=2).read()
        break
    except Exception:
        time.sleep(0.5)
else:
    log("la estación no arrancó: revisa " + srv_log.name)
    srv.kill()
    sys.exit(1)


def nueva(nombre):
    return {"nombre": nombre, "estados": [], "alertas": {}, "fsm": {"1": set(), "2": set()},
            "fsm_falla": {"1": set(), "2": set()}, "t_estado": {}, "start": False, "pausa": None, "resume": False,
            "land": False, "manual": None, "caps": set(), "obj": {}, "visit_t": {}, "n_ws": 0, "t_ws0": None,
            "done": None, "cmd_http": [], "fin": None, "version": None, "avisos": set(), "dh_min_aire": None}


async def lazo():
    idx, cur, res, last_t, ev_vistos = -1, None, [], None, set()
    async with websockets.connect(f"ws://127.0.0.1:{a.puerto_web}/ws", max_size=None) as ws:
        while time.time() - T0 < a.timeout:
            s = json.loads(await ws.recv())
            for e in reversed(s.get("eventos") or []):
                if (e["t"], e["msg"]) not in ev_vistos:
                    ev_vistos.add((e["t"], e["msg"]))
                    log("EVENTO " + e["msg"])
            tel = s.get("tel")
            if not tel:
                continue
            t = tel["t"] or 0.0
            if last_t is None or t < last_t - 1.0:          # MATLAB empezó otra corrida (t vuelve a 0)
                if cur is not None:
                    res.append(cur)
                idx += 1
                if idx >= len(CORRIDAS):
                    log("llegó una corrida más de las esperadas; termino")
                    cur = None
                    break
                cur = nueva(CORRIDAS[idx])
                log(f"===== corrida {cur['nombre']} (t = {t:.1f} s)")
            last_t = t
            c, now = cur, time.time()
            c["n_ws"] += 1
            c["t_ws0"] = c["t_ws0"] or now
            c["version"] = tel["version_name"]
            for av in s.get("avisos") or []:
                if av["msg"] not in c["avisos"]:
                    c["avisos"].add(av["msg"])
                    log(f"AVISO {av['nivel']}: {av['msg']}")
            st = tel["mission_state_name"]
            if not c["estados"] or c["estados"][-1] != st:
                c["estados"].append(st)
                c["t_estado"][st] = t
                log(f"estado {st} (t_sim {t:.2f})")
            al = tel["alerta"]
            if al and al not in c["alertas"]:
                c["alertas"][al] = round(t, 2)
                log(f"ALERTA y(55) = {al} ({tel['alerta_nivel']}): {tel['alerta_texto']} (t_sim {t:.2f})")
                if "alerta" not in c["caps"]:
                    c["caps"].add("alerta")
                    captura(f"{a.fuente}_{c['nombre']}_alerta{al}")
            drones = tel["drones"]
            for d in ("1", "2"):
                dd = drones[d]
                k = (dd["fsm_state"], dd["fsm_texto"])
                if k not in c["fsm"][d]:
                    c["fsm"][d].add(k)
                    log(f"dron {d}: {dd['fsm_tipo']} = {dd['fsm_state']} '{dd['fsm_texto']}'"
                        f"{' FALLA' if dd['fsm_falla'] else ''} (misión {st})")
                if dd["fsm_falla"]:
                    c["fsm_falla"][d].add(dd["fsm_state"])
            d1, d2 = drones["1"], drones["2"]
            if None not in (d1["z"], d2["z"]) and min(d1["z"], d2["z"]) > 0.2 and tel["dist_horizontal"] is not None:
                c["dh_min_aire"] = min(c["dh_min_aire"] or 99.0, tel["dist_horizontal"])
            for o in s["objetivos"]:
                fila = {k: o.get(k) for k in ("id", "clase", "origen", "estado", "x", "y", "alias_de", "manual")}
                prev = c["obj"].get(o["id"])
                if prev is None or (prev["estado"], prev["origen"]) != (fila["estado"], fila["origen"]):
                    log(f"objetivo #{fila['id']} {fila['clase']} ({fila['x']:+.2f}, {fila['y']:+.2f}) "
                        f"{fila['estado']} · origen {fila['origen']}{' (manual)' if fila['manual'] else ''}")
                c["obj"][o["id"]] = fila
                if o["estado"] == "visitada" and o["id"] not in c["visit_t"]:
                    c["visit_t"][o["id"]] = round(t, 2)
                    if "visitado" not in c["caps"]:
                        c["caps"].add("visitado")
                        captura(f"{a.fuente}_{c['nombre']}_objetivo_visitado")
                        captura(f"{a.fuente}_{c['nombre']}_tabla_objetivos", "anomalias")
            # ------------------------------------------------- lo que haría el operador en la UI
            if st == "ASSIGN_ROLES" and not c["start"] and t - c["t_estado"]["ASSIGN_ROLES"] > 2.0:
                c["cmd_http"].append(("START",) + post("/api/cmd", {"cmd": "START"}))
                c["start"] = True
                log(f"HTTP START -> {c['cmd_http'][-1][1:]}")
            if st == "MISSION":
                tm = t - c["t_estado"]["MISSION"]
                if a.manual and c["manual"] is None and tm > 4.0:
                    x, y, cls = (float(v) for v in a.manual.split(","))
                    c["manual"] = post("/api/objetivos/manual", {"x": x, "y": y, "cls": int(cls)})
                    log(f"HTTP objetivo manual ({x:+.2f}, {y:+.2f}, clase {int(cls)}) -> {c['manual']}")
                if c["nombre"] in PAUSA and c["pausa"] is None and tm > 15.0:
                    c["cmd_http"].append(("PAUSE",) + post("/api/cmd", {"cmd": "PAUSE"}))
                    c["pausa"] = now
                    log(f"HTTP PAUSE -> {c['cmd_http'][-1][1:]}")
                if "mision" not in c["caps"] and tm > 22.0:
                    c["caps"].add("mision")
                    captura(f"{a.fuente}_{c['nombre']}_mision_en_curso")
            if st == "PAUSED" and c["pausa"] and not c["resume"] and now - c["pausa"] > 5.0:
                c["cmd_http"].append(("RESUME",) + post("/api/cmd", {"cmd": "RESUME"}))
                c["resume"] = True
                log(f"HTTP RESUME -> {c['cmd_http'][-1][1:]}")
            # LAND ALL como lo daría el operador: con los dos "en home" (< mision.tol_xy, lo que marca la
            # tarjeta de cada dron) durante 2 s seguidos. En LANDING cada dron baja EN SU SITIO.
            en_home = all(dd["x"] is not None and math.hypot(dd["x"] - dd["home_x"], dd["y"] - dd["home_y"]) <= TOL_XY
                          for dd in (d1, d2))
            c["t_en_home"] = (c.get("t_en_home") or now) if (st == "RETURN_HOME" and en_home) else None
            if st == "RETURN_HOME" and not c["land"] and c["t_en_home"] and now - c["t_en_home"] >= 2.0:
                c["cmd_http"].append(("LAND_ALL",) + post("/api/cmd", {"cmd": "LAND_ALL"}))
                c["land"] = True
                log(f"HTTP LAND_ALL -> {c['cmd_http'][-1][1:]}")
            if st == "DONE" and c["done"] is None:
                c["done"] = round(t, 2)
                c["fin"] = {"drones": {d: {k: drones[d][k] for k in ("x", "y", "z", "home_x", "home_y")}
                                       for d in ("1", "2")},
                            "tel": {k: tel[k] for k in ("n_targets_known", "n_targets_visited", "visited_ids",
                                                        "explorer_wp_idx", "explorer_wp_total")},
                            "cmd_resultados": [(r["cmd"], r["estado"]) for r in s["puente"]["cmd_resultados"]],
                            "ws_hz": c["n_ws"] / max(now - c["t_ws0"], 1e-6)}
                log(f"DONE (t_sim {t:.2f}): {c['fin']}")
                captura(f"{a.fuente}_{c['nombre']}_done")
                if idx == len(CORRIDAS) - 1:
                    await asyncio.sleep(10)
                    break
    if cur is not None:
        res.append(cur)
    return res


def revisar(c):
    """Comprobaciones de una corrida. Devuelve la lista de fallas (vacía = pasa)."""
    f = []
    if c["done"] is None:
        f.append("no llegó a DONE")
    esperada = ALERTA_ESPERADA.get(c["nombre"])
    if esperada is None and c["alertas"]:
        f.append(f"alertas inesperadas {c['alertas']}")
    if esperada is not None and esperada not in c["alertas"]:
        f.append(f"no apareció la alerta {esperada} (alertas: {c['alertas']})")
    propios = [o for o in c["obj"].values() if o["origen"] != "simulink"]
    if not propios:
        f.append("la estación no publicó ningún objetivo")
    if any(o["origen"] == "estacion" for o in propios):
        f.append("objetivos publicados que Simulink nunca confirmó en y(56:60): "
                 + str([o["id"] for o in propios if o["origen"] == "estacion"]))
    if esperada is None:
        no_vis = [o["id"] for o in c["obj"].values() if o["estado"] != "visitada"]
        if no_vis:
            f.append(f"objetivos sin visitar al final: {no_vis}")
    if c["fin"]:
        for d, dd in c["fin"]["drones"].items():
            if dd["z"] is None or dd["z"] > 0.1:
                f.append(f"dron {d} no quedó en el suelo (z = {dd['z']})")
            elif esperada is None and math.hypot(dd["x"] - dd["home_x"], dd["y"] - dd["home_y"]) > TOL_HOME:
                f.append(f"dron {d} aterrizó lejos de su home")
        malos = [r for r in c["fin"]["cmd_resultados"] if r[1] != "recibido"]
        if malos:
            f.append(f"comandos no recibidos: {malos}")
        if c["fin"]["ws_hz"] < 10:
            f.append(f"WebSocket a {c['fin']['ws_hz']:.1f} Hz (< 10)")
    if any(r[1] != 200 for r in c["cmd_http"]):
        f.append(f"respuestas HTTP: {[(r[0], r[1]) for r in c['cmd_http']]}")
    if a.manual and (c["manual"] is None or c["manual"][0] != 200):
        f.append(f"objetivo manual: {c['manual']}")
    if c["nombre"] in PAUSA and not c["resume"]:
        f.append("no se probó PAUSE/RESUME")
    if esperada is None and (c["fsm_falla"]["1"] or c["fsm_falla"]["2"]):
        f.append(f"la UI marcó una falla del dron: {c['fsm_falla']}")
    return f


try:
    resultados = asyncio.run(lazo())
finally:
    for th in hilos_captura:
        th.join(timeout=70)
    srv.terminate()
    try:
        srv.wait(10)
    except subprocess.TimeoutExpired:
        srv.kill()

ok = True
salida = []
for c in resultados:
    fallas = revisar(c)
    ok &= not fallas
    log(f"RESUMEN {c['nombre']} ({c['version']}): {'PASA' if not fallas else 'FALLA'} | estados "
        f"{' -> '.join(c['estados'])} | alertas {c['alertas']} | visitados (t_sim) {c['visit_t']} | "
        f"objetivos {[(o['id'], o['clase'], o['origen'], o['estado']) for o in c['obj'].values()]} | "
        f"dh mín en el aire {c['dh_min_aire']} | DONE {c['done']}")
    for x in fallas:
        log(f"   - {x}")
    c = dict(c, fallas=fallas, fsm={d: sorted(v, key=str) for d, v in c["fsm"].items()})
    salida.append(c)
if len(resultados) < len(CORRIDAS):
    ok = False
    log(f"solo se vieron {len(resultados)} de {len(CORRIDAS)} corridas")
with open(os.path.join(OUT, f"resumen_{a.fuente}.json"), "w", encoding="utf-8") as fh:
    json.dump(salida, fh, indent=1, ensure_ascii=False,
              default=lambda o: sorted(o, key=str) if isinstance(o, set) else str(o))
log("RESULTADO: " + ("todas las corridas PASAN" if ok else "hay FALLAS (ver arriba)"))
LOG.close()
sys.exit(0 if ok else 1)
