"""Prueba punta a punta (E2E) de la estación web REAL contra QD2_Sim_Dual o el simulador 3D.

Levanta ``app.server`` (el mismo proceso que abre el operador), lee el estado por el WebSocket ``/ws``
(lo mismo que dibuja la página) y maneja la misión con la API HTTP de los botones:
START → objetivo manual → PAUSE → RESUME → LAND ALL en home. Toma capturas de la UI con Chrome
headless (misión en curso, objetivo visitado, alerta, DONE) y al final revisa cada corrida.

Actitud del explorador (y(61:63)): cuenta la telemetría con y(63) = 1, la inclinación máxima que muestra la
UI (``actitud_explorador``), los frames geolocalizados con / sin actitud, las detecciones descartadas por
inclinación y el error de geolocalización contra la VERDAD de la cámara sintética (``mock_world.anomalias_mock``:
``web.mock_anomalias`` o las anomalías por defecto, sobre los tubos reales ``pista.tubos`` si la config los trae):
por observación (cada detección que entra al cluster) y por objetivo
publicado. ``--sin-actitud`` corre lo mismo con ``vision.usar_actitud = false`` (para comparar) y
``--actitud-esperada si|no`` exige y(63) = 1 en MISSION o y(63) = 0 siempre (modelo viejo).

Lo de MATLAB se corre aparte, en otra terminal (ver docs/PRUEBA_E2E.md):
  * ``simulink_qdrone2/sim/qd2_e2e_web.m``  (QD2_Sim_Dual en tiempo real, COMMAND SOURCE = 2)
  * ``simulador3d/sim3d_e2e_web.m``         (simulador 3D, escenario 'web', use_vision_python)

Modo SOLO EXPLORADOR (``--modo solo [--dron-solo N]``, en la COPIA de la config): un solo dron. Se espera DONE
con el dron solo en el suelo cerca de su home, y(64) = N en toda la telemetría, el otro dron siempre en el piso
(z <= 0.1 m o sin pose), objetivos "detectada" y NINGUNO visitado (no hay respondedor). ``--fuente mock`` corre
lo mismo contra el mock interno de la estación (sin MATLAB).

Uso (desde la carpeta web, con el venv qd2_web):
  python e2e\\e2e_estacion.py --fuente simulink --corridas pid,mpc,falla
  python e2e\\e2e_estacion.py --fuente sim3d --corridas PID,MPC
  python e2e\\e2e_estacion.py --fuente sim3d --corridas PID --modo solo
  python e2e\\e2e_estacion.py --fuente mock --corridas solo --modo solo --velocidad-mock 2
  python e2e\\e2e_estacion.py --fuente simulink --corridas pid,mpc --sin-actitud --salida %TEMP%\\qd2_e2e_nivelado

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
ap.add_argument("--fuente", choices=("simulink", "sim3d", "mock"), default="simulink")
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
ap.add_argument("--sin-actitud", action="store_true",
                help="vision.usar_actitud = false en la copia de la config (geolocalización nivelada)")
ap.add_argument("--inclinacion-max", type=float, default=None,
                help="vision.inclinacion_max_deg en la copia (p. ej. 1.5 para ejercitar el descarte por inclinación)")
ap.add_argument("--actitud-esperada", choices=("si", "no", ""), default="",
                help="si = exige y(63) = 1 en MISSION y la actitud en la UI; no = exige y(63) = 0 siempre "
                     "(modelo viejo) y el aviso amarillo; '' = no lo revisa")
ap.add_argument("--modo", choices=("", "dual", "solo"), default="",
                help="mision.modo en la copia de la config: solo = solo_explorador; '' = el de config/estacion.json")
ap.add_argument("--dron-solo", type=int, choices=(1, 2), default=1, help="dron del modo solo (mision.dron_solo)")
ap.add_argument("--velocidad-mock", type=float, default=1.0, help="solo con --fuente mock")
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
if a.sin_actitud:
    cfg.setdefault("vision", {})["usar_actitud"] = False
if a.inclinacion_max is not None:
    cfg.setdefault("vision", {})["inclinacion_max_deg"] = a.inclinacion_max
if a.modo == "solo":
    cfg.setdefault("mision", {}).update(modo="solo_explorador", dron_solo=a.dron_solo)
elif a.modo == "dual":
    cfg.setdefault("mision", {})["modo"] = "dual"
    cfg["mision"].pop("dron_solo", None)
with open(cfg_path, "w", encoding="utf-8") as fh:
    json.dump(cfg, fh, indent=2, ensure_ascii=False)
TOL_XY = float(cfg.get("mision", {}).get("tol_xy", 0.12))
RADIO = float(cfg.get("vision", {}).get("radio_cluster_m", 0.25) or 0.25)
# verdad de la cámara sintética: lo mismo que dibuja la estación (server.Station._start_source)
sys.path.insert(0, WEB)
from app.config import mision_modo  # noqa: E402
from app.mock_world import anomalias_mock  # noqa: E402
MODO = mision_modo(cfg)
SOLO = MODO["dron_solo"] if MODO["modo"] == "solo_explorador" else 0    # dron solo (0 = dual)
USADOS = ("1", "2") if not SOLO else (str(SOLO),)
VERDAD = anomalias_mock(cfg)    # sobre pista.tubos (tubos reales) si los hay; el dron vuela pista.vertices


def err_verdad(cls, x, y):
    """(distancia, índice) a la anomalía verdadera más cercana de la misma clase, o (None, None)."""
    return min(((math.hypot(x - v["x"], y - v["y"]), i) for i, v in enumerate(VERDAD) if int(v["cls"]) == int(cls)),
               default=(None, None))


def stats(v):
    if not v:
        return None
    v = sorted(v)
    return {"n": len(v), "media": round(sum(v) / len(v), 4), "mediana": round(v[len(v) // 2], 4),
            "p95": round(v[min(len(v) - 1, int(0.95 * len(v)))], 4), "max": round(v[-1], 4)}
# tabla de objetivos de corridas anteriores (Station: objetivos_<config>[_<fuente>].json junto a la copia de la config)
for p_obj in [os.path.join(OUT, f) for f in os.listdir(OUT) if f.startswith("objetivos_") and f.endswith(".json")]:
    os.remove(p_obj)

LOG = open(os.path.join(OUT, f"e2e_{a.fuente}.log"), "w", encoding="utf-8")
T0 = time.time()
# Con la salida redirigida a un archivo, Windows usa cp1252: un carácter de un aviso de la estación
# que no exista ahí (p. ej. "≈" en el de actitud no disponible) tumbaba la prueba en print().
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(errors="replace")


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
                        "--puerto", str(a.puerto_web)]
                       + (["--velocidad-mock", str(a.velocidad_mock)] if a.fuente == "mock" else []), cwd=WEB, stdout=srv_log, stderr=subprocess.STDOUT,
                       env=dict(os.environ, QD2_NO_BROWSER="1", PYTHONIOENCODING="utf-8"))
log(f"estación web (pid {srv.pid}) en {BASE}, fuente {a.fuente}, config de prueba {cfg_path}, modo "
    + (f"SOLO EXPLORADOR (dron {SOLO})" if SOLO else "dual") + (f" [{MODO['aviso']}]" if MODO["aviso"] else ""))
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
            "done": None, "cmd_http": [], "fin": None, "version": None, "avisos": set(), "dh_min_aire": None,
            # actitud y(61:63) y geolocalización contra la verdad
            "att": {"n": 0, "ok": 0, "n_mision": 0, "ok_mision": 0, "max_roll_deg": 0.0, "max_pitch_deg": 0.0,
                    "ui_cruda_vista": False, "max_incl_vision_deg": 0.0},
            "obs_err": [], "obs_frames": set(), "vis0": None, "vis_fin": None, "geo": None,
            # modo solo: y(64) visto, modo de la UI y altura máxima del dron no usado
            "y64": set(), "modo_ui": set(), "z_aus_max": 0.0}


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
            c["y64"].add(tel.get("dron_solo"))
            c["modo_ui"].add(((s.get("modo") or {}).get("efectivo"), (s.get("modo") or {}).get("coincide")))
            if SOLO:
                za = tel["drones"][str(3 - SOLO)]["z"]
                if za is not None:
                    c["z_aus_max"] = max(c["z_aus_max"], za)
            # ---- actitud del explorador: y(63) y lo que muestra la UI
            at = c["att"]
            at["n"] += 1
            at["ok"] += bool(tel.get("explorer_att_ok"))
            if st == "MISSION":
                at["n_mision"] += 1
                at["ok_mision"] += bool(tel.get("explorer_att_ok"))
            cruda = (s.get("actitud_explorador") or {}).get("cruda")
            if cruda:
                at["ui_cruda_vista"] = True
                at["max_roll_deg"] = max(at["max_roll_deg"], abs(cruda[0]))
                at["max_pitch_deg"] = max(at["max_pitch_deg"], abs(cruda[1]))
            vis = s.get("vision") or {}
            va = vis.get("actitud") or {}
            cont = {"con_actitud": va.get("frames_con_actitud", 0), "nivelados": va.get("frames_nivelados", 0)}
            c["vis0"] = c["vis0"] or cont
            c["vis_fin"] = dict(cont, descartadas=vis.get("descartadas"), texto=va.get("texto"))
            if va.get("inclinacion_deg") is not None:
                at["max_incl_vision_deg"] = max(at["max_incl_vision_deg"], va["inclinacion_deg"])
            ult = vis.get("ultimo") or {}
            if vis.get("activo") and ult.get("frame_id") and ult["frame_id"] not in c["obs_frames"]:
                c["obs_frames"].add(ult["frame_id"])
                for d in ult.get("dets") or []:     # las que entraron al cluster (no descartadas)
                    if d.get("xy") and not d.get("descartada"):
                        e, _ = err_verdad(d["cls"], *d["xy"])
                        if e is not None:
                            c["obs_err"].append(e)
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
            en_home = all(dd["x"] is not None and dd["home_x"] is not None
                          and math.hypot(dd["x"] - dd["home_x"], dd["y"] - dd["home_y"]) <= TOL_XY
                          for dd in (drones[k] for k in USADOS))
            c["t_en_home"] = (c.get("t_en_home") or now) if (st == "RETURN_HOME" and en_home) else None
            if st == "RETURN_HOME" and not c["land"] and c["t_en_home"] and now - c["t_en_home"] >= 2.0:
                c["cmd_http"].append(("LAND_ALL",) + post("/api/cmd", {"cmd": "LAND_ALL"}))
                c["land"] = True
                log(f"HTTP LAND_ALL -> {c['cmd_http'][-1][1:]}")
            if st == "DONE" and c["done"] is None:
                c["done"] = round(t, 2)
                c["fin"] = {"drones": {d: {k: drones[d][k] for k in ("x", "y", "z", "home_x", "home_y")}
                                       for d in USADOS},
                            "tel": {k: tel[k] for k in ("n_targets_known", "n_targets_visited", "visited_ids",
                                                        "explorer_wp_idx", "explorer_wp_total")},
                            "cmd_resultados": [(r["cmd"], r["estado"]) for r in s["puente"]["cmd_resultados"]],
                            "ws_hz": c["n_ws"] / max(now - c["t_ws0"], 1e-6)}
                log(f"DONE (t_sim {t:.2f}): {c['fin']}")
                c["geo"] = []
                for o in s["objetivos"]:
                    if o.get("manual"):
                        continue
                    e, i = err_verdad(o["cls"], o["x"], o["y"])
                    c["geo"].append({"id": o["id"], "cls": o["cls"], "x": o["x"], "y": o["y"],
                                     "err": None if e is None else round(e, 4), "verdad": i})
                    log(f"geolocalización objetivo #{o['id']} ({o['x']:+.3f}, {o['y']:+.3f}) -> verdad #{i} "
                        f"{VERDAD[i] if i is not None else '—'}: error {e if e is None else round(e, 4)} m")
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
    if SOLO:
        # sin respondedor: nada se visita; y(64) = dron solo en TODA la telemetría; el otro dron nunca despega
        vis = [o["id"] for o in c["obj"].values() if o["estado"] == "visitada"]
        if vis:
            f.append(f"modo solo: objetivos marcados como visitados {vis}")
        if c["y64"] != {SOLO}:
            f.append(f"modo solo: y(64) vistos {sorted(c['y64'], key=str)} (se esperaba solo {SOLO})")
        if c["z_aus_max"] > 0.1:
            f.append(f"modo solo: el dron {3 - SOLO} (no usado) subió a z = {c['z_aus_max']:.2f} m")
        if ("solo_explorador", True) not in c["modo_ui"] or any(m[1] is False for m in c["modo_ui"]):
            f.append(f"modo solo: la UI no mostró el modo coincidente (snapshot.modo: {sorted(c['modo_ui'], key=str)})")
    elif esperada is None:
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
    lejos = [g["id"] for g in c["geo"] or [] if g["err"] is None or g["err"] > RADIO]
    if lejos:
        f.append(f"objetivos a más de {RADIO} m de toda anomalía verdadera de su clase: {lejos}")
    at = c["att"]
    if a.actitud_esperada == "si":
        if not at["n_mision"] or at["ok_mision"] < at["n_mision"]:
            f.append(f"y(63) = 1 solo en {at['ok_mision']} de {at['n_mision']} mensajes en MISSION")
        if not at["ui_cruda_vista"]:
            f.append("la UI nunca mostró la actitud del explorador (actitud_explorador.cruda)")
    elif a.actitud_esperada == "no":
        if at["ok"]:
            f.append(f"y(63) = 1 en {at['ok']} mensajes (se esperaba un modelo sin actitud)")
        if not any("Actitud del explorador no disponible" in m for m in c["avisos"]):
            f.append("no apareció el aviso amarillo de actitud no disponible")
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
        f"dh mín en el aire {c['dh_min_aire']} | DONE {c['done']}"
        + (f" | y(64) {sorted(c['y64'], key=str)} | z máx dron {3 - SOLO} {c['z_aus_max']:.2f} m" if SOLO else ""))
    at = c["att"]
    vf, v0 = c["vis_fin"] or {}, c["vis0"] or {}
    geo_err = [g["err"] for g in c["geo"] or [] if g["err"] is not None]
    actitud = {"usar_actitud": not a.sin_actitud, "y63_mensajes": f"{at['ok']}/{at['n']}",
               "y63_en_mision": f"{at['ok_mision']}/{at['n_mision']}", "ui_cruda_vista": at["ui_cruda_vista"],
               "max_roll_deg": round(at["max_roll_deg"], 2), "max_pitch_deg": round(at["max_pitch_deg"], 2),
               "max_incl_vision_deg": round(at["max_incl_vision_deg"], 2),
               "frames_con_actitud": vf.get("con_actitud", 0) - v0.get("con_actitud", 0),
               "frames_nivelados": vf.get("nivelados", 0) - v0.get("nivelados", 0),
               "descartadas": vf.get("descartadas"), "texto_vision": vf.get("texto")}
    verdad_vista = sorted({g["verdad"] for g in c["geo"] or [] if g["verdad"] is not None})
    geo = {"error_obs_m": stats(c["obs_err"]), "error_objetivos_m": stats(geo_err), "objetivos": c["geo"],
           "verdad_detectada": f"{len(verdad_vista)}/{len(VERDAD)}"}
    log(f"ACTITUD {c['nombre']}: {actitud}")
    log(f"GEOLOCALIZACIÓN {c['nombre']}: verdad detectada {geo['verdad_detectada']} | por observación "
        f"{geo['error_obs_m']} | por objetivo {geo['error_objetivos_m']}")
    for x in fallas:
        log(f"   - {x}")
    c = dict(c, fallas=fallas, fsm={d: sorted(v, key=str) for d, v in c["fsm"].items()}, actitud=actitud,
             geolocalizacion=geo)
    for k in ("obs_err", "obs_frames"):
        c.pop(k)
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
