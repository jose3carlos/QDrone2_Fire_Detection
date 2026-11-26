"""Estación web de 2× QDrone 2 (ARQUITECTURA §2, §3.3, §3.4, §6).

La misión vive en Simulink (o en el simulador MATLAB). Esta estación solo:
puentea (:18700), recibe video (:18800), detecta, geolocaliza, publica objetivos y muestra todo.

Uso (desde la carpeta ``web``):
    python -m app.server                    # fuente según config/estacion.json (web.fuente)
    python -m app.server --fuente mock      # simulación interna, sin MATLAB
    python -m app.server --fuente simulink  # se conecta a tcpip://127.0.0.1:18700
    python -m app.server --fuente simulink --host-puente 192.168.2.11
                                            # modelo de tierra en OTRA PC (laptop -> PC del laboratorio)
    python -m app.server --fuente mock --registro-vuelos si
                                            # registra también los vuelos del mock (web/datos/vuelos)
    python -m app.server --fuente mock --caja-negra si
                                            # graba también la caja negra del mock (*.tel.jsonl.gz)
    python -m app.server --repetir web/datos/vuelos/AAAA-MM-DD/vuelo_HHMMSS_simulink.tel.jsonl.gz --puerto 8081
                                            # repite un vuelo grabado (solo lectura; --velocidad 4 = 4x)
Luego abre http://127.0.0.1:8080
"""
from __future__ import annotations

import argparse
import asyncio
import collections
import contextlib
import copy
import json
import math
import os
import shutil
import threading
import time
from urllib.parse import urlsplit

from fastapi import Body, FastAPI, File, HTTPException, Request, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None
import numpy as np

from . import netinfo
from . import protocol as P
from .bateria import HistorialBateria
from .bridge import BridgeClient, PuenteNoListo
from .caja_negra import EN_VUELO, CajaNegra
from .cameras import CameraHub, FolderCamera, OpenCVCamera, QDF1Client, SyntheticCamera
from .config import (DEFAULT_PATH, PROJECT_DIR, WEB_DIR, Config, actitud_vision, camara_dron, es_loopback,
                     filtros_vision, mision_envolvente, mision_modo, mision_t_perdida_aterrizar,
                     puntos_fuera_limite_consignas, validar_host_puente, validar_ip, vertices_fuera_envolvente)
from .geo import CameraModel, corregir_actitud, tramos_sin_tubo, tubo_cercano, tubos_pista
from .mock_sim import MockServer, contar_waypoints
from .mock_world import anomalias_mock, render_view
from .registro_vuelo import RegistroVuelos
from .repeticion import Grabacion, ServidorRepeticion
from .vision_service import PoseHistory, VisionService, load_detector, pose_para_geo

STATIC = os.path.join(WEB_DIR, "app", "static")
DATA_DIR = os.path.join(WEB_DIR, "datos")
FUENTES = ("mock", "simulink", "sim3d")
COMMANDS_UI = ("START", "PAUSE", "RESUME", "ABORT", "LAND_ALL")
MAX_VERTICES = 16  # = p.max_vertices de simulink_qdrone2/lib/qd2_params.m
# pista.tubos (tubos REALES en el piso; solo estación: filtro de tomas y dibujo, Simulink no lo lee)
MAX_PIEZAS_TUBO = 16
MAX_PUNTOS_PIEZA = 32
TUBO_COORD_MAX = 3.0  # m; un punto más lejos del origen OptiTrack no está en la arena (captura o tecleo erróneo)
SIM_TTL_S = 5.0    # s sin ver un objetivo en y(56:60) (con telemetría fresca) -> deja de mostrarse
CHOQUE_ID_M = 0.5  # m: mismo id en la estación y en Simulink pero tan lejos -> choque de ids
REGISTRO_CLI = {"auto": "auto", "si": True, "no": False}   # --registro-vuelos -> valor de web.registro_vuelos
CAJA_CLI = REGISTRO_CLI                                      # --caja-negra -> valor de web.caja_negra
MSG_REPETICION = ("repetición: la estación está en solo lectura (no manda comandos ni cambia nada); "
                  "para volar, arráncala sin --repetir")
SIN_VIDEO_S = 2.0  # s sin frames de la cámara con el gate de visión activo -> aviso rojo "Sin video del explorador"
DX_ROLES_MIN = 0.05  # m: en dual, |x1 - x2| menor en ASSIGN_ROLES -> explorador ambiguo (el ruido lo puede voltear)


class ModoDistinto(PuenteNoListo):
    """START rechazado: el modo de misión de la config local no coincide con el del modelo de tierra (y(64))."""


class CamaraDistinta(PuenteNoListo):
    """START rechazado: el explorador (y(4) en dual, dron_solo en solo) no es el dron con la cámara (camara.dron)."""


class Station:
    """Une puente, cámara, visión y configuración. Una instancia por proceso."""

    def __init__(self, config_path=DEFAULT_PATH, fuente=None, mock_speed=1.0, host_puente=None, registro_vuelos=None,
                 caja_negra=None, repetir=None, velocidad_repeticion=1.0):
        # --repetir (repeticion.py): la telemetría sale de una caja negra grabada, con la config de ESE vuelo, por el
        # mismo puente TCP; sin cámara, sin registro ni caja negra y en solo lectura (comandos -> 409)
        self.grabacion = None
        self.repeticion = None
        if repetir:
            self.grabacion = Grabacion(repetir)
            config_path = self.grabacion.config_temporal() or config_path
            fuente = self.grabacion.fuente if self.grabacion.fuente in FUENTES else "simulink"
            host_puente, registro_vuelos, caja_negra = None, "no", "no"
            self._velocidad_rep = velocidad_repeticion
        self.cfg = Config(config_path)  # no se escribe nada al arrancar (defaults en memoria)
        self.fuente = fuente or self.cfg.web("fuente")
        if self.fuente not in FUENTES:
            raise ValueError(f"fuente desconocida '{self.fuente}'. Opciones: {FUENTES}")
        # --host-puente (solo CLI, nunca se guarda): el Stream Server :18700 corre en otra PC. Solo con la fuente
        # simulink; sim3d y mock siempre usan loopback (una sesión de prueba no se conecta por error al modelo real).
        self.host_puente_cli = validar_host_puente(host_puente) if host_puente else None
        if self.host_puente_cli and not es_loopback(self.host_puente_cli) and self.fuente != "simulink":
            raise ValueError(f"--host-puente {self.host_puente_cli} solo se permite con --fuente simulink "
                             f"(la fuente '{self.fuente}' siempre usa 127.0.0.1)")
        self.mock_speed = mock_speed
        self.mock = None
        self.bridge = None
        self.poses = PoseHistory()
        self.lock = threading.RLock()
        self.rng = np.random.default_rng(7)
        self.world = []
        self.events = collections.deque(maxlen=60)
        self._last_state = None
        self._last_sim_t = None       # para detectar que Simulink se reinició (t vuelve a 0)
        self._first_tel = True
        self._persisted = False
        self._saved_sig = None
        self.sim_targets = {}         # id -> objetivo que reporta Simulink en y(56:60) (+ "t" de reloj)
        self._last_alerta = 0
        self._n_bloqueados = 0
        self._sesion = None           # n_sesion del puente de la última telemetría
        # MPC: drones que la estación VIO fallar en esta corrida de Simulink (FSM a bordo 7/8, o 9 debiendo
        # volar). La alerta 9 es global; con esto solo sale en rojo el dron que falló (ver protocol.fsm_info).
        self._falla_vista = {1: False, 2: False}
        # alerta 2 (pérdida de rastreo): drones vistos con tracked = 0 mientras dura (el número del dron no viaja
        # en y(1..64); ver protocol.texto_alerta_rastreo). Se olvida cuando la alerta deja de ser 2.
        self._perdida_vista = frozenset()
        self._sin_video_visto = False   # para registrar en eventos cuándo aparece / se va el aviso "Sin video"
        # con el estacion.json real se guarda en web/datos; con otro (pruebas) junto a ese archivo
        same = os.path.abspath(config_path) == os.path.abspath(DEFAULT_PATH)
        self._persist_path = os.path.join(DATA_DIR if same else os.path.dirname(os.path.abspath(config_path)),
                                          "objetivos_estacion.json")
        # Registro de vuelos (registro_vuelo.py): mismo criterio, web/datos/vuelos o junto al estacion.json de prueba.
        # --registro-vuelos (CLI) pisa web.registro_vuelos; pytest nunca escribe en web/datos.
        if registro_vuelos is not None and registro_vuelos not in REGISTRO_CLI:
            raise ValueError(f"--registro-vuelos debe ser uno de {tuple(REGISTRO_CLI)}")
        self._registro_cli = REGISTRO_CLI.get(registro_vuelos)
        carpeta = os.path.join(os.path.dirname(self._persist_path), "vuelos")
        self._registro_bloqueado = bool(os.environ.get("PYTEST_CURRENT_TEST")) and _dentro(carpeta, DATA_DIR)
        # Caja negra (caja_negra.py): misma carpeta y mismo criterio; --caja-negra (CLI) pisa web.caja_negra
        if caja_negra is not None and caja_negra not in CAJA_CLI:
            raise ValueError(f"--caja-negra debe ser uno de {tuple(CAJA_CLI)}")
        self._caja_cli = CAJA_CLI.get(caja_negra)
        try:
            decimacion = min(10, max(1, int(self.cfg.web("caja_negra_decimacion"))))
        except (TypeError, ValueError):
            decimacion = 1
        self.caja = CajaNegra(carpeta, lambda: self.fuente, lambda: self.caja_modo()[0], log_fn=self.log,
                              cabecera_fn=self._cabecera_caja, muestra_fn=self._muestra_caja, base_rel=PROJECT_DIR,
                              decimacion=decimacion)
        # con la caja negra abierta, el registro de observaciones usa su mismo nombre (vuelo_HHMMSS_<fuente>)
        self.registro = RegistroVuelos(carpeta, lambda: self.fuente, lambda: self.registro_modo()[0], log_fn=self.log,
                                       resumen_fn=self._resumen_vuelo, base_rel=PROJECT_DIR,
                                       base_fn=self.caja.base_actual)
        # Batería (bateria.py): SOLO se muestra y se registra; la estación no actúa por ella (el corte es a bordo)
        self.bateria = HistorialBateria()
        # el video se reconecta solo (cameras.CameraHub); cada caída / recuperación va a Eventos con antirrebote
        self.camera = CameraHub(self._camera_key, self._camera_factory, evento_fn=self.log)
        det, name, err = self._make_detector()
        self.vision = VisionService(self.camera, self.poses, self._vision_gate, self._publish_targets,
                                    lambda: self.cfg.data, det, name, err,
                                    synthetic_fn=lambda: self.camera.key == ("sintetica",))
        self.vision.obs_fn = self._obs_vuelo

    # ------------------------------------------------------------------ ciclo de vida
    def start(self):
        self._start_source()
        self.camera.start()
        self.vision.start()
        return self

    def stop(self):
        self.camera.stop()
        self.vision.stop()
        self.registro.detener()
        if self.bridge:
            self.bridge.stop()
        self.caja.detener()             # después del puente: graba hasta el último paquete
        if self.mock:
            self.mock.stop()
        if self.repeticion:
            self.repeticion.stop()
        if self.grabacion is not None and os.path.basename(os.path.dirname(self.cfg.path)).startswith("qd2_repeticion_"):
            shutil.rmtree(os.path.dirname(self.cfg.path), ignore_errors=True)   # config temporal de la repetición

    def _start_source(self):
        cfg = self.cfg.data
        self._last_sim_t, self._first_tel, self._sesion = None, True, None
        with self.lock:
            self.sim_targets = {}
        self._last_alerta = 0
        self._falla_vista = {1: False, 2: False}
        self._perdida_vista = frozenset()
        if self.grabacion is None:
            self._load_targets()
        # web.mock_anomalias o, si no, sobre los tubos reales (pista.tubos) o la ruta (sin pista.tubos)
        self.world = anomalias_mock(cfg)
        if self.grabacion is not None:
            if self.repeticion is None:
                self.repeticion = ServidorRepeticion(self.grabacion, self._velocidad_rep).start()
            host, port = "127.0.0.1", self.repeticion.port
        elif self.fuente == "mock":
            self.mock = MockServer(cfg, "127.0.0.1", 0, speed=self.mock_speed,
                                   cfg_fn=self.cfg.reload_if_changed).start()
            host, port = "127.0.0.1", self.mock.port
        else:
            host, port = self.host_puente(), int(cfg["red"].get("puerto_puente", 18700))
        self.bridge = BridgeClient(host, port, params_fn=self._bridge_params,
                                   cmd_timeout_s=float(self.cfg.web("cmd_timeout_s")))
        crudos = getattr(self.bridge, "raw_listeners", None)
        if self.grabacion is None and crudos is not None:
            crudos.append(self.caja.telemetria)   # cada paquete crudo, antes de procesarlo
        self.bridge.listeners.append(self._on_telemetry)
        self.bridge.on_cmd_result = self._on_cmd_result
        self.bridge.start()
        self.log(f"Fuente '{self.fuente}': puente {self.bridge.uri}")

    def host_puente(self):
        """Host del Stream Server :18700. simulink: --host-puente de la CLI, si no ``web.host_puente`` (defecto
        127.0.0.1). sim3d y mock: SIEMPRE 127.0.0.1."""
        if self.fuente == "simulink":
            return self.host_puente_cli or self.cfg.web("host_puente") or "127.0.0.1"
        return "127.0.0.1"

    def puente_remoto(self):
        """Host remoto del puente (str) si la estación se conecta a un modelo de tierra en otra PC; si no, None."""
        h = getattr(self.bridge, "host", None) if self.bridge is not None and self.fuente == "simulink" else None
        return h if h and not es_loopback(h) else None

    def set_fuente(self, fuente):
        if self.grabacion is not None:
            raise ValueError(MSG_REPETICION)
        if fuente not in FUENTES:
            raise ValueError(f"fuente desconocida '{fuente}'")
        if fuente != "simulink":
            remoto = self.host_puente_cli or self.cfg.web("host_puente")
            if remoto and not es_loopback(remoto):
                self.log(f"Fuente '{fuente}': el puente usa 127.0.0.1 (no el host remoto {remoto})")
        with self.lock:
            if self.bridge:
                self.bridge.stop()
            if self.mock:
                self.mock.stop()
                self.mock = None
            self.registro.cerrar("cambio de fuente")
            self.caja.cerrar("cambio de fuente")
            self.caja.descartar_buffer()      # lo de la fuente anterior no entra al siguiente archivo
            self.bateria.reiniciar()
            self.fuente = fuente
            self.poses.clear()
            self.vision.reset()
            det, name, err = self._make_detector()
            self.vision.detector, self.vision.detector_name, self.vision.detector_error = det, name, err
            self._start_source()
        self.cfg.update({"web": {"fuente": fuente}})

    def log(self, text):
        if self.grabacion is not None:  # repetición: los eventos son los grabados (Grabacion.eventos_en)
            return
        now = time.time()
        self.events.appendleft({"t": time.strftime("%H:%M:%S", time.localtime(now)), "msg": text, "ts": round(now, 3)})
        caja = getattr(self, "caja", None)
        if caja is not None:
            caja.evento(text)

    # ------------------------------------------------------------------ puente
    def _bridge_params(self):
        m = self.cfg.data.get("mision", {})
        return {"z_explorer": m.get("z_explorador", 0.0), "z_responder": m.get("z_respondedor", 0.0),
                "t_mark": m.get("t_mark", 0.0), "d_min_h": m.get("d_min_h", 0.0)}

    # ------------------------------------------------------------------ ids de objetivos
    # Simulink busca los objetivos por id e ignora actualizaciones de un id ya visitado. Por eso
    # los ids NUNCA se reinician mientras Simulink conserve su tabla: la tabla de la estación se
    # guarda en web/datos/objetivos_estacion.json y se recupera al reiniciar la estación.
    def _load_targets(self):
        try:
            with open(self._persist_path, encoding="utf-8") as fh:
                d = json.load(fh)
        except (OSError, ValueError):
            return
        if d.get("fuente") != self.fuente or not d.get("clusters"):
            return
        self.vision.import_state(d)
        self._last_sim_t = d.get("sim_t")
        self._persisted = True
        n = len([c for c in d["clusters"] if c.get("id")])
        self._saved_sig = None
        self.log(f"Recuperada la tabla de objetivos guardada ({n} publicados, siguiente id {d.get('next_id')})")

    def _save_targets(self, targets):
        if self.grabacion is not None:
            return
        sig = tuple((t[0], t[1], round(t[2], 2), round(t[3], 2)) for t in targets)
        if sig == self._saved_sig:
            return
        self._saved_sig = sig
        d = self.vision.export_state()
        d.update({"fuente": self.fuente, "sim_t": self._last_sim_t, "guardado": time.strftime("%Y-%m-%d %H:%M:%S")})
        try:
            os.makedirs(DATA_DIR, exist_ok=True)
            tmp = self._persist_path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(d, fh)
            os.replace(tmp, self._persist_path)
        except OSError:
            pass

    def _sync_ids(self, tel):
        """Detecta reinicios de Simulink y evita que los ids nuevos choquen con su tabla."""
        t = tel.get("t")
        known = int(tel.get("n_targets_known") or 0)
        restarted = False
        if t is not None and self._last_sim_t is not None and t < self._last_sim_t - 1.0:
            restarted = True     # el tiempo de Simulink volvió a empezar: su tabla está vacía
        sesion = getattr(self.bridge, "n_sesion", 0)
        if self._first_tel or sesion != self._sesion:
            # 1.ª telemetría de la estación o de una conexión nueva: el puente todavía no mandó objetivos en
            # esta conexión, así que Simulink sin objetivos y la estación con objetivos publicados significa
            # que Simulink se reinició (o que la tabla guardada es de una corrida anterior). En la 1.ª telemetría
            # de la estación solo cuenta la tabla guardada (un objetivo manual recién puesto no se borra).
            reconexion = not self._first_tel
            if (reconexion or self._persisted) and known == 0 and self.vision.targets()[0]:
                restarted = True
            self._first_tel = False
            self._sesion = sesion
            self._persisted = False
        if restarted:
            self._falla_vista = {1: False, 2: False}
            self._perdida_vista = frozenset()
        if restarted:
            self.registro.cerrar("Simulink se reinició")
        if restarted and (self.vision.targets()[0] or self.vision.targets()[1]):
            self.vision.reset()
            self._saved_sig = None
            self.log("Simulink se reinició (t volvió a empezar): lista de objetivos de la estación reiniciada")
        with self.lock:
            if restarted or known == 0:
                self.sim_targets = {}      # la tabla de Simulink está vacía
            sim_ids = list(self.sim_targets)
        if t is not None:
            self._last_sim_t = t
        floor = max([known, int(tel.get("responder_target_id") or 0)] + list(tel.get("visited_ids") or [])
                    + sim_ids) + 1
        self.vision.ensure_next_id(floor)

    def _sync_sim_targets(self, tel, stamp):
        """y(56:60): un objetivo de la tabla de Simulink por mensaje. Se acumulan por id (1..32) para que la UI
        vea también los que Simulink conoce y esta estación no publicó (p. ej. el simulador 3D con su
        detector interno)."""
        ob = tel.get("objetivo_sim")
        if ob is None or int(tel.get("n_targets_known") or 0) == 0:
            return
        with self.lock:
            nuevo = ob["id"] not in self.sim_targets
            self.sim_targets[ob["id"]] = {**ob, "t": stamp}
        if nuevo:
            propios = self.vision.targets()[0]
            radio = float(self.cfg.get("vision", "radio_cluster_m", default=0.25) or 0.25)
            cerca = [t["id"] for t in propios if math.hypot(t["x"] - ob["x"], t["y"] - ob["y"]) < radio]
            if ob["id"] in {t["id"] for t in propios}:
                pass
            elif cerca:
                self.log(f"Simulink tiene el objetivo {ob['id']} en el mismo lugar que el #{cerca[0]} de esta "
                         "estación: es el mismo objeto (Simulink guardó el id de la estación como alias)")
            else:
                self.log(f"Simulink reporta el objetivo {ob['id']} ({P.CLASSES.get(ob['cls'])}, "
                         f"{ob['x']:+.2f}, {ob['y']:+.2f}) que esta estación no publicó")

    def _registrar_perdida(self, tel):
        """Alerta 2: recuerda qué drones se vieron con tracked = 0 (en solo explorador, solo el que vuela)."""
        if int(tel.get("alerta") or 0) != 2:
            self._perdida_vista = frozenset()
            return
        if tel.get("mission_state") not in P.ESTADOS_ATERRIZAJE_RASTREO + (P.PAUSED,):
            return
        for d in P.drones_sin_rastreo(tel):
            if d not in self._perdida_vista:
                self._perdida_vista = self._perdida_vista | {d}   # copia: _tel_ui la lee desde otro hilo
                que = ("aterrizaje por pérdida de rastreo" if tel.get("mission_state") in P.ESTADOS_ATERRIZAJE_RASTREO
                       else "pérdida de rastreo (PAUSED)")
                self.log(f"Dron {d}: {que} (tracked = 0)")

    def _texto_alerta(self, tel):
        """y(55) con contexto: la alerta 2 dice qué dron aterriza (o si la misión está en PAUSA)."""
        if int(tel.get("alerta") or 0) == 2:
            txt = P.texto_alerta_rastreo(tel.get("mission_state"), self._perdida_vista or P.drones_sin_rastreo(tel))
            if txt:
                return txt
        return tel.get("alerta_texto")

    def _check_alerta(self, tel):
        a = int(tel.get("alerta") or 0)
        if a != self._last_alerta:
            if a:
                self.log(f"ALERTA {a}: {self._texto_alerta(tel)}")
            else:
                self.log(f"Alerta {self._last_alerta} despejada")
            self._last_alerta = a
        if tel.get("ack_operador"):
            self.log("cmd_seq_ack con +0.5: petición del operador (LAND ALL desde los switches) en curso")

    def _on_telemetry(self, tel, stamp):
        self._sync_sim_targets(tel, stamp)
        self._sync_ids(tel)
        self._registrar_perdida(tel)
        self._check_alerta(tel)
        self._registrar_fallas(tel)
        e = tel.get("explorer_id", 0)
        if e in (1, 2):
            d = tel["drones"][e]
            if d["tracked"] and None not in (d["x"], d["y"], d["z"], d["yaw"]):
                # y(61:63): roll/pitch del explorador solo si y(63) = 1 (modelos viejos: None -> nivelado)
                ok = tel.get("explorer_att_ok")
                self.poses.add(stamp, d["x"], d["y"], d["z"], d["yaw"],
                               tel.get("explorer_roll") if ok else None, tel.get("explorer_pitch") if ok else None)
        st = tel.get("mission_state")
        if st != self._last_state:
            self.log(f"Estado de misión: {P.MISSION_STATES_ES.get(st, st)}")
            self._last_state = st
        # caídas del video "en el vuelo" (snapshot.camara.caidas_vuelo): cuentan desde que la misión entra a un
        # estado de vuelo (los mismos de la caja negra) y se conservan hasta el siguiente vuelo
        if (st in EN_VUELO) != self.camera.en_vuelo:
            if st in EN_VUELO:
                self.camera.iniciar_vuelo()
            else:
                self.camera.fin_vuelo()
        self._tick_registro(tel)
        self._registrar_bateria(tel, stamp)

    def _registrar_fallas(self, tel):
        """MPC: recuerda qué dron reportó una falla a bordo (7/8, o 9 debiendo volar) en esta corrida."""
        if tel.get("version") != 2:
            return
        for d in (1, 2):
            dd = tel["drones"][d]
            if not self._falla_vista[d] and P.fsm_info(2, dd["fsm_state"], tel.get("mission_state"), 0)[2]:
                self._falla_vista[d] = True
                self.log(f"Dron {d}: falla a bordo (FSM {dd['fsm_state']}, {dd['fsm_texto']})")

    def _tel_ui(self, tel):
        """Telemetría para la UI: en MPC, con la alerta 9, el 9 del dron que NO falló es su fin de vuelo. Con la
        alerta 2, el texto dice qué dron aterriza por pérdida de rastreo (deducido de tracked = 0)."""
        if tel and int(tel.get("alerta") or 0) == 2:
            txt = self._texto_alerta(tel)
            if txt != tel.get("alerta_texto"):
                tel = dict(tel)
                tel["alerta_texto"] = txt
        if not tel or tel.get("version") != 2 or not any(self._falla_vista.values()):
            return tel
        tel = dict(tel)
        tel["drones"] = {d: dict(dd) for d, dd in tel["drones"].items()}
        for d, dd in tel["drones"].items():
            dd["fsm_tipo"], dd["fsm_texto"], dd["fsm_falla"] = P.fsm_info(
                2, dd["fsm_state"], tel.get("mission_state"), tel.get("alerta"), self._falla_vista.get(d))
        return tel

    def _publish_targets(self, targets):
        if self.grabacion is not None:
            return
        if self.bridge:
            self.bridge.set_targets(targets)
        self._save_targets(targets)

    def _on_cmd_result(self, r):
        txt = {"recibido": "recibido por Simulink", "caducado": "CADUCADO (no se ejecutará)",
               "descartado": "descartado", "reemplazado": "reemplazado"}.get(r["estado"], r["estado"])
        self.log(f"Comando {r['cmd']} (cmd_seq {r['cmd_seq'] or '—'}) {txt}. {r['detalle']}")
        self.caja.ack(r)

    def command(self, name, origen="estación"):
        """Pone el comando en la cola del puente. ``PuenteNoListo`` si no hay telemetría fresca. La caja negra graba
        el pedido y lo que respondió la estación (``origen`` = quién lo pidió, p. ej. "web 127.0.0.1")."""
        try:
            seq = self._command(name)
        except PuenteNoListo as exc:
            self.caja.comando(name, origen, "409", detalle=str(exc))
            raise
        except ValueError as exc:
            self.caja.comando(name, origen, "400", detalle=str(exc))
            raise
        self.caja.comando(name, origen, "enviado" if seq else "en cola", seq)
        return seq

    def _command(self, name):
        if self.grabacion is not None:
            raise PuenteNoListo(MSG_REPETICION)
        if name not in COMMANDS_UI:
            raise ValueError(f"comando desconocido '{name}'")
        if self.bridge is None:
            raise PuenteNoListo("el puente no está iniciado")
        if name == "START" and self.fuente != "mock":
            m = self.modo_info()
            if m["tel"] is not None and m["fresca"] and not m["coincide"]:
                self.log("START rechazado: el modo de la config local no coincide con el del modelo de tierra")
                raise ModoDistinto("START rechazado: el modo de la config local no coincide con el del modelo de "
                                   f"tierra (config: {_modo_txt(m['config'])}; modelo: {_modo_txt(m['tel'])}). "
                                   "Copia estacion.json a la PC y recompila (Build).")
        if name == "START":
            # el video llega SOLO del dron con la RealSense (camara.dron): si el explorador es el otro, no se detecta
            # nada en todo el vuelo. Mismo patrón que el modo: solo START; ABORT, LAND, PAUSE y RESUME siempre pasan
            ci = self.camara_info()
            if ci["aplica"] and not ci["coincide"]:
                self.log("START rechazado: " + ci["msg"])
                raise CamaraDistinta("START rechazado: " + ci["msg"])
        seq = self.bridge.send_command(name)
        self.log(f"Comando {name} " + (f"enviado (cmd_seq {seq})" if seq else "en cola"))
        return seq

    # ------------------------------------------------------------------ cámara y visión
    def _synthetic_allowed(self):
        """La cámara sintética dibuja anomalías INVENTADAS (mock_world) y el detector falso las publica a
        Simulink. Solo se permite con el mock interno, con el simulador MATLAB 3D (sim3d) o si se
        habilita a propósito ``web.permitir_sintetica_en_simulink`` (p. ej. con QD2_Sim_Dual)."""
        return self.fuente in ("mock", "sim3d") or bool(self.cfg.web("permitir_sintetica_en_simulink"))

    def _synthetic(self):
        return self.fuente == "mock" or (self.cfg.get("camara", "fuente") == "sintetica" and self._synthetic_allowed())

    def _make_detector(self):
        if self.grabacion is not None:    # repetición: sin cámara (la visión grabada sale de las líneas "muestra")
            return load_detector("ninguno", PROJECT_DIR, self.cfg.data, False)
        return load_detector(self.cfg.web("detector"), PROJECT_DIR, self.cfg.data, self._synthetic())

    def _camera_key(self):
        if self.grabacion is not None:    # repetición: NUNCA se conecta a la cámara (ni a la red de los drones)
            return ("ninguna",)
        if self.fuente == "mock":
            return ("sintetica",)
        src = str(self.cfg.get("camara", "fuente", default="dron"))
        if src == "sintetica" and not self._synthetic_allowed():
            return ("ninguna",)   # bloqueada: ver avisos()
        if src == "dron":
            # SIEMPRE el dron con la RealSense (camara.dron), no el explorador de turno: antes del START,
            # ASSIGN_ROLES reevalúa los roles (menor x) y el video saltaría entre IPs. Si el explorador no es
            # este dron, START queda bloqueado (camara_info).
            e = camara_dron(self.cfg.data)["dron"]
            ip = self.cfg.get("red", f"ip_dron{e}", default="")
            return ("dron", ip, int(self.cfg.get("red", "puerto_video", default=18800))) if ip else ("ninguna",)
        return (src,)

    def _camera_factory(self, key):
        kind = key[0]
        if kind == "ninguna":
            return None
        if kind == "dron":
            return QDF1Client(key[1], key[2])
        if kind == "sintetica":
            return SyntheticCamera(self.render_synthetic, fps=15.0)
        if kind.startswith("opencv"):
            idx = kind.split(":")[1] if ":" in kind else 0
            return OpenCVCamera(idx)
        if kind == "carpeta":
            return FolderCamera(os.path.join(DATA_DIR, "imagenes_prueba"))
        raise ValueError(f"fuente de cámara desconocida '{kind}'")

    # ------------------------------------------------------------------ modo de misión (dual / solo explorador)
    def modo_info(self):
        """Modo de la config local y del modelo de tierra (y(64)). La verdad del vuelo es la telemetría; la config
        local solo sirve para avisar si no coinciden (el modelo de tierra usa el estacion.json de SU PC al compilar).

        ``efectivo``: el de la telemetría si es fresca, si no el de la config. ``coincide``: telemetría fresca y
        config dan el mismo (modo, dron_solo); sin telemetría fresca no hay con qué comparar (True)."""
        mc = mision_modo(self.cfg.data)
        tel = self.bridge.telemetry if self.bridge else None
        try:
            fresca = bool(tel) and bool(self.bridge.fresh())
        except Exception:
            fresca = False
        mt = None
        if tel and "modo_mision" in tel:
            mt = {"modo": tel["modo_mision"], "dron_solo": int(tel.get("dron_solo") or 0)}
        if fresca and mt is not None:
            efectivo, ds = mt["modo"], mt["dron_solo"]
            coincide = (mt["modo"], mt["dron_solo"]) == (mc["modo"], mc["dron_solo"])
        else:
            efectivo, ds, coincide = mc["modo"], mc["dron_solo"], True
        host = getattr(self.bridge, "host", None) if self.bridge is not None else None
        host = host or self.host_puente()
        return {"config": mc, "tel": mt, "fresca": fresca, "efectivo": efectivo, "dron_solo": ds,
                "coincide": coincide, "host_puente": host, "remoto": self.puente_remoto() is not None}

    def camara_info(self):
        """Dron con la cámara (``camara.dron``) frente al explorador, para el bloqueo de START y el aviso rojo.

        * ``explorador``: en solo explorador, el ``dron_solo`` efectivo (telemetría fresca o, sin ella, la config;
          y(4) = y(64)); en dual, y(4) de la telemetría fresca (en ASSIGN_ROLES Simulink lo reevalúa: el de menor x);
          None si no se sabe (dual sin telemetría fresca o todavía sin roles).
        * ``aplica``: la cámara es la física del dron (``camara.fuente`` = "dron") y la fuente no es el mock (la
          cámara sintética sigue al explorador, sea cual sea).
        * ``coincide``: False si se sabe el explorador y no es ``camara.dron`` o, en dual en ASSIGN_ROLES, si las x
          de los dos drones difieren menos de DX_ROLES_MIN (explorador ambiguo); ``msg`` = el motivo."""
        cd = camara_dron(self.cfg.data)
        m = self.modo_info()
        tel = self.bridge.telemetry if self.bridge else None
        e = None
        if m["efectivo"] == "solo_explorador" and m["dron_solo"] in (1, 2):
            e = m["dron_solo"]
        elif m["fresca"] and tel and tel.get("explorer_id") in (1, 2):
            e = tel["explorer_id"]
        aplica = self.fuente != "mock" and str(self.cfg.get("camara", "fuente", default="dron")) == "dron"
        coincide = e is None or e == cd["dron"]
        msg = None
        if coincide and e is not None and m["efectivo"] == "dual" and tel.get("mission_state") == P.ASSIGN_ROLES:
            # Simulink reevalúa el de menor x en cada tick hasta procesar START: con las x casi iguales el ruido de
            # OptiTrack puede voltear el explorador entre el paquete que revisa la estación y el START
            d1, d2 = (tel.get("drones") or {}).get(1) or {}, (tel.get("drones") or {}).get(2) or {}
            try:
                dx = abs(float(d1["x"]) - float(d2["x"]))
            except (KeyError, TypeError, ValueError):
                dx = None
            if dx is not None and math.isfinite(dx) and dx < DX_ROLES_MIN:
                coincide = False
                msg = (f"Los dos drones tienen casi la misma x (|x1 - x2| = {dx * 100:.1f} cm < "
                       f"{DX_ROLES_MIN * 100:g} cm): el explorador (el de menor x) puede cambiar al dar START; "
                       f"deja el dron {cd['dron']} (cámara) al menos {DX_ROLES_MIN * 100:g} cm más atrás en x")
        if not coincide and msg is None:
            msg = f"El explorador es el dron {e} pero la cámara está en el dron {cd['dron']}: "
            if m["efectivo"] == "solo_explorador":
                msg += (f"vuela con el dron {cd['dron']} (mision.dron_solo; copia estacion.json a la PC y recompila) "
                        "o cambia camara.dron")
            else:
                msg += "reacomoda los drones (el explorador es el de menor x) o cambia camara.dron"
        return {"dron": cd["dron"], "aviso": cd["aviso"], "explorador": e, "aplica": aplica, "coincide": coincide,
                "msg": msg}

    def sin_video(self):
        """True si la visión está explorando (gate activo) y la cámara no entrega frames hace más de SIN_VIDEO_S."""
        if not self._vision_gate():
            return False
        _, _, stamp = self.camera.latest()
        return not stamp or time.time() - stamp > SIN_VIDEO_S

    def _avisos_camara(self):
        out = []
        ci = self.camara_info()
        if ci["aplica"] and not ci["coincide"]:
            out.append({"nivel": "rojo", "msg": ci["msg"] + ". START queda bloqueado hasta que coincidan."})
        if ci["aviso"]:
            out.append({"nivel": "amarillo", "msg": "estacion.json: " + ci["aviso"]})
        sv = self.sin_video()
        if sv:
            out.append({"nivel": "rojo", "msg": "Sin video del explorador: no se detecta nada (sin frames de la cámara "
                        f"del dron {ci['dron']} hace más de {SIN_VIDEO_S:g} s; revisa el streamer y red.ip_dron"
                        f"{ci['dron']})."})
        if sv and not self._sin_video_visto:
            self._sin_video_visto = True
            self.log("Sin video del explorador con la visión activa")
        elif self._sin_video_visto and not sv:
            # "de vuelta" solo si de verdad llegan frames: si el aviso se quitó porque se cerró el gate (fin de la
            # misión, telemetría vieja, bajo z_min_deteccion) el video sigue sin llegar y no se registra nada
            _, _, stamp = self.camera.latest()
            if stamp and time.time() - stamp <= SIN_VIDEO_S:
                self._sin_video_visto = False
                self.log("Video del explorador de vuelta")
        return out

    def _avisos_modo(self, tel):
        out = []
        m = self.modo_info()
        mc, mt = m["config"], m["tel"]
        if tel and tel.get("y64_invalido"):
            raw = (tel.get("reserved") or [None])[0]
            out.append({"nivel": "rojo", "msg": f"y(64) = {raw if raw is not None else 'NaN'} no es 0, 1 ni 2 "
                        "(dron_solo): telemetría corrupta o TELEMETRY PACK mal cableado. Revisa el modelo de tierra."})
        if tel and tel.get("dron_solo") and tel.get("explorer_id") != tel["dron_solo"]:
            out.append({"nivel": "rojo", "msg": f"Telemetría incoherente: modo solo explorador con dron "
                        f"{tel['dron_solo']} (y(64)) pero el explorador es el dron {tel.get('explorer_id')} (y(4))."})
        if m["fresca"] and mt is not None and self.fuente != "mock":
            if mc["modo"] == "solo_explorador" and mt["modo"] == "dual":
                out.append({"nivel": "rojo", "msg": "El modelo de tierra está en DUAL: espera los dos drones rastreados "
                            "y no despegará con uno. Copia estacion.json a la PC y recompila."})
            elif mc["modo"] == "dual" and mt["modo"] == "solo_explorador":
                out.append({"nivel": "amarillo", "msg": f"El modelo de tierra está en SOLO EXPLORADOR (dron "
                            f"{mt['dron_solo']}) pero el estacion.json de esta estación dice dual: START queda "
                            "bloqueado hasta que coincidan."})
            elif mc["modo"] == mt["modo"] == "solo_explorador" and mc["dron_solo"] != mt["dron_solo"]:
                out.append({"nivel": "rojo", "msg": f"Dron solo distinto: el modelo de tierra usa el dron {mt['dron_solo']} "
                            f"y el estacion.json de esta estación el dron {mc['dron_solo']}. Copia estacion.json a la "
                            "PC y recompila."})
        if mc["aviso"]:
            out.append({"nivel": "amarillo", "msg": "estacion.json: " + mc["aviso"]})
        env = mision_envolvente(self.cfg.data)
        if env["aviso"]:
            out.append({"nivel": "amarillo", "msg": "estacion.json: " + env["aviso"]})
        fuera = vertices_fuera_envolvente(self.cfg.data)
        if fuera:
            out.append({"nivel": "amarillo", "msg": "Ruta fuera de la envolvente de consignas (mision.limites_x / "
                        "limites_y): " + ", ".join(f"({x:+.2f}, {y:+.2f})" for x, y in fuera) + ". Simulink recorta "
                        "la consigna del explorador y, si el waypoint queda a más de tol_xy, no lo alcanza y se queda "
                        "en hover ahí. Mueve esos vértices dentro."})
        tp = mision_t_perdida_aterrizar(self.cfg.data)
        if tp["aviso"]:
            out.append({"nivel": "amarillo", "msg": "estacion.json: " + tp["aviso"]})
        elif tp["valor"] > 0.2:
            out.append({"nivel": "amarillo", "msg": f"estacion.json: mision.t_perdida_aterrizar_s = {tp['valor']:g} s: "
                        f"entre 0.2 s y {tp['valor']:g} s sin rastreo la misión pasa a PAUSED; con el binario PID el dron "
                        "ya cae a los 0.2 s (déjalo en 0.2)."})
        if tel and m["efectivo"] == "solo_explorador" and m["dron_solo"] in (1, 2):
            da = 3 - m["dron_solo"]
            dd = (tel.get("drones") or {}).get(da) or {}
            if dd.get("tracked"):
                out.append({"nivel": "amarillo", "msg": f"El dron {da} (no usado) está a la vista de OptiTrack: no "
                            "recibe arm/takeoff, pero sácalo de la arena (afecta el paro automático y el respaldo "
                            "del operador)."})
            if dd.get("comm_ok") == 1:
                out.append({"nivel": "amarillo", "msg": f"El dron {da} está conectado al modelo de tierra; no recibirá "
                            "arm/takeoff."})
        # pista compilada en tierra (y(7)) frente a la de esta estación (misma regla que qd2_track_waypoints)
        if (tel and m["fresca"] and self.fuente != "mock" and (tel.get("mission_state") or 0) >= P.TAKEOFF
                and (tel.get("explorer_wp_total") or 0) > 0):
            n_local = contar_waypoints(self.cfg.get("pista", "vertices", default=[]),
                                       self.cfg.get("mision", "espaciado_waypoints", default=0.25))
            if n_local != tel["explorer_wp_total"]:
                out.append({"nivel": "amarillo", "msg": f"La pista compilada en tierra tiene {tel['explorer_wp_total']} "
                            f"waypoints y la de esta estación {n_local}: ¿copiaste estacion.json a la PC y recompilaste?"})
        return out

    def avisos(self):
        """Avisos para la barra roja/amarilla de la página (en la repetición, los grabados en ese momento)."""
        if self.grabacion is not None:
            return [dict(a) for a in (self._rep_muestra().get("avisos") or []) if isinstance(a, dict)]
        out = []
        synth_cam = self.camera.key == ("sintetica",)
        if synth_cam and self.fuente != "mock":
            out.append({"nivel": "rojo", "msg": f"CÁMARA SINTÉTICA con la fuente '{self.fuente}': las anomalías son "
                        "INVENTADAS (mock_world) y se publican a Simulink. No usar con drones reales."})
        if self.cfg.get("camara", "fuente") == "sintetica" and not self._synthetic_allowed():
            out.append({"nivel": "rojo", "msg": "camara.fuente = 'sintetica' está BLOQUEADA con la fuente 'simulink' "
                        "(mandaría anomalías inventadas a los drones). Pon camara.fuente = 'dron', o si es una "
                        "simulación habilita web.permitir_sintetica_en_simulink en estacion.json."})
        clases = self.vision.detector_classes()
        if not synth_cam and 1 not in clases:
            out.append({"nivel": "rojo", "msg": f"El detector '{self.vision.detector_name}' NO detecta tomas "
                        f"(clase 1){': solo fuego' if 0 in clases else ''}. Revisa vision/detector.py y "
                        "vision/modelos/mejor.onnx (ver web/README.md)."})
        bloq = self.vision.blocked()
        if bloq:
            out.append({"nivel": "rojo", "msg": f"Límite de ids: {len(bloq)} anomalía(s) confirmada(s) NO se publicaron "
                        "a Simulink porque el protocolo solo admite ids 1..32 (visited_mask)."})
            if len(bloq) > self._n_bloqueados:
                self.log(f"Sin ids libres (1..32): {len(bloq)} anomalía(s) confirmada(s) sin publicar")
        self._n_bloqueados = len(bloq)
        rech = (self.bridge.status().get("n_targets_rechazados") if self.bridge else 0) or 0
        if rech:
            out.append({"nivel": "rojo", "msg": f"{rech} objetivo(s) con id fuera de 1..32 no se mandaron a Simulink."})
        choques = self._choques_id()
        if choques:
            out.append({"nivel": "rojo", "msg": "Choque de ids con Simulink: " + ", ".join(f"#{i}" for i in choques)
                        + " son objetos distintos en la estación y en Simulink (¿publican la estación y un detector "
                        "interno a la vez?). Deja que publique solo uno."})
        # La versión real la define el modelo que corre (y(3)); version_control de estacion.json decide qué
        # binario lanzan los comandos quarc_run de la pestaña Conexiones. Si no coinciden, se lanzaría el dron
        # equivocado (tierra MPC con el binario PID -> alerta 10; tierra PID con el dron MPC -> no arma).
        tel = self.bridge.telemetry if self.bridge else None
        v_cfg = str(self.cfg.get("version_control", default="PID")).upper()
        v_tel = (tel or {}).get("version_name")
        if v_tel in ("PID", "MPC") and v_tel != v_cfg and self.fuente != "mock":
            out.append({"nivel": "amarillo", "msg": f"El modelo que corre es {v_tel} pero estacion.json dice "
                        f"version_control = {v_cfg}: los comandos quarc_run de Conexiones lanzarían el binario "
                        f"{v_cfg} en los drones. Cambia la versión en la pestaña Misión antes de lanzar los drones."})
        ci = self.vision.cam_info or {}
        if ci.get("aviso"):
            out.append({"nivel": "amarillo", "msg": "Cámara: " + ci["aviso"]})
        if str(ci.get("origen", "")).startswith("HFOV"):
            out.append({"nivel": "amarillo", "msg": "Geolocalización con HFOV/VFOV (sin intrínsecos fx, fy, cx, cy en "
                        "estacion.json para esta resolución): copia los que imprime el streamer a camara.intrinsecos."})
        # Actitud y(61:63): con el explorador en misión y sin actitud se geolocaliza nivelado (modelo viejo)
        if (tel and self.bridge.fresh() and actitud_vision(self.cfg.data)["usar"] and not tel.get("explorer_att_ok")
                and tel.get("mission_state") in (P.MISSION, P.PAUSED) and self.explorer_pose() is not None):
            out.append({"nivel": "amarillo", "msg": "Actitud del explorador no disponible (y(63) = 0): la "
                        "geolocalización supone el dron nivelado (5° de inclinación ≈ 0.13 m de error a 1.5 m). "
                        "¿Modelo de Simulink sin y(61:63)? Regenera los modelos (construir_modelos)."})
        out.extend(self._avisos_camara())
        out.extend(self._avisos_modo(tel))
        return out

    def envolvente_ui(self):
        """Envolvente de las consignas xy (mision.limites_x / limites_y, = qd2_limites_xy) para el mapa, la vista 3D
        y la pestaña Anomalías: ``None`` sin envolvente (sin las claves); si no ``{"x": [lo, hi], "y": [lo, hi]}`` =
        límite EFECTIVO por eje (el eje sin clave queda en limites_xy -/+ 0.15, como en MATLAB) y ``ejes`` = los
        que vienen de la config. Solo se dibuja e informa: la estación no bloquea nada por ella."""
        env = mision_envolvente(self.cfg.data)
        if not env["definida"]:
            return None
        r = lambda v: round(v, 4)
        return {"x": [r(env["lo"][0]), r(env["hi"][0])], "y": [r(env["lo"][1]), r(env["hi"][1])],
                "ejes": [e for e in ("x", "y") if env[e]]}

    def _cam_model(self):
        """Modelo de la cámara con la resolución configurada (huella en el mapa y cámara sintética)."""
        return CameraModel.from_config(self.cfg.get("camara", default={}), self.cfg.data)

    def explorer_pose(self, tel=None):
        tel = tel if tel is not None else (self.bridge.telemetry if self.bridge else None)
        if not tel or tel.get("explorer_id") not in (1, 2):
            return None
        d = tel["drones"][tel["explorer_id"]]
        if not d["tracked"] or None in (d["x"], d["y"], d["z"], d["yaw"]):
            return None
        return (d["x"], d["y"], d["z"], d["yaw"])

    def explorer_attitude(self):
        """(roll, pitch) del explorador tal como llegan en y(61:62), o None si y(63) = 0 (sin actitud)."""
        tel = self.bridge.telemetry if self.bridge else None
        if not tel or not tel.get("explorer_att_ok"):
            return None
        r, p = tel.get("explorer_roll"), tel.get("explorer_pitch")
        return None if r is None or p is None else (r, p)

    def actitud_ui(self):
        """Actitud del explorador para su tarjeta en la UI (grados), sin depender del video:
        ``cruda`` = y(61:62) tal como llegan (la que se mira en la prueba del libro) y ``usada`` = la que
        entra a la geolocalización, ya con ``vision.actitud_signos`` / ``actitud_offset_deg`` (None con
        ``usar_actitud`` = false o sin y(63)). ``ajustada`` = hay signos u offsets distintos de [1, 1] / [0, 0]."""
        act = actitud_vision(self.cfg.data)
        ajustada = (act["signo_roll"], act["signo_pitch"], act["offset_roll"], act["offset_pitch"]) != (1.0, 1.0, 0.0, 0.0)
        out = {"usar": act["usar"], "ajustada": ajustada, "cruda": None, "usada": None}
        att = self.explorer_attitude()
        if att is None or not all(math.isfinite(a) for a in att):
            return out
        out["cruda"] = [round(math.degrees(a), 2) for a in att]
        if act["usar"]:
            r, p = corregir_actitud(att[0], att[1], act["signo_roll"], act["signo_pitch"],
                                    act["offset_roll"], act["offset_pitch"])
            out["usada"] = [round(math.degrees(r), 2), round(math.degrees(p), 2)]
        return out

    def render_synthetic(self):
        cam = self._cam_model()
        pose = self.explorer_pose()
        if pose is None or pose[2] < 0.15:
            img = np.full((cam.h, cam.w, 3), 25, np.uint8)
            cv2.putText(img, "Explorador en tierra / sin pose", (120, cam.h // 2), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (180, 180, 180), 1, cv2.LINE_AA)
            return img
        # la cámara virtual simula la cámara FÍSICA: siempre con la actitud verdadera (sin importar
        # vision.usar_actitud), para que apagar la corrección muestre el error en lugar de esconderlo
        att = self.explorer_attitude() or (0.0, 0.0)
        # tubería = tubos reales (pista.tubos o la ruta partida en tramos_sin_tubo), el mismo criterio del filtro
        # de tomas; el explorador vuela pista.vertices
        pista = self.cfg.data.get("pista") or {}
        return render_view(cam, tuple(pose) + tuple(att), pista.get("vertices") or [], self.world, rng=self.rng,
                           tubos=tubos_pista(pista))

    def _vision_gate(self, tel=None):
        """La visión geolocaliza y agrupa solo con el explorador en misión (MISSION/PAUSED), rastreado y sobre
        ``web.z_min_deteccion``. ``tel`` = telemetría recién llegada (el registro de vuelo la evalúa en cada una);
        sin ``tel``, la última del puente si es fresca."""
        if tel is None:
            tel = self.bridge.telemetry if self.bridge else None
            if not tel or not self.bridge.fresh():
                return False
        pose = self.explorer_pose(tel)
        zmin = float(self.cfg.web("z_min_deteccion"))
        return tel.get("mission_state") in (P.MISSION, P.PAUSED) and pose is not None and pose[2] >= zmin

    # ------------------------------------------------------------------ registro de vuelos (registro_vuelo.py)
    def registro_modo(self):
        """(habilitado, texto). ``web.registro_vuelos``: "auto" (por defecto: solo la fuente simulink = vuelo real),
        true (siempre) o false (nunca); ``--registro-vuelos auto|si|no`` lo pisa."""
        if self.grabacion is not None:
            return False, "desactivado (repetición)"
        if self._registro_bloqueado:
            return False, "desactivado (pytest: nunca se escribe en web/datos)"
        modo = self._registro_cli if self._registro_cli is not None else self.cfg.web("registro_vuelos")
        if modo is True:
            return True, "siempre"
        if modo is False:
            return False, "desactivado (web.registro_vuelos = false)"
        if self.fuente == "simulink":
            return True, "auto (fuente simulink)"
        return False, f"desactivado con la fuente {self.fuente} (auto; --registro-vuelos si para registrarla)"

    # ------------------------------------------------------------------ caja negra (caja_negra.py) y repetición
    def caja_modo(self):
        """(habilitada, texto). ``web.caja_negra``: "auto" (por defecto: solo la fuente simulink = vuelo real), true
        o false; ``--caja-negra auto|si|no`` lo pisa. Nunca en una repetición ni en pytest dentro de web/datos."""
        if self.grabacion is not None:
            return False, "desactivada (repetición)"
        if self._registro_bloqueado:
            return False, "desactivada (pytest: nunca se escribe en web/datos)"
        modo = self._caja_cli if self._caja_cli is not None else self.cfg.web("caja_negra")
        if modo is True:
            return True, "siempre"
        if modo is False:
            return False, "desactivada (web.caja_negra = false)"
        if self.fuente == "simulink":
            return True, "auto (fuente simulink)"
        return False, f"desactivada con la fuente {self.fuente} (auto; --caja-negra si para grabarla)"

    def _cabecera_caja(self):
        """Contexto del vuelo para la cabecera de la caja negra (todo en memoria: corre en el hilo del puente)."""
        cfg = self.cfg.data
        mi = self.modo_info()
        config = self.cfg.path.replace(os.sep, "/")
        if _dentro(self.cfg.path, PROJECT_DIR):
            config = os.path.relpath(self.cfg.path, PROJECT_DIR).replace(os.sep, "/")
        tel = self.bridge.telemetry if self.bridge else None
        return {"config": config, "config_datos": copy.deepcopy(cfg), "modo_mision": mi["efectivo"],
                "dron_solo": mi["dron_solo"], "modo_config": mi["config"], "explorador": (tel or {}).get("explorer_id"),
                "version_control": cfg.get("version_control"), "version_modelo": (tel or {}).get("version_name"),
                "puente": getattr(self.bridge, "uri", None), "camara_dron": camara_dron(cfg)["dron"],
                "registro_vuelos": self.registro_modo()[1], "eventos_previos": list(self.events)[:15],
                "estacion": {k: self.cfg.web(k) for k in ("z_min_deteccion", "cmd_timeout_s", "camara_latencia_s")}}

    def _muestra_caja(self):
        """Línea "muestra" de la caja negra (~1 Hz, hilo de la caja): cámara, visión, puente, avisos y objetivos."""
        targets, pending = self.vision.targets()
        b = self.bridge.status() if self.bridge else {}
        tel, stamp = self.bridge.latest() if self.bridge else (None, 0.0)
        return {"camara": self._camara_ui(), "vision": self.vision.status(),
                "puente": {k: b.get(k) for k in ("connected", "fresh", "rx_hz", "tx_hz", "rx_age_s", "reconnects",
                                                  "error", "cmd", "cmd_seq", "cmd_seq_ack", "cola")},
                "tel_edad_s": round(time.time() - stamp, 2) if stamp else None,
                "avisos": self.avisos(), "objetivos": targets, "candidatos": pending,
                "objetivos_sin_id": len(self.vision.blocked())}

    def _rep_muestra(self):
        """Repetición: la línea "muestra" grabada en el instante que se está repitiendo ({} si todavía no hay)."""
        if self.grabacion is None or self.repeticion is None:
            return {}
        return self.grabacion.muestra_en(self.repeticion.t) or {}

    def _targets_estacion(self):
        """(objetivos, candidatos) de la estación: los de la visión o, en la repetición, los grabados."""
        if self.grabacion is not None:
            m = self._rep_muestra()
            return ([dict(t) for t in m.get("objetivos") or []], [dict(c) for c in m.get("candidatos") or []])
        return self.vision.targets()

    def repeticion_control(self, accion=None, velocidad=None, t=None):
        """Pausa / seguir / velocidad / saltar en la repetición. Al saltar se olvida lo acumulado (poses, batería,
        objetivos de Simulink) como si Simulink se hubiera reiniciado."""
        if self.repeticion is None:
            raise ValueError("la estación no está en una repetición (--repetir)")
        if accion not in (None, "", "pausa", "seguir", "alternar"):
            raise ValueError("accion debe ser pausa, seguir o alternar")
        if velocidad is not None:
            self.repeticion.set_velocidad(velocidad)
        if t is not None:
            self.repeticion.saltar(float(t))
            self.poses.clear()
            self.bateria.reiniciar()
            self.vision.reset()
            with self.lock:
                self.sim_targets = {}
            self._last_sim_t, self._last_state, self._last_alerta = None, None, 0
            self._falla_vista = {1: False, 2: False}
            self._perdida_vista = frozenset()
        if accion:
            pausado = {"pausa": True, "seguir": False}.get(accion, not self.repeticion.estado()["pausado"])
            self.repeticion.pausa(pausado)
        return self.repeticion.estado()

    def _tick_registro(self, tel):
        try:
            self.registro.tick(self._vision_gate(tel), tel.get("mission_state"), lambda: self._cabecera_vuelo(tel))
        except Exception as exc:          # el registro nunca debe afectar al puente
            self.registro.error = f"{type(exc).__name__}: {exc}"

    def _registrar_bateria(self, tel, stamp):
        """Media de 1 s de battery_V de cada dron: historia de la UI y, con vuelo abierto, líneas ``bateria`` del
        registro (solo de los drones activos: en modo solo, el dron solo de y(64)). Solo visualización y registro."""
        try:
            muestras = self.bateria.agregar(tel, stamp)
            ds = tel.get("dron_solo")
            activos = (ds,) if ds in (1, 2) else (1, 2)
            muestras = [m for m in muestras if m["dron"] in activos]
            if muestras:
                self.registro.bateria(muestras)
        except Exception as exc:          # nunca debe afectar al puente
            self.registro.error = f"{type(exc).__name__}: {exc}"

    def bateria_ui(self, tel):
        """Bloque ``bateria`` del snapshot: los drones activos (modo solo: solo el dron solo)."""
        m = self.modo_info()
        activos = (m["dron_solo"],) if m["efectivo"] == "solo_explorador" and m["dron_solo"] in (1, 2) else (1, 2)
        return self.bateria.estado(tel, activos, time.time())

    def _obs_vuelo(self, stamp, lineas, n_bajo):
        """Observaciones del hilo de visión (gate activo). Si la telemetría que abrió el gate todavía no pasó por
        ``_tick_registro``, el vuelo se abre aquí."""
        if not self.registro.abierto():
            tel = self.bridge.telemetry if self.bridge else None
            if tel:
                self._tick_registro(tel)
        self.registro.observaciones(stamp, lineas, n_bajo)

    def _cabecera_vuelo(self, tel):
        """Contexto del vuelo para la primera línea del registro."""
        cfg = self.cfg.data
        pista = cfg.get("pista") or {}
        vis = cfg.get("vision") or {}
        cam = cfg.get("camara") or {}
        mi = self.modo_info()
        tubos_cfg = pista.get("tubos")
        modelo = dict(self.vision.cam_info or {})
        if not modelo:                    # todavía sin frames: el modelo con la resolución configurada
            try:
                m = self._cam_model()
                modelo = {"origen": m.origen, "fx": round(m.fx, 1), "fy": round(m.fy, 1), "cx": round(m.cx, 1),
                          "cy": round(m.cy, 1), "w": m.w, "h": m.h, "aviso": m.aviso}
            except Exception:
                modelo = {}
        config = self.cfg.path.replace(os.sep, "/")
        if _dentro(self.cfg.path, PROJECT_DIR):
            config = os.path.relpath(self.cfg.path, PROJECT_DIR).replace(os.sep, "/")
        return {"config": config, "modo_mision": mi["efectivo"], "dron_solo": mi["dron_solo"],
                "explorador": tel.get("explorer_id"), "version_control": cfg.get("version_control"),
                "version_modelo": tel.get("version_name"), "detector": self.vision.detector_name,
                "pista": {"vertices": pista.get("vertices") or [], "tubos": tubos_pista(pista),
                          "tubos_de": "pista.tubos" if isinstance(tubos_cfg, list) and tubos_cfg else "pista.vertices",
                          "tramos_sin_tubo": sorted(tramos_sin_tubo(pista))},
                "limites_xy": (cfg.get("mision") or {}).get("limites_xy"),
                "envolvente": self.envolvente_ui(),
                "filtros": filtros_vision(cfg), "confianza_min": vis.get("confianza_min"),
                "radio_cluster_m": vis.get("radio_cluster_m"), "min_observaciones": vis.get("min_observaciones"),
                "actitud": {k: vis.get(k) for k in ("usar_actitud", "actitud_signos", "actitud_offset_deg")},
                "camara": {"offset_camara_m": cam.get("offset_camara_m"), "rotacion_montaje": cam.get("rotacion_montaje"),
                           "giro_en_dron": cam.get("giro_en_dron", 0), "resolucion": cam.get("resolucion"),
                           "modelo": modelo},
                "z_min_deteccion": self.cfg.web("z_min_deteccion"), "camara_latencia_s": self.cfg.web("camara_latencia_s"),
                "caja_negra": self.caja.archivo_actual()}

    def _resumen_vuelo(self):
        """Objetivos y candidatos de la estación al cerrar el vuelo (con su distancia al tubo actual)."""
        targets, pending = self.vision.targets()
        tubos = tubos_pista(self.cfg.data.get("pista"))

        def fila(t, *claves):
            tc = tubo_cercano(t["x"], t["y"], tubos)
            out = {k: (round(t[k], 3) if isinstance(t[k], float) else t[k]) for k in claves if k in t}
            out["dist_tubo_m"] = round(tc["dist"], 3) if tc else None
            return out
        return {"objetivos": [fila(t, "id", "cls", "x", "y", "conf", "n", "manual") for t in targets],
                "candidatos": [fila(c, "cls", "x", "y", "n") for c in pending],
                "objetivos_sin_id": len(self.vision.blocked())}

    # ------------------------------------------------------------------ objetivos (estación + Simulink)
    def _sim_vigentes(self):
        """Objetivos de y(56:60) vistos hace poco (con telemetría fresca se olvidan tras SIM_TTL_S)."""
        fresh = bool(self.bridge and self.bridge.fresh())
        now = time.time()
        with self.lock:
            if fresh:
                for k in [k for k, o in self.sim_targets.items() if now - o["t"] > SIM_TTL_S]:
                    del self.sim_targets[k]
            return {k: dict(o) for k, o in self.sim_targets.items()}

    def _choques_id(self):
        sim = self._sim_vigentes()
        out = []
        for t in self.vision.targets()[0]:
            o = sim.get(t["id"])
            if o and (o["cls"] != t["cls"] or math.hypot(o["x"] - t["x"], o["y"] - t["y"]) > CHOQUE_ID_M):
                out.append(t["id"])
        return out

    def merged_targets(self, tel):
        """Tabla de la UI: los objetivos que publicó la estación más los que Simulink reporta en y(56:60),
        sin duplicar: mismo id = mismo objetivo, y un objetivo de Simulink a menos de radio_cluster de uno de la
        estación con otro id también (Simulink guardó el id de la estación como alias; ``alias_de``).
        ``origen``: estacion | estacion+simulink | simulink."""
        targets, _ = self._targets_estacion()
        sim = self._sim_vigentes()
        visited = set(tel["visited_ids"]) if tel else set()
        cur = tel["responder_target_id"] if tel else 0
        propios = {t["id"] for t in targets}
        radio = float(self.cfg.get("vision", "radio_cluster_m", default=0.25) or 0.25)
        out, alias_de_sim = [], set()
        for t in targets:
            o = sim.get(t["id"])
            alias = None
            if o is None:
                # Simulink registra como ALIAS (sin agregarlo a su tabla) un id nuevo a menos de radio_cluster
                # de un objetivo que ya tiene (qd2_mission_step.actualizar_objetivos, sin mirar la clase).
                # Ese objetivo de Simulink es el mismo objeto: no se muestra dos veces.
                cerca = [(math.hypot(s["x"] - t["x"], s["y"] - t["y"]), k) for k, s in sim.items()
                         if k not in propios]
                d, k = min(cerca, default=(None, None))
                if d is not None and d < radio:
                    o, alias = sim[k], k
                    alias_de_sim.add(k)
            t["origen"] = "estacion+simulink" if o else "estacion"
            if o:
                t["x_sim"], t["y_sim"], t["estado_sim"] = round(o["x"], 3), round(o["y"], 3), o["estado"]
            if alias is not None:
                t["alias_de"] = alias      # en Simulink es el objetivo #alias (este id quedó como alias suyo)
            t["x"], t["y"], t["conf"] = round(t["x"], 3), round(t["y"], 3), round(t["conf"], 3)
            out.append(t)
        for k in propios | alias_de_sim:
            sim.pop(k, None)
        for tid, o in sim.items():
            if not (1 <= tid <= P.MAX_TARGETS):
                continue
            out.append({"id": tid, "cls": o["cls"], "x": round(o["x"], 3), "y": round(o["y"], 3), "conf": None,
                        "n": None, "manual": False, "origen": "simulink", "estado_sim": o["estado"]})
        for t in out:
            es = t.get("estado_sim")
            # visited_mask y responder_target_id llegan en cada mensaje; el estado de y(56:60) puede tener
            # hasta 32 mensajes de retraso, así que solo se usa para "visitado" (es permanente)
            t["estado"] = ("visitada" if t["id"] in visited or es == 3
                           else "asignada" if cur and cur in (t["id"], t.get("alias_de")) else "detectada")
            t["clase"] = P.CLASSES.get(t["cls"], str(t["cls"]))
        return sorted(out, key=lambda t: t["id"])

    # ------------------------------------------------------------------ estado para la UI
    def huella(self):
        """Polígono que ve la cámara del explorador sobre el piso, con la misma actitud (corregida con
        vision.actitud_signos / actitud_offset_deg, o nivelada sin y(63) o con usar_actitud = false) con
        la que se geolocaliza."""
        pose = self.explorer_pose()
        if not pose or pose[2] <= 0.1:
            return []
        pose_geo, _ = pose_para_geo(tuple(pose) + tuple(self.explorer_attitude() or (None, None)),
                                    actitud_vision(self.cfg.data))
        return self._cam_model().footprint(*pose_geo, z_plane=0.02)

    def _camara_ui(self):
        """Estado del video para la UI más ``dron`` (camara.dron), ``explorador`` y ``coincide`` (camara_info)."""
        ci = self.camara_info()
        return dict(self.camera.status(), dron=ci["dron"], explorador=ci["explorador"],
                    coincide=ci["coincide"] or not ci["aplica"], msg=ci["msg"] if ci["aplica"] else None)

    def snapshot(self):
        tel, stamp = self.bridge.latest() if self.bridge else (None, 0.0)
        tel = self._tel_ui(tel)
        _, pending = self._targets_estacion()
        targets = self.merged_targets(tel)
        foot = self.huella()
        cfg = self.cfg.data
        snap = {
            "t_servidor": time.time(), "fuente": self.fuente,
            "puente": self.bridge.status() if self.bridge else None,
            "tel": tel, "tel_edad_s": round(time.time() - stamp, 2) if stamp else None,
            "objetivos": targets, "candidatos": [{**c, "x": round(c["x"], 3), "y": round(c["y"], 3)} for c in pending],
            "objetivos_sin_id": len(self.vision.blocked()),
            "huella": foot, "camara": self._camara_ui(), "vision": self.vision.status(),
            "actitud_explorador": self.actitud_ui(),
            "pista": cfg.get("pista", {}).get("vertices", []),
            "pista_sin_tubo": sorted(tramos_sin_tubo(cfg.get("pista"))),
            "tubos": tubos_pista(cfg.get("pista")),   # tubos reales (pista.tubos o la ruta sin los huecos)
            "mision": cfg.get("mision", {}), "version_config": cfg.get("version_control", "PID"),
            "envolvente": self.envolvente_ui(),
            "verdad_mock": self.world if self.fuente == "mock" else [],
            "mock": self.mock.estado_pruebas() if self.mock is not None else None,
            "eventos": list(self.events)[:15], "avisos": self.avisos(), "modo": self.modo_info(),
            "registro_vuelo": dict(self.registro.estado(), modo=self.registro_modo()[1]),
            "caja_negra": dict(self.caja.estado(), modo=self.caja_modo()[1]),
            "bateria": self.bateria_ui(tel),
            "repeticion": self.repeticion.estado() if self.repeticion is not None else None,
        }
        if self.grabacion is not None:
            # lo que no viaja en la telemetría sale de la grabación (línea "muestra" de ese instante y eventos)
            m = self._rep_muestra()
            snap["camara"] = dict(m.get("camara") or {"fuente": ["repeticion"], "conectada": False, "fps": 0})
            if m.get("vision"):
                snap["vision"] = m["vision"]
            snap["objetivos_sin_id"] = m.get("objetivos_sin_id") or 0
            snap["eventos"] = self.grabacion.eventos_en(self.repeticion.t if self.repeticion else 0.0)
        return snap


# ---------------------------------------------------------------------------- FastAPI
def create_app(station: Station, start=True):
    @contextlib.asynccontextmanager
    async def lifespan(_app):
        if start:
            station.start()
        try:
            yield
        finally:
            station.stop()

    app = FastAPI(title="Estación QDrone 2 — tomas clandestinas", docs_url="/api/docs", redoc_url=None,
                  lifespan=lifespan)
    app.state.station = station
    os.makedirs(DATA_DIR, exist_ok=True)

    app.mount("/static", StaticFiles(directory=STATIC), name="static")

    # Otra página abierta en el mismo navegador no puede mandar comandos, subir archivos ni leer el
    # WebSocket: si el navegador manda Origin, debe ser la propia estación.
    @app.middleware("http")
    async def check_origin(request: Request, call_next):
        if request.method not in ("GET", "HEAD", "OPTIONS") and not origin_ok(request.headers):
            return JSONResponse({"detail": "origen no permitido"}, status_code=403)
        # repetición: solo lectura (comandos, fuente, config, objetivos, mock...); solo se controla la repetición
        if (station.grabacion is not None and request.method not in ("GET", "HEAD", "OPTIONS")
                and request.url.path != "/api/repeticion"):
            return JSONResponse({"detail": MSG_REPETICION}, status_code=409)
        return await call_next(request)

    @app.get("/", response_class=HTMLResponse)
    def index():
        return FileResponse(os.path.join(STATIC, "index.html"))

    # -------------------------------------------------------------- estado y comandos
    @app.get("/api/estado")
    def estado():
        return JSONResponse(station.snapshot())

    @app.post("/api/cmd")
    def cmd(request: Request, body: dict = Body(...)):
        origen = f"web {request.client.host if request.client else '?'}"
        try:
            seq = station.command(str(body.get("cmd", "")).upper(), origen=origen)
        except ValueError as exc:
            raise HTTPException(400, str(exc))
        except PuenteNoListo as exc:
            raise HTTPException(409, str(exc))
        return {"ok": True, "cmd_seq": seq, "estado": "enviado" if seq else "en cola"}

    @app.get("/api/repeticion")
    def repeticion_estado():
        if station.repeticion is None:
            raise HTTPException(404, "la estación no está en una repetición (--repetir)")
        return station.repeticion.estado()

    @app.post("/api/repeticion")
    def repeticion(body: dict = Body(...)):
        """``{"accion": "pausa" | "seguir" | "alternar", "velocidad": N, "t": s}`` (cualquiera de las tres)."""
        try:
            return station.repeticion_control(body.get("accion"), body.get("velocidad"), body.get("t"))
        except (TypeError, ValueError) as exc:
            raise HTTPException(400, str(exc))

    @app.post("/api/fuente")
    def fuente(body: dict = Body(...)):
        try:
            station.set_fuente(body.get("fuente"))
        except ValueError as exc:
            raise HTTPException(400, str(exc))
        return {"ok": True, "fuente": station.fuente}

    @app.post("/api/mock/reiniciar")
    def mock_reset():
        if station.mock is None:
            raise HTTPException(400, "solo disponible con la fuente mock")
        station.mock.reset()
        station.registro.cerrar("mock reiniciado")
        station.bateria.reiniciar()
        station.vision.reset()
        station.poses.clear()
        with station.lock:
            station.sim_targets = {}
        station.log("Mock reiniciado")
        return {"ok": True}

    # Solo para probar la UI con la fuente mock: forzar y(55) y agregar objetivos "de Simulink" (y(56:60))
    @app.post("/api/mock/alerta")
    def mock_alerta(body: dict = Body(...)):
        if station.mock is None:
            raise HTTPException(400, "solo disponible con la fuente mock")
        code = body.get("codigo")
        try:
            station.mock.forzar_alerta(None if code is None else int(code))
        except (TypeError, ValueError) as exc:
            raise HTTPException(400, str(exc))
        station.log("Mock: " + ("alerta real (sin forzar)" if code is None else f"y(55) forzada a {int(code)}"))
        return {"ok": True, "codigo": code}

    @app.post("/api/mock/objetivo_interno")
    def mock_objetivo(body: dict = Body(...)):
        if station.mock is None:
            raise HTTPException(400, "solo disponible con la fuente mock")
        try:
            cls = body.get("cls", 0)
            cls = P.CLASS_IDS[cls] if isinstance(cls, str) else int(cls)
            x, y = float(body["x"]), float(body["y"])
        except (KeyError, ValueError, TypeError) as exc:
            raise HTTPException(400, f"datos inválidos: {exc}")
        if cls not in P.CLASSES:
            raise HTTPException(400, "cls debe ser 0 (fuego) o 1 (toma)")
        tid = station.mock.objetivo_interno(cls, x, y)
        if tid is None:
            raise HTTPException(409, "la tabla del mock ya tiene 32 objetivos (ids 1..32)")
        return {"ok": True, "id": tid}

    @app.post("/api/objetivos/manual")
    def manual(body: dict = Body(...)):
        try:
            cls = body.get("cls", 0)
            cls = P.CLASS_IDS[cls] if isinstance(cls, str) else int(cls)
            x, y = float(body["x"]), float(body["y"])
        except (KeyError, ValueError, TypeError) as exc:
            raise HTTPException(400, f"datos inválidos: {exc}")
        if cls not in P.CLASSES:
            raise HTTPException(400, "cls debe ser 0 (fuego) o 1 (toma)")
        lim = station.cfg.get("mision", "limites_xy", default=[-1.5, 1.5])
        if not (lim[0] <= x <= lim[1] and lim[0] <= y <= lim[1]):
            raise HTTPException(400, f"la posición debe estar dentro de la arena {lim}")
        try:
            tid = station.vision.add_manual(cls, x, y)
        except ValueError as exc:
            raise HTTPException(400, str(exc))
        station.log(f"Objetivo manual {tid}: {P.CLASSES.get(cls)} ({x:+.2f}, {y:+.2f})")
        return {"ok": True, "id": tid}

    @app.post("/api/objetivos/reiniciar")
    def objetivos_reset():
        # Solo si Simulink no tiene objetivos: si no, los ids volverían a empezar en 1 y chocarían
        # con su tabla (Simulink ignora un id ya visitado). Reiniciar Simulink vacía su tabla y la
        # estación lo detecta sola.
        tel = station.bridge.telemetry if station.bridge else None
        if not tel or not station.bridge.fresh():
            raise HTTPException(409, "sin telemetría del puente: no se sabe si Simulink ya tiene objetivos")
        if int(tel.get("n_targets_known") or 0) > 0:
            raise HTTPException(409, f"Simulink ya tiene {tel['n_targets_known']} objetivos: no se puede reiniciar "
                                     "la lista (reinicia el modelo de Simulink / el mock para empezar de cero)")
        station.registro.cerrar("objetivos reiniciados")
        station.vision.reset()
        station.log("Lista de objetivos de la estación reiniciada")
        return {"ok": True}

    # -------------------------------------------------------------- configuración
    @app.get("/api/config")
    def get_config():
        return station.cfg.reload_if_changed()

    @app.post("/api/config/red")
    def set_red(body: dict = Body(...)):
        patch = {}
        try:
            for k in ("ip_pc", "ip_dron1", "ip_dron2"):
                if k in body:
                    patch[k] = validar_ip(body[k])
        except ValueError as exc:
            raise HTTPException(400, str(exc))
        if "interfaz_preferida" in body:
            patch["interfaz_preferida"] = str(body["interfaz_preferida"])
        ip1 = patch.get("ip_dron1", station.cfg.get("red", "ip_dron1", default=""))
        ip2 = patch.get("ip_dron2", station.cfg.get("red", "ip_dron2", default=""))
        if ip1 and ip1 == ip2:
            raise HTTPException(400, "los dos drones no pueden tener la misma IP")
        station.cfg.update({"red": patch})
        return {"ok": True, "red": station.cfg.data["red"]}

    @app.post("/api/config/version")
    def set_version(body: dict = Body(...)):
        v = str(body.get("version_control", "")).upper()
        if v not in ("PID", "MPC"):
            raise HTTPException(400, "version_control debe ser PID o MPC")
        station.cfg.update({"version_control": v})
        return {"ok": True}

    @app.post("/api/config/optitrack")
    def set_optitrack(body: dict = Body(...)):
        patch = {}
        for k in ("carpeta", "archivo_cal", "archivo_motive"):
            if k in body:
                patch[k] = str(body[k])
        for k in ("rigid_body_dron1", "rigid_body_dron2"):
            if k in body:
                try:
                    v = int(body[k])
                except (TypeError, ValueError):
                    raise HTTPException(400, f"{k} debe ser un entero")
                if v < 1:
                    raise HTTPException(400, f"{k} debe ser ≥ 1")
                patch[k] = v
        rb1 = patch.get("rigid_body_dron1", station.cfg.get("optitrack", "rigid_body_dron1"))
        rb2 = patch.get("rigid_body_dron2", station.cfg.get("optitrack", "rigid_body_dron2"))
        if rb1 == rb2:
            raise HTTPException(400, "los dos drones no pueden usar el mismo rigid body")
        for k in ("archivo_cal", "archivo_motive"):
            ext = ".cal" if k == "archivo_cal" else ".motive"
            if patch.get(k) and not patch[k].lower().endswith(ext):
                raise HTTPException(400, f"{k} debe terminar en {ext}")
        station.cfg.update({"optitrack": patch})
        return {"ok": True, "optitrack": station.cfg.data["optitrack"]}

    @app.get("/api/optitrack/archivos")
    def list_optitrack(carpeta: str = ""):
        folder = carpeta or station.cfg.get("optitrack", "carpeta", default="")
        if not folder or not os.path.isdir(folder):
            return {"ok": False, "carpeta": folder, "error": "la carpeta no existe en esta PC", "cal": [], "motive": []}
        out = {"cal": [], "motive": []}
        base_depth = folder.rstrip("\\/").count(os.sep)
        for root, dirs, files in os.walk(folder):
            if root.count(os.sep) - base_depth >= 3:
                dirs[:] = []
            for f in files:
                ext = os.path.splitext(f)[1].lower()
                if ext in (".cal", ".motive"):
                    p = os.path.join(root, f)
                    st = os.stat(p)
                    out[ext[1:]].append({"ruta": p, "nombre": os.path.relpath(p, folder), "kb": round(st.st_size / 1024, 1),
                                         "modificado": time.strftime("%Y-%m-%d %H:%M", time.localtime(st.st_mtime)),
                                         "_t": st.st_mtime})
            if len(out["cal"]) + len(out["motive"]) > 500:
                break
        for k in out:
            out[k].sort(key=lambda r: -r["_t"])
            for r in out[k]:
                r.pop("_t")
        return {"ok": True, "carpeta": folder, **out}

    @app.post("/api/optitrack/subir")
    async def upload_optitrack(archivo: UploadFile = File(...)):
        name = os.path.basename(archivo.filename or "")
        ext = os.path.splitext(name)[1].lower()
        if ext not in (".cal", ".motive"):
            raise HTTPException(400, "solo se aceptan archivos .cal o .motive")
        dest_dir = os.path.join(DATA_DIR, "optitrack")
        os.makedirs(dest_dir, exist_ok=True)
        dest = os.path.join(dest_dir, name)
        data = await archivo.read()
        if len(data) > 200 * 1024 * 1024:
            raise HTTPException(400, "archivo demasiado grande")
        with open(dest, "wb") as fh:
            fh.write(data)
        return {"ok": True, "ruta": dest, "tipo": ext[1:]}

    def _pares(lista, nombre):
        try:
            return [[round(float(a), 4), round(float(b), 4)] for a, b in lista]
        except (TypeError, ValueError):
            raise HTTPException(400, f"{nombre} debe ser una lista de pares [x, y]")

    @app.post("/api/config/pista")
    def set_pista(body: dict = Body(...)):
        """Guarda UN arreglo de la pista según ``destino`` (el resto de estacion.json queda byte a byte):

        * ``"vertices"`` (por defecto): ``pista.vertices`` = ruta de VUELO que compila Simulink (requiere recompilar);
          2-16 vértices dentro de ``mision.limites_xy``. Responde ``fuera_envolvente`` = vértices fuera del límite
          efectivo de las consignas (envolvente o limites_xy -/+ 0.15): solo aviso, no bloquea;
        * ``"tubos"``: ``pista.tubos`` = tubos REALES en el piso (filtro de tomas y dibujo; Simulink no lo lee), lista de
          piezas, cada una con 2 o más puntos [x, y]. Una lista VACÍA quita los tubos reales (``"tubos": []``): el
          filtro y el dibujo vuelven a usar la ruta partida en ``tramos_sin_tubo`` (como sin la clave)."""
        destino = body.get("destino", "vertices")
        if destino == "tubos":
            tubos = body.get("tubos")
            if not isinstance(tubos, list):
                raise HTTPException(400, "tubos debe ser una lista de piezas (cada una, lista de puntos [x, y]); "
                                         "una lista vacía quita los tubos reales")
            if len(tubos) > MAX_PIEZAS_TUBO:
                raise HTTPException(400, f"como máximo {MAX_PIEZAS_TUBO} piezas de tubo")
            piezas = []
            for k, pieza in enumerate(tubos, 1):
                P_ = _pares(pieza if isinstance(pieza, list) else None, f"la pieza {k} de tubos")
                if len(P_) < 2:
                    raise HTTPException(400, f"la pieza {k} de tubos tiene {len(P_)} punto(s): cada pieza necesita "
                                             "al menos 2 (sus extremos)")
                if len(P_) > MAX_PUNTOS_PIEZA:
                    raise HTTPException(400, f"la pieza {k} de tubos tiene más de {MAX_PUNTOS_PIEZA} puntos")
                if any(not (math.isfinite(v) and abs(v) <= TUBO_COORD_MAX) for p in P_ for v in p):
                    raise HTTPException(400, f"la pieza {k} de tubos tiene puntos fuera de ±{TUBO_COORD_MAX:g} m")
                piezas.append(P_)
            station.cfg.update({"pista": {"tubos": piezas}})
            station.log(f"Tubos reales guardados (pista.tubos: {len(piezas)} pieza(s), "
                        f"{sum(len(p) for p in piezas)} puntos)" if piezas else
                        "Tubos reales quitados (pista.tubos = []): el filtro de tomas vuelve a usar la ruta")
            return {"ok": True, "destino": "tubos", "tubos": piezas}
        if destino != "vertices":
            raise HTTPException(400, "destino debe ser 'vertices' (ruta de vuelo) o 'tubos' (tubos reales)")
        if not isinstance(body.get("vertices"), list):
            raise HTTPException(400, "vertices debe ser una lista de pares [x, y]")
        verts = _pares(body["vertices"], "vertices")
        lim = station.cfg.get("mision", "limites_xy", default=[-1.5, 1.5])
        # qd2_params_from_config de Simulink usa como máximo p.max_vertices = 16
        if len(verts) < 2 or len(verts) > MAX_VERTICES:
            raise HTTPException(400, f"se necesitan entre 2 y {MAX_VERTICES} vértices (límite de Simulink)")
        if any(not (lim[0] <= v <= lim[1]) for p in verts for v in p):
            raise HTTPException(400, f"todos los vértices deben estar dentro de {lim}")
        station.cfg.update({"pista": {"vertices": verts}})
        fuera = puntos_fuera_limite_consignas(station.cfg.data, verts)
        station.log(f"Pista guardada ({len(verts)} vértices)"
                    + (f"; {len(fuera)} fuera de la envolvente de consignas" if fuera else ""))
        return {"ok": True, "destino": "vertices", "vertices": verts, "fuera_envolvente": fuera}

    @app.post("/api/pista/capturar")
    def capturar(body: dict = Body(...)):
        dron = int(body.get("dron", 1))
        tel = station.bridge.telemetry if station.bridge else None
        if dron not in (1, 2) or not tel or not station.bridge.fresh():
            raise HTTPException(409, "no hay telemetría reciente del puente")
        d = tel["drones"][dron]
        if not d["tracked"] or d["x"] is None:
            raise HTTPException(409, f"el dron {dron} no está rastreado por OptiTrack")
        rb = station.cfg.get("optitrack", f"rigid_body_dron{dron}")
        return {"ok": True, "dron": dron, "rigid_body": rb, "x": round(d["x"], 4), "y": round(d["y"], 4),
                "z": round(d["z"], 4)}

    # -------------------------------------------------------------- red
    @app.get("/api/red")
    def red():
        cfg = station.cfg.data
        return {"interfaces": netinfo.interfaces(cfg["red"].get("interfaz_preferida", "Wi-Fi 2")),
                "ip_recomendada": netinfo.ip_recomendada(cfg, station.puente_remoto()), "red": cfg["red"]}

    @app.get("/api/comandos")
    def comandos(ip_pc: str = ""):
        cfg = station.cfg.data
        return netinfo.comandos(cfg, ip_pc or None, mision_modo(cfg), station.puente_remoto())

    # -------------------------------------------------------------- video
    def _no_signal(text):
        img = np.full((360, 480, 3), 30, np.uint8)
        cv2.putText(img, text, (20, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (190, 190, 190), 1, cv2.LINE_AA)
        return img

    def _jpeg(img, q=75):
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), q])
        return buf.tobytes()

    @app.get("/api/camara.jpg")
    def snapshot_jpg():
        frame, _, _ = station.camera.latest()
        img = station.vision.annotate(frame) if frame is not None else _no_signal("Sin imagen de cámara")
        return Response(_jpeg(img, 85), media_type="image/jpeg")

    @app.get("/video.mjpg")
    def video():
        def gen():
            last = 0
            while True:
                got = station.camera.wait_newer(last, 2.0)
                if got is None:
                    img = _no_signal("Sin video: " + (station.camera.error or "esperando"))[:, :, :]
                else:
                    frame, last, _ = got
                    img = station.vision.annotate(frame)
                jpg = _jpeg(img)
                yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpg)).encode()
                       + b"\r\n\r\n" + jpg + b"\r\n")
                time.sleep(1 / 20)
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    # -------------------------------------------------------------- WebSocket
    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        if not origin_ok(sock.headers):
            await sock.close(code=1008)
            return
        await sock.accept()
        try:
            while True:
                snap = await asyncio.get_running_loop().run_in_executor(None, station.snapshot)
                await sock.send_json(_clean(snap))
                await asyncio.sleep(1 / 15)
        except (WebSocketDisconnect, RuntimeError):
            pass

    return app


def origin_ok(headers):
    """True si no hay Origin (curl, Python) o si coincide con el Host de la propia estación."""
    origin = headers.get("origin")
    if not origin:
        return True
    try:
        return urlsplit(origin).netloc.lower() == (headers.get("host") or "").lower()
    except ValueError:
        return False


def _clean(o):
    """JSON estricto: NaN/inf -> None."""
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, np.generic):
        return _clean(o.item())
    return o


def _dentro(ruta, carpeta):
    """True si ``ruta`` está dentro de ``carpeta`` (o es ella)."""
    try:
        return os.path.commonpath([os.path.abspath(ruta), os.path.abspath(carpeta)]) == os.path.abspath(carpeta)
    except ValueError:            # otra unidad en Windows
        return False


def _modo_txt(m):
    if not m:
        return "?"
    return f"solo explorador (dron {m.get('dron_solo')})" if m.get("modo") == "solo_explorador" else "dual"


def _host_puente_arg(v):
    try:
        return validar_host_puente(v)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc))


def build_parser():
    ap = argparse.ArgumentParser(description="Estación web 2× QDrone 2")
    ap.add_argument("--config", default=DEFAULT_PATH)
    ap.add_argument("--fuente", choices=FUENTES)
    ap.add_argument("--host", help="IP donde escucha la web (por defecto 127.0.0.1, solo esta PC)")
    ap.add_argument("--puerto", type=int)
    ap.add_argument("--velocidad-mock", type=float, default=1.0)
    ap.add_argument("--host-puente", type=_host_puente_arg, default=None,
                    help="IP de la PC donde corre el modelo de tierra (Stream Server :18700), p. ej. 192.168.2.11. "
                         "Solo con --fuente simulink; no se guarda en estacion.json")
    ap.add_argument("--registro-vuelos", choices=tuple(REGISTRO_CLI), default=None,
                    help="registro de observaciones por vuelo en web/datos/vuelos: auto (por defecto: solo con la fuente "
                         "simulink), si o no. Pisa web.registro_vuelos de estacion.json")
    ap.add_argument("--caja-negra", choices=tuple(CAJA_CLI), default=None,
                    help="caja negra por vuelo (telemetría cruda, comandos, avisos; *.tel.jsonl.gz en web/datos/vuelos): "
                         "auto (por defecto: solo con la fuente simulink), si o no. Pisa web.caja_negra de estacion.json")
    ap.add_argument("--repetir", metavar="ARCHIVO", default=None,
                    help="repite una caja negra (.tel.jsonl.gz) por el mismo camino que la telemetría real, en solo "
                         "lectura (comandos -> 409); usa la config grabada en el archivo")
    ap.add_argument("--velocidad", type=float, default=1.0, help="velocidad de la repetición (con --repetir)")
    ap.add_argument("--imprimir-puerto", action="store_true", help="solo imprime el puerto web y sale (lo usa el .bat)")
    return ap


def main(argv=None):
    ap = build_parser()
    a = ap.parse_args(argv)
    if a.imprimir_puerto:
        print(a.puerto or int(Config(a.config).get("red", "puerto_web", default=8080)))
        return
    import uvicorn
    try:
        station = Station(a.config, a.fuente, a.velocidad_mock, host_puente=a.host_puente,
                          registro_vuelos=a.registro_vuelos, caja_negra=a.caja_negra, repetir=a.repetir,
                          velocidad_repeticion=a.velocidad)
    except (ValueError, OSError) as exc:
        ap.error(str(exc))
    host = a.host or station.cfg.web("host") or "127.0.0.1"
    port = a.puerto or int(station.cfg.get("red", "puerto_web", default=8080))
    if host not in ("127.0.0.1", "localhost"):
        print(f"  AVISO: la estación queda accesible desde la red en {host}:{port} (sin contraseña).")
    if station.fuente == "simulink" and not es_loopback(station.host_puente()):
        print(f"  Puente remoto: modelo de tierra en tcpip://{station.host_puente()}:"
              f"{int(station.cfg.get('red', 'puerto_puente', default=18700))}")
    m = mision_modo(station.cfg.data)
    print(f"  Modo de misión (estacion.json de esta estación): {_modo_txt(m)}"
          + (f"  AVISO: {m['aviso']}" if m["aviso"] else ""))
    print(f"  Registro de vuelos: {station.registro_modo()[1]} -> {station.registro.estado()['carpeta']}")
    print(f"  Caja negra: {station.caja_modo()[1]} -> {station.caja.estado()['carpeta']}")
    if station.grabacion is not None:
        g = station.grabacion
        print(f"  REPETICIÓN de {g.ruta}\n    {g.cabecera.get('fecha')}, fuente {g.fuente}, {len(g.tel)} paquetes, "
              f"{g.t_fin - g.t_ini:.1f} s, velocidad {a.velocidad:g}x"
              + ("  (archivo cortado: se repite hasta donde se alcanzó a escribir)" if g.fin is None else ""))
    print(f"\n  Estación QDrone 2 ({station.fuente}) en  http://127.0.0.1:{port}\n")
    uvicorn.run(create_app(station), host=host, port=port, log_level="warning")


if __name__ == "__main__":
    main()
