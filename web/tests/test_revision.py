"""Pruebas de los defectos de la revisión adversarial (comandos, intrínsecos, ids, mock, seguridad)."""
import hashlib
import json
import math
import os
import shutil
import socket
import time

import numpy as np
import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from app import protocol as P
from app.bridge import BridgeClient, PuenteNoListo
from app.config import Config
from app.geo import CameraModel, intrinsics_from_config
from app.mock_sim import MockMission, MockServer, hdist
from app.mock_world import ColorDetector, path_points, render_view
from app.server import Station, create_app
from test_mock_mision import Driver
from test_puente import FakeSimulink, wait

PROJECT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REAL_CFG = os.path.join(PROJECT, "config", "estacion.json")


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _cfg(path):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def _patch_cfg(path, fn):
    d = _cfg(path)
    fn(d)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(d, fh, indent=2)


# =============================================================================== 1 y 9: comandos
def test_comando_rechazado_sin_telemetria():
    br = BridgeClient("127.0.0.1", _free_port(), reconnect_s=0.2).start()
    try:
        with pytest.raises(PuenteNoListo):
            br.send_command("START")
        st = br.status()
        assert st["cmd_seq"] == 0 and st["cmd"] == "NADA" and st["cola"] == []
    finally:
        br.stop()


def test_comando_sin_ack_caduca_y_no_se_ejecuta_al_reconectar():
    sim = FakeSimulink()
    sim.acking = False                                   # Simulink no contesta (enlace roto a medias)
    br = BridgeClient("127.0.0.1", sim.port, reconnect_s=0.2, cmd_timeout_s=0.6).start()
    try:
        assert wait(br.fresh)
        assert br.send_command("START") == 1
        assert wait(lambda: any(m["cmd"] == P.CMD_START for _, m in sim.rx))
        assert wait(lambda: br.status()["cmd_resultados"] and br.status()["cmd_resultados"][-1]["estado"] == "caducado", 3)
        n0 = len(sim.rx)
        assert wait(lambda: len(sim.rx) > n0 + 5)
        last = [m for _, m in sim.rx[n0:]]
        assert all(m["cmd"] == 0 and m["cmd_seq"] == 2 for m in last)   # cmd_seq cambia con cmd = 0
        # el puente se cae y vuelve: nunca se vuelve a mandar START
        sim.kill_connection()
        assert wait(lambda: br.status()["reconnects"] >= 1 and br.fresh(), 6)
        n1 = len(sim.rx)
        assert wait(lambda: len(sim.rx) > n1 + 5)
        assert all(m["cmd"] == 0 for _, m in sim.rx[n1:])
    finally:
        br.stop()
        sim.close()


def test_cola_abort_luego_land_all_llegan_los_dos():
    sim = FakeSimulink()
    br = BridgeClient("127.0.0.1", sim.port).start()
    try:
        assert wait(br.fresh)
        s1 = br.send_command("ABORT")
        s2 = br.send_command("LAND_ALL")                  # en el mismo periodo de 50 ms
        assert s1 == 1 and s2 is None and br.status()["cola"] == ["LAND_ALL"]
        assert wait(lambda: [r["estado"] for r in br.status()["cmd_resultados"]] == ["recibido", "recibido"], 3)
        sent = [(m["cmd"], m["cmd_seq"]) for _, m in sim.rx if m["cmd"]]
        assert (P.CMD_ABORT, 1) in sent and (P.CMD_LAND_ALL, 2) in sent
        assert sent.index((P.CMD_ABORT, 1)) < sent.index((P.CMD_LAND_ALL, 2))
    finally:
        br.stop()
        sim.close()


def test_emergencia_reemplaza_y_descarta_comandos_normales():
    sim = FakeSimulink()
    sim.acking = False
    br = BridgeClient("127.0.0.1", sim.port, cmd_timeout_s=5.0).start()
    try:
        assert wait(br.fresh)
        assert br.send_command("START") == 1
        assert br.send_command("PAUSE") is None           # en cola detrás de START
        assert br.send_command("ABORT") == 2              # reemplaza a START y descarta PAUSE
        res = {r["cmd"]: r["estado"] for r in br.status()["cmd_resultados"]}
        assert res == {"START": "reemplazado", "PAUSE": "descartado"}
        assert wait(lambda: sim.rx[-1][1]["cmd"] == P.CMD_ABORT and sim.rx[-1][1]["cmd_seq"] == 2)
    finally:
        br.stop()
        sim.close()


def test_cmd_seq_nuevo_distinto_del_ack_de_una_sesion_anterior():
    sim = FakeSimulink()
    sim.ack = 7                                           # Simulink conserva el ack de la estación anterior
    br = BridgeClient("127.0.0.1", sim.port).start()
    try:
        assert wait(lambda: br.fresh() and br.status()["cmd_seq_ack"] is not None)
        # (la estación nueva manda cmd_seq 0 con cmd 0: Simulink aplica "nada" y el ack pasa a 0)
        assert br.send_command("RESUME") >= 1
        assert wait(lambda: br.status()["cmd_resultados"] and br.status()["cmd_resultados"][-1]["estado"] == "recibido", 3)
    finally:
        br.stop()
        sim.close()


# =============================================================================== 2: intrínsecos
def test_intrinsecos_de_la_config():
    cfg = _cfg(REAL_CFG)
    fx, fy, cx, cy, origen = intrinsics_from_config(cfg, 640, 480)
    assert (fx, fy, cx, cy) == (617.1, 617.1, 320.0, 240.0) and "intrinsecos_nominales" in origen
    cam = CameraModel.from_config(cfg["camara"], cfg, (640, 480))
    assert cam.fx == cam.fy == 617.1 and cam.hfov_deg == pytest.approx(54.8, abs=0.1)
    # camara.intrinsecos (medidos con rs2) tienen prioridad
    cfg["camara"]["intrinsecos"] = {"640x480": {"fx": 610.0, "fy": 611.0, "cx": 318.2, "cy": 243.9}}
    cam = CameraModel.from_config(cfg["camara"], cfg, (640, 480))
    assert (cam.fx, cam.fy, cam.cx, cam.cy) == (610.0, 611.0, 318.2, 243.9)
    # imagen reducida: mismo sensor escalado
    small = CameraModel.from_config(cfg["camara"], cfg, (320, 240))
    assert small.fx == pytest.approx(305.0) and small.cy == pytest.approx((243.9 + 0.5) / 2 - 0.5)


@pytest.mark.parametrize("giro", [90, 180, 270])
def test_imagen_girada_en_el_dron(giro):
    """Con --rotar en el streamer (camara.giro_en_dron) un objeto da el mismo punto en el suelo."""
    import cv2
    cfg = _cfg(REAL_CFG)
    cfg["camara"]["intrinsecos"] = {"640x480": {"fx": 600.0, "fy": 630.0, "cx": 330.0, "cy": 235.0}}
    base = CameraModel.from_config(cfg["camara"], cfg, (640, 480))
    pose = (0.2, -0.3, 1.5, math.radians(30))
    img = np.zeros((480, 640), np.uint8)
    u, v = 517, 101
    img[v, u] = 255
    rot = cv2.rotate(img, {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}[giro])
    v2, u2 = np.argwhere(rot == 255)[0]
    cfg["camara"]["giro_en_dron"] = giro
    # rotacion_montaje describe la imagen TAL COMO LLEGA: con --rotar G se le resta G
    cfg["camara"]["rotacion_montaje"] = (cfg["camara"]["rotacion_montaje"] - giro) % 360
    m = CameraModel.from_config(cfg["camara"], cfg, (rot.shape[1], rot.shape[0]))
    assert (m.w, m.h) == (rot.shape[1], rot.shape[0]) and m.aviso == ""
    assert m.pixel_to_ground(u2, v2, *pose) == pytest.approx(base.pixel_to_ground(u, v, *pose), abs=1e-9)
    if giro in (90, 270):
        assert (m.fx, m.fy) == (630.0, 600.0)


def test_render_con_intrinsecos_reales_y_geolocalizacion_con_la_config():
    """La imagen se dibuja con los intrínsecos 'reales' y se geolocaliza con los de estacion.json
    (no circular). Con el modelo viejo por HFOV 69.4° el error en el borde pasaba de 20 cm."""
    cfg = _cfg(REAL_CFG)
    real = CameraModel(640, 480, 0, 0, 0, cfg["camara"]["offset_camara_m"], fx=617.1, fy=617.1, cx=320.0, cy=240.0)
    est = CameraModel.from_config(cfg["camara"], cfg, (640, 480))
    viejo = CameraModel(640, 480, 69.4, 42.5, 0, cfg["camara"]["offset_camara_m"])
    pose = (0.0, 0.0, 1.5, 0.0)
    errs_est, errs_viejo = [], []
    for u_obj in (330, 450, 560, 610):                   # objeto cada vez más al borde de la imagen
        # punto del suelo que cae en (u_obj, 240) con la cámara real
        x, y = real.pixel_to_ground(u_obj, 240, *pose, z_plane=0.03)
        truth = [{"cls": 0, "x": x, "y": y}]
        img = render_view(real, pose, [[-1.2, 1.2], [1.2, 1.2]], truth, rng=np.random.default_rng(1))
        dets = [d for d in ColorDetector().detect(img) if d[0] == 0]
        assert dets, u_obj
        _, _, x1, y1, x2, y2 = dets[0]
        uc, vc = (x1 + x2) / 2, (y1 + y2) / 2
        g = est.pixel_to_ground(uc, vc, *pose)
        gv = viejo.pixel_to_ground(uc, vc, *pose)
        errs_est.append(math.hypot(g[0] - x, g[1] - y))
        errs_viejo.append(math.hypot(gv[0] - x, gv[1] - y))
    assert max(errs_est) < 0.03, errs_est
    assert max(errs_viejo) > 0.15, errs_viejo             # el defecto que se corrigió


# =============================================================================== 11: config
def test_arrancar_no_reescribe_estacion_json(tmp_path):
    dst = tmp_path / "estacion.json"
    shutil.copy(REAL_CFG, dst)
    h0 = hashlib.md5(dst.read_bytes()).hexdigest()
    st = Station(str(dst), "mock")
    assert st.cfg.web("fuente") == "mock" and st.cfg.web("cmd_timeout_s") == 1.5   # defaults en memoria
    assert hashlib.md5(dst.read_bytes()).hexdigest() == h0
    Config(str(dst)).update({})                          # guardar sin cambios = mismos bytes
    assert hashlib.md5(dst.read_bytes()).hexdigest() == h0
    Config(str(dst)).update({"red": {"ip_dron2": "192.168.2.17"}})
    a = open(REAL_CFG, "rb").read().splitlines()
    b = dst.read_bytes().splitlines()
    assert [i for i, (x, y) in enumerate(zip(a, b)) if x != y] == [6]   # solo cambió la línea de ip_dron2
    assert (b"\r\n" in open(REAL_CFG, "rb").read()) == (b"\r\n" in dst.read_bytes())


# =============================================================================== 5 y 6: mock
def test_mock_primer_contacto_absorbe_comando_viejo(cfg_path):
    d = Driver(_cfg(cfg_path))
    d.seq, d.cmd = 5, P.CMD_START                        # la estación trae un START viejo
    d.run_until(lambda: False, 4)
    assert d.m.state == P.ASSIGN_ROLES and d.m.cmd_seq_ack == 5


def test_mock_reinicio_de_estacion_y_reglas_de_simulink(cfg_path):
    d = Driver(_cfg(cfg_path))
    d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    d.send("START")
    assert d.run_until(lambda: d.m.state == P.TAKEOFF, 1)
    d.send("PAUSE")                                      # PAUSE en TAKEOFF: Simulink lo ignora
    d.step(5)
    assert d.m.state == P.TAKEOFF and d.m.cmd_seq_ack == 2
    assert d.run_until(lambda: d.m.state == P.MISSION, 15)
    d.send("PAUSE")
    assert d.run_until(lambda: d.m.state == P.PAUSED, 0.5)
    # estación reiniciada: cmd_seq vuelve a 0 con cmd = 0 (Simulink lo aplica como "nada")
    d.seq, d.cmd = 0, 0
    d.step(5)
    assert d.m.cmd_seq_ack == 0 and d.m.state == P.PAUSED
    d.send("RESUME")                                     # cmd_seq 1 de la estación nueva
    assert d.run_until(lambda: d.m.state == P.MISSION, 1)


def test_mock_start_en_done_ignorado(cfg_path):
    d = Driver(_cfg(cfg_path))
    d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    d.send("START")
    d.run_until(lambda: d.m.state == P.MISSION, 15)
    d.send("LAND_ALL")
    assert d.run_until(lambda: d.m.state == P.DONE, 20)
    d.send("START")
    d.step(10)
    assert d.m.state == P.DONE


def test_mock_invierte_la_n_hacia_el_home_de_e(cfg_path):
    cfg = _cfg(cfg_path)
    v = cfg["pista"]["vertices"]
    d = Driver(cfg, homes=((1.0, 1.0), (-0.2, -1.2)))  # E = dron 2, cerca del ÚLTIMO vértice
    d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    d.send("START")
    d.step(3)
    assert d.m.E == 2 and tuple(d.m.wps[0][:2]) == pytest.approx(tuple(v[-1]))
    assert tuple(d.m.wps[-1][:2]) == pytest.approx(tuple(v[0]))


def _mision_aleatoria(cfg, seed, dt):
    rng = np.random.default_rng(seed)
    while True:
        h1 = (rng.uniform(0.4, 1.3), rng.uniform(-1.3, 1.3))
        h2 = (rng.uniform(-1.3, -0.4), rng.uniform(-1.3, 1.3))
        if hdist(h1, h2) > 1.2:
            break
    m = MockMission(cfg, homes=(h1, h2))
    n = int(rng.integers(1, 6))
    an = [(k + 1, int(rng.integers(0, 2)), x + rng.normal(0, 0.1), y + rng.normal(0, 0.1), 0.9)
          for k, (x, y, _, _) in enumerate(path_points(cfg["pista"]["vertices"], sorted(rng.uniform(0.02, 0.98, n))))]
    radii = rng.uniform(0.1, 1.8, n)
    pub, hb, seq, cmd, dmin = [], 0, 0, 0, 9.0
    while m.t < 200:
        hb += 1
        if m.state == P.ASSIGN_ROLES and seq == 0:
            seq, cmd = 1, P.CMD_START
        dE = m.drones.get(m.E)
        if dE is not None and m.state == P.MISSION:
            pub += [a for k, a in enumerate(an) if a not in pub and hdist(dE.p, a[2:4]) < radii[k]]
        k0 = (hb * 3) % max(1, len(pub))
        m.apply_command_message(P.unpack_command(P.pack_command(hb, cmd, seq, len(pub), (pub[k0:] + pub[:k0])[:3])))
        if m.cmd_seq_ack == seq:
            cmd = 0
        m.step(dt)
        dR = m.drones.get(m.R)
        if dE is not None and dE.flying and dR.flying and dE.p[2] > 0.3 and dR.p[2] > 0.3:
            dmin = min(dmin, hdist(dE.p, dR.p))
        if m.state == P.RETURN_HOME and all(hdist(x.p, x.home) < 0.12 for x in m.drones.values()):
            break
    return dmin, m, len(pub)


@pytest.mark.parametrize("seed", range(10))
def test_guard_pid_con_tiempos_aleatorios(cfg_path, seed):
    """Homes, anomalías, momento de publicación y paso de tiempo aleatorios: nunca < d_min_h y termina."""
    cfg = _cfg(cfg_path)
    dmin, m, npub = _mision_aleatoria(cfg, 1000 + seed, [0.02, 0.04, 0.08][seed % 3])
    assert dmin >= cfg["mision"]["d_min_h"], dmin
    # visited_mask cuenta también los ids que el mock (como Simulink) guardó como alias de uno visitado
    vis = P.unpack_telemetry(P.pack_telemetry(m.telemetry()))["visited_ids"]
    assert m.state == P.RETURN_HOME and len(vis) == npub and m.t < 199


# =============================================================================== servidor
@pytest.fixture
def client(cfg_path):
    st = Station(cfg_path, "mock", mock_speed=4.0)
    with TestClient(create_app(st)) as c:
        c.station = st
        c.cfg_path = cfg_path
        yield c


def test_cmd_409_si_el_puente_no_esta(cfg_path):
    _patch_cfg(cfg_path, lambda d: d["red"].update(puerto_puente=_free_port()))
    st = Station(cfg_path, "simulink")
    with TestClient(create_app(st)) as c:
        r = c.post("/api/cmd", json={"cmd": "START"})
        assert r.status_code == 409 and "NO se envió" in r.json()["detail"]
        assert st.bridge.status()["cmd_seq"] == 0


def test_origen_ajeno_rechazado(client):
    wait(lambda: client.station.bridge.fresh())
    r = client.post("/api/cmd", json={"cmd": "PAUSE"}, headers={"Origin": "http://pagina-ajena.example"})
    assert r.status_code == 403
    up = client.post("/api/optitrack/subir", files={"archivo": ("x.cal", b"x", "application/octet-stream")},
                     headers={"Origin": "http://pagina-ajena.example"})
    assert up.status_code == 403
    assert client.post("/api/cmd", json={"cmd": "PAUSE"}, headers={"Origin": "http://testserver"}).status_code == 200
    with pytest.raises(WebSocketDisconnect):
        with client.websocket_connect("/ws", headers={"Origin": "http://pagina-ajena.example"}) as ws:
            ws.receive_json()
    with client.websocket_connect("/ws", headers={"Origin": "http://testserver"}) as ws:
        assert "puente" in ws.receive_json()


def test_validaciones(client):
    v17 = [[-1.2 + 0.1 * k, 0.0] for k in range(17)]
    assert client.post("/api/config/pista", json={"vertices": v17}).status_code == 400
    assert client.post("/api/config/pista", json={"vertices": v17[:16]}).status_code == 200
    assert client.post("/api/objetivos/manual", json={"cls": 5, "x": 0, "y": 0}).status_code == 400
    assert client.post("/api/objetivos/manual", json={"cls": "humo", "x": 0, "y": 0}).status_code == 400
    assert client.post("/api/objetivos/manual", json={"cls": 0, "x": 3.0, "y": 0}).status_code == 400
    assert client.post("/api/config/red", json={"ip_dron1": "192.168.2.20", "ip_dron2": "192.168.2.20"}).status_code == 400
    assert client.post("/api/config/red", json={"ip_dron1": "192.168.2.20"}).status_code == 200
    assert client.post("/api/config/red", json={"ip_dron2": "192.168.2.20"}).status_code == 400


def test_reiniciar_lista_bloqueado_si_simulink_tiene_objetivos(client):
    assert wait(lambda: client.station.bridge.fresh())
    assert client.post("/api/objetivos/reiniciar").status_code == 200       # Simulink vacío: se puede
    assert client.post("/api/objetivos/manual", json={"cls": "toma", "x": 0.3, "y": -0.4}).json()["id"] == 1
    assert wait(lambda: client.station.bridge.telemetry["n_targets_known"] == 1)
    r = client.post("/api/objetivos/reiniciar")
    assert r.status_code == 409 and "ya tiene 1" in r.json()["detail"]


def test_ids_no_se_reinician_al_reiniciar_la_estacion(cfg_path):
    cfg = _cfg(cfg_path)
    mock = MockServer(cfg, "127.0.0.1", 0, speed=2.0).start()
    _patch_cfg(cfg_path, lambda d: d["red"].update(puerto_puente=mock.port))
    try:
        st = Station(cfg_path, "simulink").start()
        assert wait(lambda: st.bridge.fresh())
        assert st.vision.add_manual(1, -0.2, -0.05) == 1
        assert wait(lambda: mock.mission.telemetry()["n_targets_known"] == 1)
        st.stop()
        # estación nueva (mismo Simulink): recupera la tabla y no reutiliza el id 1
        st2 = Station(cfg_path, "simulink").start()
        assert wait(lambda: st2.bridge.fresh())
        time.sleep(0.3)
        tg, _ = st2.vision.targets()
        assert [(t["id"], t["cls"]) for t in tg] == [(1, 1)]
        assert st2.vision.add_manual(0, 0.6, 1.28) == 2
        assert wait(lambda: mock.mission.telemetry()["n_targets_known"] == 2)
        assert mock.mission.targets[1]["x"] == pytest.approx(-0.2)          # el 1 no cambió de lugar
        # Simulink se reinicia (t vuelve a 0, tabla vacía): la estación también empieza de cero
        mock.reset()
        assert wait(lambda: st2.vision.targets()[0] == [], 5)
        assert st2.vision.add_manual(0, 0.1, 0.1) == 1
        st2.stop()
        # estación nueva contra un Simulink sin objetivos con una tabla guardada vieja: se descarta
        mock.reset()
        st3 = Station(cfg_path, "simulink").start()
        assert wait(lambda: st3.bridge.fresh())
        assert wait(lambda: st3.vision.targets()[0] == [], 3)
        st3.stop()
    finally:
        mock.stop()


# =============================================================================== 3 y 7: avisos
def test_sintetica_bloqueada_con_simulink(cfg_path):
    _patch_cfg(cfg_path, lambda d: (d["camara"].update(fuente="sintetica"), d["red"].update(puerto_puente=_free_port())))
    st = Station(cfg_path, "simulink")
    assert st._camera_key() == ("ninguna",) and not st._synthetic()
    av = st.avisos()
    assert any(a["nivel"] == "rojo" and "BLOQUEADA" in a["msg"] for a in av)
    st3 = Station(cfg_path, "sim3d")
    assert st3._camera_key() == ("sintetica",) and st3._synthetic()


def test_detector_sin_tomas_aviso_rojo(cfg_path):
    _patch_cfg(cfg_path, lambda d: d.setdefault("web", {}).update(detector="ninguno"))
    st = Station(cfg_path, "simulink")
    assert any(a["nivel"] == "rojo" and "NO detecta tomas" in a["msg"] for a in st.avisos())
    _patch_cfg(cfg_path, lambda d: d["web"].update(detector="falso"))
    st = Station(cfg_path, "simulink")                     # falso con cámara real: no se carga
    assert st.vision.detector is None and "solo se permite con la cámara sintética" in st.vision.detector_error
