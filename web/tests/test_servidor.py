"""Servidor FastAPI en modo mock: endpoints REST, WebSocket, configuración y video."""
import json
import os
import time

import pytest
from fastapi.testclient import TestClient

from app.server import DATA_DIR, Station, create_app


def wait(fn, timeout=10.0):
    end = time.time() + timeout
    while time.time() < end:
        v = fn()
        if v:
            return v
        time.sleep(0.05)
    return None


@pytest.fixture
def client(cfg_path):
    st = Station(cfg_path, "mock", mock_speed=4.0)
    with TestClient(create_app(st)) as c:
        c.station = st
        c.cfg_path = cfg_path
        yield c


def estado(c):
    return c.get("/api/estado").json()


def test_pagina_y_estado(client):
    r = client.get("/")
    assert r.status_code == 200 and "Estación QDrone 2" in r.text and "three@0.160.0" in r.text
    assert client.get("/static/app.js").status_code == 200
    s = wait(lambda: (lambda e: e if e["tel"] and e["tel"]["mission_state_name"] == "ASSIGN_ROLES" else None)(estado(client)))
    assert s and s["fuente"] == "mock" and s["puente"]["connected"]
    assert s["tel"]["explorer_id"] == 2 and s["tel"]["responder_id"] == 1
    assert len(s["pista"]) >= 2 and len(s["verdad_mock"]) == 3


def test_comandos_y_ack(client):
    # START solo aplica con roles asignados (igual que el supervisor de Simulink)
    wait(lambda: (estado(client)["tel"] or {}).get("mission_state_name") == "ASSIGN_ROLES")
    assert client.post("/api/cmd", json={"cmd": "VOLAR"}).status_code == 400
    r = client.post("/api/cmd", json={"cmd": "START"}).json()
    assert r["ok"] and r["cmd_seq"] == 1
    s = wait(lambda: (lambda e: e if e["tel"]["cmd_seq_ack"] == 1 and e["tel"]["mission_state"] >= 3 else None)(estado(client)))
    assert s, "el mock no confirmó el START"
    assert client.post("/api/cmd", json={"cmd": "PAUSE"}).json()["cmd_seq"] == 2


def test_websocket(client):
    with client.websocket_connect("/ws") as ws:
        msgs = [ws.receive_json() for _ in range(5)]
    assert all("puente" in m and "objetivos" in m and "camara" in m for m in msgs)
    assert msgs[-1]["t_servidor"] > msgs[0]["t_servidor"]


def test_red_y_comandos(client):
    assert client.post("/api/config/red", json={"ip_dron1": "999.1.1.1"}).status_code == 400
    r = client.post("/api/config/red", json={"ip_dron1": "192.168.2.16", "ip_pc": "192.168.2.11"})
    assert r.status_code == 200
    disk = json.load(open(client.cfg_path, encoding="utf-8"))
    assert disk["red"]["ip_dron1"] == "192.168.2.16" and disk["red"]["ip_dron2"] == ""
    cmds = client.get("/api/comandos").json()
    d1, d2 = cmds["drones"]["1"], cmds["drones"]["2"]
    assert d1["uri_host"] == "-URI_Host tcpip://192.168.2.11:18001" and not d1["pendiente"]
    assert "tcpip://192.168.2.16:17000" in d1["quarc_run"] and "QD2_DroneStack_PID_2021a.rt-linux_qdrone2" in d1["quarc_run"]
    assert d2["pendiente"] and "<IP_DRON2>" in d2["quarc_run"] and d2["uri_host"].endswith(":18002")
    red = client.get("/api/red").json()
    assert isinstance(red["interfaces"], list)


def test_motive(client, tmp_path):
    carpeta = tmp_path / "OptiTrack" / "Nov 20_Team2"
    carpeta.mkdir(parents=True)
    (carpeta / "Calibration Exceptional (MeanErr 0.494 mm).cal").write_text("x")
    (carpeta / "RigidBody.motive").write_text("y")
    (carpeta / "otra_cosa.txt").write_text("z")
    r = client.get("/api/optitrack/archivos", params={"carpeta": str(tmp_path / "OptiTrack")}).json()
    assert r["ok"] and len(r["cal"]) == 1 and len(r["motive"]) == 1
    assert client.get("/api/optitrack/archivos", params={"carpeta": str(tmp_path / "no_existe")}).json()["ok"] is False
    assert client.post("/api/config/optitrack", json={"rigid_body_dron1": 3, "rigid_body_dron2": 3}).status_code == 400
    assert client.post("/api/config/optitrack", json={"archivo_cal": "a.txt"}).status_code == 400
    ok = client.post("/api/config/optitrack", json={"archivo_cal": r["cal"][0]["ruta"], "archivo_motive": r["motive"][0]["ruta"],
                                                    "rigid_body_dron1": 2, "rigid_body_dron2": 1})
    assert ok.status_code == 200
    disk = json.load(open(client.cfg_path, encoding="utf-8"))["optitrack"]
    assert disk["rigid_body_dron1"] == 2 and disk["archivo_motive"].endswith("RigidBody.motive")
    assert disk["bias_z_dron1"] == -0.07                     # lo demás intacto
    up = client.post("/api/optitrack/subir", files={"archivo": ("prueba_subida.cal", b"cal", "application/octet-stream")}).json()
    try:
        assert up["ok"] and up["tipo"] == "cal" and os.path.isfile(up["ruta"])
    finally:
        os.remove(os.path.join(DATA_DIR, "optitrack", "prueba_subida.cal"))
    assert client.post("/api/optitrack/subir", files={"archivo": ("x.exe", b"no", "application/octet-stream")}).status_code == 400


def test_pista_y_captura(client):
    assert client.post("/api/config/pista", json={"vertices": [[0, 0]]}).status_code == 400
    assert client.post("/api/config/pista", json={"vertices": [[0, 0], [2.0, 0]]}).status_code == 400
    v = [[-1.1, 1.15], [1.1, 1.2], [-1.15, -1.1], [1.2, -1.2]]
    assert client.post("/api/config/pista", json={"vertices": v}).status_code == 200
    assert json.load(open(client.cfg_path, encoding="utf-8"))["pista"]["vertices"] == v
    wait(lambda: client.station.bridge.fresh())
    cap = client.post("/api/pista/capturar", json={"dron": 2}).json()
    assert cap["ok"] and cap["rigid_body"] == 2 and cap["x"] == pytest.approx(-0.9, abs=0.01)


def test_objetivo_manual_se_publica(client):
    wait(lambda: estado(client)["tel"])
    r = client.post("/api/objetivos/manual", json={"cls": "toma", "x": 0.3, "y": -0.4}).json()
    assert r["ok"] and r["id"] == 1
    s = wait(lambda: (lambda e: e if e["tel"]["n_targets_known"] == 1 else None)(estado(client)))
    assert s, "el mock no recibió el objetivo por el puente"
    t = s["objetivos"][0]
    assert t["clase"] == "toma" and t["estado"] == "detectada" and t["manual"]


def test_video(client):
    assert wait(lambda: client.station.camera.frame_id > 2)
    r = client.get("/api/camara.jpg")
    assert r.status_code == 200 and r.headers["content-type"] == "image/jpeg" and r.content[:2] == b"\xff\xd8"


def test_cambio_de_fuente(client):
    assert client.post("/api/fuente", json={"fuente": "nada"}).status_code == 400
    r = client.post("/api/fuente", json={"fuente": "simulink"}).json()
    assert r["fuente"] == "simulink"
    assert client.station.bridge.port == 18700 and client.station.mock is None
    assert json.load(open(client.cfg_path, encoding="utf-8"))["web"]["fuente"] == "simulink"
    client.post("/api/fuente", json={"fuente": "mock"})
    assert wait(lambda: client.station.bridge.fresh())
