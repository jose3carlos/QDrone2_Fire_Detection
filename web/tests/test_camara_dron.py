"""Cámara del explorador (``camara.dron``): el video se conecta SIEMPRE a ``red.ip_dron{camara.dron}``, START se
rechaza (409) si el explorador no es ese dron (dual: y(4); solo: dron_solo) y el aviso rojo "Sin video del
explorador" sale con la visión activa y sin frames de la cámara.

Ninguna prueba abre conexiones: el puente es falso y la clave de la cámara solo se calcula (IPs 10.0.0.x).
"""
import json
import time

import pytest
from fastapi.testclient import TestClient

from app import protocol as P
from app.config import Config, camara_dron
from app.server import DX_ROLES_MIN, SIN_VIDEO_S, CamaraDistinta, ModoDistinto, Station, create_app
from test_solo_explorador import _hay, _set, _station, _tel_modo

IP1, IP2 = "10.0.0.1", "10.0.0.2"
TXT = "El explorador es el dron {e} pero la cámara está en el dron {c}"
SIN_VIDEO = "Sin video del explorador: no se detecta nada"


def _cam(path, dron, fuente=None):
    def fn(c):
        c["camara"]["dron"] = dron
        if fuente is not None:
            c["camara"]["fuente"] = fuente
        c["red"].update(ip_dron1=IP1, ip_dron2=IP2)
    return _set(path, fn)


# =============================================================================== 1. camara.dron en la config
def test_camara_dron_tabla(tmp_path):
    casos = [({}, 1, False), ({"dron": None}, 1, False), ({"dron": []}, 1, False), ({"dron": 1}, 1, False),
             ({"dron": 2}, 2, False), ({"dron": 2.0}, 2, False), ({"dron": 0}, 1, True), ({"dron": 3}, 1, True),
             ({"dron": 1.5}, 1, True), ({"dron": "2"}, 1, True), ({"dron": True}, 1, True), ({"dron": [1, 2]}, 1, True)]
    for k, (cam, dron, aviso) in enumerate(casos):
        r = camara_dron({"camara": cam})
        assert (r["dron"], bool(r["aviso"])) == (dron, aviso), (cam, r)
        p = tmp_path / f"c{k}.json"                       # también con los números como los guarda Config (_Num)
        p.write_text(json.dumps({"camara": cam}), encoding="utf-8")
        r2 = camara_dron(Config(str(p)).data)
        assert (r2["dron"], bool(r2["aviso"])) == (dron, aviso), (cam, r2)
    assert camara_dron({}) == {"dron": 1, "aviso": None}
    assert camara_dron({"camara": {"dron": float("nan")}})["aviso"]


def test_configs_del_repo_tienen_camara_dron():
    import os
    from conftest import PROJECT
    for nombre in ("estacion.json", "estacion_dual.json"):
        c = Config(os.path.join(PROJECT, "config", nombre)).data
        assert "dron" in c["camara"] and camara_dron(c) == {"dron": 1, "aviso": None}, nombre


# =============================================================================== 2. video = ip_dron{camara.dron}
def test_video_conectado_al_dron_de_la_camara(cfg_path):
    port = int(Config(cfg_path).get("red", "puerto_video", default=18800))
    st = _station(_cam(cfg_path, 2), _tel_modo(explorer=1, responder=2))
    assert st._camera_key() == ("dron", IP2, port)          # NO la del explorador (dron 1)
    # ASSIGN_ROLES reevalúa los roles antes del START: el video no salta de IP
    st.bridge.telemetry = _tel_modo(explorer=2, responder=1)
    assert st._camera_key() == ("dron", IP2, port)
    st.bridge.telemetry = None
    assert st._camera_key() == ("dron", IP2, port)
    _cam(cfg_path, 1)
    st.cfg.reload()
    assert st._camera_key() == ("dron", IP1, port)
    # sin la IP de ese dron: sin video (no cae a la del otro)
    _set(cfg_path, lambda c: c["red"].update(ip_dron1=""))
    st.cfg.reload()
    assert st._camera_key() == ("ninguna",)


# =============================================================================== 3. START 409 en dual
def test_start_409_dual_explorador_distinto(cfg_path):
    st = _station(_cam(cfg_path, 1), _tel_modo(explorer=2, responder=1))
    msg = TXT.format(e=2, c=1)
    av = st.avisos()
    assert _hay(av, "rojo", msg) and _hay(av, "rojo", "reacomoda los drones (el explorador es el de menor x)")
    # un cruce de rigid body / IP da el mismo rechazo: el texto dice que NO se reacomode si la cámara ya va atrás
    assert _hay(av, "rojo", "si el dron de la CÁMARA ya está en la menor x, los rigid bodies o las IPs están "
                            "cruzados: NO reacomodes, revisa Motive y Conexiones")
    with pytest.raises(CamaraDistinta):
        st.command("START")
    assert st.bridge.sent == []
    # ABORT, LAND, PAUSE y RESUME nunca se bloquean
    for c in ("PAUSE", "RESUME", "ABORT", "LAND_ALL"):
        st.command(c)
    assert st.bridge.sent == ["PAUSE", "RESUME", "ABORT", "LAND_ALL"]
    with TestClient(create_app(st, start=False)) as c:
        r = c.post("/api/cmd", json={"cmd": "START"})
        assert r.status_code == 409 and msg in r.json()["detail"] and "camara.dron" in r.json()["detail"]
        for cmd in ("ABORT", "LAND_ALL", "PAUSE"):
            assert c.post("/api/cmd", json={"cmd": cmd}).status_code == 200, cmd
        cam = c.get("/api/estado").json()["camara"]
        assert (cam["dron"], cam["explorador"], cam["coincide"]) == (1, 2, False)
    assert "START" not in st.bridge.sent
    assert any("START rechazado: " + msg in e["msg"] for e in st.events)
    # cámara en el dron 2: ahora coincide
    _cam(cfg_path, 2)
    st.cfg.reload()
    assert not _hay(st.avisos(), "rojo", "pero la cámara está en el dron")
    assert st.command("START")
    assert st.bridge.sent[-1] == "START"


def test_start_ok_dual_coinciden(cfg_path):
    st = _station(_cam(cfg_path, 1), _tel_modo(explorer=1, responder=2))
    assert not _hay(st.avisos(), "rojo", "pero la cámara está en el dron")
    with TestClient(create_app(st, start=False)) as c:
        r = c.post("/api/cmd", json={"cmd": "START"})
        assert r.status_code == 200 and r.json()["ok"]
        cam = c.get("/api/estado").json()["camara"]
        assert (cam["dron"], cam["explorador"], cam["coincide"]) == (1, 1, True)
    assert st.bridge.sent == ["START"]


def test_start_409_dual_x_casi_iguales(cfg_path):
    """En ASSIGN_ROLES Simulink reevalúa el de menor x en cada tick: con |x1 - x2| < DX_ROLES_MIN el explorador
    puede voltearse entre el paquete que revisa la estación y el tick que procesa START."""
    t = _tel_modo(explorer=1, responder=2)
    t["drones"][1].update(x=-1.0, y=-0.5)
    t["drones"][2].update(x=-1.0 + DX_ROLES_MIN / 5, y=0.5)
    st = _station(_cam(cfg_path, 1), t)
    ci = st.camara_info()
    assert ci["explorador"] == 1 and not ci["coincide"] and "casi la misma x" in ci["msg"]
    assert _hay(st.avisos(), "rojo", "casi la misma x")
    with pytest.raises(CamaraDistinta):
        st.command("START")
    with TestClient(create_app(st, start=False)) as c:
        assert c.post("/api/cmd", json={"cmd": "START"}).status_code == 409
        cam = c.get("/api/estado").json()["camara"]
        assert cam["coincide"] is False and "casi la misma x" in cam["msg"]
        assert c.post("/api/cmd", json={"cmd": "ABORT"}).status_code == 200
    assert st.bridge.sent == ["ABORT"]
    # separados en x lo suficiente: pasa
    t["drones"][2]["x"] = -1.0 + DX_ROLES_MIN * 1.5
    st.bridge.telemetry = t
    assert st.camara_info()["coincide"] and st.command("START")
    assert st.bridge.sent == ["ABORT", "START"]
    # ya en vuelo los roles están fijos: la x parecida no importa
    t = _tel_mision()
    t["drones"][2]["x"] = t["drones"][1]["x"]
    st.bridge.telemetry = t
    assert st.camara_info()["coincide"]
    # sin la cámara física (sintética en sim3d) no aplica
    _cam(cfg_path, 1, fuente="sintetica")
    t = _tel_modo(explorer=1, responder=2)
    t["drones"][2]["x"] = t["drones"][1]["x"]
    st = _station(cfg_path, t, fuente="sim3d")
    assert st.command("START") == 1 and st._camara_ui()["coincide"] and st._camara_ui()["msg"] is None


def test_dual_sin_roles_o_sin_telemetria_no_bloquea(cfg_path):
    _cam(cfg_path, 2)
    # todavía sin explorador (INIT / WAIT_TRACKING): no se sabe, no hay aviso
    st = _station(cfg_path, _tel_modo(explorer=0, responder=0, ms=P.WAIT_TRACKING))
    assert st.camara_info()["explorador"] is None and st.camara_info()["coincide"]
    assert not _hay(st.avisos(), "rojo", "pero la cámara está en el dron")
    # telemetría vieja: no se compara
    st = _station(cfg_path, _tel_modo(explorer=1, responder=2))
    st.bridge._fresh = False
    assert st.camara_info()["coincide"] and not _hay(st.avisos(), "rojo", "pero la cámara está en el dron")


# =============================================================================== 4. START 409 en solo explorador
def test_start_409_solo_dron_distinto(cfg_path_solo):
    st = _station(_cam(cfg_path_solo, 2), _tel_modo(ds=1, responder=0))
    msg = TXT.format(e=1, c=2)
    assert _hay(st.avisos(), "rojo", msg)
    with pytest.raises(CamaraDistinta):
        st.command("START")
    with TestClient(create_app(st, start=False)) as c:
        r = c.post("/api/cmd", json={"cmd": "START"})
        assert r.status_code == 409 and msg in r.json()["detail"]
        assert c.post("/api/cmd", json={"cmd": "ABORT"}).status_code == 200
        assert c.post("/api/cmd", json={"cmd": "LAND_ALL"}).status_code == 200
    assert st.bridge.sent == ["ABORT", "LAND_ALL"]
    # sin telemetría fresca el aviso sale igual (dron_solo de la config)
    st.bridge._fresh = False
    assert _hay(st.avisos(), "rojo", msg)
    # el modo distinto se revisa primero (mismo 409, su propio motivo)
    st = _station(cfg_path_solo, _tel_modo(explorer=2, responder=1))
    with pytest.raises(ModoDistinto):
        st.command("START")


def test_start_ok_solo_coinciden(cfg_path_solo):
    st = _station(_cam(cfg_path_solo, 1), _tel_modo(ds=1, responder=0))
    assert not _hay(st.avisos(), "rojo", "pero la cámara está en el dron")
    assert st.command("START") == 1 and st.bridge.sent == ["START"]
    # dron solo 2 con la cámara en el 2
    _set(cfg_path_solo, lambda c: (c["mision"].update(dron_solo=2), c["camara"].update(dron=2)))
    st = _station(cfg_path_solo, _tel_modo(ds=2, explorer=2, responder=0))
    assert not _hay(st.avisos(), "rojo", "pero la cámara está en el dron")
    assert st.command("START") == 1


def test_no_bloquea_con_camara_virtual(cfg_path):
    """mock (cámara sintética que sigue al explorador) y camara.fuente distinta de "dron": no hay RealSense física."""
    _cam(cfg_path, 1)
    st = _station(cfg_path, _tel_modo(explorer=2, responder=1), fuente="mock")
    assert st.command("START") == 1
    assert not _hay(st.avisos(), "rojo", "pero la cámara está en el dron")
    _cam(cfg_path, 1, fuente="sintetica")
    st = _station(cfg_path, _tel_modo(explorer=2, responder=1), fuente="sim3d")
    assert st.command("START") == 1
    assert st._camara_ui()["coincide"]


def test_camara_dron_invalido_aviso(cfg_path):
    st = _station(_cam(cfg_path, 3), _tel_modo(explorer=1, responder=2))
    assert _hay(st.avisos(), "amarillo", "camara.dron inválido")
    assert st.camara_info()["dron"] == 1 and st.command("START") == 1


# =============================================================================== 5. aviso "Sin video"
def _tel_mision(explorer=1):
    t = _tel_modo(explorer=explorer, responder=3 - explorer, ms=P.MISSION)
    t["drones"][explorer].update(z=1.5)
    return t


def test_aviso_sin_video(cfg_path):
    st = _station(_cam(cfg_path, 1), _tel_mision())
    assert st._vision_gate()
    # nunca llegó un frame
    assert _hay(st.avisos(), "rojo", SIN_VIDEO)
    assert any("Sin video del explorador" in e["msg"] for e in st.events)
    # vuelve el video -> desaparece
    st.camera.stamp = time.time()
    assert not _hay(st.avisos(), "rojo", SIN_VIDEO)
    assert any("Video del explorador de vuelta" in e["msg"] for e in st.events)
    # frames viejos (> SIN_VIDEO_S) -> vuelve el aviso
    st.camera.stamp = time.time() - SIN_VIDEO_S - 0.5
    assert _hay(st.avisos(), "rojo", SIN_VIDEO)
    st.camera.stamp = time.time() - SIN_VIDEO_S + 1.0
    assert not _hay(st.avisos(), "rojo", SIN_VIDEO)
    # solo es un aviso: los comandos siguen pasando
    st.camera.stamp = 0.0
    assert st.command("PAUSE") == 1


def test_sin_video_solo_con_la_vision_activa(cfg_path):
    # en tierra / antes de MISSION: el gate está cerrado y no hay aviso aunque no haya frames
    st = _station(_cam(cfg_path, 1), _tel_modo(explorer=1, responder=2, ms=P.ASSIGN_ROLES))
    assert not st._vision_gate() and not _hay(st.avisos(), "rojo", SIN_VIDEO)
    # explorando, pero bajo web.z_min_deteccion: tampoco
    t = _tel_mision()
    t["drones"][1]["z"] = 0.1
    st.bridge.telemetry = t
    assert not st._vision_gate() and not _hay(st.avisos(), "rojo", SIN_VIDEO)
    # telemetría vieja: gate cerrado
    st.bridge.telemetry = _tel_mision()
    st.bridge._fresh = False
    assert not _hay(st.avisos(), "rojo", SIN_VIDEO)


def test_sin_video_no_dice_de_vuelta_al_cerrar_el_gate(cfg_path):
    """Si el aviso se quita porque se cerró el gate (fin de misión, telemetría vieja) sin que llegue un frame, el
    registro no dice "de vuelta" ni repite "Sin video" con la telemetría intermitente."""
    def n(txt):
        return sum(txt in e["msg"] for e in st.events)
    st = _station(_cam(cfg_path, 1), _tel_mision())
    assert _hay(st.avisos(), "rojo", SIN_VIDEO) and n("Sin video del explorador con la visión activa") == 1
    for _ in range(3):                       # telemetría intermitente
        st.bridge._fresh = False
        assert not _hay(st.avisos(), "rojo", SIN_VIDEO)
        st.bridge._fresh = True
        assert _hay(st.avisos(), "rojo", SIN_VIDEO)
    t = _tel_mision()                        # la misión termina: gate cerrado, sigue sin video
    t["mission_state"] = P.LANDING
    st.bridge.telemetry = t
    assert not st._vision_gate() and not _hay(st.avisos(), "rojo", SIN_VIDEO)
    assert n("Video del explorador de vuelta") == 0 and n("Sin video del explorador con la visión activa") == 1
    # llegan frames (aunque el gate siga cerrado): ahora sí
    st.camera.stamp = time.time()
    st.avisos()
    assert n("Video del explorador de vuelta") == 1


def test_sin_video_con_el_mock_real(cfg_path):
    """Con la cámara sintética del mock llegan frames: nunca sale el aviso mientras explora."""
    st = Station(cfg_path, "mock", mock_speed=6.0)
    with TestClient(create_app(st)) as c:
        from test_puente import wait
        assert wait(lambda: (c.get("/api/estado").json().get("tel") or {}).get("mission_state") == P.ASSIGN_ROLES, 10)
        assert c.post("/api/cmd", json={"cmd": "START"}).status_code == 200
        assert wait(lambda: st._vision_gate(), 30)
        assert wait(lambda: st.camera.latest()[2] > 0, 5)
        s = c.get("/api/estado").json()
        assert not _hay(s["avisos"], "rojo", SIN_VIDEO), s["avisos"]
        assert s["camara"]["dron"] == 1


# =============================================================================== 6. preflight
def test_preflight_camara_dron(cfg_path_solo, cfg_path, monkeypatch, capsys):
    import socket
    from test_solo_explorador import _preflight

    def no_conectar(*a, **k):
        raise AssertionError("preflight --remoto abrió una conexión")
    monkeypatch.setattr(socket, "create_connection", no_conectar)
    monkeypatch.setattr(socket.socket, "connect", no_conectar)
    pc = "10.0.0.9"
    pf = _preflight()
    _cam(cfg_path_solo, 2)
    pf.main(["--config", cfg_path_solo, "--remoto", pc])
    out = capsys.readouterr().out
    assert "[FALLA] camara.dron = 2 pero el dron solo es el 1" in out
    _cam(cfg_path_solo, 1)
    pf.main(["--config", cfg_path_solo, "--remoto", pc])
    assert "[OK   ] camara.dron = 1 = dron solo" in capsys.readouterr().out
    _cam(cfg_path, 2)
    pf.main(["--config", cfg_path, "--remoto", pc])
    assert "[INFO ] camara.dron = 2: coloca ese dron con MENOR x" in capsys.readouterr().out


# =============================================================================== demo mock: homes según camara.dron
@pytest.mark.parametrize("modo, dron, explorador", [("dual", 1, 1), ("dual", 2, 2), ("solo_explorador", 1, 1)])
def test_demo_mock_saca_atras_al_dron_de_la_camara(modo, dron, explorador):
    """A18: en la demo dual el explorador (el de menor x en ASSIGN_ROLES) es el dron de camara.dron, como hay que
    colocarlo en el laboratorio; antes los homes eran fijos y siempre salía el dron 2."""
    from app.mock_sim import HOMES_MOCK, MockMission, homes_mock
    cfg = {"mision": {"modo": modo, "dron_solo": 1}, "camara": {"dron": dron},
           "pista": {"vertices": [[-1.2, 1.2], [1.2, 1.2], [-1.2, -1.2], [1.2, -1.2]]}}
    if modo == "dual":
        del cfg["mision"]["dron_solo"]
    homes = homes_mock(cfg)
    assert homes[dron - 1] == (-0.9, 0.4) or modo != "dual"
    if modo != "dual" or dron == 2:
        assert homes == HOMES_MOCK                       # como antes
    m = MockMission(cfg, homes=homes)
    for _ in range(int(3 / 0.02)):
        m.step(0.02)
        if m.state == P.ASSIGN_ROLES:
            break
    assert m.state == P.ASSIGN_ROLES and m.E == explorador
