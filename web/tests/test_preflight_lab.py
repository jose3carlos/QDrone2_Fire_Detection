"""preflight_lab.py: .motive (rigid bodies, auxiliar en dual vs solo, ilegible) y build de tierra vs config / lib
(sin .rt-win64, más viejo, más nuevo, sello con otro sha). Todo en carpetas temporales; no toca la PC del lab."""
import glob
import importlib.util
import json
import os
import re
import time
import zipfile

import pytest

WEB = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT = os.path.dirname(WEB)
MOTIVE_REAL = os.path.join(PROJECT, "config", "motive", "RigidBody_dron1_aux2.motive")
O = {"rigid_body_dron1": 1, "rigid_body_dron2": 2}


@pytest.fixture(scope="module")
def pf():
    spec = importlib.util.spec_from_file_location("preflight_lab_t", os.path.join(WEB, "preflight_lab.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def correr(fn, *a, **k):
    out = []
    fn(*a, rep=lambda n, t: out.append((n, t)), **k)
    return out


def niveles(out, nivel):
    return [t for n, t in out if n == nivel]


# ============================================================================================ .motive
def _bloques(xml):
    return re.findall(r"\s*<rigid_body .*?</rigid_body>", xml, flags=re.S)


def _motive(tmp_path, nombre, transformar=None):
    with open(MOTIVE_REAL, encoding="utf-8", newline="") as f:
        xml = f.read()
    if transformar:
        xml = transformar(xml)
    p = tmp_path / nombre
    p.write_bytes(xml.encode("utf-8") if isinstance(xml, str) else xml)
    return str(p)


def _sin_aux(xml):
    return xml.replace(_bloques(xml)[1], "")


def _dron2_real(xml, misma_geometria=False):
    """Cambia el auxiliar por un rigid body de dron (copia del 1 con UserData 2; con otra geometría si no se pide)."""
    b1, aux = _bloques(xml)
    b2 = b1.replace("<value>RigidBody</value>", "<value>dron2</value>").replace(
        "<name>Color</name>", "<name>UserData</name>\n<value>2</value>\n</property>\n<property>\n<name>Color</name>", 1)
    if not misma_geometria:
        b2 = b2.replace("0.237732,0.0342415,0.098397", "0.197732,0.0342415,0.148397")
    return xml.replace(aux, b2)


def test_motive_real_solo_y_dual(pf, tmp_path):
    p = _motive(tmp_path, "real.motive")
    rbs = pf.leer_motive(p)
    assert [(r["id"], r["id_implicito"], len(r["marcadores"])) for r in rbs] == [(1, True, 4), (2, False, 3)]
    assert rbs[0]["auxiliar"] is None and 0.40 < rbs[0]["envergadura"] < 0.45
    assert "triángulo" in rbs[1]["auxiliar"]
    # solo explorador con el dron 1: el auxiliar como ID 2 es correcto
    out = correr(pf.revisar_motive, p, O, True, 1)
    assert not niveles(out, "FALLA") and not niveles(out, "AVISO"), out
    assert any("rigid body 1 del dron 1" in t for t in niveles(out, "OK"))
    assert any("auxiliar" in t and "correcto en solo explorador" in t for t in niveles(out, "INFO"))
    # dual: el dron 2 tendría el auxiliar -> FALLA
    out = correr(pf.revisar_motive, p, O, False, 0)
    f = niveles(out, "FALLA")
    assert len(f) == 1 and "DUAL" in f[0] and "dron 2" in f[0] and "AUXILIAR" in f[0] and "WAIT_TRACKING" in f[0]
    # solo explorador con el dron 2 (el del auxiliar) -> FALLA
    out = correr(pf.revisar_motive, p, O, True, 2)
    assert any("dron solo 2" in t and "AUXILIAR" in t for t in niveles(out, "FALLA")), out
    # copia del repo: las FALLAs bajan a AVISO
    out = correr(pf.revisar_motive, p, O, False, 0, copia=True)
    assert not niveles(out, "FALLA") and any("AUXILIAR" in t for t in niveles(out, "AVISO"))


def test_motive_sin_id2(pf, tmp_path):
    p = _motive(tmp_path, "solo1.motive", _sin_aux)
    for solo in (True, False):
        out = correr(pf.revisar_motive, p, O, solo, 1 if solo else 0)
        f = niveles(out, "FALLA")
        assert len(f) == 1 and "rigid body 2 del dron 2" in f[0] and "NO está" in f[0], out
    # otro ID configurado para el dron 1
    out = correr(pf.revisar_motive, p, {"rigid_body_dron1": 3, "rigid_body_dron2": 1}, True, 2)
    assert any("rigid body 3 del dron 1" in t for t in niveles(out, "FALLA"))


def test_motive_dron2_real_y_auxiliar_por_geometria(pf, tmp_path):
    p = _motive(tmp_path, "dos.motive", _dron2_real)
    out = correr(pf.revisar_motive, p, O, False, 0)
    assert not niveles(out, "FALLA") and not niveles(out, "AVISO"), out
    assert len([t for t in niveles(out, "OK") if "rigid body" in t]) == 2
    # misma geometría que el dron 1 -> AVISO (Motive los confundiría)
    p = _motive(tmp_path, "iguales.motive", lambda x: _dron2_real(x, misma_geometria=True))
    out = correr(pf.revisar_motive, p, O, False, 0)
    assert any("misma geometría" in t for t in niveles(out, "AVISO")) and not niveles(out, "FALLA")
    # auxiliar renombrado: se reconoce por el triángulo
    p = _motive(tmp_path, "renombrado.motive", lambda x: x.replace("AUX_ID2_NO_USAR", "dron2"))
    out = correr(pf.revisar_motive, p, O, False, 0)
    assert any("AUXILIAR" in t and "triángulo" in t for t in niveles(out, "FALLA")), out


def test_motive_id_repetido_con_el_auxiliar(pf, tmp_path):
    """Dron 2 real (UserData 2) agregado SIN borrar el auxiliar (también UserData 2): en dual es FALLA aunque el real
    vaya primero en el archivo (QUARC puede tomar el auxiliar); en solo explorador con el dron 1 basta el AVISO."""
    def real_y_aux(xml):
        b1, aux = _bloques(xml)
        return xml.replace(aux, _bloques(_dron2_real(xml))[1] + aux)
    p = _motive(tmp_path, "real_y_aux.motive", real_y_aux)
    assert [r["id"] for r in pf.leer_motive(p)] == [1, 2, 2]
    out = correr(pf.revisar_motive, p, O, False, 0)
    f = niveles(out, "FALLA")
    assert len(f) == 1 and "rigid body 2 del dron 2" in f[0] and "AUXILIAR" in f[0] and "BORRA" in f[0], out
    assert not any("del dron 2" in t for t in niveles(out, "OK"))
    out = correr(pf.revisar_motive, p, O, True, 2)                   # solo explorador con el dron 2: igual
    assert any("BORRA" in t for t in niveles(out, "FALLA")), out
    out = correr(pf.revisar_motive, p, O, True, 1)                   # el dron 2 no vuela
    assert not niveles(out, "FALLA") and any("ID repetido [2]" in t for t in niveles(out, "AVISO")), out
    # la FALLA dual de siempre también pide borrar el auxiliar
    out = correr(pf.revisar_motive, _motive(tmp_path, "real.motive"), O, False, 0)
    assert "BORRA el auxiliar" in niveles(out, "FALLA")[0]


@pytest.mark.parametrize("contenido, motivo", [
    (b"\x00\x01\x02 no es xml", "no es XML"),
    (None, "no es XML"),                                   # XML cortado a la mitad
    (b'<?xml version="1.0"?><Profile><NodeAssets/></Profile>', "ningún <rigid_body>"),
    (b'<Profile><rigid_body><properties><property><name>UserData</name><value>dos</value></property>'
     b'</properties></rigid_body></Profile>', "UserData 'dos'"),
])
def test_motive_ilegible_es_aviso(pf, tmp_path, contenido, motivo):
    if contenido is None:
        contenido = open(MOTIVE_REAL, "rb").read()[:2000]
    p = tmp_path / "malo.motive"
    p.write_bytes(contenido)
    out = correr(pf.revisar_motive, str(p), O, False, 0)
    assert not niveles(out, "FALLA"), out
    a = niveles(out, "AVISO")
    assert len(a) == 1 and "no pude interpretar" in a[0] and motivo in a[0], out


def test_motive_en_main(pf, cfg_path, cfg_path_solo, tmp_path, capsys, monkeypatch):
    """main: el .motive del día (ruta absoluta) se interpreta; archivo_motive vacío es FALLA."""
    monkeypatch.setattr(pf, "procesos_abiertos", lambda: ["explorer.exe", "Motive.exe"])
    p = _motive(tmp_path, "real.motive")

    def poner(ruta, v):
        with open(ruta, encoding="utf-8") as f:
            c = json.load(f)
        c["optitrack"]["archivo_motive"] = v
        with open(ruta, "w", encoding="utf-8") as f:
            json.dump(c, f)
    poner(cfg_path, p)
    assert pf.main(["--config", cfg_path]) == 1
    out = capsys.readouterr().out
    assert f"[OK   ] optitrack.archivo_motive: {p} existe" in out
    assert "[FALLA] modo DUAL y el rigid body 2 del dron 2 es el AUXILIAR" in out
    assert "[AVISO] Motive está abierto (Motive.exe)" in out
    poner(cfg_path_solo, p)
    pf.main(["--config", cfg_path_solo])
    out = capsys.readouterr().out
    assert "correcto en solo explorador" in out
    assert not [ln for ln in out.splitlines() if ln.startswith("[FALLA]") and ("AUXILIAR" in ln or "motive" in ln)]
    poner(cfg_path_solo, "")
    pf.main(["--config", cfg_path_solo])
    assert "[FALLA] optitrack.archivo_motive vacío" in capsys.readouterr().out


def test_procesos_motive(pf):
    txt = '"System Idle Process","0","Services","0","8 K"\r\n"Motive.exe","1234","Console","1","300,000 K"\r\n'
    assert pf.nombres_tasklist(txt) == ["System Idle Process", "Motive.exe"]
    assert pf.motive_abierto(pf.nombres_tasklist(txt)) == ["Motive.exe"]
    assert pf.motive_abierto(["python.exe", "MATLAB.exe"]) == []


# ============================================================================================ dual vs estacion.json
def _par_configs(tmp_path):
    """Copias de config/estacion.json y config/estacion_dual.json del repo en tmp_path/config."""
    d = tmp_path / "config"
    d.mkdir(parents=True)
    for n in ("estacion.json", "estacion_dual.json"):
        (d / n).write_bytes(open(os.path.join(PROJECT, "config", n), "rb").read())
    return str(d / "estacion.json"), str(d / "estacion_dual.json")


def _editar(ruta, fn):
    with open(ruta, encoding="utf-8-sig") as f:
        c = json.load(f)
    fn(c)
    with open(ruta, "w", encoding="utf-8") as f:
        json.dump(c, f)


def test_dual_avisa_lo_medido_solo_en_estacion_json(pf, tmp_path, capsys, monkeypatch):
    """Lo que el checklist guarda en estacion.json (etapas 3-5) y no se copió a estacion_dual.json da AVISO en el
    preflight dual; las claves propias del dual (modo, dron_solo, ip_dron2, .motive) no."""
    solo, dual = _par_configs(tmp_path)
    out = correr(pf.revisar_par_dual, dual)
    assert niveles(out, "OK") and not niveles(out, "AVISO"), out          # hoy difieren modo, dron_solo y limites_x
    _editar(dual, lambda c: (c["red"].update(ip_dron2="192.168.2.17"),
                             c["optitrack"].update(archivo_motive="C:/OptiTrack/hoy/dos_drones.motive")))
    out = correr(pf.revisar_par_dual, dual)
    assert niveles(out, "OK") and not niveles(out, "AVISO"), out
    # la checklist mide en estacion.json: bias del dron 2, offset e intrínsecos de la cámara, signos de la actitud
    _editar(solo, lambda c: (c["optitrack"].update(bias_z_dron2=-0.04),
                             c["camara"].update(offset_camara_m=[0.212, 0.0325, 0.019],
                                                intrinsecos={"640x480": {"fx": 1, "fy": 1, "cx": 1, "cy": 1}}),
                             c["vision"].update(actitud_signos=[1, -1])))
    out = correr(pf.revisar_par_dual, dual)
    a = niveles(out, "AVISO")
    assert len(a) == 1 and not niveles(out, "OK"), out
    for k in ("optitrack.bias_z_dron2", "camara.offset_camara_m", "camara.intrinsecos", "vision.actitud_signos"):
        assert k in a[0], a[0]
    assert "mision.modo" not in a[0] and "ip_dron2" not in a[0] and "Etapa 8-0" in a[0]
    # con la propia estacion.json (o sin estacion.json al lado) no compara
    assert correr(pf.revisar_par_dual, solo) == []
    os.remove(solo)
    assert correr(pf.revisar_par_dual, dual) == []
    # main en dual lo corre; en solo explorador no
    _par_configs(tmp_path / "b")
    solo_b, dual_b = str(tmp_path / "b" / "config" / "estacion.json"), str(tmp_path / "b" / "config" / "estacion_dual.json")
    _editar(solo_b, lambda c: c["optitrack"].update(bias_z_dron2=-0.04))
    monkeypatch.setattr(pf, "SQ", str(tmp_path))                  # sin .rt-win64 aquí
    pf.main(["--config", dual_b])
    assert "[AVISO] estacion_dual.json difiere de estacion.json en optitrack.bias_z_dron2" in capsys.readouterr().out
    pf.main(["--config", solo_b])
    assert "difiere de estacion.json" not in capsys.readouterr().out


def test_dual_limites_x_solo_en_dual_no_avisa(pf, tmp_path):
    """mision.limites_x solo en estacion_dual.json (decisión del usuario, 26-nov: x [-1.35, 1.2] en dual; el solo
    explorador no cambia) es esperado: sin AVISO por esa clave, con otro valor ni si estacion.json también la trae.
    Otra clave de mision que difiera (limites_y) sí avisa, y la exclusión no tapa claves con el mismo prefijo."""
    solo, dual = _par_configs(tmp_path)
    with open(dual, encoding="utf-8-sig") as f:
        assert json.load(f)["mision"]["limites_x"] == [-1.35, 1.2]       # la config del repo la trae
    with open(solo, encoding="utf-8-sig") as f:
        assert "limites_x" not in json.load(f)["mision"]                   # y estacion.json no
    out = correr(pf.revisar_par_dual, dual)
    assert not niveles(out, "AVISO") and "mision.limites_x" in niveles(out, "OK")[0], out
    _editar(dual, lambda c: c["mision"].update(limites_x=[-1.2, 1.0]))
    assert not niveles(correr(pf.revisar_par_dual, dual), "AVISO")
    _editar(solo, lambda c: c["mision"].update(limites_x=[-1.35, 1.35]))
    assert not niveles(correr(pf.revisar_par_dual, dual), "AVISO")
    _editar(dual, lambda c: c["mision"].update(limites_y=[-1.3, 1.1], limites_x_nota="otra"))
    a = niveles(correr(pf.revisar_par_dual, dual), "AVISO")
    assert len(a) == 1, a
    claves = a[0].split(" difiere de estacion.json en ", 1)[1].split(": ", 1)[0].split(", ")
    assert sorted(claves) == ["mision.limites_x_nota", "mision.limites_y"], a[0]


def test_dual_sin_limites_x_avisa(pf, tmp_path, capsys, monkeypatch):
    """Etapa 8-0 copia estacion.json (sin limites_x) encima de estacion_dual.json: revisar_par_dual no compara esa
    clave, así que main avisa aparte si en DUAL falta (o es inválida); con la del repo, o en solo explorador, no."""
    monkeypatch.setattr(pf, "SQ", str(tmp_path))                  # sin .rt-win64 aquí
    solo, dual = _par_configs(tmp_path)
    aviso = "[AVISO] modo DUAL sin mision.limites_x"
    pf.main(["--config", dual])
    assert aviso not in capsys.readouterr().out
    pf.main(["--config", solo])
    assert aviso not in capsys.readouterr().out
    for v in (None, [1.2, -1.35]):                                # olvidada (copiada encima) o inválida
        def poner(c, v=v):
            c["mision"].pop("limites_x")
            if v is not None:
                c["mision"]["limites_x"] = v
        _editar(dual, poner)
        pf.main(["--config", dual])
        out = capsys.readouterr().out
        ln = [x for x in out.splitlines() if x.startswith(aviso)]
        assert len(ln) == 1 and "-1.35..1.35" in ln[0] and "[-1.35, 1.2]" in ln[0], out
        assert "[OK   ] estacion_dual.json = estacion.json salvo" in out or v is not None
        _editar(dual, lambda c: c["mision"].update(limites_x=[-1.35, 1.2]))


# ============================================================================================ build
def test_modelo_por_modo(pf):
    assert pf.modelo_tierra("PID", True) == "QD2_MissionCtrl_Dual_PID"
    assert pf.modelo_tierra("PID", False) == "QD2_MissionCtrl_Dual_PID_2D"
    assert pf.modelo_tierra("mpc", True) == "QD2_MissionCtrl_Dual_MPC"
    assert pf.config_del_modelo("QD2_MissionCtrl_Dual_PID_2D", "R").endswith(os.path.join("config", "estacion_dual.json"))
    assert pf.config_del_modelo("QD2_MissionCtrl_Dual_PID", "R").endswith(os.path.join("config", "estacion.json"))


class Arbol:
    """raiz/config/estacion.json, raiz/simulink_qdrone2/{qd2_cargar_config.m, lib/*.m}, versiones/R2022a."""

    def __init__(self, tmp_path, pf):
        self.pf = pf
        self.raiz = str(tmp_path)
        self.sq = os.path.join(self.raiz, "simulink_qdrone2")
        self.carpeta = os.path.join(self.sq, "versiones", "R2022a")
        os.makedirs(os.path.join(self.sq, "lib"))
        os.makedirs(self.carpeta)
        os.makedirs(os.path.join(self.raiz, "config"))
        self.cfg = os.path.join(self.raiz, "config", "estacion.json")
        self.t0 = time.time() - 10000
        self.escribir(self.cfg, '{"mision": {"modo": "solo_explorador"}}')
        self.escribir(os.path.join(self.sq, "qd2_cargar_config.m"), "function qd2_cargar_config\n")
        for n in ("qd2_a.m", "qd2_b.m"):
            self.escribir(os.path.join(self.sq, "lib", n), f"% {n}\n")
        self.rt = os.path.join(self.carpeta, "QD2_MissionCtrl_Dual_PID.rt-win64")
        self.sello = os.path.join(self.carpeta, "QD2_MissionCtrl_Dual_PID.config_usada.jsonl")

    def escribir(self, p, txt, t=None):
        with open(p, "w", encoding="utf-8", newline="") as f:
            f.write(txt)
        t = self.t0 if t is None else t
        os.utime(p, (t, t))

    def build(self, t):
        self.escribir(self.rt, "binario", t)

    def linea_sello(self, t, sha_cfg=None, archivos=None, modelo="QD2_MissionCtrl_Dual_PID", texto=None):
        pf = self.pf
        arch = archivos if archivos is not None else [
            {"ruta": r, "sha256": pf.sha256(p)} for r, p in pf.fuentes_build(self.sq).items()]
        e = {"formato": 1, "t_posix": t, "fecha": pf._fecha(t), "modelo": modelo, "matlab": "2022a",
             "config": {"ruta": self.cfg, "sha256": sha_cfg or pf.sha256(self.cfg)}, "modo": "solo_explorador",
             "dron_solo": 1, "rb_ids": [1, 2], "archivo_motive": "", "archivos": arch}
        if texto is not None:
            e["config"]["texto"] = texto
        with open(self.sello, "a", encoding="utf-8") as f:
            f.write(json.dumps(e) + "\n")

    def revisar(self, solo=True):
        return correr(self.pf.revisar_build, self.cfg, "PID", solo, self.carpeta, self.sq, raiz=self.raiz)


@pytest.fixture
def arbol(tmp_path, pf):
    return Arbol(tmp_path, pf)


def test_build_ausente(arbol):
    out = arbol.revisar()
    f = niveles(out, "FALLA")
    assert len(f) == 1 and "no hay versiones/R2022a/QD2_MissionCtrl_Dual_PID.rt-win64" in f[0]
    out = arbol.revisar(solo=False)          # dual -> _PID_2D, que lee estacion_dual.json (no existe aquí)
    assert any("QD2_MissionCtrl_Dual_PID_2D.rt-win64" in t for t in niveles(out, "FALLA"))
    assert any("estacion_dual.json" in t and "configs distintas" in t for t in niveles(out, "AVISO"))


def test_build_por_fechas(arbol):
    arbol.build(arbol.t0 + 100)                       # más nuevo que todo
    out = arbol.revisar()
    assert not niveles(out, "FALLA") and any("sin sello" in t for t in niveles(out, "INFO"))
    assert any("por fechas" in t for t in niveles(out, "OK"))
    # config editada después del build
    arbol.escribir(arbol.cfg, '{"mision": {"modo": "solo_explorador"}, "x": 1}', arbol.t0 + 200)
    f = niveles(arbol.revisar(), "FALLA")
    assert len(f) == 1 and "recompila QD2_MissionCtrl_Dual_PID" in f[0] and "estacion.json" in f[0]
    # una .m de lib después del build
    arbol.escribir(arbol.cfg, '{"mision": {"modo": "solo_explorador"}}')
    arbol.escribir(os.path.join(arbol.sq, "lib", "qd2_b.m"), "% cambio\n", arbol.t0 + 200)
    f = niveles(arbol.revisar(), "FALLA")
    assert len(f) == 1 and "lib/qd2_b.m" in f[0] and "estacion.json" not in f[0]
    # qd2_cargar_config.m
    arbol.escribir(os.path.join(arbol.sq, "lib", "qd2_b.m"), "% cambio\n")
    arbol.escribir(os.path.join(arbol.sq, "qd2_cargar_config.m"), "x\n", arbol.t0 + 200)
    assert any("qd2_cargar_config.m" in t for t in niveles(arbol.revisar(), "FALLA"))
    # .slx más nuevo: solo AVISO
    arbol.escribir(os.path.join(arbol.sq, "qd2_cargar_config.m"), "x\n")
    arbol.escribir(os.path.join(arbol.carpeta, "QD2_MissionCtrl_Dual_PID.slx"), "zip", arbol.t0 + 200)
    out = arbol.revisar()
    assert not niveles(out, "FALLA") and any(".slx" in t and "fecha posterior" in t for t in niveles(out, "AVISO"))


def test_build_sello_mismo_contenido(arbol):
    arbol.linea_sello(arbol.t0 + 80)
    arbol.build(arbol.t0 + 100)
    # git pull / copia: todo con fecha posterior al build, mismo contenido -> OK por el sello
    for p in [arbol.cfg] + list(arbol.pf.fuentes_build(arbol.sq).values()):
        os.utime(p, (arbol.t0 + 500, arbol.t0 + 500))
    out = arbol.revisar()
    assert not niveles(out, "FALLA"), out
    assert any("sello del" in t and "mismo contenido" in t for t in niveles(out, "OK"))


def test_build_sello_otro_sha(arbol):
    arbol.linea_sello(arbol.t0 + 80, sha_cfg="0" * 64)
    arbol.build(arbol.t0 + 100)
    f = niveles(arbol.revisar(), "FALLA")        # aunque por fechas el build sea más nuevo
    assert len(f) == 1 and "la config" in f[0] and "sha256 000000000000" in f[0], f


def test_build_sello_lib_cambiada_y_nueva(arbol):
    arbol.linea_sello(arbol.t0 + 80)
    arbol.build(arbol.t0 + 100)
    arbol.escribir(os.path.join(arbol.sq, "lib", "qd2_a.m"), "% otra cosa\n")          # misma fecha vieja
    arbol.escribir(os.path.join(arbol.sq, "lib", "qd2_c.m"), "% nueva\n")
    f = niveles(arbol.revisar(), "FALLA")
    assert len(f) == 1 and "lib/qd2_a.m" in f[0] and "lib/qd2_c.m (nuevo)" in f[0] and "qd2_b" not in f[0], f


def test_build_sello_toma_la_linea_del_build(arbol):
    """Un Connect / Update Diagram DESPUÉS del build con otra config no cuenta; el build usó la de antes."""
    arbol.linea_sello(arbol.t0 + 80)
    arbol.build(arbol.t0 + 100)
    arbol.linea_sello(arbol.t0 + 300, sha_cfg="1" * 64)
    arbol.linea_sello(arbol.t0 + 90, modelo="QD2_MissionCtrl_Dual_PID_2D", sha_cfg="2" * 64)   # otro modelo
    assert not niveles(arbol.revisar(), "FALLA")
    # y al revés: el build usó otra config y después se volvió a cargar la buena (sin recompilar) -> FALLA
    os.remove(arbol.sello)
    arbol.linea_sello(arbol.t0 + 80, sha_cfg="3" * 64)
    arbol.linea_sello(arbol.t0 + 300)
    assert any("la config" in t for t in niveles(arbol.revisar(), "FALLA"))


def test_build_sello_viejo_o_posterior_usa_fechas(arbol):
    arbol.linea_sello(arbol.t0 - 5000, sha_cfg="0" * 64)       # 85 min antes del build: ese build no dejó sello
    arbol.build(arbol.t0 + 100)
    out = arbol.revisar()
    assert not niveles(out, "FALLA") and any("no dejó sello" in t for t in niveles(out, "INFO")), out
    os.remove(arbol.sello)
    arbol.linea_sello(arbol.t0 + 300, sha_cfg="0" * 64)        # solo líneas posteriores al build
    with open(arbol.sello, "a", encoding="utf-8") as f:
        f.write("{esto no es json\n")
    out = arbol.revisar()
    assert not niveles(out, "FALLA")
    assert any("posteriores al build" in t for t in niveles(out, "INFO"))
    assert any("ilegibles" in t for t in niveles(out, "AVISO"))


def test_build_sello_sin_archivos_usa_fechas_para_lib(arbol):
    arbol.linea_sello(arbol.t0 + 80, archivos=[])
    arbol.build(arbol.t0 + 100)
    assert not niveles(arbol.revisar(), "FALLA")
    arbol.escribir(os.path.join(arbol.sq, "lib", "qd2_a.m"), "% x\n", arbol.t0 + 200)
    assert any("lib/qd2_a.m" in t for t in niveles(arbol.revisar(), "FALLA"))


def test_build_sello_solo_claves_que_lee_el_modelo(arbol):
    """Con el texto de la config en el sello: cambiar claves de la estación (red.ip_*, pista.tubos, camara, web,
    vision.* salvo radio_cluster_m) no pide recompilar (INFO); las que lee el modelo sí, y la FALLA dice cuál."""
    base = {"version_control": "PID",
            "red": {"ip_dron1": "", "interfaz_preferida": "", "puerto_dron1": 18001, "puerto_puente": 18700},
            "optitrack": {"rigid_body_dron1": 1, "rigid_body_dron2": 2},
            "mision": {"modo": "solo_explorador", "tol_xy": 0.12},
            "pista": {"vertices": [[0, 0], [1, 0]], "tubos": []},
            "vision": {"radio_cluster_m": 0.25, "confianza_min": 0.9}}
    texto = json.dumps(base)
    arbol.escribir(arbol.cfg, texto)
    arbol.linea_sello(arbol.t0 + 80, texto=texto)
    arbol.build(arbol.t0 + 100)
    c = json.loads(texto)
    c["red"]["ip_dron1"] = "10.0.0.5"
    c["red"]["interfaz_preferida"] = "Wi-Fi 2"
    c["pista"]["tubos"] = [[[0, 0], [1, 1]]]
    c["camara"] = {"offset_camara_m": [0.2, 0.0, -0.05]}
    c["vision"]["confianza_min"] = 0.8
    c["web"] = {"fuente": "simulink"}
    arbol.escribir(arbol.cfg, json.dumps(c, indent=2), arbol.t0 + 200)
    out = arbol.revisar()
    assert not niveles(out, "FALLA"), out
    i = [t for t in niveles(out, "INFO") if "no lee" in t]
    assert len(i) == 1 and "no hace falta recompilar" in i[0], out
    assert all(k in i[0] for k in ("red.ip_dron1", "red.interfaz_preferida", "pista.tubos", "camara", "web",
                                   "vision.confianza_min")), i
    # solo formato (mismo JSON con otro espaciado)
    arbol.escribir(arbol.cfg, json.dumps(base, indent=4), arbol.t0 + 200)
    assert any("(formato)" in t for t in niveles(arbol.revisar(), "INFO"))
    # claves que lee el modelo: FALLA con su nombre (y no las de la estación)
    for ruta, valor in (("mision.tol_xy", 0.13), ("pista.vertices", [[0, 0], [2, 0]]), ("vision.radio_cluster_m", 0.3),
                        ("red.puerto_dron1", 18011), ("version_control", "MPC"), ("optitrack.archivo_motive", "x.motive"),
                        ("mision.limites_y", [-1.0, 1.0])):
        c = json.loads(texto)
        c["red"]["ip_dron1"] = "10.0.0.5"
        *pre, k = ruta.split(".")
        (c[pre[0]] if pre else c)[k] = valor
        arbol.escribir(arbol.cfg, json.dumps(c), arbol.t0 + 200)
        f = niveles(arbol.revisar(), "FALLA")
        assert len(f) == 1 and f"({ruta})" in f[0] and "recompila" in f[0], (ruta, f)
    c = json.loads(texto)
    del c["pista"]
    arbol.escribir(arbol.cfg, json.dumps(c), arbol.t0 + 200)
    assert any("(pista)" in t for t in niveles(arbol.revisar(), "FALLA"))
    # config ilegible ahora: no se sabe qué clave -> FALLA como antes
    arbol.escribir(arbol.cfg, "{roto", arbol.t0 + 200)
    assert any("no sé qué clave" in t for t in niveles(arbol.revisar(), "FALLA"))


def test_build_lib_que_tierra_no_usa(arbol):
    """lib/*.m de LIB_NO_TIERRA (planta, a bordo): cambiarlas o borrarlas no pide recompilar el modelo de tierra."""
    planta = os.path.join(arbol.sq, "lib", "qd2_quad_plant.m")
    arbol.escribir(planta, "% planta\n")
    assert "lib/qd2_quad_plant.m" not in arbol.pf.fuentes_build(arbol.sq)
    arch = [{"ruta": r, "sha256": arbol.pf.sha256(p)} for r, p in arbol.pf.fuentes_build(arbol.sq).items()]
    arbol.linea_sello(arbol.t0 + 80, archivos=arch + [{"ruta": "lib/qd2_quad_plant.m", "sha256": "0" * 64}])
    arbol.build(arbol.t0 + 100)
    arbol.escribir(planta, "% otra planta\n", arbol.t0 + 200)
    out = arbol.revisar()
    assert not niveles(out, "FALLA") and not niveles(out, "AVISO"), out
    os.remove(planta)
    out = arbol.revisar()
    assert not niveles(out, "FALLA") and not niveles(out, "AVISO"), out
    arbol.escribir(os.path.join(arbol.sq, "lib", "qd2_a.m"), "% cambio\n")      # una que sí usa
    assert any("lib/qd2_a.m" in t for t in niveles(arbol.revisar(), "FALLA"))


def test_lib_no_tierra_no_la_alcanzan_los_modelos_de_tierra(pf):
    """Las .m de LIB_NO_TIERRA existen y ni QD2_Mission_Lib.slx / los .slx de tierra ni qd2_cargar_config.m las
    llaman (directa o indirectamente, buscando el nombre de cada función en el código de las .m que sí se usan)."""
    sq = os.path.join(PROJECT, "simulink_qdrone2")
    fuentes = {}
    for p in glob.glob(os.path.join(sq, "lib", "*.m")):
        with open(p, encoding="utf-8", errors="replace") as f:
            fuentes[os.path.basename(p)[:-2]] = f.read()
    assert all(os.path.exists(os.path.join(sq, r)) for r in pf.LIB_NO_TIERRA)
    semillas = []
    for n in ("QD2_Mission_Lib", "QD2_MissionCtrl_Dual_PID", "QD2_MissionCtrl_Dual_PID_2D", "QD2_MissionCtrl_Dual_MPC"):
        with zipfile.ZipFile(os.path.join(sq, "versiones", "R2022a", n + ".slx")) as z:
            semillas.append("\n".join(z.read(q).decode("utf-8", "replace") for q in z.namelist()))
    with open(os.path.join(sq, "qd2_cargar_config.m"), encoding="utf-8") as f:
        semillas.append(f.read())
    usa = lambda txt: {n for n in fuentes if re.search(r"\b%s\b" % n, txt)}  # noqa: E731
    alcanzadas, pend = set(), set().union(*(usa(t) for t in semillas))
    while pend:
        n = pend.pop()
        alcanzadas.add(n)
        pend |= usa(fuentes[n]) - alcanzadas
    assert "qd2_mission_step" in alcanzadas and "qd2_params_from_config" in alcanzadas
    assert not {"lib/%s.m" % n for n in alcanzadas} & pf.LIB_NO_TIERRA, alcanzadas


def test_lib_de_un_modelo_solo_la_llama_ese_modelo(pf):
    """Cada lib/*.m de LIB_DE_UN_MODELO existe, la llama el .slx de su modelo (versiones/R2022a) y no la llaman los
    otros .slx de tierra, QD2_Mission_Lib.slx, qd2_cargar_config.m ni las demás lib/*.m."""
    sq = os.path.join(PROJECT, "simulink_qdrone2")
    assert pf.LIB_DE_UN_MODELO
    textos = {}
    for n in ("QD2_Mission_Lib", "QD2_MissionCtrl_Dual_PID", "QD2_MissionCtrl_Dual_PID_2D", "QD2_MissionCtrl_Dual_MPC"):
        with zipfile.ZipFile(os.path.join(sq, "versiones", "R2022a", n + ".slx")) as z:
            textos[n] = "\n".join(z.read(q).decode("utf-8", "replace") for q in z.namelist())
    with open(os.path.join(sq, "qd2_cargar_config.m"), encoding="utf-8") as f:
        textos["qd2_cargar_config"] = f.read()
    for r, mdl in pf.LIB_DE_UN_MODELO.items():
        assert os.path.exists(os.path.join(sq, r)), r
        fn = os.path.basename(r)[:-2]
        otras = {os.path.basename(p)[:-2]: p for p in glob.glob(os.path.join(sq, "lib", "*.m"))}
        otras.pop(fn)
        for n, p in otras.items():
            with open(p, encoding="utf-8", errors="replace") as f:
                textos["lib/" + n] = f.read()
        llaman = {n for n, t in textos.items() if re.search(r"\b%s\b" % fn, t)}
        assert llaman == {mdl}, (r, llaman)


def test_build_lib_de_un_modelo_no_cuenta_para_los_otros(arbol):
    """lib/qd2_coordenadas_objetivos.m (solo la usa QD2_MissionCtrl_Dual_PID): el dual (_PID_2D) compilado sin ella no
    pide recompilar al aparecer (git checkout de esta rama) ni avisa si el sello la tiene y ya no existe; el solo
    explorador sí la cuenta (nueva, cambiada o faltante)."""
    pf = arbol.pf
    coords = os.path.join(arbol.sq, "lib", "qd2_coordenadas_objetivos.m")
    assert "lib/qd2_coordenadas_objetivos.m" in pf.LIB_DE_UN_MODELO
    # dual: config estacion_dual.json (la que lee su InitFcn) con el mismo contenido
    dual = "QD2_MissionCtrl_Dual_PID_2D"
    cfg_dual = os.path.join(arbol.raiz, "config", "estacion_dual.json")
    with open(arbol.cfg, encoding="utf-8") as f:
        arbol.escribir(cfg_dual, f.read())
    arch_sin = [{"ruta": r, "sha256": pf.sha256(p)} for r, p in pf.fuentes_build(arbol.sq).items()]
    arbol.sello = os.path.join(arbol.carpeta, dual + ".config_usada.jsonl")
    arbol.linea_sello(arbol.t0 + 80, archivos=arch_sin, modelo=dual)
    arbol.escribir(os.path.join(arbol.carpeta, dual + ".rt-win64"), "binario", arbol.t0 + 100)
    revisar_dual = lambda: correr(pf.revisar_build, cfg_dual, "PID", False, arbol.carpeta, arbol.sq,  # noqa: E731
                                  raiz=arbol.raiz)
    assert not niveles(revisar_dual(), "FALLA")
    arbol.escribir(coords, "% coords\n", arbol.t0 + 200)
    assert "lib/qd2_coordenadas_objetivos.m" not in pf.fuentes_build(arbol.sq, dual)
    assert "lib/qd2_coordenadas_objetivos.m" in pf.fuentes_build(arbol.sq, "QD2_MissionCtrl_Dual_PID")
    out = revisar_dual()
    assert not niveles(out, "FALLA") and not niveles(out, "AVISO"), out
    # sello del dual CON la lib (build en esta rama) y la lib borrada (vuelta a Quique): sin AVISO
    arbol.linea_sello(arbol.t0 + 300, archivos=arch_sin + [{"ruta": "lib/qd2_coordenadas_objetivos.m",
                                                            "sha256": pf.sha256(coords)}], modelo=dual)
    arbol.escribir(os.path.join(arbol.carpeta, dual + ".rt-win64"), "binario", arbol.t0 + 320)
    os.remove(coords)
    out = revisar_dual()
    assert not niveles(out, "FALLA") and not niveles(out, "AVISO"), out
    # solo explorador: compilado sin ella -> al aparecer, FALLA (nuevo); con ella y borrada -> AVISO
    arbol.sello = os.path.join(arbol.carpeta, "QD2_MissionCtrl_Dual_PID.config_usada.jsonl")
    arbol.linea_sello(arbol.t0 + 80, archivos=arch_sin)
    arbol.build(arbol.t0 + 100)
    assert not niveles(arbol.revisar(), "FALLA")
    arbol.escribir(coords, "% coords\n", arbol.t0 + 200)
    f = niveles(arbol.revisar(), "FALLA")
    assert len(f) == 1 and "lib/qd2_coordenadas_objetivos.m (nuevo)" in f[0], f
    arbol.linea_sello(arbol.t0 + 300, archivos=arch_sin + [{"ruta": "lib/qd2_coordenadas_objetivos.m",
                                                            "sha256": pf.sha256(coords)}])
    arbol.build(arbol.t0 + 320)
    assert not niveles(arbol.revisar(), "FALLA")
    arbol.escribir(coords, "% otra\n", arbol.t0 + 400)
    assert any("lib/qd2_coordenadas_objetivos.m" in t for t in niveles(arbol.revisar(), "FALLA"))
    os.remove(coords)
    a = niveles(arbol.revisar(), "AVISO")
    assert any("lib/qd2_coordenadas_objetivos.m" in t and "ya no existen" in t for t in a), a


def test_build_en_main_con_remoto_no_falla(pf, cfg_path_solo, capsys, monkeypatch, tmp_path):
    monkeypatch.setattr(pf, "SQ", str(tmp_path))                  # sin .rt-win64 aquí
    pf.main(["--config", cfg_path_solo, "--remoto", "10.0.0.9"])
    out = capsys.readouterr().out
    assert "[INFO ] build de tierra (QD2_MissionCtrl_Dual_PID.rt-win64): está en la PC 10.0.0.9" in out
    assert "rt-win64" not in "".join(ln for ln in out.splitlines() if ln.startswith("[FALLA]"))
