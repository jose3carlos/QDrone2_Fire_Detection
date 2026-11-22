"""Persistencia de config/estacion.json: no renombra, no borra, conserva cambios ajenos y el estilo."""
import json

import pytest

from app.config import Config, dumps, validar_ip


MUESTRA = """{
  "version_control": "PID",
  "red": {
    "ip_pc": "",
    "puerto_dron1": 18001
  },
  "optitrack": {
    "carpeta": "C:\\\\Users\\\\user\\\\Documents\\\\OptiTrack",
    "bias_z_dron1": -0.07
  },
  "mision": {
    "t_mark": 3.0,
    "auto_land": false,
    "limites_xy": [-1.5, 1.5]
  },
  "pista": {
    "nota": "Estimación inicial desde fotos",
    "vertices": [[-1.2, 1.2], [1.2, 1.2], [-1.2, -1.2], [1.2, -1.2]]
  },
  "camara": {
    "resolucion": [640, 480],
    "offset_camara_m": [0.2, 0.0, -0.05]
  }
}
"""


def test_guardar_sin_cambios_conserva_el_texto():
    """El serializador reproduce el estilo del estacion.json original (sin ruido en git)."""
    assert dumps(json.loads(MUESTRA)) + "\n" == MUESTRA


def test_parche_conserva_claves_y_cambios_ajenos(cfg_path):
    c = Config(cfg_path)
    antes = json.load(open(cfg_path, encoding="utf-8"))
    # otra herramienta agrega una clave mientras la estación está abierta
    otro = json.load(open(cfg_path, encoding="utf-8"))
    otro["simulink"] = {"agregado_por": "otra herramienta"}
    open(cfg_path, "w", encoding="utf-8").write(json.dumps(otro, indent=2, ensure_ascii=False))
    c.update({"red": {"ip_dron1": "192.168.2.16"}})
    despues = json.load(open(cfg_path, encoding="utf-8"))
    assert despues["red"]["ip_dron1"] == "192.168.2.16"
    assert despues["simulink"] == {"agregado_por": "otra herramienta"}
    for sec, val in antes.items():                       # ninguna clave original desaparece
        assert sec in despues
        if isinstance(val, dict):
            assert set(val) <= set(despues[sec])
    assert despues["red"]["ip_dron2"] == antes["red"]["ip_dron2"]
    assert c.data["red"]["ip_dron1"] == "192.168.2.16"


def test_defaults_web_solo_agregan(cfg_path):
    c = Config(cfg_path)
    c.ensure_web_defaults()
    d = json.load(open(cfg_path, encoding="utf-8"))
    assert d["web"]["host"] == "127.0.0.1" and d["web"]["fuente"] in ("mock", "simulink", "sim3d")
    c.update({"web": {"fuente": "simulink"}})
    c.ensure_web_defaults()                               # no pisa lo que ya existe
    assert json.load(open(cfg_path, encoding="utf-8"))["web"]["fuente"] == "simulink"


def test_formato_listas_en_linea(cfg_path):
    c = Config(cfg_path)
    c.update({"pista": {"vertices": [[-1.1, 1.2], [1.0, 1.25], [-1.0, -1.2], [1.2, -1.15]]}})
    txt = open(cfg_path, encoding="utf-8").read()
    assert '"vertices": [[-1.1, 1.2], [1.0, 1.25], [-1.0, -1.2], [1.2, -1.15]]' in txt
    assert '"limites_xy": [-1.5, 1.5]' in txt


def test_validar_ip():
    assert validar_ip(" 192.168.2.15 ") == "192.168.2.15"
    assert validar_ip("") == ""
    with pytest.raises(ValueError):
        validar_ip("192.168.2.300")
    with pytest.raises(ValueError):
        validar_ip("dron1")
