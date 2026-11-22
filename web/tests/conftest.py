"""Configuración común de pytest: importa ``app`` desde web/ y da una copia temporal de estacion.json."""
import os
import shutil
import sys

import pytest

WEB = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT = os.path.dirname(WEB)
sys.path.insert(0, WEB)


@pytest.fixture
def cfg_path(tmp_path):
    """Copia de config/estacion.json: las pruebas nunca escriben el archivo real."""
    dst = tmp_path / "estacion.json"
    shutil.copy(os.path.join(PROJECT, "config", "estacion.json"), dst)
    return str(dst)
