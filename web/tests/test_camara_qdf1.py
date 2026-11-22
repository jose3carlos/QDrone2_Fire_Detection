"""Video QDF1: el streamer del dron (modo sintético) y el cliente de la estación hablan el mismo protocolo."""
import os
import socket
import sys
import threading

import numpy as np

from app.cameras import HEADER, QDF1Client

DRONE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "drone")
sys.path.insert(0, DRONE)
import qdrone2_camera_streamer as streamer  # noqa: E402


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_streamer_sintetico_a_cliente():
    port = free_port()
    th = threading.Thread(target=streamer.main, args=(["--camara", "sintetica", "--puerto", str(port), "--host", "127.0.0.1",
                                                       "--max-frames", "6", "--fps", "60", "--fps-envio", "60"],), daemon=True)
    th.start()
    cli = None
    for _ in range(50):
        try:
            cli = QDF1Client("127.0.0.1", port)
            break
        except OSError:
            threading.Event().wait(0.1)
    assert cli is not None
    frames = [cli.read() for _ in range(5)]
    cli.close()
    th.join(timeout=5)
    for f in frames:
        assert f.shape == (480, 640, 3) and f.dtype == np.uint8
    # el "fuego" sintético (rojo) aparece donde el streamer lo dibujó
    b, g, r = frames[-1][160, 320]
    assert r > 200 and b < 80


def test_rotacion_y_formato_crudo():
    img = np.zeros((4, 6, 3), np.uint8)
    img[0, 0] = 255
    rot = streamer.rotar(img, 90)
    assert rot.shape == (6, 4, 3) and rot[0, -1, 0] == 255   # 90° horario: la esquina sup-izq pasa a sup-der
    assert streamer.rotar(img, 180)[-1, -1, 0] == 255
    pkt = streamer.empaquetar(np.zeros((480, 640, 3), np.uint8), 80)
    magic, fmt, h, w, n = HEADER.unpack(pkt[:HEADER.size])
    assert magic == b"QDF1" and fmt == 1 and (h, w) == (480, 640) and n == len(pkt) - HEADER.size
