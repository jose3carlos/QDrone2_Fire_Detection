"""Cliente de prueba del PYTHON BRIDGE de QD2_Sim_Dual (tcpip://127.0.0.1:18700).

Hace de estación Python mínima (solo biblioteca estándar):
  - envía 32 doubles a 20 Hz con heartbeat creciente,
  - manda START cuando la misión está en ASSIGN_ROLES (estado 2),
  - manda LAND_ALL cuando los dos drones llevan 6 s en su home en RETURN_HOME,
  - imprime cada cambio de estado y termina en DONE (8) o por tiempo.

Uso:  python prueba_puente.py [puerto] [timeout_s]
Sirve para probar el puente sin la UI web (simulación con COMMAND SOURCE = 2).
"""
import socket
import struct
import sys
import time

N_TEL, N_CMD = 64, 32
NOMBRES = {0: "INIT", 1: "WAIT_TRACKING", 2: "ASSIGN_ROLES", 3: "TAKEOFF", 4: "HOVER_READY",
           5: "MISSION", 6: "RETURN_HOME", 7: "LANDING", 8: "DONE", 9: "ABORT", 10: "PAUSED"}


def main():
    puerto = int(sys.argv[1]) if len(sys.argv) > 1 else 18700
    t_max = float(sys.argv[2]) if len(sys.argv) > 2 else 400.0
    t0 = time.time()
    s = None
    while time.time() - t0 < 120:
        try:
            s = socket.create_connection(("127.0.0.1", puerto), timeout=2.0)
            break
        except OSError:
            time.sleep(0.5)
    if s is None:
        print("no se pudo conectar a", puerto)
        return 1
    print("conectado a :%d tras %.1f s" % (puerto, time.time() - t0), flush=True)
    s.settimeout(0.05)
    buf = b""
    hb = seq = cmd = 0
    n_rx = 0
    estado_prev = None
    tel = None
    t_casa = None
    start_ok = land_ok = False
    t_ultimo_envio = 0.0
    t1 = time.time()
    while time.time() - t1 < t_max:
        try:
            d = s.recv(65536)
            if not d:
                print("el servidor cerró la conexión")
                break
            buf += d
        except socket.timeout:
            pass
        while len(buf) >= 8 * N_TEL:
            tel = struct.unpack("<%dd" % N_TEL, buf[:8 * N_TEL])
            buf = buf[8 * N_TEL:]
            n_rx += 1
        if tel is not None:
            estado = int(tel[1])
            if estado != estado_prev:
                print("t_sim %7.2f  estado %2d %-13s  E=%d R=%d  wp %d/%d  objetivos %d visitados %d  "
                      "dh %.2f dz %.2f  ack %d" % (tel[0], estado, NOMBRES.get(estado, "?"), tel[3], tel[4],
                                                   tel[5], tel[6], tel[8], tel[9], tel[11], tel[12], tel[53]),
                      flush=True)
                estado_prev = estado
            if estado == 2 and not start_ok:
                seq += 1; cmd = 1; start_ok = True
                print("-> START (cmd_seq %d)" % seq, flush=True)
            en_casa = False
            if estado == 6:
                d1 = ((tel[16] - tel[28]) ** 2 + (tel[17] - tel[29]) ** 2) ** 0.5
                d2 = ((tel[34] - tel[46]) ** 2 + (tel[35] - tel[47]) ** 2) ** 0.5
                en_casa = d1 < 0.15 and d2 < 0.15
            if en_casa:
                t_casa = t_casa or time.time()
                if not land_ok and time.time() - t_casa > 6.0:
                    seq += 1; cmd = 3; land_ok = True
                    print("-> LAND_ALL (cmd_seq %d)" % seq, flush=True)
            else:
                t_casa = None
            if estado == 8:
                break
        ahora = time.time()
        if ahora - t_ultimo_envio >= 0.05:
            hb += 1
            v = [0.0] * N_CMD
            v[0], v[1], v[2] = float(hb), float(cmd), float(seq)
            try:
                s.sendall(struct.pack("<%dd" % N_CMD, *v))
            except OSError:
                print("error al enviar")
                break
            t_ultimo_envio = ahora
    print("paquetes de telemetría recibidos: %d (64 doubles c/u), heartbeats enviados: %d" % (n_rx, hb))
    if tel is not None:
        print("final: estado %d (%s), visitados %d de %d, mask %d, alerta %d, t_sim %.2f s, t_real %.2f s"
              % (tel[1], NOMBRES.get(int(tel[1]), "?"), tel[9], tel[8], tel[50], tel[54], tel[0], time.time() - t1))
    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
