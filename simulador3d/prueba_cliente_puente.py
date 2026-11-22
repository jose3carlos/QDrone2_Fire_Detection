"""Cliente de prueba del puente §3.3 del simulador 3D (tcpip://127.0.0.1:18700).

Se conecta como lo haría la estación Python, lee la telemetría de 64 doubles, manda paquetes
de 32 doubles a 20 Hz (heartbeat creciente) y da START. Opcionalmente publica un objetivo
(para probar ``use_vision_python = true``) o manda una secuencia de comandos.

Modos:
  * por defecto: socket crudo + struct (no depende de nada del repo);
  * ``--con-bridge``: usa ``web/app/bridge.BridgeClient`` (el cliente REAL de la estación).

Uso (con el simulador corriendo y usar_tcp = true):
    python simulador3d/prueba_cliente_puente.py --segundos 40 --start-en 2
    python simulador3d/prueba_cliente_puente.py --con-bridge --segundos 30
    python simulador3d/prueba_cliente_puente.py --objetivo 0.3 1.2 --segundos 60
    python simulador3d/prueba_cliente_puente.py --objetivo 0.603 1.28 0 --objetivo -0.172 -0.059 1 ...

Comprobaciones extra (modo socket):
    --comando-sim T NOMBRE      comando cuando el tiempo SIMULADO llega a T (útil a x16)
    --prohibir-pausa-hb         falla si la misión entra a PAUSED sin alerta de rastreo (y55 = 2)
    --rastreo D T1 T2           verifica que el dron D llegue con tracked = 0 y pose NaN en [T1, T2)
                                simulados y con tracked = 1 fuera (margen de 0.1 s)
    --verificar-56              verifica que y(56:60) rote por TODOS los objetivos conocidos
    --start-primer-paquete      manda START (cmd_seq 1) en el primer paquete durante 2 s y verifica
                                que el simulador no lo ejecute ni lo confirme

Sale con código 0 si todas las comprobaciones pasan. Solo usa la biblioteca estándar.
"""
from __future__ import annotations

import argparse
import json
import os
import select
import socket
import struct
import sys
import time

TEL = struct.Struct("<64d")
CMD = struct.Struct("<32d")
ESTADOS = ["INIT", "WAIT_TRACKING", "ASSIGN_ROLES", "TAKEOFF", "HOVER_READY", "MISSION",
           "RETURN_HOME", "LANDING", "DONE", "ABORT", "PAUSED"]
CMDS = {"START": 1, "ABORT": 2, "LAND_ALL": 3, "PAUSE": 4, "RESUME": 5}


def conectar(host, puerto, espera):
    t0 = time.time()
    while True:
        try:
            s = socket.create_connection((host, puerto), timeout=2.0)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.setblocking(False)
            return s, time.time() - t0
        except OSError:
            if time.time() - t0 > espera:
                raise
            time.sleep(0.5)


def resumen_tel(v):
    """Campos principales (índices base 1 del contrato -> base 0 aquí)."""
    d = {"t": v[0], "estado": int(v[1]), "version": int(v[2]), "explorer_id": int(v[3]),
         "responder_id": int(v[4]), "wp": int(v[5]), "wp_total": int(v[6]),
         "n_targets_known": int(v[8]), "n_targets_visited": int(v[9]), "guard": int(v[10]),
         "dist_h": round(v[11], 3), "dist_v": round(v[12], 3), "visited_mask": int(v[50]),
         "cmd_seq_ack": int(v[53]), "alerta_55": v[54], "ext_56_60": [round(x, 3) for x in v[55:60]]}
    for i, o in ((1, 14), (2, 32)):
        d[f"dron{i}"] = {"tracked": int(v[o]), "fsm": int(v[o + 1]),
                         "xyz": [round(v[o + 2], 3), round(v[o + 3], 3), round(v[o + 4], 3)],
                         "sp": [round(v[o + 6], 3), round(v[o + 7], 3), round(v[o + 8], 3)],
                         "bat": round(v[o + 13], 2), "home": [round(v[o + 14], 3), round(v[o + 15], 3)],
                         "rol": int(v[o + 16]), "comm": int(v[o + 17])}
    return d


def modo_socket(a):
    s, t_con = conectar(a.host, a.puerto, a.espera)
    print(f"conectado a {a.host}:{a.puerto} tras {t_con:.1f} s")
    buf = bytearray()
    n_rx = 0
    t_ini = time.time()
    hb = 0
    seq = 0
    cmd = 0
    pendiente = None          # (seq, nombre, t_envio)
    acks = []
    estados = []
    ult = None
    rx_t = []
    t_tx = time.time()
    programados = list(a.comando)          # [(t, nombre)]
    if a.start_en is not None:
        programados.append((a.start_en, "START"))
    programados.sort()
    hecho = [False] * len(programados)
    prog_sim = sorted(a.comando_sim)                  # [(t_sim, nombre)]
    hecho_sim = [False] * len(prog_sim)
    t_sim = 0.0
    trans = []                                        # (t_sim, estado_anterior, estado, y55)
    rastreo_mal = []                                  # muestras que no cumplen --rastreo
    n_rastreo = [0, 0]                                # muestras [dentro, fuera] de la ventana
    ids_56 = set()
    n_known_max = 0
    primer = {"ack_max": 0, "estado_max": 0}
    t_primer = None
    if a.start_primer_paquete:
        seq, cmd = 1, CMDS["START"]
        t_primer = time.time() + 2.0
    while time.time() - t_ini < a.segundos:
        r, _, _ = select.select([s], [], [], 0.01)
        if r:
            chunk = s.recv(65536)
            if not chunk:
                print("el servidor cerró la conexión")
                break
            buf += chunk
            while len(buf) >= TEL.size:
                v = TEL.unpack(bytes(buf[:TEL.size]))
                del buf[:TEL.size]
                n_rx += 1
                rx_t.append(time.time())
                ult = v
                e = int(v[1])
                t_sim = v[0]
                if not estados or estados[-1][1] != e:
                    if estados:
                        trans.append((round(v[0], 2), estados[-1][1], e, v[54]))
                    estados.append((round(time.time() - t_ini, 2), e, round(v[0], 2)))
                if t_primer is not None and time.time() < t_primer:
                    primer["ack_max"] = max(primer["ack_max"], int(v[53]))
                    primer["estado_max"] = max(primer["estado_max"], e)
                if a.rastreo:
                    d, t1, t2 = int(a.rastreo[0]), a.rastreo[1], a.rastreo[2]
                    o = 14 if d == 1 else 32
                    trk = int(v[o])
                    pose_nan = all(x != x for x in v[o + 2:o + 6])
                    if t1 + 0.1 <= v[0] < t2 - 0.1:
                        n_rastreo[0] += 1
                        if trk != 0 or not pose_nan:
                            rastreo_mal.append((round(v[0], 2), trk, v[o + 2]))
                    elif v[0] < t1 - 0.1 or v[0] >= t2 + 0.1:
                        n_rastreo[1] += 1
                        if trk != 1 or pose_nan:
                            rastreo_mal.append((round(v[0], 2), trk, v[o + 2]))
                n_known_max = max(n_known_max, int(v[8]))
                if v[55] > 0:
                    ids_56.add(int(v[55]))
                if pendiente and int(v[53]) == pendiente[0]:
                    acks.append({"cmd": pendiente[1], "cmd_seq": pendiente[0],
                                 "latencia_s": round(time.time() - pendiente[2], 3)})
                    pendiente = None
                    cmd = 0
        ahora = time.time()
        if t_primer is not None and ahora >= t_primer:
            t_primer = None
            seq += 1                      # "caduca" el START del primer paquete (como BridgeClient)
            cmd = 0
            print(f"t={ahora - t_ini:6.2f} s  START del primer paquete: ack máx. {primer['ack_max']}, "
                  f"estado máx. {primer['estado_max']}; se manda cmd 0 con cmd_seq {seq}")
        for i, (ts, nombre) in enumerate(prog_sim):
            if not hecho_sim[i] and t_sim >= ts and pendiente is None:
                hecho_sim[i] = True
                seq = max(seq, int(ult[53]) if ult else 0) + 1
                cmd = CMDS[nombre]
                pendiente = (seq, nombre, ahora)
                print(f"t_sim={t_sim:7.2f} s  -> {nombre} (cmd_seq {seq})")
        for i, (tc, nombre) in enumerate(programados):
            if not hecho[i] and ahora - t_ini >= tc and pendiente is None and t_primer is None:
                hecho[i] = True
                seq = max(seq, int(ult[53]) if ult else 0) + 1
                cmd = CMDS[nombre]
                pendiente = (seq, nombre, ahora)
                print(f"t={ahora - t_ini:6.2f} s  -> {nombre} (cmd_seq {seq})")
        if ahora - t_tx >= 0.05:          # 20 Hz como la estación
            t_tx = ahora
            hb += 1
            v = [0.0] * 32
            v[0], v[1], v[2] = hb, cmd, seq
            if a.objetivo:
                # ids 1..n; 3 ranuras por paquete, rotando (como BridgeClient)
                n = len(a.objetivo)
                v[3] = n
                for j in range(min(3, n)):
                    k = (hb * 3 + j) % n if n > 3 else j
                    ob = a.objetivo[k]
                    cls = ob[2] if len(ob) > 2 else 1.0
                    v[4 + 5 * j:9 + 5 * j] = [k + 1, cls, ob[0], ob[1], 0.9]
            s.setblocking(True)
            s.sendall(CMD.pack(*v))
            s.setblocking(False)
    s.close()
    dur = time.time() - t_ini
    ventana = [x for x in rx_t if x > rx_t[-1] - 5.0] if rx_t else []
    res = {
        "paquetes_rx": n_rx, "hz_medio": round(n_rx / dur, 1),
        "hz_ultimos_5s": round(len(ventana) / 5.0, 1) if ventana else 0,
        "heartbeats_tx": hb,
        "estados": [{"t_cliente": t, "estado": ESTADOS[e], "t_sim": ts} for t, e, ts in estados],
        "acks": acks, "ultimo": resumen_tel(ult) if ult else None,
        "transiciones_t_sim": trans,
    }
    chk = {}
    if a.prohibir_pausa_hb:
        malas = [x for x in trans if x[2] == 10 and x[3] != 2]
        chk["pausa_por_heartbeat"] = {"ok": not malas, "transiciones_a_PAUSED_sin_alerta_2": malas}
    if a.rastreo:
        chk["rastreo"] = {"ok": not rastreo_mal and n_rastreo[0] > 0 and n_rastreo[1] > 0,
                          "muestras_dentro_fuera": n_rastreo, "malas": rastreo_mal[:10]}
    if a.verificar_56:
        # los ids de la misión no tienen que ser 1..n (la misión puede fusionar objetivos):
        # basta con que por y(56) pasen tantos ids distintos como objetivos conocidos
        chk["rotacion_56"] = {"ok": n_known_max > 0 and len(ids_56) >= n_known_max, "ids_vistos": sorted(ids_56),
                              "n_targets_known_max": n_known_max}
    if a.start_primer_paquete:
        chk["start_primer_paquete"] = {"ok": primer["ack_max"] < 1 and primer["estado_max"] < 3, **primer}
    res["comprobaciones"] = chk
    return res


def modo_bridge(a):
    raiz = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, os.path.join(raiz, "web"))
    from app.bridge import BridgeClient   # cliente real de la estación
    b = BridgeClient(a.host, a.puerto).start()
    t_ini = time.time()
    while not b.fresh():
        if time.time() - t_ini > a.espera:
            raise TimeoutError("sin telemetría del simulador")
        time.sleep(0.2)
    print(f"BridgeClient con telemetría tras {time.time() - t_ini:.1f} s")
    if a.objetivo:
        b.set_targets([(k + 1, ob[2] if len(ob) > 2 else 1, ob[0], ob[1], 0.9) for k, ob in enumerate(a.objetivo)])
    estados = []
    enviados = []
    t0 = time.time()
    programados = list(a.comando)
    if a.start_en is not None:
        programados.append((a.start_en, "START"))
    programados.sort()
    hecho = [False] * len(programados)
    while time.time() - t0 < a.segundos:
        tel, _ = b.latest()
        if tel:
            e = tel["mission_state"]
            if not estados or estados[-1][1] != e:
                estados.append((round(time.time() - t0, 2), e, round(tel["t"], 2)))
        for i, (tc, nombre) in enumerate(programados):
            if not hecho[i] and time.time() - t0 >= tc:
                hecho[i] = True
                enviados.append((nombre, b.send_command(nombre)))
                print(f"t={time.time() - t0:6.2f} s  -> {nombre} con BridgeClient")
        time.sleep(0.02)
    st = b.status()
    tel, _ = b.latest()
    b.stop()
    return {"estados": [{"t_cliente": t, "estado": ESTADOS[e], "t_sim": ts} for t, e, ts in estados],
            "enviados": enviados, "status_bridge": {k: st[k] for k in ("connected", "rx_hz", "tx_hz", "cmd_seq",
                                                                        "cmd_seq_ack", "cmd_resultados")},
            "ultimo": {k: tel[k] for k in ("t", "mission_state_name", "explorer_id", "n_targets_known",
                                           "n_targets_visited", "dist_horizontal", "cmd_seq_ack")} if tel else None}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--puerto", type=int, default=18700)
    ap.add_argument("--segundos", type=float, default=30.0, help="duración de la prueba tras conectar")
    ap.add_argument("--espera", type=float, default=180.0, help="s máximos esperando al servidor")
    ap.add_argument("--start-en", type=float, default=2.0, help="s tras conectar para mandar START (-1 = no)")
    ap.add_argument("--comando", nargs=2, action="append", default=[], metavar=("T", "NOMBRE"),
                    help="comando extra: --comando 25 PAUSE --comando 30 RESUME")
    ap.add_argument("--objetivo", nargs="+", type=float, action="append", default=[], metavar="X Y [CLS]",
                    help="publica un objetivo en (X, Y) (clase 1 = toma por defecto; 0 = fuego); se puede repetir")
    ap.add_argument("--comando-sim", nargs=2, action="append", default=[], metavar=("T_SIM", "NOMBRE"),
                    help="comando cuando el tiempo simulado llega a T_SIM")
    ap.add_argument("--prohibir-pausa-hb", action="store_true")
    ap.add_argument("--rastreo", nargs=3, type=float, metavar=("DRON", "T1", "T2"))
    ap.add_argument("--verificar-56", action="store_true")
    ap.add_argument("--start-primer-paquete", action="store_true")
    ap.add_argument("--con-bridge", action="store_true", help="usar web/app/bridge.BridgeClient")
    ap.add_argument("--json", help="guardar el resumen en este archivo")
    ap.add_argument("--min-estado", type=int, default=3, help="estado mínimo que se debe alcanzar")
    a = ap.parse_args()
    a.comando = [(float(t), n.upper()) for t, n in a.comando]
    a.comando_sim = [(float(t), n.upper()) for t, n in a.comando_sim]
    for ob in a.objetivo:
        if len(ob) not in (2, 3):
            ap.error("--objetivo necesita X Y [CLS]")
    if a.start_en is not None and a.start_en < 0:
        a.start_en = None
    res = modo_bridge(a) if a.con_bridge else modo_socket(a)
    print(json.dumps(res, indent=1, ensure_ascii=False))
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=1, ensure_ascii=False)
    # comprobaciones
    est = [ESTADOS.index(x["estado"]) for x in res["estados"]]
    ok = bool(est) and max(e for e in est if e != 10) >= a.min_estado
    if not a.con_bridge:
        ok = ok and res["hz_medio"] > 15 and (a.start_en is None or any(x["cmd"] == "START" for x in res["acks"]))
        for nombre, c in res.get("comprobaciones", {}).items():
            print(f"  comprobación {nombre}: {'OK' if c['ok'] else 'FALLO'}")
            ok = ok and c["ok"]
    print("RESULTADO:", "OK" if ok else "FALLO")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
