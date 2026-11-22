"""Empaquetado del puente §3.3 (índices base 1, little-endian, tamaños fijos)."""
import struct

from app import protocol as P


def test_tamanos():
    assert P.TELEMETRY_BYTES == 512 and P.COMMAND_BYTES == 256


def test_comando_indices_base1():
    slots = [(1, 0, 0.5, -0.25, 0.9), (2, 1, -1.0, 1.0, 0.7), (3, 0, 0.0, 0.0, 0.6)]
    raw = P.pack_command(41, P.CMD_START, 7, 5, slots, 1.5, 0.5, 3.0, 0.9)
    v = struct.unpack("<32d", raw)  # little-endian explícito

    def idx(i):
        return v[i - 1]

    assert idx(1) == 41 and idx(2) == 1 and idx(3) == 7 and idx(4) == 5
    assert [idx(k) for k in range(5, 10)] == [1, 0, 0.5, -0.25, 0.9]
    assert [idx(k) for k in range(10, 15)] == [2, 1, -1.0, 1.0, 0.7]
    assert [idx(k) for k in range(15, 20)] == [3, 0, 0.0, 0.0, 0.6]
    assert (idx(20), idx(21), idx(22), idx(23)) == (1.5, 0.5, 3.0, 0.9)
    assert all(idx(k) == 0.0 for k in range(24, 33))


def test_comando_ranuras_vacias_y_roundtrip():
    raw = P.pack_command(1, P.CMD_NONE, 0, 1, [(4, 1, 0.1, 0.2, 0.8)])
    m = P.unpack_command(raw)
    assert m["slots"][0] == {"id": 4, "cls": 1, "x": 0.1, "y": 0.2, "conf": 0.8}
    assert m["slots"][1]["id"] == 0 and m["slots"][2]["id"] == 0  # id 0 = ranura vacía
    assert m["n_targets"] == 1


def test_telemetria_indices_base1():
    v = [0.0] * 64
    v[1 - 1] = 12.5            # t
    v[2 - 1] = 5               # MISSION
    v[3 - 1] = 2               # MPC
    v[4 - 1], v[5 - 1] = 2, 1  # explorador 2, respondedor 1
    v[12 - 1], v[13 - 1] = 1.23, -0.98
    o = 14                     # bloque del dron 1
    v[o + 1 - 1], v[o + 3 - 1], v[o + 5 - 1], v[o + 17 - 1] = 1, 0.4, 0.5, 2
    v[o + 14 - 1] = float("nan")  # sin batería
    o = 32                     # bloque del dron 2
    v[o + 3 - 1], v[o + 4 - 1], v[o + 6 - 1] = -1.0, 0.25, 0.3
    v[o + 15 - 1], v[o + 16 - 1], v[o + 18 - 1] = -0.9, 0.4, 1
    v[51 - 1] = 0b101          # visitados 1 y 3
    v[54 - 1] = 9              # cmd_seq_ack
    tel = P.unpack_telemetry(struct.pack("<64d", *v))
    assert tel["t"] == 12.5 and tel["mission_state_name"] == "MISSION" and tel["version_name"] == "MPC"
    assert (tel["explorer_id"], tel["responder_id"]) == (2, 1)
    assert tel["dist_horizontal"] == 1.23 and tel["dist_vertical"] == -0.98
    d1, d2 = tel["drones"][1], tel["drones"][2]
    assert d1["tracked"] == 1 and d1["x"] == 0.4 and d1["z"] == 0.5 and d1["role"] == 2 and d1["battery_V"] is None
    assert d2["x"] == -1.0 and d2["y"] == 0.25 and d2["yaw"] == 0.3 and d2["home_x"] == -0.9 and d2["comm_ok"] == 1
    assert tel["visited_ids"] == [1, 3] and tel["cmd_seq_ack"] == 9


def test_telemetria_roundtrip():
    tel = {"t": 3.0, "mission_state": 10, "version": 1, "explorer_id": 1, "responder_id": 2, "visited_mask": 6,
           "cmd_seq_ack": 4, "drones": {1: {"x": 1.0, "role": 1, "battery_V": 15.9}, 2: {"y": -0.5, "role": 2}}}
    back = P.unpack_telemetry(P.pack_telemetry(tel))
    assert back["mission_state_name"] == "PAUSED" and back["visited_ids"] == [2, 3]
    assert back["drones"][1]["x"] == 1.0 and back["drones"][1]["battery_V"] == 15.9
    assert back["drones"][2]["y"] == -0.5 and back["drones"][2]["role"] == 2


def test_rotacion_de_ranuras_cubre_todos():
    rot = P.TargetRotator()
    targets = [(i, i % 2, 0.1 * i, 0.0, 0.9) for i in range(1, 8)]  # 7 objetivos
    seen = []
    for _ in range(7):  # 7 mensajes x 3 ranuras = 21 = 3 vueltas completas
        seen += [s[0] for s in rot.next_slots(targets)]
    assert sorted(set(seen)) == list(range(1, 8))
    assert all(seen.count(i) == 3 for i in range(1, 8))
    assert P.TargetRotator().next_slots(targets[:2]) == targets[:2]
    assert P.TargetRotator().next_slots([]) == []
