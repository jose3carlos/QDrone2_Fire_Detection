"""Misión mock de 2 drones: roles, N completa, cola de objetivos, guard, pausa, abort y aterrizaje."""
import json

import pytest

from app import protocol as P
from app.mock_sim import MockMission, hdist
from app.mock_world import default_anomalies

DT = 0.02


class Driver:
    """Simula a la estación: manda heartbeat en cada paso, comandos con cmd_seq y objetivos."""

    def __init__(self, cfg, homes=((0.9, -0.4), (-0.9, 0.4))):
        self.m = MockMission(cfg, homes)
        self.hb = self.seq = 0
        self.cmd = 0
        self.slots = []
        self.n = 0
        self.heartbeat_on = True
        self.max_viol = 0.0

    def send(self, name):
        self.seq += 1
        self.cmd = P.COMMANDS[name]

    def step(self, n=1):
        for _ in range(n):
            if self.heartbeat_on:
                self.hb += 1
            msg = P.unpack_command(P.pack_command(self.hb, self.cmd, self.seq, self.n, self.slots[:3]))
            self.slots = self.slots[3:] + self.slots[:3]  # rotación
            self.m.apply_command_message(msg)
            self.m.step(DT)

    def run_until(self, cond, t_max):
        for _ in range(int(t_max / DT)):
            self.step()
            if cond():
                return True
        return False

    def set_targets(self, ts):
        self.slots = [(i + 1, a["cls"], a["x"], a["y"], 0.9) for i, a in enumerate(ts)]
        self.n = len(ts)


@pytest.fixture
def cfg(cfg_path):
    with open(cfg_path, encoding="utf-8") as fh:
        return json.load(fh)


def test_roles_por_menor_x(cfg):
    d = Driver(cfg)
    assert d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    assert (d.m.E, d.m.R) == (2, 1)       # dron 2 está en x = -0.9 (atrás)
    tel = P.unpack_telemetry(P.pack_telemetry(d.m.telemetry()))
    assert tel["drones"][2]["role"] == P.ROLE_EXPLORER and tel["drones"][1]["role"] == P.ROLE_RESPONDER
    assert tel["drones"][2]["home_x"] == pytest.approx(-0.9)
    d2 = Driver(cfg, homes=((-1.0, 0.0), (1.0, 0.0)))
    assert d2.run_until(lambda: d2.m.state == P.ASSIGN_ROLES, 5) and (d2.m.E, d2.m.R) == (1, 2)


def test_mision_completa_pid(cfg):
    d = Driver(cfg)
    d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    d.send("START")
    assert d.run_until(lambda: d.m.state == P.MISSION, 15)
    assert d.m.cmd_seq_ack == 1
    zE, zR = d.m.drones[d.m.E].p[2], d.m.drones[d.m.R].p[2]
    assert zE == pytest.approx(1.5, abs=0.13) and zR == pytest.approx(0.5, abs=0.13)
    d.run_until(lambda: False, 5)
    d.set_targets(default_anomalies(cfg["pista"]["vertices"]))
    assert d.run_until(lambda: d.m.state == P.RETURN_HOME, 150)
    tel = d.m.telemetry()
    assert tel["explorer_done"] == 1 and tel["n_targets_known"] == 3 and tel["n_targets_visited"] == 3
    assert tel["visited_mask"] == 0b111 and tel["queue_length"] == 0
    assert tel["explorer_wp_idx"] == tel["explorer_wp_total"]
    # la separación horizontal se respetó (margen por la dinámica de la planta)
    assert d.m.min_hdist_flying > cfg["mision"]["d_min_h"] - 0.05
    # regreso a home y LAND ALL
    assert d.run_until(lambda: all(hdist(x.p, x.home) < 0.12 for x in d.m.drones.values()), 40)
    d.send("LAND_ALL")
    assert d.run_until(lambda: d.m.state == P.DONE, 20)
    assert all(x.p[2] < 0.05 and not x.flying for x in d.m.drones.values())


def test_pausa_por_heartbeat_y_resume(cfg):
    d = Driver(cfg)
    d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    d.send("START")
    d.run_until(lambda: d.m.state == P.MISSION, 15)
    d.heartbeat_on = False
    assert d.run_until(lambda: d.m.state == P.PAUSED, 1.5)
    assert d.m.prev_state == P.MISSION
    pE = d.m.drones[d.m.E].p.copy()
    d.run_until(lambda: False, 2)
    assert hdist(pE, d.m.drones[d.m.E].p) < 0.15         # hover en su sitio
    d.heartbeat_on = True
    d.send("RESUME")
    assert d.run_until(lambda: d.m.state == P.MISSION, 1)


def test_pause_manual_y_abort(cfg):
    d = Driver(cfg)
    d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    d.send("START")
    d.run_until(lambda: d.m.state == P.MISSION, 15)
    d.run_until(lambda: False, 4)
    d.send("PAUSE")
    assert d.run_until(lambda: d.m.state == P.PAUSED, 0.5)
    d.send("ABORT")
    assert d.run_until(lambda: d.m.state == P.ABORT, 0.5)
    assert d.run_until(lambda: all(hdist(x.p, x.home) < 0.12 for x in d.m.drones.values()), 40)
    assert d.m.state == P.ABORT                         # queda en hover sobre home
    d.send("START")                                     # START no aplica en ABORT
    d.step(5)
    assert d.m.state == P.ABORT and any("ignorado" in e[1] for e in d.m.events)


def test_mpc_sin_guard(cfg):
    cfg = dict(cfg, version_control="MPC")
    d = Driver(cfg)
    d.run_until(lambda: d.m.state == P.ASSIGN_ROLES, 5)
    d.send("START")
    d.run_until(lambda: d.m.state == P.MISSION, 15)
    d.set_targets([{"cls": 0, "x": -1.0, "y": 1.2}])   # justo sobre el camino de E
    guard_seen = False
    for _ in range(int(40 / DT)):
        d.step()
        guard_seen |= d.m.guard
    assert not guard_seen
    assert d.m.telemetry()["version"] == 2
