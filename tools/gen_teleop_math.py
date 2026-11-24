#!/usr/bin/env python3
"""Run the Python teleop arithmetic (JointMapper, the ArmLeader friction feed-forward, SpeedCap and
the Teleop loop) on fixed inputs and write the results to contract/fixtures/teleop_math.json.

    ~/maker-arm-sdk/.venv/bin/python tools/gen_teleop_math.py            # write
    ~/maker-arm-sdk/.venv/bin/python tools/gen_teleop_math.py --check    # exit 1 if stale

mapping.py, leaders/arm.py and teleop.py are identical in both SDKs; the Metal Arm's shipped Star
mapping and friction terms are read from its repository. The Teleop loop runs on a fake clock that
moves only when the loop sleeps (as VirtualClock does), against a follower that tracks every target;
"stop_time" is the clock when the hook set the stop event (None when the loop stopped by itself).
Non-finite floats are written as the strings "inf", "-inf" and "nan".
"""

import argparse
import json
import math
import random
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import yaml

from maker_arm.leaders.arm import Friction
from maker_arm.mapping import JointMapper
from maker_arm.teleop import SpeedCap, Teleop

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "contract" / "fixtures" / "teleop_math.json"
HOME = Path.home()
ARMS = {
    "maker": HOME / "maker-arm-sdk" / "maker_arm" / "profiles",
    "metal": HOME / "metal-arm-sdk" / "metal_arm" / "profiles",
}
PROFILES = {"maker": "maker_arm_v1.yaml", "metal": "metal_arm_v1.yaml"}
MAPPINGS = {"maker": "star_to_maker_v1.json", "metal": "star_to_metal_v1.json"}
SEED = 2026_11


def enc(x):
    """JSON-safe copy: non-finite floats as strings."""
    if isinstance(x, float):
        if math.isnan(x):
            return "nan"
        if math.isinf(x):
            return "inf" if x > 0 else "-inf"
        return x
    if isinstance(x, (list, tuple)):
        return [enc(v) for v in x]
    if isinstance(x, dict):
        return {k: enc(v) for k, v in x.items()}
    return x


# ── mapping ──
def mapping_cases(rng):
    custom = [
        {"name": "offset", "servo": 3, "direction": 1.0, "offset_deg": 90.0, "range_deg": [0, 300]},
        {"name": "scaled", "servo": 1, "direction": 0.5, "range_deg": [0, 170]},
        {"name": "grip", "servo": 6, "direction": -1.2, "magnitude": True, "range_deg": [-120, 0]},
        {"name": "neg", "servo": 0, "direction": -0.667, "offset_deg": -12.5, "range_deg": [-123, 81]},
    ]
    mappings = {f"{arm}_shipped": json.loads((ARMS[arm] / MAPPINGS[arm]).read_text())["joints"] for arm in ARMS}
    mappings["custom"] = custom
    out = []
    for label, joints in mappings.items():
        ids = [j["servo"] for j in joints]
        reads = [
            {sid: 0.0 for sid in ids},
            {ids[0]: 10.0},                                        # partial: the rest stay NaN
            {sid: 180.0 for sid in ids},                           # half-turn ties
            {sid: -180.0 for sid in ids},
            {sid: 540.0 for sid in ids},
            {sid: math.nan for sid in ids},                        # all missing: the last output holds
            {sid: math.inf for sid in ids},
            {sid: -0.0 for sid in ids},
        ]
        for _ in range(60):
            reads.append({sid: rng.choice([rng.uniform(-200, 200), rng.uniform(-1500, 1500),
                                           round(rng.uniform(-720, 720), 1)])
                          for sid in ids if rng.random() > 0.1})
        mapper = JointMapper(joints)
        steps = [{"raw": [[sid, deg] for sid, deg in raw.items()], "out": mapper.map(raw)} for raw in reads]
        mapper.reset()
        steps.append({"reset": True, "raw": [[ids[-1], 33.3]], "out": mapper.map({ids[-1]: 33.3})})
        out.append({"label": label, "joints": joints, "steps": steps})
    unwrap = []
    for value in [180.0, -180.0, 540.0, 900.0, -540.0, 0.5, 359.9, 1e6 + 0.5, 12345.678]:
        for lo, hi in [(-180.0, 180.0), (0.0, 340.0), (-246.0, 0.0), (-158.0 / 1.446501, 156.0 / 1.446501)]:
            unwrap.append([value, lo, hi, JointMapper.unwrap(value, lo, hi)])
    return {"cases": out, "unwrap": unwrap}


# ── friction ──
def ns_config(raw):
    return SimpleNamespace(joints=[SimpleNamespace(**j) for j in raw["joints"]],
                           friction_deadzone=raw.get("friction_deadzone"))


def friction_cases():
    velocities = [0.0, -0.0, 0.01, -0.01, 0.049999, 0.05, -0.05, 0.051, 0.2, -0.2, 0.8, -0.8, 0.81, 1.2, 1.21, -1.25,
                  2.9, 3.0, 3.1, -3.5, 10.0, -10.0, math.nan, math.inf, -math.inf, 0.123456789, -7.7e-3, 0.333]
    out = {}
    for arm in ARMS:
        raw = yaml.safe_load((ARMS[arm] / PROFILES[arm]).read_text())
        friction = Friction.from_config(ns_config(raw))
        if friction is None:  # the profile has no friction terms
            out[arm] = None
            continue
        n = len(raw["joints"])
        rows = []
        for v in velocities:
            vel = [v] * n
            rows.append({"velocities": vel, "torques": friction.torques([0.0] * n, vel)})
        rng = random.Random(SEED)
        for _ in range(40):
            vel = [rng.uniform(-4, 4) for _ in range(n)]
            rows.append({"velocities": vel, "torques": friction.torques([0.0] * n, vel)})
        out[arm] = {"viscous": friction.viscous, "coulomb": friction.coulomb, "rest": friction.rest,
                    "clamp": friction.clamp, "deadzone": friction.deadzone, "rows": rows}
    custom = Friction((0.3, 0.0, 1.0), (0.05, -0.02, 0.0), (0.01, 0.0, -0.02), (math.inf, 0.5, 2.0), 0.1)
    rows = []
    for v in velocities:
        rows.append({"velocities": [v, -v, v * 0.5], "torques": custom.torques([0.0] * 3, [v, -v, v * 0.5])})
    out["custom"] = {"viscous": custom.viscous, "coulomb": custom.coulomb, "rest": custom.rest, "clamp": custom.clamp,
                     "deadzone": custom.deadzone, "rows": rows}
    return out


# ── speed cap ──
def speedcap_cases(rng):
    inf = math.inf
    configs = [
        ([2.0, 1.0, inf], [inf, 10.0, inf]),
        ([5.0, 0.5, 3.0], [100.0, 20.0, 4.0]),
        ([inf, inf, 0.3], [50.0, inf, inf]),
    ]
    out = []
    for vmax, accel in configs:
        start = [rng.uniform(-1, 1) for _ in range(3)]
        cap = SpeedCap(start)
        steps = []
        goal = [rng.uniform(-2, 2) for _ in range(3)]
        for k in range(150):
            if k % 37 == 0:
                goal = [rng.uniform(-2, 2) for _ in range(3)]
            dt = rng.choice([0.01, 0.01, 0.005, 0.02, 0.0, 0.05, 0.013])
            command = cap.step(goal, dt, vmax, accel)
            steps.append({"goal": list(goal), "dt": dt, "command": command, "velocity": list(cap.velocity)})
        out.append({"start": start, "max_velocity": vmax, "max_acceleration": accel, "steps": steps})
    return out


# ── the Teleop loop ──
class Clock:
    def __init__(self, start=100.0):
        self.now = start

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0.0, seconds)


NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "gripper"]


class Follower:
    def __init__(self, positions):
        self.config = SimpleNamespace(joints=[SimpleNamespace(name=n, lo=-3.0, hi=3.0) for n in NAMES])
        self.positions = list(positions)
        self.state = SimpleNamespace(name="ENABLED")
        self.fault_reason = None
        self.targets = []

    def get_joint_positions(self):
        return list(self.positions)

    def set_joint_targets(self, targets):
        self.targets.append(list(targets))
        self.positions = list(targets)
        return True

    def hold_current_position(self):
        return True


class Scripted:
    name = "scripted"

    def __init__(self, script):
        self.script = list(script)

    def connect(self, cancel=None):
        pass

    def read(self):
        item = self.script.pop(0) if len(self.script) > 1 else self.script[0]
        return None if item is None else list(item)

    def close(self):
        pass


def teleop_cases(rng):
    def wave(k, phase):
        return [0.6 * math.sin(0.03 * k + phase + i) + 0.1 * i for i in range(7)]

    scenarios = [
        {"label": "sync then follow, capped", "start": [0.0] * 7, "options": {"rate_hz": 100.0, "max_velocity": 1.5,
         "max_acceleration": 8.0, "sync_threshold": 0.4, "sync_velocity": 0.5},
         "script": [wave(0, 1.0)] * 250 + [wave(k, 1.0) for k in range(400)], "cycles": 600},
        {"label": "no cap, misses", "start": wave(0, 0.0), "options": {"rate_hz": 50.0, "miss_limit": 10},
         "script": [wave(k, 0.0) if k % 7 else None for k in range(120)], "cycles": 150},
        {"label": "velocity cap only", "start": [0.2] * 7, "options": {"rate_hz": 200.0, "max_velocity": 0.7,
         "sync_threshold": 3.0}, "script": [[rng.uniform(-1, 1) for _ in range(7)] for _ in range(60)], "cycles": 120},
        {"label": "startup reading late", "start": [0.0] * 7, "options": {"rate_hz": 100.0, "sync_threshold": 0.1,
         "sync_velocity": 0.3}, "script": [None] * 5 + [wave(0, 2.0)] * 450 + [wave(k, 2.0) for k in range(100)],
         "cycles": 560},
    ]
    out = []
    for sc in scenarios:
        clock = Clock()
        stop = threading.Event()
        steps = []
        stopped_at = []

        def hook(step, steps=steps, stop=stop, n=sc["cycles"], stopped_at=stopped_at, clock=clock):
            steps.append({"elapsed_s": step.elapsed_s, "leader": step.leader, "command": step.command,
                          "syncing": step.syncing})
            if len(steps) >= n:
                stopped_at.append(clock.now)
                stop.set()

        follower = Follower(sc["start"])
        loop = Teleop(Scripted(sc["script"]), follower, on_step=hook, clock=clock, sleep=clock.sleep, **sc["options"])
        reason = loop.run(stop)
        out.append({**{k: sc[k] for k in ("label", "start", "options", "script", "cycles")}, "reason": reason,
                    "startup_gap": loop.startup_gap, "steps": steps,
                    "stop_time": stopped_at[0] if stopped_at else None})
    return out


def build():
    rng = random.Random(SEED)
    return enc({
        "generator": "tools/gen_teleop_math.py",
        "mapping": mapping_cases(rng),
        "friction": friction_cases(),
        "speedcap": speedcap_cases(rng),
        "teleop": teleop_cases(rng),
    })


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    text = json.dumps(build(), indent=1) + "\n"
    if a.check:
        if not OUT.exists() or OUT.read_text() != text:
            print(f"stale: {OUT.relative_to(ROOT)}")
            return 1
        return 0
    OUT.write_text(text)
    print("wrote", OUT.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
