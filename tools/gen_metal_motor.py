#!/usr/bin/env python3
"""Run the Python Metal Arm Motor against the Python SimBus on a fake clock and record every frame
it sends, in order, with each call's result, to contract/fixtures/metal_motor.json.

    ~/metal-arm-sdk/.venv/bin/python tools/gen_metal_motor.py            # write
    ~/metal-arm-sdk/.venv/bin/python tools/gen_metal_motor.py --check    # exit 1 if stale

Single-threaded: the simulated motors reply inside send(), `time.monotonic()` in metal_arm.motor
returns fake time, `time.sleep(x)` advances it by exactly x, and a waiter's Event.wait(x) returns at
once when set and otherwise advances fake time by x (nothing can set it while it waits).

A scenario is the simulated motors, the Motor joints, how received frames reach the motors
("dispatch": motor.route then handle_frame, as the arm does; "direct": the one motor's
handle_frame), and steps. Each step records the frames sent during it, its result, the fake time
after it, and a snapshot of every Motor and SimMotor.
"""

import argparse
import json
import math
import random
import sys
import threading
import types
from pathlib import Path

from metal_arm import _sim_motor as sim
from metal_arm import motor as motor_module
from metal_arm import protocol as p
from metal_arm.config import JointConfig
from metal_arm.motor import Motor, route
from metal_arm.transport.base import CanId

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "contract" / "fixtures" / "metal_motor.json"
SEED = 7_2026
START = 1000.0


class FakeClock:
    def __init__(self):
        self.t = START

    def monotonic(self):
        return self.t

    def sleep(self, x):
        self.t += x


CLOCK = FakeClock()


class FakeEvent:
    def __init__(self):
        self._set = False

    def set(self):
        self._set = True

    def clear(self):
        self._set = False

    def is_set(self):
        return self._set

    def wait(self, timeout=None):
        if self._set:
            return True
        CLOCK.t += timeout
        return False


motor_module.time = types.SimpleNamespace(monotonic=CLOCK.monotonic, sleep=CLOCK.sleep)
motor_module.threading = types.SimpleNamespace(Event=FakeEvent, Lock=threading.Lock, RLock=threading.RLock)


def f(x):
    if isinstance(x, float):
        if math.isnan(x):
            return "nan"
        if math.isinf(x):
            return "inf" if x > 0 else "-inf"
    return x


def jsonable(v):
    if isinstance(v, motor_module.MotorFeedback):
        return {"motor_id": v.motor_id, "position": f(v.position), "velocity": f(v.velocity), "torque": f(v.torque),
                "temperature": f(v.temperature), "temperatures": {k: f(x) for k, x in v.temperatures.items()},
                "enabled": v.enabled, "warning": v.warning, "status": v.status}
    if isinstance(v, motor_module.MotorStatus):
        return {"motor_id": v.motor_id, "has_fault": v.has_fault, "code": v.code, "text": v.text, "enabled": v.enabled}
    if isinstance(v, (list, tuple)):
        return [jsonable(x) for x in v]
    if isinstance(v, dict):
        return {str(k): jsonable(x) for k, x in v.items()}
    return f(v)


def arg(x):
    if x in ("nan", "inf", "-inf"):
        return float(x)
    return x


def make_joint(motor_id, model, **kwargs):
    defaults = dict(lo=-1.0, hi=1.0, kp=20.0, kd=0.6, name="")
    defaults.update(kwargs)
    return JointConfig(motor_id=motor_id, model=model, **defaults)


def motor_snapshot(mo):
    return {"motor_id": mo.motor_id, "reply_id": mo.reply_id, "flush_id": mo.flush_id,
            "feedback": jsonable(mo.feedback), "status": jsonable(mo.status), "fault_latched": mo.fault_latched,
            "feedback_sequence": mo.feedback_sequence, "feedback_age": f(mo.feedback_age),
            "watchdog_released": mo.watchdog_released}


def sim_snapshot(s):
    return {"send_id": s.send_id, "reply_id": s.reply_id, "position": f(s.position), "enabled": s.enabled,
            "fault": s.fault, "drop": s.drop, "held": [[c, bytes(d).hex()] for c, d in s.held], "log": jsonable(s.log),
            "registers": jsonable(s.registers), "flash": jsonable(s.flash)}


def run(scenario):
    CLOCK.t = START
    host = scenario.get("host_id", p.DEFAULT_MASTER_OFFSET)
    sims = [sim.SimMotor(s["id"], s.get("model", "jlo"), **s.get("options", {})) for s in scenario["motors"]]
    bus = sim.SimBus(sims)
    motors = {}
    for s in scenario["motors"]:
        jm = s.get("joint_model", s.get("model", "jlo"))
        motors[s["id"]] = Motor(make_joint(s["id"], jm, **s.get("joint", {})), bus, host)
    if scenario.get("callback", "dispatch") == "direct":
        bus.set_recv_callback(motors[scenario["motors"][0]["id"]].handle_frame)
    else:
        def dispatch(can_id, data):
            mo = motors.get(route(can_id, data, host))
            if mo is not None:
                mo.handle_frame(can_id, data)
        bus.set_recv_callback(dispatch)
    records = []
    for step in scenario["steps"]:
        before = len(bus.sent)
        result = None
        op = step["op"]
        try:
            if op == "call":
                result = {"ok": jsonable(getattr(motors[step["motor"]], step["method"])(*map(arg, step.get("args", []))))}
            elif op == "sim":
                s = bus.motors[step["motor"]]
                for k, v in step["set"].items():
                    setattr(s, k, arg(v))
            elif op == "register":
                bus.motors[step["motor"]].registers[step["rid"]] = step["value"]
            elif op == "held":
                bus.motors[step["motor"]].held = [(c, bytes.fromhex(d)) for c, d in step["frames"]]
            elif op == "power_cycle":
                bus.motors[step["motor"]].power_cycle()
            elif op == "advance":
                CLOCK.t += step["dt"]
            elif op == "handle_frame":
                cid = CanId(step["can_id"], extended=True) if step.get("extended") else step["can_id"]
                motors[step["motor"]].handle_frame(cid, bytes.fromhex(step["data"]))
            else:
                raise RuntimeError(op)
        except Exception as e:   # noqa: BLE001
            result = {"error": type(e).__name__, "message": str(e)}
        sent = [[c, bytes(d).hex(), x] for (c, d), x in zip(bus.sent[before:], bus.sent_extended[before:])]
        records.append({"sent": sent, "result": result, "time": CLOCK.t,
                        "motors": {str(k): motor_snapshot(mo) for k, mo in motors.items()},
                        "sims": {str(k): sim_snapshot(s) for k, s in bus.motors.items()}})
    return {**scenario, "records": records}


def call(motor, method, *args):
    return {"op": "call", "motor": motor, "method": method, "args": [f(a) for a in args]}


def fb(motor_id, model="jlo", **kw):
    return sim.feedback_bytes(motor_id, model=model, **kw).hex()


def scenarios():
    out = []

    def add(name, motors, steps, **kw):
        out.append({"name": name, "motors": motors, "steps": steps, **kw})

    every_query = [
        call(3, "check"), call(3, "read_state"), call(3, "validate"), call(3, "preload", 0.5),
        call(3, "preload", "nan"), call(3, "enable_confirmed"), call(3, "confirms", "enabled"),
        call(3, "send_command", 0.25, 0.1, 30.0, 1.0, 0.5), call(3, "health_problem"), call(3, "poll_status"),
        call(3, "read_state", 0.02, 1), call(3, "disable_confirmed"), call(3, "confirms", "disabled"),
        call(3, "enable"), call(3, "disable"), call(3, "clear_fault"), call(3, "set_zero"),
        call(3, "arm_watchdog", 500), call(3, "restore_watchdog", 0), call(3, "restore_watchdog", None),
        call(3, "arm_watchdog", 0), call(3, "arm_watchdog", 12.525), call(3, "read_param", "ctrl_mode"),
        call(3, "read_param", p.Reg.VMAX), call(3, "read_param", "PMAX"), call(3, "read_param", 59),
        call(3, "read_param", 9, "f"), call(3, "write_param", "kp_apr", 12.5), call(3, "write_param", "kp_apr", 0.1),
        call(3, "write_param", "timeout", 40, None, 0.12, True), call(3, "read_param", "nope"),
        call(3, "write_param", "timeout", -1), call(3, "write_param", 27, "inf"), call(3, "confirms", "on"),
        {"op": "advance", "dt": 0.75}, call(3, "check", 0.0), call(3, "check", 0.05, 0),
    ]
    add("healthy", [{"id": 3, "options": {"position": 0.7}}], every_query)
    add("healthy_first_id", [{"id": 1, "model": "j2", "options": {"position": -0.4, "temperature": 41,
                                                                    "rotor_temperature": 48}}],
        [call(1, "read_state"), call(1, "enable_confirmed"), call(1, "send_command", 0.5, 0, 50, 1, 0),
         call(1, "check"), call(1, "set_id", 1), call(1, "set_id", 16), call(1, "set_id", 0)])

    faulted = [call(7, "check"), call(7, "read_state"), call(7, "health_problem"), call(7, "confirms", "disabled"),
               call(7, "enable_confirmed"), call(7, "disable_confirmed"), call(7, "preload", 0.1),
               call(7, "poll_status"), call(7, "clear_fault"), call(7, "check"), call(7, "enable_confirmed"),
               call(7, "health_problem"), {"op": "sim", "motor": 7, "set": {"fault": 0xE}}, call(7, "read_state"),
               call(7, "health_problem"), call(7, "confirms", "enabled"), call(7, "clear_fault"),
               call(7, "enable_confirmed")]
    add("faulted_comm_lost", [{"id": 7, "model": "jhi", "options": {"fault": p.STATUS_COMM_LOST}}], faulted)
    add("faulted_rotor", [{"id": 2, "options": {"faulted": True}}],
        [call(2, "poll_status"), call(2, "check"), call(2, "clear_fault"), call(2, "check"),
         call(2, "write_param", "timeout", 40, None, 0.12, True)])

    offline = [call(1, "check", 0.005, 1), call(1, "check"), call(1, "read_state", 0.005, 2), call(1, "read_state"),
               call(1, "validate", 0.005), call(1, "validate"), call(1, "preload", 0.2), call(1, "enable_confirmed"),
               call(1, "disable_confirmed", 0.005, 1), call(1, "disable_confirmed"), call(1, "read_param", "ctrl_mode", None, 0.005),
               call(1, "write_param", "timeout", 100), call(1, "write_param", "timeout", 100, None, 0.12, True),
               call(1, "arm_watchdog", 500), call(1, "restore_watchdog", 0, 0.03), call(1, "set_id", 5, 0.02),
               call(1, "health_problem")]
    add("offline", [{"id": 1, "options": {"online": False}}], offline)
    add("offline_with_neighbour", [{"id": 4, "options": {"online": False}}, {"id": 3, "options": {"position": 0.2}}],
        [call(4, "read_state"), call(4, "check", 0.01, 2), call(4, "enable_confirmed")])

    add("drop", [{"id": 2, "options": {"position": 0.3, "drop": 1}}],
        [call(2, "read_state", 0.01), {"op": "sim", "motor": 2, "set": {"drop": 2}}, call(2, "read_param", "ctrl_mode"),
         {"op": "sim", "motor": 2, "set": {"drop": 1}}, call(2, "write_param", "timeout", 99),
         {"op": "sim", "motor": 2, "set": {"drop": 3}}, call(2, "enable_confirmed"),
         {"op": "sim", "motor": 2, "set": {"drop": 1}}, call(2, "arm_watchdog", 500)])

    late = [call(7, "read_state"), call(7, "check"), call(7, "validate"), call(7, "enable_confirmed"),
            call(7, "send_command", 0.3, 0, 20, 0.6, 0), call(7, "preload", 0.35), call(7, "disable_confirmed"),
            call(7, "write_param", "timeout", 10_000), call(7, "arm_watchdog", 500), call(7, "restore_watchdog", 0),
            call(7, "read_param", "timeout"), call(7, "clear_fault"), call(7, "poll_status"), call(7, "read_state")]
    add("late_gripper", [{"id": 6, "model": "jhi", "options": {"position": -0.2}},
                         {"id": 7, "model": "jhi", "options": {"position": 0.4, "late": True}}], late)
    add("late_duplicate_gripper", [{"id": 6, "model": "jhi", "options": {"position": -0.2}},
                                   {"id": 7, "model": "jhi", "options": {"position": 0.4, "late": True,
                                                                         "duplicate": True}}], late)
    add("late_alone", [{"id": 7, "model": "jhi", "options": {"position": 0.4, "late": True}}], late)
    for dup in (False, True):
        steps = [{"op": "held", "motor": 7, "frames": [[0x17, fb(7, "jhi", pos=0.1)]]}]
        for pos in (0.9, 0.7, 0.5):
            steps += [{"op": "sim", "motor": 7, "set": {"position": pos}}, call(7, "read_state")]
        add(f"stale_held_state_dup_{dup}", [{"id": 7, "model": "jhi", "options": {"position": 0.9, "late": True,
                                                                                  "duplicate": dup}}], steps)
    stale_echo = sim.register_reply(7, p.OP_WRITE, p.Reg.TIMEOUT, (200).to_bytes(4, "little")).hex()
    add("stale_register_echo", [{"id": 7, "model": "jhi", "options": {"late": True}}],
        [{"op": "held", "motor": 7, "frames": [[0x17, stale_echo]]}, call(7, "write_param", "timeout", 10_000)])
    add("late_stale_enable", [{"id": 7, "model": "jhi", "options": {"late": True, "duplicate": True}}],
        [{"op": "held", "motor": 7, "frames": [[0x17, fb(7, "jhi", status=0)]]}, call(7, "enable_confirmed"),
         {"op": "held", "motor": 7, "frames": [[0x17, fb(7, "jhi", status=1)]]}, call(7, "disable_confirmed")])

    add("punctual_trailing_flush", [{"id": 3}], [call(3, "read_state")])
    add("foreign_frames", [{"id": 1}],
        [{"op": "handle_frame", "motor": 1, "can_id": 0x12, "data": fb(1, pos=1.0)},
         {"op": "handle_frame", "motor": 1, "can_id": 0x11, "data": fb(2, pos=1.0)},
         {"op": "handle_frame", "motor": 1, "can_id": 0x11, "data": fb(1, pos=1.0), "extended": True},
         {"op": "handle_frame", "motor": 1, "can_id": 0x11, "data": fb(1, pos=1.0, status=0xB)},
         call(1, "health_problem")])
    add("disabled_ticks", [{"id": 1, "options": {"enabled": True}}],
        [call(1, "enable"), call(1, "health_problem"), {"op": "sim", "motor": 1, "set": {"enabled": False}},
         call(1, "check")] + [call(1, "health_problem")] * 6 + [call(1, "enable_confirmed"), call(1, "health_problem")])

    add("validate_mode", [{"id": 7, "model": "jhi", "options": {"registers": {p.Reg.CTRL_MODE: p.MODE_FORCE_POS}}}],
        [call(7, "validate")])
    add("validate_scaling", [{"id": 2, "model": "j2", "joint_model": "jlo", "joint": {"name": "shoulder_lift"}}],
        [call(2, "validate"), {"op": "register", "motor": 2, "rid": p.Reg.TMAX, "value": 7.0}, call(2, "validate")])
    add("watchdog_register", [{"id": 7, "model": "jhi", "options": {"registers": {p.Reg.TIMEOUT: 0}}}],
        [call(7, "arm_watchdog", 500), call(7, "restore_watchdog", 0), call(7, "arm_watchdog", 0.01),
         call(7, "restore_watchdog", 7)])
    add("custom_reply_and_host", [{"id": 3, "options": {"registers": {p.Reg.MST_ID: 0x23}}, "joint": {"reply_id": 0x23}}],
        [call(3, "read_state"), call(3, "read_param", "mst_id")], host_id=0x20)

    # set_id
    add("set_id", [{"id": 3}], [call(3, "set_id", 9), call(3, "read_state"), call(3, "set_id", 3)], callback="direct")
    for echo in ("new", "none"):
        add(f"set_id_echo_{echo}", [{"id": 7, "options": {"esc_echo": echo}}], [call(7, "set_id", 8)], callback="direct")
    add("set_id_save_reply", [{"id": 2, "options": {"save_reply": True}}], [call(2, "set_id", 1)], callback="direct")
    add("set_id_after_power_cycle", [{"id": 3, "options": {"ids_apply_at_once": False}}],
        [call(3, "set_id", 9, 0.02), {"op": "power_cycle", "motor": 3}], callback="direct")
    add("set_id_in_use", [{"id": 3}, {"id": 4}], [call(3, "set_id", 4, 0.02)])
    add("set_id_enabled", [{"id": 3, "options": {"enabled": True}}],
        [call(3, "set_id", 9, 0.02), call(3, "set_id", 16), call(3, "set_id", 3)], callback="direct")
    add("set_id_late_gripper", [{"id": 7, "model": "jhi", "options": {"late": True, "duplicate": True}}],
        [call(7, "set_id", 8)], callback="direct")
    add("set_id_late", [{"id": 7, "model": "jhi", "options": {"late": True}}],
        [call(7, "set_id", 8), call(7, "read_state")], callback="direct")
    add("set_id_high_host", [{"id": 3, "options": {"registers": {p.Reg.MST_ID: 0x7F8}}}], [call(3, "set_id", 9)],
        host_id=0x7F5, callback="direct")
    add("set_id_reply_is_command_id", [{"id": 3}], [call(3, "set_id", 9)], host_id=0x00, callback="direct")

    out.extend(random_scenarios())
    return out


def random_scenarios():
    rng = random.Random(SEED)
    out = []
    timeouts = [0.005, 0.01, 0.02, 0.05, 0.12]
    for n in range(40):
        ids = rng.choice([[7], [6, 7], [1], [1, 2], [3, 4, 5]])
        motors = []
        for i in ids:
            model = rng.choice(["jlo", "j2", "jhi"])
            opts = {"position": round(rng.uniform(-1, 1), 3)}
            for k, prob in (("late", 0.4), ("duplicate", 0.3), ("enabled", 0.2)):
                if rng.random() < prob:
                    opts[k] = True
            if rng.random() < 0.2:
                opts["drop"] = rng.randint(1, 4)
            if rng.random() < 0.15:
                opts["fault"] = rng.choice([0x8, 0xA, 0xD, 0xE])
            if rng.random() < 0.1:
                opts["online"] = False
            motors.append({"id": i, "model": model, "options": opts})
        steps = []
        for _ in range(rng.randint(10, 25)):
            mid = rng.choice(ids)
            r = rng.random()
            if r < 0.25:
                key = rng.choice(["late", "duplicate", "online", "enabled", "drop", "fault", "position"])
                val = {"late": rng.random() < 0.5, "duplicate": rng.random() < 0.5, "online": rng.random() < 0.8,
                       "enabled": rng.random() < 0.5, "drop": rng.randint(0, 5), "fault": rng.choice([0, 0, 0xD, 0x9]),
                       "position": round(rng.uniform(-1.5, 1.5), 4)}[key]
                steps.append({"op": "sim", "motor": mid, "set": {key: val}})
                continue
            if r < 0.3:
                steps.append({"op": "advance", "dt": rng.choice([0.001, 0.0123, 0.3])})
                continue
            t = rng.choice(timeouts)
            a = rng.randint(1, 3)
            method = rng.choice(["check", "read_state", "preload", "enable_confirmed", "disable_confirmed", "poll_status",
                                 "send_command", "enable", "disable", "clear_fault", "set_zero", "read_param",
                                 "write_param", "arm_watchdog", "restore_watchdog", "validate", "health_problem",
                                 "confirms"])
            args = {
                "check": [t, a], "read_state": [t, a], "preload": [round(rng.uniform(-1, 1), 4), t],
                "enable_confirmed": [t], "disable_confirmed": [t, a], "poll_status": [],
                "send_command": [round(rng.uniform(-7, 7), 4), round(rng.uniform(-2, 2), 3), rng.choice([0, 20, 100]),
                                 rng.choice([0, 0.6, 5]), round(rng.uniform(-1, 1), 3)],
                "enable": [], "disable": [], "clear_fault": [], "set_zero": [],
                "read_param": [rng.choice(["timeout", "ctrl_mode", "pmax", "sw_ver", 59]), None, t],
                "write_param": [rng.choice(["timeout", "kp_apr", "damp"]), rng.choice([0, 40, 10_000, 1.25]), None, t,
                                rng.random() < 0.2],
                "arm_watchdog": [rng.choice([500, 1, 0.5]), t], "restore_watchdog": [rng.choice([0, 10_000, None]), t],
                "validate": [t], "health_problem": [], "confirms": [rng.choice(["enabled", "disabled"])],
            }[method]
            steps.append(call(mid, method, *args))
        out.append({"name": f"random_{n}", "motors": motors, "steps": steps})
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    fixture = {"start": START, "scenarios": [run(s) for s in scenarios()]}
    text = json.dumps(fixture, allow_nan=False, separators=(",", ":")) + "\n"
    if a.check:
        if not OUT.exists() or OUT.read_text() != text:
            print(f"stale: {OUT.relative_to(ROOT)}")
            return 1
        return 0
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(text)
    n = sum(len(r["sent"]) for s in fixture["scenarios"] for r in s["records"])
    print(f"wrote {OUT.relative_to(ROOT)} ({len(text) // 1024} KiB, {len(fixture['scenarios'])} scenarios, {n} frames)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
