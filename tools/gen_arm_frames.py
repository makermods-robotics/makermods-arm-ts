#!/usr/bin/env python3
"""Run an arm SDK's real Arm (arm.py) on its simulated bus under a fake clock and record every frame
it sends: contract/fixtures/arm_frames_<product>.json.

    ~/maker-arm-sdk/.venv/bin/python tools/gen_arm_frames.py --product maker
    ~/metal-arm-sdk/.venv/bin/python tools/gen_arm_frames.py --product metal

Each scenario is a list of operations (connect, enable, a control tick with new targets, a fault
injected into a simulated motor, stop(), ...). Joints are named, not numbered. For every operation
the fixture holds the frames the backend was asked to send, in order, as (CAN id, hex data,
extended), and the arm's state, commanded positions, fault reason and the fake time afterwards. The
TypeScript replay (packages/arm-core/test/helpers/arm-frames.ts, called by each arm package's
arm-frames test) runs the same operations on VirtualClock and must produce the identical frames.

Time: `time.monotonic()` / `time.perf_counter()` in the SDK's arm and motor modules return the fake
time and `time.sleep(x)` advances it by exactly x. Threads (the control loop) take turns: one runs
at a time until it sleeps, waits for a reply, joins or waits for control ticks; then the next due
one runs, in deadline order, ties in the order they started waiting. That is how VirtualClock runs
async code, so the interleaving of the control loop with stop(), attach() and move_to() is the same
on both sides. Python's frame spacing (`Arm._busy_wait_us(150)`) becomes a 150 us sleep and
`Arm._wait_ticks` a wait that each tick wakes, as in the TypeScript port; a `threading.Event.wait`
in the motor module (a typed-parameter reply) takes exactly its timeout when no reply comes.
Python polling loops (`while ...: sleep(0.0005)`) still poll: on a path where a reply never comes
they end up to one poll step later than the TypeScript wait, so scenarios with such a path compare
time loosely from that operation on (`loose_time_from`).

Product-specific steps (motor options such as RobStride's `params_supported` or Damiao's `late`
replies, poses, gripper travel) are declared per product in PRODUCTS.
"""

import argparse
import collections
import dataclasses
import heapq
import importlib
import json
import logging
import math
import threading
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
logging.disable(logging.CRITICAL)

# ── deterministic scheduler ──
class Waiter:
    def __init__(self, name):
        self.name = name
        self.go = threading.Event()


class Sched:
    def __init__(self):
        self.t = 0.0
        self.order = 0
        self.timers = []                 # heap of (at, order, waiter)
        self.ready = collections.deque()
        self.conditions = []             # [predicate, waiter, timer entry or None]
        self.current = Waiter("main")
        self.main = self.current
        self.mutex = threading.Lock()

    def _timer(self, at, waiter):
        entry = [at, self.order, waiter, True]
        self.order += 1
        heapq.heappush(self.timers, entry)
        return entry

    def _next(self):
        if self.ready:
            return self.ready.popleft()
        while self.timers:
            entry = heapq.heappop(self.timers)
            if not entry[3]:
                continue
            self.t = max(self.t, entry[0])
            waiter = entry[2]
            for c in list(self.conditions):   # a condition waiter's timeout fired
                if c[1] is waiter:
                    self.conditions.remove(c)
            return waiter
        raise RuntimeError("deadlock: nothing to run")

    def _switch(self, me):
        """Hand the baton to the next runnable waiter and wait until it comes back to `me`."""
        nxt = self._next()
        if nxt is me:
            return
        self.current = nxt
        nxt.go.set()
        me.go.wait()
        me.go.clear()

    def sleep(self, seconds):
        me = self.current
        self._timer(self.t + max(0.0, seconds), me)
        self._switch(me)   # whatever a notification made runnable runs first

    def wait_until(self, predicate, timeout):
        if predicate():
            return True
        me = self.current
        entry = self._timer(self.t + timeout, me) if timeout is not None else None
        self.conditions.append([predicate, me, entry])
        self._switch(me)
        return predicate()

    def notify(self):
        for c in list(self.conditions):
            if c[0]():
                self.conditions.remove(c)
                if c[2] is not None:
                    c[2][3] = False
                self.ready.append(c[1])

    def start(self, waiter, run):
        """A new thread: it runs once the scheduler reaches it, as a timer due now."""
        self._timer(self.t, waiter)

        def body():
            waiter.go.wait()
            waiter.go.clear()
            try:
                run()
            finally:
                self.notify()
                nxt = self._next()
                self.current = nxt
                nxt.go.set()

        threading.Thread(target=body, daemon=True).start()


SCHED = None


class FakeTime:
    @staticmethod
    def monotonic():
        return SCHED.t

    perf_counter = monotonic

    @staticmethod
    def sleep(seconds):
        if seconds < 0:
            raise ValueError("sleep length must be non-negative")
        SCHED.sleep(seconds)


class FakeThread:
    def __init__(self, target=None, args=(), daemon=None, name=None, kwargs=None):
        self._target, self._args, self._kwargs = target, args, kwargs or {}
        self._done = False
        self._started = False

    def start(self):
        self._started = True

        def run():
            try:
                self._target(*self._args, **self._kwargs)
            finally:
                self._done = True
        SCHED.start(Waiter("loop"), run)

    def is_alive(self):
        return self._started and not self._done

    def join(self, timeout=None):
        SCHED.wait_until(lambda: self._done, timeout)


class FakeEvent:
    def __init__(self):
        self._flag = False

    def set(self):
        self._flag = True
        SCHED.notify()

    def clear(self):
        self._flag = False

    def is_set(self):
        return self._flag

    def wait(self, timeout=None):
        return SCHED.wait_until(lambda: self._flag, timeout)



def install(package):
    """Import the product's SDK and put the fake clock and scheduler under it."""
    g = globals()
    arm_module = importlib.import_module(f"{package}.arm")
    motor_module = importlib.import_module(f"{package}.motor")
    g["sim"] = importlib.import_module(f"{package}._sim_motor")
    g["ArmConfig"] = importlib.import_module(f"{package}.config").ArmConfig
    g["DEFAULT_PROFILE"] = importlib.import_module(f"{package}._product").DEFAULT_PROFILE
    Arm = g["Arm"] = arm_module.Arm
    arm_module.time = FakeTime
    motor_module.time = FakeTime
    arm_module.threading = types.SimpleNamespace(
        Thread=FakeThread, current_thread=threading.current_thread, Event=threading.Event, Lock=threading.Lock,
        RLock=threading.RLock)
    motor_module.threading = types.SimpleNamespace(
        Thread=threading.Thread, current_thread=threading.current_thread, Event=FakeEvent, Lock=threading.Lock,
        RLock=threading.RLock, Condition=threading.Condition)
    Arm._busy_wait_us = staticmethod(lambda us: SCHED.sleep(us / 1e6))
    check_health, stop_loop = Arm._check_health, Arm._stop_loop

    def _check_health(self):
        try:
            check_health(self)
        finally:
            SCHED.notify()

    def _wait_ticks(self, n, timeout=1.0):
        target = self._tick_count + n
        SCHED.wait_until(lambda: self._tick_count >= target or not self._running, timeout)
        return self._tick_count >= target

    def _stop_loop(self):
        self._running = False
        SCHED.notify()
        stop_loop(self)

    Arm._check_health = _check_health
    Arm._wait_ticks = _wait_ticks
    Arm._stop_loop = _stop_loop


def shipped(**overrides):
    return dataclasses.replace(ArmConfig.from_yaml(str(DEFAULT_PROFILE)), **overrides)


def run(scenario):
    global SCHED
    SCHED = Sched()
    config = shipped(**scenario.get("overrides", {}))
    labels = [j.label for j in config.joints]
    per_joint = scenario.get("motor_joints", {})
    motors = [sim.for_joint(j, config.host_id, q * j.direction + j.offset,
                            **{**scenario.get("motor", {}), **per_joint.get(j.label, {})})
              for j, q in zip(config.joints, scenario["start"])]
    bus = sim.SimBus(motors)
    frames = []
    send = bus.send

    def recording_send(can_id, data, extended=True):
        frames.append([can_id, bytes(data).hex(), bool(extended)])
        return send(can_id, data, extended)

    bus.send = recording_send
    arm = Arm(config, bus)
    dt = 1.0 / config.control_rate_hz
    steps = []
    for op in scenario["ops"]:
        frames.clear()
        result, error = None, None
        kind = op["op"]
        try:
            if kind == "new_arm":
                arm = Arm(config, bus)
            elif kind == "connect":
                arm.connect(op.get("clear_faults", False), op.get("clear_released", False))
            elif kind == "enable":
                arm.enable(start_loop=op.get("start_loop", True))
            elif kind == "attach":
                arm.attach()
            elif kind == "tick":
                if op.get("targets") is not None:
                    result = arm.set_joint_targets(op["targets"])
                arm._tick(dt)
                SCHED.sleep(dt)
            elif kind == "set_targets":
                result = arm.set_joint_targets(op["targets"])
            elif kind == "sleep":
                SCHED.sleep(op["s"])
            elif kind == "sim":
                setattr(motors[labels.index(op["joint"])], op["attr"], op["value"])
            elif kind == "paced":
                bus.feedback_paced_control = op["value"]
            elif kind == "sim_watchdog":
                sim.watchdog_timeout(motors[labels.index(op["joint"])])
            elif kind == "stop":
                result = arm.stop()
            elif kind == "return_to_rest":
                result = arm.return_to_rest(op["rest"])
            elif kind == "move_to":
                result = arm.move_to(op["q"], speed=op.get("speed"), timeout=op.get("timeout"))
            elif kind == "hold":
                result = arm.hold_current_position()
            elif kind == "set_gains":
                arm.set_gains(op.get("kp"), op.get("kd"))
            elif kind == "float_mode":
                arm.float_mode(op.get("kd"))
            elif kind in ("disable", "estop", "detach", "disconnect", "clear_faults"):
                result = getattr(arm, kind)()
            else:
                raise KeyError(kind)
        except Exception as e:   # noqa: BLE001 -- recorded for the TypeScript side to reproduce
            error = [type(e).__name__, str(e)]
        if isinstance(result, dict):
            result = sorted(result.items())
        steps.append({"frames": list(frames), "result": result, "error": error, "state": arm.state.value,
                      "commanded": arm.get_commanded_positions(), "fault_reason": arm.fault_reason,
                      "t": SCHED.t})
    return steps


# ── scenarios ──
# Per product: poses (rad, joint order of the shipped profile), sine amplitudes, gripper travel
# [closed, open] for its sine, joints the scenarios address, and the motor options that model a
# product-specific field behaviour.
PRODUCTS = {
    "maker": {
        "package": "maker_arm",
        "start": [0.0, -0.5, 0.8, 0.1, 0.0, 0.0, -0.5],
        "amplitude": [0.4, 0.3, 0.35, 0.3, 0.4, 0.5, 0.0],
        "gripper_sine": [-0.3, -1.5],
        "gripper_close": -0.05, "gripper_open": -1.4,
        "far": [0.5, -0.9, 1.3, 0.4, -0.3, 0.6, -1.2],
        "move": [0.3, -0.8, 1.0, 0.2, 0.1, -0.2, -0.6],
        "rest": [0.0, -0.3, 0.4, 0.0, 0.0, 0.0, -0.5],
        "gains": {"kp": [30.0, 80.0, 50.0, 15.0, 15.0, 15.0, 10.0], "kd": [2.0, 3.0, 2.0, 1.0, 1.0, 1.0, 0.3]},
        "variants": {"old_firmware": {"motor": {"params_supported": False}}},
    },
    "metal": {
        "package": "metal_arm",
        "start": [0.0, -0.6, 0.9, 0.1, 0.0, 0.0, 0.5],
        "amplitude": [0.4, 0.3, 0.35, 0.3, 0.4, 0.5, 0.0],
        "gripper_sine": [0.3, 1.8],
        "gripper_close": 0.05, "gripper_open": 2.0,
        "far": [0.5, -1.0, 1.4, 0.4, -0.3, 0.6, 1.5],
        "move": [0.3, -0.9, 1.1, 0.2, 0.1, -0.2, 0.8],
        "rest": [0.0, -0.4, 0.5, 0.0, 0.0, 0.0, 0.5],
        "gains": {"kp": [50.0, 60.0, 60.0, 30.0, 10.0, 10.0, 5.0], "kd": [3.0, 3.0, 3.0, 2.0, 0.4, 0.4, 0.3]},
        # The held reply to the gripper's first disable makes Arm._wait_reply time out (a polling loop):
        # time compares loosely from that disable on.
        "variants": {"late_gripper": {"motor_joints": {"gripper": {"late": True}}, "loose_from_op": "disable"},
                     "late_duplicate_gripper": {"motor_joints": {"gripper": {"late": True, "duplicate": True}},
                                                "loose_from_op": "disable"}},
    },
}


def build(product, gravity):
    P = PRODUCTS[product]
    START, AMP = P["start"], P["amplitude"]
    closed, opened = P["gripper_sine"]

    def sine_targets(k, rate=200.0):
        t = k / rate
        q = [s + a * math.sin(2 * math.pi * 0.5 * t + i) for i, (s, a) in enumerate(zip(START, AMP))]
        q[6] = (closed + opened) / 2 + (opened - closed) / 2 * math.sin(2 * math.pi * 0.7 * t)
        return q

    def ticks(first, count, targets=True, jump_at=None):
        ops = []
        for k in range(first, first + count):
            q = sine_targets(k) if targets else None
            if q is not None and jump_at is not None and k >= jump_at:
                q[0] += 1.2   # a target jump: rate-limited by max_velocity
            ops.append({"op": "tick", "targets": q})
        return ops

    def scenario(name, ops, overrides, loose_from_op=None, **extra):
        s = {"name": name, "start": START, "overrides": overrides, "ops": ops, **extra}
        if loose_from_op is not None:
            s["loose_time_from"] = next(i for i, op in enumerate(ops) if op["op"] == loose_from_op)
        return s

    def sine(name, ov, **extra):
        ops = [{"op": "connect"}, {"op": "enable", "start_loop": False}]
        ops += [{"op": "tick", "targets": None} for _ in range(3)]
        ops += ticks(0, 220, jump_at=120)
        ops += [{"op": "disable"}, {"op": "disconnect"}]
        return scenario(name, ops, ov, **extra)

    def fault(name, inject, ov, after=60, loose=False):
        ops = [{"op": "connect"}, {"op": "enable", "start_loop": False}] + ticks(0, 30) + inject + ticks(30, after)
        ops += [{"op": "disable"}, {"op": "clear_faults"}, {"op": "disconnect"}]
        return scenario(name, ops, ov, "disable" if loose else None)

    out = []
    for g in ([False, True] if gravity else [False]):
        tag = "gravity" if g else "nogravity"
        ov = {} if g else {"gravity_scale": 0.0}
        out.append(sine(f"sine_{tag}", ov))
        for variant, options in P["variants"].items():
            out.append(sine(f"{variant}_{tag}", ov, **options))
        out.append(fault(f"offline_{tag}", [{"op": "sim", "joint": "elbow_flex", "attr": "online", "value": False}], ov,
                         loose=True))
        # a joint silent for longer than gravity's stale limit but shorter than the feedback timeout:
        # gravity falls back to the commanded pose, then returns to the measured one
        stale = [{"op": "connect"}, {"op": "enable", "start_loop": False}] + ticks(0, 40)
        stale += [{"op": "sim", "joint": "wrist_flex", "attr": "online", "value": False}] + ticks(40, 28)
        stale += [{"op": "sim", "joint": "wrist_flex", "attr": "online", "value": True}] + ticks(68, 30)
        stale += [{"op": "disable"}, {"op": "disconnect"}]
        out.append(scenario(f"stale_feedback_{tag}", stale, ov))
        out.append(fault(f"overtemp_{tag}", [{"op": "sim", "joint": "gripper", "attr": "temperature", "value": 60.0}], ov))
        out.append(fault(f"fault_joint_{tag}", [{"op": "sim", "joint": "wrist_flex", "attr": "faulted", "value": True}], ov))
        out.append(fault(f"fault_gripper_{tag}", [{"op": "sim", "joint": "gripper", "attr": "faulted", "value": True}], ov))
        if sim.WARNING_SUPPORTED:
            out.append(fault(f"warning_gripper_{tag}", [{"op": "sim", "joint": "gripper", "attr": "warning", "value": True}],
                             ov))
        # hold, gains, float mode
        ops = [{"op": "connect"}, {"op": "enable", "start_loop": False}] + ticks(0, 30)
        ops += [{"op": "hold"}] + ticks(30, 10, targets=False)
        ops += [{"op": "set_gains", **P["gains"]}] + ticks(40, 20)
        ops += [{"op": "set_gains"}] + ticks(60, 10)
        ops += [{"op": "float_mode", "kd": 0.2}] + ticks(70, 5, targets=False)
        ops += [{"op": "sim", "joint": "shoulder_pan", "attr": "position", "value": 0.35},
                {"op": "sim", "joint": "wrist_flex", "attr": "position", "value": 0.4}] + ticks(75, 5, targets=False)
        ops += [{"op": "set_gains"}] + ticks(80, 10, targets=False)
        ops += [{"op": "disable"}, {"op": "disconnect"}]
        out.append(scenario(f"hold_gains_float_{tag}", ops, ov))
        # stop(): return to rest with the control loop running
        ops = [{"op": "connect"}, {"op": "enable", "start_loop": True}, {"op": "sleep", "s": 0.1},
               {"op": "set_targets", "targets": P["far"]}, {"op": "sleep", "s": 0.6},
               {"op": "stop"}, {"op": "disconnect"}]
        out.append(scenario(f"stop_{tag}", ops, ov))
        # stop() refused (a joint blocked), then e-stop
        nudged = list(START)
        nudged[0] = 0.5
        ops = [{"op": "connect"}, {"op": "enable", "start_loop": True}, {"op": "sleep", "s": 0.05},
               {"op": "set_targets", "targets": nudged}, {"op": "sleep", "s": 0.3},
               {"op": "sim", "joint": "shoulder_pan", "attr": "track", "value": False},
               {"op": "stop"}, {"op": "sleep", "s": 0.05}, {"op": "estop"}, {"op": "disconnect"}]
        out.append(scenario(f"stop_blocked_estop_{tag}", ops, dict(ov, rest_timeout=1.0)))
        # attach(): a previous session left the arm holding and its gripper watchdog released it
        ops = [{"op": "connect"}, {"op": "enable", "start_loop": True},
               {"op": "move_to", "q": P["move"], "speed": 3.0, "timeout": 5.0},
               {"op": "detach"}, {"op": "sim_watchdog", "joint": "gripper"}, {"op": "new_arm"},
               {"op": "connect", "clear_released": True}, {"op": "attach"}, {"op": "sleep", "s": 0.2},
               {"op": "stop"}, {"op": "return_to_rest", "rest": P["rest"]}, {"op": "disable"}, {"op": "disconnect"}]
        out.append(scenario(f"attach_{tag}", ops, ov))
        # the gripper closes on an object (blocked): closing effort capped, Kp on the protocol's grid;
        # then a feedback-paced bus (old USB-SLCAN firmware) waits for each reply before the next frame
        ops = [{"op": "connect"}, {"op": "enable", "start_loop": False}] + ticks(0, 10)
        ops += [{"op": "sim", "joint": "gripper", "attr": "track", "value": False}]
        ops += [{"op": "tick", "targets": START[:6] + [P["gripper_close"]]} for _ in range(20)]
        ops += [{"op": "tick", "targets": START[:6] + [P["gripper_open"]]} for _ in range(5)]
        ops += [{"op": "paced", "value": True}] + ticks(10, 20)
        ops += [{"op": "disable"}, {"op": "disconnect"}]
        out.append(scenario(f"gripper_blocked_paced_{tag}", ops, ov))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--product", choices=sorted(PRODUCTS), required=True)
    ap.add_argument("--no-gravity", action="store_true", help="only the gravity_scale 0 scenarios")
    ap.add_argument("--out", help="default: contract/fixtures/arm_frames_<product>.json")
    a = ap.parse_args()
    install(PRODUCTS[a.product]["package"])
    out = Path(a.out) if a.out else ROOT / "contract" / "fixtures" / f"arm_frames_{a.product}.json"
    fixture = {"source": f"{PRODUCTS[a.product]['package']}.arm", "product": a.product,
               "control_rate_hz": shipped().control_rate_hz, "scenarios": []}
    total = 0
    for s in build(a.product, not a.no_gravity):
        s = dict(s)
        s["steps"] = run(s)
        fixture["scenarios"].append(s)
        frames = sum(len(step["frames"]) for step in s["steps"])
        total += frames
        errors = [(i, step["error"]) for i, step in enumerate(s["steps"]) if step["error"]]
        print(f"{s['name']}: {len(s['steps'])} steps, {frames} frames, final {s['steps'][-1]['state']}"
              + (f", errors {errors}" if errors else ""))
    out.write_text(json.dumps(fixture, separators=(",", ":")) + "\n")
    print(f"wrote {out}: {len(fixture['scenarios'])} scenarios, {total} frames")


if __name__ == "__main__":
    main()
