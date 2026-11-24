#!/usr/bin/env python3
"""Drive the Python StarLeader, on the real motorbridge-smart-servo library, against simulated
FashionStar servos behind a pseudo-terminal, and record every byte the library writes, every byte
the servos answer and every result, to contract/fixtures/teleop_star.json.

    ~/maker-arm-sdk/.venv/bin/python tools/gen_teleop_star.py            # write
    ~/maker-arm-sdk/.venv/bin/python tools/gen_teleop_star.py --check    # exit 1 if stale

No device is opened: the library gets the slave side of a pty, and a thread on the master side plays
the servos with the model packages/arm-core/src/leaders/sim.ts uses (reported angle = physical -
origin - 360 * turns; set origin zeroes it; the multi-turn reset drops whole turns; monitor replies
carry 12000 mV and zeros besides the angle). Each step records the bytes written during it, the
replies, and its result (the follower targets, null, or the error). The run takes about 15 s of
real time: a silent servo costs the library's 100 ms reply timeout per read.
"""

import argparse
import json
import math
import os
import pty
import select
import struct
import sys
import threading
import time
import tty
from pathlib import Path

from maker_arm.leaders import LeaderError, StarLeader
from maker_arm.mapping import JointMapper

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "contract" / "fixtures" / "teleop_star.json"
HOME = Path.home()
MAPPINGS = {
    "maker": HOME / "maker-arm-sdk" / "maker_arm" / "profiles" / "star_to_maker_v1.json",
    "metal": HOME / "metal-arm-sdk" / "metal_arm" / "profiles" / "star_to_metal_v1.json",
}
DRAIN_S = 0.03   # after each step, for the servo thread to take in every byte written


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


class Servo:
    def __init__(self, sid):
        self.id = sid
        self.physical = 0.0
        self.origin = 0.0
        self.turns = 0
        self.online = True
        self.stop_mode = None

    @property
    def angle(self):
        return self.physical - self.origin - 360 * self.turns

    @angle.setter
    def angle(self, deg):
        self.physical = deg + self.origin + 360 * self.turns

    def monitor(self):
        return struct.pack("<BHHHHBih", self.id, 12000, 0, 0, 0, 0, int(round(self.angle * 10)), 0)


class ServoBus:
    """The servo side of the pty."""

    def __init__(self, ids):
        self.master, slave = pty.openpty()
        tty.setraw(self.master)
        tty.setraw(slave)
        self.port = os.ttyname(slave)
        self.slave = slave
        self.servos = {sid: Servo(sid) for sid in ids}
        self.written = bytearray()
        self.replies = bytearray()
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self.serve, daemon=True).start()

    def take(self):
        time.sleep(DRAIN_S)
        with self.lock:
            out = (bytes(self.written).hex(), bytes(self.replies).hex())
            self.written.clear()
            self.replies.clear()
        return out

    def reply(self, code, params):
        packet = bytes([0x05, 0x1C, code, len(params)]) + params
        return packet + bytes([checksum(packet)])

    def live(self, sid):
        s = self.servos.get(sid)
        return s if s is not None and s.online else None

    def serve(self):
        buf = b""
        while self.running:
            r, _, _ = select.select([self.master], [], [], 0.2)
            if not r:
                continue
            try:
                data = os.read(self.master, 4096)
            except OSError:
                return
            with self.lock:
                self.written += data
            buf += data
            while len(buf) >= 5:
                if buf[:2] != b"\x12\x4c":
                    buf = buf[1:]
                    continue
                n = buf[3]
                if len(buf) < 5 + n:
                    break
                code, params, ok = buf[2], buf[4:4 + n], buf[4 + n] == checksum(buf[:4 + n])
                buf = buf[5 + n:]
                if not ok:
                    continue
                out = b""
                if code == 1 and self.live(params[0]):
                    out = self.reply(1, params[:1])
                elif code == 24 and self.live(params[0]):
                    self.live(params[0]).stop_mode = params[1]
                elif code == 17 and self.live(params[0]):
                    s = self.live(params[0])
                    s.turns = round((s.physical - s.origin) / 360)
                elif code == 23 and self.live(params[0]):
                    s = self.live(params[0])
                    s.origin, s.turns = s.physical, 0
                elif code == 25 and params[0] == 22:
                    for sid in params[3:3 + params[2]]:
                        if self.live(sid):
                            out += self.reply(22, self.live(sid).monitor())
                if out:
                    with self.lock:
                        self.replies += out
                    os.write(self.master, out)

    def close(self):
        self.running = False
        os.close(self.slave)


def enc(x):
    if isinstance(x, float) and not math.isfinite(x):
        return "nan" if math.isnan(x) else ("inf" if x > 0 else "-inf")
    if isinstance(x, list):
        return [enc(v) for v in x]
    return x


def run(label, mapping_path, script, miss_limit=50, ids=range(7)):
    joints = json.loads(Path(mapping_path).read_text())["joints"]
    bus = ServoBus(ids)
    leader = StarLeader(bus.port, JointMapper(joints), miss_limit=miss_limit)
    steps = []
    try:
        for action, *args in script:
            if action == "angles":
                for sid, deg in args[0].items():
                    bus.servos[sid].angle = deg
                steps.append({"action": action, "angles": [[s, d] for s, d in args[0].items()]})
                continue
            if action == "online":
                for sid in args[0]:
                    bus.servos[sid].online = args[1]
                steps.append({"action": action, "ids": list(args[0]), "online": args[1]})
                continue
            result, error = None, None
            try:
                result = getattr(leader, action)()
            except LeaderError as e:
                error = ["LeaderError", str(e).replace(bus.port, "<port>")]
            written, replies = bus.take()
            step = {"action": action, "written": written, "replies": replies, "result": enc(result), "error": error}
            if action == "read":
                step["misses"] = [[s, n] for s, n in leader.misses.items()]
            steps.append(step)
    finally:
        leader.close()
        bus.close()
    return {"label": label, "joints": joints, "miss_limit": miss_limit, "ids": list(ids), "steps": steps}


def scripts():
    session = [
        ("connect",),
        ("read",),                                                          # every servo at 0
        ("angles", {0: 30.0, 1: -45.5, 2: 90.0, 3: 12.3, 4: -7.77, 5: 179.95, 6: -20.0}),
        ("read",),
        ("angles", {0: 400.0, 1: -500.25, 2: 725.0, 3: -0.05, 4: 0.15, 5: -181.0, 6: 33.333}),
        ("read",),                                                          # near-zero readings are held
        ("read",),
        ("online", [2], False),
    ] + [("read",)] * 21 + [                                                # held, then the bus loss limit
        ("online", [2], True),
        ("read",),
        ("angles", {0: 5.0, 1: -15.0, 2: 45.0, 3: 20.0, 4: -30.0, 5: 60.0, 6: -12.5}),
        ("read",),
        ("zero",),
    ] + [("read",)] * 32 + [                                                # zero readings held for 30 samples
        ("angles", {0: 1.25, 1: -2.5, 2: 3.75, 3: 361.0, 4: -722.5, 5: 0.25, 6: 10.0}),
        ("read",),
        ("online", [0, 1, 2, 3, 4, 5, 6], False),
    ] + [("read",)] * 22                                                    # nothing answers: bus errors, then lost
    return [
        ("maker session", MAPPINGS["maker"], session, 3),
        ("metal session", MAPPINGS["metal"], session, 3),
    ]


def build():
    cases = [run(label, path, script, miss_limit) for label, path, script, miss_limit in scripts()]
    # A servo that does not answer its ping: connect refuses, after closing the port.
    joints_path = MAPPINGS["maker"]
    bus = ServoBus([0, 1, 3, 4, 5, 6])
    leader = StarLeader(bus.port, JointMapper(json.loads(joints_path.read_text())["joints"]))
    try:
        leader.connect()
        error = None
    except LeaderError as e:
        error = ["LeaderError", str(e).replace(bus.port, "<port>")]
    written, replies = bus.take()
    bus.close()
    cases.append({"label": "silent servo at connect", "joints": json.loads(joints_path.read_text())["joints"],
                  "miss_limit": 50, "ids": [0, 1, 3, 4, 5, 6],
                  "steps": [{"action": "connect", "written": written, "replies": replies, "result": None, "error": error}]})
    return {"generator": "tools/gen_teleop_star.py", "library": "motorbridge-smart-servo 0.0.4", "cases": cases}


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
