#!/usr/bin/env python3
"""Record the Python SLCAN transport's behaviour for the TypeScript port's contract test.

    ~/maker-arm-sdk/.venv/bin/python tools/gen_slcan.py     # writes contract/fixtures/slcan.json

Runs maker_arm.transport.slcan_serial against an in-memory fake serial port and a virtual clock (no
serial port is ever opened): slcan_encode over every payload length and id boundary, refusals
included; SlcanFrameParser over hand-written and generated streams fed in chunks (malformed records,
Python int()/bytes.fromhex() leniency, garbage bounding); the backend's constructor refusals, the
bytes of open, send and close, and the pacing sleeps between sends.
"""

import json
import random
import subprocess
import sys
from pathlib import Path

import serial

from maker_arm.transport import slcan_serial
from maker_arm.transport.slcan_serial import SlcanFrameParser, SlcanSerialBackend, slcan_encode

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "contract" / "fixtures" / "slcan.json"
SDK = Path(slcan_serial.__file__).resolve().parents[2]


def encodes() -> list:
    rng = random.Random(1)
    cases = []
    ids = [(0, False), (1, False), (0x123, False), (0x7FF, False), (0x800, False), (-1, False),
           (0, True), (1, True), (0x7FF, True), (0x800, True), (0x0400FD01, True), (0x1FFFFFFF, True),
           (0x20000000, True), (-1, True)]
    for can_id, ext in ids:
        for n in range(0, 10):
            data = bytes(rng.randrange(256) for _ in range(n))
            cases.append(_encode(can_id, data, ext))
    for _ in range(200):
        ext = rng.random() < 0.5
        can_id = rng.randrange(0x20000000 if ext else 0x800)
        data = bytes(rng.randrange(256) for _ in range(rng.randrange(9)))
        cases.append(_encode(can_id, data, ext))
    cases.append(_encode(0x0400FD01, bytes(8), True))
    cases.append(_encode(0x123, b"\xAB\xCD", False))
    return cases


def _encode(can_id: int, data: bytes, ext: bool) -> dict:
    case = {"id": can_id, "extended": ext, "data": data.hex()}
    try:
        case["out"] = slcan_encode(can_id, data, ext).decode("ascii")
    except ValueError as e:
        case["error"] = str(e)
    return case


def _feed(chunks: list) -> dict:
    parser = SlcanFrameParser()
    feeds = []
    for chunk in chunks:
        frames = parser.feed(chunk)
        feeds.append({"frames": [[int(cid), bool(cid.extended), data.hex()] for cid, data in frames],
                      "buffered": len(parser._buf)})
    return {"chunks": [c.hex() for c in chunks], "feeds": feeds}


def _split(stream: bytes, rng: random.Random) -> list:
    chunks, i = [], 0
    while i < len(stream):
        n = rng.choice([1, 1, 2, 3, 5, 8, 13, 19, 30, 64])
        chunks.append(stream[i:i + n])
        i += n
    return chunks or [b""]


HAND = [
    # the Python unit tests' streams
    b"\rgarbage\r" + slcan_encode(0x0400FD01, bytes(8)) + slcan_encode(0x028001FD, bytes.fromhex("8000800080000159"))[:-1] + b"1234\r",
    b"T00000001900\rTZZZZZZZZ1AA\r" + slcan_encode(0x101, b"\x01"),
    # adapter replies and status lines
    b"\r", b"\x07", b"\x07\r", b"z\r", b"Z\r", b"V1013\r", b"N1234\r", b"F00\r", b"\x07t1230\r",
    b"t1230\r\x07t1230\r", b"t1230\n\r", b"\nt1230\r", b"t123\r", b"t12\r", b"T1234567\r", b"T12345678\r",
    b"T123456780\r", b"t1239\r", b"t123F\r", b"t1231\r", b"t1231A\r", b"t1231AB\r", b"t1231ABC\r",
    b"t1230FFFF\r", b"r1230\r", b"R123456780\r", b"d1230\r", b"b1230\r",
    # Python int() and bytes.fromhex() leniency
    b"t 1F0\r", b"t1_F0\r", b"t0x10\r", b"t-010\r", b"t+010\r", b"T-00000011AB\r", b"T0x0000011AB\r",
    b"T0000_0011AB\r", b"t__10\r", b"t0X10\r", b"t1 F0\r", b"t\x0b1F0\r", b"t\x1c1F0\r", b"t12 0\r",
    b"t1232 ABC\r", b"t1232AB \r", b"t1232 AB  \r", b"t1232AB\x0bC\r", b"t1232A BC\r", b"t123+AB\r",
    b"t1232abcd\r", b"t1232\xab\xcd\r", b"t\xff\xff\xff0\r", b"T0000001\xe98" + b"00" * 8 + b"\r",
    b"tfff0\r", b"t8001AA\r", b"TFFFFFFFF0\r", b"T1FFFFFFF8" + b"FF" * 8 + b"\r",
    # garbage without CR is bounded to the last 32 bytes once it passes 256
    b"x" * 256, b"x" * 257, b"x" * 300 + slcan_encode(0x123, b"\x01"),
    bytes(range(1, 13)) * 30 + b"\rt1230\r",
]


def parses() -> list:
    rng = random.Random(7)
    cases = []
    for stream in HAND:
        cases.append(_feed([stream]))
        cases.append(_feed(_split(stream, rng)))
    # garbage bound across feeds
    cases.append(_feed([b"y" * 200, b"y" * 60, b"y" * 10, b"t1230\r"]))
    cases.append(_feed([b"t12", b"3", b"2A", b"B", b"CD\rT0400F", b"D018" + b"00" * 8 + b"\r"]))
    # generated streams: valid records mixed with noise, split at random points
    alphabet = b"tT0123456789ABCDEFabcdefxX_+- \t\x0b\x07\nZr"
    for _ in range(300):
        parts = []
        for _ in range(rng.randrange(1, 8)):
            pick = rng.random()
            if pick < 0.45:
                ext = rng.random() < 0.5
                cid = rng.randrange(0x20000000 if ext else 0x800)
                rec = slcan_encode(cid, bytes(rng.randrange(256) for _ in range(rng.randrange(9))), ext)
                if rng.random() < 0.3:  # mutate one byte
                    i = rng.randrange(len(rec) - 1)
                    rec = rec[:i] + bytes([rng.choice(alphabet)]) + rec[i + 1:]
                if rng.random() < 0.2:  # timestamp
                    rec = rec[:-1] + b"%04X\r" % rng.randrange(0x10000)
                parts.append(rec)
            elif pick < 0.8:
                parts.append(bytes(rng.choice(alphabet) for _ in range(rng.randrange(0, 30))) + b"\r")
            else:
                parts.append(bytes(rng.choice(alphabet) for _ in range(rng.randrange(0, 12))))
        stream = b"".join(parts)
        cases.append(_feed(_split(stream, rng)))
    return cases


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.writes = []
        self.in_waiting = 0
        self.resets = 0
        self.closed = False

    def reset_input_buffer(self):
        self.resets += 1

    def write(self, data):
        self.writes.append([CLOCK.now, data.decode("ascii")])
        CLOCK.now += CLOCK.write_cost

    def flush(self):
        pass

    def read(self, _size):
        CLOCK.real_sleep()
        return b""

    def close(self):
        self.closed = True


class Clock:
    """Virtual time for the backend's time.monotonic() and time.sleep()."""

    def __init__(self):
        self.now = 0.0
        self.sleeps = []
        self.write_cost = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now += seconds

    @staticmethod
    def real_sleep():
        import time
        time.sleep(0.001)


CLOCK = Clock()


def _backend(**kwargs):
    global CLOCK
    CLOCK = Clock()
    slcan_serial.time = CLOCK
    opened = {}

    def open_serial(*args, **kw):
        opened["port"] = FakeSerial(*args, **kw)
        return opened["port"]

    serial.Serial = open_serial
    backend = SlcanSerialBackend(**kwargs)
    return backend, opened


def lifecycles() -> list:
    out = []
    combos = [dict(startup_delay=0), dict(), dict(auto_retransmit=True), dict(bitrate=500_000, startup_delay=0.5),
              dict(bitrate=10_000, baudrate=2_000_000, write_timeout=0.25)]
    for b in sorted(SlcanSerialBackend.BITRATE_COMMANDS):
        combos.append(dict(bitrate=b, startup_delay=0))
    for kwargs in combos:
        backend, opened = _backend(**kwargs)
        backend.open()
        port = opened["port"]
        # sends: right after open, back to back, after a gap, after a partial gap
        backend.send(0x0400FD01, bytes(8))
        backend.send(0x123, b"\xAB\xCD", extended=False)
        CLOCK.now += 0.01
        backend.send(0x7FF, b"", extended=False)
        CLOCK.now += 0.0005
        backend.send(0x1FFFFFFF, bytes(range(8)))
        backend.close()
        out.append({
            "kwargs": kwargs,
            "serial_args": list(port.args), "serial_kwargs": port.kwargs, "resets": port.resets,
            "writes": port.writes, "sleeps": CLOCK.sleeps,
            "feedback_paced_control": backend.feedback_paced_control,
            "feedback_wait_timeout": backend.feedback_wait_timeout,
        })
    return out


def pacing() -> dict:
    """Sleeps requested for sends at given gaps, with a write that itself takes time."""
    backend, opened = _backend(startup_delay=0)
    backend.open()
    CLOCK.write_cost = 0.0002
    gaps = [0.0, 0.0, 0.001, 0.0015, 0.002, 0.0, 0.0001, 0.05, 0.0]
    record = []
    for gap in gaps:
        CLOCK.now += gap
        before = len(CLOCK.sleeps)
        start = CLOCK.now
        backend.send(0x10, b"\x00", extended=False)
        record.append({"gap": gap, "start": start, "sleeps": CLOCK.sleeps[before:], "written_at": opened["port"].writes[-1][0]})
    backend.close()
    out = {"write_cost": 0.0002, "sends": record}
    for kwargs in (dict(tx_interval=0.0), dict(tx_interval=0.004)):
        backend, opened = _backend(startup_delay=0, **kwargs)
        backend.open()
        CLOCK.write_cost = 0.0
        for _ in range(3):
            backend.send(0x10, b"", extended=False)
        backend.close()
        out[f"tx_interval_{kwargs['tx_interval']}"] = {"sleeps": CLOCK.sleeps, "writes": opened["port"].writes}
    return out


def refusals() -> list:
    out = []
    for kwargs in (dict(bitrate=83_333), dict(bitrate=0), dict(tx_interval=-0.001), dict(write_timeout=0),
                   dict(write_timeout=-1)):
        try:
            SlcanSerialBackend(**kwargs)
        except ValueError as e:
            out.append({"kwargs": kwargs, "error": str(e)})
        else:
            raise AssertionError(kwargs)
    backend = SlcanSerialBackend()
    try:
        backend.send(1, b"")
    except RuntimeError as e:
        out.append({"send_closed": str(e)})
    return out


def main() -> int:
    commit = subprocess.run(["git", "-C", str(SDK), "rev-parse", "--short", "HEAD"], capture_output=True,
                            text=True).stdout.strip()
    defaults = SlcanSerialBackend()
    data = {
        "meta": {"generated_by": "tools/gen_slcan.py", "source": "maker_arm/transport/slcan_serial.py",
                 "sdk_commit": commit, "python": sys.version.split()[0]},
        "constants": {"bitrate_commands": {str(k): v for k, v in SlcanSerialBackend.BITRATE_COMMANDS.items()},
                      "defaults": {"port": defaults._port, "baudrate": defaults._baud, "bitrate": defaults._bitrate,
                                   "rtscts": defaults._rtscts, "startup_delay": defaults._startup_delay,
                                   "auto_retransmit": defaults._auto_retransmit,
                                   "tx_interval": defaults._tx_interval, "write_timeout": defaults._write_timeout,
                                   "feedback_paced_control": defaults.feedback_paced_control,
                                   "feedback_wait_timeout": defaults.feedback_wait_timeout}},
        "encode": encodes(),
        "parse": parses(),
        "lifecycle": lifecycles(),
        "pacing": pacing(),
        "refusals": refusals(),
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(data, indent=1) + "\n")
    print(f"wrote {OUT.relative_to(ROOT)}: {len(data['encode'])} encodes, {len(data['parse'])} parse streams, "
          f"{len(data['lifecycle'])} lifecycles")
    return 0


if __name__ == "__main__":
    sys.exit(main())
