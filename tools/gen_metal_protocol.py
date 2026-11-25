#!/usr/bin/env python3
"""Drive every encoder and decoder of the Metal Arm's Damiao codec over random and edge-case inputs
and write the results to contract/fixtures/metal_protocol.json for packages/metal-arm.

    ~/metal-arm-sdk/.venv/bin/python tools/gen_metal_protocol.py            # write
    ~/metal-arm-sdk/.venv/bin/python tools/gen_metal_protocol.py --check    # exit 1 if stale

Floats are written as JSON numbers (shortest round-trip repr) except NaN / ±inf, written as the
strings "nan", "inf", "-inf". Bytes are hex strings. A call that raises is {"error": <class>, "message": ...}.
"""

import argparse
import json
import math
import os
import random
import struct
import sys
from pathlib import Path

# fakes.py (classify, feedback_bytes); METAL_ARM_SDK_DIR names another checkout of the SDK
sys.path.insert(0, str(Path(os.environ.get("METAL_ARM_SDK_DIR", Path.home() / "metal-arm-sdk")) / "tests"))

from metal_arm import _sim_motor as sim  # noqa: E402
from metal_arm import motor as m  # noqa: E402
from metal_arm import protocol as p  # noqa: E402
from metal_arm.transport.base import CanId  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "contract" / "fixtures" / "metal_protocol.json"
SEED = 20261122

MIT_GOLDEN = [
    ("jlo", 100.0, 5.0, 0.0, 0.0, 0.0, "7fff7ff333fff7ff"),
    ("jlo", 100.0, 5.0, 45.0, 30.0, 1.5, "900186a333fff865"),
    ("j2", 100.0, 5.0, -90.0, -10.0, -20.0, "5ffb7db333fff6aa"),
    ("jhi", 20.0, 0.6, 120.0, 200.0, 0.3, "aaaf8ed0a31eb81e"),
    ("jhi", 0.0, 0.0, -400.0, -9999.0, -99.0, "0000000000000000"),
    ("jlo", 500.0, 5.0, 359.0, 572.0, 30.0, "ffb4ffbfffffffff"),
]
STATE_GOLDEN = [
    ("jlo", "1180007ff7ff1e20", 1, 1, 0.005490463040242452, -0.13991643348738003, -0.0073260073260073, 30, 32),
    ("j2", "02a3d71234567890", 0, 2, 100.7554872514522, -491.5264308411677, -54.94505494505495, 120, 144),
    ("jhi", "d7ffff0000002d2f", 0xD, 7, 359.817495342157, -1718.8733853924696, -20.0, 45, 47),
]


def f(x):
    """A float for JSON."""
    if isinstance(x, bool) or x is None:
        return x
    if isinstance(x, float):
        if math.isnan(x):
            return "nan"
        if math.isinf(x):
            return "inf" if x > 0 else "-inf"
    return x


def frame(fr):
    cid, data = fr
    return [cid, bytes(data).hex()]


def call(fn, *args):
    try:
        return {"ok": fn(*args)}
    except Exception as e:   # noqa: BLE001
        return {"error": type(e).__name__, "message": str(e)}


def decoded(msg):
    if msg is None:
        return None
    if isinstance(msg, p.StateFrame):
        return {"type": "state", "motor_id": msg.motor_id, "status": msg.status, "position": f(msg.position),
                "velocity": f(msg.velocity), "torque": f(msg.torque), "t_mos": f(msg.t_mos), "t_rotor": f(msg.t_rotor)}
    return {"type": "register", "motor_id": msg.motor_id, "op": msg.op, "rid": msg.rid, "raw": msg.raw.hex()}


def finish(result, conv):
    if "ok" in result:
        result["ok"] = conv(result["ok"])
    return result


def floats(rng, scale, n):
    """Random and edge-case floats around a range of half-width `scale`."""
    edge = [0.0, -0.0, scale, -scale, scale * (1 + 1e-12), -scale * (1 + 1e-12), scale * 1.5, -scale * 1.5,
            1e-300, -1e-300, 5e-324, 1e300, -1e300, scale / 2, -scale / 2, scale / 3, 0.1, -0.1, 1.0, -1.0,
            math.nextafter(scale, 0.0), math.nextafter(-scale, 0.0), math.nextafter(scale, math.inf)]
    out = list(edge)
    for _ in range(n):
        kind = rng.random()
        if kind < 0.6:
            out.append(rng.uniform(-scale * 1.2, scale * 1.2))
        elif kind < 0.8:
            out.append(round(rng.uniform(-scale, scale), rng.randint(0, 4)))
        else:
            out.append(rng.choice([-1, 1]) * 10 ** rng.uniform(-8, 4))
    return out


def gen():
    rng = random.Random(SEED)
    fx = {"seed": SEED, "models": {k: [v.p_max, v.v_max, v.t_max] for k, v in p.MODELS.items()}}

    fx["mit_golden"] = [list(g) for g in MIT_GOLDEN]
    fx["state_golden"] = [list(g) for g in STATE_GOLDEN]

    # float_to_uint / uint_to_float on their own.
    ftu, utf = [], []
    for bits in (12, 16):
        for lo, hi in ((-6.28, 6.28), (-10.0, 10.0), (-30.0, 30.0), (-120.0, 120.0), (0.0, 500.0), (0.0, 5.0),
                       (-12.5, 12.5)):
            for v in floats(rng, max(abs(lo), abs(hi)), 40) + [math.nan, math.inf, -math.inf]:
                ftu.append([f(v), lo, hi, bits, finish(call(p.float_to_uint, v, lo, hi, bits), f)])
            for u in [0, 1, (1 << bits) - 1, (1 << bits) // 2, (1 << bits) // 2 - 1] + \
                     [rng.randrange(1 << bits) for _ in range(20)]:
                utf.append([u, lo, hi, bits, f(p.uint_to_float(u, lo, hi, bits))])
    fx["float_to_uint"], fx["uint_to_float"] = ftu, utf

    # MIT encoder.
    mit = []
    for name, model in p.MODELS.items():
        ps = floats(rng, model.p_max, 120)
        vs = floats(rng, model.v_max, 120)
        ts = floats(rng, model.t_max, 120)
        kps = floats(rng, 500.0, 120) + [500.0, 0.0, 499.99, 250.0]
        kds = floats(rng, 5.0, 120) + [5.0, 0.0, 4.9999, 2.5]
        for i in range(600):
            mid = rng.choice([1, 2, 3, 7, 15, rng.randint(1, 15)])
            args = [mid, rng.choice(ps), rng.choice(vs), rng.choice(kps), rng.choice(kds), rng.choice(ts)]
            mit.append([name, *map(f, args), finish(call(p.encode_mit, *args, model), frame)])
        # All-max frames (the command-prefix guard), the ends, invalid ids and non-finite values.
        specials = [
            [1, model.p_max, model.v_max, 500.0, 5.0, model.t_max],
            [1, model.p_max * 2, model.v_max * 2, 1000.0, 50.0, model.t_max * 2],
            [1, math.nextafter(model.p_max, 0), model.v_max, 500.0, 5.0, model.t_max],
            [2, -model.p_max, -model.v_max, 0.0, 0.0, -model.t_max],
            [0, 0.0, 0.0, 0.0, 0.0, 0.0], [16, 0.0, 0.0, 0.0, 0.0, 0.0], [0x7FF, 0.0, 0.0, 0.0, 0.0, 0.0],
            [-1, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1, math.nan, 0, 0, 0, 0], [1, 0, math.inf, 0, 0, 0], [1, 0, 0, -math.inf, 0, 0], [1, 0, 0, 0, math.nan, 0],
            [1, 0, 0, 0, 0, math.inf], [0, math.nan, 0, 0, 0, 0],
        ]
        for args in specials:
            mit.append([name, *map(f, args), finish(call(p.encode_mit, *args, model), frame)])
    fx["encode_mit"] = mit

    # State encoder.
    state = []
    for name, model in p.MODELS.items():
        ps, vs, ts = floats(rng, model.p_max, 60), floats(rng, model.v_max, 60), floats(rng, model.t_max, 60)
        for _ in range(300):
            args = [rng.randint(0, 31), rng.choice(list(range(16))), rng.choice(ps), rng.choice(vs), rng.choice(ts),
                    rng.choice([0, 30, 255, 256, -1, -40, 300, rng.randint(-300, 600)]),
                    rng.choice([0, 30, 255, 256, -1, rng.randint(-300, 600), 45.7, -3.9])]
            r = call(p.encode_state, *args, model)
            state.append([name, *map(f, args), finish(r, lambda b: b.hex())])
        for args in ([1, 0, math.nan, 0, 0, 30, 30], [1, 0, 0, 0, math.inf, 30, 30]):
            state.append([name, *map(f, args), finish(call(p.encode_state, *args, model), lambda b: b.hex())])
    fx["encode_state"] = state

    # Decoder: random frames on random ids, plus encoded states and register replies.
    dec = []
    ids = [0x00, 0x01, 0x07, 0x0F, 0x10, 0x11, 0x17, 0x1F, 0x20, 0x55, 0x101, 0x10F, 0x110, 0x307, 0x30F, 0x310,
           0x400, 0x401, 0x7FE, 0x7FF, 0x800, -1]
    for name, model in p.MODELS.items():
        for i in range(700):
            cid = rng.choice(ids + [rng.randint(0, 0x7FF)])
            n = rng.choice([8, 8, 8, 8, 7, 0, 9])
            r = rng.random()
            if r < 0.3:
                data = bytes(rng.randrange(256) for _ in range(n))
            elif r < 0.6:
                data = sim.feedback_bytes(rng.randint(0, 15), pos=rng.uniform(-7, 7), vel=rng.uniform(-40, 40),
                                          tau=rng.uniform(-130, 130), temp=rng.randint(0, 120),
                                          status=rng.randrange(16), model=name)[:n]
            elif r < 0.85:
                data = bytes([rng.randint(0, 16), rng.choice([0, 0, 1]), rng.choice([0x33, 0x55, 0xAA, 0xCC, 0x00]),
                              rng.randrange(256)] + [rng.randrange(256) for _ in range(4)])[:n]
            else:
                status = rng.choice([0, 0, 1, 0xD, 2, 0xF])
                data = bytes([(status << 4) | rng.randint(1, 15), 0, rng.choice([0x33, 0x55, 0xAA, 0x34])] +
                             [rng.randrange(256) for _ in range(5)])[:n]
            ext = rng.random() < 0.1
            can_id = CanId(cid, extended=True) if ext else cid
            dec.append([name, cid, ext, data.hex(), decoded(p.decode(can_id, data, model))])
    fx["decode"] = dec

    # Command, refresh and register encoders.
    cmds = []
    for mid in [-1, 0, 1, 2, 7, 8, 15, 16, 0x7FF]:
        for mode in [1, 2, 3, 4, 5, 0]:
            for name, fn in (("command_fb", lambda i, md: p.encode_command(i, 0xFB, md)),
                             ("enable", p.encode_enable), ("disable", p.encode_disable),
                             ("set_zero", p.encode_set_zero), ("clear_error", p.encode_clear_error)):
                cmds.append([name, mid, mode, finish(call(fn, mid, mode), frame)])
        for name, fn in (("refresh", p.encode_refresh), ("save", p.encode_save)):
            cmds.append([name, mid, None, finish(call(fn, mid), frame)])
    for cmd in [0x00, 0x7F, 0xFA, 0xFF]:
        cmds.append(["command_%02x" % cmd, 3, 1, finish(call(p.encode_command, 3, cmd, 1), frame)])
    fx["commands"] = cmds

    regs = []
    for mid in [0, 1, 7, 15, 16]:
        for rid in [-1, 0, 7, 9, 10, 21, 59, 80, 255, 256]:
            regs.append(["read", mid, rid, None, None, finish(call(p.encode_read_register, mid, rid), frame)])
    values = [0, 1, 40, 10_000, 0x1234, 2**32 - 1, 2**32, -1, 6.28, 12.5, -3.75, 1e-40, 3.4028235e38, 3.4028236e38,
              3.5e38, 1e39, -1e39, 0.1, 1 / 3, math.pi, 7.9, -0.9, math.nan, math.inf, -math.inf, 2.5, 1e20]
    for _ in range(300):
        values.append(rng.choice([rng.uniform(-1e6, 1e6), rng.randint(0, 2**32 - 1), rng.uniform(-10, 10),
                                  rng.choice([-1, 1]) * 10 ** rng.uniform(-45, 39)]))
    for v in values:
        for rid in [7, 9, 10, 21, 27, 59, 300]:
            for dtype in [None, "f", "u32", "u8"]:
                if rng.random() < 0.5 and v not in (0, 1, math.nan):
                    continue
                mid = rng.choice([1, 7, 15, 0])
                regs.append(["write", mid, rid, f(v), dtype, finish(call(p.encode_write_register, mid, rid, v, dtype), frame)])
    fx["registers"] = regs

    # pack/unpack.
    pack = []
    for v in values:
        for dtype in ["f", "u32", "x"]:
            pack.append([f(v), dtype, finish(call(p.pack_value, v, dtype), lambda b: b.hex())])
    unpack = []
    raws = [bytes(4), b"\xff\xff\xff\xff", b"\x00\x00\x80\x7f", b"\x00\x00\x80\xff", b"\x01\x00\xc0\x7f",
            struct.pack("<f", 6.28), b"\x00\x00\x00", b"\x00" * 5]
    raws += [bytes(rng.randrange(256) for _ in range(4)) for _ in range(300)]
    for raw in raws:
        for dtype in ["f", "u32", "x"]:
            unpack.append([raw.hex(), dtype, finish(call(p.unpack_value, raw, dtype), f)])
    fx["pack_value"], fx["unpack_value"] = pack, unpack

    fx["register_dtype"] = [[i, p.register_dtype(i)] for i in range(0, 256)]
    fx["status_text"] = [[s, p.status_text(s)] for s in list(range(16)) + [None, 0x1F, -10]]
    fx["is_fault_status"] = [[s, p.is_fault_status(s)] for s in range(16)]
    fx["is_command_id"] = [[c, p.is_command_id(c)] for c in list(range(0, 0x810)) + [-1, 0x1000]]
    irr = []
    for _ in range(600):
        n = rng.choice([8, 8, 8, 7, 9])
        data = bytes([rng.choice([0, 1, 7, 15, 16, rng.randrange(256)]), rng.choice([0, 0, 1, rng.randrange(256)]),
                      rng.choice([0x33, 0x55, 0xAA, 0xCC, rng.randrange(256)])] + [rng.randrange(256) for _ in range(5)])[:n]
        irr.append([data.hex(), p.is_register_reply(data)])
    fx["is_register_reply"] = irr

    mm = []
    for name, model in p.MODELS.items():
        exact = [model.p_max, model.v_max, model.t_max]
        f32 = list(struct.unpack("<3f", struct.pack("<3f", *exact)))
        mm.append([*exact, p.matching_model(*exact)])
        mm.append([*f32, p.matching_model(*f32)])
        for _ in range(20):
            jit = [x * (1 + rng.choice([-1, 1]) * rng.choice([1e-6, 9.9e-6, 1e-5, 1.01e-5, 1e-4])) for x in exact]
            mm.append([*jit, p.matching_model(*jit)])
    for trip in ([12.5, 30.0, 10.0], [0.0, 0.0, 0.0], [math.inf, 10.0, 30.0], [math.nan, 10.0, 30.0], [6.28, 10.0, -30.0]):
        mm.append([*map(f, trip), p.matching_model(*trip)])
    fx["matching_model"] = mm

    # motor.route over the same kind of frames.
    routes = []
    for _ in range(1500):
        host = rng.choice([0x10, 0x10, 0x20, 0x00, 0x30])
        cid = rng.choice(ids + [host + rng.randint(-2, 17), rng.randint(0, 0x7FF)])
        n = rng.choice([8, 8, 8, 7])
        r = rng.random()
        if r < 0.4:
            data = sim.feedback_bytes(rng.randint(0, 15), status=rng.randrange(16), pos=rng.uniform(-7, 7))[:n]
        elif r < 0.7:
            data = sim.register_reply(rng.randint(0, 16), rng.choice([0x33, 0x55, 0xAA, 0xCC]), rng.randrange(256),
                                      bytes(rng.randrange(256) for _ in range(4)))[:n]
        else:
            data = bytes(rng.randrange(256) for _ in range(n))
        ext = rng.random() < 0.1
        can_id = CanId(cid, extended=True) if ext else cid
        routes.append([cid, ext, data.hex(), host, m.route(can_id, data, host)])
    fx["route"] = routes

    # Simulator helpers.
    fb = []
    for _ in range(300):
        name = rng.choice(list(p.MODELS))
        kw = dict(pos=rng.uniform(-7, 7), vel=rng.uniform(-35, 35), tau=rng.uniform(-130, 130),
                  temp=rng.choice([30.0, 30.5, 31.5, -0.5, 44.49, rng.uniform(-20, 140)]),
                  status=rng.randrange(16), rotor_temp=rng.choice([None, 2.5, 3.5, rng.uniform(-20, 140)]), model=name)
        mid = rng.randint(0, 31)
        fb.append([mid, {k: f(v) for k, v in kw.items()}, sim.feedback_bytes(mid, **kw).hex()])
    fx["feedback_bytes"] = fb
    cls = []
    frames = [p.encode_refresh(3), p.encode_read_register(3, 9), p.encode_write_register(3, 9, 1), p.encode_save(3),
              p.encode_clear_error(3), p.encode_enable(3), p.encode_disable(12), p.encode_set_zero(5),
              p.encode_command(3, 0x12), p.encode_mit(3, 0, 0, 1, 0, 0, p.MODELS["jlo"]), (0x7FF, b"\x03\x00"),
              (0x7FF, b"\x03\x01\x99\x00"), (0x203, bytes(8)), (0x403, bytes(8)), (0x7FE, bytes(8))]
    for cid, data in frames:
        cls.append([cid, bytes(data).hex(), list(sim.classify(cid, data))])
    fx["classify"] = cls
    return fx


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    text = json.dumps(gen(), allow_nan=False, separators=(",", ":")) + "\n"
    if a.check:
        if not OUT.exists() or OUT.read_text() != text:
            print(f"stale: {OUT.relative_to(ROOT)}")
            return 1
        return 0
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(text)
    print(f"wrote {OUT.relative_to(ROOT)} ({len(text) // 1024} KiB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
