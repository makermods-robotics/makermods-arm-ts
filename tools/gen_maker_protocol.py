#!/usr/bin/env python3
"""Run the Maker Arm SDK's RobStride codec (maker_arm/protocol.py) over random and edge-case inputs
and write what it produces to contract/fixtures/maker_protocol.json.

    ~/maker-arm-sdk/.venv/bin/python tools/gen_maker_protocol.py

The TypeScript test (packages/maker-arm/test/protocol-fixture.test.ts) replays every case and must
reproduce every byte and every decoded number exactly.
"""

import json
import math
import random
import struct
from pathlib import Path

from maker_arm import _sim_motor, protocol as p
from maker_arm.transport.base import CanId

OUT = Path(__file__).resolve().parent.parent / "contract" / "fixtures" / "maker_protocol.json"
rng = random.Random(20261122)


def num(x):
    """JSON-safe float: non-finite values as strings."""
    if isinstance(x, float) and not math.isfinite(x):
        return "nan" if math.isnan(x) else ("inf" if x > 0 else "-inf")
    return x


def frame(result):
    can_id, data = result
    return {"id": can_id, "data": data.hex()}


def attempt(fn, *args):
    try:
        return {"ok": frame(fn(*args))}
    except (ValueError, OverflowError, struct.error, KeyError) as e:
        return {"error": type(e).__name__}


def edge_floats(lo, hi, bits):
    span = (1 << bits) - 1
    values = [lo, hi, 0.0, -0.0, lo - 1, hi + 1, lo * 10, hi * 10, 1e300, -1e300, math.nan, math.inf, -math.inf,
              math.nextafter(lo, -math.inf), math.nextafter(lo, math.inf), math.nextafter(hi, math.inf),
              math.nextafter(hi, -math.inf), 5e-324, -5e-324]
    for _ in range(40):   # near the rounding ties of the quantization
        k = rng.randrange(span)
        x = (k + 0.5) * (hi - lo) / span + lo
        values += [x, math.nextafter(x, math.inf), math.nextafter(x, -math.inf)]
    for _ in range(40):   # exactly on the grid
        k = rng.randrange(span + 1)
        values.append(k * (hi - lo) / span + lo)
    return values


def control_cases():
    cases = []
    for model, params in p.MOTOR_PARAMS.items():
        fields = [(-params.p_max, params.p_max, 16), (-params.v_max, params.v_max, 12), (p.KP_MIN, p.KP_MAX, 12),
                  (p.KD_MIN, p.KD_MAX, 12), (-params.t_max, params.t_max, 12)]
        for f, (lo, hi, bits) in enumerate(fields):
            for x in edge_floats(lo, hi, bits):
                args = [0.1, 0.0, 10.0, 1.0, 0.0]
                args[f] = x
                cases.append((rng.randint(1, 127), *args, model))
        for _ in range(1500):
            args = [rng.uniform(lo - 0.1 * (hi - lo), hi + 0.1 * (hi - lo)) for lo, hi, _ in fields]
            cases.append((rng.randint(1, 127), *args, model))
        cases.append((1, params.p_max, params.v_max, p.KP_MAX, p.KD_MAX, params.t_max, model))  # reserved prefix
        for bad in (0, 128, -1):
            cases.append((bad, 0.0, 0.0, 0.0, 0.0, 0.0, model))
    out = []
    for motor_id, pos, vel, kp, kd, tau, model in cases:
        out.append({"args": [motor_id, num(pos), num(vel), num(kp), num(kd), num(tau)], "model": model,
                    **attempt(p.encode_control, motor_id, pos, vel, kp, kd, tau, p.MOTOR_PARAMS[model])})
    return out


def command_cases():
    out = []
    encoders = {"enable": p.encode_enable, "disable": p.encode_disable, "clear_fault": p.encode_clear_fault,
                "fault_query": p.encode_fault_query, "set_zero": p.encode_set_zero}
    for name, fn in encoders.items():
        for motor_id in [0, 1, 2, 64, 127, 128]:
            out.append({"fn": name, "args": [motor_id], **attempt(fn, motor_id)})
    for motor_id, opcode, arg in [(5, 0xAB, 0x1FF), (7, 0x12, 0), (1, 0xFC, -1)]:
        out.append({"fn": "command", "args": [motor_id, opcode, arg], **attempt(p.command, motor_id, opcode, arg)})
    return out


def param_cases():
    out = []
    indices = [p.ParamIndex.RUN_MODE, p.ParamIndex.MECH_POS, p.ParamIndex.CAN_TIMEOUT, 0, 0xFFFF, 0x1234]
    for index in indices + [0x10000, -1]:
        for motor_id in (1, 7, 127, 0):
            out.append({"fn": "read", "args": [motor_id, index], **attempt(p.encode_read_param, motor_id, index)})
    floats = [0.0, -0.0, 1.0, -1.5, 3.14159265358979, 1e-40, 1e-46, 3.4028234663852886e38, 3.4028235e38, 3.5e38,
              -3.5e38, 1e300, math.inf, -math.inf, math.nan, 0.1, 2.5e-39]
    floats += [rng.uniform(-1e6, 1e6) for _ in range(150)] + [rng.uniform(-10, 10) for _ in range(150)]
    floats += [math.ldexp(rng.uniform(1, 2), rng.randint(-160, 140)) for _ in range(150)]
    ints = {"u8": [0, 1, 255, 256, -1, 12.7, -0.5], "u16": [0, 65535, 65536, 300.9],
            "u32": [0, 10000, 4294967295, 4294967296, -1, 7.99, math.nan, math.inf]}
    for value in floats:
        out.append({"fn": "write", "args": [7, p.ParamIndex.MECH_POS, num(value), "f"],
                    **attempt(p.encode_write_param, 7, p.ParamIndex.MECH_POS, value, "f")})
    for dtype, values in ints.items():
        for value in values + [rng.randint(0, 2 ** (8 * {"u8": 1, "u16": 2, "u32": 4}[dtype]) - 1) for _ in range(30)]:
            out.append({"fn": "write", "args": [3, p.ParamIndex.CAN_TIMEOUT, num(value), dtype],
                        **attempt(p.encode_write_param, 3, p.ParamIndex.CAN_TIMEOUT, value, dtype)})
    return out


def decoded(msg):
    if msg is None:
        return None
    if isinstance(msg, p.MotorFeedback):
        return {"kind": "feedback", "motor_id": msg.motor_id, "position": msg.position, "velocity": msg.velocity,
                "torque": msg.torque, "temperature": msg.temperature, "mode": msg.mode,
                "fault_flag": msg.fault_flag, "warning_flag": msg.warning_flag}
    if isinstance(msg, p.FaultStatus):
        return {"kind": "status", "motor_id": msg.motor_id, "word": msg.word, "has_fault": msg.has_fault,
                "text": msg.text}
    values = {}
    for dtype in ("f", "u8", "u16", "u32"):
        values[dtype] = num(msg.value(dtype))
    return {"kind": "param", "motor_id": msg.motor_id, "index": msg.index, "raw": msg.raw.hex(),
            "write": msg.write, "values": values}


def parse_cases():
    out = []
    ids = [p.HOST_CAN_ID, 0xFE, 0x10, 0x301, 0x37F, 0x3FF, 0x407, 0x4FF, 0x200, 0x7FF, 0x800, 0x1FFFFFFF]
    payloads = []
    for _ in range(1500):
        n = rng.choice([8, 8, 8, 8, 5, 0, 1, 4, 6, 7])
        payloads.append(bytes(rng.randrange(256) for _ in range(n)))
    for _ in range(300):   # padded fault-status signature and plausible feedback
        payloads.append(bytes([rng.randint(1, 127)]) + struct.pack("<I", rng.getrandbits(32)) + bytes(3))
        payloads.append(_sim_motor.feedback_bytes(rng.randint(1, 127), rng.uniform(-13, 13), rng.uniform(-50, 50),
                                                  rng.uniform(-20, 20), rng.uniform(0, 120), rng.randint(0, 3),
                                                  rng.random() < 0.3, rng.random() < 0.3))
    for i, data in enumerate(payloads):
        can_id = rng.choice(ids) if i % 3 else p.HOST_CAN_ID
        model = rng.choice(list(p.MOTOR_PARAMS))
        host_id = p.HOST_CAN_ID if i % 7 else 0x10
        extended = i % 11 == 0
        cid = CanId(can_id, extended=True) if extended else can_id
        out.append({"id": can_id, "data": data.hex(), "model": model, "host_id": host_id, "extended": extended,
                    "result": decoded(p.parse_frame(cid, data, p.MOTOR_PARAMS[model], host_id)),
                    "reply_motor_id": p.reply_motor_id(cid, data)})
    return out


def fault_text_cases():
    words = [None, 0, 1, 2, 4, 1 << 6, 1 << 14, 1 << 16, 1 << 31, 0xFFFFFFFF, (1 << 2) | (1 << 14)]
    words += [rng.getrandbits(32) for _ in range(50)]
    return [{"word": w, "text": p.fault_text(w)} for w in words]


def sim_feedback_cases():
    out = []
    for _ in range(300):
        model = rng.choice(list(p.MOTOR_PARAMS))
        args = dict(pos=rng.uniform(-13, 13), vel=rng.uniform(-50, 50), tau=rng.uniform(-20, 20),
                    temp=rng.choice([rng.uniform(0, 150), round(rng.uniform(0, 150), 1), 30.05, 41.25]),
                    mode=rng.randint(0, 3), fault=rng.random() < 0.5, warn=rng.random() < 0.5)
        data = _sim_motor.feedback_bytes(rng.randint(1, 127), params=p.MOTOR_PARAMS[model], **args)
        out.append({"motor_id": data[0], "model": model, **args, "data": data.hex()})
    return out


def main():
    fixture = {"source": "maker_arm.protocol", "control": control_cases(), "commands": command_cases(),
               "params": param_cases(), "parse": parse_cases(), "fault_text": fault_text_cases(),
               "sim_feedback": sim_feedback_cases()}
    OUT.write_text(json.dumps(fixture, separators=(",", ":")) + "\n")
    print(f"wrote {OUT} ({sum(len(v) for v in fixture.values() if isinstance(v, list))} cases)")


if __name__ == "__main__":
    main()
