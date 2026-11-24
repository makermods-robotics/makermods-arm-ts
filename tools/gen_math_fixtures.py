#!/usr/bin/env python3
"""Cross-language fixtures for arm-core's math (kinematics.ts, trajectory-ik.ts, gravity.ts, linalg.ts).

Run with each arm SDK's own interpreter; each run writes that arm's files, plus the arm-independent ones:

    ~/maker-arm-sdk/.venv/bin/python tools/gen_math_fixtures.py
    ~/metal-arm-sdk/.venv/bin/python tools/gen_math_fixtures.py

Writes to contract/fixtures/:
    math_<arm>.json                      inputs and the Python SDK's outputs on the arm's shipped profile
    math_<arm>_gravity_fixtures.json     copy of the SDK's tests/unit/gravity_fixtures.json (+ URDF text)
    math_<arm>_trajectory_ik_fixtures.json  copy of the SDK's tests/unit/trajectory_ik_fixtures.json
    math_common.json                     rotations and pose-path helpers
    math_fma.json                        fused multiply-add reference values (exact rational arithmetic)

Non-finite floats are written as the strings "inf", "-inf" and "nan". Each arm file carries the raw profile
(snake_case, `urdf` holding the URDF text) and the product limits ArmConfig validation needs, so the TypeScript
tests need no arm package. Nothing here opens a device.
"""

import argparse
import importlib
import json
import math
import sys
import time
from fractions import Fraction
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "contract" / "fixtures"
HOME = Path.home()
ARMS = {"maker-arm": ("maker_arm", HOME / "maker-arm-sdk", "maker_arm_v1.yaml"),
        "metal-arm": ("metal_arm", HOME / "metal-arm-sdk", "metal_arm_v1.yaml")}


def enc(x):
    """JSON-ready: arrays to lists, non-finite floats to strings."""
    if isinstance(x, dict):
        return {k: enc(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [enc(v) for v in x]
    if isinstance(x, np.ndarray):
        return enc(x.tolist())
    if isinstance(x, (np.bool_, bool)):
        return bool(x)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (float, np.floating)):
        x = float(x)
        if math.isnan(x):
            return "nan"
        if math.isinf(x):
            return "inf" if x > 0 else "-inf"
        return x
    return x


def write(name, data):
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / name
    path.write_text(json.dumps(enc(data), allow_nan=False, separators=(",", ":")) + "\n")
    print("wrote", path.relative_to(ROOT), f"({path.stat().st_size // 1024} KiB)")


def fma(a, b, c):
    return float(Fraction(a) * Fraction(b) + Fraction(c))


# ── arm-independent ──
def fma_fixture():
    rng = np.random.default_rng(20261122)
    cases = []
    for _ in range(4000):
        a, b, c = rng.normal(size=3) * 10 ** rng.uniform(-3, 3, 3)
        if rng.random() < 0.3:
            c = -float(Fraction(a) * Fraction(b)) * (1 + rng.normal() * 1e-12)
        cases.append([a, b, c, fma(a, b, c)])
    for _ in range(1500):
        a, b, c = (float(v) for v in rng.normal(size=3) * 10.0 ** rng.integers(-320, 300, 3))
        try:
            cases.append([a, b, c, fma(a, b, c)])
        except OverflowError:
            pass
    for a, b, c in [(0.0, 5.0, -0.0), (-0.0, 5.0, 0.0), (1.0, 1.0, -1.0), (1e308, 10.0, -1e308),
                    (2.0 ** -540, 2.0 ** -540, 0.0), (2.0 ** -1000, 2.0 ** -70, 2.0 ** -1074)]:
        try:
            cases.append([a, b, c, fma(a, b, c)])
        except OverflowError:
            cases.append([a, b, c, math.inf])
    return {"fma": cases}


def common_fixture(sdk):
    K, TI = sdk.kinematics, sdk.trajectory_ik
    rng = np.random.default_rng(7)
    rpys = [rng.uniform([-math.pi, -1.5, -math.pi], [math.pi, 1.5, math.pi]) for _ in range(30)]
    rpys += [np.array(v) for v in ([0, 0, 0], [0, math.pi / 2, 0.3], [0.2, -math.pi / 2, 0.1], [math.pi, 0, 0],
                                   [math.radians(90), 0, math.radians(180)], [math.radians(180), 0, math.radians(90)])]
    rotation = [{"rpy": r, "matrix": K.rpy_to_matrix(r), "back": K.matrix_to_rpy(K.rpy_to_matrix(r))} for r in rpys]
    poses = [{"xyz": rng.normal(size=3), "rpy": r} for r in rpys[:8]]
    for p in poses:
        p["pose"] = K.xyz_rpy_to_pose(p["xyz"], p["rpy"])
        p["xyz_back"], p["rpy_back"] = K.pose_to_xyz_rpy(p["pose"])
    axis = np.array([1.0, -2.0, 0.5]) / np.linalg.norm([1.0, -2.0, 0.5])
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    rot_err = []
    for angle in [0.0, 1e-8, 1e-5, 0.3, 1.0, 2.5, math.pi - 1e-3, math.pi - 1e-6, math.pi]:
        rot = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k
        cur = K.rpy_to_matrix(rng.uniform(-1, 1, 3))
        rot_err.append({"target": rot, "current": np.eye(3), "out": K.rotation_error(rot, np.eye(3))})
        rot_err.append({"target": rot @ cur, "current": cur, "out": K.rotation_error(rot @ cur, cur)})
    ws = np.vstack([np.zeros(3), [1e-9, 0, 0], [0.3, -0.2, 0.1], [0, math.pi - 1e-7, 0], [math.pi, 0, 0],
                    rng.normal(size=(10, 3)), rng.normal(size=(5, 3)) * 1e-5,
                    axis * (math.pi - 1e-3), -axis * math.pi, axis * 2.9])
    so3 = {"w": ws, "exp": TI.so3_exp(ws)}
    so3["log"] = TI.so3_log(so3["exp"])
    lines = []
    for start, goal, ticks, ease in [
            (np.eye(4), K.xyz_rpy_to_pose([0.1, 0, 0], [0, 0, 0.5]), 30, False),
            (K.xyz_rpy_to_pose([0.2, -0.1, 0.3], [0.3, 0.2, -1.0]), K.xyz_rpy_to_pose([0.25, 0.05, 0.2], [0.1, -0.3, 0.4]), 40, True),
            (np.eye(4), K.xyz_rpy_to_pose([0, 0, 0.1], [math.pi, 0, 0]), 12, True),
            (K.xyz_rpy_to_pose([0.1, 0.1, 0.1]), K.xyz_rpy_to_pose([0.1, 0.1, 0.1]), 1, True)]:
        lines.append({"start": start, "goal": goal, "ticks": ticks, "ease": ease,
                      "out": TI.straight_line(start, goal, ticks, ease)})
    t = np.linspace(0, 1, 30)[:, None]
    savgols = []
    x30 = np.hstack([np.sin(3 * t), t ** 2, -t]) + rng.normal(0, 0.01, (30, 3))
    for x, window, order in [(x30, 15, 3), (x30, 7, 2), (x30, 5, 0), (x30, 1, 0), (x30[:9], 9, 3),
                             (rng.normal(size=(2, 20, 3)), 11, 3)]:
        savgols.append({"x": x, "window": window, "polyorder": order, "out": TI.savgol(x, window, order)})
    walk = [np.eye(4)]
    for _ in range(24):
        step = K.xyz_rpy_to_pose(rng.normal(0, 0.01, 3), rng.normal(0, 0.05, 3))
        walk.append(walk[-1] @ step)
    walk = np.array(walk)
    smooth = [{"T": walk, "window": w, "polyorder": p, "out": TI.smooth_poses(walk, w, p)}
              for w, p in [(15, 3), (6, 2), (0, 3), (31, 3)]]
    smooth.append({"T": walk[:3], "window": 15, "polyorder": 3, "out": TI.smooth_poses(walk[:3], 15, 3)})
    smooth.append({"T": walk[:5], "window": 15, "polyorder": 3, "out": TI.smooth_poses(walk[:5], 15, 3)})
    extend = [{"T": walk[:n], "rows": rows, "k": k, "out": TI.extend_poses(walk[:n], rows, k)}
              for n, rows, k in [(10, 5, 4), (10, 0, 4), (10, 3, 2), (3, 4, 4), (25, 12, 4)]]
    q = np.cumsum(rng.normal(0, 0.05, (12, 6)), axis=0)
    q_from = q[0] + rng.normal(0, 0.2, 6)
    seams = []
    for ticks, lo, hi, step in [(4, -10.0, 10.0, np.full(6, 0.2)), (4, q.min(0) + 0.05, q.max(0) - 0.05, np.full(6, 0.03)),
                                (0, -10.0, 10.0, 0.5), (20, -0.3, 0.3, np.linspace(0.01, 0.1, 6))]:
        seams.append({"q": q, "q_from": q_from, "ticks": ticks, "lo": lo, "hi": hi, "max_step": step,
                      "out": TI.seam_blend(q, q_from, ticks, lo, hi, step)})
    return {"rotation": rotation, "poses": poses, "rotation_error": rot_err, "so3": so3, "straight_line": lines,
            "savgol": savgols, "smooth_poses": smooth, "extend_poses": extend, "seam_blend": seams}


# ── per arm ──
def path_from(TI, chain, q, offset, ticks, turn=(0.0, 0.0, 0.0), ease=True):
    start = chain.fk(q)
    goal = start.copy()
    goal[:3, 3] += offset
    goal[:3, :3] = start[:3, :3] @ TI.so3_exp(np.asarray(turn, dtype=float))
    return TI.straight_line(start, goal, ticks, ease)


def solve_case(ik, name, targets, q_last, q_before=None, rate_hz=20.0, speed_cap=None, method="traj"):
    result = ik.solve(targets, q_last, q_before, rate_hz=rate_hz, speed_cap=speed_cap, method=method)
    return {"name": name, "targets": targets, "q_last": q_last, "q_before": q_before, "rate_hz": rate_hz,
            "speed_cap": speed_cap, "method": method, "q": result.q, "q_solution": result.q_solution,
            "pos_err": result.pos_err, "rot_err": result.rot_err, "capped": result.capped}


def timed(fn, repeats=5):
    times = []
    for _ in range(repeats):
        start = time.perf_counter()
        fn()
        times.append(time.perf_counter() - start)
    return float(np.median(times))


def arm_fixture(arm, sdk, sdk_dir, profile_name):
    K, TI, G = sdk.kinematics, sdk.trajectory_ik, sdk.gravity
    config_mod, motor = sdk.config, importlib.import_module(f"{sdk.__name__}.motor")
    profile_path = sdk_dir / sdk.__name__ / "profiles" / profile_name
    raw = yaml.safe_load(profile_path.read_text())
    urdf_text = (sdk_dir / sdk.__name__ / raw["urdf"]).read_text()
    raw["urdf"] = urdf_text
    config = config_mod.ArmConfig.from_yaml(str(profile_path))
    chain = K.chain_from_config(config)
    n = chain.n
    joints = config.joints[:n]
    soft_lo = np.array([j.lo for j in joints])
    soft_hi = np.array([j.hi for j in joints])
    rng = np.random.default_rng({"maker-arm": 101, "metal-arm": 202}[arm])
    product = {"models": list(motor.MODELS), "default_model": motor.DEFAULT_MODEL, "kp_range": list(motor.KP_RANGE),
               "kd_range": list(motor.KD_RANGE), "id_range": list(motor.ID_RANGE), "default_host_id": motor.DEFAULT_HOST_ID}

    # kinematics
    qs = [rng.uniform(soft_lo, soft_hi) for _ in range(40)]
    qs += [np.zeros(n), soft_lo.copy(), soft_hi.copy(), (soft_lo + soft_hi) / 2, np.full(n, -0.0), np.full(n, 1e-9),
           np.linspace(-10, 10, n), np.linspace(-100, 100, n), np.full(n, 2.0e8), rng.uniform(-math.pi, math.pi, n)]
    kin = []
    for q in qs:
        kin.append({"q": q, "fk": chain.fk(q), "jacobian": chain.jacobian(q), "gravity": chain.gravity_torques(q),
                    "potential": chain.potential_energy(q)})
    custom_g = [{"q": qs[i], "g": g, "gravity": chain.gravity_torques(qs[i], g), "potential": chain.potential_energy(qs[i], g)}
                for i, g in [(0, [0.0, 0.0, -9.80665]), (1, [1.0, -2.0, -9.0]), (2, [0.0, 0.0, 0.0])]]
    batch = np.array(qs[:6])
    poses, jacs = chain.fk_jacobian_many(batch)
    bare = K.Chain.from_urdf(sdk_dir / sdk.__name__ / config.urdf)
    partial = K.Chain.from_urdf(sdk_dir / sdk.__name__ / config.urdf, joints=chain.joint_names[:3])
    variants = []
    for label, c in [("bare", bare), ("partial", partial)]:
        cq = [rng.uniform(-1, 1, c.n) for _ in range(4)]
        variants.append({"label": label, "joint_names": c.joint_names, "masses": c.masses, "lower": c.lower,
                         "upper": c.upper, "q": cq, "fk": [c.fk(q) for q in cq],
                         "gravity": [c.gravity_torques(q) for q in cq]})

    def ik_entry(target, q0, **kw):
        res = chain.ik(target, q0, **kw)
        opts = {k: (list(v) if k == "limits" else v) for k, v in kw.items()}
        return {"target": target, "q0": q0, "options": opts, "q": res.q, "ok": res.ok, "pos_err": res.pos_err,
                "rot_err": res.rot_err, "iters": res.iters}

    iks = []
    limits = (soft_lo, soft_hi)
    for i in range(10):
        q_true = rng.uniform(soft_lo, soft_hi)
        seed = np.clip(q_true + rng.uniform(-0.2, 0.2, n), soft_lo, soft_hi)
        pose = chain.fk(q_true)
        iks.append(ik_entry(pose, seed, limits=limits))
        iks.append(ik_entry(pose[:3, 3], seed, limits=limits))
    q_mid = (soft_lo + soft_hi) / 2
    target = chain.fk(q_mid)
    iks.append(ik_entry(target[:3, 3], q_mid + 0.1, rpy=K.matrix_to_rpy(target[:3, :3]), limits=limits))
    iks.append(ik_entry(target[:3, 3], q_mid, limits=limits))
    iks.append(ik_entry(np.array([5.0, 0.0, 0.0]), q_mid, limits=limits))
    iks.append(ik_entry(target, q_mid + 0.3))
    iks.append(ik_entry(target, q_mid + 0.3, position_only=True, max_iters=5))
    iks.append(ik_entry(chain.fk(q_mid + 0.4), q_mid, tol=1e-6, rot_tol=1e-5, rot_weight=0.5, null_gain=0.0,
                        max_step=0.1, limits=limits))

    # gravity feed-forward
    ff = G.from_config(config, chain)
    rest = np.clip(np.array((config.ik or {}).get("rest_pose", np.zeros(n))), soft_lo, soft_hi)

    def sequence(ff, steps, seed):
        r = np.random.default_rng(seed)
        out, now = [], 10.0
        base = rest.copy()
        for k in range(steps):
            if k == steps * 2 // 3:
                ff.reset()
                out.append({"op": "reset"})
            now += {7: 1.0, 15: -0.5, 16: 0.0}.get(k, 0.005 + r.normal(0, 0.0005))
            measured = list(base + 0.3 * np.sin(0.05 * k + np.arange(n)) + r.normal(0, 0.01, n))
            ages = list(r.uniform(0, 0.02, n))
            goal = list(base + 0.3 * np.sin(0.05 * k + np.arange(n)))
            if k % 11 == 3:
                ages[k % n] = G.STALE_S + 0.001
            if k % 13 == 5:
                ages[(k + 1) % n] = G.STALE_S
            if k % 17 == 6:
                measured[k % n] = math.nan
            if k == 40:
                goal = [math.nan] * n
                measured[0] = math.inf
            tau = ff.torques(measured, ages, goal, now)
            out.append({"op": "torques", "measured": measured, "ages": ages, "goal": goal, "now": now, "out": tau,
                        "level": ff.level})
        return out

    seqs = [{"label": "profile", "init": "from_config", "steps": sequence(ff, 150, 1)}]
    caps = list(np.linspace(0.05, 0.4, n))
    factors = list(np.linspace(0.6, 1.8, n))
    small = G.GravityFeedforward(chain, 1.2, caps, ramp_s=0.3, stale_s=0.1, joint_scale=factors)
    seqs.append({"label": "caps", "init": {"scale": 1.2, "limits_nm": caps, "ramp_s": 0.3, "stale_s": 0.1,
                                           "joint_scale": factors}, "steps": sequence(small, 120, 2)})
    half = G.GravityFeedforward(chain, 0.5, [j.gravity_limit_nm for j in joints],
                                joint_scale=[j.gravity_factor for j in joints])
    seqs.append({"label": "half", "init": {"scale": 0.5, "limits_nm": [j.gravity_limit_nm for j in joints],
                                           "joint_scale": [j.gravity_factor for j in joints]},
                 "steps": sequence(half, 60, 3)})
    model = [{"q": q, "out": ff.model(q)} for q in qs[:20]] + [{"q": [math.inf] + list(qs[0][1:]), "out": ff.model([math.inf] + list(qs[0][1:]))}]

    # trajectory IK
    ik = TI.TrajectoryIK.from_config(config, chain=chain)
    w = ik.weights
    q0 = ik.rest
    info = {"lo": ik.lo, "hi": ik.hi, "max_velocity": ik.max_velocity, "rest": ik.rest, "rest_weight": ik.rest_weight,
            "free": ik.free, "weights": {f: getattr(w, f) for f in TI.WEIGHT_FIELDS}}
    caps_cases = []
    for rate, cap in [(20.0, None), (200.0, 0.5), (config.control_rate_hz, 2.0)]:
        step = ik.max_step(rate, cap)
        rows = q0 + np.cumsum(rng.normal(0, 0.1, (8, n)), axis=0)
        qc, capped = ik.cap(rows, q0, step)
        caps_cases.append({"rate_hz": rate, "speed_cap": cap, "max_step": step, "rows": rows, "q_last": q0, "q": qc,
                           "capped": capped})
    bounds_q = q0.copy()
    bounds_q[0] = ik.hi[0] + 0.01
    bounds_q[1] = ik.lo[1] - 0.02
    bounds = {"q_last": bounds_q, "out": ik.bounds(bounds_q)}
    ticks = []
    for i in range(6):
        q_prev = np.clip(q0 + rng.normal(0, 0.1, n), ik.lo, ik.hi)
        target = chain.fk(np.clip(q_prev + rng.normal(0, 0.05, n), ik.lo, ik.hi))
        if i == 5:
            target = chain.fk(q_prev).copy()
            target[:3, 3] += [2.0, 0.0, 0.0]
        ticks.append({"target": target, "q_prev": q_prev, "out": ik.solve_tick(target, q_prev)})
    solves = []
    t30 = path_from(TI, chain, q0, [0.03, -0.02, 0.02], 30, turn=[0.1, 0.0, 0.1])
    t48 = path_from(TI, chain, q0, [0.05, 0.03, -0.02], 48, turn=[0.0, 0.15, -0.1])
    solves.append(solve_case(ik, "smooth30", t30, q0))
    solves.append(solve_case(ik, "smooth48", t48, q0, rate_hz=config.control_rate_hz))
    solves.append(solve_case(ik, "tick-capped", path_from(TI, chain, q0, [0.15, 0.1, -0.1], 10, ease=False), q0,
                             speed_cap=0.5, method="tick"))
    solves.append(solve_case(ik, "traj-capped", path_from(TI, chain, q0, [0.15, 0.1, -0.1], 10, ease=False), q0,
                             speed_cap=0.5))
    q_hist = np.array([q0 - 0.01, q0 - 0.025])
    solves.append(solve_case(ik, "history2", t30[:16], q0, q_before=q_hist))
    solves.append(solve_case(ik, "history1", t30[:16], q0, q_before=q_hist[0]))
    solves.append(solve_case(ik, "unreachable", path_from(TI, chain, q0, [3.0, 0.0, 0.0], 10), q0))
    solves.append(solve_case(ik, "limits", path_from(TI, chain, q0, [0.0, 0.0, -2.0], 20), q0))
    past = q0.copy()
    past[0] = ik.hi[0] + 0.01
    solves.append(solve_case(ik, "past-margin", np.repeat(chain.fk(past)[None], 10, axis=0), past))
    timing = {"rows30_s": timed(lambda: ik.solve(t30, q0, rate_hz=20.0)),
              "rows48_s": timed(lambda: ik.solve(t48, q0, rate_hz=config.control_rate_hz))}
    line = TI.straight_line(chain.fk(q0), chain.fk(q0) @ K.xyz_rpy_to_pose([0.08, 0.0, 0.05]), 60, ease=False)
    stream = TI.IKStream(ik, q0, 20.0)
    feeds = []
    for a, b in [(0, 20), (20, 40), (40, 60)]:
        r = stream.feed(line[a:b])
        feeds.append({"targets": line[a:b], "q": r.q, "q_solution": r.q_solution, "pos_err": r.pos_err,
                      "rot_err": r.rot_err, "capped": r.capped})
    stepper = TI.IKStream(ik, q0, 20.0, speed_cap=1.0)
    step_poses = path_from(TI, chain, q0, [0.05, 0.05, 0.0], 12)
    steps = [{"pose": p, "out": stepper.step(p)} for p in step_poses]
    path_poses = np.concatenate([TI.straight_line(chain.fk(q0), chain.fk(q0) @ K.xyz_rpy_to_pose([0.06, -0.04, 0.03], [0.1, 0, 0]), 70),
                                 np.repeat(chain.fk(q0)[None] @ K.xyz_rpy_to_pose([0.06, -0.04, 0.03], [0.1, 0, 0]), 10, axis=0)])
    paths = []
    for rate, cap in [(config.control_rate_hz, None), (20.0, 1.0)]:
        r = TI.solve_path(ik, path_poses, q0, rate, cap)
        paths.append({"poses": path_poses, "q_last": q0, "rate_hz": rate, "speed_cap": cap, "q": r.q,
                      "q_solution": r.q_solution, "pos_err": r.pos_err, "rot_err": r.rot_err, "capped": r.capped})
    qr = np.clip(q0 + rng.normal(0, 0.3, (3, n)), ik.lo, ik.hi)
    rposes, rjacs = chain.fk_jacobian_many(qr)
    rtargets = rposes @ K.xyz_rpy_to_pose([0.02, -0.01, 0.03], [0.3, -0.2, 0.4])
    rr, rJ = TI.pose_residual(rtargets, rposes, rjacs)
    terr = TI.tool_errors(chain, qr, rtargets)
    links, all_joints = K._parse_urdf(sdk_dir / sdk.__name__ / config.urdf)
    parsed = {"links": [[name, l.mass, l.com] for name, l in links.items()],
              "joints": [{"name": j.name, "type": j.type, "parent": j.parent, "child": j.child, "origin": j.origin,
                          "axis": j.axis, "lower": j.lower, "upper": j.upper} for j in all_joints.values()]}
    return {
        "arm": arm, "sdk_commit": None, "product": product, "profile": raw, "urdf_parsed": parsed,
        "chain": {"joint_names": chain.joint_names, "n": n, "masses": chain.masses, "lower": chain.lower,
                  "upper": chain.upper, "mount": chain.mount, "sign": chain.sign, "offset": chain.offset},
        "kinematics": kin, "custom_gravity": custom_g, "batch": {"qs": batch, "poses": poses, "jacobians": jacs},
        "variants": variants, "ik": iks,
        "gravity_model": model, "gravity_sequences": seqs,
        "trajectory": {"info": info, "caps": caps_cases, "bounds": bounds, "solve_tick": ticks, "solve": solves,
                       "feed": feeds, "step": steps, "solve_path": paths,
                       "pose_residual": {"q": qr, "targets": rtargets, "poses": rposes, "jacobians": rjacs, "r": rr,
                                         "J": rJ, "pos": terr[0], "rot": terr[1]}},
        "timing": timing,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=sorted(ARMS), help="default: the SDK whose interpreter runs this script")
    a = ap.parse_args()
    arm = a.arm or next((name for name, (_, d, _) in ARMS.items() if str(d) in sys.executable), None)
    if arm is None:
        ap.error("run with an arm SDK's .venv/bin/python or pass --arm")
    package, sdk_dir, profile_name = ARMS[arm]
    sdk = importlib.import_module(package)
    for sub in ("kinematics", "trajectory_ik", "gravity", "config"):
        importlib.import_module(f"{package}.{sub}")
    import subprocess
    commit = subprocess.run(["git", "-C", str(sdk_dir), "rev-parse", "--short", "HEAD"], capture_output=True,
                            text=True).stdout.strip()
    data = arm_fixture(arm, sdk, sdk_dir, profile_name)
    data["sdk_commit"] = commit
    write(f"math_{arm.replace('-', '_')}.json", data)
    unit = sdk_dir / "tests" / "unit"
    grav = json.loads((unit / "gravity_fixtures.json").read_text())
    grav["chain"]["urdf_text"] = (sdk_dir / grav["chain"]["urdf"]).read_text()
    write(f"math_{arm.replace('-', '_')}_gravity_fixtures.json", grav)
    write(f"math_{arm.replace('-', '_')}_trajectory_ik_fixtures.json",
          json.loads((unit / "trajectory_ik_fixtures.json").read_text()))
    write("math_common.json", common_fixture(sdk))
    write("math_fma.json", fma_fixture())
    return 0


if __name__ == "__main__":
    sys.exit(main())
