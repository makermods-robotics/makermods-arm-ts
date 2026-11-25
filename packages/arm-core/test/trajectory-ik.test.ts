import { afterAll, describe, expect, it } from "vitest";
import { ValueError } from "../src/errors.js";
import { Chain, chainFromConfig, xyzRpyToPose } from "../src/kinematics.js";
import * as TI from "../src/trajectory-ik.js";
import { IKStream, IKWeights, TrajectoryIK } from "../src/trajectory-ik.js";
import { ARMS, armFixture, compare, configOf, type Json, loadFixture, Tally } from "./helpers/math-fixtures.js";
import { SLICE } from "../src/cooperate.js";
import { PLAN_S_PER_TICK } from "../src/planner.js";

const RATE = 20.0;
const tally = new Tally();
const timings: string[] = [];
afterAll(() => {
  tally.report("trajectory IK vs Python (worst over both arms)");
  console.log(`solve times (median of 5, method traj):\n${timings.join("\n")}`);
});

type Pose = number[][];
const matmul4 = (a: Pose, b: Pose): Pose => a.map((r) => [0, 1, 2, 3].map((j) => r.reduce((s, v, k) => s + v * b[k]![j]!, 0)));
const steps = (q: number[][], qLast: number[]) => q.map((row, t) => row.map((v, i) => Math.abs(v - (t ? q[t - 1]! : qLast)[i]!)));
const maxAbs = (rows: number[][]) => Math.max(...rows.flat().map(Math.abs));

function pathFrom(ik: TrajectoryIK, q: number[], offset: number[], ticks: number, turn = [0, 0, 0], ease = true): Pose[] {
  const start = ik.chain.fk(q);
  const goal = start.map((r) => [...r]);
  for (let i = 0; i < 3; i++) goal[i]![3]! += offset[i]!;
  const rot = TI.so3Exp(turn);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) goal[i]![j] = [0, 1, 2].reduce((s, k) => s + start[i]![k]! * rot[k]![j]!, 0);
  return TI.straightLine(start, goal, ticks, ease);
}

function median(fn: () => void, repeats = 5): number {
  const times: number[] = [];
  for (let k = 0; k < repeats; k++) {
    const start = performance.now();
    fn();
    times.push((performance.now() - start) / 1000);
  }
  return times.sort((a, b) => a - b)[Math.floor(repeats / 2)]!;
}

// ── the original solver's answers (tests/unit/trajectory_ik_fixtures.json) ──
for (const arm of ARMS) {
  describe(`${arm}: the original solver's answers`, () => {
    const fx: Json = loadFixture(`math_${arm}_trajectory_ik_fixtures.json`);
    const chain = Chain.fromUrdf(fx.urdf, { joints: fx.joints, tip: fx.tip, tcpXyz: fx.tcp_xyz, tcpRpy: fx.tcp_rpy });
    const ik = new TrajectoryIK(chain, { maxVelocity: fx.max_speed, rest: fx.rest_pose, restWeight: fx.rest_weight });
    const poses = (rows: number[][]): Pose[] => rows.map((r) => [r.slice(0, 4), r.slice(4, 8), r.slice(8, 12), [0, 0, 0, 1]]);

    it.each(fx.cases.map((c: Json, i: number) => [`${c.method}-cap${c.speed_cap}-${i}`, c]) as [string, Json][])("reproduces %s", (_, c) => {
      const result = ik.solve(poses(c.targets), c.q_last, { qBefore: c.q_before, rateHz: fx.rate_hz, speedCap: c.speed_cap, method: c.method });
      const diff = result.q.flat().map((v, i) => Math.abs(v - c.q.flat()[i]!)).sort((a, b) => a - b);
      expect(diff[Math.floor(0.95 * (diff.length - 1))]!).toBeLessThan(2e-3);
      expect(diff[diff.length - 1]!).toBeLessThan(1e-2);
    });
  });
}

// ── each arm's profile (port of tests/unit/test_trajectory_ik.py) ──
for (const arm of ARMS) {
  describe(`${arm}: guarantees`, () => {
    const fx: Json = armFixture(arm);
    const profile = configOf(fx);
    const ik = TrajectoryIK.fromConfig(profile);
    const q0 = ik.rest;

    it("fromConfig reads the profile", () => {
      expect(compare(ik.rest, (profile.ik as Json).rest_pose).abs).toBeLessThan(1e-12);
      ik.maxVelocity.forEach((v) => expect(v).toBeLessThanOrEqual(profile.maxVelocity + 1e-12));
      const joints = profile.joints.slice(0, ik.n);
      ik.lo.forEach((v, i) => expect(v).toBeGreaterThanOrEqual(joints[i]!.lo + ik.weights.limitMargin - 1e-12));
      ik.hi.forEach((v, i) => expect(v).toBeLessThanOrEqual(joints[i]!.hi - ik.weights.limitMargin + 1e-12));
    });

    it("reaches a smooth path and reports the error", () => {
      const targets = pathFrom(ik, q0, [0.03, -0.02, 0.02], 40, [0.1, 0.0, 0.1]);
      const result = ik.solve(targets, q0, { rateHz: RATE });
      expect(result.q.length).toBe(40);
      expect(result.capped.some(Boolean)).toBe(false);
      expect(Math.max(...result.posErr)).toBeLessThan(3e-3);
      expect(Math.max(...result.rotErr)).toBeLessThan(0.02);
      const [pos, rot] = TI.toolErrors(ik.chain, result.q, targets);
      expect(compare(pos, result.posErr).abs).toBeLessThan(1e-12);
      expect(compare(rot, result.rotErr).abs).toBeLessThan(1e-12);
    });

    it.each(["traj", "tick"] as const)("speed cap is honoured with the direction kept (%s)", (method) => {
      const targets = pathFrom(ik, q0, [0.15, 0.1, -0.1], 10, [0, 0, 0], false);
      const result = ik.solve(targets, q0, { rateHz: RATE, speedCap: 0.5, method });
      const cap = ik.maxStep(RATE, 0.5);
      steps(result.q, q0).forEach((row) => row.forEach((s, i) => expect(s).toBeLessThanOrEqual(cap[i]! + 1e-12)));
      expect(result.capped.some(Boolean)).toBe(true);
      const t = result.capped.indexOf(true);
      const before = t === 0 ? q0 : result.q[t - 1]!;
      const first = result.q[t]!.map((v, i) => v - before[i]!);
      const want = result.qSolution[t]!.map((v, i) => v - before[i]!);
      const nf = Math.hypot(...first), nw = Math.hypot(...want);
      first.forEach((v, i) => expect(v / nf).toBeCloseTo(want[i]! / nw, 7));
    });

    it("limits are honoured", () => {
      const result = ik.solve(pathFrom(ik, q0, [0.0, 0.0, -2.0], 20), q0, { rateHz: RATE });
      for (const rows of [result.q, result.qSolution]) {
        rows.forEach((r) => r.forEach((v, i) => {
          expect(v).toBeGreaterThanOrEqual(ik.lo[i]! - 1e-12);
          expect(v).toBeLessThanOrEqual(ik.hi[i]! + 1e-12);
        }));
      }
    });

    it("a joint past its margin never goes further", () => {
      const q = [...q0];
      q[0] = ik.hi[0]! + 0.01;
      const targets = Array.from({ length: 10 }, () => ik.chain.fk(q));
      const result = ik.solve(targets, q, { rateHz: RATE });
      result.q.forEach((r) => expect(r[0]).toBeLessThanOrEqual(q[0]! + 1e-12));
    });

    it("unreachable target is reported, not raised", () => {
      const result = ik.solve(pathFrom(ik, q0, [3.0, 0.0, 0.0], 10), q0, { rateHz: RATE });
      expect(result.q.flat().every(Number.isFinite)).toBe(true);
      expect(result.posErr[result.posErr.length - 1]).toBeGreaterThan(1.0);
      const cap = ik.maxStep(RATE);
      steps(result.q, q0).forEach((row) => row.forEach((s, i) => expect(s).toBeLessThanOrEqual(cap[i]! + 1e-12)));
    });

    it("successive chunks continue without a velocity jump", () => {
      const start = ik.chain.fk(q0);
      const targets = TI.straightLine(start, matmul4(start, xyzRpyToPose([0.08, 0.0, 0.05])), 60, false);
      const stream = new IKStream(ik, q0, RATE);
      const a = stream.feed(targets.slice(0, 30)).q;
      const b = stream.feed(targets.slice(30)).q;
      const joined = [q0, ...a, ...b];
      const vel = joined.slice(1).map((r, t) => r.map((v, i) => v - joined[t]![i]!));
      const acc = vel.slice(1).map((r, t) => r.map((v, i) => Math.abs(v - vel[t]![i]!)));
      const seam = Math.max(...acc[29]!);
      expect(seam).toBeLessThan(0.25 * maxAbs(vel.slice(25, 35)) + 1e-4);
      // without the history the solve starts the new chunk as if the arm were still
      const cold = ik.solve(targets.slice(30), a[29]!, { rateHz: RATE }).q;
      expect(Math.max(...cold[0]!.map((v, i) => Math.abs(v - a[29]![i]! - (a[29]![i]! - a[28]![i]!))))).toBeGreaterThan(seam);
    });

    it("step streams single poses under the cap", () => {
      const stream = new IKStream(ik, q0, RATE, 1.0);
      const path = pathFrom(ik, q0, [0.05, 0.05, 0.0], 15);
      const rows = path.map((p) => stream.step(p));
      const cap = ik.maxStep(RATE, 1.0);
      steps(rows, q0).forEach((row) => row.forEach((s, i) => expect(s).toBeLessThanOrEqual(cap[i]! + 1e-12)));
      const end = ik.chain.fk(rows[rows.length - 1]!);
      const goal = path[path.length - 1]!;
      expect(Math.hypot(end[0]![3]! - goal[0]![3]!, end[1]![3]! - goal[1]![3]!, end[2]![3]! - goal[2]![3]!)).toBeLessThan(5e-3);
    });

    it("small tool move near a singularity is a small joint move", () => {
      // The most singular pose one joint away from rest (the elbow folded or stretched to its limit).
      const smallest = (q: number[]) => smallestSingularValue(ik.chain.jacobian(q));
      let best = q0;
      let bestS = Infinity;
      for (let j = 0; j < ik.n; j++) {
        for (let k = 0; k <= 40; k++) {
          const q = [...q0];
          q[j] = ik.lo[j]! + ((ik.hi[j]! - ik.lo[j]!) * k) / 40;
          const s = smallest(q);
          if (s < bestS) {
            bestS = s;
            best = q;
          }
        }
      }
      expect(bestS).toBeLessThan(1e-2);
      const held = new TrajectoryIK(ik.chain, {
        limits: [ik.lo, ik.hi], maxVelocity: ik.maxVelocity, rest: best, weights: ik.weights.replace({ limitMargin: 0.0 }),
      });
      for (const offset of [[0.001, 0, 0], [0, 0.001, 0], [0, 0, 0.001]]) {
        const result = held.solve(pathFrom(ik, best, offset, 20), best, { rateHz: RATE });
        expect(Math.max(...result.q.flat().map((v, e) => Math.abs(v - best[e % ik.n]!)))).toBeLessThan(0.02);
        expect(result.posErr[result.posErr.length - 1]).toBeLessThan(2e-3);
      }
    });

    it("pose residual Jacobian matches finite differences", () => {
      const qs = [0, 1, 2].map((k) => q0.map((v, i) => Math.min(Math.max(v + 0.3 * Math.sin(3 * k + i), ik.lo[i]!), ik.hi[i]!)));
      const [poses, jacs] = ik.chain.fkJacobianMany(qs);
      const targets = poses.map((p) => matmul4(p, xyzRpyToPose([0.02, -0.01, 0.03], [0.3, -0.2, 0.4])));
      const [, J] = TI.poseResidual(targets, poses, jacs);
      for (let j = 0; j < ik.n; j++) {
        const shift = (h: number) => qs.map((q) => q.map((v, i) => (i === j ? v + h : v)));
        const up = TI.poseResidual(targets, ...ik.chain.fkJacobianMany(shift(1e-6)))[0];
        const down = TI.poseResidual(targets, ...ik.chain.fkJacobianMany(shift(-1e-6)))[0];
        up.forEach((r, t) => r.forEach((v, k) => expect(Math.abs((v - down[t]![k]!) / 2e-6 - J[t]![k]![j]!)).toBeLessThan(1e-6)));
      }
    });

    it("batched fk matches fk and jacobian", () => {
      const q = q0.map((v, i) => Math.min(Math.max(v - 0.3 + (0.6 * i) / (ik.n - 1), ik.lo[i]!), ik.hi[i]!));
      const [poses, jacs] = ik.chain.fkJacobianMany([q]);
      expect(compare(poses[0], ik.chain.fk(q)).ulps).toBe(0);
      expect(compare(jacs[0], ik.chain.jacobian(q)).ulps).toBe(0);
    });

    it("profile ik settings are validated", () => {
      for (const bad of [{ nope: 1 }, { pos: -1 }, { max_speed: new Array(99).fill(1.0) }, { max_speed: [0.0] }, { rest_pose: "x" }]) {
        expect(() => configOf(fx, { ik: bad })).toThrow(/ik/);
      }
      expect(() => TrajectoryIK.fromConfig(configOf(fx, { ik: { rest_pose: [0.0] } }))).toThrow(/ik.rest_pose/);
      expect(TrajectoryIK.fromConfig(configOf(fx, { ik: { pos: 20.0 } })).weights.pos).toBe(20.0);
    });

    it("bad inputs are refused", () => {
      expect(() => ik.solve([], q0, { rateHz: RATE })).toThrow(/targets must be/);
      expect(() => ik.solve([[[1, 0, 0, NaN], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]], q0, { rateHz: RATE })).toThrow(/finite/);
      expect(() => ik.solve([ik.chain.fk(q0)], [0], { rateHz: RATE })).toThrow(/q_last/);
      expect(() => ik.maxStep(0)).toThrow(/rate_hz/);
      expect(() => ik.maxStep(RATE, 0)).toThrow(/speed_cap/);
      expect(() => ik.solve([ik.chain.fk(q0)], q0, { rateHz: RATE, method: "x" as "traj" })).toThrow(/method/);
      expect(() => new TrajectoryIK(ik.chain, { limits: [1, 0] })).toThrow(/lower <= upper/);
      expect(() => new TrajectoryIK(ik.chain, { maxVelocity: 0 })).toThrow(/max_velocity/);
      expect(() => TI.fromConfig(profile, { solver: "jax" })).toThrow(ValueError);
      expect(TI.fromConfig(profile, { solver: "numpy" })).toBeInstanceOf(TrajectoryIK);
    });
  });
}

function smallestSingularValue(j: number[][]): number {
  // sqrt of the smallest eigenvalue of J^T J (cyclic Jacobi).
  const n = j[0]!.length;
  const a = Array.from({ length: n }, (_, r) => Array.from({ length: n }, (_, c) => j.reduce((s, row) => s + row[r]! * row[c]!, 0)));
  for (let sweep = 0; sweep < 60; sweep++) {
    let off = 0;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) off += a[p]![q]! ** 2;
    if (off < 1e-30) break;
    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) {
        if (Math.abs(a[p]![q]!) < 1e-300) continue;
        const theta = (a[q]![q]! - a[p]![p]!) / (2 * a[p]![q]!);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1), s = t * c;
        for (let k = 0; k < n; k++) {
          const akp = a[k]![p]!, akq = a[k]![q]!;
          a[k]![p] = c * akp - s * akq;
          a[k]![q] = s * akp + c * akq;
        }
        for (let k = 0; k < n; k++) {
          const apk = a[p]![k]!, aqk = a[q]![k]!;
          a[p]![k] = c * apk - s * aqk;
          a[q]![k] = s * apk + c * aqk;
        }
      }
    }
  }
  return Math.sqrt(Math.max(0, Math.min(...a.map((r, i) => r[i]!))));
}

// ── pieces: parity with Python and the Python tests ──
describe("pieces", () => {
  const common: Json = loadFixture("math_common.json");

  it("rotation log and exp round trip to a half turn", () => {
    const w = [[0, 0, 0], [1e-9, 0, 0], [0.3, -0.2, 0.1], [0, Math.PI - 1e-7, 0], [Math.PI, 0, 0]];
    const back = TI.so3Log(TI.so3Exp(w));
    back.forEach((r, i) => r.forEach((v, k) => expect(Math.abs(Math.abs(v) - Math.abs(w[i]![k]!))).toBeLessThan(1e-6)));
    expect(compare(TI.so3Exp(back), TI.so3Exp(w)).abs).toBeLessThan(1e-9);
  });

  it("savgol keeps a cubic and smooth_poses keeps a line", () => {
    const t = Array.from({ length: 30 }, (_, i) => i / 29);
    const cubic = t.map((v) => [v ** 3 - v, 2 * v ** 2, -v]);
    expect(compare(TI.savgol(cubic, 15, 3), cubic).abs).toBeLessThan(1e-9);
    const line = TI.straightLine(xyzRpyToPose([0, 0, 0]), xyzRpyToPose([0.1, 0, 0], [0, 0, 0.5]), 30, false);
    expect(compare(TI.smoothPoses(line), line).abs).toBeLessThan(1e-9);
    expect(TI.smoothPoses(line.slice(0, 3))).toBeDefined();
    expect(compare(TI.smoothPoses(line, 0), line).abs).toBe(0);
  });

  it("extend_poses continues the last step", () => {
    const eye = xyzRpyToPose([0, 0, 0]);
    const line = TI.straightLine(eye, xyzRpyToPose([0.1, 0, 0], [0, 0, 0.5]), 10, false);
    const longer = TI.straightLine(eye, xyzRpyToPose([0.15, 0, 0], [0, 0, 0.75]), 15, false);
    expect(compare(TI.extendPoses(line, 5), longer).abs).toBeLessThan(1e-12);
  });

  it("seam_blend eases in from the last command", () => {
    const q = Array.from({ length: 10 }, (_, i) => [i / 9, i / 9]);
    const out = TI.seamBlend(q, [0.5, 0.5], 4, -10, 10, [0.2, 0.2]);
    steps(out, [0.5, 0.5]).forEach((r) => r.forEach((s) => expect(s).toBeLessThanOrEqual(0.2 + 1e-12)));
    expect(compare(out.slice(5), q.slice(5)).abs).toBeLessThan(1e-12);
  });

  it("straightLine refuses no ticks", () => {
    expect(() => TI.straightLine(xyzRpyToPose([0, 0, 0]), xyzRpyToPose([0, 0, 0]), 0)).toThrow(/ticks/);
  });

  it("so3Exp, so3Log agree with Python", () => {
    const e = compare(TI.so3Exp(common.so3.w), common.so3.exp);
    const l = compare(TI.so3Log(common.so3.exp), common.so3.log);
    tally.add("so3Exp", e);
    tally.add("so3Log", l);
    expect(e.abs).toBeLessThanOrEqual(1e-15);
    expect(l.abs).toBeLessThanOrEqual(1e-12);
  });

  it("straightLine agrees with Python", () => {
    for (const c of common.straight_line) {
      const d = compare(TI.straightLine(c.start, c.goal, c.ticks, c.ease), c.out);
      tally.add("straightLine", d);
      expect(d.abs).toBeLessThanOrEqual(1e-12);
    }
  });

  it("savgol, smoothPoses, extendPoses, seamBlend agree with Python", () => {
    for (const c of common.savgol) {
      const d = compare(TI.savgol(c.x, c.window, c.polyorder), c.out);
      tally.add("savgol", d);
      expect(d.abs).toBeLessThanOrEqual(1e-12);
    }
    for (const c of common.smooth_poses) {
      const d = compare(TI.smoothPoses(c.T, c.window, c.polyorder), c.out);
      tally.add("smoothPoses", d);
      expect(d.abs).toBeLessThanOrEqual(1e-12);
    }
    for (const c of common.extend_poses) {
      const d = compare(TI.extendPoses(c.T, c.rows, c.k), c.out);
      tally.add("extendPoses", d);
      expect(d.abs).toBeLessThanOrEqual(1e-12);
    }
    for (const c of common.seam_blend) {
      const d = compare(TI.seamBlend(c.q, c.q_from, c.ticks, c.lo, c.hi, c.max_step), c.out);
      tally.add("seamBlend", d);
      expect(d.abs).toBeLessThanOrEqual(1e-12);
    }
  });
});

// ── parity with the Python SDK on each arm's shipped profile ──
// One tick's solve (6 joints) agrees to rounding. The whole-chunk solve does not stop at the same
// rounding: its smoothing cost, x^T H x - 2 g^T x + c with H up to ~1e7, has a rounding floor near
// 1e-8, below the Levenberg-Marquardt stopping test (rtol 1e-10 of a cost near 1), so its last
// accept/stop decisions, and with them the last ~1e-6 rad, follow the rounding of the linear solves
// (LAPACK in numpy) and of the C library's trigonometry.
const TICK_TOL = 1e-12;
const SOLVE_TOL = 1e-5;

for (const arm of ARMS) {
  describe(`${arm}: parity with Python`, () => {
    const fx: Json = armFixture(arm);
    const tr = fx.trajectory;
    const config = configOf(fx);
    const chain = chainFromConfig(config)!;
    const ik = TI.fromConfig(config, { chain });

    const check = (label: string, got: TI.IKResult, want: Json, tol = SOLVE_TOL) => {
      const d = compare(got.q, want.q);
      tally.add(`${label} q`, d);
      tally.add(`${label} qSolution`, compare(got.qSolution, want.q_solution));
      tally.add(`${label} posErr/rotErr`, compare([got.posErr, got.rotErr], [want.pos_err, want.rot_err]));
      expect(got.capped).toEqual(want.capped);
      expect(d.abs).toBeLessThanOrEqual(tol);
      expect(compare(got.qSolution, want.q_solution).abs).toBeLessThanOrEqual(tol);
      expect(compare([got.posErr, got.rotErr], [want.pos_err, want.rot_err]).abs).toBeLessThanOrEqual(tol);
    };

    it("fromConfig settings are the same", () => {
      const info = tr.info;
      expect(compare([ik.lo, ik.hi, ik.maxVelocity, ik.rest, ik.restWeight], [info.lo, info.hi, info.max_velocity, info.rest, info.rest_weight]).ulps).toBe(0);
      expect(ik.free).toBe(info.free);
      const w = ik.weights;
      expect([w.pos, w.ori, w.limit, w.lastPose, w.vel, w.acc, w.jerk, w.history, w.limitMargin]).toEqual(TI.WEIGHT_FIELDS.map((f) => info.weights[f]));
      expect(new IKWeights().limitMargin).toBe(info.weights.limit_margin);
    });

    it("bounds, maxStep and cap are the same", () => {
      expect(compare(ik.bounds(tr.bounds.q_last), tr.bounds.out).ulps).toBe(0);
      for (const c of tr.caps) {
        const step = ik.maxStep(c.rate_hz, c.speed_cap);
        expect(compare(step, c.max_step).ulps).toBe(0);
        const [q, capped] = TrajectoryIK.cap(c.rows, c.q_last, step);
        expect(compare(q, c.q).ulps).toBe(0);
        expect(capped).toEqual(c.capped);
      }
    });

    it("poseResidual and toolErrors agree", () => {
      const p = tr.pose_residual;
      const [r, J] = TI.poseResidual(p.targets, p.poses, p.jacobians);
      const dr = compare(r, p.r), dJ = compare(J, p.J);
      tally.add("poseResidual", dr);
      tally.add("poseResidual", dJ);
      expect(dr.abs).toBeLessThanOrEqual(1e-12);
      expect(dJ.abs).toBeLessThanOrEqual(1e-12);
      const [pos, rot] = TI.toolErrors(chain, p.q, p.targets);
      const de = compare([pos, rot], [p.pos, p.rot]);
      tally.add("toolErrors", de);
      expect(de.abs).toBeLessThanOrEqual(1e-12);
    });

    it("solveTick agrees", () => {
      for (const c of tr.solve_tick) {
        const d = compare(ik.solveTick(c.target, c.q_prev), c.out);
        tally.add("solveTick", d);
        expect(d.abs).toBeLessThanOrEqual(TICK_TOL);
      }
    });

    it.each(tr.solve.map((c: Json) => [c.name, c]) as [string, Json][])("solve agrees: %s", (name, c) => {
      const got = ik.solve(c.targets, c.q_last, { qBefore: c.q_before, rateHz: c.rate_hz, speedCap: c.speed_cap, method: c.method });
      check(`solve ${String(name).startsWith("smooth") ? "(smooth)" : "(edge cases)"}`, got, c);
    });

    it("IKStream.feed over consecutive chunks agrees", () => {
      const stream = new IKStream(ik, tr.info.rest, 20.0);
      for (const c of tr.feed) check("IKStream.feed", stream.feed(c.targets), c);
    });

    it("IKStream.step agrees", () => {
      const stream = new IKStream(ik, tr.info.rest, 20.0, 1.0);
      for (const c of tr.step) {
        const d = compare(stream.step(c.pose), c.out);
        tally.add("IKStream.step", d);
        expect(d.abs).toBeLessThanOrEqual(TICK_TOL);
      }
    });

    it("solvePath agrees", () => {
      for (const c of tr.solve_path) check("solvePath", TI.solvePath(ik, c.poses, c.q_last, c.rate_hz, c.speed_cap), c);
    });

    it("solve time (30 and 48 rows)", () => {
      const s30 = tr.solve.find((c: Json) => c.name === "smooth30");
      const s48 = tr.solve.find((c: Json) => c.name === "smooth48");
      const t30 = median(() => ik.solve(s30.targets, s30.q_last, { rateHz: s30.rate_hz }));
      const t48 = median(() => ik.solve(s48.targets, s48.q_last, { rateHz: s48.rate_hz }));
      timings.push(`  ${arm}: 30 rows ${(t30 * 1e3).toFixed(0)} ms (Python ${(fx.timing.rows30_s * 1e3).toFixed(0)} ms), `
        + `48 rows ${(t48 * 1e3).toFixed(0)} ms (Python ${(fx.timing.rows48_s * 1e3).toFixed(0)} ms)`);
      // The wall-clock time is reported above, not asserted (it depends on the host). Asserted: the
      // solve's estimated work (at most SLICE between pauses) fits moveEe's planning allowance per tick.
      return (async () => {
        let pauses = 0;
        await ik.solveAsync(s48.targets, s48.q_last, { rateHz: s48.rate_hz, pause: async () => void (pauses += 1) });
        expect((pauses + 1) * SLICE * 1e-6).toBeLessThan(s48.targets.length * PLAN_S_PER_TICK);
      })();
    });
  });
}

// ── cooperative (async) solves ──
const fields = (r: unknown): unknown =>
  Array.isArray(r) ? r : [(r as TI.IKResult).q, (r as TI.IKResult).qSolution, (r as TI.IKResult).posErr,
    (r as TI.IKResult).rotErr, (r as TI.IKResult).capped];
const sameBits = (a: unknown, b: unknown) => {
  expect(compare(fields(a), fields(b)).ulps).toBe(0);
};

/** A pause that records the computation time between pauses. */
function stopwatch() {
  let last = performance.now();
  const stretches: number[] = [];
  return {
    stretches,
    pause: async () => {
      stretches.push(performance.now() - last);
      await new Promise<void>((resolve) => setImmediate(resolve));
      last = performance.now();
    },
    end() {
      stretches.push(performance.now() - last);
      return Math.max(...stretches);
    },
  };
}


for (const arm of ARMS) {
  describe(`${arm}: cooperative solves`, () => {
    const fx: Json = armFixture(arm);
    const tr = fx.trajectory;
    const ik = TI.fromConfig(configOf(fx));

    it("solveAsync returns exactly what solve returns", async () => {
      for (const c of tr.solve) {
        const options = { qBefore: c.q_before, rateHz: c.rate_hz, speedCap: c.speed_cap, method: c.method };
        sameBits(await ik.solveAsync(c.targets, c.q_last, options), ik.solve(c.targets, c.q_last, options));
      }
    });

    it("feedAsync and solvePathAsync return exactly what feed and solvePath return", async () => {
      const a = new IKStream(ik, tr.info.rest, 20.0);
      const b = new IKStream(ik, tr.info.rest, 20.0);
      for (const c of tr.feed) {
        sameBits(await a.feedAsync(c.targets), b.feed(c.targets));
        sameBits(a.history, b.history);
      }
      for (const c of tr.solve_path) {
        sameBits(await TI.solvePathAsync(ik, c.poses, c.q_last, c.rate_hz, { speedCap: c.speed_cap }),
                 TI.solvePath(ik, c.poses, c.q_last, c.rate_hz, c.speed_cap));
      }
    });

    it("pauses every few milliseconds of computation", async () => {
      const s48 = tr.solve.find((c: Json) => c.name === "smooth48");
      const path = tr.solve_path[0];
      await ik.solveAsync(s48.targets, s48.q_last, { rateHz: s48.rate_hz }); // warm up the JIT
      const w1 = stopwatch();
      await ik.solveAsync(s48.targets, s48.q_last, { rateHz: s48.rate_hz, pause: w1.pause });
      const longest48 = w1.end();
      const w2 = stopwatch();
      await TI.solvePathAsync(ik, path.poses, path.q_last, path.rate_hz, { pause: w2.pause });
      const longestPath = w2.end();
      timings.push(`  ${arm}: solveAsync 48 rows ${w1.stretches.length - 1} pauses, longest stretch ${longest48.toFixed(1)} ms; `
        + `solvePathAsync ${path.poses.length} poses ${w2.stretches.length - 1} pauses, longest ${longestPath.toFixed(1)} ms`);
      // Stretch lengths depend on the host and are only reported. Pausing depends only on the
      // yielded work estimates: the same solve pauses the same number of times, often enough.
      expect(w1.stretches.length).toBeGreaterThan(10);
      expect(w2.stretches.length - 1).toBeGreaterThanOrEqual(Math.ceil(path.poses.length / 64) - 1); // at least between chunks
      let again = 0;
      await ik.solveAsync(s48.targets, s48.q_last, { rateHz: s48.rate_hz, pause: async () => void (again += 1) });
      expect(again).toBe(w1.stretches.length - 1);
    });

    it("abort rejects with the signal's reason and leaves the stream unchanged", async () => {
      const s48 = tr.solve.find((c: Json) => c.name === "smooth48");
      const controller = new AbortController();
      let pauses = 0;
      const pause = async () => {
        if (++pauses === 3) controller.abort(new Error("stop"));
        await new Promise<void>((resolve) => setImmediate(resolve));
      };
      await expect(ik.solveAsync(s48.targets, s48.q_last, { rateHz: s48.rate_hz, pause, signal: controller.signal }))
        .rejects.toThrow("stop");
      expect(pauses).toBe(3);
      const aborted = AbortSignal.abort(new Error("already"));
      await expect(ik.solveAsync(s48.targets, s48.q_last, { rateHz: s48.rate_hz, signal: aborted })).rejects.toThrow("already");
      const stream = new IKStream(ik, tr.info.rest, 20.0);
      const history = structuredClone(stream.history);
      await expect(stream.feedAsync(tr.feed[0].targets, { signal: aborted })).rejects.toThrow("already");
      expect(stream.history).toEqual(history);
      const path = tr.solve_path[0];
      await expect(TI.solvePathAsync(ik, path.poses, path.q_last, path.rate_hz, { signal: aborted })).rejects.toThrow("already");
    });

    it("bad input rejects the same way solve throws", async () => {
      await expect(ik.solveAsync([], tr.info.rest, { rateHz: RATE })).rejects.toThrow(/targets must be/);
    });
  });
}
