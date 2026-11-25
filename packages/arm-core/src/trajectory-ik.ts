/**
 * Trajectory inverse kinematics: tool poses to joint commands with a bounded step per tick.
 *
 * The algorithm is Dream Labs' `dream_ik` (Apache-2.0), solved here on this package's `Chain`.
 *
 * Per tick: reach the target tool pose (residual `log(target^-1 @ tool)`, position : orientation
 * weights 50 : 10), a stiff push back inside the margined joint limits, a light pull toward the
 * rest pose and a pull toward the previous tick's solution, by Levenberg-Marquardt from that
 * solution. Whole chunk (`method: "traj"`, the default): every tick in one least-squares problem
 * with the same pose, limit and rest costs plus joint velocity, acceleration and jerk along the
 * chunk and into it from the last commands, warm-started from the per-tick solve. Either way the
 * answer is clipped to the margined limits, then the command steps toward it under the per-joint
 * speed caps, the step scaled by its most-limited joint so its direction is kept: no tick moves a
 * joint more than `maxSpeed / rateHz`.
 *
 * Poses are 4x4 tool poses (`number[][]`) in the chain's world frame, stacks of them
 * `number[][][]`; joints are the chain's joint coordinates (rad), trajectories `number[][]`.
 */

import type { ArmConfig } from "./config.js";
import { ValueError } from "./errors.js";
import { type CooperativeOptions, macrotask, run, runAsync, type Work } from "./cooperate.js";
import { type Chain, chainFromConfig } from "./kinematics.js";
import {
  clip, ddot, dotEinsum3, fma, gemvRow, mat3Mul, norm3, npMaximum, npMinimum, pairwiseSum, projection, solveWork as linSolveWork,
} from "./linalg.js";

/** The trajectory solve's orientation weight on an arm with fewer than six free joints, as a share of its own. */
export const FEW_JOINTS_ORI_SCALE = 0.5;

/** Profile `ik:` keys that set weights (the snake_case names of IKWeights' fields). */
export const WEIGHT_FIELDS = ["pos", "ori", "limit", "last_pose", "vel", "acc", "jerk", "history", "limit_margin"] as const;
/** Per-joint profile `ik:` entries next to the weights. */
export const JOINT_FIELDS = ["rest_pose", "rest_weight", "max_speed"] as const;

const WEIGHT_KEYS = {
  pos: "pos", ori: "ori", limit: "limit", last_pose: "lastPose", vel: "vel", acc: "acc", jerk: "jerk",
  history: "history", limit_margin: "limitMargin",
} as const satisfies Record<(typeof WEIGHT_FIELDS)[number], string>;

/** Fields of IKWeights. */
export interface IKWeightsInit {
  pos: number;
  ori: number;
  limit: number;
  lastPose: number;
  vel: number;
  acc: number;
  jerk: number;
  history: number;
  limitMargin: number;
}

/**
 * Cost weights and the limit margin. `vel`, `acc`, `jerk` act on joint differences between ticks
 * (rad); `history` multiplies them where a chunk meets the commands already sent. Immutable.
 */
export class IKWeights implements IKWeightsInit {
  readonly pos: number = 50.0;
  readonly ori: number = 10.0;
  readonly limit: number = 1000.0;
  readonly lastPose: number = 1.0;
  readonly vel: number = 5.0;
  readonly acc: number = 50.0;
  readonly jerk: number = 200.0;
  readonly history: number = 5.0;
  /** rad (2 degrees). */
  readonly limitMargin: number = 2.0 * (Math.PI / 180);

  constructor(init: Partial<IKWeightsInit> = {}) {
    Object.assign(this, init);
    Object.freeze(this);
  }

  /** A copy with `changes` applied. */
  replace(changes: Partial<IKWeightsInit>): IKWeights {
    return new IKWeights({ ...this, ...changes });
  }
}

/** The result of a trajectory solve. */
export interface IKResult {
  /** (T, dof) joint commands. */
  readonly q: number[][];
  /** (T, dof) the solve's answer before the speed cap. */
  readonly qSolution: number[][];
  /** (T,) m, commanded tool position vs target. */
  readonly posErr: number[];
  /** (T,) rad. */
  readonly rotErr: number[];
  /** (T,) the speed cap shortened this tick's step. */
  readonly capped: boolean[];
}

// ── rotations (flat row-major 3x3) ──
function skew(x: number, y: number, z: number): Float64Array {
  return Float64Array.of(0.0, -z, y, z, 0.0, -x, -y, x, 0.0);
}

function exp3(w0: number, w1: number, w2: number): Float64Array {
  const theta = norm3(w0, w1, w2);
  const small = theta < 1e-8;
  const t = small ? 1.0 : theta;
  const a = small ? 1.0 - (theta * theta) / 6.0 : Math.sin(t) / t;
  const b = small ? 0.5 - (theta * theta) / 24.0 : (1.0 - Math.cos(t)) / (t * t);
  const k = skew(w0, w1, w2);
  const kk = mat3Mul(k, k);
  const out = new Float64Array(9);
  for (let e = 0; e < 9; e++) out[e] = ((e % 4 === 0 ? 1 : 0) + a * k[e]!) + b * kk[e]!;
  return out;
}

function quaternionLog(r: ArrayLike<number>, o: number): [number, number, number] {
  const m00 = r[o]!, m01 = r[o + 1]!, m02 = r[o + 2]!;
  const m10 = r[o + 3]!, m11 = r[o + 4]!, m12 = r[o + 5]!;
  const m20 = r[o + 6]!, m21 = r[o + 7]!, m22 = r[o + 8]!;
  // Each of the four candidates is well conditioned where its component is the largest.
  const cand = [
    [((1 + m00) + m11) + m22, m21 - m12, m02 - m20, m10 - m01],
    [m21 - m12, ((1 + m00) - m11) - m22, m01 + m10, m02 + m20],
    [m02 - m20, m01 + m10, ((1 - m00) + m11) - m22, m12 + m21],
    [m10 - m01, m02 + m20, m12 + m21, ((1 - m00) - m11) + m22],
  ];
  const keys = [(m00 + m11) + m22, m00, m11, m22];
  let pick = 0;
  // np.argmax: the first largest, a NaN counting as largest
  for (let i = 1; i < 4; i++) {
    if (Number.isNaN(keys[pick]!)) break;
    if (keys[i]! > keys[pick]! || Number.isNaN(keys[i]!)) pick = i;
  }
  let q = cand[pick]!;
  const qn = Math.sqrt(((q[0]! * q[0]! + q[1]! * q[1]!) + q[2]! * q[2]!) + q[3]! * q[3]!);
  q = q.map((v) => v / qn);
  if (q[0]! < 0) q = q.map((v) => -v);
  const n = norm3(q[1]!, q[2]!, q[3]!);
  const angle = 2.0 * Math.atan2(n, q[0]!);
  const f = n < 1e-12 ? 2.0 : angle / n;
  return [f * q[1]!, f * q[2]!, f * q[3]!];
}

function log3(r: ArrayLike<number>, o = 0): [number, number, number] {
  const v0 = r[o + 7]! - r[o + 5]!;
  const v1 = r[o + 2]! - r[o + 6]!;
  const v2 = r[o + 3]! - r[o + 1]!;
  const s = norm3(v0, v1, v2) / 2.0;
  const c = (((r[o]! + r[o + 4]!) + r[o + 8]!) - 1.0) / 2.0;
  if (c < -0.9) return quaternionLog(r, o); // the axis from vee is ill conditioned there
  const angle = Math.atan2(s, c);
  const f = s < 1e-9 ? 0.5 : angle / (2.0 * (s < 1e-9 ? 1.0 : s));
  return [v0 * f, v1 * f, v2 * f];
}

function rows3(m: readonly (readonly number[])[]): Float64Array {
  return Float64Array.of(m[0]![0]!, m[0]![1]!, m[0]![2]!, m[1]![0]!, m[1]![1]!, m[1]![2]!, m[2]![0]!, m[2]![1]!, m[2]![2]!);
}

function nested3(r: ArrayLike<number>, o = 0): number[][] {
  return [[r[o]!, r[o + 1]!, r[o + 2]!], [r[o + 3]!, r[o + 4]!, r[o + 5]!], [r[o + 6]!, r[o + 7]!, r[o + 8]!]];
}

/** a^T @ b of row-major 3x3 values (the transpose read in place). */
function mulTA(a: ArrayLike<number>, ao: number, b: ArrayLike<number>, bo: number): Float64Array {
  const out = new Float64Array(9);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      out[3 * i + j] = fma(a[ao + 6 + i]!, b[bo + 6 + j]!, fma(a[ao + 3 + i]!, b[bo + 3 + j]!, a[ao + i]! * b[bo + j]!));
    }
  }
  return out;
}

/** a @ b^T of row-major 3x3 values. */
function mulBT(a: ArrayLike<number>, ao: number, b: ArrayLike<number>, bo: number): Float64Array {
  const out = new Float64Array(9);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      out[3 * i + j] = fma(a[ao + 3 * i + 2]!, b[bo + 3 * j + 2]!,
                           fma(a[ao + 3 * i + 1]!, b[bo + 3 * j + 1]!, a[ao + 3 * i]! * b[bo + 3 * j]!));
    }
  }
  return out;
}

/** (3,) rotation vector -> 3x3, or (B, 3) -> (B, 3, 3). */
export function so3Exp(w: readonly number[]): number[][];
export function so3Exp(w: readonly (readonly number[])[]): number[][][];
export function so3Exp(w: readonly number[] | readonly (readonly number[])[]): number[][] | number[][][] {
  if (Array.isArray(w[0])) return (w as readonly (readonly number[])[]).map((v) => nested3(exp3(v[0]!, v[1]!, v[2]!)));
  const v = w as readonly number[];
  return nested3(exp3(v[0]!, v[1]!, v[2]!));
}

/** 3x3 -> (3,) rotation vector, angle in [0, pi]; or (B, 3, 3) -> (B, 3). */
export function so3Log(r: readonly (readonly number[])[]): number[];
export function so3Log(r: readonly (readonly (readonly number[])[])[]): number[][];
export function so3Log(r: readonly (readonly number[])[] | readonly (readonly (readonly number[])[])[]): number[] | number[][] {
  if (Array.isArray(r[0]![0])) return (r as readonly (readonly (readonly number[])[])[]).map((m) => log3(rows3(m)));
  return log3(rows3(r as readonly (readonly number[])[]));
}

/**
 * The twist [v, w] of a pose (R, p) and jlog, d log(pose @ exp(xi)) / d xi at xi = 0 (the formulas
 * of jaxlie's SE3.log / SE3.jlog). `jlog` is written as 6x6 row-major.
 */
function se3Log(rot: Float64Array, p: readonly number[], twist: Float64Array, jlog: Float64Array): void {
  const w = log3(rot);
  const theta2 = dotEinsum3(w[0], w[1], w[2], w[0], w[1], w[2]);
  const small = theta2 < 1e-12;
  const theta = Math.sqrt(small ? 1.0 : theta2);
  const st = Math.sin(theta);
  const ct = Math.cos(theta);
  const half = small ? 0.5 : 1.0 / (2.0 * (1.0 - ct));
  // V(w)^-1 = I - W / 2 + c W^2, and jlog of SO(3) is its transpose
  const c = small ? 1.0 / 12.0 : (1.0 - theta * st * half) / (small ? 1.0 : theta2);
  const W = skew(w[0], w[1], w[2]);
  const WW = mat3Mul(W, W);
  const vInv = new Float64Array(9);
  for (let e = 0; e < 9; e++) vInv[e] = ((e % 4 === 0 ? 1 : 0) - 0.5 * W[e]!) + c * WW[e]!;
  for (let i = 0; i < 3; i++) {
    twist[i] = dotEinsum3(vInv[3 * i]!, vInv[3 * i + 1]!, vInv[3 * i + 2]!, p[0]!, p[1]!, p[2]!);
    twist[3 + i] = w[i]!;
  }
  const inv2 = 1.0 / (theta * theta);
  const beta = inv2 - (st / theta) * half;
  const betaDot = -2.0 * (inv2 * inv2) + (1.0 + st / theta) * inv2 * half;
  const wp = dotEinsum3(w[0], w[1], w[2], p[0]!, p[1]!, p[2]!);
  const k1 = betaDot * wp;
  const k2 = theta2 * betaDot + 2.0 * beta;
  const v3 = [0, 1, 2].map((i) => k1 * w[i]! - k2 * p[i]!);
  const P = skew(p[0]!, p[1]!, p[2]!).map((v) => 0.5 * v);
  const wpb = wp * beta;
  jlog.fill(0);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      jlog[6 * i + j] = vInv[3 * j + i]!;
      jlog[6 * (3 + i) + 3 + j] = vInv[3 * j + i]!;
    }
  }
  if (small) {
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) jlog[6 * i + 3 + j] = P[3 * i + j]!;
    return;
  }
  const C = new Float64Array(9);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      C[3 * i + j] = ((v3[i]! * w[j]! + beta * (w[i]! * p[j]!)) + wpb * (i === j ? 1 : 0)) + P[3 * i + j]!;
    }
  }
  // C @ jlog_so3, jlog_so3 = vInv^T
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      jlog[6 * i + 3 + j] = fma(C[3 * i + 2]!, vInv[3 * j + 2]!, fma(C[3 * i + 1]!, vInv[3 * j + 1]!, C[3 * i]! * vInv[3 * j]!));
    }
  }
}

/**
 * Residual and its joint Jacobian on flat arrays: targets and poses (T*12: rotation row-major,
 * then position), Jacobians (T*6*N) -> r (T*6), J (T*6*N).
 */
function poseResidualFlat(targets: Float64Array, poses: Float64Array, jacobians: Float64Array, T: number, n: number):
    { r: Float64Array; J: Float64Array } {
  const r = new Float64Array(6 * T);
  const J = new Float64Array(6 * n * T);
  const twist = new Float64Array(6);
  const jlog = new Float64Array(36);
  const body = new Float64Array(6 * n);
  for (let t = 0; t < T; t++) {
    const to = 12 * t;
    const rErr = mulTA(targets, to, poses, to);
    const d = [0, 1, 2].map((i) => poses[to + 9 + i]! - targets[to + 9 + i]!);
    const pErr = [0, 1, 2].map((i) =>
      fma(targets[to + 6 + i]!, d[2]!, fma(targets[to + 3 + i]!, d[1]!, targets[to + i]! * d[0]!)));
    se3Log(rErr, pErr, twist, jlog);
    r.set(twist, 6 * t);
    // body = [Rt_tool @ jac[:3], Rt_tool @ jac[3:]]
    const jo = 6 * n * t;
    for (let half = 0; half < 2; half++) {
      for (let i = 0; i < 3; i++) {
        for (let c = 0; c < n; c++) {
          const base = jo + 3 * half * n;
          body[(3 * half + i) * n + c] = fma(poses[to + 6 + i]!, jacobians[base + 2 * n + c]!,
            fma(poses[to + 3 + i]!, jacobians[base + n + c]!, poses[to + i]! * jacobians[base + c]!));
        }
      }
    }
    for (let i = 0; i < 6; i++) {
      for (let c = 0; c < n; c++) {
        let s = jlog[6 * i]! * body[c]!;
        for (let k = 1; k < 6; k++) s = fma(jlog[6 * i + k]!, body[k * n + c]!, s);
        J[jo + i * n + c] = s;
      }
    }
  }
  return { r, J };
}

function flatPoses(poses: readonly (readonly (readonly number[])[])[]): Float64Array {
  const out = new Float64Array(12 * poses.length);
  poses.forEach((p, t) => {
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) out[12 * t + 3 * i + j] = p[i]![j]!;
      out[12 * t + 9 + i] = p[i]![3]!;
    }
  });
  return out;
}

/**
 * `log(target^-1 @ pose)` (T, 6) and its Jacobian (T, 6, N) in the joints, from the chain's
 * world-frame Jacobians (T, 6, N).
 */
export function poseResidual(targets: readonly (readonly (readonly number[])[])[], poses: readonly (readonly (readonly number[])[])[],
                             jacobians: readonly (readonly (readonly number[])[])[]): [number[][], number[][][]] {
  const T = targets.length;
  const n = jacobians[0]?.[0]?.length ?? 0;
  const { r, J } = poseResidualFlat(flatPoses(targets), flatPoses(poses), Float64Array.from(jacobians.flat(2)), T, n);
  return [
    Array.from({ length: T }, (_, t) => Array.from(r.subarray(6 * t, 6 * t + 6))),
    Array.from({ length: T }, (_, t) => Array.from({ length: 6 }, (_, i) => Array.from(J.subarray(6 * n * t + i * n, 6 * n * t + (i + 1) * n)))),
  ];
}

function toolErrorsFlat(chain: Chain, q: Float64Array, T: number, targets: Float64Array): [number[], number[]] {
  return run(toolErrorsWork(chain, q, T, targets));
}

function* toolErrorsWork(chain: Chain, q: Float64Array, T: number, targets: Float64Array): Work<[number[], number[]]> {
  const n = chain.n;
  const poses = new Float64Array(12 * T);
  for (let t0 = 0; t0 < T; t0 += FK_ROWS_PER_YIELD) {
    const rows = Math.min(FK_ROWS_PER_YIELD, T - t0);
    poses.set(chain.fkJacobianFlat(q.subarray(t0 * n, (t0 + rows) * n), rows).poses, 12 * t0);
    yield FK_ROW_WORK * rows;
  }
  const pos: number[] = [];
  const rot: number[] = [];
  for (let t = 0; t < T; t++) {
    const o = 12 * t;
    pos.push(norm3(poses[o + 9]! - targets[o + 9]!, poses[o + 10]! - targets[o + 10]!, poses[o + 11]! - targets[o + 11]!));
    const w = log3(mulBT(poses, o, targets, o));
    rot.push(norm3(w[0], w[1], w[2]));
  }
  return [pos, rot];
}

/** Position (m) and angle (rad) of the tool at each row of q (T, N) from targets (T, 4, 4). */
export function toolErrors(chain: Chain, q: readonly (readonly number[])[], targets: readonly (readonly (readonly number[])[])[]): [number[], number[]] {
  return toolErrorsFlat(chain, Float64Array.from(q.flat()), q.length, flatPoses(targets));
}

// ── the least-squares solver ──
interface Evaluation {
  cost: number;
  g: Float64Array;
  H: Float64Array;
}
type Evaluate = (x: Float64Array) => Work<Evaluation>;

// Work estimates (µs on this class of CPU) yielded to the cooperative runner, see cooperate.ts.
/** Rows per batch of forward kinematics between yields. */
const FK_ROWS_PER_YIELD = 16;
const sweepWork = (N: number): number => 1 + 0.01 * N * N;
const FK_ROW_WORK = 70;
const RESIDUAL_ROW_WORK = 50;

const npMax0 = (v: number): number => (Number.isNaN(v) ? NaN : v > 0.0 ? v : 0.0);

/**
 * Minimise a sum of squares from `x`: `evaluate(x) -> (cost, J^T r, J^T J)` of the smooth costs,
 * plus the stiff limit cost `weight * max(0, x - hi)` and `weight * max(0, lo - x)` per coordinate.
 * Marquardt damping; a step that would cross a limit is re-solved with that limit's (linear)
 * residual in the model.
 */
function* levenbergMarquardt(x: Float64Array, evaluate: Evaluate, lo: Float64Array, hi: Float64Array, weight: number,
                             maxIters = 100, rtol = 1e-10, lam = 1e-3): Work<Float64Array> {
  const N = x.length;
  const w2 = weight * weight;
  const over = new Float64Array(N);
  const under = new Float64Array(N);
  const total = function* (x: Float64Array): Work<Evaluation> {
    const { cost, g, H } = yield* evaluate(x);
    for (let i = 0; i < N; i++) {
      over[i] = npMax0(x[i]! - hi[i]!);
      under[i] = npMax0(lo[i]! - x[i]!);
    }
    const gOut = new Float64Array(N);
    for (let i = 0; i < N; i++) gOut[i] = g[i]! + w2 * (over[i]! - under[i]!);
    return { cost: cost + w2 * (ddot(over, over, N) + ddot(under, under, N)), g: gOut, H };
  };

  let { cost, g, H } = yield* total(x);
  const diag = new Float64Array(N);
  const upper = new Uint8Array(N);
  const lower = new Uint8Array(N);
  const rhs = new Float64Array(N);
  const g2 = new Float64Array(N);
  const dxH = new Float64Array(N);
  for (let iter = 0; iter < maxIters; iter++) {
    for (let i = 0; i < N; i++) {
      diag[i] = H[i * N + i]! + 1e-12;
      upper[i] = x[i]! > hi[i]! ? 1 : 0;
      lower[i] = x[i]! < lo[i]! ? 1 : 0;
    }
    let dx: Float64Array = new Float64Array(N);
    for (let attempt = 0; attempt < 4; attempt++) {
      // grow the set of limits the step meets until it stops crossing new ones
      const A = Float64Array.from(H);
      for (let i = 0; i < N; i++) {
        const active = upper[i]! | lower[i]!;
        const wa = w2 * active;
        A[i * N + i]! += lam * (diag[i]! + wa) + wa;
        const bound = upper[i] ? hi[i]! : lo[i]!;
        rhs[i] = -((g[i]! - w2 * (npMax0(x[i]! - hi[i]!) - npMax0(lo[i]! - x[i]!))) + wa * (x[i]! - bound));
      }
      dx = yield* linSolveWork(A, N, rhs);
      let grew = false;
      for (let i = 0; i < N; i++) {
        if (upper[i]! | lower[i]!) continue;
        const v = x[i]! + dx[i]!;
        if (v > hi[i]!) {
          upper[i] = 1;
          grew = true;
        }
        if (v < lo[i]!) {
          lower[i] = 1;
          grew = true;
        }
      }
      if (!grew) break;
    }
    // converged: the model gains nothing more
    for (let i = 0; i < N; i++) g2[i] = 2.0 * g[i]!;
    for (let j = 0; j < N; j++) {
      let s = 0.0;
      let started = false;
      for (let i = 0; i < N; i++) {
        const h = H[i * N + j]!;
        if (h === 0 && started) continue;
        s = started ? fma(dx[i]!, h, s) : dx[i]! * h;
        started = true;
      }
      dxH[j] = s;
    }
    yield sweepWork(N);
    if (-(ddot(g2, dx, N) + ddot(dxH, dx, N)) <= rtol * cost) break;
    const xNew = new Float64Array(N);
    for (let i = 0; i < N; i++) xNew[i] = x[i]! + dx[i]!;
    const next = yield* total(xNew);
    if (next.cost < cost) {
      const done = cost - next.cost <= rtol * cost;
      x = xNew;
      ({ cost, g, H } = next);
      lam = Math.max(lam * 0.1, 1e-9);
      if (done) break;
    } else {
      lam *= 10.0;
      if (lam > 1e10) break;
    }
  }
  return x;
}

const broadcast = (v: number | readonly number[], n: number, what: string): number[] => {
  if (typeof v === "number") return new Array<number>(n).fill(v);
  if (v.length === 1) return new Array<number>(n).fill(Number(v[0]));
  if (v.length !== n) throw new ValueError(`${what}: cannot broadcast ${v.length} values to (${n},)`);
  return v.map(Number);
};

const npMin = npMinimum;
const npMax = npMaximum;

/** Options of the TrajectoryIK constructor. */
export interface TrajectoryIKOptions {
  /** [lower, upper] in joint coordinates (default: the chain's URDF limits). */
  limits?: readonly [number | readonly number[], number | readonly number[]] | null;
  /** rad/s per joint, or one value. */
  maxVelocity?: number | readonly number[];
  /** The pose the solve leans toward (default: zeros, clipped into the limits). */
  rest?: readonly number[] | null;
  weights?: IKWeights;
  /** Per joint (default 0.15 on the first joint, 0.3 on the others). */
  restWeight?: number | readonly number[] | null;
}

/** Options of `TrajectoryIK.solve`. */
export interface SolveOptions {
  /** The commands before qLast, (dof,) or (rows, dof) newest first; two rows also set the jerk into the chunk; null = the arm was still. */
  qBefore?: readonly number[] | readonly (readonly number[])[] | null;
  rateHz: number;
  /** rad/s, lowers the arm's speeds. */
  speedCap?: number | null;
  /** "traj" solves the chunk as one problem, "tick" tick by tick. */
  method?: "traj" | "tick";
}

/**
 * Joint commands for tool poses on `chain`. `limits` [lower, upper] in joint coordinates (default:
 * the chain's URDF limits), `weights.limitMargin` is kept inside them; `maxVelocity` rad/s per joint
 * (or one value); `rest` the pose the solve leans toward (default: zeros, clipped into the limits),
 * with `restWeight` per joint (default 0.15 on the first joint, 0.3 on the others).
 */
export class TrajectoryIK {
  readonly chain: Chain;
  readonly n: number;
  readonly weights: IKWeights;
  /** Margined limits. */
  readonly lo: number[];
  readonly hi: number[];
  /** rad/s per joint. */
  readonly maxVelocity: number[];
  readonly rest: number[];
  readonly restWeight: number[];
  /** Joints with room to move (hi > lo). */
  readonly free: number;

  constructor(chain: Chain, options: TrajectoryIKOptions = {}) {
    const { limits = null, maxVelocity = Infinity, rest = null, weights = new IKWeights(), restWeight = null } = options;
    const n = chain.n;
    this.chain = chain;
    this.n = n;
    this.weights = weights;
    const lower = broadcast(limits === null ? chain.lower : limits[0], n, "limits");
    const upper = broadcast(limits === null ? chain.upper : limits[1], n, "limits");
    if (lower.some((v, i) => v > upper[i]!)) {
      throw new ValueError(`limits need lower <= upper, got ${JSON.stringify(lower)} ${JSON.stringify(upper)}`);
    }
    const lo = lower.map((v) => v + weights.limitMargin);
    const hi = upper.map((v) => v - weights.limitMargin);
    const mid = lower.map((v, i) => (v + upper[i]!) / 2);
    this.lo = lo.map((v, i) => (v > hi[i]! ? mid[i]! : v));
    this.hi = hi.map((v, i) => (lo[i]! > v ? mid[i]! : v));
    this.maxVelocity = broadcast(maxVelocity, n, "maxVelocity");
    if (!this.maxVelocity.every((v) => v > 0)) {
      throw new ValueError(`max_velocity must be >0, got ${JSON.stringify(this.maxVelocity)}`);
    }
    const r = rest === null ? new Array<number>(n).fill(0) : rest.map(Number);
    if (r.length !== n) throw new ValueError(`rest needs ${n} values`);
    this.rest = r.map((v, i) => clip(v, this.lo[i]!, this.hi[i]!));
    this.restWeight = restWeight === null
      ? [0.15, ...new Array<number>(Math.max(0, n - 1)).fill(0.3)].slice(0, n)
      : broadcast(restWeight, n, "restWeight");
    this.free = this.hi.filter((v, i) => v > this.lo[i]!).length;
  }

  /**
   * From an arm profile: its chain (or `chain`, built from it), its soft limits less `limitMargin`
   * (the ranges the arm is driven within; the URDF's own limits are not read), its `maxVelocity`
   * and its `ik` mapping (weights, `rest_pose`, `rest_weight`, `max_speed` per joint, each no higher
   * than `maxVelocity`).
   */
  static fromConfig(config: ArmConfig, options: { weights?: IKWeights | null; chain?: Chain | null } = {}): TrajectoryIK {
    const chain = options.chain ?? chainFromConfig(config);
    if (chain === null) throw new ValueError("the profile names no urdf");
    const n = chain.n;
    const joints = config.joints.slice(0, n);
    const lower = joints.map((j) => j.lo + config.limitMargin);
    const upper = joints.map((j) => j.hi - config.limitMargin);
    const ik: Record<string, unknown> = { ...(config.ik ?? {}) };
    for (const name of JOINT_FIELDS) {
      if (name in ik && (ik[name] as unknown[]).length !== n) {
        throw new ValueError(`ik.${name} needs ${n} values, got ${(ik[name] as unknown[]).length}`);
      }
    }
    let weights = options.weights ?? null;
    if (weights === null) {
      const changes: Partial<Record<keyof IKWeightsInit, number>> = {};
      for (const [k, v] of Object.entries(ik)) {
        if (!(WEIGHT_FIELDS as readonly string[]).includes(k)) continue;
        if (typeof v !== "number") throw new ValueError(`ik.${k} must be a number, got ${JSON.stringify(v)}`);
        changes[WEIGHT_KEYS[k as keyof typeof WEIGHT_KEYS]] = v;
      }
      weights = new IKWeights(changes);
    }
    let rest = (ik.rest_pose as number[] | undefined) ?? null;
    if (rest === null && config.restPose && config.restPose.length) rest = config.restPose.slice(0, n);
    const maxSpeed = (ik.max_speed as number[] | undefined) ?? new Array<number>(n).fill(config.maxVelocity);
    const speed = maxSpeed.map((v) => npMin(Number(v), config.maxVelocity));
    return new TrajectoryIK(chain, {
      limits: [lower, upper], maxVelocity: speed, rest, weights, restWeight: (ik.rest_weight as number[] | undefined) ?? null,
    });
  }

  // ── limits and caps ──
  /** Margined limits from `qLast`: a joint already past its margin may stay there, never go further. */
  bounds(qLast: readonly number[]): [number[], number[]] {
    return [this.lo.map((v, i) => npMin(v, Number(qLast[i]))), this.hi.map((v, i) => npMax(v, Number(qLast[i])))];
  }

  /** (dof,) rad per tick: the arm's speeds, or `speedCap` (rad/s) where lower. */
  maxStep(rateHz: number, speedCap: number | null = null): number[] {
    if (!(rateHz > 0)) throw new ValueError(`rate_hz must be >0, got ${rateHz}`);
    const cap = speedCap === null ? Infinity : Number(speedCap);
    if (!(cap > 0)) throw new ValueError(`speed_cap must be >0, got ${speedCap}`);
    return this.maxVelocity.map((v) => npMin(v, cap) / rateHz);
  }

  /** Step toward each row from the command before, the step scaled by its most-limited joint. */
  static cap(qSolution: readonly (readonly number[])[], qLast: readonly number[], maxStep: readonly number[]): [number[][], boolean[]] {
    const q: number[][] = [];
    const capped: boolean[] = [];
    let prev = qLast.map(Number);
    for (const target of qSolution) {
      const step = target.map((v, i) => v - prev[i]!);
      let m = Infinity;
      let nan = false;
      step.forEach((s, i) => {
        const r = maxStep[i]! / npMax(Math.abs(s), 1e-12);
        if (Number.isNaN(r)) nan = true;
        else if (r < m) m = r;
      });
      const smallest = nan ? NaN : m;
      const scale = smallest < 1.0 ? smallest : 1.0; // Python's min(1.0, x)
      prev = prev.map((v, i) => v + scale * step[i]!);
      q.push(prev);
      capped.push(scale < 1.0);
    }
    return [q, capped];
  }

  #checkInputs(targets: readonly (readonly (readonly number[])[])[], qLast: readonly number[]): [Float64Array, number[]] {
    if (!Array.isArray(targets) || !targets.length
        || !targets.every((p) => Array.isArray(p) && p.length === 4 && p.every((r) => Array.isArray(r) && r.length === 4))) {
      throw new ValueError(`targets must be (T, 4, 4), got ${shape(targets)}`);
    }
    if (!targets.every((p: readonly (readonly number[])[]) => p.every((r) => r.every((v) => Number.isFinite(v))))) {
      throw new ValueError("targets must be finite");
    }
    if (qLast.length !== this.n || !qLast.every((v) => Number.isFinite(v))) {
      throw new ValueError(`q_last must be ${this.n} finite values`);
    }
    return [flatPoses(targets), qLast.map(Number)];
  }

  // ── the solves ──
  /**
   * One pose from the previous tick's solution `qPrev`: the per-tick costs, clipped to [lo, hi]
   * (default: `bounds(qPrev)`).
   */
  solveTick(target: readonly (readonly number[])[], qPrev: readonly number[], lo: readonly number[] | null = null,
            hi: readonly number[] | null = null): number[] {
    return Array.from(run(this.#solveTick(flatPoses([target]), 0, qPrev.map(Number), lo, hi)));
  }

  *#solveTick(targets: Float64Array, t: number, qPrev: number[], lo: readonly number[] | null, hi: readonly number[] | null): Work<Float64Array> {
    const w = this.weights;
    const n = this.n;
    if (lo === null || hi === null) [lo, hi] = this.bounds(qPrev);
    const target = targets.subarray(12 * t, 12 * t + 12);
    const scale = [w.pos, w.pos, w.pos, w.ori, w.ori, w.ori];
    const rw = this.restWeight;
    const lp2 = w.lastPose * w.lastPose;
    const diag = rw.map((v) => v * v + lp2);
    const pull = rw.map((v, i) => v * v * this.rest[i]! + lp2 * qPrev[i]!);
    const evaluate: Evaluate = function* (this: TrajectoryIK, q: Float64Array): Work<Evaluation> {
      const { poses, jacobians } = this.chain.fkJacobianFlat(q, 1);
      const res = poseResidualFlat(target, poses, jacobians, 1, n);
      const r = res.r.map((v, k) => scale[k]! * v);
      const J = res.J.map((v, e) => scale[Math.floor(e / n)]! * v);
      const restTerms = new Float64Array(n);
      const lastTerms = new Float64Array(n);
      for (let i = 0; i < n; i++) {
        const a = rw[i]! * (q[i]! - this.rest[i]!);
        restTerms[i] = a * a;
        const b = w.lastPose * (q[i]! - qPrev[i]!);
        lastTerms[i] = b * b;
      }
      const cost = (ddot(r, r, 6) + pairwiseSum(restTerms)) + pairwiseSum(lastTerms);
      const g = new Float64Array(n);
      const H = new Float64Array(n * n);
      for (let i = 0; i < n; i++) {
        let s = J[i]! * r[0]!;
        for (let k = 1; k < 6; k++) s = fma(J[k * n + i]!, r[k]!, s);
        g[i] = (s + diag[i]! * q[i]!) - pull[i]!;
        for (let j = 0; j < n; j++) {
          let h = J[i]! * J[j]!;
          for (let k = 1; k < 6; k++) h = fma(J[k * n + i]!, J[k * n + j]!, h);
          H[i * n + j] = h + (i === j ? diag[i]! : 0);
        }
      }
      yield FK_ROW_WORK + RESIDUAL_ROW_WORK;
      return { cost, g, H };
    }.bind(this);
    const loA = Float64Array.from(lo);
    const hiA = Float64Array.from(hi);
    const x = yield* levenbergMarquardt(Float64Array.from(qPrev), evaluate, loA, hiA, w.limit);
    return x.map((v, i) => clip(v, loA[i]!, hiA[i]!));
  }

  /**
   * Joint commands for `targets` (T, 4, 4) continuing from `qLast` (dof,), the last command.
   * `qBefore`: the commands before it, (dof,) or (rows, dof) newest first; two rows also set the
   * jerk into the chunk; null = the arm was still. `speedCap` (rad/s) lowers the arm's speeds.
   * `method` "traj" solves the chunk as one problem, "tick" tick by tick.
   */
  solve(targets: readonly (readonly (readonly number[])[])[], qLast: readonly number[], options: SolveOptions): IKResult {
    return run(this.#solve(targets, qLast, options));
  }

  /**
   * `solve` without blocking the event loop: the same computation, with a pause for the event loop
   * every few milliseconds (`options.pause`, default a macrotask) and `options.signal` checked at
   * each pause. Returns exactly what `solve` returns.
   */
  solveAsync(targets: readonly (readonly (readonly number[])[])[], qLast: readonly number[],
             options: SolveOptions & CooperativeOptions): Promise<IKResult> {
    return runAsync(this.#solve(targets, qLast, options), options);
  }

  *#solve(targets: readonly (readonly (readonly number[])[])[], qLast: readonly number[], options: SolveOptions): Work<IKResult> {
    const { qBefore = null, rateHz, speedCap = null, method = "traj" } = options;
    const [flat, last] = this.#checkInputs(targets, qLast);
    const T = targets.length;
    const n = this.n;
    const maxStep = this.maxStep(rateHz, speedCap);
    const [lo, hi] = this.bounds(last);
    const warm = new Float64Array(T * n);
    let prev = last;
    for (let t = 0; t < T; t++) {
      const sol = yield* this.#solveTick(flat, t, prev, lo, hi);
      warm.set(sol, t * n);
      prev = Array.from(sol);
    }
    let qSol: Float64Array;
    if (method === "traj") {
      let before: number[][];
      if (qBefore === null) before = [last];
      else {
        const values = (qBefore as readonly unknown[]).flat() as number[];
        if (values.length % n || !values.length) throw new ValueError(`q_before must be (rows, ${n}), got ${values.length} values`);
        before = Array.from({ length: values.length / n }, (_, i) => values.slice(i * n, (i + 1) * n).map(Number));
      }
      const sol = yield* this.#solveTraj(flat, warm, T, last, before, lo, hi);
      qSol = sol.map((v, e) => clip(v, lo[e % n]!, hi[e % n]!));
    } else if (method === "tick") {
      qSol = warm;
    } else {
      throw new ValueError(`method must be 'traj' or 'tick', got '${String(method)}'`);
    }
    const qSolRows = Array.from({ length: T }, (_, t) => Array.from(qSol.subarray(t * n, (t + 1) * n)));
    const [q, capped] = TrajectoryIK.cap(qSolRows, last, maxStep);
    const [posErr, rotErr] = yield* toolErrorsWork(this.chain, Float64Array.from(q.flat()), T, flat);
    return { q, qSolution: qSolRows, posErr, rotErr, capped };
  }

  /** The velocity / acceleration / jerk costs as one linear residual `D @ q - c`: D (rows, T), c (rows, dof). */
  #smoothing(T: number, last: number[], before: number[], before2: number[], jerk0: number): [number[][], number[][]] {
    const w = this.weights;
    const hist: Record<number, number[]> = { [-1]: last, [-2]: before, [-3]: before2 };
    const rows: number[][] = [];
    const consts: number[][] = [];
    const add = (weight: number, taps: [number, number][]): void => {
      // weight * sum(coef * q[tick]) over taps (tick, coef); ticks < 0 are the known commands.
      if (weight === 0 || taps.some(([t]) => t >= T)) return;
      const row = new Array<number>(T).fill(0);
      let c = new Array<number>(this.n).fill(0);
      for (const [t, coef] of taps) {
        if (t >= 0) row[t]! += weight * coef;
        else {
          const k = weight * coef;
          c = c.map((v, i) => v - k * hist[t]![i]!);
        }
      }
      rows.push(row);
      consts.push(c);
    };
    for (let k = 0; k < T - 1; k++) add(w.vel, [[k + 1, 1.0], [k, -1.0]]);
    for (let k = 0; k < T - 2; k++) add(w.acc, [[k + 2, 1.0], [k + 1, -2.0], [k, 1.0]]);
    for (let k = 0; k < T - 3; k++) add(w.jerk, [[k + 3, 1.0], [k + 2, -3.0], [k + 1, 3.0], [k, -1.0]]);
    const h = w.history;
    add(h * w.vel, [[0, 1.0], [-1, -1.0]]);
    add(h * w.acc, [[0, 1.0], [-1, -2.0], [-2, 1.0]]);
    add(h * w.acc, [[1, 1.0], [0, -2.0], [-1, 1.0]]);
    add(h * jerk0, [[0, 1.0], [-1, -3.0], [-2, 3.0], [-3, -1.0]]);
    add(h * w.jerk, [[1, 1.0], [0, -3.0], [-1, 3.0], [-2, -1.0]]);
    add(h * w.jerk, [[2, 1.0], [1, -3.0], [0, 3.0], [-1, -1.0]]);
    return [rows, consts];
  }

  /** Every tick at once: pose, limit and rest costs per tick plus the smoothing, from `init` (T*dof). */
  *#solveTraj(targets: Float64Array, init: Float64Array, T: number, last: number[], before: number[][],
              lo: number[], hi: number[]): Work<Float64Array> {
    const w = this.weights;
    const n = this.n;
    const N = T * n;
    const jerk0 = before.length >= 2 ? w.jerk : 0.0;
    const [D, c] = this.#smoothing(T, last, before[0]!, before.length >= 2 ? before[1]! : before[0]!, jerk0);
    const R = D.length;
    // D.T @ D (T x T) and D.T @ c (T x n): BLAS's fused chain over the rows, visiting only rows where
    // the column is nonzero (a zero term leaves a fused chain unchanged).
    const rowsOf: number[][] = Array.from({ length: T }, () => []);
    for (let r = 0; r < R; r++) for (let a = 0; a < T; a++) if (D[r]![a] !== 0) rowsOf[a]!.push(r);
    const DtD = new Float64Array(T * T);
    const Dtc = new Float64Array(T * n);
    for (let a = 0; a < T; a++) {
      const ra = rowsOf[a]!;
      for (let b = 0; b < T; b++) {
        let s = 0.0;
        for (const r of ra) if (D[r]![b] !== 0) s = fma(D[r]![a]!, D[r]![b]!, s);
        DtD[a * T + b] = s + 0;
      }
      yield 0.5 * T;
      for (let i = 0; i < n; i++) {
        let s = 0.0;
        for (const r of ra) s = fma(D[r]![a]!, c[r]![i]!, s);
        Dtc[a * n + i] = s + 0;
      }
    }
    // H_s = kron(D.T @ D, I_n), g_s = (D.T @ c).reshape(-1), c_s = sum(c * c)
    const Hs = new Float64Array(N * N);
    let kd = 0; // tick bandwidth of D.T @ D
    for (let a = 0; a < T; a++) {
      for (let b = 0; b < T; b++) {
        const v = DtD[a * T + b]!;
        if (v !== 0) kd = Math.max(kd, Math.abs(a - b));
        for (let i = 0; i < n; i++) Hs[(a * n + i) * N + b * n + i] = v;
      }
      yield 0.02 * T * n;
    }
    const gs = Dtc;
    const cc = new Float64Array(R * n);
    for (let r = 0; r < R; r++) for (let i = 0; i < n; i++) cc[r * n + i] = c[r]![i]! * c[r]![i]!;
    const cs = pairwiseSum(cc);
    const ori = this.free >= 6 ? w.ori : w.ori * FEW_JOINTS_ORI_SCALE;
    const scale = [w.pos, w.pos, w.pos, ori, ori, ori];
    const rw2 = this.restWeight.map((v) => v * v);
    const band = (kd + 1) * n; // columns of H_s in row i lie within i +- band

    const evaluate: Evaluate = function* (this: TrajectoryIK, x: Float64Array): Work<Evaluation> {
      // in batches of rows (each row is computed on its own, so batching changes nothing)
      const poses = new Float64Array(12 * T);
      const jacobians = new Float64Array(6 * n * T);
      for (let t0 = 0; t0 < T; t0 += FK_ROWS_PER_YIELD) {
        const rows = Math.min(FK_ROWS_PER_YIELD, T - t0);
        const part = this.chain.fkJacobianFlat(x.subarray(t0 * n, (t0 + rows) * n), rows);
        poses.set(part.poses, 12 * t0);
        jacobians.set(part.jacobians, 6 * n * t0);
        yield FK_ROW_WORK * rows;
      }
      const res = poseResidualFlat(targets, poses, jacobians, T, n);
      const r = res.r.map((v, e) => v * scale[e % 6]!);
      const J = res.J.map((v, e) => v * scale[Math.floor(e / n) % 6]!);
      const Hq = new Float64Array(N);
      for (let i = 0; i < N; i++) Hq[i] = gemvRow(Hs, i * N, x, N, Math.max(0, i - band), Math.min(N, i + band + 1));
      const rr = r.map((v) => v * v);
      const restTerms = new Float64Array(N);
      for (let e = 0; e < N; e++) {
        const d = x[e]! - this.rest[e % n]!;
        restTerms[e] = rw2[e % n]! * (d * d);
      }
      const gs2 = gs.map((v) => 2.0 * v);
      const cost = (((pairwiseSum(rr) + pairwiseSum(restTerms)) + ddot(x, Hq, N)) - ddot(gs2, x, N)) + cs;
      yield RESIDUAL_ROW_WORK * T;
      const g = new Float64Array(N);
      const H = Float64Array.from(Hs);
      for (let t = 0; t < T; t++) {
        const jo = 6 * n * t;
        for (let i = 0; i < n; i++) {
          let s = J[jo + i]! * r[6 * t]!;
          for (let k = 1; k < 6; k++) s = fma(J[jo + k * n + i]!, r[6 * t + k]!, s);
          const e = t * n + i;
          g[e] = ((s + rw2[i]! * (x[e]! - this.rest[i]!)) + Hq[e]!) - gs[e]!;
          for (let j = 0; j < n; j++) {
            let b = 0.0;
            for (let k = 0; k < 6; k++) b += J[jo + k * n + i]! * J[jo + k * n + j]!;
            if (i === j) b += rw2[i]!;
            H[e * N + t * n + j]! += b;
          }
        }
      }
      yield sweepWork(N);
      return { cost, g, H };
    }.bind(this);
    const loX = new Float64Array(N);
    const hiX = new Float64Array(N);
    for (let e = 0; e < N; e++) {
      loX[e] = lo[e % n]!;
      hiX[e] = hi[e % n]!;
    }
    return yield* levenbergMarquardt(Float64Array.from(init), evaluate, loX, hiX, w.limit);
  }
}

/**
 * Successive chunks (`feed`) or single poses (`step`) solved from the commands already sent, so
 * each continues the last: the trajectory solve reads the last three commands, a single pose the
 * last one.
 */
export class IKStream {
  readonly ik: TrajectoryIK;
  readonly rateHz: number;
  readonly speedCap: number | null;
  /** Commands sent, newest first, at most three. */
  history: number[][] = [];
  /** The previous tick's solution (`step`). */
  solution: number[] = [];

  constructor(ik: TrajectoryIK, qLast: readonly number[], rateHz: number, speedCap: number | null = null) {
    this.ik = ik;
    this.rateHz = rateHz;
    this.speedCap = speedCap;
    this.reset(qLast);
  }

  /** Start over from `qLast` with the arm still. */
  reset(qLast: readonly number[]): void {
    this.history = [qLast.map(Number)];
    this.solution = qLast.map(Number);
  }

  get qLast(): number[] {
    return this.history[0]!;
  }

  #sent(rows: number[][], solution: number[]): void {
    this.history = [...[...rows].reverse(), ...this.history].slice(0, 3);
    this.solution = solution;
  }

  /**
   * The joint commands for a chunk of poses (T, 4, 4); they become the history of the next.
   *
   * A whole-chunk solve takes tens of milliseconds without yielding. On the thread that runs an
   * arm's control loop use feedAsync(), or the arm's feedback watchdog can trip.
   */
  feed(targets: readonly (readonly (readonly number[])[])[], method: "traj" | "tick" = "traj"): IKResult {
    const before = this.history.length > 1 ? this.history.slice(1) : null;
    const result = this.ik.solve(targets, this.qLast, { qBefore: before, rateHz: this.rateHz, speedCap: this.speedCap, method });
    this.#sent(result.q, result.qSolution[result.qSolution.length - 1]!);
    return result;
  }

  /**
   * `feed` without blocking the event loop (see `TrajectoryIK.solveAsync`); returns exactly what
   * `feed` returns. The history moves on only once the solve has finished.
   */
  async feedAsync(targets: readonly (readonly (readonly number[])[])[],
                  options: { method?: "traj" | "tick" } & CooperativeOptions = {}): Promise<IKResult> {
    const before = this.history.length > 1 ? this.history.slice(1) : null;
    const result = await this.ik.solveAsync(targets, this.qLast, {
      qBefore: before, rateHz: this.rateHz, speedCap: this.speedCap, method: options.method ?? "traj",
      pause: options.pause, signal: options.signal,
    });
    this.#sent(result.q, result.qSolution[result.qSolution.length - 1]!);
    return result;
  }

  /** The next joint command for one pose (4, 4): the per-tick solve from the previous solution, capped. */
  step(pose: readonly (readonly number[])[]): number[] {
    const [lo, hi] = this.ik.bounds(this.qLast);
    const solution = this.ik.solveTick(pose, this.solution, lo, hi);
    const [q] = TrajectoryIK.cap([solution], this.qLast, this.ik.maxStep(this.rateHz, this.speedCap));
    this.#sent(q, solution);
    return q[0]!;
  }
}

/** Trajectory IK solvers: this module's ("numpy"; "auto" picks it). */
export const SOLVERS = ["auto", "numpy"] as const;

/**
 * The trajectory IK for an arm profile (see `TrajectoryIK.fromConfig`). The Python SDK's compiled
 * JAX solver has no TypeScript port; `solver` "auto" and "numpy" both give this module's.
 */
export function fromConfig(config: ArmConfig, options: { chain?: Chain | null; solver?: string; weights?: IKWeights | null } = {}): TrajectoryIK {
  const solver = options.solver ?? "auto";
  if (!(SOLVERS as readonly string[]).includes(solver)) {
    throw new ValueError(`solver must be one of ${JSON.stringify(SOLVERS)} (the JAX solver is not available here), got '${solver}'`);
  }
  return TrajectoryIK.fromConfig(config, { weights: options.weights ?? null, chain: options.chain ?? null });
}

// ── tool paths ──
/** Python's x ** k for a small positive integer k (numpy calls the C library's pow). */
const powi = (x: number, k: number): number => Math.pow(x, k);

/**
 * (ticks, 4, 4) poses from just after `start` to `goal`: the position on the line between them,
 * the rotation turning about one fixed axis; `ease` times them minimum-jerk (still at both ends).
 */
export function straightLine(start: readonly (readonly number[])[], goal: readonly (readonly number[])[], ticks: number,
                             ease = true): number[][][] {
  const row = straightLineRows(start, goal, ticks, ease);
  return Array.from({ length: ticks }, (_, i) => row(i));
}

/** Rows (ticks per pause) straightLineAsync builds between pauses: a few milliseconds of work. */
export const STRAIGHT_LINE_ROWS_PER_PAUSE = 256;

/**
 * `straightLine` without blocking the event loop: the same rows, with `pause()` every
 * STRAIGHT_LINE_ROWS_PER_PAUSE rows (default a macrotask).
 */
export async function straightLineAsync(start: readonly (readonly number[])[], goal: readonly (readonly number[])[], ticks: number,
                                        options: { ease?: boolean; pause?: () => Promise<void> } = {}): Promise<number[][][]> {
  const row = straightLineRows(start, goal, ticks, options.ease ?? true);
  const pause = options.pause ?? macrotask;
  const out: number[][][] = [];
  for (let i = 0; i < ticks; i++) {
    if (i > 0 && i % STRAIGHT_LINE_ROWS_PER_PAUSE === 0) await pause();
    out.push(row(i));
  }
  return out;
}

function straightLineRows(start: readonly (readonly number[])[], goal: readonly (readonly number[])[], ticks: number,
                          ease: boolean): (i: number) => number[][] {
  if (ticks < 1) throw new ValueError("ticks must be >= 1");
  const s0 = rows3(start);
  const g0 = rows3(goal);
  const turn = log3(mulTA(s0, 0, g0, 0));
  const dp = [0, 1, 2].map((i) => goal[i]![3]! - start[i]![3]!);
  return (i: number) => {
    const u = (i + 1) / ticks;
    const si = ease ? (10 * powi(u, 3) - 15 * powi(u, 4)) + 6 * powi(u, 5) : u;
    const rot = mat3Mul(s0, exp3(si * turn[0], si * turn[1], si * turn[2]));
    return [0, 1, 2].map((r) => [rot[3 * r]!, rot[3 * r + 1]!, rot[3 * r + 2]!, start[r]![3]! + si * dp[r]!])
      .concat([[0, 0, 0, 1]]);
  };
}

// ── pose pre-processing ──
function savgolMatrix(window: number, polyorder: number): number[][] {
  // (window, window): a degree-polyorder least-squares fit over a window, evaluated at each of its ticks.
  const h = Math.floor(window / 2);
  const V = Array.from({ length: window }, (_, i) => Array.from({ length: polyorder + 1 }, (_, p) => (i - h) ** p));
  return projection(V);
}

function savgol2(x: readonly (readonly number[])[], P: number[][], window: number): number[][] {
  const h = Math.floor(window / 2);
  const n = x.length;
  const d = n ? x[0]!.length : 0;
  const out: number[][] = Array.from({ length: n }, () => new Array<number>(d).fill(0));
  const apply = (row: number[], from: number, i: number): void => {
    for (let j = 0; j < d; j++) {
      let s = 0;
      for (let k = 0; k < window; k++) s += row[k]! * x[from + k]![j]!;
      out[i]![j] = s;
    }
  };
  for (let i = h; i < n - h; i++) apply(P[h]!, i - h, i);
  for (let i = 0; i < h; i++) apply(P[i]!, 0, i);
  for (let i = 0; i < h; i++) apply(P[window - h + i]!, n - window, n - h + i);
  return out;
}

/**
 * Savitzky-Golay along the ticks of (ticks, d) values (or a stack of them); the first and last
 * `window // 2` ticks take the fit over the first / last window (scipy's `mode="interp"`).
 */
export function savgol(x: readonly (readonly number[])[], window: number, polyorder: number): number[][];
export function savgol(x: readonly (readonly (readonly number[])[])[], window: number, polyorder: number): number[][][];
export function savgol(x: readonly unknown[], window: number, polyorder: number): number[][] | number[][][] {
  const P = savgolMatrix(window, polyorder);
  const one = (v: readonly (readonly number[])[]): number[][] => savgol2(v, P, window);
  if (Array.isArray((x[0] as unknown[] | undefined)?.[0])) return (x as readonly (readonly (readonly number[])[])[]).map(one);
  return one(x as readonly (readonly number[])[]);
}

function smoothOne(T: readonly (readonly (readonly number[])[])[], window: number, polyorder: number): number[][][] {
  const n = T.length;
  const w = Math.min(window | 1, n % 2 ? n : n - 1);
  if (window <= 0 || w <= polyorder) return T.map((p) => p.map((r) => [...r]));
  const R0 = rows3(T[0]!);
  const rv = T.map((p) => log3(mulTA(R0, 0, rows3(p), 0)) as number[]);
  const P = savgolMatrix(w, polyorder);
  const rvS = savgol2(rv, P, w);
  const pS = savgol2(T.map((p) => [p[0]![3]!, p[1]![3]!, p[2]![3]!]), P, w);
  return T.map((_, t) => {
    const rot = mat3Mul(R0, exp3(rvS[t]![0]!, rvS[t]![1]!, rvS[t]![2]!));
    return [0, 1, 2].map((i) => [rot[3 * i]!, rot[3 * i + 1]!, rot[3 * i + 2]!, pS[t]![i]!]).concat([[...T[t]![3]!]]);
  });
}

/**
 * Savitzky-Golay over the ticks of (ticks, 4, 4) poses (or a stack of them): positions as they
 * are, rotations as rotation vectors relative to the first tick. The window shrinks to the chunk
 * (odd); 0 or too short = unchanged.
 */
export function smoothPoses(T: readonly (readonly (readonly number[])[])[], window?: number, polyorder?: number): number[][][];
export function smoothPoses(T: readonly (readonly (readonly (readonly number[])[])[])[], window?: number, polyorder?: number): number[][][][];
export function smoothPoses(T: readonly unknown[], window = 15, polyorder = 3): number[][][] | number[][][][] {
  if (Array.isArray(((T[0] as unknown[] | undefined)?.[0] as unknown[] | undefined)?.[0])) {
    return (T as readonly (readonly (readonly (readonly number[])[])[])[]).map((one) => smoothOne(one, window, polyorder));
  }
  return smoothOne(T as readonly (readonly (readonly number[])[])[], window, polyorder);
}

function extendOne(T: readonly (readonly (readonly number[])[])[], rows: number, k: number): number[][][] {
  const copy = T.map((p) => p.map((r) => [...r]));
  if (rows <= 0) return copy;
  k = Math.min(k, T.length - 1);
  const last = T[T.length - 1]!;
  const back = T[T.length - 1 - k]!;
  const dp = [0, 1, 2].map((i) => (last[i]![3]! - back[i]![3]!) / k);
  const lastR = rows3(last);
  const w = log3(mulTA(rows3(back), 0, lastR, 0)).map((v) => v / k);
  for (let s = 1; s <= rows; s++) {
    const rot = mat3Mul(lastR, exp3(s * w[0]!, s * w[1]!, s * w[2]!));
    copy.push([0, 1, 2].map((i) => [rot[3 * i]!, rot[3 * i + 1]!, rot[3 * i + 2]!, last[i]![3]! + s * dp[i]!])
      .concat([[...last[3]!]]));
  }
  return copy;
}

/**
 * (ticks, 4, 4) -> (ticks + rows, 4, 4) (or stacks of them): the added rows continue the last `k`
 * rows' mean step, the position in a line and the rotation about a fixed axis of the tool's own
 * frame (a chunk padded this way does not brake into a hold at its last pose in the trajectory
 * solve).
 */
export function extendPoses(T: readonly (readonly (readonly number[])[])[], rows: number, k?: number): number[][][];
export function extendPoses(T: readonly (readonly (readonly (readonly number[])[])[])[], rows: number, k?: number): number[][][][];
export function extendPoses(T: readonly unknown[], rows: number, k = 4): number[][][] | number[][][][] {
  if (Array.isArray(((T[0] as unknown[] | undefined)?.[0] as unknown[] | undefined)?.[0])) {
    return (T as readonly (readonly (readonly (readonly number[])[])[])[]).map((one) => extendOne(one, rows, k));
  }
  return extendOne(T as readonly (readonly (readonly number[])[])[], rows, k);
}

/**
 * Ease a tick-solved chunk q (T, dof) in from the last command `qFrom`: the plan shifted by its
 * start offset, the shift fading to zero over the first `ticks` ticks, clipped to [lo, hi] and each
 * tick kept within `maxStep` of the one before (the trajectory solve needs none of this: continuity
 * is in its costs).
 */
export function seamBlend(q: readonly (readonly number[])[], qFrom: readonly number[], ticks: number,
                          lo: number | readonly number[], hi: number | readonly number[],
                          maxStep: number | readonly number[]): number[][] {
  const at = (v: number | readonly number[], i: number): number => (typeof v === "number" ? v : v.length === 1 ? v[0]! : v[i]!);
  const out: number[][] = [];
  let prev = qFrom.map(Number);
  q.forEach((row, t) => {
    const fade = clip(1.0 - (t + 1) / (ticks + 1), 0.0, 1.0);
    const eased = row.map((v, i) => clip(v + fade * (qFrom[i]! - q[0]![i]!), at(lo, i), at(hi, i)));
    prev = eased.map((v, i) => clip(v, prev[i]! - at(maxStep, i), prev[i]! + at(maxStep, i)));
    out.push(prev);
  });
  return out;
}

/**
 * A long pose path (T, 4, 4) solved in chunks of `chunk` ticks, each continuing the one before.
 * `between` is called before each chunk; it may throw to stop the solve.
 */
export function solvePath(ik: TrajectoryIK, poses: readonly (readonly (readonly number[])[])[], qLast: readonly number[],
                          rateHz: number, speedCap: number | null = null, chunk = 64,
                          between: (() => void) | null = null): IKResult {
  const stream = new IKStream(ik, qLast, rateHz, speedCap);
  const parts: IKResult[] = [];
  for (let i = 0; i < poses.length; i += chunk) {
    if (between !== null) between();
    parts.push(stream.feed(poses.slice(i, i + chunk)));
  }
  if (!parts.length) throw new ValueError("need at least one array to concatenate");
  return {
    q: parts.flatMap((p) => p.q),
    qSolution: parts.flatMap((p) => p.qSolution),
    posErr: parts.flatMap((p) => p.posErr),
    rotErr: parts.flatMap((p) => p.rotErr),
    capped: parts.flatMap((p) => p.capped),
  };
}

/**
 * `solvePath` without blocking the event loop: the same chunks and numbers, with a pause for the
 * event loop every few milliseconds of computation and `options.signal` checked at each pause.
 * `options.between` is called before each chunk; it may throw to stop the solve.
 */
export async function solvePathAsync(ik: TrajectoryIK, poses: readonly (readonly (readonly number[])[])[],
                                     qLast: readonly number[], rateHz: number,
                                     options: { speedCap?: number | null; chunk?: number; between?: (() => void) | null }
                                       & CooperativeOptions = {}): Promise<IKResult> {
  const { speedCap = null, chunk = 64, between = null } = options;
  const stream = new IKStream(ik, qLast, rateHz, speedCap);
  const parts: IKResult[] = [];
  const pause = options.pause ?? macrotask;
  for (let i = 0; i < poses.length; i += chunk) {
    if (i > 0) {
      await pause();
      if (options.signal?.aborted) throw options.signal.reason;
    }
    if (between !== null) between();
    parts.push(await stream.feedAsync(poses.slice(i, i + chunk), { pause, signal: options.signal }));
  }
  if (!parts.length) throw new ValueError("need at least one array to concatenate");
  return {
    q: parts.flatMap((p) => p.q),
    qSolution: parts.flatMap((p) => p.qSolution),
    posErr: parts.flatMap((p) => p.posErr),
    rotErr: parts.flatMap((p) => p.rotErr),
    capped: parts.flatMap((p) => p.capped),
  };
}

function shape(x: unknown): string {
  const dims: number[] = [];
  let v: unknown = x;
  while (Array.isArray(v)) {
    dims.push(v.length);
    v = v[0];
  }
  return `(${dims.join(", ")}${dims.length === 1 ? "," : ""})`;
}
