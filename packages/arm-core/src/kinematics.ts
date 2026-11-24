/**
 * Forward/inverse kinematics and gravity torques of a serial chain read from a URDF.
 *
 * Joint coordinates are the SDK's: the URDF joint angle is `q * sign + offset` per joint, with
 * `sign` (+1/-1) and `offset` (rad) taken from the profile. Poses are 4x4 homogeneous matrices
 * (`number[][]`, row-major) in the world frame, which is the URDF root under an optional mount
 * transform (`baseXyz`, `baseRpy`): a URDF drawn y-up is turned z-up there, so gravity is always
 * `(0, 0, -9.81)`.
 *
 * Links hanging off the chain (gripper jaws on prismatic joints, a fixed tool link) are lumped
 * into the chain link they hang from, with their own joints held at 0.
 *
 * FK, Jacobians, gravity torques and potential energy keep the Python SDK's numpy operation order
 * (see linalg.ts). Angles go through `Math.sin` / `Math.cos`, which can differ from the C library's
 * in the last bit, so results agree with Python's to a few units in the last place, not bit for bit.
 */

import type { ArmConfig } from "./config.js";
import { type CooperativeOptions, run, runAsync, type Work } from "./cooperate.js";
import { ValueError } from "./errors.js";
import {
  clip, dot3, dotEinsum3, dotFma, fma, inv, mat3Mul, mat3Vec, mat4Mul, norm, npMaximum, npMinimum, pairwiseSum,
} from "./linalg.js";
import { readUrdf, type UrdfJoint } from "./urdf.js";

/** URDF joint types that can be actuated. */
export const ACTUATED = ["revolute", "continuous"] as const;
/** m/s^2, world frame (z up). */
export const GRAVITY: readonly number[] = [0.0, 0.0, -9.81];

const isActuated = (type: string): boolean => (ACTUATED as readonly string[]).includes(type);

// ── rotations ──
/** URDF fixed-axis roll, pitch, yaw: R = Rz(yaw) @ Ry(pitch) @ Rx(roll). */
export function rpyToMatrix(rpy: readonly number[]): number[][] {
  const [r, p, y] = rpy as [number, number, number];
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}

/** Inverse of rpyToMatrix; pitch in [-pi/2, pi/2] (roll = 0 at the singularity). */
export function matrixToRpy(rot: readonly (readonly number[])[]): number[] {
  const pitch = Math.atan2(-rot[2]![0]!, Math.hypot(rot[0]![0]!, rot[1]![0]!));
  if (Math.abs(Math.abs(pitch) - Math.PI / 2) < 1e-9) {
    return [0.0, pitch, Math.atan2(-rot[0]![1]!, rot[1]![1]!)];
  }
  return [Math.atan2(rot[2]![1]!, rot[2]![2]!), pitch, Math.atan2(rot[1]![0]!, rot[0]![0]!)];
}

/** 4x4 pose from a position and URDF roll, pitch, yaw. */
export function xyzRpyToPose(xyz: readonly number[], rpy: readonly number[] = [0.0, 0.0, 0.0]): number[][] {
  const r = rpyToMatrix(rpy);
  return [
    [r[0]![0]!, r[0]![1]!, r[0]![2]!, xyz[0]!],
    [r[1]![0]!, r[1]![1]!, r[1]![2]!, xyz[1]!],
    [r[2]![0]!, r[2]![1]!, r[2]![2]!, xyz[2]!],
    [0, 0, 0, 1],
  ];
}

/** Position and roll, pitch, yaw of a 4x4 pose. */
export function poseToXyzRpy(pose: readonly (readonly number[])[]): [number[], number[]] {
  return [[pose[0]![3]!, pose[1]![3]!, pose[2]![3]!], matrixToRpy(pose.slice(0, 3).map((r) => r.slice(0, 3)))];
}

/** World-frame rotation vector (axis * angle) taking `current` onto `target` (3x3 each). */
export function rotationError(target: readonly (readonly number[])[], current: readonly (readonly number[])[]): number[] {
  const t = Float64Array.from([...target[0]!.slice(0, 3), ...target[1]!.slice(0, 3), ...target[2]!.slice(0, 3)]);
  const ct = new Float64Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) ct[3 * i + j] = current[j]![i]!;
  return rotationErrorFlat(t, ct);
}

/** rotationError on row-major 3x3 values: `target` and the transpose of `current`. */
function rotationErrorFlat(target: Float64Array, currentT: Float64Array, tOff = 0): number[] {
  const rot = mat3Mul(tOff ? target.subarray(tOff, tOff + 9) : target, currentT);
  const trace = rot[0]! + rot[4]! + rot[8]!;
  const c = pyMax(-1.0, pyMin(1.0, (trace - 1.0) / 2.0));
  const angle = Math.acos(c);
  const vee = [rot[7]! - rot[5]!, rot[2]! - rot[6]!, rot[3]! - rot[1]!];
  if (angle < 1e-6) return vee.map((v) => v / 2.0);
  if (Math.PI - angle < 1e-4) {
    // Near a half turn Math.sin(angle) vanishes: the axis is the dominant column of (R + I) / 2.
    const sym = Array.from({ length: 9 }, (_, i) => (rot[i]! + (i % 4 === 0 ? 1 : 0)) / 2.0);
    let k = 0;
    for (let i = 1; i < 3; i++) if (sym[4 * i]! > sym[4 * k]!) k = i;
    const d = Math.sqrt(Math.max(sym[4 * k]!, 1e-12));
    let axis = [sym[k]! / d, sym[3 + k]! / d, sym[6 + k]! / d];
    if (dot3(axis[0]!, axis[1]!, axis[2]!, vee[0]!, vee[1]!, vee[2]!) < 0) axis = axis.map((v) => -v);
    return axis.map((v) => v * angle);
  }
  const f = angle / (2.0 * Math.sin(angle));
  return vee.map((v) => v * f);
}

function skew(v: ArrayLike<number>, off = 0): Float64Array {
  const x = v[off]!, y = v[off + 1]!, z = v[off + 2]!;
  return Float64Array.from([0.0, -z, y, z, 0.0, -x, -y, x, 0.0]);
}

/** The result of `Chain.ik`. */
export interface IKResult {
  /** Joint coordinates, rad (the best found, even when not ok). */
  readonly q: number[];
  readonly ok: boolean;
  /** m. */
  readonly posErr: number;
  /** rad; 0 for a position-only solve. */
  readonly rotErr: number;
  readonly iters: number;
}

/** Options of `Chain.fromUrdf`. */
export interface FromUrdfOptions {
  /** Actuated joints in chain order (default: every revolute/continuous joint on the path). */
  joints?: readonly string[] | null;
  /** Default: the URDF root. */
  base?: string | null;
  /** Default: the child of the last of `joints`, else the end of the branch with the most revolute joints. */
  tip?: string | null;
  tcpXyz?: readonly number[];
  tcpRpy?: readonly number[];
  sign?: readonly number[] | null;
  offset?: readonly number[] | null;
  baseXyz?: readonly number[];
  baseRpy?: readonly number[];
}

/** Options of `Chain.ik`. */
export interface IKOptions {
  /** With an xyz target: the tool orientation (else position only). */
  rpy?: readonly number[] | null;
  /** Ignore a pose's orientation. */
  positionOnly?: boolean | null;
  /** [lower, upper] in joint coordinates (default: the URDF limits). */
  limits?: readonly [readonly number[], readonly number[]] | null;
  maxIters?: number;
  /** m. */
  tol?: number;
  /** rad. */
  rotTol?: number;
  rotWeight?: number;
  nullGain?: number;
  maxStep?: number;
}

/** Joint frames of one configuration (flat, row-major). */
interface Frames {
  axes: Float64Array; // n x 3
  pivots: Float64Array; // n x 3
  segR: Float64Array; // n x 9
  segP: Float64Array; // n x 3
  toolR: Float64Array; // 9
  toolP: Float64Array; // 3
}

/** A serial chain of N actuated joints. Build it with `Chain.fromUrdf`. */
export class Chain {
  readonly jointNames: string[];
  readonly n: number;
  /** Per segment 0..N (0 = before joint 1), kg. */
  readonly masses: number[];
  readonly sign: number[];
  readonly offset: number[];
  /** URDF root -> world, 4x4. */
  readonly mount: number[][];
  /** URDF limits in joint coordinates. */
  readonly lower: number[];
  readonly upper: number[];
  readonly #fixedR: Float64Array;
  readonly #fixedP: Float64Array;
  readonly #axes: Float64Array;
  readonly #k: Float64Array;
  readonly #k2: Float64Array;
  readonly #postR: Float64Array;
  readonly #postP: Float64Array;
  readonly #coms: Float64Array;
  readonly #mountR: Float64Array;
  readonly #mountP: Float64Array;

  constructor(names: readonly string[], fixed: readonly (readonly (readonly number[])[])[], axes: readonly (readonly number[])[],
              post: readonly (readonly number[])[], masses: readonly number[], coms: readonly (readonly number[])[],
              lower: readonly number[], upper: readonly number[],
              options: { sign?: readonly number[] | null; offset?: readonly number[] | null; mount?: readonly (readonly number[])[] | null } = {}) {
    this.jointNames = [...names];
    const n = (this.n = this.jointNames.length);
    this.#fixedR = new Float64Array(9 * n);
    this.#fixedP = new Float64Array(3 * n);
    fixed.forEach((f, i) => {
      for (let r = 0; r < 3; r++) {
        for (let c = 0; c < 3; c++) this.#fixedR[9 * i + 3 * r + c] = f[r]![c]!;
        this.#fixedP[3 * i + r] = f[r]![3]!;
      }
    });
    this.#axes = Float64Array.from(axes.flat());
    if (this.#axes.length !== 3 * n) throw new ValueError(`need ${n} joint axes`);
    this.#k = new Float64Array(9 * n);
    this.#k2 = new Float64Array(9 * n);
    for (let i = 0; i < n; i++) {
      const k = skew(this.#axes, 3 * i);
      this.#k.set(k, 9 * i);
      this.#k2.set(mat3Mul(k, k), 9 * i);
    }
    this.#postR = Float64Array.from([...post[0]!.slice(0, 3), ...post[1]!.slice(0, 3), ...post[2]!.slice(0, 3)]);
    this.#postP = Float64Array.from([post[0]![3]!, post[1]![3]!, post[2]![3]!]);
    this.masses = [...masses];
    this.#coms = Float64Array.from(coms.flat());
    if (this.#coms.length !== 3 * (n + 1)) throw new ValueError(`need ${n + 1} segment centres of mass`);
    this.sign = options.sign == null ? new Array<number>(n).fill(1) : options.sign.map(Number);
    this.offset = options.offset == null ? new Array<number>(n).fill(0) : options.offset.map(Number);
    if (this.sign.length !== n || this.offset.length !== n) throw new ValueError(`sign and offset need ${n} values`);
    if (!this.sign.every((s) => Math.abs(s) === 1.0)) {
      throw new ValueError(`sign must be +1 or -1 per joint, got ${JSON.stringify(this.sign)}`);
    }
    this.mount = options.mount == null
      ? [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
      : options.mount.map((r) => r.map(Number));
    this.#mountR = Float64Array.from([...this.mount[0]!.slice(0, 3), ...this.mount[1]!.slice(0, 3), ...this.mount[2]!.slice(0, 3)]);
    this.#mountP = Float64Array.from([this.mount[0]![3]!, this.mount[1]![3]!, this.mount[2]![3]!]);
    // URDF limits in joint coordinates.
    const a = lower.map((v, i) => (v - this.offset[i]!) * this.sign[i]!);
    const b = upper.map((v, i) => (v - this.offset[i]!) * this.sign[i]!);
    this.lower = a.map((v, i) => npMinimum(v, b[i]!));
    this.upper = a.map((v, i) => npMaximum(v, b[i]!));
  }

  // ── construction ──
  /**
   * The chain from `base` (default: the URDF root) to `tip` (default: the child of the last of
   * `joints`, else the end of the branch with the most revolute joints). `joints` names the
   * actuated joints in order (default: every revolute/continuous joint on the path); any other
   * movable joint on the path is held at 0. `tcp*` place the tool point on the tip link.
   */
  static fromUrdf(xmlText: string, options: FromUrdfOptions = {}): Chain {
    let { joints = null, base = null, tip = null } = options;
    const { tcpXyz = [0, 0, 0], tcpRpy = [0, 0, 0], sign = null, offset = null, baseXyz = [0, 0, 0], baseRpy = [0, 0, 0] } = options;
    const { links, joints: allJoints } = readUrdf(xmlText);
    const byParent = new Map<string, UrdfJoint[]>();
    const childOf = new Map<string, UrdfJoint>();
    for (const j of allJoints.values()) {
      const list = byParent.get(j.parent);
      if (list) list.push(j);
      else byParent.set(j.parent, [j]);
      childOf.set(j.child, j);
    }
    if (base === null) {
      const roots = [...links.keys()].filter((name) => !childOf.has(name));
      if (roots.length !== 1) throw new ValueError(`URDF: expected one root link, found ${pyList(roots)}`);
      base = roots[0]!;
    }
    if (!links.has(base)) throw new ValueError(`URDF: no link '${base}'`);
    if (tip === null && joints && joints.length) {
      const last = joints[joints.length - 1]!;
      const j = allJoints.get(last);
      if (!j) throw new ValueError(`URDF: no joint '${last}'`);
      tip = j.child;
    }
    if (tip === null) {
      const memo = new Map<string, number>();
      const score = (j: UrdfJoint): number => Number(isActuated(j.type)) + actuatedBelow(j.child);
      const actuatedBelow = (link: string): number => {
        const known = memo.get(link);
        if (known !== undefined) return known;
        let best = 0;
        for (const j of byParent.get(link) ?? []) best = Math.max(best, score(j));
        memo.set(link, best);
        return best;
      };
      tip = base;
      while (actuatedBelow(tip)) {
        let pick: UrdfJoint | null = null;
        for (const j of byParent.get(tip)!) if (pick === null || score(j) > score(pick)) pick = j;
        tip = pick!.child;
      }
    }
    if (!links.has(tip)) throw new ValueError(`URDF: no link '${tip}'`);
    const pathJoints: UrdfJoint[] = [];
    let link = tip;
    while (link !== base) {
      const j = childOf.get(link);
      if (!j) throw new ValueError(`URDF: link '${tip}' is not below '${base}'`);
      pathJoints.push(j);
      link = j.parent;
    }
    pathJoints.reverse();
    const names = joints === null ? pathJoints.filter((j) => isActuated(j.type)).map((j) => j.name) : [...joints];
    const onPath = pathJoints.map((j) => j.name);
    const missing = names.filter((name) => !onPath.includes(name));
    if (missing.length) throw new ValueError(`URDF: joints ${pyList(missing)} are not on the chain ${base} -> ${tip}`);
    const ordered = onPath.filter((name) => names.includes(name));
    if (ordered.length !== names.length || ordered.some((name, i) => name !== names[i])) {
      throw new ValueError(`URDF: joints must be listed in chain order ${pyList(onPath)}`);
    }
    if (names.some((name) => !isActuated(allJoints.get(name)!.type))) {
      throw new ValueError(`URDF: only revolute/continuous joints can be actuated: ${pyList(names)}`);
    }

    const origins = new Map<UrdfJoint, number[][]>();
    const originOf = (j: UrdfJoint): number[][] => {
      let o = origins.get(j);
      if (!o) origins.set(j, (o = xyzRpyToPose(j.xyz, j.rpy)));
      return o;
    };
    const pathLinks = new Set([base, ...pathJoints.map((j) => j.child)]);
    const segmentsM = new Array<number>(names.length + 1).fill(0.0);
    const segmentsMc = Array.from({ length: names.length + 1 }, () => [0, 0, 0]);
    const lump = (name: string, frame: number[][], segment: number): void => {
      // `name` and everything off the chain below it, at their joints' zero, into `segment`.
      const body = links.get(name);
      if (!body) throw new ValueError(`URDF: no link '${name}'`);
      segmentsM[segment]! += body.mass;
      const c = body.com;
      const mc = segmentsMc[segment]!;
      for (let r = 0; r < 3; r++) {
        const fr = frame[r]!;
        const w = dot3(fr[0]!, fr[1]!, fr[2]!, c[0]!, c[1]!, c[2]!) + fr[3]!;
        mc[r] = mc[r]! + body.mass * w;
      }
      for (const j of byParent.get(name) ?? []) {
        if (!pathLinks.has(j.child) || name === tip) lump(j.child, mat4Mul(frame, originOf(j)), segment);
      }
    };

    const fixed: number[][][] = [];
    const axes: number[][] = [];
    const lower: number[] = [];
    const upper: number[] = [];
    const eye = (): number[][] => [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
    let acc = eye();
    let segment = 0;
    lump(base, acc, 0);
    for (const j of pathJoints) {
      acc = mat4Mul(acc, originOf(j));
      if (names.includes(j.name)) {
        fixed.push(acc);
        axes.push([...j.axis]);
        lower.push(j.lower);
        upper.push(j.upper);
        acc = eye();
        segment += 1;
      }
      lump(j.child, acc, segment);
    }
    const coms = segmentsM.map((m, i) => (m > 0 ? segmentsMc[i]!.map((v) => v / m) : [0, 0, 0]));
    const post = mat4Mul(acc, xyzRpyToPose(tcpXyz, tcpRpy));
    return new Chain(names, fixed, axes, post, segmentsM, coms, lower, upper,
                     { sign, offset, mount: xyzRpyToPose(baseXyz, baseRpy) });
  }

  // ── kinematics ──
  #theta(q: readonly number[]): Float64Array {
    if (q.length !== this.n) throw new ValueError(`need ${this.n} joint values, got (${q.length},)`);
    const theta = new Float64Array(this.n);
    for (let i = 0; i < this.n; i++) theta[i] = Number(q[i]) * this.sign[i]! + this.offset[i]!;
    return theta;
  }

  /** World-frame joint axes and pivots, each segment's rotation/origin after its joint, and the tool pose. */
  #frames(theta: ArrayLike<number>, thetaOff = 0): Frames {
    const n = this.n;
    const axes = new Float64Array(3 * n);
    const pivots = new Float64Array(3 * n);
    const segR = new Float64Array(9 * n);
    const segP = new Float64Array(3 * n);
    let rot: Float64Array = this.#mountR;
    let pos: Float64Array = this.#mountP;
    const turn = new Float64Array(9);
    const tmp = new Float64Array(3);
    for (let i = 0; i < n; i++) {
      const th = theta[thetaOff + i]!;
      const s = Math.sin(th);
      const c = 1.0 - Math.cos(th);
      for (let e = 0; e < 9; e++) turn[e] = ((e % 4 === 0 ? 1 : 0) + s * this.#k[9 * i + e]!) + c * this.#k2[9 * i + e]!;
      mat3Vec(rot, this.#fixedP, tmp, 3 * i);
      pos = Float64Array.of(pos[0]! + tmp[0]!, pos[1]! + tmp[1]!, pos[2]! + tmp[2]!);
      rot = mat3Mul(rot, this.#fixedR.subarray(9 * i, 9 * i + 9));
      axes.set(mat3Vec(rot, this.#axes, tmp, 3 * i), 3 * i);
      pivots.set(pos, 3 * i);
      rot = mat3Mul(rot, turn);
      segR.set(rot, 9 * i);
      segP.set(pos, 3 * i);
    }
    const toolR = mat3Mul(rot, this.#postR);
    mat3Vec(rot, this.#postP, tmp);
    const toolP = Float64Array.of(pos[0]! + tmp[0]!, pos[1]! + tmp[1]!, pos[2]! + tmp[2]!);
    return { axes, pivots, segR, segP, toolR, toolP };
  }

  /** Tool pose (4x4, world frame). */
  fk(q: readonly number[]): number[][] {
    const f = this.#frames(this.#theta(q));
    return toPose(f.toolR, f.toolP);
  }

  /** 6xN geometric Jacobian of the tool point: rows 0-2 linear (m/rad), 3-5 angular, world frame. */
  jacobian(q: readonly number[]): number[][] {
    const f = this.#frames(this.#theta(q));
    const jac = new Float64Array(6 * this.n);
    this.#jacobianInto(f, jac, 0);
    return toRows(jac, 6, this.n);
  }

  /** `fk` and `jacobian` of B joint vectors at once: qs (B, N) -> [poses (B, 4, 4), Jacobians (B, 6, N)]. */
  fkJacobianMany(qs: readonly (readonly number[])[]): [number[][][], number[][][]] {
    if (!qs.every((q) => q.length === this.n)) {
      throw new ValueError(`need (B, ${this.n}) joint values, got (${qs.length}, ${qs[0]?.length ?? 0})`);
    }
    const { poses, jacobians } = this.fkJacobianFlat(Float64Array.from(qs.flat()), qs.length);
    const out: number[][][] = [];
    const jacs: number[][][] = [];
    for (let b = 0; b < qs.length; b++) {
      out.push(toPose(poses.subarray(12 * b, 12 * b + 9), poses.subarray(12 * b + 9, 12 * b + 12)));
      jacs.push(toRows(jacobians.subarray(6 * this.n * b, 6 * this.n * (b + 1)), 6, this.n));
    }
    return [out, jacs];
  }

  /**
   * `fkJacobianMany` on flat arrays, for the solvers: qs (B*N) -> poses (B*12: rotation row-major,
   * then position) and Jacobians (B*6*N, row-major per row).
   */
  fkJacobianFlat(qs: Float64Array, b: number): { poses: Float64Array; jacobians: Float64Array } {
    const n = this.n;
    const poses = new Float64Array(12 * b);
    const jacobians = new Float64Array(6 * n * b);
    const theta = new Float64Array(n * b);
    for (let i = 0; i < n * b; i++) theta[i] = qs[i]! * this.sign[i % n]! + this.offset[i % n]!;
    for (let r = 0; r < b; r++) {
      const f = this.#frames(theta, n * r);
      poses.set(f.toolR, 12 * r);
      poses.set(f.toolP, 12 * r + 9);
      this.#jacobianInto(f, jacobians, 6 * n * r);
    }
    return { poses, jacobians };
  }

  #jacobianInto(f: Frames, jac: Float64Array, off: number): void {
    const n = this.n;
    const { axes, pivots, toolP } = f;
    for (let i = 0; i < n; i++) {
      const a0 = axes[3 * i]!, a1 = axes[3 * i + 1]!, a2 = axes[3 * i + 2]!;
      const d0 = toolP[0]! - pivots[3 * i]!, d1 = toolP[1]! - pivots[3 * i + 1]!, d2 = toolP[2]! - pivots[3 * i + 2]!;
      const s = this.sign[i]!;
      jac[off + i] = (a1 * d2 - a2 * d1) * s;
      jac[off + n + i] = (a2 * d0 - a0 * d2) * s;
      jac[off + 2 * n + i] = (a0 * d1 - a1 * d0) * s;
      jac[off + 3 * n + i] = a0 * s;
      jac[off + 4 * n + i] = a1 * s;
      jac[off + 5 * n + i] = a2 * s;
    }
  }

  /**
   * Per joint (joint coordinates, N*m): the torque that holds the chain still at `q`, i.e. dV/dq of
   * its potential energy V = -sum(m g . c).
   */
  gravityTorques(q: readonly number[], gravity: readonly number[] = GRAVITY): number[] {
    const n = this.n;
    const { axes, pivots, segR, segP } = this.#frames(this.#theta(q));
    const coms = this.#coms;
    const m = this.masses.slice(1);
    const mc = new Float64Array(3 * n);
    for (let i = 0; i < n; i++) {
      for (let r = 0; r < 3; r++) {
        const o = 9 * i + 3 * r;
        const w = dotEinsum3(segR[o]!, segR[o + 1]!, segR[o + 2]!, coms[3 * i + 3]!, coms[3 * i + 4]!, coms[3 * i + 5]!);
        mc[3 * i + r] = m[i]! * (w + segP[3 * i + r]!);
      }
    }
    const cumM = new Float64Array(n);
    const cumMc = new Float64Array(3 * n);
    for (let i = n - 1; i >= 0; i--) {
      cumM[i] = i === n - 1 ? m[i]! : cumM[i + 1]! + m[i]!;
      for (let r = 0; r < 3; r++) cumMc[3 * i + r] = i === n - 1 ? mc[3 * i + r]! : cumMc[3 * (i + 1) + r]! + mc[3 * i + r]!;
    }
    const g0 = Number(gravity[0]), g1 = Number(gravity[1]), g2 = Number(gravity[2]);
    const tau = new Array<number>(n);
    for (let i = 0; i < n; i++) {
      const v0 = cumMc[3 * i]! - cumM[i]! * pivots[3 * i]!;
      const v1 = cumMc[3 * i + 1]! - cumM[i]! * pivots[3 * i + 1]!;
      const v2 = cumMc[3 * i + 2]! - cumM[i]! * pivots[3 * i + 2]!;
      const m0 = v1 * g2 - v2 * g1;
      const m1 = v2 * g0 - v0 * g2;
      const m2 = v0 * g1 - v1 * g0;
      tau[i] = -dotEinsum3(axes[3 * i]!, axes[3 * i + 1]!, axes[3 * i + 2]!, m0, m1, m2) * this.sign[i]!;
    }
    return tau;
  }

  /** Potential energy -sum(m g . c) of the segments after joint 1, J. */
  potentialEnergy(q: readonly number[], gravity: readonly number[] = GRAVITY): number {
    const n = this.n;
    const { segR, segP } = this.#frames(this.#theta(q));
    const coms = this.#coms;
    const terms = new Float64Array(n);
    const g0 = Number(gravity[0]), g1 = Number(gravity[1]), g2 = Number(gravity[2]);
    for (let i = 0; i < n; i++) {
      const c = [0, 1, 2].map((r) => {
        const o = 9 * i + 3 * r;
        return dotEinsum3(segR[o]!, segR[o + 1]!, segR[o + 2]!, coms[3 * i + 3]!, coms[3 * i + 4]!, coms[3 * i + 5]!)
          + segP[3 * i + r]!;
      });
      terms[i] = this.masses[i + 1]! * dot3(c[0]!, c[1]!, c[2]!, g0, g1, g2);
    }
    return -pairwiseSum(terms);
  }

  // ── inverse kinematics ──
  /**
   * Joints near `q0` that put the tool at `target`.
   *
   * `target` is a 4x4 pose, or an xyz point (with `rpy` for a full pose, else position only).
   * `positionOnly: true` ignores a pose's orientation. Damped least squares with adaptive
   * (Levenberg-Marquardt) damping; every iterate is clamped into `limits` (default: the URDF
   * limits, as [lower, upper] in joint coordinates), and the redundant directions are pulled toward
   * `q0`. `tol` (m) and `rotTol` (rad) decide `ok`.
   */
  ik(target: readonly number[] | readonly (readonly number[])[], q0In: readonly number[], options: IKOptions = {}): IKResult {
    return run(this.#ik(target, q0In, options));
  }

  /**
   * `ik` without blocking the event loop: the same iterations, with a pause for the event loop every
   * few milliseconds (`options.pause`, default a macrotask) and `options.signal` checked at each
   * pause. Returns exactly what `ik` returns.
   */
  ikAsync(target: readonly number[] | readonly (readonly number[])[], q0: readonly number[],
          options: IKOptions & CooperativeOptions = {}): Promise<IKResult> {
    return runAsync(this.#ik(target, q0, options), options);
  }

  *#ik(target: readonly number[] | readonly (readonly number[])[], q0In: readonly number[], options: IKOptions): Work<IKResult> {
    const { rpy = null, limits = null, maxIters = 100, tol = 1e-4, rotTol = 1e-3, rotWeight = 0.2, nullGain = 0.1, maxStep = 0.3 } = options;
    let positionOnly = options.positionOnly ?? null;
    let goalP: number[];
    let goalR: Float64Array | null;
    const isPose = target.length === 4 && target.every((r) => Array.isArray(r) && r.length === 4);
    const isPoint = target.length === 3 && target.every((v) => typeof v === "number");
    if (isPose) {
      const t = target as readonly (readonly number[])[];
      goalP = [t[0]![3]!, t[1]![3]!, t[2]![3]!];
      goalR = Float64Array.from([0, 1, 2].flatMap((r) => t[r]!.slice(0, 3)));
      if (positionOnly === null) positionOnly = false;
    } else if (isPoint) {
      goalP = [...(target as readonly number[])];
      goalR = rpy !== null ? Float64Array.from(rpyToMatrix(rpy).flat()) : null;
      if (positionOnly === null) positionOnly = rpy === null;
    } else {
      throw new ValueError(`target must be a 4x4 pose or an xyz point, got shape ${shapeOf(target)}`);
    }
    if (!positionOnly && goalR === null) throw new ValueError("a full-pose solve needs rpy or a 4x4 target");
    if (!(target as readonly unknown[]).flat().every((v) => Number.isFinite(v))) throw new ValueError("target must be finite");
    const lo = limits === null ? this.lower : limits[0].map(Number);
    const hi = limits === null ? this.upper : limits[1].map(Number);
    const n = this.n;
    if (q0In.length !== n || !q0In.every((v) => Number.isFinite(v))) throw new ValueError(`q0 must be ${n} finite values`);
    const q0 = q0In.map(Number);
    const rows = positionOnly ? 3 : 6;
    const weight = [1.0, 1.0, 1.0, rotWeight, rotWeight, rotWeight].slice(0, rows);

    const evaluate = (q: number[]): [number[], Float64Array] => {
      const f = this.#frames(this.#theta(q));
      const err = new Array<number>(rows);
      for (let r = 0; r < 3; r++) err[r] = goalP[r]! - f.toolP[r]!;
      if (rows === 6) {
        const ct = new Float64Array(9);
        for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) ct[3 * i + j] = f.toolR[3 * j + i]!;
        const e = rotationErrorFlat(goalR!, ct);
        err[3] = e[0]!;
        err[4] = e[1]!;
        err[5] = e[2]!;
      }
      const full = new Float64Array(6 * n);
      this.#jacobianInto(f, full, 0);
      return [err, full.subarray(0, rows * n)];
    };
    const done = (err: number[]): boolean =>
      norm(err, 3) <= tol && (rows === 3 || norm(err.slice(3)) <= rotTol);
    const costOf = (err: number[]): number => pairwiseSum(err.map((e, r) => {
      const w = weight[r]! * e;
      return w * w;
    }));

    let q = q0.map((v, i) => clip(v, lo[i]!, hi[i]!));
    let [err, jac] = evaluate(q);
    let cost = costOf(err);
    let damping = 1e-4;
    let iters = 0;
    const steps = (q: number[], err: number[], jac: Float64Array, damping: number): [number[], number[]] => {
      // The task step and the null-space pull toward q0, with joints that sit on a limit and would
      // be pushed past it frozen (a clamped joint must not absorb the step).
      const free = new Array<number>(n).fill(1.0);
      let task: number[] = [];
      let nullStep: number[] = [];
      for (let attempt = 0; attempt < 3; attempt++) {
        const jw = new Float64Array(rows * n);
        for (let r = 0; r < rows; r++) for (let c = 0; c < n; c++) jw[r * n + c] = weight[r]! * jac[r * n + c]! * free[c]!;
        const a = new Float64Array(rows * rows);
        for (let r = 0; r < rows; r++) {
          for (let s = 0; s < rows; s++) a[r * rows + s] = dotFma(jw, jw, n, r * n, 1, s * n, 1) + (r === s ? damping : 0);
        }
        const ainv = inv(a, rows);
        // pinv = jw.T @ ainv  (n x rows)
        const pinv = new Float64Array(n * rows);
        for (let c = 0; c < n; c++) {
          for (let s = 0; s < rows; s++) {
            let sum = jw[c]! * ainv[s]!;
            for (let r = 1; r < rows; r++) sum = fma(jw[r * n + c]!, ainv[r * rows + s]!, sum);
            pinv[c * rows + s] = sum;
          }
        }
        const we = err.map((e, r) => weight[r]! * e);
        task = Array.from({ length: n }, (_, c) => dotFma(pinv, we, rows, c * rows));
        const pull = q0.map((v, c) => nullGain * (v - q[c]!) * free[c]!);
        // (I - pinv @ jw) @ pull
        nullStep = Array.from({ length: n }, (_, c) => {
          const row = new Array<number>(n);
          for (let d = 0; d < n; d++) {
            let sum = pinv[c * rows]! * jw[d]!;
            for (let r = 1; r < rows; r++) sum = fma(pinv[c * rows + r]!, jw[r * n + d]!, sum);
            row[d] = (c === d ? 1 : 0) - sum;
          }
          return dotFma(row, pull, n);
        });
        const step = task.map((t, c) => t + nullStep[c]!);
        const blocked = step.map((s, c) => (q[c]! <= lo[c]! && s < 0) || (q[c]! >= hi[c]! && s > 0));
        if (!blocked.some((b, c) => b && free[c]! > 0)) break;
        blocked.forEach((b, c) => {
          if (b) free[c] = 0.0;
        });
      }
      return [task, nullStep];
    };

    while (iters < maxIters && !done(err)) {
      iters += 1;
      const [task, nullStep] = steps(q, err, jac, damping);
      let accepted = false;
      for (let step of [task.map((t, c) => t + nullStep[c]!), task]) {
        const size = Math.max(...step.map(Math.abs));
        if (size > maxStep) step = step.map((s) => s * (maxStep / size));
        const trial = q.map((v, c) => clip(v + step[c]!, lo[c]!, hi[c]!));
        const [trialErr, trialJac] = evaluate(trial);
        const trialCost = costOf(trialErr);
        if (trialCost < cost) {
          q = trial;
          err = trialErr;
          jac = trialJac;
          cost = trialCost;
          accepted = true;
          break;
        }
      }
      yield IK_ITERATION_WORK;
      if (accepted) {
        damping = Math.max(damping / 3.0, 1e-12);
      } else {
        damping *= 8.0;
        if (damping > 1e3) break;
      }
    }
    return { q, ok: done(err), posErr: norm(err, 3), rotErr: rows === 3 ? 0.0 : norm(err.slice(3)), iters };
  }
}

// ── helpers ──
/** Work estimate (µs) of one `ik` iteration, for the cooperative runner (cooperate.ts). */
const IK_ITERATION_WORK = 300;

/** Python's min(a, b) / max(a, b) of two floats. */
const pyMin = (a: number, b: number): number => (b < a ? b : a);
const pyMax = (a: number, b: number): number => (b > a ? b : a);

function toPose(r: ArrayLike<number>, p: ArrayLike<number>): number[][] {
  return [
    [r[0]!, r[1]!, r[2]!, p[0]!],
    [r[3]!, r[4]!, r[5]!, p[1]!],
    [r[6]!, r[7]!, r[8]!, p[2]!],
    [0.0, 0.0, 0.0, 1.0],
  ];
}

function toRows(a: ArrayLike<number>, rows: number, cols: number): number[][] {
  return Array.from({ length: rows }, (_, r) => Array.from({ length: cols }, (_, c) => a[r * cols + c]!));
}

function shapeOf(x: unknown): string {
  const dims: number[] = [];
  let v: unknown = x;
  while (Array.isArray(v)) {
    dims.push(v.length);
    v = v[0];
  }
  return `(${dims.join(", ")}${dims.length === 1 ? "," : ""})`;
}

function pyList(names: readonly string[]): string {
  return `[${names.map((s) => `'${s}'`).join(", ")}]`;
}

// ── profile glue ──
/**
 * The arm's chain from its profile, or null when the profile has no URDF.
 *
 * Reads `urdf` (the URDF text), `urdfJoints`, `urdfBaseRpy` (rad), `tcpXyz`, `tcpRpy` (rad) and, per
 * joint of the first N, `urdfSign` and `urdfOffset` (rad).
 */
export function chainFromConfig(config: ArmConfig): Chain | null {
  const urdf = config.urdf;
  if (!urdf) return null;
  const probe = Chain.fromUrdf(urdf, { joints: config.urdfJoints ?? null });
  const joints = config.joints.slice(0, probe.n);
  if (joints.length < probe.n) {
    throw new ValueError(`the URDF has ${probe.n} actuated joints but the profile only ${joints.length}`);
  }
  const triple = (v: readonly number[] | null | undefined): number[] => (v && v.length ? [...v] : [0.0, 0.0, 0.0]);
  return Chain.fromUrdf(urdf, {
    joints: probe.jointNames,
    tcpXyz: triple(config.tcpXyz),
    tcpRpy: triple(config.tcpRpy),
    sign: joints.map((j) => j.urdfSign ?? 1),
    offset: joints.map((j) => j.urdfOffset ?? 0.0),
    baseRpy: triple(config.urdfBaseRpy),
  });
}
