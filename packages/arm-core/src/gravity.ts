/**
 * Gravity feed-forward for the arm joints' MIT frames, from the profile's URDF.
 *
 * Per joint the torque that holds the arm still at a pose (`Chain.gravityTorques`), times the
 * profile's overall `gravityScale` and the joint's `gravityFactor` (a per-joint correction for mass
 * the URDF does not describe: cabling, gearboxes), eased in after torque-on and capped per joint at
 * `gravityLimitNm`. Angles and torques are joint coordinates (rad, N*m). The torques agree with
 * the Python SDK's to a few units in the last place (see kinematics.ts).
 */

import type { ArmConfig } from "./config.js";
import { ValueError } from "./errors.js";
import { type Chain, chainFromConfig } from "./kinematics.js";

/** 1 = the URDF model, 0 = no feed-forward. */
export const DEFAULT_GRAVITY_SCALE = 1.0;
export const MAX_GRAVITY_SCALE = 1.2;

/** s, ease-in from zero after torque-on. */
export const RAMP_S = 0.5;
/** s, the most one frame may advance the ease-in, so a pause never skips it. */
export const RAMP_STEP_S = 0.05;
/** s, a measured pose needs a reply from every joint within this. */
export const STALE_S = 0.15;

/** Options of the GravityFeedforward constructor. */
export interface GravityFeedforwardOptions {
  /** 0..MAX_GRAVITY_SCALE. */
  scale?: number;
  /** N*m, one cap per chain joint (required). */
  limitsNm?: readonly number[] | null;
  rampS?: number;
  staleS?: number;
  /** Per chain joint, 0..2 (default 1). */
  jointScale?: readonly number[] | null;
}

const fmt = (v: unknown): string => JSON.stringify(v, (_, x) => (typeof x === "number" && !Number.isFinite(x) ? String(x) : x));

/** The chain joints' torques for one session of torque-on, with its ease-in and guards. */
export class GravityFeedforward {
  readonly chain: Chain;
  readonly scale: number;
  readonly limitsNm: number[];
  readonly jointScale: number[];
  readonly rampS: number;
  readonly staleS: number;
  /** 0..1, how far into the ease-in. */
  level = 0.0;
  #last: number | null = null;

  constructor(chain: Chain, options: GravityFeedforwardOptions = {}) {
    const { scale = DEFAULT_GRAVITY_SCALE, limitsNm = null, rampS = RAMP_S, staleS = STALE_S, jointScale = null } = options;
    if (!(scale >= 0.0 && scale <= MAX_GRAVITY_SCALE)) {
      throw new ValueError(`gravity_scale must be in [0, ${MAX_GRAVITY_SCALE}], got ${scale}`);
    }
    if (limitsNm === null || limitsNm.length !== chain.n) {
      throw new ValueError(`gravity limits need one value per chain joint (${chain.n}), got ${fmt(limitsNm)}`);
    }
    if (!limitsNm.every((v) => Number.isFinite(v) && v >= 0)) {
      throw new ValueError(`gravity limits must be finite and >= 0, got ${fmt(limitsNm)}`);
    }
    const factors = jointScale === null ? new Array<number>(chain.n).fill(1.0) : [...jointScale];
    if (factors.length !== chain.n || !factors.every((f) => Number.isFinite(f) && f >= 0 && f <= 2)) {
      throw new ValueError(`gravity factors need ${chain.n} values in [0, 2], got ${fmt(factors)}`);
    }
    if (!(rampS > 0) || !(staleS > 0)) throw new ValueError("ramp_s and stale_s must be > 0");
    this.chain = chain;
    this.scale = scale;
    this.limitsNm = limitsNm.map(Number);
    this.jointScale = factors.map(Number);
    this.rampS = rampS;
    this.staleS = staleS;
  }

  get n(): number {
    return this.chain.n;
  }

  /** Holding torque at `q` with the scales, before the ease-in and caps; NaN for a non-finite pose. */
  model(q: readonly number[]): number[] {
    if (!q.every((v) => Number.isFinite(v))) return new Array<number>(this.n).fill(NaN);
    return this.chain.gravityTorques(q).map((t, i) => t * this.jointScale[i]! * this.scale);
  }

  /** Start the ease-in again (call at torque-on). */
  reset(): void {
    this.level = 0.0;
    this.#last = null;
  }

  /**
   * Torques for the frames about to go out, one step further into the ease-in: at the measured
   * pose when every joint replied within `staleS`, else at the goal. Non-finite values become 0.
   */
  torques(measuredQ: readonly number[], ages: readonly number[], goalQ: readonly number[], now: number): number[] {
    if (this.#last !== null) {
      this.level = pyMin(1.0, this.level + pyMin(RAMP_STEP_S, pyMax(0.0, now - this.#last)) / this.rampS);
    }
    this.#last = now;
    const pairs = Math.min(measuredQ.length, ages.length);
    let fresh = true;
    for (let i = 0; i < pairs; i++) {
      if (!(Number.isFinite(measuredQ[i]!) && ages[i]! <= this.staleS)) {
        fresh = false;
        break;
      }
    }
    const tau = this.model((fresh ? measuredQ : goalQ).slice(0, this.n));
    const out: number[] = [];
    for (let i = 0; i < Math.min(tau.length, this.limitsNm.length); i++) {
      const t = tau[i]!;
      const limit = this.limitsNm[i]!;
      out.push(Number.isFinite(t) ? pyMax(-limit, pyMin(limit, t * this.level)) : 0.0);
    }
    return out;
  }
}

/** Python's min(a, b) / max(a, b) of two floats: the first argument unless the second is strictly beyond it. */
function pyMin(a: number, b: number): number {
  return b < a ? b : a;
}
function pyMax(a: number, b: number): number {
  return b > a ? b : a;
}

/**
 * The profile's feed-forward, or null when it has no URDF or its `gravityScale` is 0.
 *
 * Reads `gravityScale` and, for each of the chain's joints, `gravityLimitNm` (required) and
 * `gravityFactor` (default 1).
 */
export function fromConfig(config: ArmConfig, chain: Chain | null = null): GravityFeedforward | null {
  const scale = config.gravityScale ?? DEFAULT_GRAVITY_SCALE;
  if (!scale) return null;
  const c = chain ?? chainFromConfig(config);
  if (c === null) return null;
  const joints = config.joints.slice(0, c.n);
  const limits = joints.map((j) => j.gravityLimitNm ?? null);
  const missing = joints.flatMap((j, i) => (limits[i] === null ? [j.name || String(i)] : []));
  if (missing.length) {
    throw new ValueError(`gravity feed-forward needs gravity_limit_nm on joints [${missing.map((m) => `'${m}'`).join(", ")}]`);
  }
  return new GravityFeedforward(c, {
    scale, limitsNm: limits as number[], jointScale: joints.map((j) => j.gravityFactor ?? 1.0),
  });
}
