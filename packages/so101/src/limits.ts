/** The console's motion math for one SO-101 arm, in the driver's seven slots (degrees; the pad; the gripper 0..100):
the final command limiter (servo/command-limit.ts limitCommand) and the return to rest's ramp, arrival and checks
(servo/settings.ts limitTarget, easedRestStep, arrivedAtRest, returnTimeout, validateState). Unchanged in order and
arithmetic. */

import type { Range, So101ArmConfig } from "./profile.js";

/** Return to rest speed cap (each arm's own max_velocity_deg_s / max_gripper_deg_s still apply). */
export const REST_SPEED_DEG_S = 15.0;
/** Under load the position controller settles 1-2° short, so arrival allows this much. */
export const REST_ARRIVAL_TOLERANCE_DEG = 3.0;
/** Seconds the return to rest takes to reach full speed, and to slow to a stop. */
export const RAMP_EASE_S = 0.6;
/** How far short of its open endpoint a target may drive the gripper: a fully open command can stall the jaws on
their stop and heat the motor. */
export const GRIPPER_OPEN_MARGIN_DEG = 2.0;
/** The longest tick the rate caps count. */
export const LIMIT_DT_S = 0.1;
/** How fast a joint whose last command is outside its limits is brought in: a seated SO-101 elbow sits a few degrees
past dream's reference range, at most the tolerance (10°) away, two seconds at this pace. */
export const LIMIT_ENTRY_SPEED_DEG_S = 5.0;
/** The widest goal grid the driver rounds a command to (a tick is 360/4096 = 0.088°; the gripper's, under 0.07 units),
with margin: a return-to-rest command applied within this is taken as applied. */
export const REST_GRID_DEG = 0.1;
/** Feedback older than this cannot start a return to rest, seconds. */
export const FEEDBACK_MAX_AGE_S = 0.5;
/** A command that differs from its target by less than this was not clamped (rounding). */
const CLAMP_EPS_DEG = 1e-6;

/** A joint lagged its command by more than max_tracking_error_deg during the return to rest. */
export class TrackingError extends Error {
  override name = "TrackingError";
}

/** arm_limits: the six slot ranges and the gripper's (closed/open sorted). */
export function armLimits(arm: So101ArmConfig): Range[] {
  const g = [arm.gripper_closed_deg, arm.gripper_open_deg].sort((a, b) => a - b) as Range;
  return [...arm.limits_deg.map((r) => [...r] as Range), g];
}

/** Every slot clipped to armLimits. */
export function clipToLimits(q: readonly number[], arm: So101ArmConfig): number[] {
  return armLimits(arm).map(([lo, hi], i) => Math.min(hi, Math.max(lo, q[i] as number)));
}

/** clipToLimits, with the gripper also held GRIPPER_OPEN_MARGIN_DEG short of fully open (for targets). */
export function clipPolicyTarget(q: readonly number[], arm: So101ArmConfig): number[] {
  const out = clipToLimits(q, arm);
  const toward = Math.sign(arm.gripper_open_deg - arm.gripper_closed_deg);
  const open = arm.gripper_open_deg - toward * GRIPPER_OPEN_MARGIN_DEG;
  if (toward * ((out[6] as number) - open) > 0) out[6] = open;
  return out;
}

/** One arm's rate caps, deg/s (the gripper in its own unit per second). */
export interface RateCaps {
  joints: readonly number[];
  gripper: number;
}

/** What one tick's limiter did to a target. */
export interface Limited {
  command: number[];
  /** A joint or the gripper was outside the limits (or the open margin) and was brought inside. */
  position: boolean;
  /** A step was cut to the rate cap. */
  rate: boolean;
  /** A joint was outside its limits on the last command and is being brought in. */
  entering: boolean;
  /** The largest |command - target| over the six joint slots, degrees. */
  jointDeviationDeg: number;
  /** The gripper's |command - target|, percent of its stroke. */
  gripperDeviationPct: number;
}

/** The same caps on every joint. */
export function policyCaps(jointDegS: number, gripperDegS: number): RateCaps {
  return { joints: Array(6).fill(jointDegS), gripper: gripperDegS };
}

/** The limits a target must keep, per slot: clipPolicyTarget's. */
export function commandBand(arm: So101ArmConfig): [number, number][] {
  const lo = clipPolicyTarget(Array(7).fill(-Infinity), arm);
  const hi = clipPolicyTarget(Array(7).fill(Infinity), arm);
  return lo.map((v, i) => [v, hi[i] as number]);
}

/** The final command limiter: `target` within the limits and the gripper's open margin, or, for a joint whose `last`
command is outside them, between that command and the limits (never further out) moving in at
LIMIT_ENTRY_SPEED_DEG_S at most; every slot at most its rate cap from `last` over `dt` (bounded by LIMIT_DT_S). */
export function limitCommand(target: readonly number[], last: readonly number[] | undefined, arm: So101ArmConfig, caps: RateCaps, dt: number): Limited {
  if (target.length !== 7 || !target.every(Number.isFinite)) throw new RangeError(`${arm.name}: non-finite policy target`);
  const clipped = clipPolicyTarget(target, arm);
  const band = commandBand(arm);
  const span = Math.min(Math.max(Number.isFinite(dt) ? dt : 0, 0), LIMIT_DT_S);
  let position = false;
  let rate = false;
  let entering = false;
  let jointDeviationDeg = 0;
  let gripperDeviationPct = 0;
  const stroke = Math.abs(arm.gripper_open_deg - arm.gripper_closed_deg);
  const command = clipped.map((t, i) => {
    if (Math.abs(t - (target[i] as number)) > CLAMP_EPS_DEG) position = true;
    const l = last?.[i];
    let v = t;
    if (l !== undefined && Number.isFinite(l)) {
      const [lo, hi] = band[i] as [number, number];
      const outside = l < lo - CLAMP_EPS_DEG || l > hi + CLAMP_EPS_DEG;
      if (outside) entering = true;
      const cap = Math.min(i < 6 ? (caps.joints[i] as number) : caps.gripper, outside ? LIMIT_ENTRY_SPEED_DEG_S : Infinity);
      const step = cap * span;
      v = l + Math.min(step, Math.max(-step, t - l));
      if (Math.abs(v - t) > CLAMP_EPS_DEG) rate = true;
    }
    const change = Math.abs(v - (target[i] as number));
    if (i < 6) jointDeviationDeg = Math.max(jointDeviationDeg, change);
    else gripperDeviationPct = stroke > 0 ? (change / stroke) * 100 : 0;
    return v;
  });
  return { command, position, rate, entering, jointDeviationDeg, gripperDeviationPct };
}

/** control.limit_target: one rate-limited step toward target (the return to rest). A holding gripper stops at
contact, short of its command, so callers may leave it out of the tracking check. */
export function limitTarget(
  target: readonly number[],
  previous: readonly number[],
  measured: readonly number[],
  arm: So101ArmConfig,
  dt: number,
  trackGripper = true,
): number[] {
  if (!target.every(Number.isFinite) || !measured.every(Number.isFinite)) throw new RangeError("Non-finite robot state or command");
  const tracked = trackGripper ? 7 : 6;
  let lag = -Infinity;
  for (let i = 0; i < tracked; i++) lag = Math.max(lag, Math.abs((previous[i] as number) - (measured[i] as number)));
  if (lag > arm.max_tracking_error_deg) throw new TrackingError(`${arm.name}: tracking error exceeded; motion stopped`);
  const clipped = clipToLimits(target, arm);
  // Cap dt so a stalled servo never spends its accumulated time in one large step.
  const span = Math.min(Math.max(dt, 0), 0.05);
  return clipped.map((t, i) => {
    const step = (i < 6 ? arm.max_velocity_deg_s : arm.max_gripper_deg_s) * span;
    const p = previous[i] as number;
    return p + Math.min(step, Math.max(-step, t - p));
  });
}

/** `arm` with its joint speed cap eased for a ramp to a pose: rising to `jointCap` over RAMP_EASE_S from a start
`elapsed` s ago, and falling as the farthest joint nears its target (`remainingDeg` away) so the move slows to a stop
there: v = cap * sqrt(d / d0), d0 the distance a stop from full speed takes. Never below a tenth of the cap. */
export function easedArm(arm: So101ArmConfig, jointCap: number, gripperCap: number, elapsed: number, remainingDeg: number): So101ArmConfig {
  const rise = Math.min(1, Math.max(0, elapsed) / RAMP_EASE_S);
  const fall = Math.min(1, Math.sqrt(Math.max(0, remainingDeg) / ((jointCap * RAMP_EASE_S) / 2)));
  return { ...arm, max_velocity_deg_s: jointCap * Math.max(0.1, Math.min(rise, fall)), max_gripper_deg_s: gripperCap };
}

/** The arm config the return to rest ramps with: both speed caps lowered to REST_SPEED_DEG_S. */
export function restArm(arm: So101ArmConfig): So101ArmConfig {
  return {
    ...arm,
    max_velocity_deg_s: Math.min(REST_SPEED_DEG_S, arm.max_velocity_deg_s),
    max_gripper_deg_s: Math.min(REST_SPEED_DEG_S, arm.max_gripper_deg_s),
  };
}

/** control._rest_command's step (gripper left out of the tracking check). */
export function restStep(rest: readonly number[], last: readonly number[], measured: readonly number[], arm: So101ArmConfig, dt: number): number[] {
  return limitTarget(rest, last, measured, restArm(arm), dt, false);
}

/** One step of the return to rest: restStep under easedArm at REST_SPEED_DEG_S, so the arm speeds up from standstill
over RAMP_EASE_S (`elapsedS` since the return began) and slows to a stop on `rest`. */
export function easedRestStep(arm: So101ArmConfig, rest: readonly number[], last: readonly number[], measured: readonly number[], elapsedS: number, dt: number): number[] {
  const remaining = Math.max(...rest.slice(0, 6).map((r, i) => Math.abs(r - (last[i] as number))));
  const eased = easedArm(arm, Math.min(REST_SPEED_DEG_S, arm.max_velocity_deg_s), Math.min(REST_SPEED_DEG_S, arm.max_gripper_deg_s), elapsedS, remaining);
  return restStep(rest, last, measured, eased, dt);
}

/** control._begin_return: seconds allowed before the return to rest times out (90 s at most). */
export function returnTimeout(arm: So101ArmConfig, rest: readonly number[], measured: readonly number[]): number {
  let worst = 0;
  rest.forEach((r, i) => (worst = Math.max(worst, Math.abs(r - (measured[i] as number)))));
  const duration = worst / Math.min(REST_SPEED_DEG_S, arm.max_velocity_deg_s, arm.max_gripper_deg_s);
  return Math.min(90.0, duration + 5.0);
}

/** numpy.allclose(a, b, rtol=1e-5, atol). */
export function allClose(a: readonly number[], b: readonly number[], atol: number, rtol = 1e-5): boolean {
  return a.length === b.length && a.every((x, i) => Math.abs(x - (b[i] as number)) <= atol + rtol * Math.abs(b[i] as number));
}

/** Arrival at rest: the commanded ramp is at rest (atol 0.05) and every measured joint, gripper excluded (it may be
holding an object open), is within 3° of rest. A joint that started past its limit has its rest clipped to the
limit; held there it may sag back toward where it started, so anywhere from rest to `start` (± 3°) counts too. */
export function arrivedAtRest(last: readonly number[], rest: readonly number[], measured: readonly number[], start: readonly number[] = rest): boolean {
  return (
    allClose(last, rest, 0.05) &&
    measured.slice(0, 6).every((m, i) => {
      const [r, s] = [rest[i] as number, start[i] as number];
      return Math.min(r, s) - REST_ARRIVAL_TOLERANCE_DEG <= m && m <= Math.max(r, s) + REST_ARRIVAL_TOLERANCE_DEG;
    })
  );
}

const SLOT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "pad", "gripper"];

/** Python's `:g` for limits in messages. */
const fmtG = (x: number) => String(Number(x.toPrecision(6)));

/** control._validate_state: seven finite values within the limits ± `toleranceDeg`. */
export function validateState(q: readonly number[] | undefined, arm: So101ArmConfig, toleranceDeg: number): void {
  if (!q || q.length !== 7 || !q.every(Number.isFinite)) throw new RangeError(`${arm.name}: invalid robot feedback`);
  armLimits(arm).forEach(([lo, hi], i) => {
    const v = q[i] as number;
    if (!(lo - toleranceDeg <= v && v <= hi + toleranceDeg)) {
      throw new RangeError(`${arm.name}: measured position outside configured limits (${SLOT_NAMES[i]} ${v.toFixed(1)}° vs ${fmtG(lo)}–${fmtG(hi)}°)`);
    }
  });
}
