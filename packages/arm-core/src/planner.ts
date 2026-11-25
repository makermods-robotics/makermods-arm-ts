/**
 * Arm.moveEe's path planning.
 *
 * The Python SDK solves the path in a helper process so the solve never holds the interpreter lock
 * its control loop needs. Here the solve runs cooperatively on the event loop (Chain.ikAsync,
 * solvePathAsync): it pauses every few milliseconds of computation, so the control loop and the
 * receive path keep running. The limits are the Python SDK's.
 *
 * Planning is bounded. A path longer than MAX_PATH_TICKS is refused before it is built. A plan has
 * PLAN_START_S to solve the end pose; once the path length is known it gets PLAN_MARGIN_S plus
 * PLAN_S_PER_TICK per tick to solve it (again for each slower retiming), never more than
 * PLAN_CEILING_S in all. The caller's `limit` and `cancel` apply on top. The budget is checked at
 * every pause of the solve, between chunks of the path and once more at the end; a plan that runs
 * out of time or is cancelled is refused (EEPlan.q null, with the reason).
 */

import type { Clock } from "./clock.js";
import { degrees, radians } from "./config.js";
import { macrotask } from "./cooperate.js";
import * as kinematics from "./kinematics.js";
import * as trajectoryIk from "./trajectory-ik.js";

/** moveEe: how far the planned path may end from the goal (m). */
export const EE_POSITION_TOL = 1e-3;
/** moveEe: how far the planned path may end from the goal (rad). */
export const EE_ANGLE_TOL = radians(0.5);
/** Shortest moveEe path, s. */
export const EE_MIN_SECONDS = 0.25;
/** s, to solve the end pose (the Python SDK's helper also starts within it). */
export const PLAN_START_S = 5.0;
/** s, path solve time allowed per tick. */
export const PLAN_S_PER_TICK = 0.025;
/** s, added to the per-tick allowance. */
export const PLAN_MARGIN_S = 3.0;
/** s, the most any one plan may take, all retimings included. */
export const PLAN_CEILING_S = 60.0;
/** The longest path planned (60 s at 200 Hz); longer moves are refused: split them. */
export const MAX_PATH_TICKS = 12000;

/** Planning ran out of time or was cancelled. */
export class PlanStopped extends Error {
  constructor(message?: string) {
    super(message);
    this.name = "PlanStopped";
  }
}

/**
 * The time one plan may take: PLAN_START_S until the path length is known, then PLAN_MARGIN_S plus
 * PLAN_S_PER_TICK per tick, never past PLAN_CEILING_S or the caller's `limit`; `cancel()` returning
 * true stops it at once.
 */
export class Budget {
  readonly started: number;
  readonly ceiling: number;
  readonly caller: number | null;
  allowed: number;
  #clock: Clock;
  #cancel: (() => boolean) | null;

  constructor(clock: Clock, limit: number | null = null, cancel: (() => boolean) | null = null) {
    this.#clock = clock;
    this.#cancel = cancel;
    this.started = clock.now();
    this.ceiling = this.started + PLAN_CEILING_S;
    this.caller = limit === null ? null : this.started + limit;
    this.allowed = this.started + PLAN_START_S;
  }

  /** The path has `n` ticks: its allowance replaces the current one, after checking it has not run out. */
  ticks(n: number): void {
    this.check();
    this.allowed = this.#clock.now() + PLAN_MARGIN_S + n * PLAN_S_PER_TICK;
  }

  end(): number {
    let end = this.allowed;
    if (this.ceiling < end) end = this.ceiling;
    if (this.caller !== null && this.caller < end) end = this.caller;
    return end;
  }

  /** Throws PlanStopped when cancelled or out of time. */
  check(): void {
    const now = this.#clock.now();
    if (this.#cancel !== null && this.#cancel()) {
      throw new PlanStopped(`path planning cancelled after ${(now - this.started).toFixed(2)} s`);
    }
    if (now >= this.end()) throw new PlanStopped(`path planning timed out after ${(now - this.started).toFixed(2)} s`);
  }
}

/** A planned path: one row of chain joint commands per control tick, or null with the reason it was refused. */
export interface EEPlan {
  q: number[][] | null;
  reason: string;
}

export interface PlanEeOptions {
  /** Called with the number of ticks before each solve of the path. */
  progress?: ((ticks: number) => void) | null;
  /** Called between chunks of the path and at every pause; it may throw to stop the solve. */
  check?: (() => void) | null;
  /** Gives the event loop a turn (default: a macrotask). */
  pause?: () => Promise<void>;
}

/**
 * The straight-line tool path from the chain pose `q0` to `target` (and `rpy`), solved per control
 * tick. Paths longer than MAX_PATH_TICKS are refused before they are built.
 */
export async function planEe(chain: kinematics.Chain, ik: trajectoryIk.TrajectoryIK, target: readonly number[],
                             rpy: readonly number[] | null, q0: readonly number[], rate: number, speed: number | null,
                             options: PlanEeOptions = {}): Promise<EEPlan> {
  const { progress = null, check = null } = options;
  const basePause = options.pause ?? macrotask;
  const pause = async (): Promise<void> => {
    await basePause();
    if (check !== null) check();
  };
  const goalXyz = Array.from(target);
  const goalRpy = rpy === null ? null : Array.from(rpy);
  const q = Array.from(q0);
  const clipped = q.map((v, i) => Math.min(ik.hi[i]!, Math.max(ik.lo[i]!, v)));
  const end = await chain.ikAsync(goalXyz, clipped, { rpy: goalRpy, limits: [ik.lo, ik.hi], pause });
  if (!end.ok) {
    return { q: null, reason: `no solution within the limits (${(end.posErr * 1e3).toFixed(1)} mm, ${degrees(end.rotErr).toFixed(2)} deg off)` };
  }
  const start = chain.fk(q);
  const goal = chain.fk(end.q);
  for (let r = 0; r < 3; r++) goal[r]![3] = goalXyz[r]!;
  if (goalRpy !== null) {
    const rot = kinematics.rpyToMatrix(goalRpy);
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) goal[r]![c] = rot[r]![c]!;
  }
  const speeds = ik.maxVelocity.map((v: number) => Math.min(v, speed === null ? Infinity : speed));
  // Minimum jerk peaks at 1.875x the mean speed; a path the cap still shortens is retimed slower.
  let peak = -Infinity;
  end.q.forEach((v: number, i: number) => {
    peak = Math.max(peak, 1.875 * Math.abs(v - q[i]!) / speeds[i]!);
  });
  const seconds = Math.max(EE_MIN_SECONDS, 1.25 * peak);
  let ticks = Math.ceil(seconds * rate);
  let plan!: trajectoryIk.IKResult;
  for (let attempt = 0; attempt < 4; attempt++) {
    // the goal held a while at the end lets the smoothed solve settle onto it
    const hold = Math.max(Math.ceil(EE_MIN_SECONDS * rate / 2), Math.floor(ticks / 5));
    if (ticks + hold > MAX_PATH_TICKS) {
      return {
        q: null,
        reason: `the path needs ${((ticks + hold) / rate).toFixed(0)} s at this speed, longer than the `
          + `${(MAX_PATH_TICKS / rate).toFixed(0)} s planned at most: split the move or raise the speed`,
      };
    }
    // The current allowance is checked before the path's own replaces it (in progress()).
    if (check !== null) check();
    const poses = await trajectoryIk.straightLineAsync(start, goal, ticks, { pause });
    for (let k = 0; k < hold; k++) poses.push(goal.map((row) => [...row]));
    if (progress !== null) progress(poses.length);
    plan = await trajectoryIk.solvePathAsync(ik, poses, q, rate, { speedCap: speed, pause, between: check });
    if (!plan.capped.some(Boolean)) break;
    ticks = Math.ceil(ticks * 1.5);
  }
  const lastPos = plan.posErr[plan.posErr.length - 1]!, lastRot = plan.rotErr[plan.rotErr.length - 1]!;
  if (lastPos > EE_POSITION_TOL || lastRot > EE_ANGLE_TOL) {
    return { q: null, reason: `the path ends ${(lastPos * 1e3).toFixed(1)} mm, ${degrees(lastRot).toFixed(2)} deg off the goal` };
  }
  return { q: plan.q.map((row: readonly number[]) => [...row]), reason: "" };
}

export interface PlanOptions {
  clock: Clock;
  /** s; the plan is refused once this has passed. */
  limit?: number | null;
  /** True once planning should stop (the caller's cancel, or the motion superseded). */
  cancel?: (() => boolean) | null;
  /** Gives the event loop a turn (default: a macrotask). */
  pause?: () => Promise<void>;
}

/**
 * planEe under a Budget: the plan is refused (q null) after `limit` seconds, when `cancel()` is
 * true, or past the planning allowance. An error of the solve itself is thrown.
 */
export async function plan(chain: kinematics.Chain, ik: trajectoryIk.TrajectoryIK, target: readonly number[],
                           rpy: readonly number[] | null, q0: readonly number[], rate: number, speed: number | null,
                           options: PlanOptions): Promise<EEPlan> {
  const budget = new Budget(options.clock, options.limit ?? null, options.cancel ?? null);
  try {
    budget.check();
    const result = await planEe(chain, ik, target, rpy, q0, rate, speed,
      { progress: (n) => budget.ticks(n), check: () => budget.check(), pause: options.pause });
    budget.check(); // the last chunk may have crossed the limit: a late plan authorises nothing
    return result;
  } catch (e) {
    if (e instanceof PlanStopped) return { q: null, reason: e.message };
    throw e;
  }
}
