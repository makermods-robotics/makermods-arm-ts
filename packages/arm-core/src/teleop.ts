/**
 * Teleoperation: a leader drives a follower arm. The reference loop, written to be read.
 *
 *     await leader.connect();
 *     await follower.connect(); await follower.enable();
 *     const reason = await new Teleop(leader, follower).run(signal); // why it stopped; the arm is holding
 *     await follower.stop();                                         // return to rest, then release
 *     await leader.close();
 *
 * Each cycle reads the leader (follower joint targets, rad), shapes them (a slow approach at the
 * start when the poses differ, then an optional speed/acceleration cap), and hands them to
 * `follower.setJointTargets`. The follower's own control loop does the rest (rate limit, soft
 * limits, gravity, faults). The loop stops, and the follower holds its measured pose, when:
 *
 * - the signal is aborted (`"stopped"`),
 * - the leader throws LeaderError or misses `missLimit` reads in a row (`"leader_lost: ..."`),
 * - the follower leaves ENABLED, e.g. on a fault (`"follower_fault: ..."`),
 * - the stall hook reports a stall (`"stall: motor N"`).
 *
 * What happens next (return to rest, release, retry) is the caller's decision. Bimanual: one
 * Teleop per pair, run together with runMany().
 */

import { type Clock, RealClock } from "./clock.js";
import { ValueError } from "./errors.js";
import { type Leader, LeaderError } from "./leaders/base.js";
import type { Logger } from "./logger.js";

/** onStep promises that may be unsettled at once; beyond it, cycles skip their onStep call. */
export const MAX_PENDING_HOOKS = 8;

/** The per-cycle hook; it may return a promise, which the loop observes but never awaits. */
export type OnStep = (step: TeleopStep) => void | PromiseLike<unknown>;

/** s: the most time one cycle may claim for the speed cap, so a stall never banks motion. */
export const MAX_STEP_S = 0.05;

/** What one cycle did, for `onStep` (logging, recording, a UI). */
export class TeleopStep {
  constructor(
    readonly elapsedS: number,
    /** null when this cycle's read missed. */
    readonly leader: number[] | null,
    /** What went to setJointTargets. */
    readonly command: number[],
    readonly syncing: boolean,
  ) {
    Object.freeze(this);
  }
}

const pyMin = (a: number, b: number): number => (b < a ? b : a);
const pyMax = (a: number, b: number): number => (b > a ? b : a);
/** math.copysign. */
const copysign = (x: number, y: number): number => (y < 0 || Object.is(y, -0) ? -Math.abs(x) : Math.abs(x));
const isInf = (x: number): boolean => x === Infinity || x === -Infinity;

/**
 * Per-joint speed and acceleration limit on a moving target (rad, rad/s, rad/s²).
 *
 * The command speeds up at most `maxAcceleration` and starts braking early enough to stop on the
 * goal instead of overshooting it. A joint with both limits infinite follows its goal exactly.
 */
export class SpeedCap {
  position: number[];
  velocity: number[];

  constructor(start: readonly number[]) {
    this.position = [...start];
    this.velocity = new Array<number>(this.position.length).fill(0.0);
  }

  /** Advance by `dt` s toward `goal`; returns the new command. */
  step(goal: readonly number[], dt: number, maxVelocity: readonly number[], maxAcceleration: readonly number[]): number[] {
    const n = Math.min(goal.length, maxVelocity.length, maxAcceleration.length);
    for (let i = 0; i < n; i++) {
      const target = goal[i]!;
      const vmax = maxVelocity[i]!;
      const accel = maxAcceleration[i]!;
      const error = target - this.position[i]!;
      if (isInf(vmax) && isInf(accel)) {
        this.position[i] = target;
        this.velocity[i] = 0.0;
        continue;
      }
      if (dt <= 0) continue;
      const change = accel * dt;
      // Fastest speed that can still brake to a stop on the goal, counted in whole ticks.
      const braking = isInf(accel) ? vmax : 0.5 * (-change + Math.sqrt(change * change + 8.0 * Math.abs(error) * change / dt));
      const wanted = copysign(pyMin(vmax, braking), error);
      let v = isInf(accel) ? wanted : this.velocity[i]! + pyMax(-change, pyMin(change, wanted - this.velocity[i]!));
      v = pyMax(-vmax, pyMin(vmax, v));
      let move = v * dt;
      if ((error >= 0 && move > error) || (error <= 0 && move < error)) {
        move = error;
        if (isInf(accel)) v = 0.0;
      }
      this.position[i] = this.position[i]! + move;
      this.velocity[i] = v;
    }
    return [...this.position];
  }
}

/**
 * Something that reports stalls (MotorStateRecorder does). An async record() is awaited (its
 * events decide whether the loop stops); an abort does not wait for it.
 */
export interface StallHook {
  record(elapsedS: number, leaderTargets: number[]): ReadonlyArray<{ readonly motorId: number }>
    | Promise<ReadonlyArray<{ readonly motorId: number }>>;
}

/** What Teleop uses of the follower Arm. */
export interface TeleopFollower {
  readonly state: string;
  readonly config: { readonly joints: readonly { readonly name: string }[] };
  readonly faultReason?: string | null;
  /** The arm's clock: the loop's default. */
  readonly clock?: Clock;
  getJointPositions(): number[];
  setJointTargets(targets: readonly number[]): boolean;
  holdCurrentPosition(): boolean;
}

/** Options of the Teleop constructor. */
export interface TeleopOptions {
  /** Leader reads per second (default 100). */
  rateHz?: number;
  /** rad/s cap on every joint's command (default: none beyond the follower's own). */
  maxVelocity?: number | null;
  /** rad/s² cap (default: none). */
  maxAcceleration?: number | null;
  /** rad: a larger startup gap between leader and follower is closed slowly (default 0.8) ... */
  syncThreshold?: number;
  /** ... at this speed (rad/s, default 0.3), before normal following starts. */
  syncVelocity?: number;
  /** Consecutive leader misses that stop following (default 25). */
  missLimit?: number;
  /** s to wait for the first leader reading (default 2). */
  startupTimeout?: number;
  /** `stall.record(elapsedS, targets)` returns events with `.motorId` (MotorStateRecorder). */
  stall?: StallHook | null;
  /**
   * Called after every cycle, never awaited. A promise it returns is observed (a rejection is
   * logged); while MAX_PENDING_HOOKS of them are unsettled, further calls are skipped (logged).
   */
  onStep?: OnStep | null;
  /** Joint names the speed cap leaves alone after the sync (default: the gripper). */
  exempt?: readonly string[];
  /** Default: the follower's clock, else a RealClock. */
  clock?: Clock;
  /** Default: console. */
  logger?: Logger;
}

const stateOf = (arm: { readonly state: unknown }): string => String(arm.state).toUpperCase();
const errorText = (e: unknown): string => (e instanceof Error ? e.message : String(e));
const pyStr = (v: unknown): string => (v === null || v === undefined ? "None" : String(v));

/** One leader driving one follower. */
export class Teleop {
  readonly leader: Leader;
  readonly follower: TeleopFollower;
  readonly period: number;
  readonly syncThreshold: number;
  readonly syncVelocity: number;
  readonly missLimit: number;
  readonly startupTimeout: number;
  readonly stall: StallHook | null;
  readonly onStep: OnStep | null;
  readonly clock: Clock;
  /** rad, the largest leader-follower difference at startup (null before). */
  startupGap: number | null = null;
  /** Why the last run stopped (null before). */
  stopReason: string | null = null;
  /** onStep calls skipped because MAX_PENDING_HOOKS earlier ones had not finished. */
  droppedSteps = 0;
  #pendingHooks = 0;
  #log: Logger;
  #followVelocity: number[];
  #followAcceleration: number[];
  #syncAcceleration: number[];

  /** Throws ValueError for a rate, sync speed, sync threshold, miss limit or cap out of range. */
  constructor(leader: Leader, follower: TeleopFollower, options: TeleopOptions = {}) {
    const {
      rateHz = 100.0, maxVelocity = null, maxAcceleration = null, syncThreshold = 0.8, syncVelocity = 0.3,
      missLimit = 25, startupTimeout = 2.0, stall = null, onStep = null, exempt = ["gripper"],
    } = options;
    if (!(rateHz > 0) || !(syncVelocity > 0) || !(syncThreshold >= 0) || missLimit < 1) {
      throw new ValueError("rate_hz and sync_velocity must be > 0, sync_threshold >= 0, miss_limit >= 1");
    }
    for (const [name, value] of [["max_velocity", maxVelocity], ["max_acceleration", maxAcceleration]] as const) {
      if (value !== null && !(value > 0)) throw new ValueError(`${name} must be > 0`);
    }
    this.leader = leader;
    this.follower = follower;
    this.period = 1.0 / rateHz;
    this.syncThreshold = syncThreshold;
    this.syncVelocity = syncVelocity;
    this.missLimit = missLimit;
    this.startupTimeout = startupTimeout;
    this.stall = stall;
    this.onStep = onStep;
    this.clock = options.clock ?? follower.clock ?? new RealClock();
    this.#log = options.logger ?? console;
    const names = follower.config.joints.map((j) => j.name);
    const cap = maxVelocity === null ? Infinity : maxVelocity;
    const accel = maxAcceleration === null ? Infinity : maxAcceleration;
    this.#followVelocity = names.map((n) => (exempt.includes(n) ? Infinity : cap));
    this.#followAcceleration = names.map((n) => (exempt.includes(n) ? Infinity : accel));
    this.#syncAcceleration = names.map(() => accel);
  }

  // ── the loop ──
  /**
   * Follow until a stop condition; resolves to its reason with the follower holding its pose. Never
   * rejects.
   *
   * Aborting `signal` puts the follower into hold at once, from the abort itself, and the run then
   * resolves to "stopped" without waiting for a leader read or stall hook in progress (what such a
   * call returns later is discarded). Only the signal stops a run: dropping the returned promise
   * does not.
   */
  async run(signal?: AbortSignal): Promise<string> {
    const stop = new AbortController();
    const onAbort = (): void => {
      stop.abort();
      this.#hold();
    };
    if (signal?.aborted) stop.abort();
    else signal?.addEventListener("abort", onAbort, { once: true });
    try {
      this.stopReason = await this.#follow(stop.signal);
    } catch (e) {
      this.#log.error(`teleop loop failed: ${e instanceof Error && e.stack ? e.stack : String(e)}`);
      this.stopReason = `error: ${errorText(e)}`;
    } finally {
      signal?.removeEventListener("abort", onAbort);
      this.#hold();
    }
    this.#log.info(`teleop stopped: ${this.stopReason}`);
    return this.stopReason;
  }

  /** Hold the follower's measured pose if it is ENABLED or in FAULT (never throws). */
  #hold(): void {
    try {
      const state = stateOf(this.follower);
      if (state === "ENABLED" || state === "FAULT") this.follower.holdCurrentPosition();
    } catch (e) {
      this.#log.error(`teleop: holding the follower failed: ${errorText(e)}`);
    }
  }

  /** The leader's reading, or ABORTED as soon as `signal` aborts. */
  #read(signal: AbortSignal): Promise<number[] | null | typeof ABORTED> {
    return untilAbort(Promise.resolve().then(() => this.leader.read(signal)), signal);
  }

  /** Sleep `seconds`, or less when `signal` aborts. */
  async #sleep(seconds: number, signal: AbortSignal): Promise<void> {
    await untilAbort(this.clock.sleep(seconds), signal);
  }

  /** Call onStep; a promise it returns is observed (a rejection is logged), never awaited. */
  #callOnStep(step: TeleopStep): void {
    if (this.onStep === null) return;
    if (this.#pendingHooks >= MAX_PENDING_HOOKS) {
      if (this.droppedSteps++ % 100 === 0) {
        this.#log.warn(`onStep skipped: ${this.#pendingHooks} earlier calls have not finished (${this.droppedSteps} skipped)`);
      }
      return;
    }
    let result: unknown;
    try {
      result = this.onStep(step);
    } catch (e) {
      this.#log.error(`onStep failed: ${errorText(e)}`);
      return;
    }
    if (result !== null && typeof result === "object" && typeof (result as PromiseLike<unknown>).then === "function") {
      this.#pendingHooks += 1;
      Promise.resolve(result as PromiseLike<unknown>).then(
        () => undefined,
        (e: unknown) => this.#log.error(`onStep failed: ${errorText(e)}`),
      ).finally(() => {
        this.#pendingHooks -= 1;
      });
    }
  }

  async #firstReading(signal: AbortSignal): Promise<number[] | null> {
    const deadline = this.clock.now() + this.startupTimeout;
    while (!signal.aborted && this.clock.now() < deadline) {
      const targets = await this.#read(signal);
      if (targets === ABORTED) return null;
      if (targets !== null) return targets;
      await this.#sleep(this.period, signal);
    }
    return null;
  }

  async #follow(signal: AbortSignal): Promise<string> {
    if (stateOf(this.follower) !== "ENABLED") return `follower_not_enabled: ${stateOf(this.follower)}`;
    let goal: number[] | null;
    try {
      goal = await this.#firstReading(signal);
    } catch (e) {
      if (e instanceof LeaderError) return `leader_lost: ${e.message}`;
      throw e;
    }
    if (goal === null) return signal.aborted ? "stopped" : "leader_lost: no reading at startup";

    // Startup alignment: a large gap is closed at syncVelocity, never at follow speed.
    const present = this.follower.getJointPositions();
    const n = Math.min(goal.length, present.length);
    let gap = Math.abs(goal[0]! - present[0]!);
    for (let i = 1; i < n; i++) gap = pyMax(gap, Math.abs(goal[i]! - present[i]!));
    this.startupGap = gap;
    let syncing = this.startupGap > this.syncThreshold;
    if (syncing) {
      this.#log.warn(`leader and follower differ by ${this.startupGap.toFixed(2)} rad: syncing slowly at `
        + `${this.syncVelocity.toFixed(2)} rad/s`);
    }
    const shaper = new SpeedCap(present);
    const syncVelocity = goal.map(() => this.syncVelocity);

    let misses = 0;
    const start = this.clock.now();
    let last = start - this.period; // the first cycle moves one period's worth
    while (!signal.aborted) {
      const now = this.clock.now();
      const dt = pyMin(pyMax(now - last, 0.0), MAX_STEP_S);
      last = now;

      // 1. read the leader; a miss keeps the last goal
      let reading: number[] | null;
      try {
        const got = await this.#read(signal);
        if (got === ABORTED) return "stopped";
        reading = got;
      } catch (e) {
        if (signal.aborted) return "stopped";
        if (e instanceof LeaderError) return `leader_lost: ${e.message}`;
        this.#log.warn(`leader read failed: ${errorText(e)}`);
        reading = null;
      }
      if (reading === null) {
        misses += 1;
        if (misses >= this.missLimit) return `leader_lost: ${misses} reads missed in a row`;
      } else {
        misses = 0;
        goal = reading;
      }

      // 2. shape it and send it
      const command = syncing
        ? shaper.step(goal, dt, syncVelocity, this.#syncAcceleration)
        : shaper.step(goal, dt, this.#followVelocity, this.#followAcceleration);
      if (syncing && command.slice(0, goal.length).every((c, i) => Math.abs(c - goal![i]!) < 1e-9)) {
        syncing = false;
        this.#log.info("leader and follower aligned: following");
      }
      if (signal.aborted) return "stopped";
      if (!this.follower.setJointTargets(command) && stateOf(this.follower) !== "ENABLED") {
        return `follower_${stateOf(this.follower).toLowerCase()}: ${pyStr(this.follower.faultReason)}`;
      }

      // 3. hooks
      const elapsed = now - start;
      if (this.stall !== null) {
        const stall = this.stall;
        const goalNow = [...goal];
        const events = await untilAbort(Promise.resolve().then(() => stall.record(elapsed, goalNow)), signal);
        if (events === ABORTED) return "stopped";
        if (events.length) return `stall: motor ${events[0]!.motorId}`;
      }
      this.#callOnStep(new TeleopStep(elapsed, reading, command, syncing));

      const remain = this.period - (this.clock.now() - now);
      // An overrun cycle still yields once, so an abort (a click, a timer) can get through.
      await this.#sleep(remain > 0 ? remain : 0, signal);
    }
    return "stopped";
  }
}

/**
 * Run several pairs (bimanual) concurrently. When one stops, for any reason, all stop: every
 * follower is put into hold at that moment, not when its own loop comes round again. One arm
 * carrying on alone is never what the operator meant. Resolves to each reason; never rejects.
 */
export async function runMany(teleops: readonly Teleop[], signal?: AbortSignal): Promise<string[]> {
  const shared = new AbortController();
  const stopAll = (): void => shared.abort();
  if (signal?.aborted) shared.abort();
  else signal?.addEventListener("abort", stopAll, { once: true });
  try {
    const reasons = await Promise.all(teleops.map(async (t) => {
      try {
        return await t.run(shared.signal);
      } finally {
        shared.abort();
      }
    }));
    return reasons.map((r) => r || "stopped");
  } finally {
    signal?.removeEventListener("abort", stopAll);
  }
}

const ABORTED: unique symbol = Symbol("aborted");

/** `promise`'s outcome, or ABORTED as soon as `signal` aborts (a later rejection is still observed). */
function untilAbort<T>(promise: Promise<T>, signal: AbortSignal): Promise<T | typeof ABORTED> {
  if (signal.aborted) {
    promise.catch(() => undefined);
    return Promise.resolve(ABORTED);
  }
  return new Promise<T | typeof ABORTED>((resolve, reject) => {
    const onAbort = (): void => resolve(ABORTED);
    signal.addEventListener("abort", onAbort, { once: true });
    promise.then(
      (value) => {
        signal.removeEventListener("abort", onAbort);
        resolve(value);
      },
      (error: unknown) => {
        signal.removeEventListener("abort", onAbort);
        reject(error);
      },
    );
  });
}
