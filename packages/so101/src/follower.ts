/** One SO-101 follower while it is energized: the console's So101Follower (dreamlabs FeetechChain's open, enable,
read, command and relax), unchanged in what it sends. So101Arm creates one per enable().

- Starting switches the servos off while they are configured (P 16, I 0, D 32, position mode, the gripper's torque
  caps; LeRobot's SO follower), then each holds where it is: its goal is set to its reading before the torque goes
  on. A take-over keeps the torque on and only re-reads the pose.
- A read takes every position in one sync read; a command writes every goal in one sync write. A read that fails
  holds the tick (MissedReadError); MAX_MISSED_MS without one fails the arm. A servo that reports a fault, or dropped
  its torque while energized (an overload), fails the arm for good (latched).
- The switch-off goes servo by servo (Torque_Enable 0, then Lock 0: LeRobot's disable_torque), each confirmed by a
  read-back, so one that refuses leaves the others off and is named. */

import type { Arbiter, FeetechBus, So101Link } from "./bus.js";
import { MODE_POSITION, errorText, isFault, type ServoState } from "./protocol.js";
import {
  D_COEFFICIENT,
  FIRST_READ_ATTEMPTS,
  FIRST_READ_RETRY_MS,
  GRIPPER_CAPS,
  MAX_COMMAND_DEG_S,
  MAX_COMMAND_DT_S,
  MAX_MISSED_MS,
  MissedReadError,
  PAD_SLOT,
  P_COEFFICIENT,
  ReleaseError,
  SO101_IDS,
  SO101_MOTORS,
  So101Error,
  So101FaultError,
  StartCancelledError,
  calibrationDifferences,
  disableAllServos,
  readHeldCalibration,
  readStates,
  ticksToVector,
  vectorToTicks,
  type So101Calibration,
} from "./so101.js";

/** What a follower needs. */
export interface So101FollowerOptions {
  name: string;
  link: So101Link;
  calibration: So101Calibration;
  /** Soft limits per slot of the seven (five joints, the pad, the gripper), degrees (the gripper 0..100). */
  armLimits: ReadonlyArray<readonly [number, number]>;
  /** Sees every state batch the follower reads, complete or not (for feedback and status). */
  onBatch?: (batch: ReadonlyMap<number, ServoState>) => void;
}

/** One SO-101 follower: open/enable/read/command/relax over one link. */
export class So101Follower {
  readonly bus: FeetechBus;
  readonly arbiter: Arbiter;
  cleanupProblems: string[] = [];
  private open = true;
  private energized = false;
  private last: number[] | null = null;
  private missedSince: number | null = null;
  /** The last goal sent (degrees) and when (clock ms), for the step bound. */
  private goal: number[] | null = null;
  private commandAt: number | null = null;
  /** A fault this arm reported (latched): it is never read as healthy or commanded again. */
  fault: So101FaultError | null = null;
  /** The newest complete state read, servo order, and when it arrived (clock ms). */
  states: ServoState[] | null = null;
  statesAt = -Infinity;

  constructor(private readonly options: So101FollowerOptions) {
    this.bus = options.link.bus;
    this.arbiter = options.link.arbiter;
  }

  get name(): string {
    return this.options.name;
  }

  get isOpen(): boolean {
    return this.open;
  }

  /** Whether the start has switched any servo on that has not been confirmed off since. */
  get isEnergized(): boolean {
    return this.energized;
  }

  private guard(signal?: AbortSignal) {
    if (signal?.aborted) throw new StartCancelledError(`${this.name}: start cancelled`);
  }

  /** FeetechChain.open + enable. With `takeOver`, an arm left holding keeps its torque: no configure (which needs the
  torque off), each servo's goal set to its reading, and the torque checked on. The signal is checked before every
  write: a stop cancels the rest of the start (the caller then switches off what did start). A servo reporting a
  fault refuses the start. */
  async connect(signal?: AbortSignal, options: { takeOver?: boolean } = {}): Promise<number[]> {
    const { calibration } = this.options;
    const clock = this.options.link.clock;
    const differ = calibrationDifferences(calibration, await this.arbiter.run(() => readHeldCalibration(this.bus), { label: "held calibration" }));
    if (differ.length) throw new So101Error(`${this.name} holds another calibration than its file: ${differ.join(", ")}`);
    this.guard(signal);
    // Every startup batch is inspected (partial ones too) and a fault latched: a start never enables an arm a servo
    // reported a fault on, and a fault is never retried away as a missed reply.
    const startup = (b: ReadonlyMap<number, ServoState>) => this.inspect(b);
    await this.arbiter.run(() => readStates(this.bus, startup), { label: "fault check" }).catch((exc: unknown) => {
      if (!this.fault) throw exc;
    });
    if (this.fault) throw this.fault;
    this.guard(signal);
    if (!options.takeOver) {
      await this.arbiter.run(async () => {
        // LeRobot writes these registers with the torque off: the arm must be resting.
        const refused = await this.disableAll();
        if (refused.length) throw new So101Error(`${this.name}: ${refused.join("; ")}`);
        for (const id of SO101_IDS) {
          this.guard(signal);
          await this.bus.write(id, "Return_Delay_Time", 0);
          await this.bus.write(id, "Maximum_Acceleration", 254);
          await this.bus.write(id, "Acceleration", 254);
          await this.bus.write(id, "Operating_Mode", MODE_POSITION);
          await this.bus.write(id, "P_Coefficient", P_COEFFICIENT);
          await this.bus.write(id, "I_Coefficient", 0);
          await this.bus.write(id, "D_Coefficient", D_COEFFICIENT);
          // RAM left over from an auto-calibration this power cycle (MakerLab motor_power.py): its 380 torque limit
          // and its probe speed would weaken and throttle the run.
          await this.bus.write(id, "Torque_Limit", id === 6 ? 500 : 1000);
          await this.bus.write(id, "Goal_Velocity", 0);
        }
        for (const [reg, value] of GRIPPER_CAPS) await this.bus.write(6, reg, value);
      }, { label: "configure" });
    }
    this.guard(signal);
    let states: ServoState[] | null = null;
    for (let attempt = 0; attempt < FIRST_READ_ATTEMPTS && !states; attempt++) {
      try {
        states = await this.arbiter.run(() => readStates(this.bus, startup), { label: "first read" });
      } catch {
        if (this.fault) throw this.fault;
        await clock.sleep(FIRST_READ_RETRY_MS, signal);
        this.guard(signal);
      }
    }
    if (this.fault) throw this.fault;
    if (!states) throw new So101Error(`${this.name}: no pose in ${FIRST_READ_ATTEMPTS} reads`);
    this.guard(signal);
    const held = states.map((st) => st.position);
    await this.arbiter.run(async () => {
      // Each servo holds where it is: its goal is its reading before the torque goes on.
      this.guard(signal);
      if (this.fault) throw this.fault;
      await this.bus.syncWrite("Goal_Position", SO101_IDS.map((id, i) => [id, held[i] as number] as const));
      for (const id of SO101_IDS) {
        this.guard(signal);
        this.energized = true;
        await this.bus.write(id, "Torque_Enable", 1);
      }
      for (const id of SO101_IDS) await this.bus.write(id, "Lock", 1);
    }, { priority: true, label: "enable" });
    const after = await this.arbiter.run(() => readStates(this.bus, startup), { label: "torque check" }).catch((exc: unknown) => {
      throw this.fault ?? exc;
    });
    if (this.fault) throw this.fault;
    const off = after.flatMap((st, i) => (st.torque ? [] : [SO101_MOTORS[i]]));
    if (off.length) throw new So101Error(`${this.name}: ${off.join(", ")} did not switch on`);
    this.states = after;
    this.statesAt = clock.now();
    this.last = ticksToVector(held, calibration);
    this.goal = [...this.last];
    this.commandAt = clock.now();
    return [...this.last];
  }

  /** Every servo's state in one sync read: the pose (degrees), and each servo's torque and fault bits. A read that got
  no complete answer throws MissedReadError (the caller holds the tick: nothing is commanded on stale feedback) until
  MAX_MISSED_MS without one fails the arm. A servo that reports a fault, or dropped its torque while energized, fails
  the arm for good (latched). */
  async read(): Promise<number[]> {
    if (this.fault) throw this.fault;
    const { calibration } = this.options;
    const clock = this.options.link.clock;
    let states: ServoState[];
    try {
      // Faults and torque loss are judged inside the read's transaction, on every batch that arrives: a command
      // queued behind this read (a hold frame) then finds the latch set before it can write.
      states = await this.arbiter.run(() => readStates(this.bus, (batch) => this.inspect(batch)), { label: "read" });
    } catch (exc) {
      if (this.fault) throw this.fault;
      if (!(exc instanceof MissedReadError)) throw exc;
      const now = clock.now();
      this.missedSince ??= now;
      if (now - this.missedSince > MAX_MISSED_MS) throw new So101Error(`${this.name}: no position for ${(now - this.missedSince).toFixed(0)} ms: ${exc.message}`);
      throw exc;
    }
    if (this.fault) throw this.fault;
    this.missedSince = null;
    this.last = ticksToVector(states.map((st) => st.position), calibration);
    this.states = states;
    this.statesAt = clock.now();
    return [...this.last];
  }

  /** Latches a fault, or a servo that dropped its torque while energized, from whatever servos answered. */
  private inspect(batch: ReadonlyMap<number, ServoState>): void {
    this.options.onBatch?.(batch);
    if (this.fault) return;
    const states = SO101_IDS.map((id) => batch.get(id));
    const faulted = states.flatMap((st, i) => (st && isFault(st) ? [`${SO101_MOTORS[i]} (${errorText(st.error || 0x20)})`] : []));
    if (faulted.length) {
      this.fault = new So101FaultError(`${this.name} ${faulted.join(", ")}: the servo reports a fault. Support the arm, then check nothing holds it`);
      return;
    }
    if (!this.energized) return;
    const off = states.flatMap((st, i) => (st && !st.torque ? [SO101_MOTORS[i]] : []));
    if (off.length) this.fault = new So101FaultError(`${this.name} ${off.join(", ")}: torque switched off by the servo (an overload: it pushed into a stop or an object). Support the arm, then check nothing holds it`);
  }

  /** Each slot's bounds for a goal: the soft limits, widened to `from` (the last goal) for a joint already past them,
  so an arm that rests past a limit is brought in by the command limiter's ramp, never snapped there, and never sent
  further out. */
  private bounds(from: readonly number[] | null): [number, number][] {
    return this.options.armLimits.map(([lo, hi], i) => {
      const f = i < 6 ? from?.[i] : undefined;
      return f === undefined || !Number.isFinite(f) ? [lo, hi] : [Math.min(lo, f), Math.max(hi, f)];
    });
  }

  /** One goal per servo in one sync write; resolves to what was applied (degrees of the goal ticks). */
  async sendAction(action: readonly number[], epoch?: number, priority = false): Promise<number[]> {
    if (action.length !== 7 || !action.every(Number.isFinite)) throw new RangeError(`${this.name}: non-finite command`);
    if (this.fault) throw this.fault;
    const { calibration } = this.options;
    const clock = this.options.link.clock;
    let applied: number[] = [];
    await this.arbiter.run(
      async () => {
        this.arbiter.check(epoch, "command");
        if (this.fault) throw this.fault; // latched by a read that ran ahead of this command
        // At most MAX_COMMAND_DEG_S from the last goal (over the time since it, capped): a goal far from the last one
        // (a gap in playback, a stale target) is approached, never jumped to at the servos' top speed.
        const now = clock.now();
        const dt = this.commandAt === null ? MAX_COMMAND_DT_S : Math.min(MAX_COMMAND_DT_S, Math.max(0, now - this.commandAt) / 1000);
        const step = MAX_COMMAND_DEG_S * dt;
        const from = this.goal ?? this.last;
        const bounds = this.bounds(from);
        const target = action
          .map((v, i) => Math.min((bounds[i] as [number, number])[1], Math.max((bounds[i] as [number, number])[0], v)))
          .map((v, i) => (from && i < 6 ? Math.min((from[i] as number) + step, Math.max((from[i] as number) - step, v)) : v));
        const ticks = vectorToTicks(target, calibration, this.options.armLimits);
        await this.bus.syncWrite("Goal_Position", SO101_IDS.map((id, i) => [id, ticks[i] as number] as const));
        applied = ticksToVector(ticks, calibration);
        applied[PAD_SLOT] = 0;
        this.goal = applied;
        this.commandAt = now;
      },
      { label: "command", epoch, priority },
    );
    return [...applied];
  }

  /** The limits' clamp, then the goal ticks sendAction would write for it (rounded inside the limits, the gripper
  GRIP_MARGIN inside its open end), in degrees: the goal sendAction applies for `q` once it is reached. */
  goalFor(q: readonly number[]): number[] {
    const { calibration, armLimits } = this.options;
    const clamped = this.bounds(null).map(([lo, hi], i) => Math.min(hi, Math.max(lo, q[i] as number)));
    const ticks = vectorToTicks(clamped, calibration, armLimits);
    const goal = ticksToVector(ticks, calibration);
    goal[PAD_SLOT] = 0;
    return goal;
  }

  /** Best effort, ahead of anything queued: every servo's goal at `q`. */
  holdFrame(q: readonly number[]): Promise<void> {
    return this.sendAction(q, undefined, true).then(() => undefined);
  }

  /** LeRobot's disable_torque per servo, each retried and confirmed by reading Torque_Enable back. */
  private disableAll(): Promise<string[]> {
    return disableAllServos(this.bus);
  }

  /** Torque off with read-back, ahead of anything queued; the port stays open. ReleaseError names every servo that
  did not confirm it. */
  async switchOffAll(): Promise<void> {
    const failed = await this.arbiter.run(() => this.disableAll(), { priority: true, label: "release" }).catch((exc: unknown) => [String(exc)]);
    if (!failed.length) {
      this.energized = false;
      return;
    }
    const others = failed.length < SO101_IDS.length ? "the other servos are off" : "no servo confirmed it";
    throw new ReleaseError([`${this.name} switch-off: ${others}; ${failed.join("; ")}. Support the arm, then switch its power off and on`]);
  }

  /** Torque off with read-back, then the port closes; ReleaseError naming every servo that did not confirm it. */
  async release(): Promise<void> {
    if (!this.open) return;
    try {
      await this.switchOffAll();
    } finally {
      await this.disconnect();
    }
  }

  /** The port closes; the servos keep whatever they hold. */
  async disconnect(): Promise<void> {
    if (!this.open) return;
    this.open = false;
    await this.options.link.close();
  }
}
