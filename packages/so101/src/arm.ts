/** So101Arm: the SO-101 with the same calls as the other arms' `Arm` (maker-arm-sdk's arm.py), built on the console's
SO-101 driver (./follower.ts) and its controller's rules for one arm (servo/controller.ts: the servo loop, the final
command limiter, the return to rest and the fault paths).

Units at this surface are radians and rad/s. The six joints are shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_roll and gripper; each is `(ticks - 2048) * 2π / 4096` from the LeRobot middle pose (the gripper's servo
angle too). gripperOpening() is 0 (closed, the calibration's range_min) .. 1 (open, range_max).

Lifecycle: connect() opens the port and reads the arm (torque untouched); enable() switches the servos on holding
where they are and starts the control loop (servo_hz, 40 Hz); setJointTargets() only stores a target, which the loop
sends through the command limiter; stop() returns to the pose the arm was enabled at and switches torque off only
once it arrived; estop() switches torque off at once from anywhere. */

import type { ArmStatus, Clock, JointStatus } from "@makermods/arm-core";
import { ArmState, ConnectError, Mutex, RealClock, StateError, ValueError } from "@makermods/arm-core";
import { StaleEpochError, openSo101Link, portLabel, type SerialPortLike, type So101Link } from "./bus.js";
import { MemoryCalibrationStore, So101CalibrationSession, type CalibrationStore, type So101CalibrationOptions } from "./calibration.js";
import { msClock, type MsClock } from "./clock-ms.js";
import { So101Follower } from "./follower.js";
import { wiggleGripper, type WiggleResult } from "./identify.js";
import {
  FEEDBACK_MAX_AGE_S,
  REST_ARRIVAL_TOLERANCE_DEG,
  REST_GRID_DEG,
  armLimits,
  arrivedAtRest,
  clipToLimits,
  easedRestStep,
  limitCommand,
  policyCaps,
  returnTimeout,
  validateState,
} from "./limits.js";
import { armConfigOf, so101Profile, type So101ArmConfig, type So101Profile } from "./profile.js";
import { errorText, isFault, type ServoState } from "./protocol.js";
import {
  CENTER_TICK,
  MissedReadError,
  ReleaseError,
  SERVO_SLOTS,
  SO101_IDS,
  SO101_MOTORS,
  StartCancelledError,
  TICKS_PER_TURN,
  calibrationDifferences,
  disableAllServos,
  faultsOf,
  parseSo101Calibration,
  readHeldCalibration,
  readStates,
  ticksToVector,
  type So101Calibration,
} from "./so101.js";

const RAD_PER_TICK = (2 * Math.PI) / TICKS_PER_TURN;
const RAD2DEG = 180 / Math.PI;
const DEG2RAD = Math.PI / 180;
const N = SO101_MOTORS.length;

const message = (exc: unknown) => (exc instanceof Error ? exc.message : String(exc));
const pyName = (exc: unknown) => (exc instanceof Error ? exc.name : "Error");

/** One joint in So101Arm.status(): the core JointStatus (torque always null) plus the servo's own readings. */
export interface So101JointStatus extends JointStatus {
  /** Supply voltage at the servo, V. */
  voltage: number | null;
  /** Present_Load, the servo's raw signed load (not N·m). */
  load: number | null;
  /** Every bit of the servo's last error byte in words (the input-voltage warning included), or null. */
  errorBits: string | null;
}

/** So101Arm.status(): the core ArmStatus plus the calibration's state. */
export interface So101Status extends ArmStatus {
  joints: Record<string, So101JointStatus>;
  calibration: { name: string; loaded: boolean; problem: string | null };
}

/** How an So101Arm runs. */
export interface So101ArmOptions {
  /** Time source (default RealClock). Tests pass VirtualClock. */
  clock?: Clock;
  /** Limits, speeds and the loop rate over the side's shipped defaults (so101Profile validates them). */
  profile?: Partial<So101Profile>;
  /** Where calibrations are kept (default: in memory). */
  calibrationStore?: CalibrationStore;
  /** This arm's calibration's name in the store (default `so101_<side>`). */
  calibrationName?: string;
  /** A calibration to use instead of the store's (it is saved to the store under calibrationName). */
  calibration?: So101Calibration;
  /** The arm's name in messages (default its side). */
  name?: string;
}

/** moveTo's options. */
export interface MoveToOptions {
  /** Joint speed cap, rad/s (default: the profile's max_policy_speed_deg_s). */
  speed?: number;
  /** Seconds to wait for arrival (default: generous for the distance). */
  timeout?: number;
  /** Wait for arrival (default true). */
  wait?: boolean;
  /** Aborting stops the motion where it is (holding the measured pose) and resolves false. */
  signal?: AbortSignal;
}

/** One servo's newest state and when it arrived (clock seconds). */
interface Feedback {
  state: ServoState;
  at: number;
}

/** A return to rest under way. */
interface Return {
  rest: number[];
  goal: number[];
  start: number[];
  ramp: number[];
  startedAt: number;
  deadline: number;
  /** stop(): switch torque off on arrival. */
  release: boolean;
  resolve: (ok: boolean) => void;
  done: Promise<boolean>;
}

/** The SO-101 arm on one serial port. */
export class So101Arm {
  readonly port: SerialPortLike;
  readonly clock: Clock;
  readonly profile: So101Profile;
  readonly name: string;
  readonly calibrationStore: CalibrationStore;
  readonly calibrationName: string;
  /** Why the arm faulted, kept until clearFaults(). */
  faultReason: string | null = null;
  faultReasons: string[] = [];

  readonly #ms: MsClock;
  readonly #arm: So101ArmConfig;
  readonly #lifecycle = new Mutex();
  #state = ArmState.DISCONNECTED;
  #link: So101Link | null = null;
  #calibration: So101Calibration | null = null;
  #calibrationProblem: string | null = null;
  #givenCalibration: So101Calibration | null;
  #session: So101CalibrationSession | null = null;
  #feedback: Array<Feedback | null> = new Array<Feedback | null>(N).fill(null);
  #follower: So101Follower | null = null;
  #enableAbort: AbortController | null = null;
  #wiggleHalt: AbortController | null = null;
  #generation = 0;
  #running = false;
  #loop: Promise<void> | null = null;
  /** Driver slots (degrees, pad, gripper 0..100). */
  #measured: number[] | null = null;
  #sampledAt = -Infinity;
  #last: number[] | null = null;
  #target: number[] | null = null;
  #speedDegS: number | null = null;
  #restPose: number[] | null = null;
  #restStart: number[] | null = null;
  #return: Return | null = null;

  constructor(port: SerialPortLike, options: So101ArmOptions = {}) {
    this.port = port;
    this.clock = options.clock ?? new RealClock();
    this.#ms = msClock(this.clock);
    this.profile = so101Profile(options.profile);
    this.name = options.name ?? this.profile.side;
    this.#arm = armConfigOf(this.profile, this.name);
    this.calibrationStore = options.calibrationStore ?? new MemoryCalibrationStore();
    this.calibrationName = options.calibrationName ?? `so101_${this.profile.side}`;
    this.#givenCalibration = options.calibration ? parseSo101Calibration(options.calibration, this.calibrationName) : null;
  }

  get state(): ArmState {
    return this.#state;
  }

  get jointNames(): string[] {
    return [...SO101_MOTORS];
  }

  /** The calibration in use (null before connect, or when there is none). */
  get calibration(): So101Calibration | null {
    return this.#calibration ? structuredClone(this.#calibration) : null;
  }

  /** Why enable() would refuse for calibration reasons (none loaded, the servos hold another), or null. */
  get calibrationProblem(): string | null {
    return this.#calibrationProblem;
  }

  // ---------- connection ----------

  /** Opens the port and reads every servo without changing its torque state. A servo that does not answer, or reports
  a fault, refuses the connection (the port is closed again; a Feetech fault is cleared by switching the arm's power
  off and on). Loads the calibration and compares it with what the servos hold: a missing or different one does not
  refuse the connection, but enable() refuses until it is fixed (calibrate(), or the right calibrationName). */
  async connect(): Promise<void> {
    await this.#lifecycle.run(async () => {
      if (this.#state !== ArmState.DISCONNECTED) throw new StateError(`connect() requires DISCONNECTED state, currently ${this.#state}`);
      this.#feedback.fill(null);
      const link = await openSo101Link(this.port, this.clock);
      try {
        let states: ServoState[];
        try {
          states = await link.arbiter.run(() => readStates(link.bus, (batch) => this.#store(batch)), { label: "connect read" });
        } catch (exc) {
          throw new ConnectError(`${this.name}: ${message(exc)} -- check the arm's power supply, its cables and that no other program holds ${portLabel(this.port)}`);
        }
        const faults = faultsOf(states);
        if (faults.length) {
          throw new ConnectError(`${this.name}: servos reporting a fault: ${faults.join(", ")} -- support the arm, then switch its power off and on`);
        }
        this.#link = link;
        await this.#loadCalibration();
      } catch (exc) {
        this.#link = null;
        await link.close().catch(() => undefined);
        throw exc;
      }
      this.#state = ArmState.CONNECTED;
    });
  }

  /** Switches torque off immediately (the arm drops unless it is at rest or supported) and closes the port. Call
  stop() first for a safe return. */
  async disconnect(): Promise<void> {
    let error: unknown = null;
    if (this.#state === ArmState.ENABLED || this.#state === ArmState.FAULT) {
      await this.disable().catch((exc: unknown) => (error = exc));
    }
    await this.#lifecycle.run(async () => {
      await this.#session?.halt().catch(() => undefined);
      const link = this.#link;
      this.#link = null;
      this.#follower = null;
      await link?.close().catch(() => undefined);
      this.#state = ArmState.DISCONNECTED;
    });
    if (error) throw error;
  }

  /** Fresh feedback while not enabled; never changes torque state. Which servos answered without a fault (all false
  while enabled: the control loop owns the bus then). */
  async refresh(): Promise<boolean[]> {
    const link = this.#link;
    if (!link || this.#state !== ArmState.CONNECTED) return new Array<boolean>(N).fill(false);
    const read = await link.arbiter.run(() => link.bus.syncReadState(SO101_IDS), { label: "refresh" });
    this.#store(read);
    return SO101_IDS.map((id) => {
      const st = read.get(id);
      return st !== undefined && !isFault(st);
    });
  }

  // ---------- feedback (cached; NaN where there is none) ----------

  #store(batch: ReadonlyMap<number, ServoState>): void {
    const at = this.clock.now();
    SO101_IDS.forEach((id, i) => {
      const state = batch.get(id);
      if (state) this.#feedback[i] = { state, at };
    });
  }

  /** Joint positions, rad. */
  getJointPositions(): number[] {
    return this.#feedback.map((f) => (f ? (f.state.position - CENTER_TICK) * RAD_PER_TICK : NaN));
  }

  /** Joint velocities, rad/s (Present_Velocity). */
  getJointVelocities(): number[] {
    return this.#feedback.map((f) => (f ? f.state.velocity * RAD_PER_TICK : NaN));
  }

  /** Servo temperatures, °C. */
  getTemperatures(): number[] {
    return this.#feedback.map((f) => (f ? f.state.temperatureC : NaN));
  }

  /** The command the control loop last applied (the goal ticks it wrote), rad; NaN before enable. */
  getCommandedPositions(): number[] {
    const last = this.#last;
    return last && this.#calibration ? this.#toRadians(last) : new Array<number>(N).fill(NaN);
  }

  /** JSON-serialisable snapshot in the core Arm.status() shape (state, faultReason, joints by name with motorId,
  position, velocity, torque, temperature, temperatures, enabled, fault, feedbackAge), plus the SO-101's own fields:
  calibration, and per joint voltage, load and errorBits. torque is always null: an STS3215 reports load, not N·m. */
  status(): So101Status {
    const pos = this.getJointPositions();
    const vel = this.getJointVelocities();
    const now = this.clock.now();
    const finite = (x: number) => (Number.isFinite(x) ? x : null);
    const joints: Record<string, So101JointStatus> = {};
    SO101_MOTORS.forEach((m, i) => {
      const f = this.#feedback[i];
      const st = f?.state;
      joints[m] = {
        motorId: SO101_IDS[i] as number,
        position: finite(pos[i] as number),
        velocity: finite(vel[i] as number),
        torque: null,
        temperature: st ? st.temperatureC : null,
        temperatures: st ? { servo: st.temperatureC } : {},
        enabled: st ? st.torque === 1 : null,
        fault: st && isFault(st) ? errorText(st.error || 0x20) : null,
        feedbackAge: f ? now - f.at : null,
        voltage: st ? st.voltage / 10 : null,
        load: st ? st.load : null,
        // Every bit of the servo's last error byte, the input-voltage warning (not a fault) included.
        errorBits: st?.error ? errorText(st.error) : null,
      };
    });
    return {
      state: this.#state,
      faultReason: this.faultReason,
      joints,
      calibration: { name: this.calibrationName, loaded: this.#calibration !== null, problem: this.#calibrationProblem },
    };
  }

  // ---------- wiggle ----------

  /** Wiggles the gripper on this arm's open port so the operator sees which arm it is (wiggleGripper: the gripper alone
  energized, jogged inside its EEPROM limits, returned, then switched off with read-back). Only while CONNECTED with
  torque off: refused (StateError) while enabled or faulted (the control loop or a holding arm owns the servos), while
  calibrating, and with the gripper reporting a fault or holding torque. `signal` ends the jog early; estop() switches
  the gripper off at once. */
  async wiggle(options: { signal?: AbortSignal } = {}): Promise<WiggleResult> {
    return this.#lifecycle.run(async () => {
      if (this.#state !== ArmState.CONNECTED || !this.#link) throw new StateError(`wiggle() requires CONNECTED state (torque off), currently ${this.#state}`);
      if (this.#session?.active) throw new StateError(`${this.name}: a calibration is running`);
      const link = this.#link;
      const halt = new AbortController();
      this.#wiggleHalt = halt;
      try {
        return await link.arbiter.run(async () => {
          const st = (await link.bus.syncReadState([6])).get(6);
          if (!st) throw new ConnectError(`${this.name}: the gripper (servo 6) did not answer; nothing was switched on`);
          this.#store(new Map([[6, st]]));
          if (isFault(st)) throw new ConnectError(`${this.name}: the gripper reports a fault (${errorText(st.error || 0x20)}); nothing was switched on`);
          if (st.torque) throw new ConnectError(`${this.name}: the gripper already has torque on; nothing was sent to it`);
          return wiggleGripper(link, portLabel(this.port), options.signal, halt.signal);
        }, { label: "wiggle" });
      } finally {
        if (this.#wiggleHalt === halt) this.#wiggleHalt = null;
      }
    });
  }

  // ---------- gripper ----------

  /** Measured gripper opening: 0 closed, 1 fully open (not clipped; NaN without feedback or calibration). */
  gripperOpening(): number {
    const f = this.#feedback[5];
    const c = this.#calibration?.gripper;
    if (!f || !c) return NaN;
    return (f.state.position - c.range_min) / (c.range_max - c.range_min);
  }

  /** The gripper joint position (rad) for `opening` (0 closed, 1 fully open; clipped to [0, 1]). */
  gripperTarget(opening: number): number {
    if (!Number.isFinite(opening)) throw new ValueError(`opening must be finite, got ${opening}`);
    const c = this.#calibration?.gripper;
    if (!c) throw new StateError(`${this.name}: the gripper's range comes from the calibration, and none is loaded`);
    const ticks = c.range_min + Math.min(1, Math.max(0, opening)) * (c.range_max - c.range_min);
    return (ticks - CENTER_TICK) * RAD_PER_TICK;
  }

  // ---------- units ----------

  /** Radians (six joints) to the driver's seven slots. */
  #toSlots(q: readonly number[]): number[] {
    const c = (this.#calibration as So101Calibration).gripper;
    const slots = new Array<number>(7).fill(0);
    for (let i = 0; i < 5; i++) slots[i] = (q[i] as number) * RAD2DEG;
    const ticks = (q[5] as number) / RAD_PER_TICK + CENTER_TICK;
    slots[6] = ((ticks - c.range_min) / (c.range_max - c.range_min)) * 100;
    return slots;
  }

  /** The driver's seven slots to radians (six joints). */
  #toRadians(slots: readonly number[]): number[] {
    const c = (this.#calibration as So101Calibration).gripper;
    const out = SERVO_SLOTS.slice(0, 5).map((s) => (slots[s] as number) * DEG2RAD);
    const ticks = c.range_min + ((slots[6] as number) / 100) * (c.range_max - c.range_min);
    out.push((ticks - CENTER_TICK) * RAD_PER_TICK);
    return out;
  }

  // ---------- calibration ----------

  async #loadCalibration(): Promise<void> {
    const link = this.#link as So101Link;
    if (this.#givenCalibration) {
      await this.calibrationStore.save(this.calibrationName, this.#givenCalibration);
      this.#givenCalibration = null;
    }
    this.#calibration = await this.calibrationStore.load(this.calibrationName);
    this.#calibrationProblem = null;
    if (!this.#calibration) {
      this.#calibrationProblem = `${this.name} has no calibration '${this.calibrationName}': calibrate it first`;
      return;
    }
    try {
      const held = await link.arbiter.run(() => readHeldCalibration(link.bus), { label: "held calibration" });
      const differ = calibrationDifferences(this.#calibration, held);
      if (differ.length) {
        this.#calibrationProblem = `${this.name} holds another calibration than '${this.calibrationName}': ${differ.join(", ")}. Is that calibration this arm's (a swapped cable puts the other arm here)? Calibrate it again or choose its calibration`;
      }
    } catch (exc) {
      this.#calibrationProblem = `${this.name}: the servos' calibration could not be read (${message(exc)}); connect again`;
    }
  }

  /** Calibrates this arm (torque off, CONNECTED): by hand or automatically (So101CalibrationSession). enable() is
  refused while it runs; once it completes the new calibration is loaded. */
  calibrate(options: Omit<So101CalibrationOptions, "name" | "store" | "side"> & Partial<Pick<So101CalibrationOptions, "side">>): So101CalibrationSession {
    if (this.#state !== ArmState.CONNECTED || !this.#link) throw new StateError(`calibrate() requires CONNECTED state, currently ${this.#state}`);
    if (this.#session?.active) throw new StateError(`${this.name}: a calibration is already running`);
    const session = new So101CalibrationSession(
      this.#link,
      { side: this.profile.side, ...options, name: this.calibrationName, store: this.calibrationStore },
      false,
      portLabel(this.port),
    );
    this.#session = session;
    void session.done.then(async () => {
      if (this.#link) await this.#lifecycle.run(() => this.#loadCalibration()).catch(() => undefined);
    });
    return session;
  }

  // ---------- enabling ----------

  /** Torque on, holding the measured pose, and the control loop started. Refused (nothing switched on) without a
  calibration matching what the servos hold, with an incomplete calibration, while calibrating, with a servo that
  reports a fault or already has torque on, or with a joint further past its limit than the profile's tolerance.
  `signal` (like estop()) cancels an enable in progress: what was switched on is switched off again and StateError is
  thrown. */
  async enable(options: { signal?: AbortSignal } = {}): Promise<void> {
    await this.#lifecycle.run(async () => {
      if (this.#state !== ArmState.CONNECTED) throw new StateError(`enable() requires CONNECTED state, currently ${this.#state}`);
      if (this.#loop) throw new StateError("the previous control loop is still running; refusing to enable");
      if (this.#session?.active) throw new StateError(`${this.name}: a calibration is running`);
      const link = this.#link as So101Link;
      const generation = this.#generation;
      const abort = new AbortController();
      this.#enableAbort = abort;
      const cancelled = () => {
        if (this.#generation !== generation) throw new StateError("e-stop during enable");
        if (options.signal?.aborted) throw new StateError("enable cancelled");
      };
      options.signal?.addEventListener("abort", () => abort.abort(), { once: true });
      try {
        cancelled();
        // Fresh calibration, faults, torque and pose from every servo; cached values never authorize torque.
        await this.#loadCalibration();
        if (this.#calibrationProblem || !this.#calibration) throw new StateError(this.#calibrationProblem ?? `${this.name}: no calibration`);
        if (await this.calibrationStore.isIncomplete(this.calibrationName)) {
          throw new StateError(`${this.name}: the calibration '${this.calibrationName}' was left incomplete; calibrate the arm again before enabling`);
        }
        const calibration = this.#calibration;
        let states: ServoState[];
        try {
          states = await link.arbiter.run(() => readStates(link.bus, (b) => this.#store(b)), { label: "enable check" });
        } catch (exc) {
          throw new ConnectError(`${this.name}: ${message(exc)}`);
        }
        const faults = faultsOf(states);
        if (faults.length) throw new ConnectError(`${this.name}: servos reporting a fault: ${faults.join(", ")}; nothing was switched on`);
        const holding = states.flatMap((st, i) => (st.torque ? [`${SO101_IDS[i]} (${SO101_MOTORS[i]})`] : []));
        if (holding.length) {
          throw new ConnectError(`${this.name}: servos ${holding.join(", ")} already have torque on; nothing was sent to them. Another program may be holding the arm: close it, or support the arm and switch its power off and on`);
        }
        try {
          validateState(ticksToVector(states.map((s) => s.position), calibration), this.#arm, this.profile.limit_tolerance_deg);
        } catch (exc) {
          throw new ConnectError(message(exc));
        }
        cancelled();
        const follower = new So101Follower({ name: this.name, link, calibration, armLimits: armLimits(this.#arm), onBatch: (b) => this.#store(b) });
        this.#follower = follower;
        let initial: number[];
        try {
          await follower.connect(abort.signal);
          initial = await this.#readFresh(follower);
          validateState(initial, this.#arm, this.profile.limit_tolerance_deg);
          cancelled();
        } catch (exc) {
          // Still where it was limp a moment ago, so switching torque off drops nothing.
          const problems = await follower.switchOffAll().then(
            () => [] as string[],
            (e: unknown) => (e instanceof ReleaseError ? e.problems : [message(e)]),
          );
          if (problems.length) {
            this.#setFault(`Enable failed: ${pyName(exc)}: ${message(exc)}; torque off failed: ${problems.join("; ")}`);
            this.#state = ArmState.FAULT;
          } else {
            this.#follower = null;
          }
          if (exc instanceof StartCancelledError) cancelled();
          throw exc;
        }
        this.#measured = initial;
        this.#sampledAt = this.clock.now();
        this.#last = [...initial];
        this.#target = null;
        this.#speedDegS = null;
        // The pose captured at enable, clamped into the soft limits the driver enforces, so a return to rest can arrive.
        this.#restPose = clipToLimits(initial, this.#arm);
        this.#restStart = [...initial];
        this.#state = ArmState.ENABLED;
        this.#running = true;
        const session = this.#generation;
        this.#loop = this.#motionLoop(session)
          .catch((exc: unknown) => this.#holdAndFault(`Control loop failed: ${pyName(exc)}: ${message(exc)}. The arm is holding; support it, then power-cycle`))
          .finally(() => {
            this.#loop = null;
          });
      } finally {
        if (this.#enableAbort === abort) this.#enableAbort = null;
      }
    });
  }

  /** follower.read(), retried past missed replies until the follower's own MAX_MISSED_MS gives up. */
  async #readFresh(follower: So101Follower): Promise<number[]> {
    for (;;) {
      try {
        return await follower.read();
      } catch (exc) {
        if (!(exc instanceof MissedReadError)) throw exc;
        await this.#ms.sleep(1000 / this.profile.servo_hz);
      }
    }
  }

  // ---------- the control loop ----------

  /** The servo loop while energized (the console's motionLoop for one arm): read, check, compute, send, at servo_hz,
  non-overlapping ticks (a late tick starts now, never a burst to catch up). */
  async #motionLoop(session: number): Promise<void> {
    const follower = this.#follower as So101Follower;
    const live = () => this.#running && session === this.#generation;
    let previous = this.clock.now();
    let deadline = previous;
    while (live()) {
      const period = 1 / this.profile.servo_hz;
      const tick = this.clock.now();
      let arrived = false;
      try {
        const measured = await follower.read();
        if (!live()) break;
        validateState(measured, this.#arm, this.profile.limit_tolerance_deg);
        this.#measured = measured;
        this.#sampledAt = tick;
        const ret = this.#return;
        if (ret && tick > ret.deadline) {
          this.#holdAndFault("Return to rest timed out before arrival. The arm is holding; support it, then power-cycle");
          break;
        }
        let command: number[];
        if (ret) {
          const step = this.#restCommand(ret, measured, tick - previous, tick);
          arrived = step === null;
          command = step ?? (this.#last as number[]);
        } else if (this.#target) {
          const joint = Math.min(this.profile.max_policy_speed_deg_s, this.#speedDegS ?? Infinity);
          command = limitCommand(this.#target, this.#last ?? undefined, this.#arm, policyCaps(joint, this.profile.max_policy_gripper_speed_deg_s), tick - previous).command;
        } else {
          command = this.#last as number[];
        }
        // Stop and write are serialized by the arbiter's epoch: once a stop bumped it, an old command is dropped.
        const epoch = follower.arbiter.epoch;
        try {
          const applied = await follower.sendAction(command, epoch);
          if (live() && epoch === follower.arbiter.epoch) {
            this.#last = applied;
            if (ret && this.#return === ret) ret.ramp = ret.ramp.map((v, i) => (Math.abs(v - (applied[i] as number)) <= REST_GRID_DEG ? v : (applied[i] as number)));
          }
        } catch (exc) {
          if (!(exc instanceof StaleEpochError)) throw exc;
        }
      } catch (exc) {
        if (!live()) break;
        // A read without a complete answer: this tick holds (nothing commanded on stale feedback); the follower fails
        // the arm after MAX_MISSED_MS of them.
        if (!(exc instanceof MissedReadError)) {
          await this.#recover(exc, follower);
          if (!live()) break;
        }
      }
      if (arrived && this.#return) {
        const ret = this.#return;
        if (ret.release) {
          await this.#releaseAtRest(ret, follower);
          break;
        }
        this.#return = null;
        this.#target = null;
        ret.resolve(true);
      }
      previous = tick;
      const now = this.clock.now();
      deadline = Math.max(deadline + period, now);
      await this.clock.sleep(Math.max(0, deadline - this.clock.now()));
    }
  }

  /** A failed tick (the console's recover for one arm). An arm that still answers returns to rest and switches off,
  its fault kept; one that no longer answers keeps holding, since switching it off would drop it. */
  async #recover(exc: unknown, follower: So101Follower): Promise<void> {
    const text = `Control fault: ${pyName(exc)}: ${message(exc)}`;
    if (this.#return || this.#restPose === null) {
      this.#holdAndFault(`${text}. The arm is holding; support it, then power-cycle`);
      return;
    }
    let fresh: number[];
    try {
      fresh = await follower.read();
    } catch {
      this.#holdAndFault(`${text}. The arm stopped answering and keeps holding; support it, then power-cycle`);
      return;
    }
    this.#measured = fresh;
    this.#sampledAt = this.clock.now();
    this.#setFault(`${text}. Returning to rest, then torque off`);
    this.#state = ArmState.FAULT;
    try {
      this.#beginReturn(this.#restPose, this.#restStart ?? this.#restPose, true);
    } catch (e) {
      this.#holdAndFault(`Can't return to rest (${message(e)}). The arm is holding; support it, then power-cycle`);
    }
  }

  /** The arm can't be brought back: it keeps holding (torque on, nothing drops) and nothing more is sent. */
  #holdAndFault(text: string): void {
    this.#setFault(text);
    this.#state = ArmState.FAULT;
    this.#halt();
  }

  #setFault(text: string): void {
    this.faultReason = text;
    this.faultReasons.push(text);
  }

  /** Stops the loop sending: later commands are dropped by the epoch; a pending return resolves false. */
  #halt(): void {
    this.#running = false;
    this.#generation++;
    this.#link?.arbiter.bumpEpoch();
    const ret = this.#return;
    this.#return = null;
    ret?.resolve(false);
  }

  /** control._begin_return: from where the arm is (a joint tracking normally keeps its command, so a raised arm does
  not lose its holding torque for a moment), toward the goal the driver applies for `rest`, eased, timed out. */
  #beginReturn(rest: number[], start: number[], release: boolean): Promise<boolean> {
    if (this.clock.now() - this.#sampledAt > FEEDBACK_MAX_AGE_S) throw new RangeError("arm feedback is stale");
    const measured = this.#measured as number[];
    const follower = this.#follower as So101Follower;
    validateState(measured, this.#arm, this.profile.limit_tolerance_deg);
    validateState(rest, this.#arm, this.profile.limit_tolerance_deg);
    const last = this.#last;
    const tracking = this.#arm.max_tracking_error_deg;
    this.#last = measured.map((m, i) => (i < 6 && last && Math.abs((last[i] as number) - m) <= tracking / 2 ? (last[i] as number) : m));
    this.#target = null;
    const startedAt = this.clock.now();
    let resolve!: (ok: boolean) => void;
    const done = new Promise<boolean>((r) => (resolve = r));
    this.#return?.resolve(false);
    this.#return = {
      rest,
      goal: follower.goalFor(rest),
      start,
      ramp: [...this.#last],
      startedAt,
      deadline: startedAt + returnTimeout(this.#arm, rest, measured),
      release,
      resolve,
      done,
    };
    return done;
  }

  /** control._rest_command: null once arrived (the gripper left out of the tracking and arrival checks: it may hold an
  object), else the next eased step. */
  #restCommand(ret: Return, measured: number[], dt: number, now: number): number[] | null {
    const last = this.#last as number[];
    if (arrivedAtRest(last, ret.goal, measured, ret.start)) return null;
    // Each step continues the ramp's own command where the applied goal is that command on the driver's grid.
    const from = last.map((v, i) => (Math.abs((ret.ramp[i] as number) - v) <= REST_GRID_DEG ? (ret.ramp[i] as number) : v));
    const command = easedRestStep(this.#arm, ret.goal, from, measured, now - ret.startedAt, dt);
    ret.ramp = [...command];
    return command;
  }

  /** At rest: torque off with read-back; the loop ends. A servo that did not confirm it is a fault (torque may still
  be on). */
  async #releaseAtRest(ret: Return, follower: So101Follower): Promise<void> {
    this.#return = null;
    this.#running = false;
    try {
      await follower.switchOffAll();
      this.#follower = null;
      if (this.#state === ArmState.ENABLED) this.#state = ArmState.CONNECTED;
      ret.resolve(true);
    } catch (exc) {
      const problems = exc instanceof ReleaseError ? exc.problems : [message(exc)];
      this.#setFault(`Switching torque off at rest failed: ${problems.join("; ")}. Torque may still be on; support the arm, then power-cycle`);
      this.#state = ArmState.FAULT;
      ret.resolve(false);
    }
  }

  // ---------- targets ----------

  /** Stores joint targets (rad, six values) for the control loop, which sends them through the command limiter (the
  soft limits, the gripper's open margin, the rate caps, a joint past its limit brought in slowly). False, and
  ignored, unless ENABLED and not returning to rest, or with a wrong count or a non-finite value. */
  setJointTargets(targets: readonly number[]): boolean {
    return this.#setTargets(targets, null);
  }

  #setTargets(targets: readonly number[], speedDegS: number | null): boolean {
    if (targets.length !== N || !targets.every(Number.isFinite)) return false;
    if (this.#state !== ArmState.ENABLED || this.#return || !this.#running) return false;
    this.#target = this.#toSlots(targets);
    this.#speedDegS = speedDegS;
    return true;
  }

  /** Drives to joint pose `q` (rad) at up to `speed` rad/s. With `wait` (default) it resolves true once every joint
  but the gripper is within REST_ARRIVAL_TOLERANCE_DEG of the goal the driver applies for `q` (clipped to the
  limits), false on timeout, a fault, the loop stopping, newer targets taking over, or `signal` (which holds the
  measured pose). */
  async moveTo(q: readonly number[], options: MoveToOptions = {}): Promise<boolean> {
    const { speed, wait = true, signal } = options;
    if (speed !== undefined && !(Number.isFinite(speed) && speed > 0)) throw new ValueError("speed must be positive");
    if (signal?.aborted) return false;
    const speedDegS = speed === undefined ? null : speed * RAD2DEG;
    if (!this.#setTargets(q, speedDegS)) return false;
    if (!wait) return true;
    const target = this.#target as number[];
    const follower = this.#follower as So101Follower;
    const goal = follower.goalFor(clipToLimits(target, this.#arm));
    const rate = Math.min(this.profile.max_policy_speed_deg_s, speedDegS ?? Infinity);
    let timeout = options.timeout;
    if (timeout === undefined) {
      const start = this.#measured ?? goal;
      const distance = Math.max(0, ...goal.slice(0, 5).map((g, i) => Math.abs(g - (start[i] as number))));
      timeout = (1.5 * distance) / rate + 2.0;
    }
    const deadline = this.clock.now() + timeout;
    const tolerance = REST_ARRIVAL_TOLERANCE_DEG;
    while (this.clock.now() < deadline) {
      if (this.#state !== ArmState.ENABLED || !this.#running || this.#follower !== follower) return false;
      if (signal?.aborted) {
        this.holdCurrentPosition();
        return false;
      }
      if (this.#target !== target) return false;
      const m = this.#measured;
      if (m && goal.slice(0, 5).every((g, i) => Math.abs((m[i] as number) - g) <= tolerance)) return true;
      await this.clock.sleep(0.005);
    }
    return false;
  }

  /** Freezes the arm at its latest measured pose without releasing torque (ends a returnToRest() in progress). False
  unless the control loop is running. */
  holdCurrentPosition(): boolean {
    if ((this.#state !== ArmState.ENABLED && this.#state !== ArmState.FAULT) || !this.#running || !this.#measured) return false;
    const ret = this.#return;
    if (ret?.release && this.#state === ArmState.FAULT) return false; // a fault's return to rest always finishes
    this.#return = null;
    ret?.resolve(false);
    this.#target = [...this.#measured];
    this.#last = [...this.#measured];
    this.#speedDegS = null;
    return true;
  }

  /** Drives back to `rest` (rad; default the pose at enable, clipped to the limits) at the rest speed, eased, and
  resolves true on arrival (every joint but the gripper within 3° of it); torque stays on. New targets are refused
  while it runs. Never throws: false when it can't start or doesn't arrive. */
  async returnToRest(rest?: readonly number[]): Promise<boolean> {
    try {
      if (this.#state !== ArmState.ENABLED || !this.#running) return false;
      if (this.#return) return await this.#return.done;
      let pose: number[];
      if (rest === undefined) pose = this.#restPose as number[];
      else {
        if (rest.length !== N || !rest.every(Number.isFinite)) return false;
        pose = clipToLimits(this.#toSlots(rest), this.#arm);
      }
      return await this.#beginReturn(pose, rest === undefined ? (this.#restStart ?? pose) : pose, false);
    } catch {
      return false;
    }
  }

  // ---------- stopping ----------

  /** Returns to the pose the arm was enabled at, then switches torque off, only once it arrived. An arm without brakes
  must never be released away from rest: if the return cannot be completed, the arm keeps holding and false is
  returned (the caller decides when to disable()). True when torque is off at rest (or was never on). */
  async stop(): Promise<boolean> {
    if (this.#state !== ArmState.ENABLED && this.#state !== ArmState.FAULT) return true;
    // A follower is kept from enable until torque off was confirmed: without one, torque is off.
    if (this.#follower === null) return true;
    const ret = this.#return;
    if (ret) {
      ret.release = true;
      return ret.done;
    }
    // After a fault the arm only holds: nothing more is sent to it.
    if (this.#state === ArmState.FAULT || !this.#running || this.#restPose === null) return false;
    try {
      return await this.#beginReturn(this.#restPose, this.#restStart ?? this.#restPose, true);
    } catch (exc) {
      this.#holdAndFault(`Can't return to rest (${message(exc)}). The arm is holding; support it, then power-cycle`);
      return false;
    }
  }

  /** Switches torque off now, with read-back (the arm drops unless it is at rest or supported). Prefer stop(). Throws
  ReleaseError, and the arm is FAULT, when a servo did not confirm it. */
  async disable(): Promise<void> {
    await this.#lifecycle.run(() => this.#disableNow());
  }

  async #disableNow(): Promise<void> {
    if (this.#state === ArmState.DISCONNECTED || !this.#link) return;
    this.#halt();
    await this.#loop?.catch(() => undefined);
    await this.#switchOff();
    this.#state = ArmState.CONNECTED;
  }

  /** Every servo off with read-back, ahead of anything queued. */
  async #switchOff(): Promise<void> {
    const link = this.#link as So101Link;
    const follower = this.#follower;
    try {
      if (follower) await follower.switchOffAll();
      else {
        const failed = await link.arbiter.run(() => disableAllServos(link.bus), { priority: true, label: "release" });
        if (failed.length) throw new ReleaseError([`${this.name} switch-off: ${failed.join("; ")}. Support the arm, then switch its power off and on`]);
      }
      this.#follower = null;
    } catch (exc) {
      const problems = exc instanceof ReleaseError ? exc.problems : [message(exc)];
      this.#setFault(`Switching torque off failed: ${problems.join("; ")}. Torque may still be on; support the arm, then power-cycle`);
      this.#state = ArmState.FAULT;
      throw exc instanceof ReleaseError ? exc : new ReleaseError(problems);
    }
  }

  /** Callable from any state and any async context: an enable in progress is cancelled, the loop stops sending, and
  every servo is switched off now with read-back (the arm drops). */
  async estop(): Promise<void> {
    this.#enableAbort?.abort();
    this.#wiggleHalt?.abort();
    this.#halt();
    if (this.#state === ArmState.DISCONNECTED || !this.#link) return;
    // A calibration owns the bus while it runs: it switches off at once, then the switch-off is confirmed here.
    await this.#session?.halt();
    await this.#switchOff();
    await this.#loop?.catch(() => undefined);
    await this.#lifecycle.run(async () => {
      if (this.#state !== ArmState.DISCONNECTED) this.#state = ArmState.CONNECTED;
    });
  }

  /** Explicit operator action: torque off (with read-back), then the fault evidence cleared; CONNECTED. */
  async clearFaults(): Promise<void> {
    await this.#lifecycle.run(async () => {
      if (this.#state === ArmState.DISCONNECTED) throw new StateError("clearFaults() requires a connected arm");
      if (this.#state === ArmState.ENABLED || this.#state === ArmState.FAULT) await this.#disableNow();
      this.faultReason = null;
      this.faultReasons = [];
      this.#state = ArmState.CONNECTED;
    });
  }
}

/** `new So101Arm(port, options)`, as the other arm packages' createArm. */
export function createArm(port: SerialPortLike, options: So101ArmOptions = {}): So101Arm {
  return new So101Arm(port, options);
}
