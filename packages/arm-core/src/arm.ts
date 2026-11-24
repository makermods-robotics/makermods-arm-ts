/**
 * Arm layer: state machine, control loop and all safety logic, in joint coordinates (SI units).
 *
 * Non-clearing fault checks, startup sync from the measured pose, full-turn correction, latched
 * fault evidence, gripper holding-effort regulation and temperature protection, and a verified
 * return to rest before torque is released. Everything motor-specific goes through the Motor seam
 * of the arm's Product.
 */

import { type Clock, Mutex, RealClock, Signal } from "./clock.js";
import { type ArmConfig, checkGains, degrees, JointConfig, radians } from "./config.js";
import { ArmError, ConnectError, FaultLatched, ParamTimeout, StateError, ValueError } from "./errors.js";
import { GripperHold } from "./gripper.js";
import type { Logger } from "./logger.js";
import * as gravity from "./gravity.js";
import * as kinematics from "./kinematics.js";
import type { Motor } from "./motor.js";
import { ArmState } from "./state.js";
import * as trajectoryIk from "./trajectory-ik.js";
import type { CanBackend } from "./transport/base.js";

/** move_ee: how far the planned path may end from the goal (m). */
export const EE_POSITION_TOL = 1e-3;
/** move_ee: how far the planned path may end from the goal (rad). */
export const EE_ANGLE_TOL = radians(0.5);
/** Shortest moveEe path, s. */
export const EE_MIN_SECONDS = 0.25;
/** Longest path moveEe plans, s. */
export const EE_MAX_SECONDS = 120;
/** Frame spacing on a real bus: the least time between two motor command frames of one tick, s. */
export const FRAME_SPACING = 150e-6;

export type { Logger } from "./logger.js";

/** Extra feed-forward added while floating: (joint positions, joint velocities) -> N·m per joint. */
export type ExtraTorque = (positions: number[], velocities: number[]) => readonly number[];

export interface ArmOptions {
  /** Default: a RealClock. */
  clock?: Clock;
  /** Default: console. */
  logger?: Logger;
  /** Least time between two command frames of a tick, s (default FRAME_SPACING; 0 for simulated motors). */
  frameSpacing?: number;
}

export interface EnableOptions {
  /** Start the control loop (default true). */
  startLoop?: boolean;
  /** Aborts an enable in progress, like estop(). */
  signal?: AbortSignal;
  /** rad/s cap on the motion until the first target (default: maxVelocity). */
  speed?: number;
}

export interface MoveOptions {
  /** rad/s per joint (default: maxVelocity). */
  speed?: number;
  /** s (default: generous for the distance). */
  timeout?: number;
  /** Wait for arrival (default true). */
  wait?: boolean;
  /** Stops the motion where it is (holding the measured pose). */
  signal?: AbortSignal;
}

export interface MoveEeOptions extends MoveOptions {
  /** rad, world frame; without it the end orientation is the one a position-only IK reaches with the least joint motion. */
  rpy?: number[];
}

/** One joint of Arm.status(). */
export interface JointStatus {
  motorId: number;
  position: number | null;
  velocity: number | null;
  torque: number | null;
  temperature: number | null;
  temperatures: Record<string, number>;
  enabled: boolean | null;
  fault: string | null;
  feedbackAge: number | null;
}

/** Arm.status(): JSON-serialisable. */
export interface ArmStatus {
  state: string;
  faultReason: string | null;
  joints: Record<string, JointStatus>;
}

const finiteOrNull = (x: number): number | null => (Number.isFinite(x) ? x : null);
/** Python's min(a, b) / max(a, b) (the first argument wins a tie, signed zeros included). */
const pyMin = (a: number, b: number): number => (b < a ? b : a);
const pyMax = (a: number, b: number): number => (b > a ? b : a);
const clip = (lo: number, hi: number, x: number): number => pyMin(hi, pyMax(lo, x));
const errorText = (e: unknown): string => (e instanceof Error ? e.message : String(e));
const signed = (x: number, digits: number): string => (x >= 0 ? "+" : "") + x.toFixed(digits);
const sameList = (a: readonly number[], b: readonly number[]): boolean =>
  a.length === b.length && a.every((v, i) => v === b[i]);

interface LoopHandle {
  promise: Promise<void>;
  alive: boolean;
}

/** A whole arm: lifecycle, control loop and safety logic over the motors of its Product. */
export class Arm {
  /** s, per request. */
  static readonly REPLY_TIMEOUT = 0.05;
  static readonly REPLY_ATTEMPTS = 3;
  /** s, typed parameter replies. */
  static readonly PARAM_TIMEOUT = 0.12;
  /** Damping used by floatMode(). */
  static readonly FLOAT_KD = 0.1;

  REPLY_TIMEOUT = Arm.REPLY_TIMEOUT;
  REPLY_ATTEMPTS = Arm.REPLY_ATTEMPTS;
  PARAM_TIMEOUT = Arm.PARAM_TIMEOUT;

  readonly config: ArmConfig;
  motors: Motor[];
  /** Why the arm entered FAULT (the first reason), or null. */
  faultReason: string | null = null;
  faultReasons: string[] = [];
  /** Motor ids whose torque release the last disable could not confirm. */
  unconfirmedDisable: number[] = [];
  /** The pose stop() returns to (set at enable; null after attach). */
  restPose: number[] | null = null;

  #backend: CanBackend;
  #clock: Clock;
  #log: Logger;
  #frameSpacing: number;
  #enableGate = new Mutex();
  #byId: Map<number, Motor>;
  private profileGains: Array<[number, number]>;
  #state = ArmState.DISCONNECTED;
  #lifecycle = new Mutex();
  /** Bumped by estop() to cancel an enable in progress. */
  private generation = 0;
  /** Bumped on disable: a stale loop may not send. */
  private session = 0;
  private userTargets: number[];
  private internal: number[];
  private synced: boolean[];
  private targetsReceived = false;
  /** moveTo() speed cap, cleared by the next target. */
  private speed: number | null = null;
  private stuckAnchor: Array<[number, number]>;
  /** A return to rest owns the targets. */
  private returning = false;
  /** Disabled after a fault (release_on_fault). */
  private released = new Set<number>();
  /** Frozen at the fault pose. */
  private faulted = new Set<number>();
  private watchdogRestore = new Map<number, unknown>();
  private hold = new Map<number, GripperHold>();
  /** seq, position, kp, kd. */
  private holdOut = new Map<number, [number, number, number, number]>();
  private releaseSent = new Map<number, number>();
  private pollSent = new Map<number, number>();
  /** When the control loop took over status polling. */
  private pollingSince = 0.0;
  private gravity: gravity.GravityFeedforward | null = null;
  /** floatMode(extraTorque). */
  private extraTorque: ExtraTorque | null = null;
  /** floatMode() until setGains() or a fault. */
  private floating = false;
  #chain: [string, kinematics.Chain] | null = null;
  private tickCount = 0;
  private running = false;
  private loop: LoopHandle | null = null;
  #tickSignal = new Signal();
  /** Per-session full-turn correction (joint coordinates). */
  private wrap: number[];

  constructor(config: ArmConfig, backend: CanBackend, options: ArmOptions = {}) {
    config.validate();
    this.config = config;
    this.#backend = backend;
    this.#clock = options.clock ?? new RealClock();
    this.#log = options.logger ?? console;
    this.#frameSpacing = options.frameSpacing ?? FRAME_SPACING;
    const product = config.product;
    this.motors = config.joints.map((j) => product.createMotor(j, backend, config.hostId, this.#clock, this.#log));
    this.#byId = new Map(this.motors.map((m) => [m.motorId, m]));
    this.profileGains = config.joints.map((j) => [j.kp, j.kd]);
    const n = config.nJoints;
    this.userTargets = new Array<number>(n).fill(0.0);
    this.internal = new Array<number>(n).fill(0.0);
    this.synced = new Array<boolean>(n).fill(true);
    this.stuckAnchor = Array.from({ length: n }, () => [0.0, 0.0] as [number, number]);
    this.wrap = new Array<number>(n).fill(0.0);
  }

  get state(): ArmState {
    return this.#state;
  }

  get jointNames(): string[] {
    return this.config.joints.map((j) => j.label);
  }

  get backend(): CanBackend {
    return this.#backend;
  }

  get clock(): Clock {
    return this.#clock;
  }

  // ── RX dispatch (transport-layer callback: fast in, fast out) ──
  #onFrame = (canId: number, data: Uint8Array, extended: boolean): void => {
    const id = this.config.product.route(canId, data, this.config.hostId, extended);
    const m = id === null ? undefined : this.#byId.get(id);
    if (m !== undefined) m.handleFrame(canId, data, extended);
  };

  // ── connection management ──
  /**
   * Find every motor without changing torque state.
   *
   * Each motor first gets a non-clearing fault query. A faulted motor is reported and the fault is
   * kept (pass `clearFaults = true` to clear it explicitly). `clearReleased = true` clears only
   * the state a release_on_fault joint's motor watchdog leaves behind (the gripper let go after
   * commands stopped; see Motor.watchdogReleased). Fault-free motors then return a fresh position.
   * Torque is never enabled or released here.
   */
  connect(clearFaults = false, clearReleased = false): Promise<void> {
    return this.#lifecycle.run(async () => {
      if (this.#state !== ArmState.DISCONNECTED) {
        throw new StateError(`connect() requires DISCONNECTED state, currently ${stateName(this.#state)}`);
      }
      for (const m of this.motors) m.resetCache();
      this.released.clear();
      this.faulted.clear();
      this.#backend.setRecvCallback(this.#onFrame);
      await this.#backend.open();
      try {
        await this.#checkMotors(clearFaults, clearReleased);
        for (const m of this.motors) await m.validate(this.PARAM_TIMEOUT);
      } catch (e) {
        await this.#backend.close();
        throw e;
      }
      // Full-turn correction: a power cycle can reset the motor's multi-turn count, shifting a
      // reading by ±2π. Joint travel is under one turn, so a reading outside the limits that lands
      // inside them after a whole-turn shift is unambiguous. Anything else is a stale zero and
      // enable() refuses it.
      const twoPi = 2 * Math.PI;
      const grace = this.config.fullTurnGrace;
      this.wrap = new Array<number>(this.config.nJoints).fill(0.0);
      this.config.joints.forEach((j, i) => {
        const raw = this.#toJoint(i, this.motors[i]!.feedback!.position);
        if (j.lo - grace <= raw && raw <= j.hi + grace) return;
        for (const k of [-twoPi, twoPi]) {
          if (j.lo - grace <= raw + k && raw + k <= j.hi + grace) {
            this.wrap[i] = k;
            this.#log.info(`${j.label} reading ${signed(raw, 3)} rad is one turn off, correcting ${signed(k, 3)} for this session`);
            break;
          }
        }
      });
      this.#state = ArmState.CONNECTED;
      this.#log.info(`connected: ${this.motors.length} motors online`);
    });
  }

  /** Non-clearing fault query, then a fresh state read, for every motor. */
  async #checkMotors(clearFaults = false, clearReleased = false): Promise<void> {
    const missing: number[] = [];
    const faulted: string[] = [];
    const joints = this.config.joints;
    for (const [i, m] of this.motors.entries()) {
      const j = joints[i]!;
      let status;
      try {
        status = await m.check(this.REPLY_TIMEOUT, this.REPLY_ATTEMPTS);
      } catch (e) {
        if (e instanceof ParamTimeout) {
          missing.push(m.motorId);
          continue;
        }
        throw e;
      }
      if (status.hasFault && clearReleased && j.releaseOnFault && m.watchdogReleased) {
        this.#log.info(`${j.label}: clearing the state its motor watchdog left (${status.text})`);
        await m.clearFault();
        await this.#clock.sleep(0.01);
        status = await m.check(this.REPLY_TIMEOUT, this.REPLY_ATTEMPTS);
      }
      if (status.hasFault && clearFaults) {
        this.#log.warn(`${j.label}: clearing fault 0x${hex8(status.code)} (${status.text}) on request`);
        await m.clearFault();
        await this.#clock.sleep(0.01);
        status = await m.check(this.REPLY_TIMEOUT, this.REPLY_ATTEMPTS);
      }
      if (status.hasFault) faulted.push(`${m.motorId} (${j.label}: ${status.text})`);
    }
    if (!missing.length && !faulted.length) {
      for (const [i, m] of this.motors.entries()) {
        const j = joints[i]!;
        try {
          await m.readState(this.REPLY_TIMEOUT, this.REPLY_ATTEMPTS);
        } catch (e) {
          if (e instanceof ParamTimeout) missing.push(m.motorId);
          else if (!(e instanceof FaultLatched)) throw e;
        }
        if (m.faultLatched) faulted.push(`${m.motorId} (${j.label}: ${m.faultLatched})`);
      }
    }
    if (missing.length || faulted.length) {
      const online = this.motors.map((m) => m.motorId).filter((id) => !missing.includes(id));
      const parts: string[] = [];
      if (missing.length) {
        parts.push(`no reply from motors [${missing.join(", ")}] (online: ${online.length ? `[${online.join(", ")}]` : "none"}) `
          + "-- check power, wiring, termination, 1 Mbps, MIT protocol and that no other program holds the adapter");
      }
      if (faulted.length) {
        parts.push(`motors reporting a fault: ${faulted.join(", ")} -- fault kept for inspection; `
          + "clear it explicitly (connect(true) or clearFaults()) once the cause is fixed");
      }
      throw new ConnectError(parts.join("; "));
    }
  }

  /** Release torque immediately and close the bus. Call stop() first for a safe return. */
  disconnect(): Promise<void> {
    return this.#lifecycle.run(async () => {
      if (this.#state === ArmState.ENABLED || this.#state === ArmState.FAULT) await this.#disable();
      await this.#backend.close();
      this.#state = ArmState.DISCONNECTED;
    });
  }

  /**
   * Stop the control loop and close the bus, leaving torque on: each motor holds its last command,
   * except that a joint with an armed motor watchdog releases once it times out. Returns those
   * joints (see watchdogJoints()).
   */
  detach(): Promise<Record<string, number>> {
    return this.#lifecycle.run(async () => {
      const releasing = this.watchdogJoints();
      await this.#stopLoop();
      await this.#backend.close();
      this.#state = ArmState.DISCONNECTED;
      return releasing;
    });
  }

  /**
   * Joints whose motor watchdog this session armed: label -> seconds after the last command at
   * which that motor releases torque by itself.
   */
  watchdogJoints(): Record<string, number> {
    const joints = this.config.joints;
    const out: Record<string, number> = {};
    for (const i of [...this.watchdogRestore.keys()].sort((a, b) => a - b)) {
      out[joints[i]!.label] = joints[i]!.canTimeoutMs! / 1000;
    }
    return out;
  }

  /** Joints that would drop after detach(): an armed watchdog on a joint that is not release_on_fault. */
  detachUnsafe(): string[] {
    const byLabel = new Map(this.config.joints.map((j) => [j.label, j]));
    return Object.keys(this.watchdogJoints()).filter((n) => !byLabel.get(n)!.releaseOnFault);
  }

  /**
   * Request fresh feedback while not enabled. Never changes torque state or clears a reported
   * fault.
   *
   * Each motor gets a non-clearing fault query first; only a fault-free motor is asked for its
   * position. With `wait = true` (default) the result says which motors replied.
   */
  async refresh(wait = true, timeout = 0.05): Promise<boolean[] | null> {
    if (this.#state === ArmState.ENABLED || this.#state === ArmState.FAULT) {
      return wait ? new Array<boolean>(this.motors.length).fill(false) : null;
    }
    if (timeout <= 0) throw new ValueError("refresh timeout must be positive");
    const fresh: boolean[] = [];
    for (const m of this.motors) {
      try {
        if ((await m.check(timeout, 1)).hasFault) throw new FaultLatched(`motor ${m.motorId}: ${m.faultLatched}`);
        await m.readState(timeout, 1);
        fresh.push(true);
      } catch (e) {
        if (e instanceof ParamTimeout || e instanceof FaultLatched) fresh.push(false);
        else throw e;
      }
    }
    return wait ? fresh : null;
  }

  // ── coordinate conversion (direction/offset only appear in these two functions) ──
  #toMotor(i: number, jointPos: number): number {
    const j = this.config.joints[i]!;
    return (jointPos - this.wrap[i]!) * j.direction + j.offset;
  }

  #toJoint(i: number, motorPos: number): number {
    const j = this.config.joints[i]!;
    return (motorPos - j.offset) * j.direction + this.wrap[i]!;
  }

  // ── feedback getters (joint coordinates; no feedback -> NaN / null) ──
  getJointPositions(): number[] {
    return this.motors.map((m, i) => (m.feedback ? this.#toJoint(i, m.feedback.position) : NaN));
  }

  getJointVelocities(): number[] {
    return this.motors.map((m, i) => (m.feedback ? m.feedback.velocity * this.config.joints[i]!.direction : NaN));
  }

  getJointTorques(): number[] {
    return this.motors.map((m, i) => (m.feedback ? m.feedback.torque * this.config.joints[i]!.direction : NaN));
  }

  getTemperatures(): number[] {
    return this.motors.map((m) => (m.feedback ? m.feedback.temperature : NaN));
  }

  /** Latest fault code per motor (0 = no fault, null = never queried). */
  getFaults(): Array<number | null> {
    return this.motors.map((m) => (m.status ? (m.status.hasFault ? m.status.code : 0) : null));
  }

  /**
   * JSON-serialisable snapshot: arm state plus, per joint, the latest feedback (null where there is
   * none), its age and any motor fault.
   */
  status(): ArmStatus {
    const pos = this.getJointPositions(), vel = this.getJointVelocities(), tau = this.getJointTorques();
    const joints: Record<string, JointStatus> = {};
    this.motors.forEach((m, i) => {
      const j = this.config.joints[i]!;
      const fb = m.feedback, st = m.status;
      joints[j.label] = {
        motorId: m.motorId,
        position: finiteOrNull(pos[i]!),
        velocity: finiteOrNull(vel[i]!),
        torque: finiteOrNull(tau[i]!),
        temperature: fb ? finiteOrNull(fb.temperature) : null,
        temperatures: fb ? { ...fb.temperatures } : {},
        enabled: fb ? fb.enabled : null,
        fault: m.faultLatched || (st !== null && st.hasFault ? st.text : null),
        feedbackAge: finiteOrNull(m.feedbackAge),
      };
    });
    return { state: this.#state, faultReason: this.faultReason, joints };
  }

  // ── gripper (the joint named "gripper") ──
  /** [index, closed position, fully open position] of the gripper joint, rad. */
  #gripperRange(): [number, number, number] {
    const joints = this.config.joints;
    for (let i = 0; i < joints.length; i++) {
      const j = joints[i]!;
      if (j.name === "gripper") {
        const [closed, far] = j.closingDirection > 0 ? [j.hi, j.lo] : [j.lo, j.hi];
        return [i, closed, j.openPosition === null ? far : j.openPosition];
      }
    }
    throw new ArmError("this profile has no joint named 'gripper'");
  }

  /** Measured gripper opening: 0 closed, 1 fully open (not clipped; NaN without feedback). */
  gripperOpening(): number {
    const [i, closed, full] = this.#gripperRange();
    return (this.getJointPositions()[i]! - closed) / (full - closed);
  }

  /** Gripper joint position (rad) for `opening` (0 closed, 1 fully open; clipped to [0, 1]). */
  gripperTarget(opening: number): number {
    if (!Number.isFinite(opening)) throw new ValueError(`opening must be finite, got ${opening}`);
    const [, closed, full] = this.#gripperRange();
    return closed + pyMin(1.0, pyMax(0.0, opening)) * (full - closed);
  }

  // ── enabling ──
  /**
   * Torque on, holding the measured pose. `signal` (like estop()) aborts an enable in progress:
   * torque already applied is released and StateError is thrown. `speed` (rad/s) caps the motion
   * until the first target, such as a joint resting past a limit being brought inside it (default:
   * maxVelocity).
   */
  enable(options: EnableOptions = {}): Promise<void> {
    const { startLoop = true, signal, speed } = options;
    if (speed !== undefined && speed !== null && !(Number.isFinite(speed) && speed > 0)) {
      return Promise.reject(new ValueError("speed must be positive"));
    }
    // Taken when enable() is called, so an estop() made while it waits its turn cancels it too.
    const generation = this.generation;
    return this.#lifecycle.run(async () => {
      if (this.#state !== ArmState.CONNECTED) {
        throw new StateError(`enable() requires CONNECTED state, currently ${stateName(this.#state)}`);
      }
      if (this.loop !== null && this.loop.alive) {
        throw new StateError("the previous control loop is still running; refusing to enable");
      }
      const abortIfCancelled = (): void => {
        if (this.generation !== generation) throw new StateError("e-stop during enable");
        if (signal?.aborted) throw new StateError("enable cancelled");
      };

      abortIfCancelled();
      for (const m of this.motors) {
        if (m.faultLatched) {
          throw new ConnectError(`motor ${m.motorId} has a latched fault (${m.faultLatched}); clearFaults() before enabling`);
        }
      }
      // Fresh fault status and position from every motor; cached values never authorize torque.
      await this.#checkMotors();
      // A motor already holding (another session) would lose its torque to the neutral preload,
      // and a failed enable releases every motor: refuse before any frame.
      const holding = this.motors
        .map((m, i) => [m, this.config.joints[i]!] as const)
        .filter(([m]) => m.feedback !== null && m.feedback.enabled)
        .map(([m, j]) => `${m.motorId} (${j.label})`);
      if (holding.length) {
        throw new ConnectError(`motors ${holding.join(", ")} already have torque on; nothing was sent to them. `
          + "Take the arm over with attach() (returnToRest(rest) then disable() bring it down and release it)");
      }
      const pos = this.getJointPositions();
      this.#checkStart(pos, range(this.config.nJoints));
      this.gravity = this.#makeGravity();
      await this.#armWatchdogs();
      this.#startTargets(pos, speed ?? null);
      this.restPose = [...pos];
      try {
        for (const [i, m] of this.motors.entries()) {
          abortIfCancelled();
          // Neutral preload (kp=kd=0 at the present pose) so the enable cannot resume a stale
          // setpoint left in the motor by an earlier session.
          if (!(await m.preload(this.#toMotor(i, pos[i]!), this.REPLY_TIMEOUT))) {
            throw new ConnectError(`motor ${m.motorId} did not acknowledge the neutral preload`);
          }
          // The gate keeps estop()'s torque-off from landing between this check and the enable
          // frames (retries included): it waits for the confirmation, then releases every motor.
          const confirmed = await this.#enableGate.run(() => {
            abortIfCancelled();
            return m.enableConfirmed(this.REPLY_TIMEOUT);
          });
          if (m.faultLatched) throw new ConnectError(`motor ${m.motorId} reported a fault on enable (${m.faultLatched})`);
          if (!confirmed) {
            throw new ConnectError(`motor ${m.motorId} enable rejected: no reply, or its feedback does not report torque on`);
          }
          await this.#clock.sleep(0.01);
        }
        abortIfCancelled();
      } catch (e) {
        // Includes cancellation and a motor whose enable may have landed before the error.
        await this.#releaseAll();
        throw e;
      }
      this.#goEnabled(startLoop);
    });
  }

  /**
   * Take over an arm left holding with torque on and no owning process (a crashed program), from
   * CONNECTED, without letting it sag: nothing is sent to a motor that reports torque on except
   * holding frames at its measured pose (profile gains, gravity feed-forward) from the control
   * loop. A motor that reports torque off (a gripper its watchdog released; connect with
   * `clearReleased = true`) is enabled at its measured pose as enable() does. A motor whose
   * firmware does not report torque state first gets one holding frame, then an enable, so whether
   * it was holding or not it ends holding its measured pose.
   *
   * Refused (ConnectError, nothing sent) when every motor reports torque off (use enable()), on
   * missing or stale feedback, a latched fault, or a joint outside its limits. On a later failure
   * only the motors this call enabled are released. Then ENABLED, with every protection enable()
   * gives. `restPose` stays null, so stop() holds rather than releasing until a rest target is
   * given (returnToRest(rest), or set `restPose`). `signal` and `speed` as in enable().
   */
  attach(signal?: AbortSignal, speed?: number): Promise<void> {
    if (speed !== undefined && speed !== null && !(Number.isFinite(speed) && speed > 0)) {
      return Promise.reject(new ValueError("speed must be positive"));
    }
    const generation = this.generation;
    return this.#lifecycle.run(async () => {
      if (this.#state !== ArmState.CONNECTED) {
        throw new StateError(`attach() requires CONNECTED state, currently ${stateName(this.#state)}`);
      }
      if (this.loop !== null && this.loop.alive) {
        throw new StateError("the previous control loop is still running; refusing to attach");
      }
      const abortIfCancelled = (): void => {
        if (this.generation !== generation) throw new StateError("e-stop during attach");
        if (signal?.aborted) throw new StateError("attach cancelled");
      };

      abortIfCancelled();
      for (const m of this.motors) {
        if (m.faultLatched) throw new ConnectError(`motor ${m.motorId} has a latched fault (${m.faultLatched}); refusing to attach`);
      }
      await this.#checkMotors();
      const torque = this.motors.map((m) => m.feedback!.enabled);
      if (!torque.some((t) => t !== false)) {
        throw new ConnectError("no motor reports torque on: nothing to attach to; use enable()");
      }
      for (const [i, m] of this.motors.entries()) {
        if (m.feedbackAge > this.config.feedbackTimeout) {
          throw new ConnectError(`${this.config.joints[i]!.label}: feedback is ${m.feedbackAge.toFixed(2)} s old; refusing to attach`);
        }
      }
      const pos = this.getJointPositions();
      const enabling = torque.flatMap((t, i) => (t !== true ? [i] : []));
      this.#checkStart(pos, enabling, "attach");
      this.gravity = this.#makeGravity();
      await this.#armWatchdogs();
      this.#startTargets(pos, speed ?? null);
      this.restPose = null;
      const enabled: number[] = [];
      try {
        const feedforward = await this.#feedforward([...pos], this.#clock.now());
        for (const i of enabling) {
          const m = this.motors[i]!, j = this.config.joints[i]!;
          abortIfCancelled();
          if (torque[i] === null) {
            // Holds now if the motor is energized; the enable below resumes this frame.
            await m.sendCommand(this.#toMotor(i, pos[i]!), 0.0, j.kp, j.kd, feedforward[i]!);
            await this.#clock.sleep(0.002);
          } else if (!(await m.preload(this.#toMotor(i, pos[i]!), this.REPLY_TIMEOUT))) {
            throw new ConnectError(`motor ${m.motorId} did not acknowledge the neutral preload`);
          }
          const confirmed = await this.#enableGate.run(() => {
            abortIfCancelled();
            enabled.push(i);
            return m.enableConfirmed(this.REPLY_TIMEOUT);
          });
          if (m.faultLatched) throw new ConnectError(`motor ${m.motorId} reported a fault on enable (${m.faultLatched})`);
          if (!confirmed) {
            throw new ConnectError(`motor ${m.motorId} enable rejected: no reply, or its feedback does not report torque on`);
          }
          await this.#clock.sleep(0.01);
        }
        abortIfCancelled();
      } catch (e) {
        // The motors that were holding keep their torque: releasing them would drop the arm.
        await this.#releaseEnabled(enabled.filter((i) => torque[i] === false));
        await this.#restoreWatchdogs(new Set(this.watchdogRestore.keys()));
        throw e;
      }
      this.#goEnabled(true);
    });
  }

  /**
   * Refuse to start without a position, outside the limits, or (joints in `checkTemperature`) at
   * or above the enable temperature.
   */
  #checkStart(pos: number[], checkTemperature: Iterable<number>, action = "enable"): void {
    const grace = this.config.fullTurnGrace;
    const hot = new Set(checkTemperature);
    this.config.joints.forEach((j, i) => {
      const p = pos[i]!;
      if (!Number.isFinite(p)) throw new ConnectError(`${j.label}: no position before ${action}, refusing to ${action}`);
      if (p < j.lo - grace || p > j.hi + grace) {
        throw new ConnectError(
          `${j.label} (motor ${j.motorId}) at ${signed(degrees(p), 1)} deg is outside its limits `
          + `[${degrees(j.lo).toFixed(1)}, ${degrees(j.hi).toFixed(1)}] deg even after full-turn correction -- `
          + `stale or missing zero. Re-zero with setZero() at the zero pose (${this.config.product.zeroPose})`);
      }
      const temp = this.motors[i]!.feedback!.temperature;
      const limit = j.enableTemperatureC !== null ? j.enableTemperatureC : j.maxTemperatureC;
      if (hot.has(i) && limit !== null && temp >= limit) {
        throw new ConnectError(`${j.label} is ${temp.toFixed(1)} °C; ${action} needs it below ${limit.toFixed(0)} °C`);
      }
    });
  }

  /** Targets, gripper regulation and fault evidence for a session starting at `pos`. */
  #startTargets(pos: number[], speed: number | null): void {
    const now = this.#clock.now();
    this.userTargets = [...pos];
    this.internal = [...pos];
    this.synced = new Array<boolean>(this.config.nJoints).fill(false);
    this.targetsReceived = false;
    this.returning = false;
    this.speed = speed;
    this.stuckAnchor = pos.map((p) => [p, now]);
    this.hold = new Map();
    this.config.joints.forEach((j, i) => {
      if (j.holdTorqueNm !== null) {
        this.hold.set(i, new GripperHold(j.holdTorqueNm, [j.lo, j.hi], this.config.product.kpRange, j.closingDirection));
      }
    });
    this.holdOut.clear();
    this.releaseSent.clear();
    this.released.clear();
    this.faulted.clear();
    this.faultReason = null;
    this.faultReasons = [];
  }

  #goEnabled(startLoop: boolean): void {
    const now = this.#clock.now();
    // Each motor's next status poll is due from its last status reply, so a slow enable cannot
    // leave a reply older than the status deadline.
    this.pollSent = new Map(this.motors.map((m) => [m.motorId, now - pyMin(m.statusAge, now)]));
    this.pollingSince = now;
    this.#state = ArmState.ENABLED;
    if (startLoop) {
      this.running = true;
      const handle: LoopHandle = { promise: Promise.resolve(), alive: true };
      const session = this.session;
      handle.promise = this.#controlLoop(session)
        .catch((e: unknown) => {
          // Nothing services the motors any more: refuse targets and show it.
          if (this.running && this.session === session) {
            this.running = false;
            this.faultReason = `control loop stopped: ${errorText(e)}`;
            this.faultReasons.push(this.faultReason);
            this.#state = ArmState.FAULT;
            this.#tickSignal.notify();
          }
          try {
            this.#log.error(`control loop stopped: ${errorText(e)}`);
          } catch {
            // nothing left to report through
          }
        })
        .finally(() => {
          handle.alive = false;
        });
      this.loop = handle;
    }
  }

  /** Verified torque-off on the motors at `indices` only. */
  async #releaseEnabled(indices: number[]): Promise<void> {
    const positions = this.getJointPositions();
    const unconfirmed: number[] = [];
    for (const i of indices) {
      const m = this.motors[i]!;
      try {
        if (Number.isFinite(positions[i])) await m.sendCommand(this.#toMotor(i, positions[i]!), 0.0, 0.0, 0.0, 0.0);
        if (!(await m.disableConfirmed(this.REPLY_TIMEOUT, this.REPLY_ATTEMPTS))) unconfirmed.push(m.motorId);
      } catch (e) {
        this.#log.error(`failed to disable motor ${m.motorId}: ${errorText(e)}`);
        unconfirmed.push(m.motorId);
      }
    }
    this.unconfirmedDisable = unconfirmed;
    if (unconfirmed.length) {
      this.#log.error(`could not confirm torque release on motors [${unconfirmed.join(", ")}]; cut motor power if they hold`);
    }
  }

  #waitReply(m: Motor, sequence: number, timeout: number): Promise<boolean> {
    return m.changed.wait(() => m.feedbackSequence !== sequence, timeout, this.#clock);
  }

  /**
   * Let late replies to earlier requests arrive, so the next reply answers our request (MIT
   * replies carry no transaction ID).
   */
  async #waitQuiet(m: Motor, quiet = 0.005, limit = 0.05): Promise<void> {
    const deadline = this.#clock.now() + limit;
    while (this.#clock.now() < deadline) {
      const sequence = m.feedbackSequence;
      await this.#clock.sleep(quiet);
      if (m.feedbackSequence === sequence) return;
    }
  }

  /** Set and verify the motor-side CAN timeout where configured. */
  async #armWatchdogs(): Promise<void> {
    this.watchdogRestore.clear();
    try {
      for (const [i, j] of this.config.joints.entries()) {
        if (j.canTimeoutMs === null) continue;
        const token = await this.motors[i]!.armWatchdog(j.canTimeoutMs, this.PARAM_TIMEOUT);
        if (token !== null && token !== undefined) this.watchdogRestore.set(i, token);
      }
    } catch (e) {
      await this.#restoreWatchdogs(new Set(this.watchdogRestore.keys()));
      if (e instanceof ParamTimeout) throw new ConnectError(`motor watchdog setup failed: ${e.message}`, { cause: e });
      throw e;
    }
  }

  /** Restore the original CAN timeout, only on motors confirmed disabled. */
  async #restoreWatchdogs(indices: Set<number>): Promise<void> {
    for (const i of [...indices].sort((a, b) => a - b)) {
      const token = this.watchdogRestore.get(i);
      this.watchdogRestore.delete(i);
      if (token === undefined || token === null) continue;
      try {
        await this.motors[i]!.restoreWatchdog(token, this.PARAM_TIMEOUT);
      } catch (e) {
        this.#log.error(`could not restore ${this.config.joints[i]!.label} CAN timeout: ${errorText(e)}`);
      }
    }
  }

  /** Fast unconfirmed torque-off broadcast (e-stop / fault path). */
  async #disableAll(): Promise<void> {
    for (const m of this.motors) {
      try {
        await m.disable();
      } catch (e) {
        this.#log.error(`failed to disable motor ${m.motorId}: ${errorText(e)}`);
      }
    }
  }

  /**
   * Neutralize then disable each motor, confirming the reply (and the reported mode where the
   * firmware reports it). Returns the motor IDs that could not be confirmed.
   */
  async #disableConfirmed(): Promise<number[]> {
    // Torque off everywhere first so an offline motor cannot delay the healthy ones, after a
    // zero-gain command so a later enable cannot resume the old setpoint.
    const positions = this.getJointPositions();
    for (const [i, m] of this.motors.entries()) {
      if (Number.isFinite(positions[i])) {
        try {
          await m.sendCommand(this.#toMotor(i, positions[i]!), 0.0, 0.0, 0.0, 0.0);
        } catch (e) {
          this.#log.error(`failed to neutralize motor ${m.motorId}: ${errorText(e)}`);
        }
      }
    }
    await this.#disableAll();
    const unconfirmed: number[] = [];
    for (const m of this.motors) {
      let confirmed = false;
      for (let attempt = 0; attempt < this.REPLY_ATTEMPTS; attempt++) {
        let sequence: number;
        try {
          await this.#waitQuiet(m);
          sequence = m.feedbackSequence;
          await m.disable();
        } catch (e) {
          this.#log.error(`failed to disable motor ${m.motorId}: ${errorText(e)}`);
          continue;
        }
        if ((await this.#waitReply(m, sequence, this.REPLY_TIMEOUT)) && m.confirms("disabled")) {
          confirmed = true;
          break;
        }
      }
      if (!confirmed) unconfirmed.push(m.motorId);
    }
    return unconfirmed;
  }

  async #releaseAll(): Promise<void> {
    const unconfirmed = await this.#disableConfirmed();
    await this.#restoreWatchdogs(new Set(range(this.motors.length).filter((i) => !unconfirmed.includes(this.motors[i]!.motorId))));
    this.unconfirmedDisable = unconfirmed;
    if (unconfirmed.length) {
      this.#log.error(`could not confirm torque release on motors [${unconfirmed.join(", ")}]; their watchdog stays armed. `
        + "Cut motor power if they are still holding.");
    }
  }

  /** Resolves true once the loop has ended, false after `timeout` seconds. */
  #joinLoop(handle: LoopHandle, timeout: number): Promise<boolean> {
    if (!handle.alive) return Promise.resolve(true);
    return new Promise<boolean>((resolve) => {
      const cancel = this.#clock.timer(timeout, () => resolve(!handle.alive));
      void handle.promise.then(() => {
        cancel();
        resolve(true);
      });
    });
  }

  async #stopLoop(): Promise<void> {
    this.running = false;
    this.session += 1; // a tick still in flight stops sending at its next command
    this.#tickSignal.notify();
    const handle = this.loop;
    if (handle) {
      if (!(await this.#joinLoop(handle, 1.0))) {
        this.#log.error("control loop did not stop within 1 s; waiting");
        await this.#joinLoop(handle, 2.0);
      }
      if (handle.alive) this.#log.error("control loop is still alive; enable() stays refused until it exits");
      else this.loop = null;
    }
  }

  /**
   * Release torque immediately. Prefer stop(), which returns to rest first. Waits for a lifecycle
   * call in progress (a stop() finishes its return to rest first); estop() is the call that does
   * not wait.
   */
  disable(): Promise<void> {
    return this.#lifecycle.run(() => this.#disable());
  }

  async #disable(): Promise<void> {
    if (this.#state === ArmState.DISCONNECTED) return;
    await this.#stopLoop();
    await this.#releaseAll();
    this.released.clear();
    this.faulted.clear();
    this.returning = false;
    if (this.#state === ArmState.ENABLED || this.#state === ArmState.FAULT) this.#state = ArmState.CONNECTED;
  }

  /**
   * Callable from any state and any async context: torque off now (the arm will drop). The flags
   * that stop an enable and the control loop are set before the returned promise is created.
   */
  async estop(): Promise<void> {
    this.#log.warn("e-stop: releasing torque on all motors");
    this.generation += 1;
    this.running = false;
    this.session += 1;
    this.#tickSignal.notify();
    // An enable frame being confirmed (and its retries) goes out first, never after the torque-off.
    await this.#enableGate.run(() => undefined);
    if (this.#state !== ArmState.DISCONNECTED) await this.#disableAll();
    await this.disable();
  }

  /**
   * Return to the rest pose, then release torque only if the return was verified.
   *
   * An arm without brakes must never be released away from rest. If the return cannot be verified
   * the arm keeps holding its current pose and false is returned; the caller decides when to
   * disable().
   */
  stop(): Promise<boolean> {
    return this.#lifecycle.run(async () => {
      if (this.#state !== ArmState.ENABLED && this.#state !== ArmState.FAULT) return true;
      let verified: boolean;
      try {
        verified = await this.#returnToRest();
      } catch (e) {
        this.#log.error(`return to rest failed: ${errorText(e)}`);
        verified = false;
      }
      if (verified) {
        await this.#disable();
        return true;
      }
      this.returning = false;
      this.holdCurrentPosition();
      return false;
    });
  }

  // ── operator actions while torque is off ──
  /**
   * Make each selected joint's (default: every joint's) present position its motor's zero.
   * Persists in the motors. Torque must be off (CONNECTED); refresh() afterwards.
   */
  setZero(indices?: number[] | null): Promise<void> {
    return this.#lifecycle.run(async () => {
      if (this.#state !== ArmState.CONNECTED) {
        throw new StateError(`setZero() requires CONNECTED state, currently ${stateName(this.#state)}`);
      }
      const selected = indices === undefined || indices === null ? range(this.config.nJoints) : [...indices];
      for (const i of selected) {
        if (!(Number.isInteger(i) && i >= 0 && i < this.config.nJoints)) throw new RangeError(`joint index ${i} out of range`);
      }
      for (const i of selected) {
        await this.motors[i]!.setZero();
        this.wrap[i] = 0.0;
        await this.#clock.sleep(0.01);
      }
    });
  }

  /**
   * Persistently change a motor's CAN ID from `oldId` to `newId` while torque is off.
   *
   * Works connected or disconnected (the bus is opened for the call). Refuses if `newId` already
   * answers. A configured joint that had `oldId` is routed to `newId` from then on (refresh()
   * reads it again); update the profile to keep the change.
   */
  async setMotorId(oldId: number, newId: number, timeout = 0.5): Promise<void> {
    const product = this.config.product;
    const [lo, hi] = product.idRange;
    for (const value of [oldId, newId]) {
      if (!(lo <= value && value <= hi)) throw new ValueError(`motor IDs must be in [${lo}, ${hi}], got ${value}`);
    }
    if (oldId === newId) throw new ValueError("old and new motor IDs are the same");
    return this.#lifecycle.run(async () => {
      if (this.#state !== ArmState.DISCONNECTED && this.#state !== ArmState.CONNECTED) {
        throw new StateError(`setMotorId() requires torque off, currently ${stateName(this.#state)}`);
      }
      const hostId = this.config.hostId;
      const joint = this.config.joints.find((j) => j.motorId === oldId)
        ?? new JointConfig({ motorId: oldId, lo: -Math.PI, hi: Math.PI, kp: 0.0, kd: 0.0, model: product.defaultModel });
      const target = product.createMotor(joint, this.#backend, hostId, this.#clock, this.#log);
      const probe = product.createMotor(joint.replace({ motorId: newId }), this.#backend, hostId, this.#clock, this.#log);
      const temporary = new Map([[oldId, target], [newId, probe]]);
      const receive = (canId: number, data: Uint8Array, extended: boolean): void => {
        const id = product.route(canId, data, hostId, extended);
        const m = id === null ? undefined : temporary.get(id);
        if (m !== undefined) m.handleFrame(canId, data, extended);
      };
      const opened = this.#state === ArmState.DISCONNECTED;
      this.#backend.setRecvCallback(receive);
      try {
        if (opened) await this.#backend.open();
        let taken: boolean;
        try {
          await probe.check(this.REPLY_TIMEOUT, this.REPLY_ATTEMPTS);
          taken = true;
        } catch (e) {
          if (!(e instanceof ParamTimeout)) throw e;
          taken = false;
        }
        if (taken) throw new ArmError(`motor ID ${newId} already answers on the bus`);
        await target.setId(newId, timeout);
        this.#rekey(oldId, newId, target);
      } finally {
        if (opened) await this.#backend.close();
        this.#backend.setRecvCallback(this.#onFrame);
      }
    });
  }

  /** After an id change, route the joint that had `oldId` to its motor at `newId`. */
  #rekey(oldId: number, newId: number, changed: Motor): void {
    const index = this.config.joints.findIndex((j) => j.motorId === oldId);
    if (index < 0) return;
    let joint = this.config.joints[index]!;
    const replyId = joint.replyId !== null ? (changed.replyId ?? null) : null;
    joint = joint.replace({ motorId: newId, replyId });
    this.config.joints[index] = joint;
    if (this.config.requiredMotorIds !== null) {
      this.config.requiredMotorIds = this.config.requiredMotorIds.map((i) => (i === oldId ? newId : i));
    }
    this.motors[index] = this.config.product.createMotor(joint, this.#backend, this.config.hostId, this.#clock, this.#log);
    this.#byId = new Map(this.motors.map((m) => [m.motorId, m]));
    this.pollSent.delete(oldId);
  }

  // ── gains ──
  /**
   * Per-joint gains for the frames that follow (one value per joint; null keeps the present
   * ones). With neither given, the profile's gains come back. A joint with kp 0 floats: its
   * command follows the measured pose, so raising kp again holds it there.
   */
  setGains(kp?: readonly number[] | null, kd?: readonly number[] | null): void {
    const n = this.config.nJoints;
    let gains: Array<[number, number]>;
    if ((kp === undefined || kp === null) && (kd === undefined || kd === null)) {
      gains = [...this.profileGains];
    } else {
      const kps = kp === undefined || kp === null ? this.config.joints.map((j) => j.kp) : kp.map(Number);
      const kds = kd === undefined || kd === null ? this.config.joints.map((j) => j.kd) : kd.map(Number);
      if (kps.length !== n || kds.length !== n) throw new ValueError(`need ${n} kp and kd values`);
      gains = kps.map((p, i) => [p, kds[i]!]);
    }
    this.config.joints.forEach((j, i) => checkGains(this.config.product, j.motorId, gains[i]![0], gains[i]![1]));
    this.config.joints = this.config.joints.map((j, i) => j.replace({ kp: gains[i]![0], kd: gains[i]![1] }));
    this.extraTorque = null;
    this.floating = false;
  }

  /**
   * Back-drivable arm (teleop leader): kp 0 and light damping on every joint, gravity feed-forward
   * only. `extraTorque(positions, velocities)` (joint coordinates, one N·m value per joint) is
   * added to the feed-forward every tick, e.g. friction compensation; if it throws or returns a bad
   * value it is dropped (logged) and the frames go on without it. setGains() ends float mode: with
   * no arguments the profile's gains come back. A fault, or a failure of the gravity feed-forward,
   * ends it too: the arm stiffens at the measured pose with the profile's gains and holds there as
   * on any fault.
   */
  floatMode(kd?: number | null, extraTorque?: ExtraTorque | null): void {
    const n = this.config.nJoints;
    if (extraTorque !== undefined && extraTorque !== null && typeof extraTorque !== "function") {
      throw new TypeError("extraTorque must be callable");
    }
    this.setGains(new Array<number>(n).fill(0.0), new Array<number>(n).fill(kd === undefined || kd === null ? Arm.FLOAT_KD : kd));
    this.extraTorque = extraTorque ?? null;
    this.floating = true;
  }

  /** Leave float mode on a fault: targets at the measured pose, then the profile gains, no extra torque. */
  #endFloat(): void {
    const positions = this.getJointPositions();
    positions.forEach((p, i) => {
      if (Number.isFinite(p)) this.userTargets[i] = this.internal[i] = p;
    });
    this.config.joints = this.config.joints.map((j, i) => j.replace({ kp: this.profileGains[i]![0], kd: this.profileGains[i]![1] }));
    this.extraTorque = null;
    this.floating = false;
  }

  // ── kinematics (needs the profile's urdf) ──
  /** The arm's kinematic chain from the profile (built on first use). ArmError without a URDF. */
  get chain(): kinematics.Chain {
    const cfg = this.config;
    const key = JSON.stringify([cfg.urdf ? String(cfg.urdf) : null, cfg.urdfJoints ?? [], cfg.urdfBaseRpy, cfg.tcpXyz, cfg.tcpRpy,
      cfg.joints.map((j) => [j.urdfSign, j.urdfOffset])]);
    if (this.#chain === null || this.#chain[0] !== key) {
      if (!cfg.urdf) throw new ArmError("the profile names no urdf: kinematics (eePose, moveEe) is unavailable");
      let chain: kinematics.Chain | null;
      try {
        chain = kinematics.chainFromConfig(cfg);
      } catch (e) {
        if (e instanceof ValueError) throw new ArmError(`cannot build the kinematic chain from the profile's urdf: ${e.message}`, { cause: e });
        throw e;
      }
      if (!chain) throw new ArmError("the profile names no urdf: kinematics (eePose, moveEe) is unavailable");
      this.#chain = [key, chain];
    }
    return this.#chain[1];
  }

  /** Measured tool point: [xyz in m, rpy in rad] in the world frame. */
  eePose(): [number[], number[]] {
    const chain = this.chain;
    const q = this.getJointPositions().slice(0, chain.n);
    if (!q.every(Number.isFinite)) throw new StateError("no position feedback; connect() or refresh() first");
    const [xyz, rpy] = kinematics.poseToXyzRpy(chain.fk(q));
    return [[...xyz], [...rpy]];
  }

  /**
   * Move the tool point in a straight line to `xyz` (m, world frame), turning to `options.rpy`
   * (rad) when given; without it the end orientation is the one a position-only IK reaches with the
   * least joint motion. `speed` caps each joint in rad/s, as in moveTo().
   *
   * The path starts at the commanded pose and eases in and out (minimum jerk); each control tick of
   * it is solved by the trajectory IK and streamed through the joint targets, so no tick moves a
   * joint further than its speed allows and a small tool move stays a small joint move. Joints past
   * the chain (the gripper) keep their command. Refused (false, nothing sent) when not ENABLED or
   * when the end pose is out of reach within the soft limits. With `wait` it waits until arrival as
   * moveTo() does (false on timeout, a fault or newer targets taking over); without it the path
   * streams in the background. Aborting `signal` stops the motion where it is (holding the measured
   * pose) and resolves false. ArmError without a URDF.
   */
  async moveEe(xyz: readonly number[], options: MoveEeOptions = {}): Promise<boolean> {
    const { rpy, speed, timeout, wait = true, signal } = options;
    const chain = this.chain;
    if (this.#state !== ArmState.ENABLED) {
      this.#log.warn(`moveEe rejected: state=${stateName(this.#state)}`);
      return false;
    }
    const target = Array.from(xyz);
    const rpyGoal = rpy === undefined || rpy === null ? null : Array.from(rpy);
    const three = (v: readonly unknown[]): boolean => v.length === 3 && v.every((x) => typeof x === "number" && Number.isFinite(x));
    if (!three(target) || (rpyGoal !== null && !three(rpyGoal))) {
      throw new ValueError("xyz (and rpy) need three finite values");
    }
    // Planning takes several turns of the event loop; the plan is only used in the session it was made for.
    const session = this.session;
    const superseded = (): boolean => {
      if (this.session === session && this.#state === ArmState.ENABLED) return false;
      this.#log.warn("moveEe dropped: the arm was stopped or re-enabled while the path was planned");
      return true;
    };
    if (speed !== undefined && speed !== null && !(Number.isFinite(speed) && speed > 0)) {
      throw new ValueError("speed must be positive");
    }
    const cfg = this.config;
    const ik = trajectoryIk.TrajectoryIK.fromConfig(cfg, { chain });
    const current = this.getCommandedPositions();
    const q0 = current.slice(0, chain.n);
    // The solves give the event loop a turn every few milliseconds: the control loop and the
    // receive path share this thread and must keep running while the path is planned.
    let end: kinematics.IKResult;
    try {
      end = await chain.ikAsync(target, q0.map((q, i) => clip(ik.lo[i]!, ik.hi[i]!, q)),
        { rpy: rpyGoal, limits: [ik.lo, ik.hi], signal });
    } catch (e) {
      if (signal?.aborted) return false;
      throw e;
    }
    if (superseded()) return false;
    if (!end.ok) {
      this.#log.warn(`moveEe rejected: no solution within the limits (${(end.posErr * 1e3).toFixed(1)} mm, `
        + `${degrees(end.rotErr).toFixed(2)} deg off)`);
      return false;
    }
    const start = chain.fk(q0);
    const goal = chain.fk(end.q);
    for (let r = 0; r < 3; r++) goal[r]![3] = target[r]!;
    if (rpyGoal !== null) {
      const rot = kinematics.rpyToMatrix(rpyGoal);
      for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) goal[r]![c] = rot[r]![c]!;
    }
    const rate = cfg.controlRateHz;
    const speeds = ik.maxVelocity.map((v: number) => Math.min(v, speed === undefined || speed === null ? Infinity : speed));
    // Minimum jerk peaks at 1.875x the mean speed; a path the cap still shortens is retimed slower.
    let peak = -Infinity;
    end.q.forEach((q: number, i: number) => {
      peak = Math.max(peak, 1.875 * Math.abs(q - q0[i]!) / speeds[i]!);
    });
    const seconds = pyMax(EE_MIN_SECONDS, 1.25 * peak);
    let ticks = Math.ceil(seconds * rate);
    const maxTicks = Math.ceil(EE_MAX_SECONDS * rate);
    let plan!: trajectoryIk.IKResult;
    for (let attempt = 0; attempt < 4; attempt++) {
      // the goal held a while at the end lets the smoothed solve settle onto it
      if (ticks > maxTicks) {
        // Building and solving the path runs on the thread that services the motors.
        this.#log.warn(`moveEe rejected: the path would take ${(ticks / rate).toFixed(0)} s (limit ${EE_MAX_SECONDS} s); raise the speed or move in steps`);
        return false;
      }
      const holdTicks = Math.max(Math.ceil(EE_MIN_SECONDS * rate / 2), Math.floor(ticks / 5));
      const poses = [...trajectoryIk.straightLine(start, goal, ticks)];
      for (let k = 0; k < holdTicks; k++) poses.push(goal.map((row) => [...row]));
      try {
        plan = await trajectoryIk.solvePathAsync(ik, poses, q0, rate, { speedCap: speed ?? null, signal });
      } catch (e) {
        if (signal?.aborted) return false;
        throw e;
      }
      if (superseded()) return false;
      if (!plan.capped.some(Boolean)) break;
      ticks = Math.ceil(ticks * 1.5);
    }
    const lastPos = plan.posErr[plan.posErr.length - 1]!, lastRot = plan.rotErr[plan.rotErr.length - 1]!;
    if (lastPos > EE_POSITION_TOL || lastRot > EE_ANGLE_TOL) {
      this.#log.warn(`moveEe rejected: the path ends ${(lastPos * 1e3).toFixed(1)} mm, ${degrees(lastRot).toFixed(2)} deg off the goal`);
      return false;
    }
    const rows = plan.q.map((q: readonly number[]) => [...q, ...current.slice(chain.n)]);
    if (signal?.aborted) return false;
    const deadline = timeout === undefined || timeout === null ? null : this.#clock.now() + timeout;
    if (!wait) {
      if (!this.#setTargets(rows[0]!, speed ?? null)) return false;
      void this.#stream(rows, speed ?? null, signal, deadline).catch((e) => this.#log.error(`moveEe stream failed: ${errorText(e)}`));
      return true;
    }
    if (!(await this.#stream(rows, speed ?? null, signal, deadline))) return false;
    const remaining = deadline === null ? undefined : Math.max(0.0, deadline - this.#clock.now());
    return this.moveTo(rows[rows.length - 1]!, { speed, timeout: remaining, wait: true, signal });
  }

  /**
   * Send `rows` as joint targets, one per control period. False when cancelled (holding the
   * measured pose), past `deadline`, refused, or when newer targets take over.
   */
  async #stream(rows: number[][], speed: number | null, signal: AbortSignal | undefined, deadline: number | null): Promise<boolean> {
    const period = 1.0 / this.config.controlRateHz;
    let due = this.#clock.now();
    let sent: number[] | null = null;
    for (const row of rows) {
      if (signal?.aborted) {
        this.holdCurrentPosition();
        return false;
      }
      if (deadline !== null && this.#clock.now() > deadline) return false;
      if (sent !== null && !sameList(this.userTargets, sent)) return false;
      if (!this.#setTargets(row, speed)) return false;
      sent = row;
      due += period;
      await this.#clock.sleep(Math.max(0.0, due - this.#clock.now()));
    }
    return true;
  }

  // ── targets ──
  /** New joint targets (rad, one per joint); false (logged) when refused. */
  setJointTargets(targets: readonly number[]): boolean {
    return this.#setTargets(targets, null);
  }

  #setTargets(targets: readonly number[], speed: number | null): boolean {
    // Array.from fills holes, which every() would skip.
    const dense = Array.from(targets);
    if (dense.length !== this.config.nJoints || !dense.every((v) => typeof v === "number" && Number.isFinite(v))) {
      this.#log.warn(`setJointTargets rejected: need ${this.config.nJoints} finite targets, got [${dense.join(", ")}]`);
      return false;
    }
    // Admission is checked together with the ownership a return to rest takes, so a target can
    // never land after the return has started.
    if (this.#state !== ArmState.ENABLED || this.returning) {
      this.#log.warn(`setJointTargets rejected: state=${stateName(this.#state)} returning=${this.returning ? "True" : "False"}`);
      return false;
    }
    if (!this.targetsReceived) {
      // Startup sync is judged against the first real target, not the enable pose.
      const now = this.#clock.now();
      this.stuckAnchor = this.getJointPositions().map((p) => [p, now]);
      this.targetsReceived = true;
    }
    this.userTargets = dense;
    this.speed = speed;
    return true;
  }

  /**
   * Drive to joint pose `q` (rad) at up to `speed` rad/s (default: maxVelocity).
   *
   * With `wait` it waits until every stiff joint (kp > 0) is within restArrived of its clipped
   * goal and resolves true; false on timeout (default: generous for the distance), a fault, the
   * loop stopping, or newer targets taking over. Aborting `signal` stops the motion where it is
   * (holding the measured pose) and resolves false. Targets are refused unless ENABLED.
   */
  async moveTo(q: readonly number[], options: MoveOptions = {}): Promise<boolean> {
    const { speed, wait = true, signal } = options;
    let timeout = options.timeout;
    const goalQ = Array.from(q); // not converted: null or "" must be refused, not read as 0
    if (speed !== undefined && speed !== null && !(Number.isFinite(speed) && speed > 0)) {
      throw new ValueError("speed must be positive");
    }
    if (signal?.aborted) return false;
    if (!this.#setTargets(goalQ, speed ?? null)) return false;
    if (!wait) return true;
    const cfg = this.config;
    const goal = cfg.joints.map((j, i) => pyMin(j.hi - cfg.limitMargin, pyMax(j.lo + cfg.limitMargin, goalQ[i]!)));
    const rate = speed === undefined || speed === null ? cfg.maxVelocity : pyMin(cfg.maxVelocity, speed);
    if (timeout === undefined || timeout === null) {
      const start = this.getJointPositions();
      let distance = 0.0;
      let first = true;
      goal.forEach((g, i) => {
        const s = start[i]!;
        if (!Number.isFinite(s)) return;
        const d = Math.abs(g - s);
        if (first || d > distance) distance = d;
        first = false;
      });
      timeout = 1.5 * distance / rate + 2.0;
    }
    const deadline = this.#clock.now() + timeout;
    while (this.#clock.now() < deadline) {
      if (this.#state !== ArmState.ENABLED || !this.running) return false;
      if (signal?.aborted) {
        this.holdCurrentPosition();
        return false;
      }
      if (!sameList(this.userTargets, goalQ)) return false;
      const pos = this.getJointPositions();
      if (cfg.joints.every((j, i) => !(j.kp > 0) || (Number.isFinite(pos[i]) && Math.abs(pos[i]! - goal[i]!) <= cfg.restArrived))) {
        return true;
      }
      await this.#clock.sleep(0.005);
    }
    return false;
  }

  /** The rate-limited joint command currently sent by the control loop. */
  getCommandedPositions(): number[] {
    return [...this.internal];
  }

  /** Freeze the arm at its latest measured pose without releasing torque. */
  holdCurrentPosition(): boolean {
    if (this.#state !== ArmState.ENABLED && this.#state !== ArmState.FAULT) return false;
    const positions = this.getJointPositions();
    positions.forEach((p, i) => {
      if (Number.isFinite(p)) {
        this.userTargets[i] = p;
        this.internal[i] = p;
      }
    });
    return true;
  }

  /**
   * Drive joints marked return_to_rest back to `rest` (default: the pose at enable).
   *
   * Linear ramp from the measured pose at restSpeed, setpoints at restRateHz, judged by
   * convergence: arrived within restArrived, or within restSettle once improvement has stalled (a
   * loaded joint holds a standing error). Bounded by restTimeout. Never throws. New targets are
   * refused while it runs.
   */
  async returnToRest(rest?: readonly number[] | null): Promise<boolean> {
    try {
      return await this.#returnToRest(rest);
    } catch (e) {
      this.#log.error(`return to rest failed: ${errorText(e)}`);
      return false;
    } finally {
      this.returning = false;
    }
  }

  async #returnToRest(restArg?: readonly number[] | null): Promise<boolean> {
    const cfg = this.config;
    const rest = Array.from(restArg !== undefined && restArg !== null ? restArg : (this.restPose ?? []));
    if (!this.#restActive() || rest.length !== cfg.nJoints) return false;
    if (!rest.every((v) => typeof v === "number" && Number.isFinite(v))) {
      this.#log.error(`return to rest refused: the rest pose needs ${cfg.nJoints} finite values, got [${rest.join(", ")}]`);
      return false;
    }
    // Take exclusive ownership: refuse new targets, freeze at the measured pose and let any
    // command already computed by the loop go out before measuring the start.
    const positions = this.getJointPositions();
    this.returning = true;
    this.speed = null;
    positions.forEach((p, i) => {
      if (Number.isFinite(p)) {
        this.userTargets[i] = p;
        this.internal[i] = p;
      }
    });
    const sequences = this.motors.map((m) => m.feedbackSequence);
    if (!(await this.#waitTicks(2))) {
      this.#log.error("return to rest aborted: control loop is not running");
      return false;
    }
    const include = range(cfg.nJoints).filter((i) => cfg.joints[i]!.returnToRest && !this.released.has(i));
    if (!include.length) return true;
    const start = this.getJointPositions();
    if (!this.#restFeedbackOk(include, start)) return false;
    this.synced = new Array<boolean>(cfg.nJoints).fill(true);
    let worst = maxOf(include.map((i) => Math.abs(start[i]! - rest[i]!)));
    if (worst <= cfg.restArrived && this.#measuredSince(include, sequences)) return true;
    const period = 1.0 / cfg.restRateHz;
    const duration = worst / cfg.restSpeed;
    const began = this.#clock.now();
    const deadline = began + cfg.restTimeout;
    let frac = 0.0;
    while (frac < 1.0) {
      // The ramp follows the clock, so late wake-ups do not slow it below restSpeed.
      frac = pyMin(1.0, pyMax(period, this.#clock.now() - began) / duration);
      if (this.#clock.now() >= deadline || !this.#restActive()) break;
      for (const i of include) this.userTargets[i] = start[i]! + (rest[i]! - start[i]!) * frac;
      await this.#clock.sleep(period);
      if (!this.#restFeedbackOk(include, this.getJointPositions())) return false;
    }
    let best = Infinity, stalled = 0;
    while (this.#clock.now() < deadline && this.#restActive()) {
      const pos = this.getJointPositions();
      if (!this.#restFeedbackOk(include, pos)) return false;
      worst = maxOf(include.map((i) => Math.abs(pos[i]! - rest[i]!)));
      const fresh = this.#measuredSince(include, sequences);
      if (worst <= cfg.restArrived && fresh) return true;
      if (best - worst < cfg.restStallImprovement) {
        stalled += 1;
        if (stalled >= cfg.restStallPolls && fresh) return worst <= cfg.restSettle;
      } else {
        stalled = 0;
      }
      best = pyMin(best, worst);
      await this.#clock.sleep(period);
    }
    return false;
  }

  /** True once `n` more control ticks have run; false after `timeout` s or when the loop stops. */
  async #waitTicks(n: number, timeout = 1.0): Promise<boolean> {
    const target = this.tickCount + n;
    await this.#tickSignal.wait(() => this.tickCount >= target || !this.running, timeout, this.#clock);
    return this.tickCount >= target;
  }

  #restActive(): boolean {
    return this.running && (this.#state === ArmState.ENABLED || this.#state === ArmState.FAULT);
  }

  /**
   * Has every joint in `include` reported since its feedback sequence was `sequences`? Arrival is
   * judged only on such readings, never on the cached pose of a motor that has gone silent.
   */
  #measuredSince(include: number[], sequences: number[]): boolean {
    return include.every((i) => this.motors[i]!.feedbackSequence !== sequences[i]);
  }

  #restFeedbackOk(include: number[], positions: number[]): boolean {
    for (const i of include) {
      const label = this.config.joints[i]!.label;
      if (this.faulted.has(i)) {
        this.#log.error(`return to rest aborted: ${label} is faulted`);
        return false;
      }
      if (!Number.isFinite(positions[i]) || this.motors[i]!.feedbackAge > this.config.feedbackTimeout) {
        this.#log.error(`return to rest aborted: ${label} feedback is stale`);
        return false;
      }
    }
    return true;
  }

  // ── control loop ──
  /** The gap left after each command frame of a tick. */
  async #spaceFrames(): Promise<void> {
    if (this.#frameSpacing > 0) await this.#clock.sleep(this.#frameSpacing);
  }

  #skipped(i: number): boolean {
    return this.released.has(i) || this.faulted.has(i);
  }

  #maySend(session: number | null): boolean {
    return session === null || (this.running && session === this.session);
  }

  /** One control period: rate-limited commands, frames, a status poll, health checks. */
  private async tick(dt: number, session: number | null = null): Promise<void> {
    const cfg = this.config;
    const now = this.#clock.now();
    const present = this.getJointPositions();
    const speed = this.speed === null ? cfg.maxVelocity : pyMin(cfg.maxVelocity, this.speed);
    const maxstep = speed * dt;
    cfg.joints.forEach((j, i) => {
      if (this.#skipped(i)) return; // released: off; faulted: its target stays frozen at the fault pose
      if (j.kp === 0) {
        // Floating joint: the command follows the measured pose, so restoring kp holds here.
        if (Number.isFinite(present[i])) this.userTargets[i] = this.internal[i] = present[i]!;
        return;
      }
      let goal = pyMin(j.hi - cfg.limitMargin, pyMax(j.lo + cfg.limitMargin, this.userTargets[i]!));
      if (!this.synced[i] && this.targetsReceived && Number.isFinite(present[i])) {
        goal = this.#startupSync(i, goal, present[i]!, now);
      }
      const step = pyMin(maxstep, pyMax(-maxstep, goal - this.internal[i]!));
      this.internal[i]! += step;
    });
    const commands = [...this.internal];
    const feedforward = await this.#feedforward(commands, now);
    const paced = this.#backend.feedbackPacedControl;
    const joints = cfg.joints;
    for (let i = 0; i < joints.length; i++) {
      const j = joints[i]!;
      if (!this.#maySend(session)) return;
      const motor = this.motors[i]!;
      if (this.released.has(i)) {
        // Keep re-sending torque-off: a single lost disable must not leave it energized.
        if (now - (this.releaseSent.get(i) ?? 0.0) >= 0.1) {
          this.releaseSent.set(i, now);
          try {
            await motor.disable();
          } catch (e) {
            this.#log.error(`failed to disable motor ${motor.motorId}: ${errorText(e)}`);
          }
        }
        continue;
      }
      // A faulted (not released) joint keeps receiving its frozen hold command.
      let position = commands[i]!, kp = j.kp, kd = j.kd;
      const sequence = motor.feedbackSequence;
      try {
        const hold = this.hold.get(i);
        if (hold !== undefined && !this.faulted.has(i) && kp > 0) {
          [position, kp, kd] = this.#gripperCommand(i, j, hold, commands[i]!, now);
        }
        await motor.sendCommand(this.#toMotor(i, position), 0.0, kp, kd, feedforward[i]!);
      } catch (e) {
        if (!this.faulted.has(i)) await this.#enterFault(`${j.label}: command failed (${errorText(e)})`, i);
        continue;
      }
      if (paced) {
        // Old USB-SLCAN firmware has no outbound USB queue and a three-frame CAN RX FIFO: wait for
        // this reply before commanding the next motor. A timeout does not skip the remaining
        // motors; the feedback watchdog makes the safety decision.
        await this.#waitReply(motor, sequence, this.#backend.feedbackWaitTimeout);
      } else {
        await this.#spaceFrames();
      }
    }
    // Non-clearing status poll: at most one motor per tick (the longest-waiting one) so the replies
    // never burst into a small adapter FIFO.
    let dueAt = 0.0, dueIndex = -1;
    for (const [i, m] of this.motors.entries()) {
      if (this.#skipped(i)) continue;
      const sentAt = this.pollSent.get(m.motorId) ?? 0.0;
      if (dueIndex < 0 || sentAt < dueAt) {
        dueAt = sentAt;
        dueIndex = i;
      }
    }
    if (dueIndex >= 0 && this.#maySend(session)) {
      const sentAt = dueAt, i = dueIndex;
      if (now - sentAt >= cfg.faultPollInterval) {
        const m = this.motors[i]!;
        this.pollSent.set(m.motorId, now);
        try {
          await m.pollStatus();
        } catch (e) {
          this.#log.error(`fault query to motor ${m.motorId} failed: ${errorText(e)}`);
        }
      }
    }
    this.tickCount += 1;
    try {
      await this.#checkHealth();
    } finally {
      this.#tickSignal.notify();
    }
  }

  /**
   * This session's gravity feed-forward, or null (no URDF or gravityScale 0). Throws ValueError for
   * a bad scale or a missing cap, before any frame goes out.
   */
  #makeGravity(): gravity.GravityFeedforward | null {
    const cfg = this.config;
    if (!(cfg.gravityScale >= 0 && cfg.gravityScale <= gravity.MAX_GRAVITY_SCALE)) {
      throw new ValueError(`gravity_scale must be in [0, ${gravity.MAX_GRAVITY_SCALE}], got ${cfg.gravityScale}`);
    }
    if (!cfg.urdf || !cfg.gravityScale) return null;
    let chain: kinematics.Chain;
    try {
      chain = this.chain;
    } catch (e) {
      if (e instanceof ArmError) throw new ValueError(e.message, { cause: e });
      throw e;
    }
    return gravity.fromConfig(cfg, chain);
  }

  /**
   * Torque per joint in motor sign for this tick's frames: gravity (0 past the model's joints) plus
   * float mode's extra torque. A failure of either drops that term (gravity until the next enable,
   * the extra torque until floatMode() again), never the hold frames.
   */
  async #feedforward(commands: number[], now: number): Promise<number[]> {
    const cfg = this.config;
    const n = cfg.nJoints;
    let tau = new Array<number>(n).fill(0.0);
    const model = this.gravity;
    if (model !== null) {
      let failure: unknown = null;
      try {
        const k = model.n;
        const ages = this.motors.slice(0, k).map((m) => m.feedbackAge);
        tau.splice(0, k, ...model.torques(this.getJointPositions().slice(0, k), ages, commands.slice(0, k), now));
      } catch (e) {
        failure = e;
      }
      if (failure !== null) {
        this.#log.error(`gravity feed-forward failed; off until the next enable: ${errorText(failure)}`);
        this.gravity = null;
        tau = new Array<number>(n).fill(0.0);
        if (this.floating) await this.#enterFault(`gravity feed-forward failed while floating (${errorText(failure)})`);
      }
    }
    const extra = this.extraTorque;
    if (extra !== null) {
      try {
        const add = Array.from(extra(this.getJointPositions(), this.getJointVelocities()), (v) => {
          if (typeof v !== "number") throw new ValueError(`extra_torque returned a non-number: ${String(v)}`);
          return v;
        });
        if (add.length !== n || !add.every(Number.isFinite)) {
          throw new ValueError(`extra_torque must return ${n} finite values, got [${add.join(", ")}]`);
        }
        tau = tau.map((t, i) => t + add[i]!);
      } catch (e) {
        this.#log.error(`extra torque failed; off until floatMode() sets it again: ${errorText(e)}`);
        this.extraTorque = null;
      }
    }
    const joints = cfg.joints;
    return tau.slice(0, joints.length).map((t, i) => t * joints[i]!.direction);
  }

  /**
   * Holding regulation on one fresh, self-consistent feedback sample; without a new sample the last
   * output is reused. `j` is the tick's own copy of the joint, so a gain change made meanwhile
   * cannot reach the regulator half-applied.
   */
  #gripperCommand(i: number, j: JointConfig, hold: GripperHold, target: number, now: number): [number, number, number] {
    const m = this.motors[i]!;
    const fb = m.feedback!, sequence = m.feedbackSequence;
    const last = this.holdOut.get(i);
    if (last !== undefined && last[0] === sequence) return [last[1], last[2], last[3]];
    const position = hold.update(target, this.#toJoint(i, fb.position), fb.velocity * j.direction,
      fb.torque * j.direction, j.kp, now);
    const [kp, kd] = hold.gains(j.kp, j.kd);
    this.holdOut.set(i, [sequence, position, kp, kd]);
    return [position, kp, kd];
  }

  /** Cap the goal near the measured pose until the joint has caught up with its target. */
  #startupSync(i: number, goal: number, present: number, now: number): number {
    const cfg = this.config;
    if (Math.abs(goal - present) <= cfg.startupSynced) {
      this.synced[i] = true;
      return goal;
    }
    const [anchor, since] = this.stuckAnchor[i]!;
    if (Math.abs(present - anchor) > cfg.startupStuck) {
      this.stuckAnchor[i] = [present, now];
    } else if (now - since >= cfg.startupStuckSeconds) {
      this.synced[i] = true;
      return goal;
    }
    return pyMin(present + cfg.startupStep, pyMax(present - cfg.startupStep, goal));
  }

  async #controlLoop(session: number): Promise<void> {
    await this.#clock.sleep(0); // the loop starts once its starter is waiting, as a new thread would
    const dt = 1.0 / this.config.controlRateHz;
    let nextT = this.#clock.now();
    while (this.#maySend(session)) {
      try {
        await this.tick(dt, session);
      } catch (e) {
        // Never abandon energized motors: the hold only exists while commands go out, so keep
        // servicing (and reporting) until disable()/estop().
        try {
          if (this.#state === ArmState.ENABLED || this.faultReasons.length < 100) {
            await this.#enterFault(`control loop exception: ${errorText(e)}`);
          }
        } catch {
          // reporting failed; the next tick still goes out
        }
      }
      nextT += dt;
      try {
        const remain = nextT - this.#clock.now();
        if (remain > 0) {
          await this.#clock.sleep(remain);
        } else {
          nextT = this.#clock.now(); // if we've fallen behind, reset rather than trying to catch up
          await this.#clock.sleep(0); // and still give received frames a turn
        }
      } catch {
        // A clock that fails must not end the loop while motors are energized.
        nextT = performance.now() / 1000;
        await new Promise<void>((resolve) => setTimeout(resolve, dt * 1000));
      }
    }
  }

  /** Evaluated every tick, in FAULT too: a second motor can still fail while holding. */
  async #checkHealth(): Promise<void> {
    if (this.#state !== ArmState.ENABLED && this.#state !== ArmState.FAULT) return;
    const cfg = this.config;
    const statusDeadline = 2 * cfg.faultPollInterval + cfg.feedbackTimeout;
    // Only the control loop asks for status, so a reply's age counts from when it took over.
    const pollingFor = this.#clock.now() - this.pollingSince;
    const joints = cfg.joints;
    for (const [i, m] of this.motors.entries()) {
      const j = joints[i]!;
      if (this.#skipped(i)) continue;
      let reason: string | null;
      const fb = m.feedback;
      const statusAge = pyMin(m.statusAge, pollingFor);
      if (m.feedbackAge > cfg.feedbackTimeout) {
        reason = `feedback timeout ${m.feedbackAge.toFixed(2)}s -- check bus/power`;
      } else if (m.faultLatched) {
        reason = `motor fault -- ${m.faultLatched}`;
      } else if (statusAge > statusDeadline) {
        reason = `no fault-status reply for ${statusAge.toFixed(2)}s`;
      } else if (j.maxTemperatureC !== null && fb!.temperature >= j.maxTemperatureC) {
        reason = `overtemperature ${fb!.temperature.toFixed(1)} °C (limit ${j.maxTemperatureC.toFixed(0)} °C)`;
      } else if (j.warningIsFault && fb!.warning) {
        reason = "motor warning flag set";
      } else {
        reason = m.healthProblem();
      }
      if (reason) await this.#enterFault(`${j.label}: ${reason}`, i);
    }
  }

  private async enterFault(reason: string, index: number | null = null): Promise<void> {
    return this.#enterFault(reason, index);
  }

  async #enterFault(reason: string, index: number | null = null): Promise<void> {
    this.#log.error(`FAULT: ${reason}`);
    this.faultReasons.push(reason);
    if (this.faultReason === null) this.faultReason = reason;
    if (this.floating) this.#endFloat();
    if (index !== null) {
      // A faulted joint holds its measured pose and is excluded from the return to rest (which
      // then cannot be verified); a release_on_fault joint (the gripper) is released rather than
      // held.
      const position = this.getJointPositions()[index]!;
      if (Number.isFinite(position)) this.userTargets[index] = this.internal[index] = position;
      this.faulted.add(index);
      if (this.config.joints[index]!.releaseOnFault) {
        this.released.add(index);
        try {
          await this.motors[index]!.disable();
        } catch (e) {
          this.#log.error(`failed to disable motor ${this.motors[index]!.motorId}: ${errorText(e)}`);
        }
      }
    }
    if (!this.config.holdOnFault) {
      this.running = false;
      this.#tickSignal.notify();
      await this.#disableAll();
      this.#state = ArmState.FAULT;
      return;
    }
    if (this.#state === ArmState.ENABLED) {
      // Hold the measured pose (prevents a drop); the loop keeps sending hold frames. Released by
      // stop()/disable()/estop()/clearFaults().
      this.#state = ArmState.FAULT;
      this.holdCurrentPosition();
      this.#log.error("holding the current pose; stop() returns to rest, disable() releases torque");
    }
  }

  /** Explicit operator action: release torque, clear motor faults and latched evidence. */
  clearFaults(): Promise<void> {
    return this.#lifecycle.run(async () => {
      if (this.#state === ArmState.DISCONNECTED) throw new StateError("clearFaults() requires a connected arm");
      if (this.#state === ArmState.ENABLED || this.#state === ArmState.FAULT) await this.#disable();
      for (const m of this.motors) {
        await m.clearFault();
        await this.#clock.sleep(0.01);
      }
      this.faultReason = null;
      this.faultReasons = [];
      this.#state = ArmState.CONNECTED;
    });
  }
}

function stateName(state: ArmState): string {
  return state.toUpperCase();
}

function hex8(n: number): string {
  return n.toString(16).toUpperCase().padStart(8, "0");
}

function range(n: number): number[] {
  return Array.from({ length: n }, (_, i) => i);
}

/** Python's max() of a non-empty list (the first of equal values wins). */
function maxOf(values: number[]): number {
  let best = values[0]!;
  for (const v of values.slice(1)) best = pyMax(best, v);
  return best;
}
