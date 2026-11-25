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
import { macrotask } from "./cooperate.js";
import { ArmError, ConnectError, FaultLatched, ParamTimeout, RouteUnverified, StateError, ValueError } from "./errors.js";
import { GripperHold } from "./gripper.js";
import type { Logger } from "./logger.js";
import * as gravity from "./gravity.js";
import * as kinematics from "./kinematics.js";
import type { Motor } from "./motor.js";
import * as planner from "./planner.js";
import { ArmState } from "./state.js";
import * as trajectoryIk from "./trajectory-ik.js";
import type { CanBackend } from "./transport/base.js";

/** moveEe: how far the planned path may end from the goal (m). */
export const EE_POSITION_TOL = planner.EE_POSITION_TOL;
/** moveEe: how far the planned path may end from the goal (rad). */
export const EE_ANGLE_TOL = planner.EE_ANGLE_TOL;
/** Shortest moveEe path, s. */
export const EE_MIN_SECONDS = planner.EE_MIN_SECONDS;
/**
 * A late control tick advances the commands by the time that really passed, up to this many
 * periods, so speed limits hold in real time when the computer cannot keep the profile's rate.
 */
export const MAX_TICK_PERIODS = 4;
/**
 * Rest: near the supported rest pose the return presses the weight-carrying joints toward it by at
 * most REST_PRESS (rad) and at most REST_PRESS_NM of proportional effort (offset * kp).
 */
export const REST_PRESS = radians(3.0);
/** N·m; see REST_PRESS. */
export const REST_PRESS_NM = 1.75;
/**
 * Before torque off, stop() stops the control loop and checks one fresh reading per returned
 * joint: answered within REST_FRESH_S (s) and no further from rest than when the return was
 * verified (or than restArrived), give or take REST_RECHECK (rad); otherwise the loop restarts
 * and the arm holds.
 */
export const REST_FRESH_S = 0.05;
/** rad; see REST_FRESH_S. */
export const REST_RECHECK = radians(1.0);
/** A joint that moves the tool point up or down at least this much (m/rad) carries weight. */
export const HEIGHT_JOINT_M_PER_RAD = 0.02;
/**
 * Start-to-start spacing (s) of the frames one tick sends on a real bus: about one command and its
 * reply on a 1 Mbit/s bus. A send that itself takes longer needs no extra wait.
 */
export const FRAME_SPACING_S = 200e-6;
/** The default frame spacing (ArmOptions.frameSpacing), s: FRAME_SPACING_S. */
export const FRAME_SPACING = FRAME_SPACING_S;
/** attach(): below this model holding torque (N·m, over the chain) gravity starts from zero. */
export const ATTACH_LOAD_MIN_NM = 0.1;
/** loopRateHz is measured over this window, s. */
export const LOOP_RATE_WINDOW_S = 1.0;
/** Below this share of controlRateHz for two windows: warn once per session. */
export const SLOW_LOOP_SHARE = 0.9;

export type { Logger } from "./logger.js";

/** Extra feed-forward added while floating: (joint positions, joint velocities) -> N·m per joint. */
export type ExtraTorque = (positions: number[], velocities: number[]) => readonly number[];

export interface ArmOptions {
  /** Default: a RealClock. */
  clock?: Clock;
  /** Default: console. */
  logger?: Logger;
  /** Start-to-start spacing of the frames of a tick, s (default FRAME_SPACING_S; 0 for simulated motors). */
  frameSpacing?: number;
  /**
   * How attach() starts gravity feed-forward. "full" (default): at the share of the model's
   * holding torque the motors are carrying (from their feedback torques), so a joint the previous
   * owner was holding keeps its feed-forward at takeover, then eased in from there; from zero when
   * any chain joint reports torque off or no torque state. "ramp": always eased in from zero over
   * gravity.RAMP_S, as the Python SDK does.
   */
  attachGravity?: "full" | "ramp";
}

export interface SetMotorIdOptions {
  /** s, for the motor to answer on its new id (default 0.5). */
  timeout?: number;
  /** The caller made sure the motor is de-energized (needed where its firmware cannot report torque state). */
  torqueOffConfirmed?: boolean;
  /** The joint (by name) a motor not in the profile replaces: needed to take that joint's id. */
  replacing?: string | null;
}

export interface StopOptions {
  /**
   * The port of a second Ctrl-C during the Python SDK's stop(): aborting it ends the return to
   * rest (or the recheck before the release) where it is; stop() then resolves false with the arm
   * holding its measured pose at full gains, the control loop running.
   */
  signal?: AbortSignal;
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
  /** Measured control-loop rate, Hz (one decimal), null until measured. Always set by Arm.status(). */
  loopRateHz?: number | null;
  /** Motors whose torque release could not be confirmed (Arm.unconfirmedDisable). Always set by Arm.status(). */
  unconfirmedDisable?: number[];
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
  /** Measured control-loop rate, Hz; null until measured. */
  loopRateHz: number | null = null;

  #backend: CanBackend;
  #clock: Clock;
  #log: Logger;
  #frameSpacing: number;
  #attachGravity: "full" | "ramp";
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
  /**
   * Motion ownership: bumped by every new target, stop, estop, disable, fault, return to rest and
   * new session. A moveEe commits its rows only while the value it took is still current.
   */
  private motion = 0;
  /** Joint index -> why its id no longer surely reaches its motor (a set-id that left both ids answering). */
  private unverified = new Map<number, string>();
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
  /** Per motor, when the oldest command still unanswered went out; null: nothing outstanding. */
  private awaiting: Array<number | null>;
  /** Per motor, when the oldest status query still unanswered went out; null: nothing outstanding. */
  private awaitingStatus: Array<number | null>;
  /** [start, tick count at start] of the loop-rate window. */
  private rateWindow: [number, number] = [0.0, 0];
  private slowWindows = 0;
  /** How moveEe's planning gives the event loop a turn. */
  private planPause: () => Promise<void> = macrotask;
  /**
   * Frame replay only: send a frame computed before a hold even when the hold superseded it, as
   * the Python SDK does (see "Intended differences" in docs/porting.md).
   */
  private pythonFrames = false;
  /** moveEe's planner (replaceable in tests). */
  private planFn: typeof planner.plan = planner.plan;
  /** E-stops whose confirmed shutdown has not finished; enable() and attach() refuse meanwhile. */
  #estopPending = 0;
  /** The session stop()'s own recheck tick may send in (null: none). */
  #manualSession: number | null = null;
  /** Per motor: commands the loop sent and feedback sequence, at the last baseline (see #outstanding). */
  #commandBase: Array<[number, number]> = [];
  /** Per motor: commands the loop has sent. */
  #commands: number[] = [];
  /** The slow-loop warning went out this session. */
  #slowWarned = false;
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
    this.#attachGravity = options.attachGravity ?? "full";
    if (this.#attachGravity !== "full" && this.#attachGravity !== "ramp") {
      throw new ValueError(`attachGravity must be "full" or "ramp", got ${JSON.stringify(options.attachGravity)}`);
    }
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
    this.awaiting = new Array<number | null>(n).fill(null);
    this.awaitingStatus = new Array<number | null>(n).fill(null);
    this.#commands = new Array<number>(n).fill(0);
    this.#commandBase = this.motors.map((m) => [0, m.feedbackSequence]);
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
    const rate = this.loopRateHz;
    return {
      state: this.#state, faultReason: this.faultReason, loopRateHz: rate === null ? null : pyRound(rate, 1),
      unconfirmedDisable: [...this.unconfirmedDisable], joints,
    };
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
      if (this.#estopPending) throw new StateError("an e-stop is in progress; enable() once it has finished");
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
      this.#refuseUnverifiedRoutes();
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
      this.restPose = this.#restTarget(pos);
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
      if (this.#estopPending) throw new StateError("an e-stop is in progress; attach() once it has finished");
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
      this.#refuseUnverifiedRoutes();
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
      const model = this.gravity;
      if (model !== null && this.#attachGravity === "full") model.level = this.#carriedLoad(model, torque, pos);
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
   * attach()'s starting gravity level: the share of the model's holding torque the motors are
   * carrying now, from their fresh feedback torques (joint sign), dot(measured, model) /
   * dot(model, model) clamped to [0, 1]; the ease-in continues from there. 0 when any chain joint
   * reports torque off or no torque state, or when the model torque is negligible.
   */
  #carriedLoad(model: gravity.GravityFeedforward, torque: Array<boolean | null>, pos: number[]): number {
    const n = model.n;
    if (!torque.slice(0, n).every((t) => t === true)) return 0.0;
    const tau = model.model(pos.slice(0, n)).map((t, i) => pyMax(-model.limitsNm[i]!, pyMin(model.limitsNm[i]!, t)));
    const measured = this.getJointTorques().slice(0, n);
    let gg = 0.0, mg = 0.0;
    for (let i = 0; i < n; i++) {
      if (!Number.isFinite(tau[i]) || !Number.isFinite(measured[i])) return 0.0;
      gg += tau[i]! * tau[i]!;
      mg += measured[i]! * tau[i]!;
    }
    if (Math.sqrt(gg) < ATTACH_LOAD_MIN_NM) return 0.0;
    return pyMin(1.0, pyMax(0.0, mg / gg));
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
    this.motion += 1;
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
    // enable()/attach() asked every motor already; the loop's first frames follow at once.
    this.awaiting = new Array<number | null>(this.motors.length).fill(now);
    this.awaitingStatus = new Array<number | null>(this.motors.length).fill(null);
    this.loopRateHz = null;
    this.rateWindow = [now, this.tickCount];
    this.slowWindows = 0;
    this.#slowWarned = false;
    this.#baselineReplies();
    this.#state = ArmState.ENABLED;
    if (startLoop) this.#startLoop();
  }

  #startLoop(): void {
    // The loop rate is measured from here: a restart after stop() does not count the downtime.
    this.rateWindow = [this.#clock.now(), this.tickCount];
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
  #joinLoop(handle: LoopHandle, timeout: number, signal?: AbortSignal): Promise<boolean> {
    if (!handle.alive) return Promise.resolve(true);
    return new Promise<boolean>((resolve) => {
      const onAbort = (): void => resolve(!handle.alive);
      const cancel = this.#clock.timer(timeout, () => resolve(!handle.alive));
      signal?.addEventListener("abort", onAbort, { once: true });
      void handle.promise.then(() => {
        cancel();
        signal?.removeEventListener("abort", onAbort);
        resolve(true);
      });
    });
  }

  /** `signal`: an abort stops waiting for the loop to end (it still ends: its session is over). */
  async #stopLoop(signal?: AbortSignal): Promise<void> {
    this.running = false;
    this.session += 1; // a tick still in flight stops sending at its next command
    this.#tickSignal.notify();
    const handle = this.loop;
    if (handle) {
      if (!(await this.#joinLoop(handle, 1.0, signal)) && !signal?.aborted) {
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
    this.#endMotion();
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
    this.#estopPending += 1;
    try {
      this.#log.warn("e-stop: releasing torque on all motors");
      this.#endMotion();
      this.generation += 1;
      this.running = false;
      this.session += 1;
      this.#manualSession = null;
      this.#tickSignal.notify();
      // An enable frame being confirmed (and its retries) goes out first, never after the torque-off.
      await this.#enableGate.run(() => undefined);
      if (this.#state !== ArmState.DISCONNECTED) await this.#disableAll();
      await this.disable();
    } finally {
      this.#estopPending -= 1;
    }
  }

  /**
   * Return to the rest pose, then release torque only if the return was verified.
   *
   * An arm without brakes must never be released away from rest. After a verified return the
   * control loop stops and each returned joint, and each weight-carrying joint left out of the
   * return, is read once more; torque goes off only if every reading answers that read, is fresh,
   * no motor reports a fault and no joint moved away from rest. Resolves true only once every motor
   * confirmed torque off. Otherwise, and on any other way out (an error, aborting
   * `options.signal` before the first torque-off frame), the arm keeps holding its measured pose at
   * the profile's gains with the control loop running and false is returned; the caller decides
   * when to disable(). When the release was sent but some motor did not confirm it, false is
   * returned with the arm CONNECTED, the control loop stopped and those motors listed in
   * `unconfirmedDisable` (and status()); disable() tries the release again.
   */
  stop(options: StopOptions = {}): Promise<boolean> {
    const { signal } = options;
    return this.#lifecycle.run(async () => {
      if (this.#state !== ArmState.ENABLED && this.#state !== ArmState.FAULT) return true;
      let released = false;
      // An abort wakes every wait of the return and invalidates stop()'s own recheck tick.
      const wake = (): void => {
        this.#manualSession = null;
        this.#tickSignal.notify();
        for (const m of this.motors) m.changed.notify();
      };
      signal?.addEventListener("abort", wake);
      try {
        const include = this.#returnedJoints();
        if (include === null) return false;
        if (!(await this.#returnToRest(null, signal))) return false;
        released = await this.#releaseIfStill(include, this.restPose!, signal);
        return released && this.unconfirmedDisable.length === 0;
      } catch (e) {
        this.#log.error(`return to rest failed: ${errorText(e)}`);
        return false;
      } finally {
        signal?.removeEventListener("abort", wake);
        this.#manualSession = null;
        this.returning = false;
        // An e-stop in progress owns the bus: nothing here may send another holding frame.
        if (!released && !this.#estopPending && (this.#state === ArmState.ENABLED || this.#state === ArmState.FAULT)) {
          this.#holdAtProfileGains();
          if (!this.running) this.#startLoop();
        }
      }
    });
  }

  /**
   * Hold the measured pose with the profile's gains: float mode, gains set by setGains() and
   * float mode's extra torque end first, so the hold is serviced at full stiffness.
   */
  #holdAtProfileGains(): void {
    this.config.joints = this.config.joints.map((j, i) => j.replace({ kp: this.profileGains[i]![0], kd: this.profileGains[i]![1] }));
    this.extraTorque = null;
    this.floating = false;
    this.holdCurrentPosition();
  }

  /**
   * The joints the return brings back, or null when stop() must not release: none would be
   * brought back, or a joint that carries the arm's weight is left out and stands away from the
   * supported rest pose.
   */
  #returnedJoints(): number[] | null {
    const cfg = this.config;
    const include = range(cfg.nJoints).filter((i) => cfg.joints[i]!.returnToRest && !this.released.has(i));
    if (!include.length) {
      this.#log.error("no joint returns to rest; the arm keeps holding");
      return null;
    }
    const supported = this.#supportedRest();
    const pos = this.getJointPositions();
    for (const i of this.#excludedWeight(include, supported)) {
      if (!Number.isFinite(pos[i]) || Math.abs(pos[i]! - supported[i]!) > cfg.restSettle) {
        this.#log.error(`${cfg.joints[i]!.label} carries the arm's weight, is away from rest and does not return; the arm keeps holding`);
        return null;
      }
    }
    return include;
  }

  /** The weight-carrying joints the return leaves out. */
  #excludedWeight(include: number[], supported: readonly number[]): number[] {
    return this.#weightJoints(supported).filter((i) => !include.includes(i));
  }

  /** Start counting each motor's replies against the loop's commands from now on. */
  #baselineReplies(): void {
    this.#commandBase = this.motors.map((m, i) => [this.#commands[i]!, m.feedbackSequence]);
  }

  /**
   * Commands the loop sent to motor `i` since the baseline that have no reply yet (a motor answers
   * each command once, in order). Extra replies (a Damiao status refresh is answered with a state
   * frame) count as answers, so one of them can stand in for one lost reply.
   */
  #outstanding(i: number): number {
    const [commands, sequence] = this.#commandBase[i]!;
    return Math.max(0, (this.#commands[i]! - commands) - (this.motors[i]!.feedbackSequence - sequence));
  }

  /**
   * Stop the loop, read every checked motor once (the holding frame the loop would send) and
   * release torque only if each answered that read within the freshness bound, no motor reports a
   * fault, each returned joint is no further from `rest` than when the return was verified (or than
   * restArrived), give or take REST_RECHECK, and each weight-carrying joint left out of the return
   * is still within restSettle of the supported pose.
   *
   * The reading counted for a motor answers a command sent after the loop stopped: replies are
   * counted against the session's commands; once the loop's replies have drained (up to
   * REPLY_TIMEOUT), a motor still owing one (lost, or held back until the next frame) is read once
   * more before the recheck, so a reply up to one frame late cannot pass for the recheck's.
   * The point of no return is the first torque-off frame: an abort before it holds, after it the
   * release is finished and its real outcome reported.
   */
  async #releaseIfStill(include: number[], rest: readonly number[], signal?: AbortSignal): Promise<boolean> {
    const cfg = this.config;
    const supported = this.#supportedRest();
    const excluded = this.#excludedWeight(include, supported);
    const checked = [...include, ...excluded];
    const verified = this.getJointPositions();
    const stopped = (): boolean => Boolean(signal?.aborted) || this.#estopPending > 0;
    this.#endMotion();
    await this.#stopLoop(signal);
    if (stopped()) return this.#keepHolding("stop() was interrupted");
    if (this.loop !== null) return this.#keepHolding("the control loop did not stop");
    // Replies to the loop's commands drain first (up to REPLY_TIMEOUT). A motor still owing one (a
    // reply lost, or held back in a buffer until the next frame) gets one extra read first, so the
    // reply the recheck counts answers a command sent after the loop stopped.
    const drainUntil = this.#clock.now() + this.REPLY_TIMEOUT;
    for (const i of checked) {
      const m = this.motors[i]!;
      const left = drainUntil - this.#clock.now();
      if (this.#outstanding(i) > 0 && left > 0) {
        await m.changed.wait(() => this.#outstanding(i) === 0 || stopped(), left, this.#clock);
      }
      if (stopped()) return this.#keepHolding("stop() was interrupted");
    }
    const read = async (): Promise<boolean> => {
      this.#manualSession = this.session;
      await this.tick(1.0 / cfg.controlRateHz, this.session);
      const manual = this.#manualSession;
      this.#manualSession = null;
      return !stopped() && manual !== null;
    };
    if (checked.some((i) => this.#outstanding(i) > 0) && !(await read())) return this.#keepHolding("stop() was interrupted");
    const sequences = this.motors.map((m) => m.feedbackSequence);
    const commands = [...this.#commands];
    if (!(await read())) return this.#keepHolding("stop() was interrupted");
    const fresh = pyMin(cfg.feedbackTimeout, REST_FRESH_S);
    const answered = (i: number): boolean => this.#commands[i]! > commands[i]!
      && this.motors[i]!.feedbackSequence - sequences[i]! >= this.#commands[i]! - commands[i]!;
    const deadline = this.#clock.now() + fresh;
    for (const i of checked) {
      const m = this.motors[i]!;
      const left = deadline - this.#clock.now();
      if (left <= 0) break;
      await m.changed.wait(() => answered(i) || stopped(), left, this.#clock);
      if (stopped()) return this.#keepHolding("stop() was interrupted");
    }
    // Faults a reply latched during the waits count too: judged right before the release.
    await this.#checkHealth();
    const now = this.getJointPositions();
    let problem: string | null = null;
    const faulty = this.motors.findIndex((m) => m.faultLatched);
    if (this.#state !== ArmState.ENABLED) {
      problem = `state ${stateName(this.#state)}`;
    } else if (faulty >= 0) {
      problem = `${cfg.joints[faulty]!.label} reports a fault (${this.motors[faulty]!.faultLatched})`;
    } else if (!checked.every((i) => answered(i) && this.motors[i]!.feedbackAge <= fresh)) {
      problem = "no fresh reply";
    } else {
      for (const i of include) {
        if (!(Number.isFinite(now[i]) && Math.abs(now[i]! - rest[i]!) <= pyMax(Math.abs(verified[i]! - rest[i]!), cfg.restArrived) + REST_RECHECK)) {
          problem = `${cfg.joints[i]!.label} is ${degrees(Math.abs(now[i]! - rest[i]!)).toFixed(1)} deg from rest, `
            + `${degrees(Math.abs(verified[i]! - rest[i]!)).toFixed(1)} when verified`;
          break;
        }
      }
      for (const i of excluded) {
        if (problem !== null) break;
        if (!(Number.isFinite(now[i]) && Math.abs(now[i]! - supported[i]!) <= cfg.restSettle)) {
          problem = `${cfg.joints[i]!.label} carries the arm's weight, does not return and is `
            + `${degrees(Math.abs(now[i]! - supported[i]!)).toFixed(1)} deg from the supported pose`;
        }
      }
    }
    if (problem === null && stopped()) problem = "stop() was interrupted";
    if (problem !== null) return this.#keepHolding(problem);
    // The point of no return: from the first torque-off frame on the release is finished.
    await this.#releaseAll();
    this.released.clear();
    this.faulted.clear();
    this.#state = ArmState.CONNECTED;
    if (this.unconfirmedDisable.length) {
      this.#log.error(`stop(): torque release not confirmed on motors [${this.unconfirmedDisable.join(", ")}]; `
        + "the control loop is stopped; disable() tries again, or cut motor power");
    }
    return true;
  }

  #keepHolding(problem: string): boolean {
    this.#log.error(`after the verified return to rest: ${problem}; the arm keeps holding`);
    return false;
  }

  /** The profile's restPose, else zero, clipped into the soft limits. */
  #supportedRest(): number[] {
    const cfg = this.config;
    const base = cfg.restPose ?? new Array<number>(cfg.nJoints).fill(0.0);
    return cfg.joints.map((j, i) => pyMin(j.hi - cfg.limitMargin, pyMax(j.lo + cfg.limitMargin, base[i]!)));
  }

  /**
   * The joints that move the tool point up or down at the supported rest pose (they carry the
   * arm's weight); none without a URDF.
   */
  #weightJoints(supported: readonly number[]): number[] {
    let chain: kinematics.Chain;
    try {
      chain = this.chain;
    } catch (e) {
      if (e instanceof ArmError) return [];
      throw e;
    }
    const lift = chain.jacobian(supported.slice(0, chain.n))[2]!;
    return range(chain.n).filter((i) => Math.abs(lift[i]!) >= HEIGHT_JOINT_M_PER_RAD);
  }

  /**
   * The pose stop() returns to for an arm enabled at `pos`: `pos` itself, except near the supported
   * rest pose with restWindow set: there each joint that carries the arm's weight is pressed toward
   * it by at most REST_PRESS and at most REST_PRESS_NM of proportional effort, inside the soft
   * limits, so a return that stops a little short cannot leave the arm, session after session,
   * further from its support.
   */
  #restTarget(pos: readonly number[]): number[] {
    const cfg = this.config;
    if (cfg.restWindow === null) return [...pos];
    const window = cfg.restWindow;
    const supported = this.#supportedRest();
    const carrying = this.#weightJoints(supported).filter((i) => cfg.joints[i]!.returnToRest && cfg.joints[i]!.kp > 0);
    if (!carrying.length || carrying.some((i) => !Number.isFinite(pos[i]) || Math.abs(pos[i]! - supported[i]!) > window)) {
      return [...pos];
    }
    const target = [...pos];
    for (const i of carrying) {
      const j = cfg.joints[i]!;
      const lo = j.lo + cfg.limitMargin, hi = j.hi - cfg.limitMargin;
      // Enabled outside the soft limits (inside fullTurnGrace): no press; the loop's clip to the
      // limits already moves it, and clamping a pressed target there could exceed the press bound.
      if (pos[i]! < lo || pos[i]! > hi) continue;
      const press = pyMin(REST_PRESS, REST_PRESS_NM / j.kp);
      const moved = pos[i]! + pyMax(-press, pyMin(press, supported[i]! - pos[i]!));
      target[i] = pyMin(hi, pyMax(lo, moved));
    }
    return target;
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
   * answers, or if it belongs to a joint of the profile (even one that is offline) unless
   * `replacing` names that joint and `oldId` is no joint's id: a motor replacing a joint, or one
   * moved off its joint's id and back. If afterwards both ids answer (RouteUnverified), the joints
   * configured at `oldId` and `newId` are marked unverified and enable()/attach() refuse this Arm
   * (scan the bus, fix the profile, build a new Arm). The motor is read before the write and refused
   * if it reports torque on, holds torque or moves; where its firmware cannot report torque state,
   * `torqueOffConfirmed` says the caller made sure it is de-energized (Motor.setId). A configured
   * joint that had `oldId` is routed to `newId` from then on (refresh() reads it again); update the
   * profile to keep the change.
   */
  async setMotorId(oldId: number, newId: number, options: SetMotorIdOptions = {}): Promise<void> {
    const { timeout = 0.5, torqueOffConfirmed = false, replacing = null } = options;
    const product = this.config.product;
    const [lo, hi] = product.idRange;
    for (const value of [oldId, newId]) {
      if (!(Number.isInteger(value) && lo <= value && value <= hi)) {
        throw new ValueError(`motor IDs must be whole numbers in [${lo}, ${hi}], got ${value}`);
      }
    }
    if (oldId === newId) throw new ValueError("old and new motor IDs are the same");
    return this.#lifecycle.run(async () => {
      if (this.#state !== ArmState.DISCONNECTED && this.#state !== ArmState.CONNECTED) {
        throw new StateError(`setMotorId() requires torque off, currently ${stateName(this.#state)}`);
      }
      // A joint's motor may not take another joint's id, even while that joint does not answer. A
      // motor the profile does not have (a replacement, or one moved off its joint's id) may take a
      // joint's id when nothing answers there.
      const other = this.config.joints.find((j) => j.motorId === newId);
      if (other !== undefined && (this.config.joints.some((j) => j.motorId === oldId) || replacing !== other.name)) {
        throw new ArmError(`motor ID ${newId} belongs to ${other.label} in the profile; pick a free id, or, for a motor that `
          + `is not in the profile and replaces it, name the joint (replacing: '${other.name}'; set-id --replacing ${other.name})`);
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
        try {
          await target.setId(newId, timeout, torqueOffConfirmed);
          // Both routes are probed before the joint is routed to the new id: nothing may answer at the old
          // one any more (whatever the motor family checked itself).
          const stale = product.createMotor(joint, this.#backend, hostId, this.#clock, this.#log);
          temporary.set(oldId, stale);
          let oldAnswers: boolean;
          try {
            await stale.check(this.REPLY_TIMEOUT, this.REPLY_ATTEMPTS);
            oldAnswers = true;
          } catch (e) {
            if (!(e instanceof ParamTimeout)) throw e;
            oldAnswers = false;
          }
          if (oldAnswers) {
            throw new RouteUnverified(`${joint.label}: ID ${oldId} and ID ${newId} both answer after the id change: another `
              + "motor answers on one of them; scan the bus and fix the profile");
          }
        } catch (e) {
          if (e instanceof RouteUnverified) {
            // Either id may now reach the wrong motor: the joints configured at both ends are unverified.
            this.config.joints.forEach((j, i) => {
              if (j.motorId === oldId || j.motorId === newId) this.unverified.set(i, e.message);
            });
          }
          throw e;
        }
        this.#rekey(oldId, newId, target);
      } finally {
        if (opened) await this.#backend.close();
        this.#backend.setRecvCallback(this.#onFrame);
      }
    });
  }

  #refuseUnverifiedRoutes(): void {
    if (this.unverified.size) {
      const names = [...this.unverified.keys()].sort((a, b) => a - b).map((i) => this.config.joints[i]!.label).join(", ");
      throw new ConnectError(`${names}: id unverified after a set-id that left both ids answering; scan the bus, `
        + "fix the profile, and use a new Arm");
    }
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
   * it is solved by the trajectory IK, cooperatively, so the control loop keeps running while it is
   * planned, and streamed through the joint targets, so no tick moves a joint further than its
   * speed allows and a small tool move stays a small joint move. Joints past the chain (the
   * gripper) keep their command. Refused (false, nothing sent, the arm keeps holding its commanded
   * pose) when not ENABLED, when the end pose is out of reach within the soft limits, or when
   * planning does not finish: it is stopped after `timeout` seconds, when `signal` is aborted, or
   * past its own limit of a few seconds beyond the expected solve (planner.ts). A newer target,
   * stop(), estop(), disable(), a fault or a return to rest supersedes the motion, planning or
   * streaming, and none of its rows is sent afterwards. With `wait` it then waits until arrival as
   * moveTo() does (false after `timeout` seconds counted from the end of planning, on a fault or
   * when newer targets take over); without it the path streams in the background. Aborting
   * `signal` stops the motion where it is (holding the measured pose) and resolves false. ArmError
   * without a URDF.
   */
  async moveEe(xyz: readonly number[], options: MoveEeOptions = {}): Promise<boolean> {
    const { rpy, speed, timeout, wait = true, signal } = options;
    const chain = this.chain;
    const target = Array.from(xyz);
    const rpyGoal = rpy === undefined || rpy === null ? null : Array.from(rpy);
    const three = (v: readonly unknown[]): boolean => v.length === 3 && v.every((x) => typeof x === "number" && Number.isFinite(x));
    if (!three(target) || (rpyGoal !== null && !three(rpyGoal))) {
      throw new ValueError("xyz (and rpy) need three finite values");
    }
    if (speed !== undefined && speed !== null && !(Number.isFinite(speed) && speed > 0)) {
      throw new ValueError("speed must be positive");
    }
    // Admission and ownership together, before any setup.
    const token = this.#claimMotion();
    if (token === null) {
      this.#log.warn(`moveEe rejected: state=${stateName(this.#state)} returning=${this.returning ? "True" : "False"}`);
      return false;
    }
    const cfg = this.config;
    const ik = trajectoryIk.TrajectoryIK.fromConfig(cfg, { chain });
    const superseded = (): boolean => this.motion !== token;
    const current = this.getCommandedPositions();
    const q0 = current.slice(0, chain.n);
    const plan = await this.planFn(chain, ik, target, rpyGoal, q0, cfg.controlRateHz, speed ?? null, {
      clock: this.#clock, limit: timeout ?? null, cancel: () => superseded() || Boolean(signal?.aborted), pause: this.planPause,
    });
    if (superseded()) {
      this.#log.warn("moveEe dropped: a newer command, stop, e-stop, disable or fault took over while its path was planned; "
        + "nothing was sent");
      return false;
    }
    if (plan.q === null) {
      this.#log.warn(`moveEe rejected: ${plan.reason}; nothing was sent`);
      return false;
    }
    const rows = plan.q.map((q) => [...q, ...current.slice(chain.n)]);
    if (signal?.aborted) return false;
    const deadline = timeout === undefined || timeout === null ? null : this.#clock.now() + timeout;
    if (!wait) {
      if (!this.#setTargets(rows[0]!, speed ?? null, token)) return false;
      void this.#stream(rows, speed ?? null, signal, deadline, token).catch((e) => this.#log.error(`moveEe stream failed: ${errorText(e)}`));
      return true;
    }
    if (!(await this.#stream(rows, speed ?? null, signal, deadline, token))) return false;
    const remaining = deadline === null ? undefined : Math.max(0.0, deadline - this.#clock.now());
    return this.#moveTo(rows[rows.length - 1]!, speed, remaining, true, signal, token);
  }

  /**
   * Send `rows` as joint targets, one per control period, while motion `token` is current. False
   * when cancelled (holding the measured pose), past `deadline`, refused, or when anything newer
   * takes over.
   */
  async #stream(rows: number[][], speed: number | null, signal: AbortSignal | undefined, deadline: number | null,
                token: number): Promise<boolean> {
    const period = 1.0 / this.config.controlRateHz;
    let due = this.#clock.now();
    for (const row of rows) {
      if (signal?.aborted) {
        this.#holdIfOwner(token);
        return false;
      }
      if (deadline !== null && this.#clock.now() > deadline) return false;
      if (!this.#setTargets(row, speed, token)) return false;
      due += period;
      await this.#clock.sleep(Math.max(0.0, due - this.#clock.now()));
    }
    // A cancel during the last row's period holds too (checked on waking, as after every row).
    if (signal?.aborted) {
      this.#holdIfOwner(token);
      return false;
    }
    return true;
  }

  // ── targets ──
  /** New joint targets (rad, one per joint); false (logged) when refused. Supersedes any motion in flight. */
  setJointTargets(targets: readonly number[]): boolean {
    return this.#setTargets(targets, null);
  }

  /**
   * Set the targets. Without `token` this is a new command and supersedes any motion in flight;
   * with it, the targets are taken only while that motion is still current.
   */
  #setTargets(targets: readonly number[], speed: number | null, token: number | null = null): boolean {
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
    if (token === null) {
      this.motion += 1;
    } else if (token !== this.motion) {
      return false; // superseded: something newer owns the targets
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
  moveTo(q: readonly number[], options: MoveOptions = {}): Promise<boolean> {
    const { speed, timeout, wait = true, signal } = options;
    return this.#moveTo(q, speed, timeout, wait, signal, null);
  }

  /** moveTo(); with `token` it continues that motion (taken only while it is current). */
  async #moveTo(q: readonly number[], speed: number | undefined | null, timeoutArg: number | undefined | null, wait: boolean,
                signal: AbortSignal | undefined, tokenArg: number | null): Promise<boolean> {
    let timeout = timeoutArg;
    const goalQ = Array.from(q); // not converted: null or "" must be refused, not read as 0
    if (speed !== undefined && speed !== null && !(Number.isFinite(speed) && speed > 0)) {
      throw new ValueError("speed must be positive");
    }
    if (signal?.aborted) {
      // Continuing a motion (moveEe's arrival): its cancel holds the measured pose.
      if (tokenArg !== null) this.#holdIfOwner(tokenArg);
      return false;
    }
    const token = tokenArg ?? this.#takeMotion();
    if (!this.#setTargets(goalQ, speed ?? null, token)) return false;
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
        this.#holdIfOwner(token);
        return false;
      }
      if (this.motion !== token || !sameList(this.userTargets, goalQ)) return false;
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

  /** Freeze the arm at its latest measured pose without releasing torque. Supersedes any motion in flight. */
  holdCurrentPosition(): boolean {
    return this.#holdIfOwner(null);
  }

  /** holdCurrentPosition(); with `token`, only while that motion still owns the targets. */
  #holdIfOwner(token: number | null): boolean {
    if (this.#state !== ArmState.ENABLED && this.#state !== ArmState.FAULT) return false;
    const positions = this.getJointPositions();
    if (token !== null && token !== this.motion) return false;
    this.motion += 1;
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
  returnToRest(rest?: readonly number[] | null): Promise<boolean> {
    return this.#lifecycle.run(async () => {
      try {
        return await this.#returnToRest(rest);
      } catch (e) {
        this.#log.error(`return to rest failed: ${errorText(e)}`);
        return false;
      } finally {
        this.returning = false;
      }
    });
  }

  /** `signal` (stop()'s): once aborted the return ends where it is, unverified. */
  async #returnToRest(restArg?: readonly number[] | null, signal?: AbortSignal): Promise<boolean> {
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
    this.motion += 1;
    const token = this.motion;
    this.speed = null;
    positions.forEach((p, i) => {
      if (Number.isFinite(p)) {
        this.userTargets[i] = p;
        this.internal[i] = p;
      }
    });
    const sequences = this.motors.map((m) => m.feedbackSequence);
    if (!(await this.waitTicks(2, 1.0, signal))) {
      if (signal?.aborted) return this.#interrupted();
      this.#log.error("return to rest aborted: control loop is not running");
      return false;
    }
    const include = range(cfg.nJoints).filter((i) => cfg.joints[i]!.returnToRest && !this.released.has(i));
    if (!include.length) return true;
    const start = this.getJointPositions();
    if (!this.#restFeedbackOk(include, start)) return false;
    if (this.motion !== token) return false;
    this.synced = new Array<boolean>(cfg.nJoints).fill(true);
    let worst = maxOf(include.map((i) => Math.abs(start[i]! - rest[i]!)));
    if (worst <= cfg.restArrived && this.#measuredSince(include, sequences) && this.#recent(include)) return true;
    const period = 1.0 / cfg.restRateHz;
    const duration = worst / cfg.restSpeed;
    const began = this.#clock.now();
    const deadline = began + cfg.restTimeout;
    let frac = 0.0;
    while (frac < 1.0) {
      // The ramp follows the clock, so late wake-ups do not slow it below restSpeed.
      frac = pyMin(1.0, pyMax(period, this.#clock.now() - began) / duration);
      if (this.#clock.now() >= deadline || !this.#restActive()) break;
      // an e-stop, disable or new session took over: never write into it
      if (this.motion !== token) return false;
      if (signal?.aborted) return this.#interrupted();
      for (const i of include) this.userTargets[i] = start[i]! + (rest[i]! - start[i]!) * frac;
      await this.#sleep(period, signal);
      if (signal?.aborted) return this.#interrupted();
      if (!this.#restFeedbackOk(include, this.getJointPositions())) return false;
    }
    let best = Infinity, stalled = 0;
    while (this.#clock.now() < deadline && this.#restActive() && this.motion === token) {
      if (signal?.aborted) return this.#interrupted();
      const pos = this.getJointPositions();
      if (!this.#restFeedbackOk(include, pos)) return false;
      worst = maxOf(include.map((i) => Math.abs(pos[i]! - rest[i]!)));
      const fresh = this.#measuredSince(include, sequences) && this.#recent(include);
      if (worst <= cfg.restArrived && fresh) return true;
      if (best - worst < cfg.restStallImprovement) {
        stalled += 1;
        if (stalled >= cfg.restStallPolls && fresh) return worst <= cfg.restSettle;
      } else {
        stalled = 0;
      }
      best = pyMin(best, worst);
      await this.#sleep(period, signal);
    }
    return false;
  }

  /** clock.sleep(seconds), ended early when `signal` is aborted. */
  #sleep(seconds: number, signal?: AbortSignal): Promise<void> {
    if (!signal) return this.#clock.sleep(seconds);
    if (signal.aborted) return Promise.resolve();
    return new Promise<void>((resolve) => {
      const done = (): void => {
        cancel();
        signal.removeEventListener("abort", done);
        resolve();
      };
      const cancel = this.#clock.timer(seconds, done);
      signal.addEventListener("abort", done, { once: true });
    });
  }

  /**
   * True once `n` more control ticks have run; false after `timeout` s, when the loop stops or
   * when `signal` is aborted.
   */
  private async waitTicks(n: number, timeout = 1.0, signal?: AbortSignal): Promise<boolean> {
    const target = this.tickCount + n;
    await this.#tickSignal.wait(() => this.tickCount >= target || !this.running || Boolean(signal?.aborted), timeout, this.#clock);
    return this.tickCount >= target && !signal?.aborted;
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

  /**
   * Is every joint's latest reading younger than feedbackTimeout? Rest is judged (and torque
   * released) only on such readings: a loop that stalled after the motors last answered leaves
   * nothing unanswered, but its readings say nothing about where the arm is now.
   */
  #recent(include: number[]): boolean {
    return include.every((i) => this.motors[i]!.feedbackAge <= this.config.feedbackTimeout);
  }

  #interrupted(): boolean {
    this.#log.error("return to rest interrupted");
    return false;
  }

  #restFeedbackOk(include: number[], positions: number[]): boolean {
    for (const i of include) {
      const label = this.config.joints[i]!.label;
      if (this.faulted.has(i)) {
        this.#log.error(`return to rest aborted: ${label} is faulted`);
        return false;
      }
      if (!Number.isFinite(positions[i]) || this.#replyLate(i) > this.config.feedbackTimeout) {
        this.#log.error(`return to rest aborted: ${label} feedback is stale`);
        return false;
      }
    }
    return true;
  }

  // ── control loop ──
  /** Waits `us` microseconds (the Python SDK busy-waits; a RealClock spins the last stretch of a short sleep). */
  async #busyWaitUs(us: number): Promise<void> {
    await this.#clock.sleep(us / 1e6);
  }

  /** Space the frames so the replies never burst into a small adapter FIFO: FRAME_SPACING_S start to start. */
  async #spaceFrom(lastSend: number | null): Promise<void> {
    if (lastSend === null) return;
    const gap = this.#frameSpacing - (this.#clock.now() - lastSend);
    if (gap > 0) await this.#busyWaitUs(gap * 1e6);
  }

  #skipped(i: number): boolean {
    return this.released.has(i) || this.faulted.has(i);
  }

  #maySend(session: number | null): boolean {
    return session === null || (session === this.session && (this.running || session === this.#manualSession));
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
    const snapshot = this.motion;
    const feedforward = await this.#feedforward(commands, now);
    const paced = this.#backend.feedbackPacedControl;
    const joints = cfg.joints;
    let lastSend: number | null = null;
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
      const tau = feedforward[i]!;
      const sequence = motor.feedbackSequence;
      let sentAt: number;
      try {
        const hold = this.hold.get(i);
        if (hold !== undefined && !this.faulted.has(i) && kp > 0) {
          [position, kp, kd] = this.#gripperCommand(i, j, hold, commands[i]!, now);
        }
        await this.#spaceFrom(lastSend);
        // Waiting for the spacing lets other work run: a shutdown may have ended this session, and
        // a hold or a fault may have replaced the command this frame was computed from.
        if (!this.#maySend(session)) return;
        if (this.motion !== snapshot && !this.hold.has(i) && !this.pythonFrames) position = this.internal[i]!;
        lastSend = this.#clock.now();
        sentAt = this.#clock.now();
        await motor.sendCommand(this.#toMotor(i, position), 0.0, kp, kd, tau);
        this.#commands[i]! += 1;
      } catch (e) {
        if (!this.faulted.has(i)) await this.#enterFault(`${j.label}: command failed (${errorText(e)})`, i);
        continue;
      }
      this.awaiting[i] = this.#stillAwaiting((this.awaiting[i] ?? null), motor.feedbackAge, sentAt);
      if (paced) {
        // Old USB-SLCAN firmware has no outbound USB queue and a three-frame CAN RX FIFO: wait for
        // this reply before commanding the next motor. A timeout does not skip the remaining
        // motors; the feedback watchdog makes the safety decision.
        await this.#waitReply(motor, sequence, this.#backend.feedbackWaitTimeout);
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
        await this.#spaceFrom(lastSend);
        // A shutdown during the spacing wait does not stop this read-only, non-clearing query (the
        // Python SDK sends it too, and the frame replay records it).
        if (m.pollAnsweredByFeedback) this.#commands[i]! += 1;
        // A query that fails to go out counts as unanswered too: if none can be sent, the status
        // deadline still runs out.
        this.awaitingStatus[i] = this.#stillAwaiting((this.awaitingStatus[i] ?? null), m.statusAge, this.#clock.now());
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
   * When the oldest unanswered request to a motor went out, given one more sent at `sentAt`:
   * `since` while no reply has come after it, else `sentAt`. `replyAge`: age of the motor's latest
   * reply (feedbackAge or statusAge).
   */
  #stillAwaiting(since: number | null, replyAge: number, sentAt: number): number {
    if (since === null || this.#clock.now() - replyAge >= since) return sentAt;
    return since;
  }

  /**
   * How long the oldest request sent at `since` has gone unanswered (0 when answered or none is
   * out). Measured from the request, not from the last reply, so a control loop that could not run
   * for a while (a stalled computer) is not taken for a silent motor.
   */
  #late(since: number | null, replyAge: number): number {
    const now = this.#clock.now();
    if (since === null || now - replyAge >= since) return 0.0;
    return now - since;
  }

  /** Seconds the oldest command to motor `i` has gone without a reply (outside a session: its feedback age). */
  #replyLate(i: number): number {
    const m = this.motors[i]!;
    const since = this.awaiting[i] ?? null;
    return since === null ? m.feedbackAge : this.#late(since, m.feedbackAge);
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
    let last = nextT;
    while (this.#maySend(session)) {
      // Commands advance by the time that really passed since the previous tick (at most a few
      // periods), so a computer that cannot keep the profile's rate still moves the arm at the
      // commanded speed, and the early tick that follows a late one is credited only its shorter
      // interval.
      const started = this.#clock.now();
      const elapsed = pyMin(pyMax(0.0, started - last), MAX_TICK_PERIODS * dt);
      last = started;
      try {
        await this.tick(elapsed, session);
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
      this.#measureRate();
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

  /** Update loopRateHz once per LOOP_RATE_WINDOW_S; warn once per session when the loop is slow. */
  #measureRate(): void {
    const now = this.#clock.now();
    const [start, ticks] = this.rateWindow;
    if (now - start < LOOP_RATE_WINDOW_S) return;
    const rate = (this.tickCount - ticks) / (now - start);
    this.loopRateHz = rate;
    this.rateWindow = [now, this.tickCount];
    const target = this.config.controlRateHz;
    if (rate < SLOW_LOOP_SHARE * target) {
      this.slowWindows += 1;
      if (this.slowWindows >= 2 && !this.#slowWarned) {
        this.#slowWarned = true;
        this.#log.warn(`the control loop runs at ${rate.toFixed(0)} Hz, below the profile's ${target.toFixed(0)} Hz: this computer `
          + "cannot keep up. Speed limits still hold (each tick moves by the time that passed); fewer programs, or a faster "
          + "computer, restore the rate");
      }
    } else {
      this.slowWindows = 0;
    }
  }

  /** Evaluated every tick, in FAULT too: a second motor can still fail while holding. */
  async #checkHealth(): Promise<void> {
    if (this.#state !== ArmState.ENABLED && this.#state !== ArmState.FAULT) return;
    const cfg = this.config;
    const statusDeadline = 2 * cfg.faultPollInterval + cfg.feedbackTimeout;
    // Silence counts from the oldest unanswered command or status query, not from the last reply:
    // a loop that could not run (or poll) for a while has not heard from the motors because it did
    // not ask.
    const joints = cfg.joints;
    for (const [i, m] of this.motors.entries()) {
      const j = joints[i]!;
      if (this.#skipped(i)) continue;
      let reason: string | null;
      const fb = m.feedback;
      const late = this.#late((this.awaiting[i] ?? null), m.feedbackAge);
      const statusLate = this.#late((this.awaitingStatus[i] ?? null), m.statusAge);
      if (late > cfg.feedbackTimeout) {
        reason = `feedback timeout ${late.toFixed(2)}s -- check bus/power`;
      } else if (m.faultLatched) {
        reason = `motor fault -- ${m.faultLatched}`;
      } else if (statusLate > statusDeadline) {
        reason = `no fault-status reply for ${statusLate.toFixed(2)}s`;
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

  /** Invalidate any motion in flight (a moveEe being planned or streamed). */
  #endMotion(): void {
    this.motion += 1;
  }

  /** Start a new motion, superseding any in flight; returns its token. */
  #takeMotion(): number {
    this.motion += 1;
    return this.motion;
  }

  /** #takeMotion() if the arm takes targets now (ENABLED, not returning to rest), else null. */
  #claimMotion(): number | null {
    if (this.#state !== ArmState.ENABLED || this.returning) return null;
    this.motion += 1;
    return this.motion;
  }

  async #enterFault(reason: string, index: number | null = null): Promise<void> {
    this.#log.error(`FAULT: ${reason}`);
    this.#endMotion();
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

/** Python's round(x, digits) (half to even). */
function pyRound(x: number, digits: number): number {
  const m = 10 ** digits;
  const y = x * m;
  const f = Math.floor(y);
  const d = y - f;
  const r = d > 0.5 ? f + 1 : d < 0.5 ? f : (f % 2 === 0 ? f : f + 1);
  return r / m;
}
