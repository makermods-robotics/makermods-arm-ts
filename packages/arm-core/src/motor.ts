/**
 * The Motor seam: what the arm layer needs from one motor, whatever its protocol. Each arm
 * package implements it (its protocol, routing, status and watchdog handling) and hands the
 * implementation to the core through its Product.
 *
 * Positions, velocities and torques here are in motor coordinates (rad, rad/s, N·m). Methods that
 * put a frame on the bus or wait for a reply return a Promise.
 */

import type { Clock, Signal } from "./clock.js";
import type { JointConfig, RawProfile } from "./config.js";
import type { Logger } from "./logger.js";
import type { StarMapping } from "./mapping.js";
import type { CanBackend } from "./transport/base.js";

/** One feedback sample. Arm packages add their own fields (mode bits, status nibble). */
export interface MotorFeedback {
  readonly motorId: number;
  readonly position: number;
  readonly velocity: number;
  readonly torque: number;
  /** °C, hottest sensor. */
  readonly temperature: number;
  readonly temperatures: Readonly<Record<string, number>>;
  /** null: the firmware does not report torque state. */
  readonly enabled: boolean | null;
  readonly warning: boolean;
}

export interface MotorStatus {
  readonly motorId: number;
  readonly hasFault: boolean;
  readonly code: number;
  /** "" when healthy. */
  readonly text: string;
  readonly enabled: boolean | null;
}

export interface ParamSpec {
  readonly index: number;
  readonly dtype: string;
  readonly doc: string;
}

export interface Motor {
  readonly joint: JointConfig;
  motorId: number;
  /** CAN ID the motor replies on, where the protocol has one. */
  readonly replyId?: number | null;
  /** Largest torque a command frame can carry for this model, N·m. */
  readonly torqueLimit: number;
  /** Named parameters readParam/writeParam accept. */
  readonly params: Readonly<Record<string, ParamSpec>>;
  /** Notified after every frame this motor accepted. */
  readonly changed: Signal;

  // Receive path, called by whoever owns the backend's receive callback.
  handleFrame(canId: number, data: Uint8Array, extended?: boolean): void;

  // Cached state.
  readonly feedback: MotorFeedback | null;
  /** Seconds since the last feedback, Infinity before the first. */
  readonly feedbackAge: number;
  /** Count that tells a fresh reply from cached feedback. */
  readonly feedbackSequence: number;
  readonly status: MotorStatus | null;
  readonly statusAge: number;
  /** Why a fault was latched, or null. */
  readonly faultLatched: string | null;
  /** Does the motor report only the state its own watchdog leaves after commands stopped? */
  readonly watchdogReleased: boolean;
  resetCache(): void;

  // Commands: the frame is written, no reply is awaited.
  enable(): Promise<void>;
  disable(): Promise<void>;
  sendCommand(pos: number, vel: number, kp: number, kd: number, tau: number): Promise<void>;
  /** Non-clearing fault/status query; the reply updates `status`. */
  pollStatus(): Promise<void>;
  /** Is pollStatus() answered with a feedback frame (one that advances `feedbackSequence`)? */
  readonly pollAnsweredByFeedback?: boolean;

  // Queries: wait for the reply.
  /** Zero-gain command at `pos`; true once the motor replies. */
  preload(pos: number, timeout?: number): Promise<boolean>;
  /** Enable; true once a reply confirms torque on. */
  enableConfirmed(timeout?: number): Promise<boolean>;
  disableConfirmed(timeout?: number, attempts?: number): Promise<boolean>;
  /** Fresh non-clearing fault query. Never changes torque. Throws ParamTimeout. */
  check(timeout?: number, attempts?: number): Promise<MotorStatus>;
  /** Fresh feedback, not cached. Throws ParamTimeout. */
  readState(timeout?: number, attempts?: number): Promise<MotorFeedback>;
  /** Connect-time check of the motor's configuration. */
  validate(timeout?: number): Promise<void>;
  /** Does the newest reply confirm "enabled" / "disabled"? */
  confirms(expected: "enabled" | "disabled"): boolean;
  /** A reason to stop that only this protocol can see, or null. */
  healthProblem(): string | null;
  clearFault(): Promise<void>;
  setZero(): Promise<void>;
  /**
   * Change the motor's CAN ID (persistent). Refused unless a fresh read shows the motor
   * de-energized; where the firmware cannot report torque state, only with `torqueOffConfirmed`
   * from a caller who made sure it is. Refused when anything answers on `newId`.
   */
  setId(newId: number, timeout?: number, torqueOffConfirmed?: boolean): Promise<void>;
  /** Arm the motor-side watchdog; the token restores the previous setting (null: nothing to restore). */
  armWatchdog(timeoutMs: number, timeout?: number): Promise<unknown>;
  restoreWatchdog(token: unknown, timeout?: number): Promise<void>;
  readParam(nameOrIndex: string | number, dtype?: string, timeout?: number): Promise<number>;
  /** Resolves with the value read back after the write. */
  writeParam(nameOrIndex: string | number, value: number, dtype?: string, timeout?: number): Promise<number>;
}

/**
 * What an arm package supplies to the core: its identity, protocol limits, motor implementation
 * and shipped data. The core never names a product; it reads everything from here.
 */
export interface Product {
  /** npm package, e.g. "@makermods/maker-arm". */
  readonly packageName: string;
  readonly displayName: string;
  readonly motorVendor: string;
  /** The pose the motors' zero is set at, in words. */
  readonly zeroPose: string;

  readonly models: readonly string[];
  readonly defaultModel: string;
  readonly kpRange: readonly [number, number];
  readonly kdRange: readonly [number, number];
  readonly idRange: readonly [number, number];
  readonly defaultHostId: number;

  /** Motor id a received frame belongs to, or null. */
  route(canId: number, data: Uint8Array, hostId: number, extended: boolean): number | null;
  /** `logger` is the arm's (default: console). */
  createMotor(joint: JointConfig, backend: CanBackend, hostId: number, clock: Clock, logger?: Logger): Motor;

  /** The shipped profile as written (snake_case keys, `_deg` forms), with `urdf` holding the URDF text. */
  readonly defaultProfile: RawProfile;
  /** The shipped Star Arm 102 leader mapping onto this arm (the mapping JSON, snake_case keys). */
  readonly defaultStarMapping: StarMapping;

  /** dream_ik embodiment of this arm and its joints from this package's: embodiment = joint * sign + offset. */
  readonly dreamEmbodiment: string;
  readonly dreamJointSign: readonly number[];
  readonly dreamJointOffset: readonly number[];
}
