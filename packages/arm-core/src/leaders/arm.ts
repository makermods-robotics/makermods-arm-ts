/**
 * A second arm of the same kind as a back-drivable leader.
 *
 * The leader arm is enabled and put in `floatMode()`: kp 0, a little damping, and gravity
 * feed-forward, so it holds its own weight while a hand moves it. Optional friction compensation
 * (per joint, from the profile) is added through `floatMode(kd, extraTorque)`: a viscous term on
 * the measured velocity (clamped), a Coulomb term in the direction of travel, and a constant
 * breakaway torque at rest. `read()` returns its joint positions 1:1, clipped to the follower's
 * limits.
 *
 * Profile fields (per joint, all optional, 0 = off): `friction_viscous` (N·m·s/rad),
 * `friction_coulomb` (N·m), `friction_rest_nm` (N·m), `friction_velocity_clamp` (rad/s); arm level
 * `friction_deadzone` (rad/s, below it a joint counts as still).
 */

import type { ExtraTorque } from "../arm.js";
import { ValueError } from "../errors.js";
import { type Leader, LeaderError } from "./base.js";

/** rad/s: motor velocity noise must not flip the Coulomb term at rest. */
export const DEFAULT_DEADZONE = 0.05;

/** The joint fields Friction.fromConfig reads (JointConfig has them all). */
export interface FrictionJoint {
  readonly frictionViscous?: number | null;
  readonly frictionCoulomb?: number | null;
  readonly frictionRestNm?: number | null;
  readonly frictionVelocityClamp?: number | null;
}

/** The config fields Friction.fromConfig reads (ArmConfig has them all). */
export interface FrictionConfig {
  readonly joints: readonly FrictionJoint[];
  readonly frictionDeadzone?: number | null;
}

const pyMin = (a: number, b: number): number => (b < a ? b : a);
const pyMax = (a: number, b: number): number => (b > a ? b : a);
/** math.copysign. */
const copysign = (x: number, y: number): number => (y < 0 || Object.is(y, -0) ? -Math.abs(x) : Math.abs(x));
/** Python's `float(value or default)`: a missing or zero value takes the default. */
const orDefault = (v: number | null | undefined, fallback: number): number =>
  v === undefined || v === null || v === 0 ? fallback : Number(v);

/** Per-joint friction feed-forward: torque(velocity) in joint coordinates. */
export class Friction {
  readonly viscous: readonly number[];
  readonly coulomb: readonly number[];
  readonly rest: readonly number[];
  /** Infinity = no clamp. */
  readonly clamp: readonly number[];
  readonly deadzone: number;

  constructor(viscous: readonly number[], coulomb: readonly number[], rest: readonly number[], clamp: readonly number[],
              deadzone = DEFAULT_DEADZONE) {
    this.viscous = Object.freeze([...viscous]);
    this.coulomb = Object.freeze([...coulomb]);
    this.rest = Object.freeze([...rest]);
    this.clamp = Object.freeze([...clamp]);
    this.deadzone = deadzone;
    Object.freeze(this);
  }

  /** The profile's friction terms, or null when every viscous, Coulomb and rest term is 0. */
  static fromConfig(config: FrictionConfig): Friction | null {
    const joints = [...config.joints];
    const field = (name: keyof FrictionJoint, fallback = 0.0): number[] => {
      const values = joints.map((j) => orDefault(j[name], fallback));
      if (!values.every((v) => Number.isFinite(v) || (name === "frictionVelocityClamp" && v === Infinity))) {
        throw new ValueError(`${name} must be finite`);
      }
      return values;
    };
    const friction = new Friction(field("frictionViscous"), field("frictionCoulomb"), field("frictionRestNm"),
      field("frictionVelocityClamp", Infinity), orDefault(config.frictionDeadzone, DEFAULT_DEADZONE));
    return [...friction.viscous, ...friction.coulomb, ...friction.rest].some((v) => v !== 0) ? friction : null;
  }

  /** N·m per joint for the measured velocities (an ExtraTorque for Arm.floatMode). */
  readonly torques: ExtraTorque = (_positions: number[], velocities: number[]): number[] => {
    const out: number[] = [];
    velocities.forEach((v, i) => {
      if (!Number.isFinite(v)) {
        out.push(0.0);
      } else if (Math.abs(v) < this.deadzone) {
        out.push(this.rest[i]!);
      } else {
        const limit = this.clamp[i]!;
        out.push(this.viscous[i]! * pyMax(-limit, pyMin(limit, v)) + copysign(this.coulomb[i]!, v));
      }
    });
    return out;
  };
}

/** What ArmLeader uses of an Arm. */
export interface LeaderArm {
  readonly state: string;
  readonly config: FrictionConfig & { readonly joints: readonly (FrictionJoint & { readonly lo: number; readonly hi: number })[] };
  readonly faultReason?: string | null;
  connect(): Promise<void>;
  enable(options?: { signal?: AbortSignal }): Promise<void>;
  floatMode(kd?: number | null, extraTorque?: ExtraTorque | null): void;
  getJointPositions(): number[];
  holdCurrentPosition(): boolean;
  setGains(): void;
}

/** What ArmLeader uses of the follower: its joint limits. */
export interface LimitsSource {
  readonly config: { readonly joints: readonly { readonly lo: number; readonly hi: number }[] };
}

const stateOf = (arm: { readonly state: unknown }): string => String(arm.state).toUpperCase();
const pyStr = (v: unknown): string => (v === null || v === undefined ? "None" : String(v));

/** A second arm, floated, as the leader. */
export class ArmLeader implements Leader {
  readonly name = "arm";
  readonly arm: LeaderArm;
  readonly lower: number[];
  readonly upper: number[];
  readonly friction: Friction | null;

  /**
   * `follower` (an Arm) supplies the limits `read()` clips to; default: the leader's own.
   * `friction` overrides the leader profile's friction terms. Throws ValueError when the follower
   * has another number of joints.
   */
  constructor(arm: LeaderArm, follower?: LimitsSource | null, friction?: Friction | null) {
    this.arm = arm;
    const limitsFrom = follower ?? arm;
    this.lower = limitsFrom.config.joints.map((j) => j.lo);
    this.upper = limitsFrom.config.joints.map((j) => j.hi);
    if (this.lower.length !== arm.config.joints.length) {
      throw new ValueError(`leader has ${arm.config.joints.length} joints, follower ${this.lower.length}`);
    }
    this.friction = friction ?? Friction.fromConfig(arm.config);
  }

  /** Enable (`signal` aborts the enable, as in Arm.enable) and float the leader arm. */
  async connect(signal?: AbortSignal): Promise<void> {
    if (stateOf(this.arm) === "DISCONNECTED") await this.arm.connect();
    if (stateOf(this.arm) !== "ENABLED") await this.arm.enable({ signal });
    if (this.friction !== null) {
      this.arm.floatMode(undefined, this.friction.torques);
    } else {
      this.arm.floatMode();
    }
  }

  /** The leader's joint positions clipped to the follower's limits; null unless ENABLED and finite. */
  async read(): Promise<number[] | null> {
    const state = stateOf(this.arm);
    if (state === "FAULT") throw new LeaderError(`leader arm faulted: ${pyStr(this.arm.faultReason)}`);
    if (state !== "ENABLED") return null;
    const q = this.arm.getJointPositions();
    if (q.length !== this.lower.length || !q.every((v) => Number.isFinite(v))) return null;
    return q.map((v, i) => pyMax(this.lower[i]!, pyMin(this.upper[i]!, v)));
  }

  /**
   * Stop floating: the profile gains back, holding where the hand left it. Torque stays on; the
   * caller returns the leader to rest or releases it.
   */
  async close(): Promise<void> {
    const state = stateOf(this.arm);
    if (state !== "ENABLED" && state !== "FAULT") return;
    // Target first, gains second: stiffening toward a stale target would yank the arm.
    this.arm.holdCurrentPosition();
    this.arm.setGains();
  }
}
