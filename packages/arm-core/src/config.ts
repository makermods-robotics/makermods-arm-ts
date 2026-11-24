/**
 * Arm profile: machine-specific data (limits, gains, protection, model).
 *
 * A profile is written with snake_case keys, the same keys as the Python SDK's YAML profiles, and
 * `ArmConfig.fromRaw()` turns it into an ArmConfig with camelCase fields. Joint limits may be given
 * in degrees (`limits_deg: [lo, hi]`) or radians (`lo`/`hi`); the SDK API is radians. Any other
 * `<name>_deg` setting, top-level or per joint, a number or a list, is converted to `<name>` in
 * radians (`max_velocity_deg`, `rest_pose_deg`, `tcp_rpy_deg`, ...).
 *
 * Model fields (read by kinematics.ts and gravity.ts): `urdf` (the URDF XML text, not a path),
 * `urdf_joints` (actuated URDF joints in order; default: every revolute joint on the longest
 * chain), `urdf_base_rpy` (mount rotation turning the URDF z-up), `tcp_xyz`/`tcp_rpy` (tool point
 * on the last arm link); per joint `urdf_sign`/`urdf_offset` (URDF angle = joint * sign + offset),
 * `gravity_limit_nm` (cap, required on every URDF joint when gravity is on), `gravity_factor`
 * (0..2, per-joint correction), and the leader-arm friction terms.
 *
 * `ik` (read by trajectory-ik.ts) keeps its snake_case keys: the trajectory IK's weights and
 * `limit_margin`, and per URDF joint `rest_pose`, `rest_weight` and `max_speed` (rad/s); `_deg`
 * forms convert as above.
 */

import { ValueError } from "./errors.js";
import { DEFAULT_GRAVITY_SCALE, MAX_GRAVITY_SCALE } from "./gravity.js";
import type { Product } from "./motor.js";
import { JOINT_FIELDS, WEIGHT_FIELDS } from "./trajectory-ik.js";

/** A profile as written: snake_case keys, `_deg` forms allowed. */
export type RawProfile = Readonly<Record<string, unknown>>;

/** Same arithmetic as Python's math.radians / math.degrees, so converted limits match bit for bit. */
export const radians = (deg: number): number => deg * (Math.PI / 180);
export const degrees = (rad: number): number => rad * (180 / Math.PI);

const camel = (key: string): string => key.replace(/_([a-z0-9])/g, (_, c: string) => c.toUpperCase());

export interface JointConfigInit {
  motorId: number;
  /** rad, soft limit in joint coordinates. */
  lo: number;
  hi: number;
  kp: number;
  kd: number;
  model: string;
  name?: string;
  /** ±1. */
  direction?: number;
  /** rad: motor = joint * direction + offset. */
  offset?: number;
  /** Motor-side watchdog, where firmware supports it. */
  canTimeoutMs?: number | null;
  /** Stop (fault) at or above this. */
  maxTemperatureC?: number | null;
  /** Refuse enable unless below this. */
  enableTemperatureC?: number | null;
  /** Included in the verified return to rest on stop. */
  returnToRest?: boolean;
  /** Disable this motor (instead of holding) when it faults. */
  releaseOnFault?: boolean;
  /** Treat the feedback warning flag as a fault. */
  warningIsFault?: boolean;
  /** Gripper holding-effort regulation (see gripper.ts). */
  holdTorqueNm?: number | null;
  /** Gripper: sign of joint motion that closes the jaws. */
  closingDirection?: number;
  /** Gripper: rad, jaws fully open (null: the limit away from closing). */
  openPosition?: number | null;
  /** CAN ID the motor replies on, where the protocol has one. */
  replyId?: number | null;
  /** ±1: URDF angle = joint * urdfSign + urdfOffset. */
  urdfSign?: number;
  urdfOffset?: number;
  /** Cap on this joint's gravity feed-forward. */
  gravityLimitNm?: number | null;
  /** 0..2, scales this joint's gravity torque. */
  gravityFactor?: number;
  /** N·m·s/rad (leader arm float mode). */
  frictionViscous?: number;
  /** rad/s, velocity the viscous term saturates at. */
  frictionVelocityClamp?: number;
  /** N·m in the direction of travel. */
  frictionCoulomb?: number;
  /** N·m while still (breakaway). */
  frictionRestNm?: number;
}

const JOINT_DEFAULTS = {
  name: "",
  direction: 1,
  offset: 0.0,
  canTimeoutMs: null,
  maxTemperatureC: null,
  enableTemperatureC: null,
  returnToRest: true,
  releaseOnFault: false,
  warningIsFault: false,
  holdTorqueNm: null,
  closingDirection: 1,
  openPosition: null,
  replyId: null,
  urdfSign: 1,
  urdfOffset: 0.0,
  gravityLimitNm: null,
  gravityFactor: 1.0,
  frictionViscous: 0.0,
  frictionVelocityClamp: Infinity,
  frictionCoulomb: 0.0,
  frictionRestNm: 0.0,
} satisfies Omit<Required<JointConfigInit>, "motorId" | "lo" | "hi" | "kp" | "kd" | "model">;

const JOINT_REQUIRED = ["motorId", "lo", "hi", "kp", "kd"] as const;
const JOINT_FIELDS_ALL = new Set<string>([...JOINT_REQUIRED, "model", ...Object.keys(JOINT_DEFAULTS)]);

/** One joint of a profile. Immutable; `replace()` returns a changed copy. */
export class JointConfig {
  declare readonly motorId: number;
  declare readonly lo: number;
  declare readonly hi: number;
  declare readonly kp: number;
  declare readonly kd: number;
  declare readonly model: string;
  declare readonly name: string;
  declare readonly direction: number;
  declare readonly offset: number;
  declare readonly canTimeoutMs: number | null;
  declare readonly maxTemperatureC: number | null;
  declare readonly enableTemperatureC: number | null;
  declare readonly returnToRest: boolean;
  declare readonly releaseOnFault: boolean;
  declare readonly warningIsFault: boolean;
  declare readonly holdTorqueNm: number | null;
  declare readonly closingDirection: number;
  declare readonly openPosition: number | null;
  declare readonly replyId: number | null;
  declare readonly urdfSign: number;
  declare readonly urdfOffset: number;
  declare readonly gravityLimitNm: number | null;
  declare readonly gravityFactor: number;
  declare readonly frictionViscous: number;
  declare readonly frictionVelocityClamp: number;
  declare readonly frictionCoulomb: number;
  declare readonly frictionRestNm: number;

  constructor(init: JointConfigInit) {
    Object.assign(this, JOINT_DEFAULTS, init);
    Object.freeze(this);
  }

  get label(): string {
    return this.name || `motor ${this.motorId}`;
  }

  replace(changes: Partial<JointConfigInit>): JointConfig {
    return new JointConfig({ ...(this as JointConfigInit), ...changes });
  }
}

export interface ArmConfigInit {
  product: Product;
  joints?: JointConfig[];
  hostId?: number;
  requiredMotorIds?: number[] | null;
  controlRateHz?: number;
  /** rad/s, command rate limit toward the user target. */
  maxVelocity?: number;
  /** rad, soft-limit inward margin. */
  limitMargin?: number;
  /** s, host-side feedback watchdog. */
  feedbackTimeout?: number;
  /** s, non-clearing fault query per motor while enabled. */
  faultPollInterval?: number;
  /** rad, see Arm.connect(). */
  fullTurnGrace?: number;
  /** On FAULT, hold the current pose (prevents a drop). */
  holdOnFault?: boolean;
  /** 0..1.2, read at enable; see gravity.ts. */
  gravityScale?: number;
  /** URDF XML text of the arm's kinematic model. */
  urdf?: string | null;
  /** Actuated URDF joints, in joint order. */
  urdfJoints?: string[] | null;
  /** rad, URDF root -> world (z-up). */
  urdfBaseRpy?: number[];
  /** m, tool point on the last arm link. */
  tcpXyz?: number[];
  /** rad. */
  tcpRpy?: number[];
  /** rad/s, below it a leader joint counts as still. */
  frictionDeadzone?: number;
  /** rad per joint, the named "rest" pose. */
  restPose?: number[] | null;
  /** Trajectory IK settings (snake_case keys), see the module comment. */
  ik?: Record<string, unknown> | null;
  // Startup sync: until a joint is synced its command stays within step of the measured position;
  // synced once |target - present| <= synced, or released after it moves no more than stuck for
  // stuckSeconds.
  startupStep?: number;
  startupSynced?: number;
  startupStuck?: number;
  startupStuckSeconds?: number;
  // Return to rest (Arm.returnToRest).
  /** rad/s. */
  restSpeed?: number;
  restRateHz?: number;
  /** s. */
  restTimeout?: number;
  restArrived?: number;
  /** Accepted after a stall. */
  restSettle?: number;
  restStallImprovement?: number;
  restStallPolls?: number;
}

const armDefaults = () => ({
  requiredMotorIds: null as number[] | null,
  controlRateHz: 200.0,
  maxVelocity: 1.5,
  limitMargin: 0.0,
  feedbackTimeout: 0.25,
  faultPollInterval: 0.25,
  fullTurnGrace: radians(20.0),
  holdOnFault: true,
  gravityScale: DEFAULT_GRAVITY_SCALE,
  urdf: null as string | null,
  urdfJoints: null as string[] | null,
  urdfBaseRpy: [0.0, 0.0, 0.0],
  tcpXyz: [0.0, 0.0, 0.0],
  tcpRpy: [0.0, 0.0, 0.0],
  frictionDeadzone: 0.05,
  restPose: null as number[] | null,
  ik: null as Record<string, unknown> | null,
  startupStep: radians(1.0),
  startupSynced: radians(3.0),
  startupStuck: radians(0.2),
  startupStuckSeconds: 1.0,
  restSpeed: radians(30.0),
  restRateHz: 30.0,
  restTimeout: 10.0,
  restArrived: radians(2.0),
  restSettle: radians(6.0),
  restStallImprovement: radians(0.25),
  restStallPolls: 15,
});

const ARM_FIELDS_ALL = new Set<string>(["hostId", ...Object.keys(armDefaults())]);

const finite = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v);
const isNumber = (v: unknown): v is number => typeof v === "number";

const ARM_NUMBERS = [
  "hostId", "controlRateHz", "maxVelocity", "limitMargin", "feedbackTimeout", "faultPollInterval", "fullTurnGrace",
  "gravityScale", "frictionDeadzone", "startupStep", "startupSynced", "startupStuck", "startupStuckSeconds", "restSpeed",
  "restRateHz", "restTimeout", "restArrived", "restSettle", "restStallImprovement", "restStallPolls",
] as const;
const JOINT_NUMBERS = [
  "motorId", "lo", "hi", "kp", "kd", "direction", "offset", "closingDirection", "urdfSign", "urdfOffset", "gravityFactor",
  "frictionViscous", "frictionVelocityClamp", "frictionCoulomb", "frictionRestNm",
] as const;
const JOINT_FINITE = ["lo", "hi", "offset"] as const;
const JOINT_OPTIONAL_NUMBERS = [
  "canTimeoutMs", "maxTemperatureC", "enableTemperatureC", "holdTorqueNm", "openPosition", "replyId", "gravityLimitNm",
] as const;
const JOINT_BOOLEANS = ["returnToRest", "releaseOnFault", "warningIsFault"] as const;

/** A whole arm's profile. The product it belongs to rides along, so `new Arm(config, backend)` needs nothing else. */
export class ArmConfig {
  product: Product;
  hostId: number;
  requiredMotorIds!: number[] | null;
  controlRateHz!: number;
  maxVelocity!: number;
  limitMargin!: number;
  feedbackTimeout!: number;
  faultPollInterval!: number;
  fullTurnGrace!: number;
  holdOnFault!: boolean;
  gravityScale!: number;
  urdf!: string | null;
  urdfJoints!: string[] | null;
  urdfBaseRpy!: number[];
  tcpXyz!: number[];
  tcpRpy!: number[];
  frictionDeadzone!: number;
  restPose!: number[] | null;
  ik!: Record<string, unknown> | null;
  startupStep!: number;
  startupSynced!: number;
  startupStuck!: number;
  startupStuckSeconds!: number;
  restSpeed!: number;
  restRateHz!: number;
  restTimeout!: number;
  restArrived!: number;
  restSettle!: number;
  restStallImprovement!: number;
  restStallPolls!: number;
  joints: JointConfig[];

  /** Does not validate; fromRaw() and replace() do. */
  constructor(init: ArmConfigInit) {
    const { product, joints, ...rest } = init;
    Object.assign(this, armDefaults(), rest);
    this.product = product;
    this.hostId = init.hostId ?? product.defaultHostId;
    this.joints = joints ?? [];
  }

  get nJoints(): number {
    return this.joints.length;
  }

  /** The product's shipped profile, or `raw` (same keys as the Python SDK's YAML profiles), validated. */
  static fromRaw(raw: RawProfile, product: Product): ArmConfig {
    const { joints: rawJoints, ...top } = raw as Record<string, unknown>;
    if (rawJoints === undefined) throw new ValueError("profile: missing config item 'joints'");
    if (!Array.isArray(rawJoints)) throw new ValueError("profile: joints must be a list");
    const joints = rawJoints.map((j) => jointFromRaw(j as Record<string, unknown>, product));
    if (top.ik !== null && typeof top.ik === "object" && !Array.isArray(top.ik)) {
      top.ik = degreesToRadians(top.ik as Record<string, unknown>);
    }
    const fields: Record<string, unknown> = {};
    for (const [key, value] of Object.entries(degreesToRadians(top))) {
      const name = camel(key);
      if (!ARM_FIELDS_ALL.has(name)) {
        throw new ValueError(`profile: unrecognized config field '${key}' -- check spelling`);
      }
      fields[name] = value;
    }
    const cfg = new ArmConfig({ ...(fields as Partial<ArmConfigInit>), product, joints });
    cfg.validate();
    return cfg;
  }

  /** A validated copy with `changes` applied (the Python SDK's dataclasses.replace). */
  replace(changes: Partial<ArmConfigInit>): ArmConfig {
    const cfg = new ArmConfig({ ...(this as ArmConfigInit), ...changes });
    cfg.validate();
    return cfg;
  }

  validate(): void {
    const { idRange, models } = this.product;
    // A profile is plain data: a string where a number belongs would otherwise pass the range
    // checks below and then concatenate instead of add.
    for (const name of ARM_NUMBERS) {
      if (!isNumber(this[name])) throw new ValueError(`${name} must be a number, got ${JSON.stringify(this[name])}`);
    }
    if (typeof this.holdOnFault !== "boolean") throw new ValueError("holdOnFault must be true or false");
    if (this.requiredMotorIds !== null && !(Array.isArray(this.requiredMotorIds) && this.requiredMotorIds.every(Number.isInteger))) {
      throw new ValueError(`requiredMotorIds must list motor IDs, got ${JSON.stringify(this.requiredMotorIds)}`);
    }
    for (const j of this.joints) {
      const label = `motor ${j.motorId}`;
      for (const name of JOINT_NUMBERS) {
        if (!isNumber(j[name])) throw new ValueError(`${label}: ${name} must be a number, got ${JSON.stringify(j[name])}`);
      }
      for (const name of JOINT_FINITE) {
        if (!Number.isFinite(j[name])) throw new ValueError(`${label}: ${name} must be finite, got ${j[name]}`);
      }
      for (const name of JOINT_OPTIONAL_NUMBERS) {
        if (j[name] !== null && !isNumber(j[name])) {
          throw new ValueError(`${label}: ${name} must be a number or null, got ${JSON.stringify(j[name])}`);
        }
      }
      for (const name of JOINT_BOOLEANS) {
        if (typeof j[name] !== "boolean") throw new ValueError(`${label}: ${name} must be true or false`);
      }
      if (typeof j.model !== "string" || typeof j.name !== "string") throw new ValueError(`${label}: model and name must be strings`);
    }
    if (!(Number.isInteger(this.hostId) && this.hostId >= 0 && this.hostId <= 0x7ff)) {
      throw new ValueError("host_id must be a standard CAN ID");
    }
    const positive = [
      "controlRateHz", "maxVelocity", "feedbackTimeout", "faultPollInterval", "startupStep", "startupSynced",
      "startupStuck", "startupStuckSeconds", "restSpeed", "restRateHz", "restTimeout", "restArrived", "restSettle",
      "restStallImprovement", "restStallPolls",
    ] as const;
    for (const name of positive) {
      if (!(this[name] > 0)) throw new ValueError(`${name} must be >0, got ${this[name]}`);
    }
    if (!(this.limitMargin >= 0) || !(this.fullTurnGrace >= 0)) {
      throw new ValueError("limitMargin and fullTurnGrace must be >=0");
    }
    if (!(this.gravityScale >= 0 && this.gravityScale <= MAX_GRAVITY_SCALE)) {
      throw new ValueError(`gravityScale must be in [0, ${MAX_GRAVITY_SCALE}], got ${this.gravityScale}`);
    }
    for (const name of ["tcpXyz", "tcpRpy", "urdfBaseRpy"] as const) {
      const value = this[name];
      if (!(Array.isArray(value) && value.length === 3 && Array.from(value).every(finite))) {
        throw new ValueError(`${name} must be three finite numbers, got ${JSON.stringify(value)}`);
      }
    }
    if (this.urdf !== null && typeof this.urdf !== "string") {
      throw new ValueError("urdf must be the URDF text");
    }
    const names = this.urdfJoints;
    if (names !== null) {
      if (!(Array.isArray(names) && names.length > 0 && names.every((n) => typeof n === "string" && n !== "")
            && new Set(names).size === names.length)) {
        throw new ValueError(`urdfJoints must be distinct joint names, got ${JSON.stringify(names)}`);
      }
      if (names.length > this.joints.length) {
        throw new ValueError(`urdfJoints names ${names.length} joints, the profile has ${this.joints.length}`);
      }
    }
    if (this.restPose !== null
        && !(Array.isArray(this.restPose) && this.restPose.length === this.joints.length && Array.from(this.restPose).every(finite))) {
      throw new ValueError(
        `restPose needs one finite value per joint (${this.joints.length}), got ${JSON.stringify(this.restPose)}`);
    }
    this.validateIk();
    if (!(finite(this.frictionDeadzone) && this.frictionDeadzone >= 0)) {
      throw new ValueError(`frictionDeadzone must be finite and >=0, got ${this.frictionDeadzone}`);
    }
    const ids = this.joints.map((j) => j.motorId);
    if (new Set(ids).size !== ids.length) throw new ValueError(`motor_id must be unique: ${JSON.stringify(ids)}`);
    const required = this.requiredMotorIds;
    if (required !== null && !(ids.length === required.length && ids.every((id, i) => id === required[i]))) {
      throw new ValueError(`motor IDs must be exactly ${JSON.stringify(required)}, in order; got ${JSON.stringify(ids)}`);
    }
    for (const j of this.joints) {
      const label = `motor ${j.motorId}`;
      if (!(Number.isInteger(j.motorId) && idRange[0] <= j.motorId && j.motorId <= idRange[1])) {
        throw new ValueError(`motor_id must be in [${idRange[0]}, ${idRange[1]}], got ${j.motorId}`);
      }
      if (j.replyId !== null && !(Number.isInteger(j.replyId) && j.replyId >= 0 && j.replyId <= 0x7ff)) {
        throw new ValueError(`${label}: reply_id must be a standard CAN ID`);
      }
      if (!models.includes(j.model)) {
        throw new ValueError(`${label}: unknown model '${j.model}' (supported: ${[...models].sort().join(", ")})`);
      }
      if (j.direction !== 1 && j.direction !== -1) {
        throw new ValueError(`${label}: direction can only be ±1, got ${j.direction}`);
      }
      if (!(j.lo + 2 * this.limitMargin < j.hi)) {
        throw new ValueError(`${label}: limits must satisfy lo + 2*margin < hi, got [${j.lo}, ${j.hi}]`);
      }
      checkGains(this.product, j.motorId, j.kp, j.kd);
      if (j.canTimeoutMs !== null && !(j.canTimeoutMs > 0 && j.canTimeoutMs <= 200_000)) {
        throw new ValueError(`${label}: can_timeout_ms must be in (0, 200000]; it never disables the watchdog`);
      }
      if (j.holdTorqueNm !== null && !(j.holdTorqueNm >= 0.1 && j.holdTorqueNm <= 2.0)) {
        throw new ValueError(`${label}: hold_torque_nm must be in [0.1, 2.0]`);
      }
      if (j.closingDirection !== 1 && j.closingDirection !== -1) {
        throw new ValueError(`${label}: closing_direction can only be ±1`);
      }
      if (j.openPosition !== null
          && !(finite(j.openPosition) && j.lo <= j.openPosition && j.openPosition <= j.hi
               && j.openPosition !== (j.closingDirection > 0 ? j.hi : j.lo))) {
        throw new ValueError(
          `${label}: open_position must lie within the limits, off the closed end, got ${j.openPosition}`);
      }
      if (j.maxTemperatureC !== null && j.enableTemperatureC !== null && !(j.enableTemperatureC < j.maxTemperatureC)) {
        throw new ValueError(`${label}: enable_temperature_c must be below max_temperature_c`);
      }
      checkModelFields(j);
    }
  }

  private validateIk(): void {
    const ik = this.ik;
    if (ik === null) return;
    if (typeof ik !== "object" || Array.isArray(ik)) throw new ValueError(`ik must be a mapping, got ${JSON.stringify(ik)}`);
    const known = [...WEIGHT_FIELDS, ...JOINT_FIELDS] as readonly string[];
    const unknown = Object.keys(ik).filter((k) => !known.includes(k)).sort();
    if (unknown.length) {
      throw new ValueError(`ik: unknown settings ${JSON.stringify(unknown)} (known: ${[...known].sort().join(", ")})`);
    }
    for (const name of WEIGHT_FIELDS) {
      const value = ik[name];
      if (value !== undefined && value !== null && !(finite(value) && value >= 0)) {
        throw new ValueError(`ik.${name} must be a finite number >=0, got ${JSON.stringify(value)}`);
      }
    }
    for (const name of JOINT_FIELDS) {
      const value = ik[name];
      if (value === undefined || value === null) continue;
      if (!(Array.isArray(value) && value.length > 0 && value.length <= this.joints.length && Array.from(value).every(finite))) {
        throw new ValueError(`ik.${name} must list one finite number per URDF joint, got ${JSON.stringify(value)}`);
      }
      if (name === "max_speed" && !value.every((v) => v > 0)) {
        throw new ValueError(`ik.max_speed must be >0 per joint, got ${JSON.stringify(value)}`);
      }
      if (name === "rest_weight" && !value.every((v) => v >= 0)) {
        throw new ValueError(`ik.rest_weight must be >=0 per joint, got ${JSON.stringify(value)}`);
      }
    }
  }
}

export function checkGains(product: Product, motorId: number, kp: number, kd: number): void {
  const { kpRange, kdRange } = product;
  if (!(finite(kd) && kdRange[0] <= kd && kd <= kdRange[1])) {
    throw new ValueError(`motor ${motorId}: kd out of protocol range [${kdRange[0]},${kdRange[1]}], got ${kd}`);
  }
  if (!(finite(kp) && kpRange[0] <= kp && kp <= kpRange[1])) {
    throw new ValueError(`motor ${motorId}: kp out of protocol range [${kpRange[0]},${kpRange[1]}], got ${kp}`);
  }
}

function checkModelFields(j: JointConfig): void {
  const label = `motor ${j.motorId}`;
  if (j.urdfSign !== 1 && j.urdfSign !== -1) throw new ValueError(`${label}: urdf_sign can only be ±1, got ${j.urdfSign}`);
  if (!finite(j.urdfOffset)) throw new ValueError(`${label}: urdf_offset must be finite`);
  if (j.gravityLimitNm !== null && !(finite(j.gravityLimitNm) && j.gravityLimitNm >= 0)) {
    throw new ValueError(`${label}: gravity_limit_nm must be finite and >=0, got ${j.gravityLimitNm}`);
  }
  if (!(finite(j.gravityFactor) && j.gravityFactor >= 0 && j.gravityFactor <= 2)) {
    throw new ValueError(`${label}: gravity_factor must be in [0, 2], got ${j.gravityFactor}`);
  }
  for (const [name, value] of [
    ["friction_viscous", j.frictionViscous], ["friction_coulomb", j.frictionCoulomb], ["friction_rest_nm", j.frictionRestNm],
  ] as const) {
    if (!(finite(value) && value >= 0)) throw new ValueError(`${label}: ${name} must be finite and >=0, got ${value}`);
  }
  if (!(j.frictionVelocityClamp > 0)) {
    throw new ValueError(`${label}: friction_velocity_clamp must be >0 (Infinity = no clamp), got ${j.frictionVelocityClamp}`);
  }
}

/** `<name>_deg` keys (a number or a list) become `<name>` in radians. */
function degreesToRadians(raw: Record<string, unknown>): Record<string, unknown> {
  const out: Record<string, unknown> = {};
  for (let [key, value] of Object.entries(raw)) {
    if (key.endsWith("_deg")) {
      key = key.slice(0, -4);
      if (key in raw) throw new ValueError(`both ${key} and ${key}_deg are set`);
      const numbers = Array.isArray(value) ? value : [value];
      if (!numbers.every(isNumber)) throw new ValueError(`${key}_deg must be a number or a list of numbers, got ${JSON.stringify(value)}`);
      value = Array.isArray(value) ? value.map((v: number) => radians(v)) : radians(value as number);
    }
    out[key] = value;
  }
  return out;
}

function jointFromRaw(rawJoint: Record<string, unknown>, product: Product): JointConfig {
  const raw: Record<string, unknown> = { ...rawJoint };
  if ("limits_deg" in raw) {
    const limits = raw.limits_deg;
    if (!(Array.isArray(limits) && limits.length === 2 && limits.every(isNumber))) {
      throw new ValueError(`limits_deg must be [lo, hi] in degrees, got ${JSON.stringify(limits)}`);
    }
    const [lo, hi] = limits as [number, number];
    delete raw.limits_deg;
    raw.lo = radians(lo);
    raw.hi = radians(hi);
  }
  const fields: Record<string, unknown> = { model: product.defaultModel };
  for (const [key, value] of Object.entries(degreesToRadians(raw))) {
    const name = camel(key);
    if (!JOINT_FIELDS_ALL.has(name)) throw new ValueError(`profile: unrecognized joint field '${key}' -- check spelling`);
    fields[name] = value;
  }
  for (const name of JOINT_REQUIRED) {
    if (fields[name] === undefined) throw new ValueError(`profile: joint is missing '${name}'`);
  }
  return new JointConfig(fields as unknown as JointConfigInit);
}
