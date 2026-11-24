/** The SO-101 profile: soft limits, speeds and the control rate, with the console's shipped defaults (servo/settings.ts
ARM_TYPES.so101 and defaultArm, servo/command-limit.ts). Keys are snake_case as in the console's settings; angles
are degrees from the LeRobot middle pose, the gripper LeRobot's 0 (closed) .. 100 (open). So101Arm converts to
radians at its surface. */

import { ValueError } from "@makermods/arm-core";

/** A [low, high] range. */
export type Range = [number, number];

/** The five joints' limits both reference arms allow together (ARM_TYPES.so101.limits_deg without the pad). */
export const SO101_VENDOR_LIMITS_DEG: readonly Range[] = [
  [-112.2, 118.5],
  [-102.4, 105.5],
  [-109.8, 89.0],
  [-113.7, 105.6],
  [-177.2, 182.7],
];

/** Each side's soft limits: its so101_faas URDF limits (the reference arm's calibrated range), rounded inward to
0.1° (ARM_TYPES.so101.side_limits_deg without the pad). */
export const SO101_SIDE_LIMITS_DEG: Readonly<Record<"left" | "right", readonly Range[]>> = {
  left: [[-84.1, 118.5], [-102.4, 105.5], [-102.3, 89.0], [-102.1, 105.3], [-177.2, 182.7]],
  right: [[-112.2, 78.3], [-98.8, 104.5], [-109.8, 85.1], [-113.7, 105.6], [-177.2, 182.7]],
};

/** How far past a soft limit a measured joint may be before enable and control refuse it (a seated elbow can sit a
few degrees past the reference range). */
export const SO101_LIMIT_TOLERANCE_DEG = 10.0;

/** The gripper's endpoints on its 0..100 scale. */
export const GRIPPER_CLOSED = 0;
export const GRIPPER_OPEN = 100;

/** One arm's limits and speeds, the console's ArmConfig fields the motion math reads. */
export interface So101ArmConfig {
  name: string;
  /** Six ranges: the five joints, then the pad's [0, 0]. */
  limits_deg: Range[];
  gripper_closed_deg: number;
  gripper_open_deg: number;
  max_velocity_deg_s: number;
  max_gripper_deg_s: number;
  max_tracking_error_deg: number;
}

/** What So101Arm runs with. */
export interface So101Profile {
  /** Which reference arm this is ("left" or "right"): its limits and the calibration alignment. */
  side: "left" | "right";
  /** The five joints' soft limits, degrees; within SO101_VENDOR_LIMITS_DEG. */
  limits_deg: Range[];
  /** How far past a limit a measured joint may be, degrees (at most SO101_LIMIT_TOLERANCE_DEG). */
  limit_tolerance_deg: number;
  /** The return to rest's joint speed cap, deg/s (it also stays under rest_speed_deg_s). */
  max_velocity_deg_s: number;
  /** The return to rest's gripper speed cap, gripper units (0..100) per second. */
  max_gripper_deg_s: number;
  /** The return to rest stops when a joint lags its command by more than this, degrees. */
  max_tracking_error_deg: number;
  /** The return to rest's speed, deg/s (the console's REST_SPEED_DEG_S). */
  rest_speed_deg_s: number;
  /** Each joint's rate cap for targets (setJointTargets, moveTo), deg/s. */
  max_policy_speed_deg_s: number;
  /** The gripper's rate cap for targets, gripper units per second. */
  max_policy_gripper_speed_deg_s: number;
  /** The control loop's rate, Hz. */
  servo_hz: number;
}

/** The console's defaults for a side. */
export function defaultSo101Profile(side: "left" | "right" = "left"): So101Profile {
  return {
    side,
    limits_deg: SO101_SIDE_LIMITS_DEG[side].map((r) => [...r] as Range),
    limit_tolerance_deg: SO101_LIMIT_TOLERANCE_DEG,
    max_velocity_deg_s: 15,
    max_gripper_deg_s: 20,
    max_tracking_error_deg: 12,
    rest_speed_deg_s: 15,
    max_policy_speed_deg_s: 15,
    max_policy_gripper_speed_deg_s: 45,
    servo_hz: 40,
  };
}

const within = (path: string, v: unknown, gt: number, le: number): number => {
  if (typeof v !== "number" || !(v > gt && v <= le)) throw new ValueError(`${path} must be a number above ${gt} and at most ${le}, got ${String(v)}`);
  return v;
};

/** A profile from the side's defaults and `overrides`, validated as the console validates an SO-101 arm's settings:
limits within the vendor limits, speeds and the rate in their bounds, nothing looser than the shipped tolerance. */
export function so101Profile(overrides: Partial<So101Profile> = {}): So101Profile {
  const side = overrides.side ?? "left";
  if (side !== "left" && side !== "right") throw new ValueError(`side must be "left" or "right", got ${String(side)}`);
  const p: So101Profile = { ...defaultSo101Profile(side), ...overrides, side };
  if (!Array.isArray(p.limits_deg) || p.limits_deg.length !== 5) throw new ValueError("limits_deg must have five ranges, base to wrist roll");
  p.limits_deg = p.limits_deg.map((r, i) => {
    const [vLo, vHi] = SO101_VENDOR_LIMITS_DEG[i] as Range;
    if (!Array.isArray(r) || r.length !== 2 || !r.every(Number.isFinite)) throw new ValueError(`limits_deg.${i} must be two finite numbers`);
    const [lo, hi] = r as Range;
    if (!(vLo <= lo && lo < hi && hi <= vHi)) throw new ValueError("joint limits must be within SO-101 vendor limits");
    return [lo, hi];
  });
  within("limit_tolerance_deg", p.limit_tolerance_deg, 0, SO101_LIMIT_TOLERANCE_DEG);
  within("max_velocity_deg_s", p.max_velocity_deg_s, 0, 60);
  within("max_gripper_deg_s", p.max_gripper_deg_s, 0, 60);
  within("max_tracking_error_deg", p.max_tracking_error_deg, 0, 30);
  within("rest_speed_deg_s", p.rest_speed_deg_s, 0, 15);
  within("max_policy_speed_deg_s", p.max_policy_speed_deg_s, 0, 180);
  within("max_policy_gripper_speed_deg_s", p.max_policy_gripper_speed_deg_s, 0, 360);
  if (typeof p.servo_hz !== "number" || !(p.servo_hz >= 10 && p.servo_hz <= 100)) throw new ValueError(`servo_hz must be 10 to 100, got ${String(p.servo_hz)}`);
  return p;
}

/** The motion math's view of a profile (the pad's [0, 0] appended). */
export function armConfigOf(profile: So101Profile, name: string = profile.side): So101ArmConfig {
  return {
    name,
    limits_deg: [...profile.limits_deg.map((r) => [...r] as Range), [0, 0]],
    gripper_closed_deg: GRIPPER_CLOSED,
    gripper_open_deg: GRIPPER_OPEN,
    max_velocity_deg_s: Math.min(profile.max_velocity_deg_s, profile.rest_speed_deg_s),
    max_gripper_deg_s: profile.max_gripper_deg_s,
    max_tracking_error_deg: profile.max_tracking_error_deg,
  };
}
