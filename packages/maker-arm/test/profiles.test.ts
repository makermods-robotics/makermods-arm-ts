// Port of maker-arm-sdk tests/unit/test_profiles.py: the shipped profile holds the reference values
// below.
import { describe, expect, it } from "vitest";
import { defaultConfig, degrees, MAKER_ARM, protocol } from "@makermods/maker-arm";
import { JointMapper } from "../../arm-core/src/mapping.js";
import { URDF } from "../src/generated/urdf.js";

// name: [motor_id, model, limits_deg, kp, kd]
const REFERENCE: Record<string, [number, string, [number, number], number, number]> = {
  shoulder_pan: [1, "RS00", [-158.2, 156.1], 60, 4.0],
  shoulder_lift: [2, "RS02", [-175.3, -3.2], 150, 4.5],
  elbow_flex: [3, "RS02", [2.8, 236.1], 90, 3.0],
  wrist_flex: [4, "RS00", [-65.2, 104.0], 30, 2.0],
  wrist_yaw: [5, "RS00", [-97.6, 78.0], 30, 2.0],
  wrist_roll: [6, "RS00", [-153.2, 152.0], 30, 2.0],
  gripper: [7, "RS00", [-96.0, -2.5], 20, 0.5],
};

const STAR_DIRECTIONS: Record<string, number> = {
  shoulder_pan: 1.446501, shoulder_lift: -0.859866, elbow_flex: -0.857332,
  wrist_flex: -0.85871, wrist_yaw: -1.322941, wrist_roll: 0.954213, gripper: -1.983613,
};

// tests/unit/gravity_fixtures.json "chain" of the Python SDK, the values the gravity reference was
// checked with (its "urdf" entry is the path of the URDF the TS profile carries as text).
const GRAVITY_CHAIN = {
  sign: [1, 1, 1, 1, -1, 1],
  offset_deg: [0, 0, 0, 0, 0, 0],
  base_rpy_deg: [90, 0, 180],
  gravity_factor: [1, 1, 1, 1, 1, 1],
  gravity_limit_nm: [0.5, 15.0, 7.5, 2.0, 0.5, 0.5],
  limits_deg: [[-158.2, 156.1], [-175.3, -3.2], [2.8, 236.1], [-65.2, 104.0], [-97.6, 78.0], [-153.2, 152.0]],
  tcp_xyz: [0.0, 0.0, -0.1696387],
  tcp_rpy_deg: [180.0, 0.0, 90.0],
};

/** pytest.approx over a list, default tolerance. */
function approxList(actual: readonly number[], expected: readonly number[]): void {
  expect(actual.length).toBe(expected.length);
  actual.forEach((a, i) => {
    const e = expected[i]!;
    expect(Math.abs(a - e), `[${i}] ${a} ≈ ${e}`).toBeLessThanOrEqual(Math.max(1e-6 * Math.abs(e), 1e-12));
  });
}

/** Python round(x, 6). */
const round6 = (x: number): number => Number(x.toFixed(6));

describe("profiles", () => {
  it("profile matches the reference", () => {
    const config = defaultConfig();
    expect(config.joints.map((j) => j.name)).toEqual(Object.keys(REFERENCE));
    for (const j of config.joints) {
      const [motorId, model, [lo, hi], kp, kd] = REFERENCE[j.name]!;
      expect([j.motorId, j.model, j.kp, j.kd, j.direction, j.offset]).toEqual([motorId, model, kp, kd, 1, 0.0]);
      approxList([degrees(j.lo), degrees(j.hi)], [lo, hi]);
    }
  });

  it("only the gripper has watchdog and temperature protection", () => {
    const config = defaultConfig();
    const armJoints = config.joints.slice(0, -1);
    const gripper = config.joints[config.joints.length - 1]!;
    expect([gripper.canTimeoutMs, gripper.maxTemperatureC, gripper.enableTemperatureC]).toEqual([500, 55, 45]);
    expect(gripper.releaseOnFault && !gripper.returnToRest).toBe(true);
    approxList([degrees(gripper.openPosition!)], [-96.0]); // the open limit
    for (const j of armJoints) {
      expect(j.canTimeoutMs).toBeNull();
      expect(j.maxTemperatureC).toBeNull();
      expect(j.returnToRest && !j.releaseOnFault).toBe(true);
    }
    expect(config.feedbackTimeout).toBe(0.25);
  });

  it("star mapping matches the reference", () => {
    const mapping = MAKER_ARM.defaultStarMapping;
    expect(mapping.joints.map((j) => [j.name, j.servo])).toEqual(Object.keys(STAR_DIRECTIONS).map((n, i) => [n, i]));
    for (const j of mapping.joints) {
      expect(j.direction).toBe(STAR_DIRECTIONS[j.name!]);
      const [lo, hi] = REFERENCE[j.name!]![2];
      if (j.name !== "gripper") expect(j.range_deg).toEqual([Math.trunc(lo), Math.trunc(hi)]);
    }
    JointMapper.fromJson(mapping);
  });

  it("profile model is the gravity reference", () => {
    const spec = GRAVITY_CHAIN;
    const config = defaultConfig();
    const arm = config.joints.slice(0, 6);
    // Python compares resolve_urdf(config.urdf) with the URDF path; the TS profile carries the text.
    expect(config.urdf!.startsWith("<?xml")).toBe(true);
    expect(config.urdf).toBe(URDF);
    approxList(config.urdfBaseRpy.map(degrees), spec.base_rpy_deg);
    approxList(config.tcpXyz, spec.tcp_xyz);
    approxList(config.tcpRpy.map(degrees), spec.tcp_rpy_deg);
    expect(arm.map((j) => j.urdfSign)).toEqual(spec.sign);
    expect(arm.map((j) => degrees(j.urdfOffset))).toEqual(spec.offset_deg);
    expect(arm.map((j) => j.gravityLimitNm)).toEqual(spec.gravity_limit_nm);
    expect(arm.map((j) => j.gravityFactor)).toEqual(spec.gravity_factor);
    expect(arm.map((j) => [round6(degrees(j.lo)), round6(degrees(j.hi))])).toEqual(spec.limits_deg);
    for (const j of arm) expect(j.gravityLimitNm!).toBeLessThan(protocol.MOTOR_PARAMS[j.model]!.tMax);
  });
});
