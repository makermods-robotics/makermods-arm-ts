// SO-101 units, calibration files, Dream's reference arm, the profile and the driver's goal ticks. Ported from the
// console's test/so101.test.ts ("SO-101 units") and test/command-limit.test.ts ("SO-101 goal ticks").
import { ValueError } from "@makermods/arm-core";
import { describe, expect, it } from "vitest";
import { armLimits, limitCommand, policyCaps } from "../src/limits.js";
import { SO101_SIDE_LIMITS_DEG, armConfigOf, so101Profile } from "../src/profile.js";
import {
  CENTER_TICK,
  REFERENCE_UNITS,
  alignToReference,
  calibrationDifferences,
  degToTicks,
  parseSo101Calibration,
  referenceRanges,
  shortJoints,
  ticksToDeg,
  ticksToVector,
  vectorToTicks,
  type So101Calibration,
} from "../src/so101.js";
import fixtures from "./fixtures/calibrations.json" with { type: "json" };

const LEFT_FILE = fixtures.follower_left as So101Calibration;

describe("SO-101 units (dreamlabs robots/feetech.py, so101.py)", () => {
  it("reads joints as (ticks - 2048) * 360 / 4096 and the gripper 0..100 over its calibrated range", () => {
    expect(ticksToDeg(CENTER_TICK)).toBe(0);
    expect(ticksToDeg(3072)).toBe(90);
    expect(degToTicks(-90)).toBe(1024);
    const ticks = [2048, 3072, 1024, 2048, 2048, 1278];
    expect(ticksToVector(ticks, LEFT_FILE)).toEqual([0, 90, -90, 0, 0, 0, 0]);
    expect(ticksToVector([...ticks.slice(0, 5), 2842], LEFT_FILE)[6]).toBe(100);
    // Goals: the pad is ignored, joints clamped to the calibrated range, the gripper kept 3 inside its open end.
    const goal = vectorToTicks([0, 90, -170, 0, 0, 55, 100], LEFT_FILE);
    expect(goal.slice(0, 2)).toEqual([2048, 3072]);
    expect(goal[2]).toBe(LEFT_FILE.elbow_flex.range_min);
    expect(goal[5]).toBe(Math.round(1278 + 0.97 * (2842 - 1278)));
  });

  it("validates a calibration file as dream's read_calibration does", () => {
    expect(parseSo101Calibration(LEFT_FILE)).toEqual(LEFT_FILE);
    expect(() => parseSo101Calibration({ ...LEFT_FILE, gripper: { ...LEFT_FILE.gripper, id: 7 } })).toThrow(/id 7/);
    expect(() => parseSo101Calibration({ ...LEFT_FILE, elbow_flex: { ...LEFT_FILE.elbow_flex, range_min: 4000 } })).toThrow(/range/);
    expect(() => parseSo101Calibration({ shoulder_pan: LEFT_FILE.shoulder_pan })).toThrow(/shoulder_lift/);
    expect(calibrationDifferences(LEFT_FILE, { ...LEFT_FILE, wrist_flex: { ...LEFT_FILE.wrist_flex, homing_offset: 1824 } })).toEqual(["wrist_flex homing_offset 1824 (file 1823)"]);
  });

  it("puts each joint's zero at the reference arm's middle (so101_calibration.finish)", () => {
    const ref = referenceRanges("right");
    const { scale, offset } = REFERENCE_UNITS.right;
    const radPerTick = (2 * Math.PI) / 4096;
    expect((ref.shoulder_pan!.low - 2047) * radPerTick).toBeCloseTo(offset[0]! - 100 * scale[0]!, 9);
    // A hand range centred 40 ticks above the reference's middle: the homing goes up by 40, the range down.
    const mid = Math.round(ref.elbow_flex!.middle) + 40;
    const half = Math.round((ref.elbow_flex!.high - ref.elbow_flex!.low) / 2);
    const entries = { ...LEFT_FILE, elbow_flex: { id: 3, drive_mode: 0, homing_offset: 100, range_min: mid - half, range_max: mid + half } };
    const { calibration, movedDeg } = alignToReference("right", entries);
    const shift = Math.round(mid - ref.elbow_flex!.middle);
    expect(calibration.elbow_flex).toEqual({ id: 3, drive_mode: 0, homing_offset: 100 + shift, range_min: mid - half - shift, range_max: mid + half - shift });
    expect(movedDeg.elbow_flex).toBeCloseTo((-shift * 360) / 4096, 1);
    expect(calibration.wrist_roll).toEqual(LEFT_FILE.wrist_roll);
    expect(calibration.gripper).toEqual(LEFT_FILE.gripper);
    const far = { ...entries, elbow_flex: { ...entries.elbow_flex, range_min: mid - half + 341, range_max: mid + half + 341 } };
    expect(() => alignToReference("right", far)).toThrow(/degrees from its middle/);
  });

  it("an automatic range cut short by an obstruction is refused (shortJoints)", () => {
    expect(shortJoints(LEFT_FILE, "left", "either")).toEqual([]);
    const blocked = { ...LEFT_FILE, shoulder_pan: { ...LEFT_FILE.shoulder_pan, range_min: 1700, range_max: 2500 } };
    expect(shortJoints(blocked, "left", "either")).toEqual(["base turn"]);
    const shut = { ...LEFT_FILE, gripper: { ...LEFT_FILE.gripper, range_min: 2000, range_max: 2200 } };
    expect(shortJoints(shut, "right", "either")).toEqual(["gripper"]);
  });

  it("an automatic wrist bend a little shorter than the right reference arm's is an SO-101's (a real right follower)", () => {
    const wrist = { ...LEFT_FILE, wrist_flex: { ...LEFT_FILE.wrist_flex, range_min: 861, range_max: 3233 } };
    expect(shortJoints(wrist, "right", "side")).toEqual(["wrist bend"]);
    expect(shortJoints(wrist, "right", "either")).toEqual([]);
    const held = { ...LEFT_FILE, wrist_flex: { ...LEFT_FILE.wrist_flex, range_min: 1200, range_max: 3233 } };
    expect(shortJoints(held, "right", "either")).toEqual(["wrist bend"]);
  });
});

describe("the SO-101 profile (console settings ARM_TYPES.so101)", () => {
  it("gives each side its limits, the pad and the 0..100 gripper", () => {
    for (const side of ["left", "right"] as const) {
      const p = so101Profile({ side });
      expect(p.limits_deg).toEqual(SO101_SIDE_LIMITS_DEG[side]);
      expect(armLimits(armConfigOf(p))).toEqual([...SO101_SIDE_LIMITS_DEG[side], [0, 0], [0, 100]]);
      expect(p).toMatchObject({ limit_tolerance_deg: 10, servo_hz: 40, rest_speed_deg_s: 15, max_policy_speed_deg_s: 15, max_policy_gripper_speed_deg_s: 45 });
    }
  });

  it("refuses limits outside the vendor limits and anything looser than the shipped bounds", () => {
    const left = SO101_SIDE_LIMITS_DEG.left.map((r) => [...r] as [number, number]);
    expect(() => so101Profile({ limits_deg: left.map((r, i) => (i === 2 ? [r[0], 95] : r)) })).toThrow(/vendor limits/);
    expect(() => so101Profile({ limits_deg: left.slice(0, 4) })).toThrow(ValueError);
    expect(() => so101Profile({ limit_tolerance_deg: 15 })).toThrow(/limit_tolerance_deg/);
    expect(() => so101Profile({ rest_speed_deg_s: 30 })).toThrow(/rest_speed_deg_s/);
    expect(() => so101Profile({ servo_hz: 5 })).toThrow(/servo_hz/);
    expect(so101Profile({ limits_deg: left.map((r, i) => (i === 0 ? [-80, 80] : r)) }).limits_deg[0]).toEqual([-80, 80]);
  });
});

describe("SO-101 goal ticks (console command-limit.test.ts)", () => {
  const SO101_CAL: So101Calibration = {
    shoulder_pan: { id: 1, drive_mode: 0, homing_offset: 0, range_min: 0, range_max: 4095 },
    shoulder_lift: { id: 2, drive_mode: 0, homing_offset: 0, range_min: 0, range_max: 4095 },
    elbow_flex: { id: 3, drive_mode: 0, homing_offset: 0, range_min: 0, range_max: 4095 },
    wrist_flex: { id: 4, drive_mode: 0, homing_offset: 0, range_min: 0, range_max: 4095 },
    wrist_roll: { id: 5, drive_mode: 0, homing_offset: 0, range_min: 0, range_max: 4095 },
    gripper: { id: 6, drive_mode: 0, homing_offset: 0, range_min: 1000, range_max: 3000 },
  };
  const arm = armConfigOf(so101Profile({ side: "left" }));

  it("a joint within its limit is rounded to a tick inside it, not to the nearest", () => {
    const q = [0, 0, 89.0, 0, 0, 0, 50];
    expect(ticksToDeg(vectorToTicks(q, SO101_CAL)[2]!)).toBeCloseTo(89.033, 3); // nearest: past 89.0
    expect(ticksToDeg(vectorToTicks(q, SO101_CAL, arm.limits_deg)[2]!)).toBeLessThanOrEqual(89.0);
    expect(ticksToDeg(vectorToTicks([0, 0, -102.3, 0, 0, 0, 50], SO101_CAL, arm.limits_deg)[2]!)).toBeGreaterThanOrEqual(-102.3);
    // Outside its bound (a joint being brought in from past it): the nearest tick, as before.
    expect(vectorToTicks([0, 0, 90.5, 0, 0, 0, 50], SO101_CAL, arm.limits_deg)[2]).toBe(vectorToTicks([0, 0, 90.5, 0, 0, 0, 50], SO101_CAL)[2]);
  });

  it("the final limiter keeps every command inside the band, rate-capped, and brings a joint past its limit in slowly", () => {
    const caps = policyCaps(15, 45);
    const last = [0, -95, 95, 60, -80, 0, 10]; // the elbow 6° past 89°
    let command = last;
    for (let t = 0; t < 4; t += 0.025) {
      const r = limitCommand([0, -95, 120, 60, -80, 0, 100], command, arm, caps, 0.025);
      r.command.forEach((v, i) => expect(Math.abs(v - command[i]!)).toBeLessThanOrEqual((i === 6 ? 45 : 15) * 0.025 + 1e-9));
      expect(r.command[2]!).toBeLessThanOrEqual(Math.max(89, command[2]!) + 1e-9);
      if (command[2]! > 89 + 1e-6) expect(r.entering).toBe(true);
      command = r.command;
    }
    expect(command[2]).toBe(89);
    expect(command[6]).toBe(98); // 2 short of fully open
    // A stalled tick never spends its backlog in one step.
    expect(limitCommand([0, 0, 0, 0, 0, 0, 0], [0, 50, 0, 0, 0, 0, 0], arm, caps, 5).command[1]).toBeCloseTo(50 - 15 * 0.1, 9);
  });
});
