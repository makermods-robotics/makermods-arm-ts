/** Port of the Metal SDK's tests/unit/test_profiles.py (product and profile parts), and the product object. */

import { readFileSync } from "node:fs";
import { ArmConfig, degrees, radians, ValueError } from "@makermods/arm-core";
import { describe, expect, it } from "vitest";
import { DamiaoMotor as Motor, defaultConfig, METAL_ARM, protocol } from "../src/index.js";
import { PROFILE } from "../src/generated/profile.js";
import { URDF } from "../src/generated/urdf.js";
import { SimBus } from "../src/sim-motor.js";
import { makeJoint } from "./fakes.js";
import { JointMapper } from "../../arm-core/src/mapping.js";

// name: [motor_id, reply_id, model, limits_deg, kp, kd]
const REFERENCE: Record<string, [number, number, string, [number, number], number, number]> = {
  shoulder_pan: [1, 0x11, "jlo", [-160.0, 160.0], 100, 5.0],
  shoulder_lift: [2, 0x12, "j2", [-180.0, 0.0], 100, 5.0],
  elbow_flex: [3, 0x13, "jlo", [0.0, 180.0], 100, 5.0],
  wrist_flex: [4, 0x14, "jlo", [-123.0, 81.0], 50, 5.0],
  wrist_yaw: [5, 0x15, "jhi", [-85.0, 85.0], 20, 0.6],
  wrist_roll: [6, 0x16, "jhi", [-145.0, 145.0], 20, 0.6],
  gripper: [7, 0x17, "jhi", [0.0, 137.5], 20, 0.6],
};

const STAR_DIRECTIONS: Record<string, number> = {
  shoulder_pan: -1.0, shoulder_lift: -1.0, elbow_flex: -0.667, wrist_flex: -1.0,
  wrist_yaw: 1.0, wrist_roll: -1.0, gripper: 1.895,
};

type RawJoint = Record<string, unknown> & { limits_deg: [number, number]; model: string; kp: number; kd: number };
const raw = PROFILE as Record<string, unknown> & { joints: RawJoint[] };

describe("product", () => {
  it("identity and protocol limits", () => {
    expect(METAL_ARM.packageName).toBe("@makermods/metal-arm");
    expect(METAL_ARM.displayName).toBe("Metal Arm");
    expect(METAL_ARM.motorVendor).toBe("Damiao");
    expect(METAL_ARM.zeroPose).toBe("folded at rest, as the arm sits unpowered, gripper fully closed");
    expect(METAL_ARM.models).toEqual(["jlo", "j2", "jhi"]);
    expect(METAL_ARM.defaultModel).toBe("jlo");
    expect(METAL_ARM.kpRange).toEqual([0, 500]);
    expect(METAL_ARM.kdRange).toEqual([0, 5]);
    expect(METAL_ARM.idRange).toEqual([1, 15]);
    expect(METAL_ARM.defaultHostId).toBe(0x10);
    expect(METAL_ARM.dreamEmbodiment).toBe("metal");
    expect(METAL_ARM.dreamJointSign).toEqual([1, 1, 1, 1, 1, 1]);
    expect(METAL_ARM.dreamJointOffset).toEqual([0, 0, 0, 0, 0, 0]);
    expect(METAL_ARM.defaultProfile.urdf).toBe(URDF);
  });

  it("routes and creates Damiao motors", () => {
    expect(METAL_ARM.route(0x17, Uint8Array.from([0x17, 0, 0, 0, 0, 0, 0, 0]), 0x10, false)).toBe(7);
    expect(METAL_ARM.route(0x17, Uint8Array.from([0x17, 0, 0, 0, 0, 0, 0, 0]), 0x10, true)).toBeNull();
    const motor = METAL_ARM.createMotor(makeJoint(3, "jhi"), new SimBus([]), 0x10, { now: () => 0, sleep: async () => {}, timer: () => () => {} });
    expect(motor).toBeInstanceOf(Motor);
    expect(motor.torqueLimit).toBe(20);
  });

  it("profile matches the reference", () => {
    expect(raw.joints.map((j) => j.name)).toEqual(Object.keys(REFERENCE));
    expect(raw.required_motor_ids).toEqual([1, 2, 3, 4, 5, 6, 7]);
    expect(raw.host_id).toBe(protocol.DEFAULT_MASTER_OFFSET);
    for (const j of raw.joints) {
      const [motorId, replyId, model, limits, kp, kd] = REFERENCE[j.name as string]!;
      expect([j.motor_id, j.reply_id, j.model, j.kp, j.kd]).toEqual([motorId, replyId, model, kp, kd]);
      expect(j.limits_deg).toEqual(limits);
      expect(j.reply_id).toBe((raw.host_id as number) + (j.motor_id as number));
    }
  });

  it("model scaling", () => {
    expect(protocol.MODELS.jlo).toEqual({ pMax: 6.28, vMax: 10.0, tMax: 30.0 });
    expect(protocol.MODELS.j2).toEqual({ pMax: 6.28, vMax: 10.0, tMax: 120.0 });
    expect(protocol.MODELS.jhi).toEqual({ pMax: 6.28, vMax: 30.0, tMax: 20.0 });
  });

  it("limits fit inside the MIT position range and gains inside the protocol ranges", () => {
    for (const j of raw.joints) {
      const pMax = protocol.MODELS[j.model]!.pMax;
      expect(j.limits_deg.every((v) => Math.abs(radians(v)) <= pMax)).toBe(true);
      expect(protocol.KP_MIN <= j.kp && j.kp <= protocol.KP_MAX).toBe(true);
      expect(protocol.KD_MIN <= j.kd && j.kd <= protocol.KD_MAX).toBe(true);
    }
  });

  it("speed cap and safety settings", () => {
    expect(raw.max_velocity_deg).toBe(280);
    const gripper = raw.joints.at(-1)!;
    const arm = raw.joints.slice(0, -1);
    expect(gripper.can_timeout_ms).toBe(500);
    expect((gripper.can_timeout_ms as number) * protocol.TIMEOUT_TICKS_PER_MS).toBe(10_000);
    expect([gripper.max_temperature_c, gripper.enable_temperature_c]).toEqual([95, 90]);
    expect([gripper.hold_torque_nm, gripper.closing_direction]).toEqual([0.5, -1]);
    expect(gripper.release_on_fault).toBe(true);
    expect(gripper.return_to_rest).toBe(false);
    for (const j of arm) {
      expect("can_timeout_ms" in j).toBe(false);
      expect("max_temperature_c" in j).toBe(false);
    }
  });

  it("the shipped profile loads through defaultConfig()", () => {
    const cfg = defaultConfig();
    expect(cfg).toBeInstanceOf(ArmConfig);
    expect(cfg.product).toBe(METAL_ARM);
    expect(cfg.joints.map((j) => j.motorId)).toEqual([1, 2, 3, 4, 5, 6, 7]);
    expect(cfg.maxVelocity).toBeCloseTo(radians(280), 12);
    expect(cfg.joints.at(-1)!.replyId).toBe(0x17);
    expect(cfg.urdf).toBe(URDF);
    expect(cfg.hostId).toBe(0x10);
  });

  it("defaultConfig applies and validates overrides", () => {
    expect(defaultConfig({ control_rate_hz: 50 }).controlRateHz).toBe(50);
    expect(() => defaultConfig({ control_rate_hz: 0 })).toThrow(ValueError);
    expect(() => defaultConfig({ no_such_field: 1 })).toThrow(ValueError);
  });

  it("leader friction model", () => {
    const cfg = defaultConfig();
    expect(cfg.joints.map((j) => j.frictionViscous)).toEqual([0.21, 0.998, 0.572, 0.294, 0.0045, 0.0105, 0.01]);
    expect(cfg.joints.map((j) => j.frictionVelocityClamp)).toEqual([Infinity, Infinity, 0.8, 1.2, Infinity, Infinity, 3.0]);
    expect(cfg.joints.map((j) => j.frictionCoulomb)).toEqual([0, 0, 0, 0, 0, 0, 0.03]);
    expect(cfg.joints.map((j) => j.frictionRestNm)).toEqual([0, 0, 0, 0, 0, 0, 0.06]);
    expect(cfg.frictionDeadzone).toBe(0.05);
  });

  it("gripper fully open at the end of its stroke", () => {
    const gripper = defaultConfig().joints.at(-1)!;
    expect(gripper.name).toBe("gripper");
    expect(gripper.closingDirection).toBe(-1);
    expect(gripper.openPosition! * (180 / Math.PI)).toBeCloseTo(116.4, 9);
  });

  it("star mapping matches the reference", () => {
    const mapping = METAL_ARM.defaultStarMapping;
    expect(mapping.joints.map((j) => [j.name, j.servo])).toEqual(Object.keys(STAR_DIRECTIONS).map((n, i) => [n, i]));
    for (const j of mapping.joints) {
      expect(j.direction).toBe(STAR_DIRECTIONS[j.name!]);
      const [lo, hi] = REFERENCE[j.name!]![3];
      const expected = j.name === "gripper" ? [0, 115] : [Math.trunc(lo), Math.trunc(hi)];
      expect(j.range_deg).toEqual(expected);
    }
    JointMapper.fromJson(mapping);
  });

  it("profile model is the gravity reference", () => {
    // urdf, signs, TCP, caps and factors in the profile are the ones the gravity fixture's chain holds
    // (the Python SDK's tests/unit/gravity_fixtures.json, copied to contract/fixtures/).
    const spec = JSON.parse(readFileSync(new URL("../../../contract/fixtures/math_metal_arm_gravity_fixtures.json", import.meta.url), "utf8")).chain;
    const cfg = defaultConfig();
    const arm = cfg.joints.slice(0, 6);
    expect(cfg.urdf).toBe(URDF);
    const close = (a: number[], b: number[]) => a.length === b.length && a.every((v, i) => Math.abs(v - b[i]!) <= Math.max(1e-6 * Math.abs(b[i]!), 1e-12));
    expect(close(cfg.urdfBaseRpy.map(degrees), spec.base_rpy_deg)).toBe(true);
    expect(close(cfg.tcpXyz, spec.tcp_xyz)).toBe(true);
    expect(close(cfg.tcpRpy.map(degrees), spec.tcp_rpy_deg)).toBe(true);
    expect(arm.map((j) => j.urdfSign)).toEqual(spec.sign);
    expect(arm.map((j) => degrees(j.urdfOffset))).toEqual(spec.offset_deg);
    expect(arm.map((j) => j.gravityLimitNm)).toEqual(spec.gravity_limit_nm);
    expect(arm.map((j) => j.gravityFactor)).toEqual(spec.gravity_factor);
  });

  it("every profile gravity cap is below the motor's torque range", () => {
    for (const j of defaultConfig().joints.slice(0, 6)) {
      expect(j.gravityLimitNm!).toBeLessThan(protocol.MODELS[j.model]!.tMax);
    }
  });
});
