// Port of the Python SDK's tests/unit/test_config.py. The YAML snippets are raw profile objects with the
// same snake_case keys, read by ArmConfig.fromRaw().
import { describe, expect, it } from "vitest";
import { ArmConfig, JointConfig, type JointConfigInit, radians, ValueError } from "@makermods/arm-core";
import { type ArmFakes, type FakeMotorLike } from "./fakes.js";

/** Registers the shared tests on `fk`'s arm. */
export function configSuite<M extends FakeMotorLike>(fk: ArmFakes<M>): void {
  const PRODUCT = fk.product;
  const ID_LO = PRODUCT.idRange[0];
  const KD_HI = PRODUCT.kdRange[1];

  /** pytest.approx with its default tolerance. */
  function approx(actual: number, expected: number): void {
    expect(Math.abs(actual - expected), `${actual} ≈ ${expected}`).toBeLessThanOrEqual(
      Math.max(1e-6 * Math.abs(expected), 1e-12));
  }

  /** Python JointConfig(**kw): model defaults to the product's. */
  const jointConfig = (kw: Omit<JointConfigInit, "model"> & { model?: string }): JointConfig =>
    new JointConfig({ model: PRODUCT.defaultModel, ...kw });

  describe("config", () => {
    it("limits in degrees and deg suffixed settings", () => {
      const cfg = ArmConfig.fromRaw({
        startup_step_deg: 2.0,
        joints: [{ motor_id: 1, limits_deg: [-90, 45], kp: 10, kd: 1 }],
      }, PRODUCT);
      approx(cfg.joints[0]!.lo, -Math.PI / 2);
      approx(cfg.joints[0]!.hi, Math.PI / 4);
      approx(cfg.startupStep, radians(2.0));
    });

    it("unknown field is reported", () => {
      const raw = { wire_protocol: "private", joints: [] };
      expect(() => ArmConfig.fromRaw(raw, PRODUCT)).toThrow(ValueError);
      expect(() => ArmConfig.fromRaw(raw, PRODUCT)).toThrow(/unrecognized/);
    });

    it.each([
      [{ motorId: ID_LO - 1, lo: -1, hi: 1, kp: 1, kd: 1 }, "motor_id"],
      [{ motorId: 1, lo: -1, hi: 1, kp: 1, kd: KD_HI + 1 }, "kd"],
      [{ motorId: 1, lo: -1, hi: 1, kp: 1, kd: 1, model: "NO-SUCH-MODEL" }, "unknown model"],
      [{ motorId: 1, lo: 1, hi: -1, kp: 1, kd: 1 }, "limits"],
      [{ motorId: 1, lo: -1, hi: 1, kp: 1, kd: 1, canTimeoutMs: 0 }, "can_timeout_ms"],
      [{ motorId: 1, lo: -1, hi: 1, kp: 1, kd: 1, maxTemperatureC: 50, enableTemperatureC: 50 }, "enable_temperature_c"],
    ] as const)("validation %#: %s -> %s", (joint, message) => {
      const cfg = new ArmConfig({ product: PRODUCT, joints: [jointConfig(joint)] });
      expect(() => cfg.validate()).toThrow(ValueError);
      expect(() => cfg.validate()).toThrow(new RegExp(message));
    });

    it("required ids in order", () => {
      const joints = [2, 1].map((i) => jointConfig({ motorId: i, lo: -1, hi: 1, kp: 1, kd: 1 }));
      const cfg = new ArmConfig({ product: PRODUCT, requiredMotorIds: [1, 2], joints });
      expect(() => cfg.validate()).toThrow(ValueError);
      expect(() => cfg.validate()).toThrow(/exactly/);
    });

    it("rest pose in degrees one value per joint", () => {
      const cfg = ArmConfig.fromRaw({
        rest_pose_deg: [10],
        joints: [{ motor_id: 1, limits_deg: [-90, 45], kp: 10, kd: 1 }],
      }, PRODUCT);
      expect(cfg.restPose!.length).toBe(1);
      approx(cfg.restPose![0]!, radians(10));
      const bad = new ArmConfig({ product: PRODUCT, restPose: [0.0, 0.0], joints: cfg.joints });
      // Python matches "rest_pose"; the TS message names the field restPose.
      expect(() => bad.validate()).toThrow(ValueError);
      expect(() => bad.validate()).toThrow(/rest_?pose/i);
    });

    it("gripper open position in degrees inside the limits", () => {
      const cfg = ArmConfig.fromRaw({
        joints: [{
          name: "gripper", motor_id: 1, limits_deg: [0, 120], kp: 10, kd: 1, closing_direction: -1, open_position_deg: 100,
        }],
      }, PRODUCT);
      approx(cfg.joints[0]!.openPosition!, radians(100));
      for (const bad of [1.5, -0.1, 0.0, NaN]) { // outside the limits, or the closed end
        const c = new ArmConfig({
          product: PRODUCT,
          joints: [jointConfig({ motorId: 1, lo: 0.0, hi: 1.0, kp: 1, kd: 1, closingDirection: -1, openPosition: bad })],
        });
        expect(() => c.validate(), `open_position ${bad}`).toThrow(ValueError);
        expect(() => c.validate(), `open_position ${bad}`).toThrow(/open_position/);
      }
    });
  });
}
