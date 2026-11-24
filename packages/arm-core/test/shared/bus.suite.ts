// Port of the Python SDK's tests/unit/test_bus.py: release(), verified torque-off motor by motor, where
// Arm.connect refuses.
import { describe, expect, it } from "vitest";
import { ConnectError, release } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import { type ArmFakes, type FakeMotorLike, type FakeMotorOptions } from "./fakes.js";

/** Registers the shared tests on `fk`'s arm. */
export function busSuite<M extends FakeMotorLike>(fk: ArmFakes<M>): void {
  const MODEL = fk.product.defaultModel;
  const { FakeBus, FakeMotor, makeArm, twoJointConfig } = fk;
  function motors(second: FakeMotorOptions = {}): M[] {
    return [
      new FakeMotor(1, MODEL, { position: 0.3, paramsSupported: true, enabled: true, faulted: true }),
      new FakeMotor(2, MODEL, { position: -0.5, paramsSupported: true, enabled: true, ...second }),
    ];
  }

  describe("release", () => {
    it("release turns off an arm with a latched fault", async () => {
      const fakes = motors();
      // makeArm sets PARAM_TIMEOUT = 0.005 like the Python autouse fixture.
      const { arm } = makeArm(fakes);
      await expect(arm.connect()).rejects.toThrow(ConnectError);
      const result = await release(new FakeBus(fakes), twoJointConfig(), new VirtualClock());
      expect(result).toEqual({ ok: true, released: ["j1", "gripper"], unconfirmed: [], missing: [] });
      expect(fakes.some((m) => m.enabled)).toBe(false);
    });

    it("release reports a missing motor and releases the rest", async () => {
      const fakes = motors({ online: false });
      const result = await release(new FakeBus(fakes), twoJointConfig(), new VirtualClock());
      expect(!result.ok && JSON.stringify(result.missing) === '["gripper"]').toBe(true);
      expect(result.released).toEqual(["j1"]);
      expect(fakes[0]!.enabled).toBe(false);
    });

    it("release of a silent bus is not ok", async () => {
      const fakes = [new FakeMotor(1, MODEL, { online: false }), new FakeMotor(2, MODEL, { online: false })];
      const result = await release(new FakeBus(fakes), twoJointConfig(), new VirtualClock());
      expect(result.ok).toBe(false);
      expect(result.missing).toEqual(["j1", "gripper"]);
      expect(result.released).toEqual([]);
    });
  });
}
