// Behaviour the Python SDK gets from its threads and locks, checked on the async port.
import { describe, expect, it } from "vitest";
import { ArmConfig, ArmState, StateError, ValueError } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import type { ArmFakes, FakeMotorLike, FakeMotorOptions } from "./fakes.js";

/** Registers the tests on `fk`'s arm. */
export function safetySuite<M extends FakeMotorLike>(fk: ArmFakes<M>): void {
  const MODEL = fk.product.defaultModel;
  const { classify, FakeMotor, makeArm } = fk;
  const Base = FakeMotor as unknown as new (motorId: number, model?: string, options?: FakeMotorOptions) => FakeMotorLike;
  const make = (motors?: M[] | null) => makeArm(motors, {}, new VirtualClock({ start: 1000 }));

  describe(`${fk.name}: e-stop and enable`, () => {
    it("an e-stop made right after enable() cancels the enable before any motor is energized", async () => {
      const { arm, bus, fakes } = make();
      await arm.connect();
      const sent = bus.sent.length;
      const enabling = arm.enable({ startLoop: false });
      const stopping = arm.estop();
      await expect(enabling).rejects.toThrow(StateError);
      await stopping;
      expect(arm.state).toBe(ArmState.CONNECTED);
      expect(Object.values(fakes).every((m) => !m.enabled)).toBe(true);
      expect(bus.sent.slice(sent).some(([id, data]) => classify(id, data)[0] === "enable")).toBe(false);
    });

    it("no enable frame follows the e-stop's torque-off, even when the motor did not answer its enable", async () => {
      let estopped: Promise<void> | null = null;

      class SilentFirstEnable extends Base {
        arm: { estop(): Promise<void> } | null = null;
        enables = 0;

        override handle(canId: number, data: Uint8Array): Array<[number, Uint8Array]> {
          if (classify(canId, data)[0] === "enable" && this.enables++ === 0) {
            estopped = this.arm!.estop();
            return []; // the reply is lost, so a protocol that retries its enable would send it again
          }
          return super.handle(canId, data);
        }
      }

      const motor2 = new SilentFirstEnable(2, MODEL, { position: -0.5 });
      const { arm, bus, fakes, clock } = make([new FakeMotor(1, MODEL, { position: 0.3 }), motor2 as unknown as M]);
      motor2.arm = arm;
      await arm.connect();
      const sent = bus.sent.length;
      await expect(arm.enable({ startLoop: false })).rejects.toThrow();
      await clock.sleep(0.5);
      await estopped;
      const kinds = bus.sent.slice(sent).map(([id, data]) => `${classify(id, data)[0]}:${classify(id, data)[1]}`);
      const firstDisable = kinds.findIndex((k) => k.startsWith("disable"));
      expect(firstDisable).toBeGreaterThanOrEqual(0);
      expect(kinds.slice(firstDisable).filter((k) => k.startsWith("enable"))).toEqual([]);
      expect(arm.state).toBe(ArmState.CONNECTED);
      expect(fakes[1]!.enabled).toBe(false);
      expect(fakes[2]!.enabled).toBe(false);
    });
  });

  describe(`${fk.name}: targets`, () => {
    it("targets with a hole or a non-number are refused", async () => {
      const { arm } = make();
      await arm.connect();
      await arm.enable({ startLoop: false });
      const before = arm.getCommandedPositions();
      const holed = [0.1, 0.1];
      delete holed[0];
      expect(arm.setJointTargets(holed)).toBe(false);
      expect(arm.setJointTargets(["0.1", 0.1] as unknown as number[])).toBe(false);
      expect(arm.setJointTargets([NaN, 0.1])).toBe(false);
      expect(await arm.moveTo(holed, { wait: false })).toBe(false);
      expect(await arm.moveTo([null, 0.1] as unknown as number[], { wait: false })).toBe(false);
      expect(arm.getCommandedPositions()).toEqual(before);
      await arm.disable();
    });

    it("a rest pose with a missing value is refused, and the arm keeps holding", async () => {
      const { arm, clock } = make();
      await arm.connect();
      await arm.enable();
      await clock.sleep(0.1);
      const rest = [0.3, -0.5];
      delete rest[1];
      expect(await arm.returnToRest(rest)).toBe(false);
      expect(await arm.returnToRest([0.3, NaN])).toBe(false);
      expect(arm.state).toBe(ArmState.ENABLED);
      await arm.disable();
    });
  });
}

/** Registers the profile type checks on `fk`'s shipped profile. */
export function profileTypeSuite<M extends FakeMotorLike>(fk: ArmFakes<M>): void {
  describe(`${fk.name}: profile types`, () => {
    it.for([
      ["limit_margin", "0.05"],
      ["max_velocity", "1.5"],
      ["feedback_timeout", null],
      ["hold_on_fault", "false"],
    ] as const)("a top-level %s of the wrong type is refused", ([key, value]) => {
      const raw = { ...fk.product.defaultProfile, [key]: value };
      expect(() => ArmConfig.fromRaw(raw, fk.product)).toThrow(ValueError);
    });

    it("a vector with a hole is refused", () => {
      const holed = [0, 0, 0];
      delete holed[1];
      expect(() => ArmConfig.fromRaw({ ...fk.product.defaultProfile, tcp_xyz: holed }, fk.product)).toThrow(ValueError);
    });

    it.for([
      ["kp", "20"],
      ["limits_deg", ["-10", "10"]],
      ["offset", "0"],
      ["max_temperature_c", "55"],
      ["release_on_fault", "no"],
    ] as const)("a joint %s of the wrong type is refused", ([key, value]) => {
      const joints = (fk.product.defaultProfile.joints as Array<Record<string, unknown>>).map((j, i) => (i === 0 ? { ...j, [key]: value } : j));
      expect(() => ArmConfig.fromRaw({ ...fk.product.defaultProfile, joints }, fk.product)).toThrow(ValueError);
    });
  });
}
