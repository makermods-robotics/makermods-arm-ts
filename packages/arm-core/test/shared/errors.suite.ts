// Port of the Python SDK's tests/unit/test_errors.py.
import { describe, expect, it } from "vitest";
import { ArmError, ConnectError, FaultLatched, ParamTimeout, StateError } from "@makermods/arm-core";
import { type ArmFakes, type FakeMotorLike } from "./fakes.js";

/** Registers the shared tests on `fk`'s arm. */
export function errorsSuite<M extends FakeMotorLike>(_fk: ArmFakes<M>): void {
  describe("errors", () => {
    it("hierarchy", () => {
      for (const cls of [ConnectError, ParamTimeout, StateError, FaultLatched]) {
        expect(cls.prototype).toBeInstanceOf(ArmError);
        expect(new cls("x")).toBeInstanceOf(ArmError);
      }
    });
  });
}
