// Port of maker-arm-sdk tests/unit/test_gripper.py. arm-core never imports an arm package, so the
// product's Kp range is passed explicitly.
import { describe, expect, it } from "vitest";
import { CAPPED_CLOSING_KD, GripperHold, radians, ValueError } from "@makermods/arm-core";

const LIMITS: [number, number] = [radians(-96.0), radians(-2.5)];
const KP_RANGE: [number, number] = [0.0, 500.0];

describe("GripperHold", () => {
  it("free closing scales kp to the cap and lowers kd", () => {
    const hold = new GripperHold(0.5, LIMITS, KP_RANGE);
    const sent = hold.update(-0.05, -1.0, 0.0, 0.0, 20.0, 0.0);
    expect(Math.abs(sent - -0.05)).toBeLessThanOrEqual(Math.max(1e-6 * 0.05, 1e-12));
    const [kp, kd] = hold.gains(20.0, 0.5);
    expect(kp * (sent - -1.0)).toBeLessThanOrEqual(hold.closingCapNm + 1e-9);
    expect(kd).toBe(CAPPED_CLOSING_KD);
    expect(hold.closingCapped).toBe(true);
  });

  it("opening uses full gains", () => {
    const hold = new GripperHold(0.5, LIMITS, KP_RANGE);
    hold.update(-1.0, -0.1, 0.0, 0.0, 20.0, 0.0);
    expect(hold.gains(20.0, 0.5)).toEqual([20.0, 0.5]);
  });

  it("contact switches to holding regulation", () => {
    const hold = new GripperHold(0.5, LIMITS, KP_RANGE);
    let t = 0.0;
    let sent = NaN;
    for (let i = 0; i < 10; i++) { // blocked: closing effort 0.8 Nm, not moving, for > 100 ms
      sent = hold.update(-0.05, -0.5, 0.0, 0.8, 20.0, t);
      t += 0.02;
    }
    expect(hold.holding).toBe(true);
    // Effort above target: the command relaxes back toward the measured position.
    expect(sent).toBeLessThan(-0.05);
  });

  it("opening releases the hold", () => {
    const hold = new GripperHold(0.5, LIMITS, KP_RANGE);
    let t = 0.0;
    for (let i = 0; i < 10; i++) {
      hold.update(-0.05, -0.5, 0.0, 0.8, 20.0, t);
      t += 0.02;
    }
    hold.update(-1.0, -0.5, 0.0, 0.8, 20.0, t);
    expect(hold.holding).toBe(false);
  });

  it.each([0.0, 3.0, NaN])("torque range: %s", (torque) => {
    expect(() => new GripperHold(torque, LIMITS, KP_RANGE)).toThrow(ValueError);
  });
});
