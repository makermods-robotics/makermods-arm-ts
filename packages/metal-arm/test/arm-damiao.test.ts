// Port of the Metal SDK's tests/unit/test_arm_damiao.py: arm behaviour that needs the Damiao motor,
// an id change that the arm keeps routing afterwards and the late, duplicating gripper.
import { describe, expect, it } from "vitest";
import { ArmState, protocol, RouteUnverified } from "@makermods/metal-arm";
import { FakeMotor, makeArm, twoJointConfig } from "./fakes.js";

function approxAll(actual: number[], expected: number[], abs: number): void {
  expect(actual.length).toBe(expected.length);
  actual.forEach((v, i) => expect(Math.abs(v - expected[i]!), `${v} ≈ ${expected[i]}`).toBeLessThanOrEqual(abs));
}

/** make_arm(...) with Arm.PARAM_TIMEOUT = 0.02 (the Python autouse fixture). */
function make(...args: Parameters<typeof makeArm>) {
  const made = makeArm(...args);
  made.arm.PARAM_TIMEOUT = 0.02;
  return made;
}

describe("arm on Damiao motors", () => {
  it.each([false, true])("setMotorId reroutes the joint to the new id (explicit reply id %s)", async (explicitReplyId) => {
    let joints = twoJointConfig().joints;
    if (explicitReplyId) joints = [joints[0]!.replace({ replyId: 0x11 }), joints[1]!];
    const { arm, fakes } = make(null, { joints, requiredMotorIds: [1, 2] });
    await arm.connect();
    await arm.setMotorId(1, 9);
    expect([fakes[1]!.sendId, fakes[1]!.replyId]).toEqual([9, 0x19]);
    expect(arm.config.joints[0]!.motorId).toBe(9);
    expect(arm.config.requiredMotorIds).toEqual([9, 2]);
    expect(arm.config.joints[0]!.replyId).toBe(explicitReplyId ? 0x19 : null);
    expect(arm.motors[0]!.motorId).toBe(9);
    expect(arm.motors[0]!.replyId).toBe(0x19);
    expect(await arm.refresh()).toEqual([true, true]);
    approxAll(arm.getJointPositions(), [0.3, -0.5], 1e-3);
    await arm.enable({ startLoop: false });
    expect(arm.state).toBe(ArmState.ENABLED);
    expect(fakes[1]!.enabled).toBe(true);
    await arm.disable();
    await arm.disconnect();
  });

  it("setMotorId while disconnected opens and closes the bus", async () => {
    const { arm, bus, fakes } = make([new FakeMotor(1, "jlo", { position: 0.3 }), new FakeMotor(2, "jlo", { position: -0.5 })]);
    await arm.setMotorId(2, 5);
    expect(bus.isOpen).toBe(false);
    expect(fakes[2]!.sendId).toBe(5);
    expect(arm.motors.map((m) => m.motorId)).toEqual([1, 5]);
    await arm.connect();
    approxAll(arm.getJointPositions(), [0.3, -0.5], 1e-3);
    await arm.disconnect();
  });

  it("setMotorId refuses an id that is not a whole number before the bus is opened", async () => {
    const { arm, bus } = make([new FakeMotor(1, "jlo", { position: 0.3 }), new FakeMotor(2, "jlo", { position: -0.5 })]);
    await expect(arm.setMotorId(1, 4.9)).rejects.toThrow(/motor IDs must be/);
    expect(bus.isOpen).toBe(false);
    expect(bus.sent).toEqual([]);
  });

  it("both ids answering after the writes is RouteUnverified and blocks enable and attach (stricter)", async () => {
    // Another motor starts answering on the old id once the new id is saved.
    const source = new FakeMotor(1, "jlo", { position: 0.3 }), occupant = new FakeMotor(5, "jlo", { position: 0.1, online: false });
    const original = source.handle.bind(source);
    source.handle = (canId: number, data: Uint8Array) => {
      if (canId === protocol.PARAM_CAN_ID && data[2] === protocol.OP_SAVE) {
        occupant.sendId = 1;
        occupant.replyId = protocol.DEFAULT_MASTER_OFFSET + 1;
        occupant.registers[protocol.Reg.ESC_ID] = 1;
        occupant.registers[protocol.Reg.MST_ID] = protocol.DEFAULT_MASTER_OFFSET + 1;
        occupant.online = true;
      }
      return original(canId, data);
    };
    const { arm } = make([source, new FakeMotor(2, "jlo", { position: -0.5 }), occupant]);
    await expect(arm.setMotorId(1, 9)).rejects.toThrow(RouteUnverified);
    expect(arm.config.joints[0]!.motorId).toBe(1); // not routed to the new id
    await arm.connect();
    await expect(arm.enable()).rejects.toThrow(/unverified/);
    await expect(arm.attach()).rejects.toThrow(/unverified/);
    expect(source.enabled || occupant.enabled).toBe(false);
    await arm.disconnect();
  });

  it("enable and disable with the late duplicating gripper", async () => {
    // The gripper withholds each reply until another frame is on the bus, then sends it twice.
    const { arm, fakes } = make([new FakeMotor(1, "jlo", { position: 0.3 }),
      new FakeMotor(2, "jlo", { position: -0.5, late: true, duplicate: true })]);
    await arm.connect();
    await arm.enable({ startLoop: false });
    expect(arm.state).toBe(ArmState.ENABLED);
    expect(fakes[1]!.enabled && fakes[2]!.enabled).toBe(true);
    expect(arm.getFaults()).toEqual([0, 0]); // status "enabled" is not a fault code
    await arm.disable();
    expect(arm.unconfirmedDisable).toEqual([]);
    expect(fakes[1]!.enabled || fakes[2]!.enabled).toBe(false);
  });
});
