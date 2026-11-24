// Port of the Python SDK's tests/unit/test_arm_robstride.py: arm behaviour that depends on RobStride
// specifics (parameter-capable firmware, mode bits, RUN_MODE, the CAN-timeout watchdog and the
// separate fault poll).
import { afterEach, describe, expect, it, vi } from "vitest";
import { type ArmConfigInit, ArmState, ConnectError, RobStrideMotor } from "@makermods/maker-arm";
import { VirtualClock } from "@makermods/maker-arm/testing";
import { classify, FakeMotor, makeArm } from "./fakes.js";

// Python's monotonic clock is far from zero; the tests' 0.0 "sent long ago" sentinels rely on it.
function make(motors?: FakeMotor[] | null, overrides: Partial<ArmConfigInit> = {}) {
  return makeArm(motors, overrides, new VirtualClock({ start: 1000 }));
}

async function enabledArm(motors?: FakeMotor[] | null, overrides: Partial<ArmConfigInit> = {}) {
  const made = make(motors, overrides);
  await made.arm.connect();
  await made.arm.enable({ startLoop: false });
  return made;
}

const count = (log: FakeMotor["log"], entry: string): number => log.filter((e) => e === entry).length;

afterEach(() => {
  vi.restoreAllMocks();
});

describe("RobStride arm behaviour", () => {
  it("connect refuses a motor not in MIT control mode", async () => {
    const { arm } = make([new FakeMotor(1), new FakeMotor(2, "RS00", { position: -0.5, paramsSupported: true, runMode: 1 })]);
    const p = arm.connect();
    await expect(p).rejects.toThrow(ConnectError);
    await expect(p).rejects.toThrow(/RUN_MODE/);
  });

  it("the gripper watchdog is armed only with parameter support, and restored", async () => {
    const gripper = new FakeMotor(2, "RS00", { position: -0.5, paramsSupported: true, canTimeout: 0 });
    const { arm, fakes } = make([new FakeMotor(1, "RS00", { paramsSupported: true }), gripper]);
    await arm.connect();
    await arm.enable({ startLoop: false });
    expect(gripper.canTimeout).toBe(10000); // 500 ms
    expect(fakes[1]!.canTimeout).toBe(0); // joints: no watchdog
    await arm.disable();
    expect(gripper.canTimeout).toBe(0);
  });

  it("the watchdog restore of a zero original still happens", async () => {
    const gripper = new FakeMotor(2, "RS00", { position: -0.5, paramsSupported: true, canTimeout: 0 });
    const { arm } = make([new FakeMotor(1), gripper]);
    await arm.connect();
    await arm.enable({ startLoop: false });
    expect(arm["watchdogRestore"]).toEqual(new Map([[1, 0]]));
    await arm.disable();
    expect(gripper.canTimeout).toBe(0);
    expect(arm["watchdogRestore"].size).toBe(0);
  });

  it("old firmware runs without a watchdog", async () => {
    // caplog: the motor logs through console (the product's createMotor gets no logger), the arm through its logger.
    const warn = vi.spyOn(console, "warn").mockImplementation(() => {});
    const { arm, logger } = make();
    await arm.connect();
    await arm.enable({ startLoop: false });
    expect(arm.state).toBe(ArmState.ENABLED);
    const messages = [...logger.messages.map(([, m]) => m), ...warn.mock.calls.map((c) => String(c[0]))];
    expect(messages.some((m) => m.includes("no parameter access"))).toBe(true);
  });

  it("enable is rejected when the mode does not reach run", async () => {
    class Stubborn extends FakeMotor {
      override get mode(): number {
        return 0;
      }
    }

    const { arm, fakes } = make([new FakeMotor(1, "RS00", { paramsSupported: true }),
      new Stubborn(2, "RS00", { position: -0.5, paramsSupported: true })]);
    await arm.connect();
    const p = arm.enable({ startLoop: false });
    await expect(p).rejects.toThrow(ConnectError);
    await expect(p).rejects.toThrow(/enable rejected/);
    expect(fakes[1]!.enabled).toBe(false);
  });

  it("tick polls faults without clearing", async () => {
    const { arm, fakes } = await enabledArm();
    arm["pollSent"] = new Map([[1, 0.0], [2, 0.0]]);
    fakes[1]!.log.length = 0;
    fakes[2]!.log.length = 0;
    await arm["tick"](0.005);
    expect(count(fakes[1]!.log, "status_query")).toBe(1);
    expect(fakes[2]!.log).not.toContain("status_query");
    await arm["tick"](0.005);
    expect(count(fakes[2]!.log, "status_query")).toBe(1); // one motor per tick, round-robin
    expect([...fakes[1]!.log, ...fakes[2]!.log]).not.toContain("clear");
  });

  it("the status deadline counts from when the loop took over", async () => {
    const { arm, fakes, clock } = await enabledArm();
    const answer = fakes[1]!.handle.bind(fakes[1]!);
    fakes[1]!.handle = (canId, data) => (classify(canId, data)[0] === "status_query" ? [] : answer(canId, data));
    (arm.motors[0] as unknown as RobStrideMotor).statusTime = clock.now() - 5.0; // a slow enable: the last reply is old
    await arm["tick"](0.005);
    expect(arm.state).toBe(ArmState.ENABLED);
    arm["pollingSince"] -= 1.0; // the loop has been asking for a second
    await arm["tick"](0.005);
    expect(arm.state).toBe(ArmState.FAULT);
    expect(arm.faultReason).toContain("no fault-status reply");
  });

  it("modern firmware leaving run mode faults", async () => {
    const { arm, fakes } = await enabledArm([new FakeMotor(1, "RS00", { paramsSupported: true }), new FakeMotor(2, "RS00", { position: -0.5 })]);
    for (let k = 0; k < RobStrideMotor.MODE_FAULT_TICKS - 1; k++) {
      fakes[1]!.enabled = false; // motor dropped out of run mode by itself
      await arm["tick"](0.005);
    }
    expect(arm.state).toBe(ArmState.ENABLED);
    await arm["tick"](0.005);
    expect(arm.state).toBe(ArmState.FAULT);
    expect(arm.faultReason).toContain("left run mode");
  });

  it("the mode count restarts at every enable", async () => {
    const { arm, fakes } = await enabledArm([new FakeMotor(1, "RS00", { paramsSupported: true }), new FakeMotor(2, "RS00", { position: -0.5 })]);
    fakes[1]!.enabled = false;
    for (let k = 0; k < RobStrideMotor.MODE_FAULT_TICKS - 1; k++) await arm["tick"](0.005);
    await arm.disable();
    await arm.enable({ startLoop: false });
    fakes[1]!.enabled = false;
    await arm["tick"](0.005);
    expect(arm.state).toBe(ArmState.ENABLED);
  });

  it("an unconfirmed disable keeps the gripper watchdog", async () => {
    const DISABLE = Uint8Array.from([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd]);

    class DropsDisable extends FakeMotor {
      override handle(canId: number, data: Uint8Array): Array<[number, Uint8Array]> {
        if (data.length === DISABLE.length && data.every((b, i) => b === DISABLE[i])) return [];
        return super.handle(canId, data);
      }
    }

    const gripper = new DropsDisable(2, "RS00", { position: -0.5, paramsSupported: true });
    const { arm } = make([new FakeMotor(1, "RS00", { paramsSupported: true }), gripper]);
    await arm.connect();
    await arm.enable({ startLoop: false });
    await arm.disable();
    expect(arm.unconfirmedDisable).toEqual([2]);
    expect(gripper.canTimeout).toBe(10000); // not removed while the gripper may still be enabled
  });

  it("a watchdog read-back mismatch refuses enable and restores", async () => {
    class Clamps extends FakeMotor {
      override handle(canId: number, data: Uint8Array): Array<[number, Uint8Array]> {
        const replies = super.handle(canId, data);
        if ((canId & 0x700) === 0x400) this.canTimeout = 7; // firmware stores something else
        return replies;
      }
    }

    const gripper = new Clamps(2, "RS00", { position: -0.5, paramsSupported: true, canTimeout: 123 });
    const { arm } = make([new FakeMotor(1), gripper]);
    await arm.connect();
    const p = arm.enable({ startLoop: false });
    await expect(p).rejects.toThrow(ConnectError);
    await expect(p).rejects.toThrow(/read back/);
    expect(gripper.log).toContainEqual(["write", 0x7028]);
    expect(gripper.log).not.toContain("enable");
  });

  it("setMotorId is refused without a frame", async () => {
    const { arm, bus } = make();
    await expect(arm.setMotorId(1, 9)).rejects.toThrow(/Motor Studio/);
    for (const [, data] of bus.sent) {
      expect([...data.subarray(0, 6)]).toEqual([0xff, 0xff, 0xff, 0xff, 0xff, 0xff]);
      expect([...data.subarray(6)]).toEqual([0x00, 0xfb]);
    }
    expect(bus.isOpen).toBe(false);
  });
});
