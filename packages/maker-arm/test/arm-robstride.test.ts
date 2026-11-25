// Port of the Python SDK's tests/unit/test_arm_robstride.py: arm behaviour that depends on RobStride
// specifics (parameter-capable firmware, mode bits, RUN_MODE, the CAN-timeout watchdog and the
// separate fault poll).
import { afterEach, describe, expect, it, vi } from "vitest";
import { type ArmConfigInit, ArmState, ConnectError, protocol, RobStrideMotor, StateError } from "@makermods/maker-arm";
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

  it("the status deadline counts from the first unanswered query", async () => {
    const { arm, fakes, clock } = await enabledArm();
    const answer = fakes[1]!.handle.bind(fakes[1]!);
    fakes[1]!.handle = (canId, data) => (classify(canId, data)[0] === "status_query" ? [] : answer(canId, data));
    (arm.motors[0] as unknown as RobStrideMotor).statusTime = clock.now() - 5.0; // a slow enable: the last reply is old
    await arm["tick"](0.005);
    expect(arm.state).toBe(ArmState.ENABLED); // nothing was asked yet in this session
    arm["pollSent"] = new Map([[1, 0.0], [2, 1e9]]);
    await arm["tick"](0.005); // asks motor 1, which never answers
    expect(arm.state).toBe(ArmState.ENABLED);
    arm["awaitingStatus"][0]! -= 1.0; // ... for a second
    await arm["tick"](0.005);
    expect(arm.state).toBe(ArmState.FAULT);
    expect(arm.faultReason).toContain("no fault-status reply");
  });

  it("status queries that fail to send still count as unanswered", async () => {
    const { arm, clock } = await enabledArm();
    arm.motors[0]!.pollStatus = () => Promise.reject(new Error("send failed"));
    (arm.motors[0] as unknown as RobStrideMotor).statusTime = clock.now() - 5.0; // its last status reply is old
    arm["pollSent"] = new Map([[1, 0.0], [2, 1e9]]);
    await arm["tick"](0.005); // the query to motor 1 fails to go out
    expect(arm.state).toBe(ArmState.ENABLED);
    const deadline = 2 * arm.config.faultPollInterval + arm.config.feedbackTimeout;
    arm["awaitingStatus"][0]! -= deadline + 0.1; // ... and nothing reached it since
    await arm["tick"](0.005);
    expect(arm.state).toBe(ArmState.FAULT);
    expect(arm.faultReason).toContain("no fault-status reply");
  });

  it("a loop that could not poll is not a missing status reply", async () => {
    const { arm, clock } = await enabledArm();
    arm["pollSent"] = new Map([[1, 0.0], [2, 1e9]]);
    await arm["tick"](0.005); // motor 1 answers this query
    (arm.motors[0] as unknown as RobStrideMotor).statusTime = clock.now() - 5.0; // then the loop did not run for five seconds
    await arm["tick"](0.005);
    expect(arm.state).toBe(ArmState.ENABLED);
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

  it("an id that is not a whole number is refused at the arm, the motor and the protocol", async () => {
    const { arm, bus } = make();
    await expect(arm.setMotorId(1, 42.9, { torqueOffConfirmed: true })).rejects.toThrow(/motor IDs must be/);
    expect(bus.isOpen).toBe(false);
    expect(bus.sent).toEqual([]);
    await expect(arm.motors[0]!.setId(42.9, 0.5, true)).rejects.toThrow(/motor id must be/);
    expect(() => protocol.encodeSetId(1, 42.9)).toThrow(/MIT motor ID/);
    expect(bus.sent).toEqual([]);
  });

  it("setMotorId changes the motor and routes the joint to it", async () => {
    const { arm, bus, fakes: motors } = make();
    await arm.setMotorId(1, 9, { torqueOffConfirmed: true });
    expect(motors[1]!.motorId).toBe(9);
    expect(bus.isOpen).toBe(false);
    expect(arm.config.joints[0]!.motorId).toBe(9);
    await arm.setMotorId(9, 1, { torqueOffConfirmed: true });
    expect(motors[1]!.motorId).toBe(1);
    expect(arm.config.joints[0]!.motorId).toBe(1);
  });

  const idWrites = (bus: { sent: Array<[number, Uint8Array]> }): number[] =>
    bus.sent.filter(([, d]) => d.subarray(0, 6).every((b) => b === 0xff) && d[7] === 0xfa).map(([id]) => id);

  it("setMotorId refuses an id another joint has in the profile, even when it is offline", async () => {
    const { arm, bus, fakes: motors } = make();
    motors[2]!.online = false;
    await expect(arm.setMotorId(1, 2, { torqueOffConfirmed: true })).rejects.toThrow(/belongs to/);
    expect(idWrites(bus)).toEqual([]);
    expect(motors[1]!.motorId).toBe(1);
  });

  it("setMotorId reads the motor before writing", async () => {
    // The arm builds a fresh Motor, which has no feedback until setId reads it.
    const { arm, bus, fakes: motors } = make([new FakeMotor(1, "RS00", { position: 0.3 }), new FakeMotor(2, "RS00", { position: -0.5 })]);
    await expect(arm.setMotorId(1, 9)).rejects.toThrow(StateError); // old firmware: torque state unknown
    await expect(arm.setMotorId(1, 9)).rejects.toThrow(/torqueOffConfirmed|torque_off_confirmed/);
    motors[1]!.torque = 0.3; // holding torque: energized
    await expect(arm.setMotorId(1, 9, { torqueOffConfirmed: true })).rejects.toThrow(/energized/);
    motors[1]!.torque = 0.0;
    motors[1]!.paramsSupported = true;
    motors[1]!.enabled = true; // energized by another program
    await expect(arm.setMotorId(1, 9, { torqueOffConfirmed: true })).rejects.toThrow(/torque on/);
    expect(idWrites(bus)).toEqual([]);
    expect(motors[1]!.motorId).toBe(1);
  });

  /** A motor whose id write is ignored, while a motor silent before answers on the target id after it. */
  function ambiguous(source: FakeMotor, occupant: FakeMotor): void {
    source.applyIdChange = false;
    const original = source.handle.bind(source);
    source.handle = (canId: number, data: Uint8Array) => {
      if (data.subarray(0, 6).every((b) => b === 0xff) && data[7] === 0xfa) occupant.online = true;
      return original(canId, data);
    };
  }

  it("a set-id that leaves both ids answering keeps the joint from enabling", async () => {
    const source = new FakeMotor(1, "RS00", { position: 0.3 }), occupant = new FakeMotor(9, "RS00", { online: false });
    ambiguous(source, occupant);
    const { arm } = make([source, new FakeMotor(2, "RS00", { position: -0.5 }), occupant]);
    await expect(arm.setMotorId(1, 9, { torqueOffConfirmed: true })).rejects.toThrow(/both answer/);
    await arm.connect();
    await expect(arm.enable()).rejects.toThrow(ConnectError);
    await expect(arm.enable()).rejects.toThrow(/unverified/);
    await expect(arm.attach()).rejects.toThrow(/unverified/);
    expect(source.enabled).toBe(false);
    await arm.disconnect();
  });

  it("both ids answering after a replacement blocks the replaced joint too", async () => {
    const source = new FakeMotor(9, "RS00", { position: 0.3 }), occupant = new FakeMotor(2, "RS00", { position: -0.5, online: false });
    ambiguous(source, occupant);
    const { arm } = make([new FakeMotor(1, "RS00", { position: 0.3 }), source, occupant]);
    await expect(arm.setMotorId(9, 2, { torqueOffConfirmed: true, replacing: "gripper" })).rejects.toThrow(/both answer/);
    await arm.connect();
    await expect(arm.enable()).rejects.toThrow(/unverified/);
    await arm.disconnect();
  });

  it("an arm made after an ambiguous set-id is not refused", async () => {
    const source = new FakeMotor(1, "RS00", { position: 0.3 }), occupant = new FakeMotor(9, "RS00", { online: false });
    ambiguous(source, occupant);
    const made = make([source, new FakeMotor(2, "RS00", { position: -0.5 }), occupant]);
    await expect(made.arm.setMotorId(1, 9, { torqueOffConfirmed: true })).rejects.toThrow(/both answer/);
    expect(made.arm["unverified"].size).toBe(1);
    const fresh = new (made.arm.constructor as typeof import("@makermods/maker-arm").Arm)(made.arm.config, made.bus,
      { clock: made.clock, logger: made.logger, frameSpacing: 0 });
    expect(fresh["unverified"].size).toBe(0);
  });
});
