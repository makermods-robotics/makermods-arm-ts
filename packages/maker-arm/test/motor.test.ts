// Port of maker-arm-sdk tests/unit/test_motor.py.
import { describe, expect, it } from "vitest";
import {
  ArmConfig, ArmError, ConnectError, DEFAULT_HOST_ID, DEFAULT_MODEL, FaultLatched, ID_RANGE, JointConfig, KD_RANGE,
  KP_RANGE, type Logger, MAKER_ARM, MODELS, PARAMS, ParamTimeout, protocol as p, RobStrideMotor, RouteUnverified, route,
  StateError, ValueError,
} from "@makermods/maker-arm";
import { VirtualClock } from "@makermods/maker-arm/testing";
import { FakeBus, FakeMotor, feedbackBytes } from "./fakes.js";

const silent: Logger = { info() {}, warn() {}, error() {} };

/** pytest.approx: |a - b| <= max(rel * |b|, abs); abs alone disables rel. */
function approx(actual: number, expected: number, opts: { abs?: number; rel?: number } = {}): void {
  const tol = opts.abs !== undefined && opts.rel === undefined
    ? opts.abs : Math.max((opts.rel ?? 1e-6) * Math.abs(expected), opts.abs ?? 1e-12);
  expect(Math.abs(actual - expected), `${actual} ≈ ${expected}`).toBeLessThanOrEqual(tol);
}

/** struct.pack("<H2xI", index, value). */
function paramBytes(index: number, value: number): Uint8Array {
  const out = new Uint8Array(8);
  const view = new DataView(out.buffer);
  view.setUint16(0, index, true);
  view.setUint32(4, value, true);
  return out;
}

function joint(motorId: number, model = "RS00", extra: { name?: string } = {}): JointConfig {
  return new JointConfig({ motorId, lo: -1.0, hi: 1.0, kp: 10.0, kd: 1.0, model, ...extra });
}

function wired(fake: FakeMotor): [RobStrideMotor, FakeBus] {
  const bus = new FakeBus([fake]);
  const motor = new RobStrideMotor(joint(fake.motorId, fake.model, { name: "j" }), bus, DEFAULT_HOST_ID,
    new VirtualClock(), silent);
  bus.setRecvCallback((id, data, ext) => motor.handleFrame(id, data, ext));
  return [motor, bus];
}

// ── module-level seam ──
describe("module-level seam", () => {
  it("constants describe robstride mit", () => {
    expect(MODELS).toEqual(["RS00", "RS02"]);
    // Python: JointConfig(1, -1, 1, 1, 1).model == "RS00". The TS JointConfig requires `model`; a
    // profile joint without one takes the product default.
    const cfg = ArmConfig.fromRaw({ joints: [{ motor_id: 1, lo: -1, hi: 1, kp: 1, kd: 1 }] }, MAKER_ARM);
    expect(DEFAULT_MODEL === "RS00" && cfg.joints[0]!.model === "RS00").toBe(true);
    expect(MAKER_ARM.defaultModel).toBe("RS00");
    expect(KP_RANGE).toEqual([0.0, 500.0]);
    expect(KD_RANGE).toEqual([0.0, 5.0]);
    expect(ID_RANGE).toEqual([1, 127]);
    expect(DEFAULT_HOST_ID).toBe(0xfd);
  });

  it("route uses the can id for param replies and data0 on the host id", () => {
    const reply = paramBytes(p.ParamIndex.CAN_TIMEOUT, 1); // data[0] is 0x28, not a motor
    expect(route(0x307, reply, 0xfd)).toBe(7);
    expect(route(0x407, reply, 0xfd)).toBe(7);
    expect(route(0xfd, feedbackBytes(5), 0xfd)).toBe(5);
    expect(route(0xfe, feedbackBytes(5), 0xfd)).toBeNull();
    expect(route(0xfd, feedbackBytes(5), 0xfd, true)).toBeNull();
    expect(route(0xfd, new Uint8Array(0), 0xfd)).toBeNull();
  });

  it("feedback is neutral with robstride extras", async () => {
    const [motor] = wired(new FakeMotor(3, "RS00", { position: 0.7, temperature: 41.5, enabled: true, warning: true }));
    const fb = await motor.readState();
    expect(fb).not.toBeNull();
    approx(fb.position, 0.7, { abs: 1e-3 });
    approx(fb.temperature, 41.5);
    expect(Object.keys(fb.temperatures)).toEqual(["motor"]);
    approx(fb.temperatures.motor!, 41.5);
    expect(fb.enabled).toBeNull(); // parameter support not probed: mode bits unknown
    expect(fb.warning && fb.mode === 0 && !fb.faultFlag).toBe(true);
  });

  it("enabled is reported on parameter capable firmware", async () => {
    const fake = new FakeMotor(1, "RS00", { paramsSupported: true, enabled: true });
    const [motor] = wired(fake);
    await motor.validate();
    expect((await motor.readState()).enabled).toBe(true);
    fake.enabled = false;
    expect((await motor.readState()).enabled).toBe(false);
  });

  it("torque limit follows the model", () => {
    const clock = new VirtualClock();
    expect(new RobStrideMotor(joint(1, "RS00"), new FakeBus([]), DEFAULT_HOST_ID, clock, silent).torqueLimit).toBe(14.0);
    expect(new RobStrideMotor(joint(1, "RS02"), new FakeBus([]), DEFAULT_HOST_ID, clock, silent).torqueLimit).toBe(17.0);
  });

  it("frames for another motor are ignored", () => {
    const [motor] = wired(new FakeMotor(1));
    motor.handleFrame(0xfd, feedbackBytes(2, { pos: 0.5 }));
    expect(motor.feedback).toBeNull();
    expect(motor.feedbackSequence).toBe(0);
  });
});

// ── commands ──
describe("commands", () => {
  it("every command is a standard frame", async () => {
    const [motor, bus] = wired(new FakeMotor(1));
    await motor.enable();
    await motor.sendCommand(0.1, 0, 10, 1, 0);
    await motor.disable();
    await motor.pollStatus();
    expect(bus.sentExtended).toEqual([false, false, false, false]);
  });

  it("read state returns fresh feedback without changing torque", async () => {
    const fake = new FakeMotor(3, "RS00", { position: 0.7, enabled: true });
    const [motor] = wired(fake);
    const fb = await motor.readState();
    approx(fb.position, 0.7, { abs: 1e-3 });
    expect(fake.enabled).toBe(true);
  });

  it("check does not clear and latches", async () => {
    const fake = new FakeMotor(2, "RS00", { faultWord: 1 << 2 });
    const [motor] = wired(fake);
    const status = await motor.check();
    expect(status.hasFault && status.code === 1 << 2 && status.text === "undervoltage").toBe(true);
    expect(status.enabled).toBeNull();
    expect(fake.faultWord).toBe(1 << 2);
    expect(motor.faultLatched).toContain("undervoltage");
    await expect(motor.readState()).rejects.toThrow(FaultLatched);
    expect(fake.log).not.toContain("clear");
  });

  it("healthy status has empty text and an age", async () => {
    const [motor] = wired(new FakeMotor(1));
    expect(motor.status).toBeNull();
    expect(motor.statusAge).toBe(Infinity);
    const status = await motor.check();
    expect(!status.hasFault && status.code === 0 && status.text === "").toBe(true);
    expect(motor.status).toBe(status);
    expect(motor.statusAge).toBeLessThan(1.0);
  });

  it("poll status sends the non clearing query", async () => {
    const fake = new FakeMotor(1, "RS00", { faultWord: 1 });
    const [motor] = wired(fake);
    await motor.pollStatus();
    expect(fake.log).toEqual(["status_query"]);
    expect(motor.status!.hasFault).toBe(true);
    expect(fake.faultWord).toBe(1);
  });

  it("feedback fault flag latches", async () => {
    const [motor] = wired(new FakeMotor(1, "RS00", { faultFlag: true, enabled: true }));
    await motor.sendCommand(0.0, 0, 10, 1, 0);
    expect(motor.faultLatched).toBe("feedback fault flag set");
  });

  it("explicit clear fault resets latch", async () => {
    const fake = new FakeMotor(1, "RS00", { faultWord: 1 });
    const [motor] = wired(fake);
    await motor.check();
    await motor.clearFault();
    expect(motor.faultLatched).toBeNull();
    expect(fake.faultWord).toBe(0);
  });

  it("reset cache forgets everything", async () => {
    const [motor] = wired(new FakeMotor(1, "RS00", { faultWord: 1 }));
    await motor.check();
    motor.resetCache();
    expect(motor.status).toBeNull();
    expect(motor.feedback).toBeNull();
    expect(motor.faultLatched).toBeNull();
  });

  it("missing motor times out", async () => {
    const [motor] = wired(new FakeMotor(1, "RS00", { online: false }));
    await expect(motor.readState(0.005, 2)).rejects.toThrow(ParamTimeout);
    await expect(motor.check(0.005, 1)).rejects.toThrow(ParamTimeout);
  });

  it("set zero persists in the motor", async () => {
    const fake = new FakeMotor(1, "RS00", { position: 0.4 });
    const [motor] = wired(fake);
    await motor.setZero();
    expect(fake.log).toEqual(["zero"]);
    expect(fake.position).toBe(0.0);
  });

  it("set id writes the id and verifies the motor answers there", async () => {
    const fake = new FakeMotor(7);
    const [motor, bus] = wired(fake);
    const owner = bus.recvCallback;
    await motor.setId(8, undefined, true);
    expect(fake.motorId === 8 && motor.motorId === 8).toBe(true);
    expect(fake.log).toContainEqual(["set_id", 8]);
    expect(bus.sent.map(([id, d]) => [id, [...d]])).toContainEqual([7, [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 8, 0xfa]]);
    expect(bus.recvCallback).toBe(owner); // the tap is removed
    await motor.setId(7, undefined, true);
    expect(fake.motorId === 7 && motor.motorId === 7).toBe(true);
  });

  it("set id reports a motor that keeps its id", async () => {
    const fake = new FakeMotor(7);
    fake.applyIdChange = false;
    const [motor] = wired(fake);
    const done = motor.setId(8, 0.02, true);
    await expect(done).rejects.toThrow(StateError);
    await expect(done).rejects.toThrow(/still answers on ID 7/);
    expect(motor.motorId).toBe(7);
  });

  it("set id reports a motor that stops answering", async () => {
    const fake = new FakeMotor(7);
    const [motor] = wired(fake);
    const original = fake.handle.bind(fake);
    fake.handle = (canId, data) => {
      const replies = original(canId, data);
      if (data[7] === 0xfa) fake.online = false;
      return replies;
    };
    await expect(motor.setId(8, 0.02, true)).rejects.toThrow(/neither ID 7 nor ID 8/);
  });

  it("set id refuses bad ids a fault and an enabled motor without writing", async () => {
    const fake = new FakeMotor(3, "RS00", { paramsSupported: true });
    const [motor, bus] = wired(fake);
    await motor.validate(0.005);
    await expect(motor.setId(128)).rejects.toThrow(ValueError);
    await motor.setId(3); // no-op
    fake.faulted = true;
    const faulted = motor.setId(9, 0.02, true);
    await expect(faulted).rejects.toThrow(StateError);
    await expect(faulted).rejects.toThrow(/fault/);
    await motor.clearFault(); // the operator clears it
    await motor.enableConfirmed();
    await expect(motor.setId(9, 0.02, true)).rejects.toThrow(/torque on/);
    expect(idWrites(bus)).toEqual([]);
    expect(fake.motorId).toBe(3);
  });

  it("set id refuses a fresh motor object on an energized motor", async () => {
    // A motor object made for the call has no feedback of its own: setId reads the motor first.
    const fake = new FakeMotor(7, "RS00", { paramsSupported: true, enabled: true });
    const [motor, bus] = wired(fake);
    await expect(motor.setId(8, 0.02, true)).rejects.toThrow(/torque on/);
    expect(idWrites(bus)).toEqual([]);
    expect(fake.motorId).toBe(7);
  });

  it("set id with unknown torque state needs the confirmation", async () => {
    const fake = new FakeMotor(7); // old firmware: no mode bits
    const [motor, bus] = wired(fake);
    await expect(motor.setId(8, 0.02)).rejects.toThrow(/torqueOffConfirmed/);
    expect(idWrites(bus)).toEqual([]);
    await motor.setId(8, 0.02, true);
    expect(fake.motorId).toBe(8);
  });

  it.each([{ torque: 0.2 }, { torque: -0.05 }, { velocity: 0.5 }])("set id refuses a motor holding torque or moving %j",
    async (reading) => {
      const fake = new FakeMotor(7);
      Object.assign(fake, reading);
      const [motor, bus] = wired(fake);
      await expect(motor.setId(8, 0.02, true)).rejects.toThrow(/energized|moving/);
      expect(idWrites(bus)).toEqual([]);
      expect(fake.motorId).toBe(7);
    });

  it("set id probes the destination before writing", async () => {
    const source = new FakeMotor(7), occupant = new FakeMotor(8);
    const bus = new FakeBus([source, occupant]);
    const motor = new RobStrideMotor(joint(7, "RS00", { name: "j" }), bus, DEFAULT_HOST_ID, new VirtualClock(), silent);
    bus.setRecvCallback((id, data, ext) => motor.handleFrame(id, data, ext));
    const done = motor.setId(8, 0.02, true);
    await expect(done).rejects.toThrow(ArmError);
    await expect(done).rejects.toThrow(/already answers/);
    expect(idWrites(bus)).toEqual([]);
    expect(source.motorId).toBe(7);
  });

  it("set id needs the old id to go silent", async () => {
    // The source ignores the write and a motor on the new id, silent during the probe, answers
    // afterwards: the joint must not be rekeyed to that other motor.
    const source = new FakeMotor(7), occupant = new FakeMotor(8, "RS00", { online: false });
    source.applyIdChange = false;
    const original = source.handle.bind(source);
    source.handle = (canId, data) => {
      if (isIdWrite(data)) occupant.online = true;
      return original(canId, data);
    };
    const bus = new FakeBus([source, occupant]);
    const motor = new RobStrideMotor(joint(7, "RS00", { name: "j" }), bus, DEFAULT_HOST_ID, new VirtualClock(), silent);
    bus.setRecvCallback((id, data, ext) => motor.handleFrame(id, data, ext));
    const done = motor.setId(8, 0.02, true);
    await expect(done).rejects.toThrow(RouteUnverified);
    await expect(done).rejects.toThrow(/ID 7 and ID 8 both answer/);
    expect(motor.motorId).toBe(7);
  });

  it("encodeSetId is command 7 with the new id in byte 6", () => {
    const [id, data] = p.encodeSetId(7, 8);
    expect([id, [...data]]).toEqual([7, [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 8, 0xfa]]);
    expect(() => p.encodeSetId(7, 0)).toThrow(ValueError);
    expect(() => p.encodeSetId(7, 128)).toThrow(ValueError);
  });
});

function isIdWrite(data: Uint8Array): boolean {
  return data.length === 8 && data.subarray(0, 6).every((b) => b === 0xff) && data[7] === 0xfa;
}

function idWrites(bus: FakeBus): number[] {
  return bus.sent.filter(([, d]) => isIdWrite(d)).map(([id]) => id);
}

// ── connect-time validation and mode bits ──
describe("connect-time validation and mode bits", () => {
  it("validate accepts old and mit mode firmware", async () => {
    const [old] = wired(new FakeMotor(1));
    await old.validate(0.005);
    expect(old.paramsSupported).toBe(false);
    const [modern] = wired(new FakeMotor(2, "RS00", { paramsSupported: true }));
    await modern.validate();
    expect(modern.paramsSupported).toBe(true);
  });

  it("validate refuses a motor outside mit mode", async () => {
    const [motor] = wired(new FakeMotor(1, "RS00", { paramsSupported: true, runMode: 1 }));
    const done = motor.validate();
    await expect(done).rejects.toThrow(ConnectError);
    await expect(done).rejects.toThrow(/RUN_MODE is 1/);
  });

  it("confirms uses mode bits only where the firmware has them", async () => {
    const [old] = wired(new FakeMotor(1));
    expect(old.confirms("enabled")).toBe(false); // no feedback yet
    await old.readState();
    expect(old.confirms("enabled") && old.confirms("disabled")).toBe(true);
    const fake = new FakeMotor(2, "RS00", { paramsSupported: true });
    const [modern] = wired(fake);
    await modern.validate();
    await modern.enable();
    expect(modern.confirms("enabled") && !modern.confirms("disabled")).toBe(true);
    await modern.disable();
    expect(modern.confirms("disabled") && !modern.confirms("enabled")).toBe(true);
    // Python raises KeyError; the TS port throws ValueError.
    expect(() => modern.confirms("sleeping" as "enabled")).toThrow(ValueError);
  });

  it("health problem after mode fault ticks and reset on enable", async () => {
    const fake = new FakeMotor(1, "RS00", { paramsSupported: true });
    const [motor] = wired(fake);
    await motor.validate();
    await motor.enable();
    fake.enabled = false;
    await motor.sendCommand(0.0, 0, 10, 1, 0);
    for (let i = 0; i < RobStrideMotor.MODE_FAULT_TICKS - 1; i++) expect(motor.healthProblem()).toBeNull();
    expect(motor.healthProblem()).toContain("left run mode (mode=0");
    await motor.enable();
    expect(motor.healthProblem()).toBeNull();
  });

  it("calibration mode is never healthy even on old firmware", () => {
    const [motor] = wired(new FakeMotor(1));
    motor.handleFrame(0xfd, feedbackBytes(1, { mode: 1 }));
    const problems = Array.from({ length: RobStrideMotor.MODE_FAULT_TICKS }, () => motor.healthProblem());
    expect(problems[problems.length - 1]).toBeTruthy();
    expect(problems.slice(0, -1).some(Boolean)).toBe(false);
  });
});

// ── watchdog and parameters ──
describe("watchdog and parameters", () => {
  it("watchdog armed verified and restored", async () => {
    const fake = new FakeMotor(7, "RS00", { paramsSupported: true, canTimeout: 42 });
    const [motor] = wired(fake);
    await motor.validate();
    const token = await motor.armWatchdog(500);
    expect(token).toBe(42);
    expect(fake.canTimeout).toBe(10000);
    await motor.restoreWatchdog(token);
    expect(fake.canTimeout).toBe(42);
  });

  it("watchdog unsupported on old firmware", async () => {
    const fake = new FakeMotor(7);
    const [motor, bus] = wired(fake);
    await motor.validate(0.005);
    bus.sent.length = 0;
    expect(await motor.armWatchdog(500)).toBeNull();
    expect(bus.sent).toEqual([]);
  });

  it("params by name index and write readback", async () => {
    const [motor] = wired(new FakeMotor(7, "RS00", { paramsSupported: true, runMode: 0 }));
    expect(await motor.probeParams()).toBe(true);
    expect(await motor.readParam("run_mode")).toBe(0);
    expect(await motor.writeParam("can_timeout", 10000)).toBe(10000);
    expect(await motor.readParam(p.ParamIndex.CAN_TIMEOUT)).toBe(10000); // dtype from PARAMS
    expect(await motor.writeParam(p.ParamIndex.CAN_TIMEOUT, 5, "u32")).toBe(5);
    expect(new Set(Object.keys(RobStrideMotor.PARAMS))).toEqual(new Set(["run_mode", "mech_pos", "can_timeout"]));
    expect(RobStrideMotor.PARAMS).toBe(PARAMS);
    await expect(motor.readParam("no_such_param")).rejects.toThrow(ValueError);
    const save = motor.writeParam("can_timeout", 1, undefined, undefined, true);
    await expect(save).rejects.toThrow(ArmError);
    await expect(save).rejects.toThrow(/flash/);
  });

  it("params probe on old firmware", async () => {
    const [old] = wired(new FakeMotor(1));
    expect(await old.probeParams(0.01)).toBe(false);
  });

  it("write acknowledgement is not a read result", async () => {
    const [motor, bus] = wired(new FakeMotor(7, "RS00", { paramsSupported: true }));
    bus.responder = () => [[0x407, paramBytes(p.ParamIndex.CAN_TIMEOUT, 10000)]];
    await expect(motor.readParam(p.ParamIndex.CAN_TIMEOUT, "u32", 0.01)).rejects.toThrow(ParamTimeout);
  });

  it("disable confirmed needs mode zero on modern firmware", async () => {
    class StaysOn extends FakeMotor {
      override get mode(): number {
        return 2;
      }
    }
    const [motor] = wired(new StaysOn(1, "RS00", { paramsSupported: true }));
    await motor.validate();
    expect(await motor.disableConfirmed(0.005, 2)).toBe(false);
    const [old] = wired(new FakeMotor(2));
    expect(await old.disableConfirmed()).toBe(true);
  });
});
