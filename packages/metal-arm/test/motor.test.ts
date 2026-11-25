/** Port of the Python SDK's tests/unit/test_motor.py. */

import { ConnectError, ParamTimeout, StateError, ValueError } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import { describe, expect, it } from "vitest";
import * as m from "../src/motor.js";
import { Motor, route } from "../src/motor.js";
import * as p from "../src/protocol.js";
import { classify, feedbackBytes, registerReply, SimBus, SimMotor } from "../src/sim-motor.js";
import { makeJoint, wired } from "./fakes.js";

const frameOf = (f: [number, Uint8Array]) => [f[0], Array.from(f[1])];
const sentFrames = (bus: SimBus) => bus.sent.map(frameOf);

const u32le = (n: number) => Uint8Array.from([n & 0xff, (n >> 8) & 0xff, (n >> 16) & 0xff, (n >>> 24) & 0xff]);
const count = (bus: SimBus, f: p.Frame) => sentFrames(bus).filter((x) => JSON.stringify(x) === JSON.stringify(frameOf(f))).length;

describe("motor", () => {
  it("seam constants", () => {
    expect(m.MODELS).toEqual(["jlo", "j2", "jhi"]);
    expect(m.MODELS).toContain(m.DEFAULT_MODEL);
    expect(m.KP_RANGE).toEqual([0.0, 500.0]);
    expect(m.KD_RANGE).toEqual([0.0, 5.0]);
    expect(m.ID_RANGE).toEqual([1, 15]);
    expect(m.DEFAULT_HOST_ID).toBe(0x10);
  });

  it("reply id defaults to host + id or the joint setting", () => {
    const bus = new SimBus([]);
    expect(new Motor(makeJoint(3), bus, 0x10).replyId).toBe(0x13);
    expect(new Motor(makeJoint(3, "jlo", "", { replyId: 0x23 }), bus, 0x10).replyId).toBe(0x23);
    expect(() => new Motor(makeJoint(3, "RS00"), bus, 0x10)).toThrow(ValueError);
    expect(() => new Motor(makeJoint(16), bus, 0x10)).toThrow(ValueError);
  });

  it("every command is a standard frame to the send id", async () => {
    const { motor, bus } = wired([new SimMotor(4)]);
    await motor.enable();
    await motor.sendCommand(0.1, 0, 10, 1, 0);
    await motor.disable();
    await motor.setZero();
    await motor.clearFault();
    expect(bus.sentExtended).toEqual([false, false, false, false, false]);
    expect(bus.sent.map(([cid]) => cid)).toEqual([4, 4, 4, 4, 4]);
    expect(bus.sent.filter(([, d]) => p.startsWithCommandPrefix(d)).map(([, d]) => d[7])).toEqual([0xfc, 0xfd, 0xfe, 0xfb]);
  });

  it("feedback from command replies and cache", async () => {
    const fake = new SimMotor(2, "j2", { enabled: true, temperature: 41, rotorTemperature: 48 });
    const { motor } = wired([fake]);
    expect(motor.feedback).toBeNull();
    expect(motor.feedbackAge).toBe(Infinity);
    expect(motor.statusAge).toBe(Infinity);
    await motor.sendCommand(0.5, 0, 50, 1, 0);
    const fb = motor.feedback!;
    expect(Math.abs(fb.position - 0.5)).toBeLessThanOrEqual(1e-3);
    expect([fb.temperature, fb.temperatures]).toEqual([48, { mos: 41, rotor: 48 }]);
    expect([fb.enabled, fb.warning, fb.status]).toEqual([true, false, 1]);
    expect(motor.feedbackSequence).toBe(1);
    expect(motor.feedbackAge).toBeLessThan(1);
    expect(motor.status).toEqual({ motorId: 2, hasFault: false, code: 1, text: "", enabled: true });
    motor.resetCache();
    expect(motor.feedback).toBeNull();
    expect(motor.status).toBeNull();
  });

  it("notifies changed after every accepted frame", async () => {
    const { motor } = wired([new SimMotor(2)]);
    let n = 0;
    const waiting = motor.changed.wait(() => ++n >= 2, 1, { now: () => 0, sleep: async () => {}, timer: () => () => {} });
    await motor.pollStatus();
    expect(await waiting).toBe(true);
  });

  it("readState is fresh and read-only", async () => {
    const fake = new SimMotor(3, "jlo", { position: 0.7, enabled: true });
    const { motor, bus } = wired([fake]);
    const fb = await motor.readState();
    expect(Math.abs(fb.position - 0.7)).toBeLessThanOrEqual(1e-3);
    expect(fake.enabled).toBe(true);
    expect(fake.log).not.toContain("clear");
    expect(frameOf(bus.sent[0]!)).toEqual(frameOf(p.encodeRefresh(3)));
  });

  it("check reports without clearing and latches", async () => {
    const fake = new SimMotor(7, "jhi", { fault: p.STATUS_COMM_LOST });
    const { motor } = wired([fake]);
    const status = await motor.check();
    expect([status.hasFault, status.code, status.enabled]).toEqual([true, 0xd, false]);
    expect(status.text).toBe("fault 0xD (communication lost)");
    expect(motor.faultLatched).toBe("fault 0xD (communication lost)");
    expect(motor.watchdogReleased).toBe(true);
    expect(fake.fault).toBe(0xd);
    expect(fake.log).not.toContain("clear");
    await motor.readState(); // never refused: the refresh clears nothing
    expect(fake.fault).toBe(0xd);
    expect(motor.healthProblem()).toBe("fault 0xD (communication lost)");
  });

  it("clearFault sends FB and resets the latch", async () => {
    const fake = new SimMotor(1, "jlo", { fault: 0x9 });
    const { motor, bus } = wired([fake]);
    await motor.check();
    await motor.clearFault();
    expect(frameOf(bus.sent.at(-1)!)).toEqual([1, [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb]]);
    expect(motor.faultLatched).toBeNull();
    expect(fake.fault).toBe(0);
    expect((await motor.check()).hasFault).toBe(false);
  });

  it("confirms from the status nibble", async () => {
    const fake = new SimMotor(1);
    const { motor } = wired([fake]);
    expect(motor.confirms("enabled") || motor.confirms("disabled")).toBe(false);
    await motor.enable();
    expect(motor.confirms("enabled") && !motor.confirms("disabled")).toBe(true);
    await motor.disable();
    expect(motor.confirms("disabled")).toBe(true);
    fake.fault = 0xe;
    await motor.check();
    expect(motor.confirms("disabled") && !motor.confirms("enabled")).toBe(true);
    expect(() => motor.confirms("on" as "enabled")).toThrow(ValueError);
  });

  it("disableConfirmed", async () => {
    const fake = new SimMotor(1, "jlo", { enabled: true });
    const { motor } = wired([fake]);
    expect(await motor.disableConfirmed()).toBe(true);
    expect(fake.enabled).toBe(false);
    const offline = wired([new SimMotor(2, "jlo", { online: false })]).motor;
    expect(await offline.disableConfirmed(0.005, 1)).toBe(false);
  });

  it("health problem on a persistent disabled status", async () => {
    const fake = new SimMotor(1, "jlo", { enabled: true });
    const { motor } = wired([fake]);
    await motor.enable();
    expect(motor.healthProblem()).toBeNull();
    fake.enabled = false;
    await motor.check();
    const problems = Array.from({ length: Motor.STATUS_DISABLED_TICKS }, () => motor.healthProblem());
    expect(problems.slice(0, -1)).toEqual(Array(Motor.STATUS_DISABLED_TICKS - 1).fill(null));
    expect(problems.at(-1)).toContain("disabled");
  });

  it("a missing motor times out", async () => {
    const { motor } = wired([new SimMotor(1, "jlo", { online: false })]);
    await expect(motor.readState(0.005, 2)).rejects.toThrow(ParamTimeout);
    await expect(motor.check(0.005, 1)).rejects.toThrow(ParamTimeout);
    await expect(motor.readParam("ctrl_mode", undefined, 0.005)).rejects.toThrow(ParamTimeout);
  });

  it("a dropped request is re-sent", async () => {
    const fake = new SimMotor(2, "jlo", { position: 0.3, drop: 1 });
    const { motor, bus } = wired([fake]);
    expect(Math.abs((await motor.readState(0.01)).position - 0.3)).toBeLessThanOrEqual(1e-3);
    expect(count(bus, p.encodeRefresh(2))).toBe(2);
  });

  it("replies on another reply id are ignored", () => {
    const { motor } = wired([new SimMotor(1)]);
    motor.handleFrame(0x12, feedbackBytes(1, { pos: 1.0 })); // right nibble, wrong master id
    motor.handleFrame(0x11, feedbackBytes(2, { pos: 1.0 })); // right master id, wrong nibble
    motor.handleFrame(0x11, feedbackBytes(1, { pos: 1.0 }), true);
    expect(motor.feedback).toBeNull();
  });

  // ---- the Metal gripper's late / duplicate replies ----

  it("a late reply is flushed out", async () => {
    const fake = new SimMotor(7, "jhi", { position: 0.4, late: true });
    const { motor, bus } = wired([fake]);
    expect(Math.abs((await motor.readState()).position - 0.4)).toBeLessThanOrEqual(1e-3);
    expect(count(bus, p.encodeRefresh(6))).toBeGreaterThan(0);
  });

  it.each([false, true])("a stale held reply is superseded by the fresh one (duplicate=%s)", async (duplicate) => {
    const fake = new SimMotor(7, "jhi", { position: 0.9, late: true, duplicate });
    fake.held = [[0x17, feedbackBytes(7, { pos: 0.1, model: "jhi" })]]; // answer to an earlier request
    const { motor } = wired([fake]);
    for (const position of [0.9, 0.7, 0.5]) {
      fake.position = position;
      expect(Math.abs((await motor.readState()).position - position)).toBeLessThanOrEqual(1e-3);
    }
  });

  it("a punctual motor needs one trailing flush only", async () => {
    const { motor, bus } = wired([new SimMotor(3)]);
    await motor.readState();
    expect(sentFrames(bus)).toEqual([frameOf(p.encodeRefresh(3)), frameOf(p.encodeRefresh(2))]);
  });

  it("the flush refreshes the neighbour and updates it", async () => {
    const { motors } = wired([new SimMotor(6, "jhi", { position: -0.2 }), new SimMotor(7, "jhi", { position: 0.4, late: true })]);
    expect(Math.abs((await motors.get(7)!.readState()).position - 0.4)).toBeLessThanOrEqual(1e-3);
    expect(Math.abs(motors.get(6)!.feedback!.position + 0.2)).toBeLessThanOrEqual(1e-3);
  });

  it("a stale register echo is skipped", async () => {
    const fake = new SimMotor(7, "jhi", { late: true });
    fake.held = [[0x17, registerReply(7, p.OP_WRITE, p.Reg.TIMEOUT, u32le(200))]];
    const { motor } = wired([fake]);
    expect(await motor.writeParam("timeout", 10_000)).toBe(10_000);
    expect(fake.watchdogTicks).toBe(10_000);
  });

  it("a register write that does not stick fails closed", async () => {
    const fake = new SimMotor(1);
    const { motor } = wired([fake]);
    const original = fake.paramFrame.bind(fake);
    fake.paramFrame = (data) => {
      const replies = original(data);
      fake.registers[p.Reg.TIMEOUT] = 0;
      return replies;
    };
    await expect(motor.writeParam("timeout", 100)).rejects.toThrow(StateError);
  });

  // ---- registers, validation, watchdog ----

  it("read and write params by name and index", async () => {
    const fake = new SimMotor(5, "jhi");
    const { motor } = wired([fake]);
    expect(await motor.readParam("ctrl_mode")).toBe(1);
    expect(await motor.readParam(p.Reg.VMAX)).toBeCloseTo(30.0, 6);
    expect(await motor.readParam("PMAX")).toBeCloseTo(6.28, 6);
    expect(await motor.writeParam("kp_apr", 12.5)).toBe(12.5);
    expect(fake.log).not.toContain("save");
    expect(new Set(Object.keys(Motor.PARAMS))).toEqual(new Set(Object.keys(p.REGISTERS)));
    expect(Motor.PARAMS.timeout).toEqual({ index: 9, dtype: "u32", doc: p.REGISTERS.timeout!.doc });
    expect(motor.params).toBe(Motor.PARAMS);
    await expect(motor.readParam("nope")).rejects.toThrow(ValueError);
  });

  it("writeParam save requires disabled and stores flash", async () => {
    const fake = new SimMotor(1);
    const { motor } = wired([fake]);
    await motor.writeParam("timeout", 40, undefined, 0.12, true);
    expect(fake.flash[p.Reg.TIMEOUT]).toBe(40);
    expect(fake.log).toContain("save");
    fake.enabled = true;
    await expect(motor.writeParam("timeout", 60, undefined, 0.12, true)).rejects.toThrow(StateError);
    expect(fake.flash[p.Reg.TIMEOUT]).toBe(40); // refused before the RAM write
    expect(fake.watchdogTicks).toBe(40);
  });

  it("validate accepts a matching motor", async () => {
    await wired([new SimMotor(2, "j2")]).motor.validate();
  });

  it("validate rejects a non-MIT mode", async () => {
    const { motor } = wired([new SimMotor(7, "jhi", { registers: { [p.Reg.CTRL_MODE]: p.MODE_FORCE_POS } })]);
    await expect(motor.validate()).rejects.toThrow(/not 1 \(MIT\)/);
  });

  it("validate rejects a scaling mismatch and names the fix", async () => {
    const fake = new SimMotor(2, "j2");
    const bus = new SimBus([fake]);
    const motor = new Motor(makeJoint(2, "jlo", "shoulder_lift"), bus, 0x10, new VirtualClock());
    bus.setRecvCallback((c, d, e) => motor.handleFrame(c, d, e));
    await expect(motor.validate()).rejects.toThrow(/model: j2/);
    fake.registers[p.Reg.TMAX] = 7.0;
    await expect(motor.validate()).rejects.toThrow(/registers 21-23/);
  });

  it("validate offline is a connect error", async () => {
    const { motor } = wired([new SimMotor(1, "jlo", { online: false })]);
    await expect(motor.validate(0.005)).rejects.toThrow(ConnectError);
  });

  it("the watchdog is set in RAM and restored", async () => {
    const fake = new SimMotor(7, "jhi", { registers: { [p.Reg.TIMEOUT]: 0 } });
    const { motor } = wired([fake]);
    const token = await motor.armWatchdog(500);
    expect(token).toBe(0);
    expect(fake.watchdogTicks).toBe(10_000);
    expect(fake.flash[p.Reg.TIMEOUT]).toBe(0);
    expect(fake.log).not.toContain("save");
    await motor.restoreWatchdog(token);
    expect(fake.watchdogTicks).toBe(0);
    await motor.restoreWatchdog(null);
    await expect(motor.armWatchdog(0)).rejects.toThrow(ValueError);
  });

  // ---- setId ----

  const direct = (fake: SimMotor) => {
    const w = wired([fake]);
    const cb = (c: number, d: Uint8Array, e: boolean) => w.motor.handleFrame(c, d, e);
    w.bus.setRecvCallback(cb);
    return { ...w, cb };
  };

  it("setId writes, saves and verifies", async () => {
    const fake = new SimMotor(3);
    const { motor, bus, cb } = direct(fake);
    await motor.setId(9);
    expect([motor.motorId, motor.replyId]).toEqual([9, 0x19]);
    expect([fake.sendId, fake.replyId]).toEqual([9, 0x19]);
    expect([fake.flash[p.Reg.ESC_ID], fake.flash[p.Reg.MST_ID]]).toEqual([9, 0x19]);
    expect(fake.log.filter((e) => Array.isArray(e) && e[0] === "write")).toEqual([["write", p.Reg.MST_ID], ["write", p.Reg.ESC_ID]]);
    expect(fake.log).toContain("save");
    expect((await motor.readState()).motorId).toBe(9);
    expect(bus.recvCallback).toBe(cb); // the backend callback is restored
  });

  it.each(["new", "none"] as const)("setId when the id write echo names the new id or is missing (%s)", async (echo) => {
    const fake = new SimMotor(7, "jlo", { escEcho: echo });
    const { motor } = direct(fake);
    await motor.setId(8);
    expect([motor.motorId, motor.replyId]).toEqual([8, 0x18]);
    expect([fake.flash[p.Reg.ESC_ID], fake.flash[p.Reg.MST_ID]]).toEqual([8, 0x18]);
  });

  it("setId when ids apply only after a power cycle", async () => {
    const fake = new SimMotor(3, "jlo", { idsApplyAtOnce: false });
    const { motor } = direct(fake);
    await expect(motor.setId(9, 0.02)).rejects.toThrow(/power-cycle/);
    expect(fake.flash[p.Reg.ESC_ID]).toBe(9);
    fake.powerCycle();
    expect([fake.sendId, fake.replyId]).toEqual([9, 0x19]);
  });

  it("setId refuses an id in use", async () => {
    const fakes = [new SimMotor(3), new SimMotor(4)];
    const { motors } = wired(fakes);
    await expect(motors.get(3)!.setId(4, 0.02)).rejects.toThrow(/already used/);
    expect(fakes[0]!.log.some((e) => Array.isArray(e) && e[0] === "write")).toBe(false);
    expect(motors.get(3)!.motorId).toBe(3);
  });

  it("setId refuses an enabled motor and bad ids", async () => {
    const fake = new SimMotor(3, "jlo", { enabled: true });
    const { motor } = wired([fake]);
    await expect(motor.setId(9, 0.02)).rejects.toThrow(/reports torque on; disable/);
    expect(fake.sendId).toBe(3);
    await expect(motor.setId(16)).rejects.toThrow(ValueError);
    await motor.setId(3); // no-op
  });

  it("setId on the late gripper", async () => {
    const fake = new SimMotor(7, "jhi", { late: true, duplicate: true });
    const { motor } = direct(fake);
    await motor.setId(8);
    expect([fake.sendId, fake.flash[p.Reg.ESC_ID]]).toEqual([8, 8]);
  });

  // ---- routing ----

  it("route", () => {
    expect(route(0x17, feedbackBytes(7, { status: 1 }), 0x10)).toBe(7);
    expect(route(0x13, registerReply(3, p.OP_READ, 10, new Uint8Array(4)), 0x10)).toBe(3);
    expect(route(0x23, feedbackBytes(3), 0x20)).toBe(3);
    expect(route(0x55, feedbackBytes(3), 0x10)).toBe(3); // custom reply id: nibble
    expect(route(0x07, feedbackBytes(7), 0x10)).toBeNull(); // a command to motor 7
    expect(route(0x7ff, p.encodeRefresh(7)[1], 0x10)).toBeNull();
    expect(route(0x307, feedbackBytes(7), 0x10)).toBeNull();
    expect(route(0x17, feedbackBytes(7), 0x10, true)).toBeNull();
    expect(route(0x17, feedbackBytes(7).slice(0, 5), 0x10)).toBeNull();
    expect(route(0x17, Uint8Array.from([0x27, ...feedbackBytes(7).slice(1)]), 0x10)).toBeNull();
  });

  it("pollStatus is a read-only refresh", async () => {
    const fake = new SimMotor(2, "jlo", { faulted: true });
    const { motor, bus } = wired([fake]);
    await motor.pollStatus();
    expect(sentFrames(bus)).toEqual([frameOf(p.encodeRefresh(2))]);
    expect(fake.log).toEqual(["status_query"]);
    expect(fake.faulted).toBe(true);
    expect(motor.status!.hasFault).toBe(true);
    expect(motor.status!.text).toBe("fault 0xC (rotor overtemperature)");
  });

  it("classify", () => {
    expect(classify(...p.encodeRefresh(3))).toEqual(["status_query", 3]);
    expect(classify(...p.encodeReadRegister(3, 9))).toEqual(["param_read", 3]);
    expect(classify(...p.encodeWriteRegister(3, 9, 1))).toEqual(["param_write", 3]);
    expect(classify(...p.encodeSave(3))).toEqual(["param_save", 3]);
    expect(classify(...p.encodeClearError(3))).toEqual(["clear", 3]);
    expect(classify(...p.encodeEnable(3))).toEqual(["enable", 3]);
    expect(classify(...p.encodeMit(3, 0, 0, 1, 0, 0, p.MODELS.jlo!))).toEqual(["command", 3]);
  });

  it("queries are serialized by the query lock", async () => {
    const { motors } = wired([new SimMotor(6, "jhi", { position: 0.1 }), new SimMotor(7, "jhi", { position: 0.4, late: true })]);
    const m7 = motors.get(7)!;
    const [a, b] = await Promise.all([m7.readState(), m7.readParam("ctrl_mode")]);
    expect(Math.abs(a.position - 0.4)).toBeLessThanOrEqual(1e-3);
    expect(b).toBe(1);
  });
});
