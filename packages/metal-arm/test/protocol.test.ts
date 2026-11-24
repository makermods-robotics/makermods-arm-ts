/** Port of the Python SDK's tests/unit/test_protocol.py. */

import { ValueError } from "@makermods/arm-core";
import { describe, expect, it } from "vitest";
import { radians } from "@makermods/arm-core";
import * as p from "../src/protocol.js";
import { feedbackBytes } from "../src/sim-motor.js";
import { hex, unhex } from "./fixture.js";

const degrees = (r: number) => r * (180 / Math.PI);
const f32 = (x: number) => { const b = new Uint8Array(4); new DataView(b.buffer).setFloat32(0, x, true); return b; };

// Golden MIT frames generated with the LeRobot Damiao encoder: (model, kp, kd, position_deg, velocity_deg_s, torque, payload)
const MIT_GOLDEN: [string, number, number, number, number, number, string][] = [
  ["jlo", 100.0, 5.0, 0.0, 0.0, 0.0, "7fff7ff333fff7ff"],
  ["jlo", 100.0, 5.0, 45.0, 30.0, 1.5, "900186a333fff865"],
  ["j2", 100.0, 5.0, -90.0, -10.0, -20.0, "5ffb7db333fff6aa"],
  ["jhi", 20.0, 0.6, 120.0, 200.0, 0.3, "aaaf8ed0a31eb81e"],
  ["jhi", 0.0, 0.0, -400.0, -9999.0, -99.0, "0000000000000000"], // clamped low
  ["jlo", 500.0, 5.0, 359.0, 572.0, 30.0, "ffb4ffbfffffffff"], // clamped high
];

// Golden decodes: (model, payload, status, id, position_deg, velocity_deg_s, torque, t_mos, t_rotor)
const STATE_GOLDEN: [string, string, number, number, number, number, number, number, number][] = [
  ["jlo", "1180007ff7ff1e20", 1, 1, 0.005490463040242452, -0.13991643348738003, -0.0073260073260073, 30, 32],
  ["j2", "02a3d71234567890", 0, 2, 100.7554872514522, -491.5264308411677, -54.94505494505495, 120, 144],
  ["jhi", "d7ffff0000002d2f", 0xd, 7, 359.817495342157, -1718.8733853924696, -20.0, 45, 47],
];

describe("protocol", () => {
  it.each(MIT_GOLDEN)("MIT encoder matches LeRobot bytes (%s kp=%d pos=%d)", (model, kp, kd, pos, vel, tau, payload) => {
    const [cid, data] = p.encodeMit(3, radians(pos), radians(vel), kp, kd, tau, p.MODELS[model]!);
    expect(cid).toBe(3);
    expect(hex(data)).toBe(payload);
  });

  it.each(STATE_GOLDEN)("state decoder matches LeRobot (%s %s)", (model, payload, status, motorId, pos, vel, tau, tMos, tRotor) => {
    const msg = p.decode(0x10 + motorId, unhex(payload), p.MODELS[model]!);
    expect(msg).toBeInstanceOf(p.StateFrame);
    const s = msg as p.StateFrame;
    expect([s.motorId, s.status]).toEqual([motorId, status]);
    expect(Math.abs(degrees(s.position) - pos)).toBeLessThanOrEqual(1e-9);
    expect(Math.abs(degrees(s.velocity) - vel)).toBeLessThanOrEqual(1e-9);
    expect(Math.abs(s.torque - tau)).toBeLessThanOrEqual(1e-9);
    expect([s.tMos, s.tRotor]).toEqual([tMos, tRotor]);
  });

  it.each([[p.encodeEnable, 0xfc], [p.encodeDisable, 0xfd], [p.encodeSetZero, 0xfe], [p.encodeClearError, 0xfb]] as const)(
    "commands are seven FF and a command byte to the send id", (encode, last) => {
      const [cid, data] = encode(7);
      expect([cid, Array.from(data)]).toEqual([7, [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, last]]);
    });

  it("command mode offsets", () => {
    expect(p.encodeClearError(7, p.MODE_FORCE_POS)[0]).toBe(0x307);
    expect(p.encodeEnable(2, p.MODE_POS_VEL)[0]).toBe(0x102);
  });

  it.each([0, 16, 0x7ff])("motor id range (%d)", (bad) => {
    expect(() => p.encodeEnable(bad)).toThrow(ValueError);
    expect(() => p.encodeRefresh(bad)).toThrow(ValueError);
  });

  it("refresh and register frames", () => {
    const f = (fr: p.Frame) => [fr[0], hex(fr[1])];
    expect(f(p.encodeRefresh(7))).toEqual([0x7ff, "0700cc0000000000"]);
    expect(f(p.encodeReadRegister(3, p.Reg.CTRL_MODE))).toEqual([0x7ff, "0300330a00000000"]);
    expect(f(p.encodeWriteRegister(7, p.Reg.TIMEOUT, 10_000))).toEqual([0x7ff, "0700550910270000"]);
    expect(f(p.encodeWriteRegister(1, p.Reg.PMAX, 6.28))).toEqual([0x7ff, "01005515" + hex(f32(6.28))]);
    expect(f(p.encodeSave(5))).toEqual([0x7ff, "0500aa0100000000"]);
  });

  it("register dtypes follow DM_CAN", () => {
    const u32 = new Set([7, 8, 9, 10, 13, 14, 15, 16, 35, 36]);
    for (const spec of Object.values(p.REGISTERS)) expect(spec.dtype).toBe(u32.has(spec.index) ? "u32" : "f");
    expect(p.registerDtype(59)).toBe("f");
    expect(() => p.encodeWriteRegister(1, p.Reg.PMAX, NaN)).toThrow(ValueError);
    expect(() => p.packValue(1, "u8")).toThrow(ValueError);
  });

  it("MIT rejects non-finite", () => {
    expect(() => p.encodeMit(1, NaN, 0, 0, 0, 0, p.MODELS.jlo!)).toThrow(ValueError);
    expect(() => p.encodeMit(1, 0, 0, Infinity, 0, 0, p.MODELS.jlo!)).toThrow(ValueError);
  });

  it("an MIT frame never reads as a command", () => {
    const m = p.MODELS.jlo!;
    const [, data] = p.encodeMit(1, m.pMax, m.vMax, p.KP_MAX, p.KD_MAX, m.tMax, m);
    expect(p.startsWithCommandPrefix(data)).toBe(false);
    expect(hex(data)).toBe("fffeffffffffffff");
  });

  it("register reply decodes and echoes the header", () => {
    const msg = p.decode(0x17, unhex("0700330a01000000"), p.MODELS.jhi!) as p.RegisterReply;
    expect(msg).toBeInstanceOf(p.RegisterReply);
    expect([msg.motorId, msg.op, msg.rid, Array.from(msg.raw)]).toEqual([7, p.OP_READ, p.Reg.CTRL_MODE, [1, 0, 0, 0]]);
    expect(msg.value()).toBe(1);
    const pmax = p.decode(0x17, unhex("07003315" + hex(f32(6.28))), p.MODELS.jhi!) as p.RegisterReply;
    expect(pmax.value()).toBeCloseTo(6.28, 6);
  });

  it("a disabled state frame that looks like a register reply is a register reply", () => {
    // status 0, id 7, position bytes 00 55: identical to a write echo header.
    expect(p.decode(0x17, unhex("0700550910270000"), p.MODELS.jhi!)).toBeInstanceOf(p.RegisterReply);
  });

  it("foreign frames are ignored", () => {
    const m = p.MODELS.jlo!;
    const data = feedbackBytes(1, { status: 1 });
    expect(p.decode(0x11, data, m, true)).toBeNull();
    expect(p.decode(0x01, data, m)).toBeNull(); // someone's command to motor 1
    expect(p.decode(0x7ff, data, m)).toBeNull(); // someone's register request
    expect(p.decode(0x101, data, m)).toBeNull(); // pos-vel command
    expect(p.decode(0x11, data.slice(0, 7), m)).toBeNull(); // short frame
    expect(p.decode(0x11, Uint8Array.from([0x21, ...data.slice(1)]), m)).toBeNull(); // unknown status 2
  });

  it("state round trip and status text", () => {
    const m = p.MODELS.j2!;
    const msg = p.decode(0x12, feedbackBytes(2, { pos: -1.0, vel: 2.0, tau: 50.0, temp: 41, rotorTemp: 55, status: 0xc, model: "j2" }), m) as p.StateFrame;
    expect(Math.abs(msg.position + 1.0)).toBeLessThanOrEqual(2e-4);
    expect(Math.abs(msg.velocity - 2.0)).toBeLessThanOrEqual(6e-3);
    expect(Math.abs(msg.torque - 50.0)).toBeLessThanOrEqual(0.06);
    expect([msg.tMos, msg.tRotor, msg.status]).toEqual([41, 55, 0xc]);
    expect(p.statusText(0xc)).toBe("fault 0xC (rotor overtemperature)");
    expect(p.statusText(1)).toBe("enabled");
    expect(p.statusText(0)).toBe("disabled");
    expect(p.statusText(null)).toBe("status unknown");
    expect(p.statusText(0x5)).toBe("fault 0x5 (unknown fault)");
    expect(p.isFaultStatus(0xd) && !p.isFaultStatus(1)).toBe(true);
  });

  it("matching model", () => {
    expect(p.matchingModel(6.28, 10.0, 120.0)).toBe("j2");
    expect(p.matchingModel(Math.fround(6.28), Math.fround(30.0), Math.fround(20.0))).toBe("jhi");
    expect(p.matchingModel(12.5, 30.0, 10.0)).toBeNull();
  });
});
