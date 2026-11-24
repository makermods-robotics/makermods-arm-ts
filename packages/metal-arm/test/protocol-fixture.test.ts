/** Every encoder and decoder against contract/fixtures/metal_protocol.json (tools/gen_metal_protocol.py). */

import { describe, expect, it } from "vitest";
import { route } from "../src/motor.js";
import * as p from "../src/protocol.js";
import { classify, feedbackBytes } from "../src/sim-motor.js";
import { enc, hex, loadFixture, num, type Outcome, outcome, sameOutcome, unhex } from "./fixture.js";

type Row = unknown[];
const fx = loadFixture<Record<string, Row[]>>("metal_protocol.json");
const frame = (f: p.Frame) => [f[0], hex(f[1])];

function decoded(msg: p.StateFrame | p.RegisterReply | null): unknown {
  if (msg === null) return null;
  if (msg instanceof p.StateFrame) {
    return { type: "state", motor_id: msg.motorId, status: msg.status, position: enc(msg.position),
      velocity: enc(msg.velocity), torque: enc(msg.torque), t_mos: enc(msg.tMos), t_rotor: enc(msg.tRotor) };
  }
  return { type: "register", motor_id: msg.motorId, op: msg.op, rid: msg.rid, raw: hex(msg.raw) };
}

/** Rows whose actual outcome differs from the recorded one (empty when everything matches). */
function mismatches(rows: Row[], run: (row: Row) => [Outcome, Outcome]): unknown[] {
  const bad: unknown[] = [];
  for (const row of rows) {
    const [actual, expected] = run(row);
    if (!sameOutcome(actual, expected)) bad.push({ row, actual });
  }
  return bad.slice(0, 5);
}

const ok = (v: unknown): Outcome => ({ ok: v });

describe("codec fixture", () => {
  it("has the Python model table", () => {
    const models = fx.models as unknown as Record<string, number[]>;
    for (const [name, m] of Object.entries(p.MODELS)) expect(models[name]).toEqual([m.pMax, m.vMax, m.tMax]);
    expect(Object.keys(p.MODELS)).toEqual(Object.keys(models));
  });

  it("float_to_uint / uint_to_float", () => {
    expect(mismatches(fx.float_to_uint!, ([v, lo, hi, bits, exp]) =>
      [outcome(() => p.floatToUint(num(v), lo as number, hi as number, bits as number), enc), exp as Outcome])).toEqual([]);
    expect(mismatches(fx.uint_to_float!, ([u, lo, hi, bits, exp]) =>
      [ok(enc(p.uintToFloat(u as number, lo as number, hi as number, bits as number))), ok(exp)])).toEqual([]);
  });

  it("encode_mit", () => {
    expect(fx.encode_mit!.length).toBeGreaterThan(1500);
    expect(mismatches(fx.encode_mit!, ([name, id, pos, vel, kp, kd, tau, exp]) => [
      outcome(() => p.encodeMit(id as number, num(pos), num(vel), num(kp), num(kd), num(tau), p.MODELS[name as string]!), frame),
      exp as Outcome,
    ])).toEqual([]);
  });

  it("encode_state", () => {
    expect(mismatches(fx.encode_state!, ([name, id, status, pos, vel, tau, tMos, tRotor, exp]) => [
      outcome(() => p.encodeState(id as number, status as number, num(pos), num(vel), num(tau), num(tMos), num(tRotor),
        p.MODELS[name as string]!), hex),
      exp as Outcome,
    ])).toEqual([]);
  });

  it("decode", () => {
    expect(fx.decode!.length).toBeGreaterThan(2000);
    expect(mismatches(fx.decode!, ([name, id, ext, data, exp]) =>
      [ok(decoded(p.decode(id as number, unhex(data as string), p.MODELS[name as string]!, ext as boolean))), ok(exp)])).toEqual([]);
  });

  it("command, refresh and save encoders", () => {
    const fns: Record<string, (id: number, mode?: number) => p.Frame> = {
      command_fb: (i, m) => p.encodeCommand(i, 0xfb, m), enable: p.encodeEnable, disable: p.encodeDisable,
      set_zero: p.encodeSetZero, clear_error: p.encodeClearError, refresh: (i) => p.encodeRefresh(i), save: (i) => p.encodeSave(i),
    };
    expect(mismatches(fx.commands!, ([name, id, mode, exp]) => {
      const n = name as string;
      const fn = fns[n] ?? ((i: number, m?: number) => p.encodeCommand(i, parseInt(n.slice(8), 16), m));
      return [outcome(() => fn(id as number, (mode ?? undefined) as number | undefined), frame), exp as Outcome];
    })).toEqual([]);
  });

  it("register encoders", () => {
    expect(fx.registers!.length).toBeGreaterThan(1000);
    expect(mismatches(fx.registers!, ([kind, id, rid, value, dtype, exp]) => [
      outcome(() => (kind === "read"
        ? p.encodeReadRegister(id as number, rid as number)
        : p.encodeWriteRegister(id as number, rid as number, num(value), dtype as string | null)), frame),
      exp as Outcome,
    ])).toEqual([]);
  });

  it("pack_value / unpack_value", () => {
    expect(mismatches(fx.pack_value!, ([v, dtype, exp]) =>
      [outcome(() => p.packValue(num(v), dtype as string), hex), exp as Outcome])).toEqual([]);
    expect(mismatches(fx.unpack_value!, ([raw, dtype, exp]) =>
      [outcome(() => p.unpackValue(unhex(raw as string), dtype as string), enc), exp as Outcome])).toEqual([]);
  });

  it("tables and predicates", () => {
    expect(mismatches(fx.register_dtype!, ([i, exp]) => [ok(p.registerDtype(i as number)), ok(exp)])).toEqual([]);
    expect(mismatches(fx.status_text!, ([s, exp]) => [ok(p.statusText(s as number | null)), ok(exp)])).toEqual([]);
    expect(mismatches(fx.is_fault_status!, ([s, exp]) => [ok(p.isFaultStatus(s as number)), ok(exp)])).toEqual([]);
    expect(mismatches(fx.is_command_id!, ([c, exp]) => [ok(p.isCommandId(c as number)), ok(exp)])).toEqual([]);
    expect(mismatches(fx.is_register_reply!, ([d, exp]) => [ok(p.isRegisterReply(unhex(d as string))), ok(exp)])).toEqual([]);
    expect(mismatches(fx.matching_model!, ([a, b, c, exp]) => [ok(p.matchingModel(num(a), num(b), num(c))), ok(exp)])).toEqual([]);
  });

  it("route", () => {
    expect(mismatches(fx.route!, ([id, ext, data, host, exp]) =>
      [ok(route(id as number, unhex(data as string), host as number, ext as boolean)), ok(exp)])).toEqual([]);
  });

  it("simulator helpers", () => {
    expect(mismatches(fx.feedback_bytes!, ([id, kw, exp]) => {
      const k = kw as Record<string, unknown>;
      const bytes = feedbackBytes(id as number, {
        pos: num(k.pos), vel: num(k.vel), tau: num(k.tau), temp: num(k.temp), status: k.status as number,
        rotorTemp: k.rotor_temp === null ? null : num(k.rotor_temp), model: k.model as string,
      });
      return [ok(hex(bytes)), ok(exp)];
    })).toEqual([]);
    expect(mismatches(fx.classify!, ([id, data, exp]) => [ok(classify(id as number, unhex(data as string))), ok(exp)])).toEqual([]);
  });
});
