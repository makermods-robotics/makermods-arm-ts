// Replays contract/fixtures/maker_protocol.json (tools/gen_maker_protocol.py): every byte and every
// decoded number must match the Python codec exactly.
import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import { protocol as p } from "@makermods/maker-arm";
import { feedbackBytes } from "@makermods/maker-arm/testing";

type Num = number | "nan" | "inf" | "-inf";
interface Outcome {
  ok?: { id: number; data: string };
  error?: string;
}

const fixture = JSON.parse(readFileSync(new URL("../../../contract/fixtures/maker_protocol.json", import.meta.url), "utf8"));

const num = (x: Num): number => (x === "nan" ? NaN : x === "inf" ? Infinity : x === "-inf" ? -Infinity : x);
const hex = (data: Uint8Array): string => Buffer.from(data).toString("hex");
const bytes = (h: string): Uint8Array => Uint8Array.from(Buffer.from(h, "hex"));

function outcome(fn: () => p.Frame): Outcome {
  try {
    const [id, data] = fn();
    return { ok: { id, data: hex(data) } };
  } catch (e) {
    return { error: (e as Error).name };
  }
}

function check(expected: Outcome, actual: Outcome, label: unknown): void {
  if (expected.ok) expect(actual, JSON.stringify(label)).toEqual({ ok: expected.ok });
  else expect(actual.error, JSON.stringify(label)).toBeDefined();
}

/** Exact equality, NaN equal to NaN, -0 distinct from 0. */
function same(a: number, b: number): boolean {
  return Object.is(a, b);
}

describe("RobStride codec matches the Python SDK", () => {
  it("encodes MIT control frames byte for byte", () => {
    expect(fixture.control.length).toBeGreaterThan(3000);
    for (const c of fixture.control) {
      const [id, ...args] = (c.args as Num[]).map(num) as [number, number, number, number, number, number];
      check(c, outcome(() => p.encodeControl(id, ...args, p.MOTOR_PARAMS[c.model]!)), c);
    }
  });

  it("encodes commands", () => {
    const fns: Record<string, (...a: number[]) => p.Frame> = {
      enable: p.encodeEnable, disable: p.encodeDisable, clear_fault: p.encodeClearFault,
      fault_query: p.encodeFaultQuery, set_zero: p.encodeSetZero, command: p.command,
    };
    for (const c of fixture.commands) check(c, outcome(() => fns[c.fn]!(...c.args)), c);
  });

  it("encodes parameter reads and writes", () => {
    for (const c of fixture.params) {
      const args = c.args as [number, number, Num, string];
      const actual = c.fn === "read"
        ? outcome(() => p.encodeReadParam(args[0], args[1]))
        : outcome(() => p.encodeWriteParam(args[0], args[1], num(args[2]), args[3]));
      check(c, actual, c);
    }
  });

  it("decodes every received frame to the same values", () => {
    for (const c of fixture.parse) {
      const data = bytes(c.data);
      const msg = p.parseFrame(c.id, data, { params: p.MOTOR_PARAMS[c.model]!, hostId: c.host_id, extended: c.extended });
      expect(p.replyMotorId(c.id, data, c.extended), c.data).toBe(c.reply_motor_id);
      const r = c.result;
      if (r === null) {
        expect(msg, `${c.id} ${c.data}`).toBeNull();
        continue;
      }
      expect(msg?.kind, `${c.id} ${c.data}`).toBe(r.kind);
      if (msg!.kind === "feedback") {
        const ok = same(msg!.position, r.position) && same(msg!.velocity, r.velocity) && same(msg!.torque, r.torque)
          && same(msg!.temperature, r.temperature);
        expect(ok, `${c.data}: ${JSON.stringify(msg)} vs ${JSON.stringify(r)}`).toBe(true);
        expect([msg!.motorId, msg!.mode, msg!.faultFlag, msg!.warningFlag]).toEqual([r.motor_id, r.mode, r.fault_flag, r.warning_flag]);
      } else if (msg!.kind === "status") {
        expect([msg!.motorId, msg!.word, msg!.hasFault, msg!.text]).toEqual([r.motor_id, r.word, r.has_fault, r.text]);
      } else {
        expect([msg!.motorId, msg!.index, hex(msg!.raw), msg!.write]).toEqual([r.motor_id, r.index, r.raw, r.write]);
        for (const [dtype, value] of Object.entries(r.values as Record<string, Num>)) {
          expect(same(msg!.value(dtype), num(value)), `${c.data} ${dtype}`).toBe(true);
        }
      }
    }
  });

  it("describes fault words", () => {
    for (const c of fixture.fault_text) expect(p.faultText(c.word)).toBe(c.text);
  });

  it("simulated feedback frames match", () => {
    for (const c of fixture.sim_feedback) {
      const data = feedbackBytes(c.motor_id, {
        pos: c.pos, vel: c.vel, tau: c.tau, temp: c.temp, mode: c.mode, fault: c.fault, warn: c.warn,
        params: p.MOTOR_PARAMS[c.model]!,
      });
      expect(hex(data), JSON.stringify(c)).toBe(c.data);
    }
  });
});
