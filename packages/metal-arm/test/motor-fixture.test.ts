/**
 * The Motor replayed against contract/fixtures/metal_motor.json (tools/gen_metal_motor.py): the
 * Python Motor on the Python SimBus under a fake clock. Every step must send the same frames in
 * the same order, return the same result, leave the same fake time and the same Motor / SimMotor
 * state.
 */

import { JointConfig } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import { describe, expect, it } from "vitest";
import { Motor, type MotorFeedback, route } from "../src/motor.js";
import { SimBus, SimMotor, type SimMotorOptions } from "../src/sim-motor.js";
import { enc, hex, loadFixture, num, unhex } from "./fixture.js";

interface Step {
  op: string;
  motor?: number;
  method?: string;
  args?: unknown[];
  set?: Record<string, unknown>;
  rid?: number;
  value?: number;
  frames?: [number, string][];
  dt?: number;
  can_id?: number;
  data?: string;
  extended?: boolean;
}

interface Scenario {
  name: string;
  host_id?: number;
  callback?: "dispatch" | "direct";
  motors: { id: number; model?: string; joint_model?: string; options?: Record<string, unknown>; joint?: Record<string, unknown> }[];
  steps: Step[];
  records: { sent: [number, string, boolean][]; result: Record<string, unknown> | null; time: number; motors: unknown; sims: unknown }[];
}

const fixture = loadFixture<{ start: number; scenarios: Scenario[] }>("metal_motor.json");

const camel = (key: string): string => key.replace(/_([a-z0-9])/g, (_, c: string) => c.toUpperCase());
const arg = (x: unknown): unknown => (x === null ? undefined : x === "nan" || x === "inf" || x === "-inf" ? num(x) : x);

const METHODS: Record<string, string> = {
  check: "check", read_state: "readState", validate: "validate", preload: "preload", enable_confirmed: "enableConfirmed",
  disable_confirmed: "disableConfirmed", clear_fault: "clearFault", set_zero: "setZero", arm_watchdog: "armWatchdog",
  restore_watchdog: "restoreWatchdog", read_param: "readParam", write_param: "writeParam", set_id: "setId",
  send_command: "sendCommand", poll_status: "pollStatus", enable: "enable", disable: "disable", confirms: "confirms",
  health_problem: "healthProblem",
};

function feedback(fb: MotorFeedback | null): unknown {
  if (fb === null) return null;
  return {
    motor_id: fb.motorId, position: enc(fb.position), velocity: enc(fb.velocity), torque: enc(fb.torque),
    temperature: enc(fb.temperature), temperatures: { mos: fb.temperatures.mos, rotor: fb.temperatures.rotor },
    enabled: fb.enabled, warning: fb.warning, status: fb.status,
  };
}

function jsonable(v: unknown): unknown {
  if (v === undefined || v === null) return null;
  if (typeof v === "number") return enc(v);
  if (typeof v !== "object") return v;
  if ("hasFault" in v) {
    const s = v as { motorId: number; hasFault: boolean; code: number; text: string; enabled: boolean | null };
    return { motor_id: s.motorId, has_fault: s.hasFault, code: s.code, text: s.text, enabled: s.enabled };
  }
  if ("temperatures" in v) return feedback(v as MotorFeedback);
  return v;
}

const motorSnapshot = (m: Motor) => ({
  motor_id: m.motorId, reply_id: m.replyId, flush_id: m.flushId, feedback: feedback(m.feedback), status: jsonable(m.status),
  fault_latched: m.faultLatched, feedback_sequence: m.feedbackSequence, feedback_age: enc(m.feedbackAge),
  watchdog_released: m.watchdogReleased,
});

const simSnapshot = (s: SimMotor) => ({
  send_id: s.sendId, reply_id: s.replyId, position: enc(s.position), enabled: s.enabled, fault: s.fault, drop: s.drop,
  held: s.held.map(([c, d]) => [c, hex(d)]), log: s.log, registers: s.registers, flash: s.flash,
});

/** Our own errors must match by class and message; Python's struct/overflow errors are ValueErrors here. */
function sameResult(actual: Record<string, unknown> | null, expected: Record<string, unknown> | null): boolean {
  if (expected === null || "ok" in expected) return JSON.stringify(actual) === JSON.stringify(expected);
  if (actual === null || !("error" in actual)) return false;
  if (["error", "OverflowError"].includes(expected.error as string)) return actual.error === "ValueError";
  return actual.error === expected.error && actual.message === expected.message;
}

async function replay(sc: Scenario) {
  const clock = new VirtualClock({ start: fixture.start });
  const host = sc.host_id ?? 0x10;
  const sims = sc.motors.map((s) => {
    const options: Record<string, unknown> = {};
    for (const [k, v] of Object.entries(s.options ?? {})) options[camel(k)] = v;
    return new SimMotor(s.id, s.model ?? "jlo", options as SimMotorOptions);
  });
  const bus = new SimBus(sims);
  const motors = new Map<number, Motor>();
  for (const s of sc.motors) {
    const extra: Record<string, unknown> = {};
    for (const [k, v] of Object.entries(s.joint ?? {})) extra[camel(k)] = v;
    const joint = new JointConfig({ motorId: s.id, model: s.joint_model ?? s.model ?? "jlo", lo: -1.0, hi: 1.0, kp: 20.0,
      kd: 0.6, name: "", ...extra });
    motors.set(s.id, new Motor(joint, bus, host, clock));
  }
  if (sc.callback === "direct") {
    const m = motors.get(sc.motors[0]!.id)!;
    bus.setRecvCallback((canId, data, extended) => m.handleFrame(canId, data, extended));
  } else {
    bus.setRecvCallback((canId, data, extended) => {
      const id = route(canId, data, host, extended);
      const m = id === null ? undefined : motors.get(id);
      m?.handleFrame(canId, data, extended);
    });
  }
  const records: Scenario["records"] = [];
  for (const step of sc.steps) {
    const before = bus.sent.length;
    let result: Record<string, unknown> | null = null;
    try {
      if (step.op === "call") {
        const m = motors.get(step.motor!)! as unknown as Record<string, (...a: unknown[]) => unknown>;
        const fn = METHODS[step.method!]!;
        result = { ok: jsonable(await m[fn]!(...(step.args ?? []).map(arg))) };
      } else if (step.op === "sim") {
        const s = bus.motors.get(step.motor!)! as unknown as Record<string, unknown>;
        for (const [k, v] of Object.entries(step.set!)) s[camel(k)] = arg(v);
      } else if (step.op === "register") {
        bus.motors.get(step.motor!)!.registers[step.rid!] = step.value!;
      } else if (step.op === "held") {
        bus.motors.get(step.motor!)!.held = step.frames!.map(([c, d]) => [c, unhex(d)]);
      } else if (step.op === "power_cycle") {
        bus.motors.get(step.motor!)!.powerCycle();
      } else if (step.op === "advance") {
        await clock.advance(step.dt!);
      } else if (step.op === "handle_frame") {
        motors.get(step.motor!)!.handleFrame(step.can_id!, unhex(step.data!), step.extended ?? false);
      } else {
        throw new Error(step.op);
      }
    } catch (e) {
      result = { error: (e as Error).name, message: (e as Error).message };
    }
    const sent = bus.sent.slice(before).map(([c, d], i) => [c, hex(d), bus.sentExtended[before + i]!] as [number, string, boolean]);
    records.push({
      sent, result, time: clock.now(),
      motors: JSON.parse(JSON.stringify(Object.fromEntries([...motors].map(([k, m]) => [String(k), motorSnapshot(m)])))),
      sims: JSON.parse(JSON.stringify(Object.fromEntries([...bus.motors].map(([k, s]) => [String(k), simSnapshot(s)])))),
    });
  }
  return records;
}

describe("motor fixture", () => {
  it("covers the queries and the field behaviour", () => {
    const names = fixture.scenarios.map((s) => s.name);
    for (const n of ["healthy", "faulted_comm_lost", "offline", "drop", "late_gripper", "late_duplicate_gripper",
      "stale_register_echo", "set_id", "set_id_late_gripper"]) expect(names).toContain(n);
    expect(fixture.scenarios.flatMap((s) => s.records).reduce((n, r) => n + r.sent.length, 0)).toBeGreaterThan(1500);
  });

  for (const sc of fixture.scenarios) {
    it(sc.name, async () => {
      const records = await replay(sc);
      for (const [i, expected] of sc.records.entries()) {
        const actual = records[i]!;
        const where = `${sc.name} step ${i} ${JSON.stringify(sc.steps[i])}`;
        expect(actual.sent, where).toEqual(expected.sent);
        expect(sameResult(actual.result, expected.result), `${where}: ${JSON.stringify(actual.result)} vs ${JSON.stringify(expected.result)}`).toBe(true);
        expect(actual.time, where).toBe(expected.time);
        expect(actual.motors, where).toEqual(expected.motors);
        expect(actual.sims, where).toEqual(expected.sims);
      }
    });
  }
});
