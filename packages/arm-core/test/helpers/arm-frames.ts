// Replays a frame fixture written by tools/gen_arm_frames.py --product <name>: the real Python Arm on
// its simulated bus under a fake clock. The TypeScript Arm on VirtualClock must send the identical
// frames, in order, and end every operation in the same state with the same commanded positions.
// An arm package's test calls replayArmFrames() with its product's pieces.
import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import { Arm, type ArmConfig, type CanBackend, type JointConfig, type Logger, type RawProfile } from "../../src/index.js";
import { type SimulatedMotor, VirtualClock } from "../../src/testing.js";

interface Op {
  op: string;
  [key: string]: unknown;
}

interface Step {
  frames: Array<[number, string, boolean]>;
  result: unknown;
  error: [string, string] | null;
  state: string;
  commanded: number[];
  fault_reason: string | null;
  t: number;
}

interface Scenario {
  name: string;
  start: number[];
  overrides: RawProfile;
  /** Options for every simulated motor (snake_case, as the Python SDK's). */
  motor?: Record<string, unknown>;
  /** Joint name -> options for that joint's simulated motor. */
  motor_joints?: Record<string, Record<string, unknown>>;
  ops: Op[];
  steps: Step[];
  loose_time_from?: number;
}

/** What one arm package supplies to replay its fixture. */
export interface FrameReplayProduct<M extends SimulatedMotor> {
  /** The fixture file (contract/fixtures/arm_frames_<product>.json). */
  fixture: URL | string;
  /** The shipped profile with raw snake_case overrides, validated. */
  defaultConfig(overrides: RawProfile): ArmConfig;
  /** A simulated motor for `joint` at `position` (motor rad); `options` camelCase. */
  forJoint(joint: JointConfig, hostId: number, position: number, options: Record<string, unknown>): M;
  /** A backend answering with `motors`. */
  bus(motors: M[]): CanBackend;
  watchdogTimeout(motor: M): void;
}

const silent: Logger = { info() {}, warn() {}, error() {} };
const hex = (data: Uint8Array): string => Buffer.from(data).toString("hex");
const camel = (key: string): string => key.replace(/_([a-z0-9])/g, (_, c: string) => c.toUpperCase());
const camelKeys = (o: Record<string, unknown> = {}): Record<string, unknown> =>
  Object.fromEntries(Object.entries(o).map(([k, v]) => [camel(k), v]));

async function replay<M extends SimulatedMotor>(p: FrameReplayProduct<M>, s: Scenario): Promise<Step[]> {
  const clock = new VirtualClock();
  const config = p.defaultConfig(s.overrides);
  const labels = config.joints.map((j) => j.label);
  const motors = config.joints.map((j, i) => p.forJoint(j, config.hostId, s.start[i]! * j.direction + j.offset,
    { ...camelKeys(s.motor), ...camelKeys(s.motor_joints?.[j.label]) }));
  const bus = p.bus(motors);
  let frames: Array<[number, string, boolean]> = [];
  const send = bus.send.bind(bus);
  bus.send = (canId: number, data: Uint8Array, extended = true): Promise<void> => {
    frames.push([canId, hex(data), extended]);
    return send(canId, data, extended);
  };
  const make = () => new Arm(config, bus, { clock, logger: silent });
  const motor = (op: Op): M => motors[labels.indexOf(op.joint as string)]!;
  let arm = make();
  const dt = 1.0 / config.controlRateHz;
  const steps: Step[] = [];
  for (const op of s.ops) {
    frames = [];
    let result: unknown = null;
    let error: [string, string] | null = null;
    try {
      switch (op.op) {
        case "new_arm":
          arm = make();
          break;
        case "connect":
          await arm.connect(Boolean(op.clear_faults), Boolean(op.clear_released));
          break;
        case "enable":
          await arm.enable({ startLoop: op.start_loop as boolean });
          break;
        case "attach":
          await arm.attach();
          break;
        case "tick":
          if (op.targets !== null) result = arm.setJointTargets(op.targets as number[]);
          await arm["tick"](dt);
          await clock.sleep(dt);
          break;
        case "set_targets":
          result = arm.setJointTargets(op.targets as number[]);
          break;
        case "sleep":
          await clock.sleep(op.s as number);
          break;
        case "sim":
          (motor(op) as unknown as Record<string, unknown>)[camel(op.attr as string)] = op.value;
          break;
        case "paced":
          bus.feedbackPacedControl = op.value as boolean;
          break;
        case "sim_watchdog":
          p.watchdogTimeout(motor(op));
          break;
        case "stop":
          result = await arm.stop();
          break;
        case "return_to_rest":
          result = await arm.returnToRest(op.rest as number[]);
          break;
        case "move_to":
          result = await arm.moveTo(op.q as number[], { speed: op.speed as number, timeout: op.timeout as number });
          break;
        case "hold":
          result = arm.holdCurrentPosition();
          break;
        case "set_gains":
          arm.setGains(op.kp as number[] | undefined, op.kd as number[] | undefined);
          break;
        case "float_mode":
          arm.floatMode(op.kd as number | undefined);
          break;
        case "disable":
          await arm.disable();
          break;
        case "estop":
          await arm.estop();
          break;
        case "detach":
          result = Object.entries(await arm.detach()).sort();
          break;
        case "disconnect":
          await arm.disconnect();
          break;
        case "clear_faults":
          await arm.clearFaults();
          break;
        default:
          throw new Error(`unknown op ${op.op}`);
      }
    } catch (e) {
      error = [(e as Error).name, (e as Error).message];
    }
    steps.push({
      frames, result: result ?? null, error, state: arm.state, commanded: arm.getCommandedPositions(),
      fault_reason: arm.faultReason, t: clock.now(),
    });
  }
  return steps;
}

/** MIT command frame fields that differ (for a readable failure). */
function frameDiff(want: [number, string, boolean], got: [number, string, boolean] | undefined): string {
  if (!got) return `missing frame ${JSON.stringify(want)}`;
  return `id ${want[0]}/${got[0]} data ${want[1]} (python) vs ${got[1]} (ts)`;
}

/** One vitest case per scenario of the product's fixture. */
export function replayArmFrames<M extends SimulatedMotor>(p: FrameReplayProduct<M>): void {
  const fixture = JSON.parse(readFileSync(p.fixture, "utf8")) as { scenarios: Scenario[] };
  describe("Arm frames match the Python SDK", () => {
    it("has scenarios", () => expect(fixture.scenarios.length).toBeGreaterThan(0));
    for (const s of fixture.scenarios) {
      it(s.name, async () => {
        const steps = await replay(p, s);
        expect(steps.length).toBe(s.steps.length);
        s.steps.forEach((want, i) => {
          const got = steps[i]!;
          const where = `${s.name} step ${i} (${JSON.stringify(s.ops[i]).slice(0, 80)})`;
          const k = want.frames.findIndex((f, n) => JSON.stringify(f) !== JSON.stringify(got.frames[n]));
          expect(k < 0 && got.frames.length === want.frames.length,
            `${where}: frame ${k} ${k < 0 ? "extra frames" : frameDiff(want.frames[k]!, got.frames[k])}`).toBe(true);
          expect(got.error?.[0] ?? null, `${where}: error ${JSON.stringify(got.error)} vs ${JSON.stringify(want.error)}`)
            .toBe(want.error?.[0] ?? null);
          expect(got.result, `${where}: result`).toEqual(want.result);
          expect(got.state, `${where}: state`).toBe(want.state);
          expect(got.fault_reason, `${where}: fault reason`).toBe(want.fault_reason);
          const exact = got.commanded.length === want.commanded.length
            && got.commanded.every((v, n) => Object.is(v, want.commanded[n]));
          expect(exact, `${where}: commanded ${JSON.stringify(got.commanded)} vs ${JSON.stringify(want.commanded)}`).toBe(true);
          if (s.loose_time_from === undefined || i < s.loose_time_from) {
            expect(got.t, `${where}: time`).toBe(want.t);
          } else {
            expect(Math.abs(got.t - want.t), `${where}: time`).toBeLessThan(0.01);
          }
        });
      });
    }
  });
}
