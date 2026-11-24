/**
 * Stand-ins for the teleop tests (the Python SDKs' tests/unit/teleop_fakes.py): an object exposing
 * only the Arm API the leaders and the loop use, a FashionStar servo bus, and a scripted leader.
 */
import { ArmState, type Clock } from "@makermods/arm-core";
import type { ExtraTorque } from "../../src/arm.js";
import type { Leader } from "../../src/leaders/base.js";
import type { ServoBus } from "../../src/leaders/star.js";

export const NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "gripper"];

export interface FakeJoint {
  name: string;
  lo: number;
  hi: number;
  kp: number;
  kd: number;
  motorId: number;
  frictionViscous?: number;
  frictionCoulomb?: number;
  frictionRestNm?: number;
  frictionVelocityClamp?: number;
}

export function joint(name: string, lo = -2.0, hi = 2.0, extra: Partial<FakeJoint> = {}): FakeJoint {
  return { name, lo, hi, kp: 30.0, kd: 2.0, motorId: NAMES.indexOf(name) + 1, ...extra };
}

export interface FakeArmOptions {
  positions?: number[];
  joints?: FakeJoint[];
  track?: boolean;
  state?: ArmState;
  frictionDeadzone?: number;
}

export type Call = string | [string, ...unknown[]];

/** Arm API by duck typing. `track: true` makes the measured pose follow every accepted target. */
export class FakeArm {
  config: { joints: FakeJoint[]; frictionDeadzone?: number };
  positions: number[];
  velocities: number[];
  state: ArmState;
  track: boolean;
  targets: number[][] = [];
  calls: Call[] = [];
  faultReason: string | null = null;
  /** floatMode()'s arguments, or null before it was called. */
  floatArgs: [number | null | undefined, ExtraTorque | null | undefined] | [] | null = null;
  signal: AbortSignal | undefined;
  clock?: Clock;

  constructor(options: FakeArmOptions = {}) {
    this.config = { joints: options.joints ?? NAMES.map((n) => joint(n)) };
    if (options.frictionDeadzone !== undefined) this.config.frictionDeadzone = options.frictionDeadzone;
    this.positions = [...(options.positions ?? new Array<number>(this.config.joints.length).fill(0.0))];
    this.velocities = new Array<number>(this.positions.length).fill(0.0);
    this.state = options.state ?? ArmState.CONNECTED;
    this.track = options.track ?? true;
  }

  async connect(): Promise<void> {
    this.calls.push("connect");
    this.state = ArmState.CONNECTED;
  }

  async enable(options: { signal?: AbortSignal } = {}): Promise<void> {
    this.calls.push("enable");
    this.signal = options.signal;
    this.state = ArmState.ENABLED;
  }

  floatMode(...args: [number | null | undefined, ExtraTorque | null | undefined] | []): void {
    this.calls.push("float_mode");
    this.floatArgs = args;
  }

  setGains(kp?: number[] | null, kd?: number[] | null): void {
    this.calls.push(["set_gains", kp ?? null, kd ?? null]);
  }

  getJointPositions(): number[] {
    return [...this.positions];
  }

  getJointVelocities(): number[] {
    return [...this.velocities];
  }

  setJointTargets(targets: readonly number[]): boolean {
    if (this.state !== ArmState.ENABLED || targets.length !== this.positions.length || !targets.every(Number.isFinite)) {
      return false;
    }
    this.targets.push([...targets]);
    if (this.track) this.positions = [...targets];
    return true;
  }

  holdCurrentPosition(): boolean {
    this.calls.push("hold");
    return this.state === ArmState.ENABLED || this.state === ArmState.FAULT;
  }

  fault(reason = "overcurrent"): void {
    this.state = ArmState.FAULT;
    this.faultReason = reason;
  }
}

/** FashionStarBus by duck typing: `angles` is what each servo reports; `silent` never reply. */
export class FakeServoBus implements ServoBus {
  angles: Map<number, number>;
  /** Never answer syncMonitor. */
  silent: Set<number>;
  /** Never answer ping. */
  dead: Set<number>;
  calls: Array<[string, number]> = [];
  closed = false;
  raiseNext: Error | null = null;

  constructor(angles: Record<number, number>, silent: number[] = [], dead: number[] = []) {
    this.angles = new Map(Object.entries(angles).map(([k, v]) => [Number(k), v]));
    this.silent = new Set(silent);
    this.dead = new Set(dead);
  }

  async ping(sid: number): Promise<boolean> {
    this.calls.push(["ping", sid]);
    return !this.dead.has(sid);
  }

  async unlock(sid: number): Promise<void> {
    this.calls.push(["unlock", sid]);
  }

  async resetMultiTurn(sid: number): Promise<void> {
    this.calls.push(["reset_multi_turn", sid]);
  }

  async setOriginPoint(sid: number): Promise<void> {
    this.calls.push(["set_origin_point", sid]);
    this.angles.set(sid, 0.0);
  }

  async syncMonitor(ids: readonly number[]): Promise<Map<number, { angleDeg: number } | null>> {
    if (this.raiseNext !== null) {
      const error = this.raiseNext;
      this.raiseNext = null;
      throw error;
    }
    return new Map(ids.map((i) => [i, this.silent.has(i) ? null : { angleDeg: this.angles.get(i)! }]));
  }

  async close(): Promise<void> {
    this.closed = true;
  }
}

/** Returns `script` items in order (an Error item is thrown), then repeats the last one. */
export class ScriptedLeader implements Leader {
  readonly name = "scripted";
  script: Array<number[] | null | Error>;
  reads = 0;

  constructor(script: Array<number[] | null | Error>) {
    this.script = [...script];
  }

  async connect(): Promise<void> {}

  async read(): Promise<number[] | null> {
    this.reads += 1;
    const item = this.script.length > 1 ? this.script.shift()! : this.script[0]!;
    if (item instanceof Error) throw item;
    return item === null ? null : [...item];
  }

  async close(): Promise<void> {}
}

/** A clock `offset` seconds ahead of another (a test moves it to fake a frozen machine). */
export class OffsetClock implements Clock {
  offset = 0;
  constructor(readonly inner: Clock) {}
  now(): number {
    return this.inner.now() + this.offset;
  }
  sleep(seconds: number): Promise<void> {
    return this.inner.sleep(seconds);
  }
  timer(seconds: number, fn: () => void): () => void {
    return this.inner.timer(seconds, fn);
  }
}
