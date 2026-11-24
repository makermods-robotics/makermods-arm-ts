/**
 * The motors of one profile on their bus without an Arm, and a verified torque-off that works
 * where `Arm.connect` refuses (a motor reporting a fault, a motor not answering).
 *
 *     const result = await release(backend, config);   // opens and closes the bus
 *     result.ok          // every motor of the profile answered and confirmed torque off
 *
 * The arm drops where it is not resting on something: support it first.
 */

import { type Clock, RealClock } from "./clock.js";
import { type ArmConfig, JointConfig } from "./config.js";
import { ConnectError, FaultLatched, ParamTimeout } from "./errors.js";
import type { Motor, MotorFeedback } from "./motor.js";
import type { CanBackend } from "./transport/base.js";

export const RELEASE_ATTEMPTS = 3;
export const RELEASE_REPLY_S = 0.05;

export function jointPosition(joint: JointConfig, motorPosition: number): number {
  return (motorPosition - joint.offset) * joint.direction;
}

/**
 * A zero-gain command at the measured position (a later enable cannot resume an old setpoint),
 * then a disable whose reply must confirm torque off.
 */
export async function releaseMotor(m: Motor, clock: Clock): Promise<boolean> {
  const fb = m.feedback;
  if (fb !== null) {
    await m.sendCommand(fb.position, 0.0, 0.0, 0.0, 0.0);
    await clock.sleep(0.005);
  }
  return m.disableConfirmed(RELEASE_REPLY_S, RELEASE_ATTEMPTS);
}

/** One motor's state from Bus.read(), joint coordinates. */
export interface BusRow {
  motorId: number;
  /** Joint name, null for a motor the profile does not name. */
  joint: string | null;
  position: number | null;
  velocity: number | null;
  torque: number | null;
  temperature: number | null;
  temperatures: Record<string, number>;
  enabled: boolean | null;
  fault: string | null;
  faultCode: number;
  cleared: boolean;
  outsideLimits: boolean;
}

/** Bus.release() / release(). */
export interface ReleaseResult {
  ok: boolean;
  released: Array<string | number>;
  unconfirmed: Array<string | number>;
  missing: string[];
}

/**
 * The motors of one profile on their bus, without an Arm. Reads never change torque; only
 * release() does. Works when a motor reports a fault (Arm.connect refuses then).
 */
export class Bus {
  readonly backend: CanBackend;
  readonly config: ArmConfig;
  readonly clock: Clock;
  readonly joints: Map<number, JointConfig>;
  readonly motors: Map<number, Motor>;
  readonly known: Set<number>;

  constructor(backend: CanBackend, config: ArmConfig, ids?: number[] | null, clock: Clock = new RealClock()) {
    this.backend = backend;
    this.config = config;
    this.clock = clock;
    const joints = new Map(config.joints.map((j) => [j.motorId, j]));
    const list = ids === undefined || ids === null ? [...joints.keys()] : ids;
    const product = config.product;
    this.joints = new Map(list.map((i) => [i, joints.get(i)
      ?? new JointConfig({ motorId: i, lo: -Math.PI, hi: Math.PI, kp: 0.0, kd: 0.0, model: product.defaultModel })]));
    this.motors = new Map([...this.joints].map(([i, j]) => [i, product.createMotor(j, backend, config.hostId, clock)]));
    this.known = new Set(joints.keys());
  }

  #receive = (canId: number, data: Uint8Array, extended: boolean): void => {
    const id = this.config.product.route(canId, data, this.config.hostId, extended);
    const m = id === null ? undefined : this.motors.get(id);
    if (m !== undefined) m.handleFrame(canId, data, extended);
  };

  /** Install the receive path and open the bus (the Python SDK's `with Bus(...)`). */
  async open(): Promise<this> {
    this.backend.setRecvCallback(this.#receive);
    try {
      await this.backend.open();
    } catch (e) {
      throw new ConnectError(
        `could not open the CAN adapter: ${e instanceof Error ? e.message : String(e)}. Close other programs using it and retry.`,
        { cause: e });
    }
    return this;
  }

  async close(): Promise<void> {
    await this.backend.close();
  }

  /** One motor's state in joint coordinates, or null when it does not answer. */
  async read(motorId: number, clear = false, timeout = 0.1): Promise<BusRow | null> {
    const m = this.motors.get(motorId)!, j = this.joints.get(motorId)!;
    let status;
    try {
      status = await m.check(timeout, 2);
    } catch (e) {
      if (e instanceof ParamTimeout) return null;
      throw e;
    }
    let cleared = false;
    if (status.hasFault && clear) {
      await m.clearFault();
      await this.clock.sleep(0.01);
      status = await m.check(timeout, 2);
      cleared = true;
    }
    let fb: MotorFeedback | null = null;
    if (!status.hasFault) {
      try {
        fb = await m.readState(timeout / 2, 3);
      } catch (e) {
        if (!(e instanceof ParamTimeout || e instanceof FaultLatched)) throw e;
      }
    }
    const fault = status.hasFault ? (status.text || `code 0x${status.code.toString(16).toUpperCase()}`) : m.faultLatched;
    const row: BusRow = {
      motorId, joint: this.known.has(motorId) ? j.name : null,
      position: null, velocity: null, torque: null, temperature: null, temperatures: {},
      enabled: null, fault, faultCode: status.hasFault ? status.code : 0, cleared, outsideLimits: false,
    };
    if (fb !== null) {
      const q = jointPosition(j, fb.position);
      Object.assign(row, {
        position: q, velocity: fb.velocity * j.direction, torque: fb.torque * j.direction,
        temperature: fb.temperature, temperatures: { ...fb.temperatures }, enabled: fb.enabled,
      });
      if (this.known.has(motorId)) {
        const grace = this.config.fullTurnGrace;
        row.outsideLimits = ![0.0, -2 * Math.PI, 2 * Math.PI].some((k) => j.lo - grace <= q + k && q + k <= j.hi + grace);
      }
    }
    return row;
  }

  async readAll(clear = false): Promise<BusRow[]> {
    const rows: Array<BusRow | null> = [];
    for (const i of this.motors.keys()) rows.push(await this.read(i, clear));
    return rows.filter((r): r is BusRow => r !== null);
  }

  missing(rows: BusRow[]): number[] {
    const found = new Set(rows.map((r) => r.motorId));
    return [...this.motors.keys()].filter((i) => this.known.has(i) && !found.has(i));
  }

  /**
   * Torque off, verified, on every motor in `rows` (default: every motor that answers now). `ok`
   * only when every motor of the profile answered and confirmed it; `unconfirmed` and `missing`
   * name the others (joint names, motor ids where the joint is unknown).
   */
  async release(rows?: BusRow[] | null): Promise<ReleaseResult> {
    const list = rows === undefined || rows === null ? await this.readAll() : rows;
    const unconfirmed: Array<string | number> = [];
    for (const r of list) {
      if (!(await releaseMotor(this.motors.get(r.motorId)!, this.clock))) unconfirmed.push(r.joint || r.motorId);
    }
    const missing = this.missing(list).map((i) => this.joints.get(i)!.name);
    const released = list.map((r) => r.joint || r.motorId).filter((n) => !unconfirmed.includes(n));
    return { ok: list.length > 0 && !unconfirmed.length && !missing.length, released, unconfirmed, missing };
  }
}

/**
 * Torque off, verified, motor by motor, on an arm this process has not connected (`backend`
 * closed: opened and closed here): see Bus.release(). Throws ConnectError when the bus cannot be
 * opened; a bus where no motor answers gives `ok` false with every joint `missing`.
 */
export async function release(backend: CanBackend, config: ArmConfig, clock: Clock = new RealClock()): Promise<ReleaseResult> {
  const bus = await new Bus(backend, config, null, clock).open();
  try {
    return await bus.release();
  } finally {
    await bus.close();
  }
}
