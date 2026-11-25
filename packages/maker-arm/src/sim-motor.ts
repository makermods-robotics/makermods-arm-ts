/**
 * Simulated RobStride MIT motors behind MockBackend, for tests and "/testing".
 *
 * A SimMotor answers the frames the SDK sends: MIT commands (an enabled motor jumps to the
 * commanded position when kp > 0), enable/disable/zero/clear, the non-clearing status query and,
 * when `paramsSupported`, parameter reads/writes (run mode, CAN watchdog). `log` records what it
 * received ("status_query", "clear", "enable", "disable", "zero", ["set_id", new id], ["cmd", kp],
 * ["write", index]),
 * `frames` every MIT command as [position, kp, tau].
 */

import { type JointConfig, MockBackend } from "@makermods/arm-core";
import * as p from "./protocol.js";

export const PREFIX = Uint8Array.from([0xff, 0xff, 0xff, 0xff, 0xff, 0xff]);
/** What `faulted = true` reports. */
export const FAULT_CODE = 1;
/** How the SDK describes FAULT_CODE. */
export const FAULT_TEXT = "motor overtemperature";

// What the protocol can show:
/** A reading one whole turn off fits the position range (multi-turn count). */
export const FULL_TURN_SUPPORTED = true;
/** Feedback carries a warning flag (`warning: true`). */
export const WARNING_SUPPORTED = true;

export interface FeedbackOptions {
  pos?: number;
  vel?: number;
  tau?: number;
  temp?: number;
  mode?: number;
  fault?: boolean;
  warn?: boolean;
  params?: p.MotorParams;
}

/** The 8-byte motion reply a motor would send. */
export function feedbackBytes(motorId: number, options: FeedbackOptions = {}): Uint8Array {
  const { pos = 0.0, vel = 0.0, tau = 0.0, temp = 30.0, mode = 0, fault = false, warn = false, params = p.RS00 } = options;
  const q = p.floatToUint(pos, params.pMin, params.pMax, 16);
  const v = p.floatToUint(vel, params.vMin, params.vMax, 12);
  const t = p.floatToUint(tau, params.tMin, params.tMax, 12);
  const temp10 = Math.trunc(p.roundHalfEven(temp * 10));
  const status = (mode << 6) | (Number(fault) << 5) | (Number(warn) << 4) | ((temp10 >> 8) & 0x0f);
  return Uint8Array.from([motorId, q >> 8, q & 0xff, v >> 4, ((v & 0xf) << 4) | (t >> 8), t & 0xff, status & 0xff,
    temp10 & 0xff]);
}

/** Kind of a host->motor frame. */
export type FrameKind =
  | "command" | "enable" | "disable" | "zero" | "set_id" | "clear" | "status_query" | "param_read" | "param_write"
  | "other";

/** [kind, motor id] of a host->motor frame. */
export function classify(canId: number, data: Uint8Array): [FrameKind, number] {
  const kind = canId & 0x700;
  if (kind === 0x300) return ["param_read", canId & 0xff];
  if (kind === 0x400) return ["param_write", canId & 0xff];
  if (data.length >= 6 && PREFIX.every((b, i) => data[i] === b)) {
    const arg = data[6], op = data[7];
    let name: FrameKind | undefined = (
      { [p.OP_ENABLE]: "enable", [p.OP_DISABLE]: "disable", [p.OP_SET_ZERO]: "zero", [p.OP_SET_ID]: "set_id" } as
        Record<number, FrameKind>)[op!];
    if (op === p.OP_CLEAR_FAULT) name = arg === 0x00 ? "status_query" : "clear";
    return [name ?? "other", canId];
  }
  return ["command", canId];
}

/** One entry of SimMotor.log. */
export type SimLogEntry = string | [string, number];

export interface SimMotorOptions {
  position?: number;
  temperature?: number;
  enabled?: boolean;
  online?: boolean;
  /** An enabled motor jumps to the commanded position. */
  track?: boolean;
  faulted?: boolean;
  warning?: boolean;
  faultWord?: number | null;
  faultFlag?: boolean;
  /** false: old firmware, no parameter access, mode bits read 0. */
  paramsSupported?: boolean;
  runMode?: number;
  canTimeout?: number;
  /** CAN id replies are sent to. */
  hostId?: number;
}

function u32le(value: number): Uint8Array {
  const out = new Uint8Array(4);
  new DataView(out.buffer).setUint32(0, value >>> 0, true);
  return out;
}

function concat(a: Uint8Array, b: Uint8Array): Uint8Array {
  const out = new Uint8Array(a.length + b.length);
  out.set(a, 0);
  out.set(b, a.length);
  return out;
}

/** A simulated RobStride motor. */
export class SimMotor {
  motorId: number;
  model: string;
  params: p.MotorParams;
  position: number;
  temperature: number;
  enabled: boolean;
  online: boolean;
  track: boolean;
  warning: boolean;
  faultWord: number;
  faultFlag: boolean;
  paramsSupported: boolean;
  runMode: number;
  canTimeout: number;
  hostId: number;
  /** N*m and rad/s the motor reports (a held load, a hand moving it). */
  torque = 0.0;
  velocity = 0.0;
  /** false: an id write is taken but the motor keeps its id. */
  applyIdChange = true;
  /** The MCU identifier an id write returns (little-endian u64). */
  uid: Uint8Array;
  log: SimLogEntry[] = [];
  lastKp = 0.0;
  /** Every MIT frame received: [position, kp, tau]. */
  frames: Array<[number, number, number]> = [];

  constructor(motorId: number, model = "RS00", options: SimMotorOptions = {}) {
    this.motorId = motorId;
    this.model = model;
    this.params = p.MOTOR_PARAMS[model]!;
    this.position = options.position ?? 0.0;
    this.temperature = options.temperature ?? 30.0;
    this.enabled = options.enabled ?? false;
    this.online = options.online ?? true;
    this.track = options.track ?? true;
    this.warning = options.warning ?? false;
    const faultWord = options.faultWord ?? null;
    this.faultWord = faultWord === null ? (options.faulted ? FAULT_CODE : 0) : faultWord;
    this.faultFlag = options.faultFlag ?? false;
    this.paramsSupported = options.paramsSupported ?? false;
    this.runMode = options.runMode ?? 0;
    this.canTimeout = options.canTimeout ?? 0;
    this.hostId = options.hostId ?? p.HOST_CAN_ID;
    this.uid = new Uint8Array(8);
    new DataView(this.uid.buffer).setBigUint64(0, 0x5354524944450000n | BigInt(motorId), true);
  }

  get faulted(): boolean {
    return Boolean(this.faultWord || this.faultFlag);
  }

  set faulted(value: boolean) {
    this.faultWord = value ? FAULT_CODE : 0;
    if (!value) this.faultFlag = false;
  }

  get mode(): number {
    return this.paramsSupported ? (this.enabled ? 2 : 0) : 0;
  }

  feedback(): [number, Uint8Array] {
    return [this.hostId, feedbackBytes(this.motorId, {
      pos: this.position, vel: this.velocity, tau: this.torque, temp: this.temperature, mode: this.mode,
      fault: this.faultFlag, warn: this.warning, params: this.params,
    })];
  }

  handle(canId: number, data: Uint8Array): Array<[number, Uint8Array]> {
    if (!this.online) return [];
    const [kind] = classify(canId, data);
    if (kind === "param_read" || kind === "param_write") {
      if (!this.paramsSupported) return [];
      const view = new DataView(data.buffer, data.byteOffset, data.length);
      const idx = view.getUint16(0, true);
      if (kind === "param_write") {
        this.log.push(["write", idx]);
        if (idx === p.ParamIndex.CAN_TIMEOUT) this.canTimeout = view.getUint32(4, true);
        return [[canId, Uint8Array.from(data)]];
      }
      const values: Record<number, number> = { [p.ParamIndex.RUN_MODE]: this.runMode, [p.ParamIndex.CAN_TIMEOUT]: this.canTimeout };
      return [[canId, concat(data.subarray(0, 4), u32le(values[idx] ?? 0))]];
    }
    if (kind === "status_query") {
      this.log.push("status_query");
      return [[this.hostId, concat(Uint8Array.of(this.motorId), u32le(this.faultWord))]];
    }
    if (kind === "set_id") {
      this.log.push(["set_id", data[6]!]);
      if (this.applyIdChange && !this.enabled) this.motorId = data[6]!;
      return [[this.motorId, Uint8Array.from(this.uid)]];
    }
    if (kind !== "command") {
      this.log.push(kind);
      if (kind === "clear") {
        this.faultWord = 0;
        this.faultFlag = false;
      } else if (kind === "enable") {
        this.enabled = true;
      } else if (kind === "disable") {
        this.enabled = false;
      } else if (kind === "zero") {
        this.position = 0.0;
      }
      return [this.feedback()];
    }
    const q = (data[0]! << 8) | data[1]!;
    const kp = (((data[3]! & 0xf) << 8) | data[4]!) * p.KP_MAX / 4095;
    this.lastKp = kp;
    this.log.push(["cmd", Math.round(kp * 10) / 10]);
    const position = p.uintToFloat(q, this.params.pMin, this.params.pMax, 16);
    const tau = p.uintToFloat(((data[6]! & 0xf) << 8) | data[7]!, this.params.tMin, this.params.tMax, 12);
    this.frames.push([position, kp, tau]);
    if (this.enabled && this.track && kp > 0) this.position = position;
    return [this.feedback()];
  }
}

/** Replies of the simulated motors to one host frame (motor replies are ignored). */
export function respond(motors: Map<number, SimMotor>, canId: number, data: Uint8Array, extended = false):
    Array<[number, Uint8Array]> {
  if (extended || canId === p.HOST_CAN_ID) return [];
  const key = canId & 0x7f;
  const m = motors.get(key);
  if (m === undefined) return [];
  const replies = m.handle(canId, data);
  if (m.motorId !== key && !motors.has(m.motorId)) {   // an id write moved it
    motors.delete(key);
    motors.set(m.motorId, m);
  }
  return replies;
}

/** MockBackend whose responder routes each frame to the SimMotor it addresses. */
export class SimBus extends MockBackend {
  motors: Map<number, SimMotor>;

  constructor(motors: Iterable<SimMotor>) {
    super();
    this.motors = new Map([...motors].map((m) => [m.motorId, m]));
    this.responder = (canId, data, extended) => respond(this.motors, canId, data, extended);
  }
}

/** What the motor's CAN watchdog does once commands stop: reset mode (torque off), no fault. */
export function watchdogTimeout(motor: SimMotor): void {
  motor.enabled = false;
}

/**
 * A SimMotor answering like the motor of `joint` at `position` (motor rad). Firmware with
 * parameter access unless `paramsSupported: false`.
 */
export function forJoint(joint: JointConfig, hostId: number, position = 0.0, options: SimMotorOptions = {}): SimMotor {
  return new SimMotor(joint.motorId, joint.model, { paramsSupported: true, ...options, position, hostId });
}
