/**
 * Simulated Damiao MIT motors behind MockBackend, for tests and "@makermods/metal-arm/testing".
 *
 * Each SimMotor keeps RAM registers and a flash copy, a status nibble, two temperatures and a
 * position that follows MIT commands while enabled (kp > 0). Options model the field behaviour the
 * driver must survive: `late` holds every reply until the next frame appears on the bus (the
 * gripper), `duplicate` releases a held reply twice, `drop` ignores that many requests, `fault`
 * reports a status code, and `idsApplyAtOnce` chooses whether an ESC_ID/MST_ID write takes effect
 * immediately or only after save + powerCycle(). `log` records what it received ("status_query",
 * "clear", "enable", "disable", "zero", ["cmd", kp], ["read", reg], ["write", reg], "save",
 * "dropped"), `frames` every MIT command as [position, kp, tau].
 */

import { type JointConfig, MockBackend } from "@makermods/arm-core";
import { round1, roundHalfEven } from "./format.js";
import * as p from "./protocol.js";
import type { Frame, MotorModel } from "./protocol.js";

export interface FeedbackBytesOptions {
  pos?: number;
  vel?: number;
  tau?: number;
  /** °C (MOS), rounded to whole degrees. */
  temp?: number;
  status?: number;
  /** °C, defaults to `temp`. */
  rotorTemp?: number | null;
  /** A model name or its scaling. */
  model?: string | MotorModel;
}

/** A Damiao state frame payload (temperatures in whole °C). */
export function feedbackBytes(motorId: number, options: FeedbackBytesOptions = {}): Uint8Array {
  const { pos = 0.0, vel = 0.0, tau = 0.0, temp = 30.0, status = 0, rotorTemp = null, model = "jlo" } = options;
  const params = typeof model === "string" ? p.MODELS[model]! : model;
  return p.encodeState(motorId, status, pos, vel, tau, roundHalfEven(temp),
    roundHalfEven(rotorTemp === null ? temp : rotorTemp), params);
}

/** A register reply payload: the request header echoed, then `raw`. */
export function registerReply(motorId: number, op: number, rid: number, raw: ArrayLike<number>): Uint8Array {
  return Uint8Array.from([motorId & 0xff, motorId >> 8, op, rid, ...Array.from(raw)]);
}

/** What `faulted = true` reports. */
export const FAULT_CODE = 0xc;
/** How the SDK describes FAULT_CODE. */
export const FAULT_TEXT = "rotor overtemperature";

// What the protocol can show:
/** Position range is ±PMAX (6.28 rad): a whole-turn offset is unrepresentable. */
export const FULL_TURN_SUPPORTED = false;
/** Damiao feedback has no warning flag. */
export const WARNING_SUPPORTED = false;

/** Kind of a host->motor frame. */
export type FrameKind =
  | "command" | "enable" | "disable" | "zero" | "clear" | "status_query" | "param_read" | "param_write" | "param_save"
  | "other";

/** [kind, motor id] of a host->motor frame. */
export function classify(canId: number, data: Uint8Array): [FrameKind, number | null] {
  data = Uint8Array.from(data);
  if (canId === p.PARAM_CAN_ID) {
    if (data.length < 4) return ["other", null];
    const target = data[0]! | (data[1]! << 8);
    const kinds: Record<number, FrameKind> = {
      [p.OP_REFRESH]: "status_query", [p.OP_READ]: "param_read", [p.OP_WRITE]: "param_write", [p.OP_SAVE]: "param_save",
    };
    return [kinds[data[2]!] ?? "other", target];
  }
  const motorId = canId & 0xff;
  if (![0, 1, 2, 3].includes(canId >> 8)) return ["other", null];
  if (p.startsWithCommandPrefix(data)) {
    const names: Record<number, FrameKind> = {
      [p.CMD_ENABLE]: "enable", [p.CMD_DISABLE]: "disable", [p.CMD_SET_ZERO]: "zero", [p.CMD_CLEAR_ERROR]: "clear",
    };
    return [names[data[7]!] ?? "other", motorId];
  }
  return ["command", motorId];
}

/** One entry of SimMotor.log. */
export type SimLogEntry = string | [string, number];

export interface SimMotorOptions {
  position?: number;
  velocity?: number;
  torque?: number;
  /** MOS °C. */
  temperature?: number;
  /** Rotor °C, defaults to MOS. */
  rotorTemperature?: number | null;
  enabled?: boolean;
  online?: boolean;
  /** An enabled motor jumps to the commanded position (kp > 0). */
  track?: boolean;
  faulted?: boolean;
  /** Accepted for the shared surface; Damiao reports none. */
  warning?: boolean;
  /** Status code 0x8..0xE (overrides `faulted`). */
  fault?: number | null;
  /** Hold each reply until the next frame on the bus. */
  late?: boolean;
  /** Release a held reply twice. */
  duplicate?: boolean;
  /** Ignore this many upcoming requests. */
  drop?: number;
  /** ESC_ID/MST_ID writes act before a power cycle. */
  idsApplyAtOnce?: boolean;
  /** Echo the 0xAA save request. */
  saveReply?: boolean;
  /** Id named in the ESC_ID write echo. */
  escEcho?: "old" | "new" | "none";
  /** Register index -> value, over the defaults. */
  registers?: Record<number, number>;
  /** Accepted for the shared surface; registers always answer. */
  paramsSupported?: boolean;
}

/** One simulated Damiao motor. */
export class SimMotor {
  motorId: number;
  model: string;
  position: number;
  velocity: number;
  torque: number;
  /** MOS °C. */
  temperature: number;
  /** Defaults to MOS. */
  rotorTemperature: number | null;
  enabled: boolean;
  online: boolean;
  track: boolean;
  warning: boolean;
  paramsSupported: boolean;
  /** Status code, 0 when healthy. */
  fault: number;
  late: boolean;
  duplicate: boolean;
  drop: number;
  idsApplyAtOnce: boolean;
  saveReply: boolean;
  escEcho: "old" | "new" | "none";
  log: SimLogEntry[] = [];
  /** Every MIT frame: [position, kp, tau]. */
  frames: Array<[number, number, number]> = [];
  /** Every MIT frame: [position, velocity, kp, kd, tau]. */
  commands: Array<[number, number, number, number, number]> = [];
  lastKp = 0.0;
  held: Frame[] = [];
  registers: Record<number, number>;
  flash: Record<number, number>;
  sendId: number;
  replyId: number;

  constructor(motorId: number, model = "jlo", options: SimMotorOptions = {}) {
    const o = options;
    this.motorId = motorId;
    this.model = model;
    this.position = o.position ?? 0.0;
    this.velocity = o.velocity ?? 0.0;
    this.torque = o.torque ?? 0.0;
    this.temperature = o.temperature ?? 30.0;
    this.rotorTemperature = o.rotorTemperature ?? null;
    this.enabled = o.enabled ?? false;
    this.online = o.online ?? true;
    this.track = o.track ?? true;
    this.warning = o.warning ?? false;
    this.paramsSupported = o.paramsSupported ?? true;
    this.fault = o.fault === undefined || o.fault === null ? (o.faulted ? FAULT_CODE : 0) : o.fault;
    this.late = o.late ?? false;
    this.duplicate = o.duplicate ?? false;
    this.drop = o.drop ?? 0;
    this.idsApplyAtOnce = o.idsApplyAtOnce ?? true;
    this.saveReply = o.saveReply ?? false;
    this.escEcho = o.escEcho ?? "old";
    const params = p.MODELS[model]!;
    const defaults: Record<number, number> = {
      [p.Reg.MST_ID]: p.DEFAULT_MASTER_OFFSET + motorId, [p.Reg.ESC_ID]: motorId, [p.Reg.TIMEOUT]: 0,
      [p.Reg.CTRL_MODE]: p.MODE_MIT, [p.Reg.PMAX]: params.pMax, [p.Reg.VMAX]: params.vMax, [p.Reg.TMAX]: params.tMax,
      14: 0x1234,
    };
    this.registers = { ...defaults, ...(o.registers ?? {}) };
    this.flash = { ...this.registers };
    this.sendId = Math.trunc(this.registers[p.Reg.ESC_ID]!);
    this.replyId = Math.trunc(this.registers[p.Reg.MST_ID]!);
  }

  get faulted(): boolean {
    return Boolean(this.fault);
  }

  set faulted(value: boolean) {
    this.fault = value ? FAULT_CODE : 0;
  }

  get params(): MotorModel {
    return p.MODELS[this.model]!;
  }

  get status(): number {
    if (this.fault) return this.fault;
    return this.enabled ? p.STATUS_ENABLED : p.STATUS_DISABLED;
  }

  get watchdogTicks(): number {
    return Math.trunc(this.registers[p.Reg.TIMEOUT]!);
  }

  /** [reply id, state frame] of this motor now. */
  feedback(): Frame {
    const rotor = this.rotorTemperature === null ? this.temperature : this.rotorTemperature;
    return [this.replyId, feedbackBytes(this.sendId, {
      pos: this.position, vel: this.velocity, tau: this.torque, temp: this.temperature, status: this.status,
      rotorTemp: rotor, model: this.params,
    })];
  }

  /** Reload flash: unsaved register writes are lost, ids from flash take effect. */
  powerCycle(): void {
    this.registers = { ...this.flash };
    this.sendId = Math.trunc(this.registers[p.Reg.ESC_ID]!);
    this.replyId = Math.trunc(this.registers[p.Reg.MST_ID]!);
    this.enabled = false;
    this.fault = 0;
    this.held = [];
  }

  addressed(canId: number, data: Uint8Array): boolean {
    if (canId === p.PARAM_CAN_ID) return data.length >= 2 && (data[0]! | (data[1]! << 8)) === this.sendId;
    return [0, 1, 2, 3].includes(canId >> 8) && (canId & 0xff) === this.sendId;
  }

  /** Replies to one frame addressed to this motor. */
  handle(canId: number, data: Uint8Array): Frame[] {
    if (!this.online) return [];
    if (this.drop) {
      this.drop -= 1;
      this.log.push("dropped");
      return [];
    }
    data = Uint8Array.from(data);
    if (canId === p.PARAM_CAN_ID) return this.paramFrame(data);
    if (p.startsWithCommandPrefix(data)) return this.command(data[7]!);
    return this.mit(data);
  }

  protected command(command: number): Frame[] {
    if (command === p.CMD_ENABLE) {
      this.log.push("enable");
      this.enabled = !this.fault;
    } else if (command === p.CMD_DISABLE) {
      this.log.push("disable");
      this.enabled = false;
    } else if (command === p.CMD_SET_ZERO) {
      this.log.push("zero");
      this.position = 0.0;
    } else if (command === p.CMD_CLEAR_ERROR) {
      this.log.push("clear");
      this.fault = 0;
    }
    return [this.feedback()];
  }

  protected mit(data: Uint8Array): Frame[] {
    const params = this.params;
    const q = (data[0]! << 8) | data[1]!;
    const v = (data[2]! << 4) | (data[3]! >> 4);
    const kp = ((((data[3]! & 0xf) << 8) | data[4]!) * p.KP_MAX) / 4095;
    const kd = (((data[5]! << 4) | (data[6]! >> 4)) * p.KD_MAX) / 4095;
    const t = ((data[6]! & 0xf) << 8) | data[7]!;
    const position = p.uintToFloat(q, -params.pMax, params.pMax, 16);
    const velocity = p.uintToFloat(v, -params.vMax, params.vMax, 12);
    const tau = p.uintToFloat(t, -params.tMax, params.tMax, 12);
    this.lastKp = kp;
    this.log.push(["cmd", round1(kp)]);
    this.frames.push([position, kp, tau]);
    this.commands.push([position, velocity, kp, kd, tau]);
    if (this.enabled && this.track && kp > 0) this.position = position;
    return [this.feedback()];
  }

  /** Replies to a frame on 0x7FF (refresh and register access). */
  paramFrame(data: Uint8Array): Frame[] {
    const op = data[2]!;
    const rid = data[3]!;
    if (op === p.OP_REFRESH) {
      this.log.push("status_query");
      return [this.feedback()];
    }
    if (op === p.OP_READ) {
      this.log.push(["read", rid]);
      const raw = p.packValue(this.registers[rid] ?? 0, p.registerDtype(rid));
      return [[this.replyId, registerReply(this.sendId, op, rid, raw)]];
    }
    if (op === p.OP_WRITE) {
      this.log.push(["write", rid]);
      const value = data.slice(4, 8);
      this.registers[rid] = p.unpackValue(value, p.registerDtype(rid));
      let reply: Frame[] = [[this.replyId, registerReply(this.sendId, op, rid, value)]];
      if (this.idsApplyAtOnce && rid === p.Reg.MST_ID) {
        this.replyId = Math.trunc(this.registers[rid]!);
        reply = [[this.replyId, reply[0]![1]]];
      }
      if (this.idsApplyAtOnce && rid === p.Reg.ESC_ID) {
        this.sendId = Math.trunc(this.registers[rid]!);
        if (this.escEcho === "none") return [];
        if (this.escEcho === "new") reply = [[this.replyId, registerReply(this.sendId, op, rid, value)]];
      }
      return reply;
    }
    if (op === p.OP_SAVE) {
      this.log.push("save");
      if (!this.enabled) this.flash = { ...this.registers };
      return this.saveReply ? [[this.replyId, Uint8Array.from(data)]] : [];
    }
    return [];
  }
}

/**
 * Replies of the simulated motors to one frame on the bus. Any frame releases held (late) replies
 * first; the motor that owns the frame's send id then answers (or holds its answer).
 */
export function respond(motors: ReadonlyMap<number, SimMotor>, canId: number, data: Uint8Array, extended = false): Frame[] {
  if (extended) return [];
  const out: Frame[] = [];
  for (const m of motors.values()) {
    if (m.held.length) {
      for (const frame of m.held) out.push(...(m.duplicate ? [frame, frame] : [frame]));
      m.held = [];
    }
  }
  for (const m of motors.values()) {
    if (m.addressed(canId, data)) {
      const replies = m.handle(canId, data);
      if (m.late) m.held.push(...replies);
      else out.push(...replies);
    }
  }
  return out;
}

/** MockBackend whose responder routes each frame to the SimMotor that owns its send id. */
export class SimBus extends MockBackend {
  readonly motors: Map<number, SimMotor>;

  constructor(motors: Iterable<SimMotor>) {
    super();
    this.motors = new Map([...motors].map((m) => [m.motorId, m]));
    // Every frame reaches the motors, whatever its identifier format, as on the Python SimBus.
    this.responder = (canId, data) => respond(this.motors, canId, data);
  }
}

/** What the motor's watchdog does once commands stop: torque off, status "communication lost". */
export function watchdogTimeout(motor: SimMotor): void {
  motor.enabled = false;
  motor.fault = p.STATUS_COMM_LOST;
}

/**
 * A SimMotor answering like the motor of `joint` at `position` (motor rad), replying on the joint's
 * reply id (default `hostId` + motor id).
 */
export function forJoint(joint: JointConfig, hostId: number, position = 0.0, options: SimMotorOptions = {}): SimMotor {
  const { registers = {}, ...rest } = options;
  return new SimMotor(joint.motorId, joint.model, {
    ...rest, position, registers: { [p.Reg.MST_ID]: joint.replyId || hostId + joint.motorId, ...registers },
  });
}
