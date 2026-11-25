/**
 * RobStride MIT wire protocol (standard 11-bit CAN, 1 Mbps): pure encode/decode, zero IO.
 *
 * Every frame is a classic standard-ID frame addressed to the motor ID (1..127). Replies arrive on
 * the host ID (0xFD) with the motor ID in payload byte 0; typed parameter replies arrive on
 * 0x300|id (read) or 0x400|id (write).
 *
 * Sources: RobStride RS00/RS02 manuals 260713 section 6 (command 7, modify motor CAN ID, as in the RS04
 * manual); LeRobot RobStride driver
 * (makermods-robotics/lerobot@5ce2fe41a); MakerModsLab maker_fault_diagnostics.py and
 * maker_gripper_hold.py.
 */

import { ValueError } from "@makermods/arm-core";

/** A frame to send: [CAN id, payload]. */
export type Frame = [number, Uint8Array];

export const HOST_CAN_ID = 0xfd;
export const KP_MIN = 0.0;
export const KP_MAX = 500.0;
export const KD_MIN = 0.0;
export const KD_MAX = 5.0;

/**
 * Per-model MIT mapping ranges. The frame format is identical across models; picking the wrong
 * table scales torque/velocity readings by the wrong factor.
 */
export class MotorParams {
  /** rad. */
  readonly pMax: number;
  /** rad/s. */
  readonly vMax: number;
  /** N·m. */
  readonly tMax: number;

  constructor(pMax: number, vMax: number, tMax: number) {
    this.pMax = pMax;
    this.vMax = vMax;
    this.tMax = tMax;
    Object.freeze(this);
  }

  get pMin(): number {
    return -this.pMax;
  }

  get vMin(): number {
    return -this.vMax;
  }

  get tMin(): number {
    return -this.tMax;
  }
}

// LeRobot type O0 = RS00, O1 = RS02. Not O2 (±33 rad/s, ±20 Nm), which is a different tier.
// RS02 firmware <=0.2.2.11 maps position to ±12.5 rad; check firmware during bring-up.
export const RS00 = new MotorParams(12.57, 33.0, 14.0);
export const RS02 = new MotorParams(12.57, 44.0, 17.0);
export const MOTOR_PARAMS: Readonly<Record<string, MotorParams>> = Object.freeze({ RS00, RS02 });

/** Typed parameter registers. */
export const ParamIndex = Object.freeze({
  /** u8: 0 = MIT control mode. */
  RUN_MODE: 0x7005,
  /** f32, rad. */
  MECH_POS: 0x7019,
  /** u32, 20 ticks per ms (10000 = 500 ms); 0 disables the watchdog. */
  CAN_TIMEOUT: 0x7028,
});

export const CAN_TIMEOUT_TICKS_PER_MS = 20;

// Command opcodes (payload byte 7 after six 0xFF bytes).
/** Byte 6 = 0xFF clears faults and returns feedback; 0x00 reads faults only. */
/** byte 6 = the new motor ID; answered with the MCU identifier on the motor ID. */
export const OP_SET_ID = 0xfa;
export const OP_CLEAR_FAULT = 0xfb;
export const OP_ENABLE = 0xfc;
export const OP_DISABLE = 0xfd;
export const OP_SET_ZERO = 0xfe;

/** MIT fault word bits (LE uint32 in the fault-status reply), source: MakerModsLab. */
export const FAULT_BITS: Readonly<Record<number, string>> = Object.freeze({
  0: "motor overtemperature",
  1: "driver chip fault",
  2: "undervoltage",
  3: "overvoltage",
  4: "phase B overcurrent",
  5: "phase C overcurrent",
  7: "encoder uncalibrated",
  8: "hardware identification fault",
  9: "position initialization fault",
  14: "stall/overload",
  16: "phase A overcurrent",
});

/** Disabled. */
export const MODE_RESET = 0;
export const MODE_CALIBRATION = 1;
/** Enabled. */
export const MODE_RUN = 2;

/** Words for a fault word ("no fault", "unknown" for null). */
export function faultText(word: number | null): string {
  if (word === null) return "unknown";
  if (!word) return "no fault";
  const parts: string[] = [];
  for (let bit = 0; bit < 32; bit++) {
    if (Math.floor(word / 2 ** bit) % 2 === 1) parts.push(FAULT_BITS[bit] ?? `bit${bit}`);
  }
  return parts.join(", ");
}

function checkId(motorId: number): number {
  if (!(motorId >= 1 && motorId <= 127)) throw new ValueError(`MIT motor ID must be in [1, 127], got ${motorId}`);
  return motorId;
}

/** Six 0xFF bytes, the argument, the opcode. */
export function command(motorId: number, opcode: number, argument = 0xff): Frame {
  return [checkId(motorId), Uint8Array.from([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, argument & 0xff, opcode])];
}

export function encodeEnable(motorId: number): Frame {
  return command(motorId, OP_ENABLE);
}

export function encodeDisable(motorId: number): Frame {
  return command(motorId, OP_DISABLE);
}

/** Clear faults and request feedback. Only send when no fault evidence must be kept. */
export function encodeClearFault(motorId: number): Frame {
  return command(motorId, OP_CLEAR_FAULT);
}

/** Read the fault word without clearing it or changing torque state. */
export function encodeFaultQuery(motorId: number): Frame {
  return command(motorId, OP_CLEAR_FAULT, 0x00);
}

export function encodeSetZero(motorId: number): Frame {
  return command(motorId, OP_SET_ZERO);
}

/** Change the motor answering on `motorId` to `newId` (manual section 6, command 7). */
export function encodeSetId(motorId: number, newId: number): Frame {
  return command(motorId, OP_SET_ID, checkId(newId));
}

/** Python's round(): nearest integer, ties to even. */
export function roundHalfEven(x: number): number {
  const f = Math.floor(x);
  const d = x - f;
  if (d > 0.5) return f + 1;
  if (d < 0.5) return f;
  return f % 2 === 0 ? f : f + 1;
}

export function floatToUint(value: number, lo: number, hi: number, bits: number): number {
  if (!Number.isFinite(value)) throw new ValueError("MIT commands must be finite");
  return roundHalfEven((Math.min(hi, Math.max(lo, value)) - lo) * ((1 << bits) - 1) / (hi - lo));
}

export function uintToFloat(value: number, lo: number, hi: number, bits: number): number {
  return value * (hi - lo) / ((1 << bits) - 1) + lo;
}

/** MIT impedance command: tau_out = kp*(pos - q) + kd*(vel - dq) + tau. */
export function encodeControl(motorId: number, pos: number, vel: number, kp: number, kd: number, tau: number,
                              params: MotorParams = RS00): Frame {
  const q = floatToUint(pos, params.pMin, params.pMax, 16);
  const v = floatToUint(vel, params.vMin, params.vMax, 12);
  const k = floatToUint(kp, KP_MIN, KP_MAX, 12);
  const d = floatToUint(kd, KD_MIN, KD_MAX, 12);
  const t = floatToUint(tau, params.tMin, params.tMax, 12);
  const payload = Uint8Array.from([q >> 8, q & 0xff, v >> 4,
    ((v & 0xf) << 4) | (k >> 8), k & 0xff, d >> 4,
    ((d & 0xf) << 4) | (t >> 8), t & 0xff]);
  if (payload.subarray(0, 6).every((b) => b === 0xff)) {
    throw new ValueError("MIT control payload collides with the reserved command prefix");
  }
  return [checkId(motorId), payload];
}

function checkIndex(index: number): void {
  if (!(Number.isInteger(index) && index >= 0 && index <= 0xffff)) throw new ValueError(`parameter index out of range: ${index}`);
}

export function encodeReadParam(motorId: number, index: number): Frame {
  checkId(motorId);
  checkIndex(index);
  const data = new Uint8Array(8);
  new DataView(data.buffer).setUint16(0, index, true);
  return [0x300 | checkId(motorId), data];
}

/** Parameter value types: "f" float32, "u8", "u16", "u32". */
export type ParamType = "f" | "u8" | "u16" | "u32";

const PARAM_TYPES: readonly string[] = ["f", "u8", "u16", "u32"];

function paramType(dtype: string): ParamType {
  if (!PARAM_TYPES.includes(dtype)) throw new ValueError(`unknown parameter type '${dtype}'`);
  return dtype as ParamType;
}

/** Python's int(): truncates toward zero; refuses NaN and infinity. */
function pyInt(value: number): number {
  if (!Number.isFinite(value)) throw new ValueError(`cannot convert ${value} to an integer`);
  return Math.trunc(value);
}

function packValue(view: DataView, offset: number, value: number, dtype: ParamType): void {
  if (dtype === "f") {
    const f = Math.fround(value);
    if (!Number.isFinite(f) && Number.isFinite(value)) throw new ValueError("float too large to pack with f format");
    view.setFloat32(offset, value, true);
    return;
  }
  const n = pyInt(value);
  const max = { u8: 0xff, u16: 0xffff, u32: 0xffffffff }[dtype];
  if (n < 0 || n > max) throw new ValueError(`${dtype} parameter value out of range: ${n}`);
  if (dtype === "u8") view.setUint8(offset, n);
  else if (dtype === "u16") view.setUint16(offset, n, true);
  else view.setUint32(offset, n, true);
}

export function encodeWriteParam(motorId: number, index: number, value: number, dtype: string = "f"): Frame {
  checkId(motorId);
  const type = paramType(dtype);
  checkIndex(index);
  const data = new Uint8Array(8);
  const view = new DataView(data.buffer);
  view.setUint16(0, index, true);
  packValue(view, 4, value, type);
  return [0x400 | checkId(motorId), data];
}

/** A motion sample. */
export interface MotorFeedback {
  readonly kind: "feedback";
  readonly motorId: number;
  /** rad, motor coordinates. */
  readonly position: number;
  /** rad/s. */
  readonly velocity: number;
  /** N·m. */
  readonly torque: number;
  /** °C (winding). */
  readonly temperature: number;
  /** 0 reset/disabled, 1 calibration, 2 run/enabled. */
  readonly mode: number;
  readonly faultFlag: boolean;
  readonly warningFlag: boolean;
}

/** A fault-status reply. */
export interface FaultStatus {
  readonly kind: "status";
  readonly motorId: number;
  /** LE uint32 fault bits, see FAULT_BITS. */
  readonly word: number;
  readonly hasFault: boolean;
  readonly text: string;
}

/** A typed parameter reply. */
export class ParamReply {
  readonly kind = "param" as const;
  readonly motorId: number;
  readonly index: number;
  /** data[4:8]. */
  readonly raw: Uint8Array;
  /** True for a write acknowledgement (0x400|id), never a read result. */
  readonly write: boolean;

  constructor(motorId: number, index: number, raw: Uint8Array, write = false) {
    this.motorId = motorId;
    this.index = index;
    this.raw = raw;
    this.write = write;
  }

  value(dtype: string = "f"): number {
    const type = paramType(dtype);
    const view = new DataView(this.raw.buffer, this.raw.byteOffset, 4);
    if (type === "f") return view.getFloat32(0, true);
    if (type === "u8") return view.getUint8(0);
    if (type === "u16") return view.getUint16(0, true);
    return view.getUint32(0, true);
  }
}

export function faultStatus(motorId: number, word: number): FaultStatus {
  return { kind: "status", motorId, word, hasFault: word !== 0, text: faultText(word) };
}

/** Motor ID of a reply without decoding it (parameter replies carry it in the CAN ID). */
export function replyMotorId(canId: number, data: Uint8Array, extended = false): number | null {
  if (extended || !(canId >= 0 && canId <= 0x7ff)) return null;
  if ((canId & 0x700) === 0x300 || (canId & 0x700) === 0x400) return canId & 0xff;
  return data.length ? data[0]! : null;
}

export interface ParseOptions {
  params?: MotorParams;
  hostId?: number;
  /** The frame has a 29-bit identifier. */
  extended?: boolean;
}

/**
 * Decode one received frame; anything that is not an MIT reply returns null.
 *
 * Fault-status replies are five bytes on old firmware and zero-padded to eight on newer firmware.
 * A motion sample with the same padding signature (temperature 0.0 °C, mode 0) is conservatively
 * treated as status so a late fault reply can never become an encoder sample.
 */
export function parseFrame(canId: number, data: Uint8Array, options: ParseOptions = {}):
    MotorFeedback | FaultStatus | ParamReply | null {
  const params = options.params ?? RS00;
  const hostId = options.hostId ?? HOST_CAN_ID;
  if (options.extended || !(canId >= 0 && canId <= 0x7ff)) return null;
  if ((canId & 0x700) === 0x300 || (canId & 0x700) === 0x400) {
    // Parameter replies: the register index low byte sits in data[0], which can alias a motor ID,
    // so these are filtered by arbitration ID before any feedback decoding.
    if (data.length === 8) {
      return new ParamReply(canId & 0xff, data[0]! | (data[1]! << 8), Uint8Array.from(data.subarray(4, 8)),
        (canId & 0x700) === 0x400);
    }
    return null;
  }
  if (canId !== hostId || (data.length !== 5 && data.length !== 8)) return null;
  if (data.length === 5 || (data[5] === 0 && data[6] === 0 && data[7] === 0)) {
    const word = new DataView(data.buffer, data.byteOffset + 1, 4).getUint32(0, true);
    return faultStatus(data[0]!, word);
  }
  const q = (data[1]! << 8) | data[2]!;
  const v = (data[3]! << 4) | (data[4]! >> 4);
  const t = ((data[4]! & 0xf) << 8) | data[5]!;
  // Byte 6 carries mode (bits 7-6) and fault/warning flags (bits 5-4) above the 12-bit
  // temperature. Old firmware leaves those bits zero for any real temperature, so masking is
  // correct for both layouts.
  const status = data[6]!;
  return {
    kind: "feedback",
    motorId: data[0]!,
    position: uintToFloat(q, params.pMin, params.pMax, 16),
    velocity: uintToFloat(v, params.vMin, params.vMax, 12),
    torque: uintToFloat(t, params.tMin, params.tMax, 12),
    temperature: (((status & 0x0f) << 8) | data[7]!) / 10.0,
    mode: status >> 6,
    faultFlag: Boolean(status & 0x20),
    warningFlag: Boolean(status & 0x10),
  };
}
