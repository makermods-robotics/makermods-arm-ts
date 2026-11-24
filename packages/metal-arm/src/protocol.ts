/**
 * Damiao MIT wire protocol (classic CAN, standard 11-bit ids, 1 Mbit/s): pure encode/decode, no I/O.
 *
 * Frames to a motor go to its send id (ESC_ID, 1..15 here). Its replies come back on its master id
 * (MST_ID, conventionally 0x10 + send id). Register access and the read-only state refresh go to the
 * shared id 0x7FF with the target send id in bytes 0-1.
 *
 * Sources: Damiao DM_CAN reference driver and motor manual (register map, 0x33/0x55/0xAA, 0xCC
 * refresh, status nibble); the vendor's Metal C++ SDK (status codes, temperatures in bytes 6/7).
 * The MIT packing matches the LeRobot Damiao encoder byte for byte (test/protocol.test.ts).
 */

import { ValueError } from "@makermods/arm-core";

/** (CAN id, payload) of one frame. */
export type Frame = [number, Uint8Array];

/** Register read/write/save and state refresh. */
export const PARAM_CAN_ID = 0x7ff;
/** The state frame carries only the low nibble of the id. */
export const ID_MIN = 1;
export const ID_MAX = 15;
/** Master (reply) id = 0x10 + send id on every Metal motor. */
export const DEFAULT_MASTER_OFFSET = 0x10;
export const KP_MIN = 0.0;
export const KP_MAX = 500.0;
export const KD_MIN = 0.0;
export const KD_MAX = 5.0;

// Command bytes (byte 7 after seven 0xFF bytes, sent to the send id + the mode offset).
export const CMD_ENABLE = 0xfc;
export const CMD_DISABLE = 0xfd;
export const CMD_SET_ZERO = 0xfe;
export const CMD_CLEAR_ERROR = 0xfb;
/** Seven 0xFF bytes: the first seven bytes of every command frame. */
export const COMMAND_PREFIX: Uint8Array = Uint8Array.from([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]);

// Register opcodes (byte 2 of a frame to 0x7FF) and the refresh request.
export const OP_READ = 0x33;
export const OP_WRITE = 0x55;
export const OP_SAVE = 0xaa;
export const OP_REFRESH = 0xcc;
export const REGISTER_OPS: readonly number[] = [OP_READ, OP_WRITE, OP_SAVE];

// Control modes (CTRL_MODE register). Commands in mode m go to send id + MODE_OFFSETS[m].
export const MODE_MIT = 1;
export const MODE_POS_VEL = 2;
export const MODE_VEL = 3;
export const MODE_FORCE_POS = 4;
export const MODE_OFFSETS: Readonly<Record<number, number>> = {
  [MODE_MIT]: 0x000, [MODE_POS_VEL]: 0x100, [MODE_VEL]: 0x200, [MODE_FORCE_POS]: 0x300,
};

// Status nibble (byte 0 high nibble of every state frame).
export const STATUS_DISABLED = 0x0;
export const STATUS_ENABLED = 0x1;
export const STATUS_COMM_LOST = 0xd;
export const STATUS_NAMES: Readonly<Record<number, string>> = {
  0x0: "disabled",
  0x1: "enabled",
  0x8: "overvoltage",
  0x9: "undervoltage",
  0xa: "overcurrent",
  0xb: "MOS overtemperature",
  0xc: "rotor overtemperature",
  0xd: "communication lost",
  0xe: "overload",
};

/** TIMEOUT register counts 50 us ticks; 0 disables the watchdog. */
export const TIMEOUT_TICKS_PER_MS = 20;

const hasStatus = (status: number): boolean => Object.hasOwn(STATUS_NAMES, status);

/** Any status but disabled / enabled is a fault. */
export function isFaultStatus(status: number): boolean {
  return status !== STATUS_DISABLED && status !== STATUS_ENABLED;
}

/** "enabled", "disabled", "fault 0xC (rotor overtemperature)", or "status unknown" for null. */
export function statusText(status: number | null | undefined): string {
  if (status === null || status === undefined) return "status unknown";
  if (!isFaultStatus(status)) return STATUS_NAMES[status]!;
  return `fault 0x${status.toString(16).toUpperCase()} (${hasStatus(status) ? STATUS_NAMES[status] : "unknown fault"})`;
}

/** MIT mapping ranges, which must equal the motor's PMAX/VMAX/TMAX registers (21/22/23). */
export interface MotorModel {
  /** rad */
  readonly pMax: number;
  /** rad/s */
  readonly vMax: number;
  /** N·m */
  readonly tMax: number;
}

const model = (pMax: number, vMax: number, tMax: number): MotorModel => Object.freeze({ pMax, vMax, tMax });

/** Metal Arm motors, source: LeRobot damiao tables.py METAL_JLO / METAL_J2 / METAL_JHI. */
export const MODELS: Readonly<Record<string, MotorModel>> = Object.freeze({
  jlo: model(6.28, 10.0, 30.0), // shoulder_pan, elbow_flex, wrist_flex
  j2: model(6.28, 10.0, 120.0), // shoulder_lift
  jhi: model(6.28, 30.0, 20.0), // wrist_yaw, wrist_roll, gripper
});

/** Register indices the SDK uses. */
export const Reg = Object.freeze({
  /** u32 master (reply) id */
  MST_ID: 7,
  /** u32 send id */
  ESC_ID: 8,
  /** u32 communication watchdog, 50 us ticks; 0 = off */
  TIMEOUT: 9,
  /** u32 1 MIT, 2 pos-vel, 3 vel, 4 force-position */
  CTRL_MODE: 10,
  /** f32 rad */
  PMAX: 21,
  /** f32 rad/s */
  VMAX: 22,
  /** f32 N·m */
  TMAX: 23,
});

/** Register dtypes: "f" (float32) or "u32". */
export type RegisterDtype = "f" | "u32";

/** One register: index, dtype ("f" float32 or "u32") and what it holds. */
export interface RegisterSpec {
  readonly index: number;
  readonly dtype: string;
  readonly doc: string;
}

const spec = (index: number, dtype: RegisterDtype, doc: string): RegisterSpec => Object.freeze({ index, dtype, doc });

/** Damiao register map (DM_CAN DM_variable). u32 registers: 7-10, 13-16, 35-36; the rest float32. */
export const REGISTERS: Readonly<Record<string, RegisterSpec>> = Object.freeze({
  uv_value: spec(0, "f", "undervoltage threshold (V)"),
  kt_value: spec(1, "f", "torque constant"),
  ot_value: spec(2, "f", "overtemperature threshold (°C)"),
  oc_value: spec(3, "f", "overcurrent threshold"),
  acc: spec(4, "f", "acceleration"),
  dec: spec(5, "f", "deceleration"),
  max_spd: spec(6, "f", "maximum speed"),
  mst_id: spec(7, "u32", "master (reply) CAN id"),
  esc_id: spec(8, "u32", "send CAN id"),
  timeout: spec(9, "u32", "communication watchdog in 50 us ticks; 0 = off"),
  ctrl_mode: spec(10, "u32", "control mode: 1 MIT, 2 pos-vel, 3 vel, 4 force-position"),
  damp: spec(11, "f", "damping"),
  inertia: spec(12, "f", "inertia"),
  hw_ver: spec(13, "u32", "hardware version"),
  sw_ver: spec(14, "u32", "firmware version"),
  sn: spec(15, "u32", "serial number"),
  npp: spec(16, "u32", "motor pole pairs"),
  rs: spec(17, "f", "phase resistance"),
  ls: spec(18, "f", "phase inductance"),
  flux: spec(19, "f", "flux linkage"),
  gr: spec(20, "f", "gear ratio"),
  pmax: spec(21, "f", "MIT position mapping range (rad)"),
  vmax: spec(22, "f", "MIT velocity mapping range (rad/s)"),
  tmax: spec(23, "f", "MIT torque mapping range (N·m)"),
  i_bw: spec(24, "f", "current loop bandwidth"),
  kp_asr: spec(25, "f", "speed loop Kp"),
  ki_asr: spec(26, "f", "speed loop Ki"),
  kp_apr: spec(27, "f", "position loop Kp"),
  ki_apr: spec(28, "f", "position loop Ki"),
  ov_value: spec(29, "f", "overvoltage threshold (V)"),
  gref: spec(30, "f", "gear efficiency"),
  deta: spec(31, "f", "speed loop damping"),
  v_bw: spec(32, "f", "speed loop filter bandwidth"),
  iq_c1: spec(33, "f", "current loop gain"),
  vl_c1: spec(34, "f", "speed loop gain"),
  can_br: spec(35, "u32", "CAN bitrate code (4 = 1 Mbit/s)"),
  sub_ver: spec(36, "u32", "sub version"),
  u_off: spec(50, "f", "U phase offset"),
  v_off: spec(51, "f", "V phase offset"),
  k1: spec(52, "f", "compensation factor 1"),
  k2: spec(53, "f", "compensation factor 2"),
  m_off: spec(54, "f", "angle offset"),
  dir: spec(55, "f", "direction"),
  p_m: spec(80, "f", "motor position (rad)"),
  xout: spec(81, "f", "output shaft position (rad)"),
});

/** Register index -> dtype. */
export const REGISTER_DTYPES: ReadonlyMap<number, string> = new Map(Object.values(REGISTERS).map((s) => [s.index, s.dtype]));

const checkDtype = (dtype: string): void => {
  if (dtype !== "f" && dtype !== "u32") throw new ValueError(`unknown register dtype '${dtype}' (choices: f, u32)`);
};

/** Known registers use their own type; unknown indices are read as float32 (DM_CAN default). */
export function registerDtype(index: number): string {
  return REGISTER_DTYPES.get(index) ?? "f";
}

/** A register value as its four little-endian bytes. */
export function packValue(value: number, dtype: string): Uint8Array {
  checkDtype(dtype);
  const out = new Uint8Array(4);
  const view = new DataView(out.buffer);
  if (dtype === "f") {
    value = Number(value);
    if (!Number.isFinite(value)) throw new ValueError("register values must be finite");
    if (!Number.isFinite(Math.fround(value))) throw new ValueError("float too large to pack with f format");
    view.setFloat32(0, value, true);
    return out;
  }
  if (Number.isNaN(value)) throw new ValueError("cannot convert float NaN to integer");
  if (!Number.isFinite(value)) throw new ValueError("cannot convert float infinity to integer");
  const n = Math.trunc(value);
  if (!(n >= 0 && n <= 0xffffffff)) throw new ValueError("'I' format requires 0 <= number <= 4294967295");
  view.setUint32(0, n, true);
  return out;
}

/** Four little-endian bytes as a register value (float32 widened to a double, or u32). */
export function unpackValue(raw: Uint8Array, dtype: string): number {
  checkDtype(dtype);
  if (raw.length !== 4) throw new ValueError("unpack requires a buffer of 4 bytes");
  const view = new DataView(Uint8Array.from(raw).buffer);
  return dtype === "f" ? view.getFloat32(0, true) : view.getUint32(0, true);
}

function checkId(motorId: number): number {
  if (!(ID_MIN <= motorId && motorId <= ID_MAX)) {
    throw new ValueError(`Damiao motor id must be in [${ID_MIN}, ${ID_MAX}], got ${motorId}`);
  }
  return motorId;
}

function registerRid(rid: number): number {
  if (!(0 <= rid && rid <= 0xff)) throw new ValueError(`register index must be in [0, 255], got ${rid}`);
  return rid;
}

const concat = (a: ArrayLike<number>, b: ArrayLike<number>): Uint8Array => {
  const out = new Uint8Array(a.length + b.length);
  out.set(a, 0);
  out.set(b, a.length);
  return out;
};

const modeOffset = (mode: number): number => {
  if (!Object.hasOwn(MODE_OFFSETS, mode)) throw new ValueError(`unknown control mode ${mode}`);
  return MODE_OFFSETS[mode]!;
};

// ---------------------------------------------------------------- encoders [canId, payload]

/** Seven 0xFF bytes and `command`, to the send id + the mode offset. */
export function encodeCommand(motorId: number, command: number, mode: number = MODE_MIT): Frame {
  return [checkId(motorId) + modeOffset(mode), concat(COMMAND_PREFIX, [command])];
}

export function encodeEnable(motorId: number, mode: number = MODE_MIT): Frame {
  return encodeCommand(motorId, CMD_ENABLE, mode);
}

export function encodeDisable(motorId: number, mode: number = MODE_MIT): Frame {
  return encodeCommand(motorId, CMD_DISABLE, mode);
}

export function encodeSetZero(motorId: number, mode: number = MODE_MIT): Frame {
  return encodeCommand(motorId, CMD_SET_ZERO, mode);
}

export function encodeClearError(motorId: number, mode: number = MODE_MIT): Frame {
  return encodeCommand(motorId, CMD_CLEAR_ERROR, mode);
}

/** Read-only state request: the motor answers with a state frame, torque state unchanged. */
export function encodeRefresh(motorId: number): Frame {
  checkId(motorId);
  return [PARAM_CAN_ID, Uint8Array.from([motorId & 0xff, motorId >> 8, OP_REFRESH, 0, 0, 0, 0, 0])];
}

export function encodeReadRegister(motorId: number, rid: number): Frame {
  checkId(motorId);
  return [PARAM_CAN_ID, Uint8Array.from([motorId & 0xff, motorId >> 8, OP_READ, registerRid(rid), 0, 0, 0, 0])];
}

/** Write to RAM only; the value is lost at power-off unless saved with encodeSave(). */
export function encodeWriteRegister(motorId: number, rid: number, value: number, dtype?: string | null): Frame {
  checkId(motorId);
  const raw = packValue(value, dtype || registerDtype(rid));
  return [PARAM_CAN_ID, concat([motorId & 0xff, motorId >> 8, OP_WRITE, registerRid(rid)], raw)];
}

/** Store every register to flash (DM_CAN save_motor_param). Only honoured while disabled. */
export function encodeSave(motorId: number): Frame {
  checkId(motorId);
  return [PARAM_CAN_ID, Uint8Array.from([motorId & 0xff, motorId >> 8, OP_SAVE, 0x01, 0, 0, 0, 0])];
}

/** Clamp then truncate, as the Damiao reference encoder does. */
export function floatToUint(value: number, lo: number, hi: number, bits: number): number {
  if (!Number.isFinite(value)) throw new ValueError("MIT commands must be finite");
  value = Math.min(hi, Math.max(lo, value));
  return Math.trunc(((value - lo) / (hi - lo)) * (2 ** bits - 1));
}

export function uintToFloat(value: number, lo: number, hi: number, bits: number): number {
  return (value / (2 ** bits - 1)) * (hi - lo) + lo;
}

/** MIT impedance command: tau_out = kp*(pos - q) + kd*(vel - dq) + tau. Motor coordinates, SI. */
export function encodeMit(motorId: number, pos: number, vel: number, kp: number, kd: number, tau: number,
                          model: MotorModel): Frame {
  const q = floatToUint(pos, -model.pMax, model.pMax, 16);
  const v = floatToUint(vel, -model.vMax, model.vMax, 12);
  const k = floatToUint(kp, KP_MIN, KP_MAX, 12);
  const d = floatToUint(kd, KD_MIN, KD_MAX, 12);
  const t = floatToUint(tau, -model.tMax, model.tMax, 12);
  const payload = Uint8Array.from([q >> 8, q & 0xff, v >> 4, ((v & 0xf) << 4) | (k >> 8), k & 0xff,
    d >> 4, ((d & 0xf) << 4) | (t >> 8), t & 0xff]);
  if (startsWithCommandPrefix(payload)) {
    // Seven 0xFF bytes would read as a command (enable/disable/zero/clear). One position LSB
    // (0.19 mrad) lower keeps the frame an MIT command.
    payload[1] = 0xfe;
  }
  return [checkId(motorId), payload];
}

/** Are the first seven bytes all 0xFF? */
export function startsWithCommandPrefix(data: Uint8Array): boolean {
  if (data.length < 7) return false;
  for (let i = 0; i < 7; i++) if (data[i] !== 0xff) return false;
  return true;
}

// ---------------------------------------------------------------- decoders

/** A motor's state frame, decoded. */
export class StateFrame {
  constructor(
    /** Low nibble of the send id. */
    readonly motorId: number,
    /** See STATUS_NAMES. */
    readonly status: number,
    /** rad, motor coordinates. */
    readonly position: number,
    /** rad/s */
    readonly velocity: number,
    /** N·m */
    readonly torque: number,
    /** °C */
    readonly tMos: number,
    /** °C */
    readonly tRotor: number,
  ) {
    Object.freeze(this);
  }
}

/** A register reply: the request header echoed and the value bytes. */
export class RegisterReply {
  constructor(
    /** Send id echoed in bytes 0-1. */
    readonly motorId: number,
    /** OP_READ / OP_WRITE / OP_SAVE. */
    readonly op: number,
    readonly rid: number,
    /** Bytes 4-7. */
    readonly raw: Uint8Array,
  ) {
    Object.freeze(this);
  }

  value(dtype?: string | null): number {
    return unpackValue(this.raw, dtype || registerDtype(this.rid));
  }
}

/**
 * Register replies echo the request header [id_lo, id_hi, op, rid]. A disabled state frame whose
 * position high byte is 0 and low byte an opcode looks the same; it is taken as a register reply
 * and that one sample is dropped.
 */
export function isRegisterReply(data: Uint8Array): boolean {
  return data.length === 8 && data[1] === 0 && REGISTER_OPS.includes(data[2]!) && ID_MIN <= data[0]! && data[0]! <= ID_MAX;
}

/** A frame addressed to a motor (any control mode) or to 0x7FF: a host's request, never a reply. */
export function isCommandId(canId: number): boolean {
  return canId === PARAM_CAN_ID || ([0, 1, 2, 3].includes(canId >> 8) && ID_MIN <= (canId & 0xff) && (canId & 0xff) <= ID_MAX);
}

/** Decode one received frame. Anything that is not a Damiao reply returns null. */
export function decode(canId: number, data: Uint8Array, model: MotorModel, extended = false): StateFrame | RegisterReply | null {
  if (extended || !(0 <= canId && canId <= 0x7ff) || data.length !== 8) return null;
  if (isCommandId(canId)) return null;
  if (isRegisterReply(data)) return new RegisterReply(data[0]!, data[2]!, data[3]!, data.slice(4, 8));
  const status = data[0]! >> 4;
  if (!hasStatus(status)) return null;
  const q = (data[1]! << 8) | data[2]!;
  const v = (data[3]! << 4) | (data[4]! >> 4);
  const t = ((data[4]! & 0xf) << 8) | data[5]!;
  return new StateFrame(
    data[0]! & 0xf,
    status,
    uintToFloat(q, -model.pMax, model.pMax, 16),
    uintToFloat(v, -model.vMax, model.vMax, 12),
    uintToFloat(t, -model.tMax, model.tMax, 12),
    data[6]!,
    data[7]!,
  );
}

/** A motor's state frame (used by simulators and tests). */
export function encodeState(motorId: number, status: number, pos: number, vel: number, tau: number,
                            tMos: number, tRotor: number, model: MotorModel): Uint8Array {
  const q = floatToUint(pos, -model.pMax, model.pMax, 16);
  const v = floatToUint(vel, -model.vMax, model.vMax, 12);
  const t = floatToUint(tau, -model.tMax, model.tMax, 12);
  return Uint8Array.from([(status << 4) | (motorId & 0xf), q >> 8, q & 0xff, v >> 4,
    ((v & 0xf) << 4) | (t >> 8), t & 0xff, Math.trunc(tMos) & 0xff, Math.trunc(tRotor) & 0xff]);
}

/** Python's math.isclose(a, b, rel_tol=relTol). */
export function isClose(a: number, b: number, relTol = 1e-9): boolean {
  if (a === b) return true;
  if (!Number.isFinite(a) || !Number.isFinite(b)) return false;
  const diff = Math.abs(b - a);
  return diff <= Math.abs(relTol * b) || diff <= Math.abs(relTol * a);
}

/** Name of the model whose scaling equals these register values, if any. */
export function matchingModel(pMax: number, vMax: number, tMax: number): string | null {
  for (const [name, m] of Object.entries(MODELS)) {
    if (isClose(pMax, m.pMax, 1e-5) && isClose(vMax, m.vMax, 1e-5) && isClose(tMax, m.tMax, 1e-5)) return name;
  }
  return null;
}
