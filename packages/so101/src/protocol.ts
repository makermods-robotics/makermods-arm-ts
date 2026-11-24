/** Feetech STS3215 bus servos (the SO-101's six motors, follower and leader alike), as lerobot's FeetechMotorsBus
(lerobot/motors/feetech, over scservo_sdk protocol 0) speaks them, over one UART at 1 Mbaud.

Packets are `FF FF id len instr params… checksum`, len = params + 2, checksum = ~(id + len + instr + Σparams);
a status reply is `FF FF id len error params… checksum`. Multi-byte fields are little-endian; Homing_Offset is
sign-magnitude with its sign at bit 11, Goal/Present_Position and the velocities at bit 15, Present_Load at bit 10
(tables.py STS_SMS_SERIES_ENCODINGS_TABLE).

Every exchange is one request and its reply (or, for a sync read, one reply per servo); the caller serializes them
(one Arbiter per port). Sync write has no reply. */

import { ArmError } from "@makermods/arm-core";

/** The SO-101 bus's baud rate. */
export const FEETECH_BAUDRATE = 1_000_000;

/** Instruction bytes (scservo_sdk) and the broadcast id. */
export const INST_PING = 0x01;
export const INST_READ = 0x02;
export const INST_WRITE = 0x03;
export const INST_REG_WRITE = 0x04;
export const INST_ACTION = 0x05;
export const INST_SYNC_READ = 0x82;
export const INST_SYNC_WRITE = 0x83;
export const BROADCAST_ID = 0xfe;

/** STS_SMS_SERIES_CONTROL_TABLE: [address, length]. */
export const REG = {
  Firmware_Major_Version: [0, 1],
  Firmware_Minor_Version: [1, 1],
  Model_Number: [3, 2],
  ID: [5, 1],
  Return_Delay_Time: [7, 1],
  Min_Position_Limit: [9, 2],
  Max_Position_Limit: [11, 2],
  Max_Torque_Limit: [16, 2],
  Phase: [18, 1],
  P_Coefficient: [21, 1],
  D_Coefficient: [22, 1],
  I_Coefficient: [23, 1],
  Protection_Current: [28, 2],
  Homing_Offset: [31, 2],
  Operating_Mode: [33, 1],
  Overload_Torque: [36, 1],
  Torque_Enable: [40, 1],
  Acceleration: [41, 1],
  Goal_Position: [42, 2],
  Goal_Time: [44, 2],
  Goal_Velocity: [46, 2],
  Torque_Limit: [48, 2],
  Lock: [55, 1],
  Present_Position: [56, 2],
  Present_Velocity: [58, 2],
  Present_Load: [60, 2],
  Present_Voltage: [62, 1],
  Present_Temperature: [63, 1],
  Status: [65, 1],
  Moving: [66, 1],
  Present_Current: [69, 2],
  Maximum_Acceleration: [85, 1],
} as const satisfies Record<string, readonly [number, number]>;
/** A control-table register's name. */
export type RegName = keyof typeof REG;

/** STS_SMS_SERIES_ENCODINGS_TABLE: the sign bit of each sign-magnitude field. */
export const SIGN_BITS: Partial<Record<RegName, number>> = {
  Present_Load: 10,
  Homing_Offset: 11,
  Goal_Position: 15,
  Goal_Velocity: 15,
  Present_Position: 15,
  Present_Velocity: 15,
};

/** The STS3215's encoder: 4096 steps a turn (model_resolution_table "sts3215"). */
export const RESOLUTION = 4096;
export const MAX_RES = RESOLUTION - 1;
/** OperatingMode.POSITION. */
export const MODE_POSITION = 0;
/** OperatingMode.VELOCITY (auto-calibration's probes). */
export const MODE_VELOCITY = 1;
/** Model_Number of an STS3215. */
export const STS3215_MODEL = 777;

/** Reply windows: a status packet is back within a millisecond at 1 Mbaud with Return_Delay_Time 0; Web Serial and
the USB adapter add a few. */
export const REPLY_TIMEOUT_MS = 30;
/** A write to the EEPROM area (addresses below 40) with the EEPROM unlocked is saved before the servo answers: about
35 ms on an STS3215 (firmware 3.10, measured 2026-10-19), past REPLY_TIMEOUT_MS. lerobot waits up to 1000 ms. */
export const EEPROM_REPLY_TIMEOUT_MS = 200;
/** Addresses below this are the EEPROM area. */
export const EEPROM_END = 40;
/** The longest single UART read while waiting for a reply. */
export const READ_SLICE_MS = 5;

/** `value` as sign-magnitude with its sign at `signBit` (RangeError past the magnitude it can hold). */
export function encodeSignMagnitude(value: number, signBit: number): number {
  const max = (1 << signBit) - 1;
  const magnitude = Math.abs(Math.trunc(value));
  if (magnitude > max) throw new RangeError(`Magnitude ${magnitude} exceeds ${max} (max for sign_bit_index=${signBit})`);
  return value < 0 ? (1 << signBit) | magnitude : magnitude;
}

/** A sign-magnitude field with its sign at `signBit`, decoded. */
export function decodeSignMagnitude(encoded: number, signBit: number): number {
  const negative = (encoded >> signBit) & 1;
  const magnitude = encoded & ((1 << signBit) - 1);
  return negative ? -magnitude : magnitude;
}

/** A register value as the servo stores it (sign-magnitude where SIGN_BITS says so). */
export function encodeValue(reg: RegName, value: number): number {
  const bit = SIGN_BITS[reg];
  return bit === undefined ? Math.trunc(value) : encodeSignMagnitude(value, bit);
}

/** A register's raw value decoded (sign-magnitude where SIGN_BITS says so). */
export function decodeValue(reg: RegName, raw: number): number {
  const bit = SIGN_BITS[reg];
  return bit === undefined ? raw : decodeSignMagnitude(raw, bit);
}

const lo = (v: number) => v & 0xff;
const hi = (v: number) => (v >> 8) & 0xff;

/** `value`'s `length` bytes, little-endian. */
export function bytesOf(value: number, length: number): number[] {
  if (length === 1) return [lo(value)];
  if (length === 2) return [lo(value), hi(value)];
  return [lo(value), hi(value), (value >> 16) & 0xff, (value >> 24) & 0xff];
}

/** The little-endian value of `length` bytes from `offset`. */
export function valueOf(bytes: ArrayLike<number>, offset: number, length: number): number {
  let v = 0;
  for (let i = length - 1; i >= 0; i--) v = v * 256 + (bytes[offset + i] as number);
  return v;
}

/** A packet's checksum: ~(id + len + instr + params) & 0xff. */
export function checksum(id: number, length: number, instr: number, params: ArrayLike<number>): number {
  let sum = id + length + instr;
  for (let i = 0; i < params.length; i++) sum += params[i] as number;
  return ~sum & 0xff;
}

/** One instruction packet: `FF FF id len instr params checksum`. */
export function packet(id: number, instr: number, params: ArrayLike<number> = []): Uint8Array {
  const length = params.length + 2;
  const out = new Uint8Array(params.length + 6);
  out.set([0xff, 0xff, id, length, instr]);
  out.set(Array.from(params), 5);
  out[out.length - 1] = checksum(id, length, instr, params);
  return out;
}

/** PING. */
export const pingPacket = (id: number) => packet(id, INST_PING);
/** READ `length` bytes from `address`. */
export const readPacket = (id: number, address: number, length: number) => packet(id, INST_READ, [address, length]);
/** WRITE `data` from `address`. */
export const writePacket = (id: number, address: number, data: readonly number[]) => packet(id, INST_WRITE, [address, ...data]);
/** SYNC_READ `length` bytes from `address` of every servo in `ids` (one reply each). */
export const syncReadPacket = (ids: readonly number[], address: number, length: number) =>
  packet(BROADCAST_ID, INST_SYNC_READ, [address, length, ...ids]);
/** SYNC_WRITE one `length`-byte register of several servos (no reply). */
export function syncWritePacket(address: number, length: number, values: ReadonlyArray<readonly [number, number]>): Uint8Array {
  const params = [address, length];
  for (const [id, raw] of values) params.push(id, ...bytesOf(raw, length));
  return packet(BROADCAST_ID, INST_SYNC_WRITE, params);
}

/** A servo's status reply: its id, error byte and parameter bytes. */
export interface StatusPacket {
  id: number;
  error: number;
  params: Uint8Array;
}

/** Every well-formed status packet in `data` (a bad checksum skips the header and resyncs). */
export function parseStatus(data: Uint8Array): { packets: StatusPacket[]; badChecksums: number } {
  const packets: StatusPacket[] = [];
  let badChecksums = 0;
  let i = 0;
  while (i + 6 <= data.length) {
    if (data[i] !== 0xff || data[i + 1] !== 0xff || data[i + 2] === 0xff) {
      i += 1;
      continue;
    }
    const id = data[i + 2] as number;
    const length = data[i + 3] as number;
    const end = i + 4 + length;
    if (length < 2) {
      i += 1;
      continue;
    }
    if (end > data.length) break;
    const error = data[i + 4] as number;
    const params = data.slice(i + 5, end - 1);
    if (data[end - 1] === checksum(id, length, error, params)) {
      packets.push({ id, error, params });
      i = end;
    } else {
      badChecksums += 1;
      i += 2;
    }
  }
  return { packets, badChecksums };
}

/** The servo's status error byte (scservo_sdk getRxPacketError). */
export function errorText(error: number): string {
  const bits: string[] = [];
  if (error & 0x01) bits.push("input voltage");
  if (error & 0x02) bits.push("angle sensor");
  if (error & 0x04) bits.push("overheat");
  if (error & 0x08) bits.push("overcurrent");
  if (error & 0x20) bits.push("overload");
  return bits.length ? bits.join(", ") : `error 0x${error.toString(16)}`;
}

/** One servo's live state (FeetechBus.syncReadState). */
export interface ServoState {
  torque: number;
  position: number;
  load: number;
  /** 0.1 V units. */
  voltage: number;
  temperatureC: number;
  /** The Status register (bit 5: overload). */
  status: number;
  /** The reply's error byte. */
  error: number;
  /** Present_Velocity, steps/s (in the same sync read; the console's driver leaves it undecoded). */
  velocity: number;
}

/** The sync read of a servo's live state: Torque_Enable at 40 through Moving at 66, 27 bytes. */
export const STATE_ADDRESS = REG.Torque_Enable[0];
export const STATE_LENGTH = REG.Moving[0] + REG.Moving[1] - STATE_ADDRESS;

/** Error bits that stop a run or refuse a start: angle sensor, overheat, overcurrent, overload (the Status register's
bit 5 too). The input-voltage bit (0x01) is reported but not a fault: it flags the supply leaving the servo's
configured window, which a 12 V board's sag at the start of a move can do, and it does not cut torque. */
export const FAULT_BITS = 0x02 | 0x04 | 0x08 | 0x20;
/** Whether a servo's state reports a fault (FAULT_BITS in its reply's error byte, or the Status overload bit). */
export const isFault = (s: Pick<ServoState, "error" | "status">) => Boolean((s.error & FAULT_BITS) || (s.status & 0x20));

/** No reply, a short reply, or a protocol misuse on a Feetech bus. */
export class FeetechError extends ArmError {}

/** A reply whose error byte is set (lerobot's raise_on_error): the servo answered, flagging a fault. */
export class FeetechServoError extends FeetechError {
  constructor(
    readonly id: number,
    readonly error: number,
    what: string,
  ) {
    super(`Servo ${id} ${what}: ${errorText(error)}`);
  }
}

// ---------- Calibration and normalization (lerobot MotorCalibration, MotorsBus._normalize) ----------

/** One servo's LeRobot calibration entry. */
export interface FeetechCalibration {
  id: number;
  drive_mode: number;
  homing_offset: number;
  range_min: number;
  range_max: number;
}

/** MotorNormMode.DEGREES: degrees from the middle of the calibrated range. */
export function rawToDegrees(raw: number, c: Pick<FeetechCalibration, "range_min" | "range_max">): number {
  const mid = (c.range_min + c.range_max) / 2;
  return ((raw - mid) * 360) / MAX_RES;
}

/** The inverse of rawToDegrees, truncated to a tick. */
export function degreesToRaw(deg: number, c: Pick<FeetechCalibration, "range_min" | "range_max">): number {
  const mid = (c.range_min + c.range_max) / 2;
  return Math.trunc((deg * MAX_RES) / 360 + mid);
}

/** MotorNormMode.RANGE_0_100 (the gripper): 0 at range_min, 100 at range_max (drive_mode flips it). */
export function rawToPercent(raw: number, c: FeetechCalibration): number {
  const bounded = Math.min(c.range_max, Math.max(c.range_min, raw));
  const norm = ((bounded - c.range_min) / (c.range_max - c.range_min)) * 100;
  return c.drive_mode ? 100 - norm : norm;
}

/** The inverse of rawToPercent, truncated to a tick. */
export function percentToRaw(value: number, c: FeetechCalibration): number {
  const v = c.drive_mode ? 100 - value : value;
  const bounded = Math.min(100, Math.max(0, v));
  return Math.trunc((bounded / 100) * (c.range_max - c.range_min) + c.range_min);
}
