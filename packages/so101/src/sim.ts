/** A simulated Feetech STS3215 bus (the SO-101's six servos) with time-based joint physics on the injected Clock, for
tests and for application developers (the console's test/sim/feetech-bus.ts). SimSerialPort puts it behind a
SerialPort-shaped handle, so So101Arm, identify and calibration run on it unchanged; `uart` is what a FeetechBus
talks to directly. Replies arrive `latencyMs` after the request (sync-read replies one after another).

Protocol (scservo_sdk protocol 0): PING, READ, WRITE, REG_WRITE, ACTION, SYNC_READ, SYNC_WRITE, broadcast id 0xFE;
a request with a bad checksum is ignored (no reply), an offline servo neither applies nor answers anything.

Register file: one 256-byte live table per servo at the STS_SMS_SERIES_CONTROL_TABLE addresses (Model_Number 777,
power-on defaults below), plus a persisted EEPROM copy of addresses 0..39 that `powerCycle` restores.

Semantics, and the assumptions where the datasheet is thin:
- Present_Position = (actual − Homing_Offset) mod 4096 (lerobot _get_half_turn_homings: "Present_Position =
  Actual_Position − Homing_Offset"; Phase bit 4 clear = single-turn feedback), Homing_Offset sign-magnitude at bit 11,
  positions and velocities at bit 15, Present_Load at bit 10.
- Lock (55): writes to EEPROM addresses (< 40) always take effect live; they persist only while Lock is 0
  (lerobot disable_torque writes Lock=0 before write_calibration, enable_torque Lock=1). Power-on Lock assumed 1.
- Torque_Enable 0: torque off, clears an overload latch. 1: torque on, holding where it is (Goal_Position := present);
  refused while the overload latch is set (the status reply carries error 0x20 and the register stays 0: "a bare
  Torque_Enable=1 NAKs on such a servo", auto_calibrate_script.py _freeze_arm). 128: re-centre, Feetech's "current
  position becomes the midpoint" (auto_calibration.py unfold_single_joint: "Torque_Enable=128 sets the current
  position to 2048"): Homing_Offset := actual − 2048 (magnitude clamped to 2047, so actual 0 reads 2047), written to
  the EEPROM copy regardless of Lock, Goal_Position := 2048, the torque state unchanged.
- Operating_Mode 0 (position): drives present toward clamp(Goal_Position, Min/Max_Position_Limit) along the 0..4095
  register line (never across the seam), at most Goal_Velocity steps/s (0 = maxSpeed), accelerating and braking at
  Acceleration × 100 steps/s² (0 = instant). Entering position mode holds the current position. Mode 1 (velocity):
  runs at Goal_Velocity (sign = direction), limits ignored, same acceleration.
- Hard stops: a joint's free arc runs up from `lo` to `hi` in actual encoder ticks (wrapping across 4095 → 0 when
  hi < lo). Motion stops dead at a stop; while the drive keeps pushing into it (velocity toward it, or a position
  target beyond it) Moving stays 1 in position mode, Present_Load reads ±Torque_Limit, and after `overloadMs` of
  continuous push the overload latch trips: torque output is cut, Status bit 5 (0x20) is set and every reply to a
  read or ping carries error 0x20 until Torque_Enable=0. Writes are applied and acknowledged cleanly while latched
  (the script's freeze writes Operating_Mode/Torque_Limit to latched servos and expects them to succeed), except
  Torque_Enable=1 as above.
- Moving = |velocity| > 1 step/s, or (position mode, torque on) pushing a stop. Present_Velocity is the rounded speed.
- Torque off: the joint stays where it is (no gravity) unless moved by hand (`moveByHand`).
Not modelled: ID/baud changes, Goal_Time, multi-turn (Phase bit 4), gravity, current. */

import type { Clock } from "@makermods/arm-core";
import { FedUart } from "./bus.js";
import { msClock, type MsClock } from "./clock-ms.js";
import {
  BROADCAST_ID,
  INST_ACTION,
  INST_PING,
  INST_READ,
  INST_REG_WRITE,
  INST_SYNC_READ,
  INST_SYNC_WRITE,
  INST_WRITE,
  REG,
  RESOLUTION,
  STS3215_MODEL,
  bytesOf,
  checksum,
  decodeValue,
  encodeValue,
  valueOf,
  type RegName,
} from "./protocol.js";
import { SO101_MOTORS, type So101Calibration } from "./so101.js";

const EEPROM_END = 40;
/** Present_Position .. Present_Current: feedback, writes ignored. */
const FEEDBACK = [56, 71] as const;
const TORQUE_ENABLE = REG.Torque_Enable[0];
const OPERATING_MODE = REG.Operating_Mode[0];
const LOCK = REG.Lock[0];
const STATUS_OVERLOAD = 0x20;

const mod = (a: number, n: number) => ((a % n) + n) % n;

/** One applied register write, for test inspection (`instr` is the instruction that carried it; ACTION for staged;
`t` the clock's seconds). */
export interface SimWrite {
  t: number;
  id: number;
  instr: number;
  address: number;
  data: number[];
}

/** One simulated STS3215: its register file, EEPROM copy and joint physics. */
export class SimStsServo {
  readonly regs = new Uint8Array(256);
  /** Persisted copy of the EEPROM area (addresses 0..39). */
  readonly eeprom = new Uint8Array(EEPROM_END);
  online = true;
  /** Physical position in unwrapped encoder ticks (the encoder reads it mod 4096). */
  pos: number;
  /** Physical velocity, steps/s. */
  vel = 0;
  /** Free arc in unwrapped ticks (min < max), or null for a free joint. */
  stops: { min: number; max: number } | null = null;
  /** Overload latch (torque cut, Status 0x20, error 0x20 on read replies). */
  latched = false;
  /** Pushing a stop this instant, and for how long continuously (ms). */
  pushing = false;
  pushDir = 1;
  pushMs = 0;
  /** Continuous push before the overload latch trips. */
  overloadMs = 1000;
  /** Top speed in position mode with Goal_Velocity 0 (steps/s). */
  maxSpeed = 3400;
  /** Torque_Limit below which the joint cannot move at all (a heavy joint at low power); 0 = always moves. */
  requiredTorque = 0;
  /** OR'ed into every status reply's error byte (e.g. 0x01 input voltage). */
  forcedError = 0;
  staged: { address: number; data: number[] } | null = null;

  constructor(
    readonly id: number,
    actual = 2048,
  ) {
    this.pos = actual;
    const set = (name: RegName, v: number) => this.setReg(name, v);
    set("Firmware_Major_Version", 3);
    set("Firmware_Minor_Version", 10);
    set("Model_Number", STS3215_MODEL);
    set("ID", id);
    set("Return_Delay_Time", 250);
    this.regs[8] = 1; // Response_Status_Level
    set("Min_Position_Limit", 0);
    set("Max_Position_Limit", RESOLUTION - 1);
    set("Max_Torque_Limit", 1000);
    set("Phase", 0x0c);
    set("P_Coefficient", 32);
    set("D_Coefficient", 32);
    set("I_Coefficient", 0);
    set("Protection_Current", 500);
    this.regs[34] = 20; // Protection_Torque
    this.regs[35] = 200; // Protection_Time
    set("Overload_Torque", 80);
    set("Torque_Limit", 1000);
    set("Lock", 1);
    set("Maximum_Acceleration", 254);
    this.eeprom.set(this.regs.subarray(0, EEPROM_END));
    this.setReg("Goal_Position", this.present);
    this.refresh();
  }

  reg(name: RegName): number {
    const [address, length] = REG[name];
    return decodeValue(name, valueOf(this.regs, address, length));
  }

  setReg(name: RegName, value: number): void {
    const [address, length] = REG[name];
    this.regs.set(bytesOf(encodeValue(name, value), length), address);
  }

  /** Encoder reading, 0..4095. */
  get actual(): number {
    return mod(Math.round(this.pos), RESOLUTION);
  }

  get present(): number {
    return mod(this.actual - this.reg("Homing_Offset"), RESOLUTION);
  }

  get torqueOn(): boolean {
    return this.regs[TORQUE_ENABLE] === 1 && !this.latched;
  }

  /** Free arc from `lo` up to `hi` (actual ticks; hi < lo wraps across 4095 → 0). A joint outside it is moved to
  the nearer stop. */
  setHardStops(lo: number, hi: number): void {
    const min = mod(lo, RESOLUTION);
    let max = mod(hi, RESOLUTION);
    if (max <= min) max += RESOLUTION;
    this.stops = { min, max };
    let p = this.actual;
    if (p < min) p += RESOLUTION;
    if (p > max) p = p - max < min + RESOLUTION - p ? max : min;
    this.place(p);
  }

  /** Puts the joint at encoder reading `actual` (inside its stops; the unwrapped copy within the arc). */
  place(actual: number): void {
    let p = mod(actual, RESOLUTION);
    if (this.stops) {
      const { min, max } = this.stops;
      if (p < min) p += RESOLUTION;
      if (p > max) throw new RangeError(`servo ${this.id}: ${actual} is outside its hard stops`);
    }
    this.pos = p;
    this.vel = 0;
  }

  /** Applies one write; returns the status error byte (0x20 for a Torque_Enable=1 refused by the latch). */
  applyWrite(address: number, data: readonly number[]): number {
    const oldLock = this.regs[LOCK];
    const oldMode = this.regs[OPERATING_MODE];
    let error = 0;
    const te = TORQUE_ENABLE - address;
    if (te >= 0 && te < data.length) error = this.torqueEnable(data[te] as number);
    data.forEach((byte, i) => {
      const a = address + i;
      if (a === TORQUE_ENABLE || (a >= FEEDBACK[0] && a <= FEEDBACK[1])) return;
      this.regs[a] = byte;
      if (a < EEPROM_END && oldLock === 0) this.eeprom[a] = byte;
    });
    if (oldMode !== 0 && this.regs[OPERATING_MODE] === 0) this.setReg("Goal_Position", this.present);
    this.refresh();
    return error;
  }

  private torqueEnable(v: number): number {
    if (v === 0) {
      this.regs[TORQUE_ENABLE] = 0;
      this.latched = false;
      this.pushMs = 0;
    } else if (v === 1) {
      if (this.latched) return STATUS_OVERLOAD;
      if (this.regs[TORQUE_ENABLE] !== 1) this.setReg("Goal_Position", this.present);
      this.regs[TORQUE_ENABLE] = 1;
    } else if (v === 128) {
      const offset = Math.max(-2047, Math.min(2047, this.actual - 2048));
      this.setReg("Homing_Offset", offset);
      const [address, length] = REG.Homing_Offset;
      this.eeprom.set(this.regs.subarray(address, address + length), address);
      this.setReg("Goal_Position", this.present);
    } else {
      this.regs[TORQUE_ENABLE] = v;
    }
    return 0;
  }

  /** Error byte of a status reply to a read or ping. */
  get replyError(): number {
    return (this.latched ? STATUS_OVERLOAD : 0) | this.forcedError;
  }

  /** Advances the physics by `dt` seconds. Returns true when the overload latch tripped during this step. */
  step(dt: number): boolean {
    const mode = this.regs[OPERATING_MODE];
    const accel = this.reg("Acceleration") * 100 || Infinity;
    let desired = 0;
    let target: number | null = null;
    if (this.torqueOn) {
      if (mode === 1) desired = this.reg("Goal_Velocity");
      else if (mode === 0) {
        const lo = this.reg("Min_Position_Limit");
        const hi = this.reg("Max_Position_Limit");
        target = Math.max(lo, Math.min(hi, this.reg("Goal_Position")));
        const err = target - mod(this.pos - this.reg("Homing_Offset"), RESOLUTION);
        const vmax = Math.abs(this.reg("Goal_Velocity")) || this.maxSpeed;
        desired = Math.abs(err) < 1e-6 ? 0 : Math.sign(err) * Math.min(vmax, Math.sqrt(2 * (accel === Infinity ? 1e9 : accel) * Math.abs(err)));
      }
      if (this.reg("Torque_Limit") < this.requiredTorque) {
        this.vel = 0;
        this.pushing = desired !== 0;
        return this.overload(dt);
      }
    }
    if (!this.torqueOn) this.vel = 0;
    // Accelerate toward the desired speed.
    const dv = desired - this.vel;
    this.vel += Math.sign(dv) * Math.min(Math.abs(dv), accel * dt);
    let next = this.pos + this.vel * dt;
    if (target !== null) {
      // Land exactly on the position target instead of overshooting it.
      const err = target - mod(this.pos - this.reg("Homing_Offset"), RESOLUTION);
      if (Math.abs(err) <= Math.abs(this.vel * dt) && Math.sign(err) === Math.sign(this.vel || err)) {
        next = this.pos + err;
        this.vel = 0;
      }
    }
    this.pushing = false;
    if (this.stops) {
      if (next > this.stops.max) {
        next = this.stops.max;
        this.vel = 0;
      } else if (next < this.stops.min) {
        next = this.stops.min;
        this.vel = 0;
      }
      const atMax = next >= this.stops.max - 1e-9;
      const atMin = next <= this.stops.min + 1e-9;
      this.pushing = this.torqueOn && ((atMax && desired > 0) || (atMin && desired < 0));
      if (this.pushing) this.pushDir = Math.sign(desired);
    }
    this.pos = next;
    return this.overload(dt);
  }

  private overload(dt: number): boolean {
    if (!this.pushing) {
      this.pushMs = 0;
      return false;
    }
    this.pushMs += dt * 1000;
    if (this.latched || this.pushMs < this.overloadMs) return false;
    this.latched = true;
    this.vel = 0;
    this.pushing = false;
    return true;
  }

  /** Recomputes the feedback registers from the physical state. */
  refresh(): void {
    const posMode = this.regs[OPERATING_MODE] === 0;
    const moving = Math.abs(this.vel) > 1 || (posMode && this.torqueOn && this.pushing);
    const tl = this.reg("Torque_Limit");
    const load = this.pushing ? tl : Math.abs(this.vel) > 1 ? Math.min(tl, 100) : 0;
    this.setReg("Present_Position", this.present);
    this.setReg("Present_Velocity", Math.round(this.vel));
    this.setReg("Present_Load", Math.min(1023, load) * (this.pushing ? this.pushDir : Math.sign(this.vel) || 1));
    this.regs[REG.Present_Voltage[0]] = 120;
    this.regs[REG.Present_Temperature[0]] = 30;
    this.regs[REG.Status[0]] = this.latched ? STATUS_OVERLOAD : 0;
    this.regs[REG.Moving[0]] = moving ? 1 : 0;
    this.setReg("Present_Current", Math.round(load / 2));
  }

  /** Back to the persisted EEPROM and power-on RAM (torque off, latch cleared); the joint stays where it is. */
  powerCycle(): void {
    const pos = this.pos;
    const stops = this.stops;
    const fresh = new SimStsServo(this.id);
    this.regs.set(fresh.regs);
    this.regs.set(this.eeprom, 0);
    this.latched = false;
    this.pushMs = 0;
    this.staged = null;
    this.stops = stops;
    this.pos = pos;
    this.vel = 0;
    this.setReg("Goal_Position", this.present);
    this.refresh();
  }
}

function statusPacket(id: number, error: number, params: readonly number[] = []): Uint8Array {
  const length = params.length + 2;
  return Uint8Array.from([0xff, 0xff, id, length, error, ...params, checksum(id, length, error, params)]);
}

/** The simulated bus: servos by id, the wire, and test helpers. */
export class SimFeetechBus {
  readonly servos = new Map<number, SimStsServo>();
  readonly uart: FedUart;
  private readonly ms: MsClock;
  /** Every request packet the host sent, raw. */
  readonly requests: number[][] = [];
  /** Every register write applied, in order. */
  readonly writes: SimWrite[] = [];
  /** Delay before a reply starts, and between consecutive sync-read replies (ms). */
  latencyMs = 0.5;
  /** Physics substep (ms). */
  stepMs = 1;
  /** Called when a servo's overload latch trips. */
  onOverload: ((id: number) => void) | null = null;
  private lastT: number;

  constructor(
    readonly clock: Clock,
    ids: readonly number[] = [1, 2, 3, 4, 5, 6],
  ) {
    this.ms = msClock(clock);
    for (const id of ids) this.servos.set(id, new SimStsServo(id));
    this.uart = new FedUart(this, this.ms);
    this.lastT = this.ms.now();
  }

  servo(id: number): SimStsServo {
    const s = this.servos.get(id);
    if (!s) throw new RangeError(`no simulated servo ${id}`);
    return s;
  }

  /** Integrates the physics up to the clock's now, in `stepMs` substeps. */
  sync(): void {
    const now = this.ms.now();
    while (this.lastT < now) {
      const dt = Math.min(this.stepMs, now - this.lastT);
      for (const s of this.servos.values()) if (s.step(dt / 1000)) this.onOverload?.(s.id);
      this.lastT += dt;
    }
    for (const s of this.servos.values()) s.refresh();
  }

  // ---------- helpers for tests ----------

  setHardStops(id: number, lo: number, hi: number): void {
    this.sync();
    this.servo(id).setHardStops(lo, hi);
  }

  /** Teleports a joint to encoder reading `actual` (test setup). */
  place(id: number, actual: number): void {
    this.sync();
    this.servo(id).place(actual);
    this.servo(id).refresh();
  }

  /** Moves a limp joint by hand to encoder reading `actual` (clamped to its stops); throws while torque holds it. */
  moveByHand(id: number, actual: number): void {
    this.sync();
    const s = this.servo(id);
    if (s.torqueOn) throw new Error(`servo ${id} is holding (torque on); it cannot be moved by hand`);
    let p = mod(actual, RESOLUTION);
    if (s.stops) {
      if (p < s.stops.min) p += RESOLUTION;
      p = Math.max(s.stops.min, Math.min(s.stops.max, p));
    }
    s.pos = p;
    s.vel = 0;
    s.refresh();
  }

  setOnline(id: number, online: boolean): void {
    this.sync();
    this.servo(id).online = online;
  }

  /** Encoder reading (physical, offset-free). */
  actual(id: number): number {
    this.sync();
    return this.servo(id).actual;
  }

  /** A live register, decoded. */
  reg(id: number, name: RegName): number {
    this.sync();
    return this.servo(id).reg(name);
  }

  /** A register's persisted EEPROM value, decoded. */
  eeprom(id: number, name: RegName): number {
    const [address, length] = REG[name];
    if (address + length > EEPROM_END) throw new RangeError(`${name} is not in EEPROM`);
    return decodeValue(name, valueOf(this.servo(id).eeprom, address, length));
  }

  powerCycle(id?: number): void {
    this.sync();
    for (const s of id === undefined ? this.servos.values() : [this.servo(id)]) s.powerCycle();
  }

  /** Values written to `reg` of servo `id` (writes covering the whole register), decoded, in order. */
  writesTo(id: number, name: RegName): number[] {
    const [address, length] = REG[name];
    return this.writes
      .filter((w) => w.id === id && w.address <= address && w.address + w.data.length >= address + length)
      .map((w) => decodeValue(name, valueOf(w.data, address - w.address, length)));
  }

  // ---------- the wire ----------

  async write(bytes: Uint8Array): Promise<void> {
    this.sync();
    const b = Array.from(bytes);
    this.requests.push(b);
    let i = 0;
    while (i + 6 <= b.length) {
      if (b[i] !== 0xff || b[i + 1] !== 0xff) {
        i++;
        continue;
      }
      const id = b[i + 2] as number;
      const length = b[i + 3] as number;
      const end = i + 4 + length;
      if (length < 2 || end > b.length) break;
      const instr = b[i + 4] as number;
      const params = b.slice(i + 5, end - 1);
      if (b[end - 1] === checksum(id, length, instr, params)) this.handle(id, instr, params);
      i = end;
    }
  }

  private reply(packets: Uint8Array[]): void {
    packets.forEach((p, i) => {
      void this.ms.sleep(this.latencyMs * (i + 1)).then(() => this.uart.feed(p));
    });
  }

  private targets(id: number): SimStsServo[] {
    if (id === BROADCAST_ID) return [...this.servos.values()].filter((s) => s.online);
    const s = this.servos.get(id);
    return s?.online ? [s] : [];
  }

  private record(s: SimStsServo, instr: number, address: number, data: number[]): number {
    this.writes.push({ t: this.clock.now(), id: s.id, instr, address, data });
    return s.applyWrite(address, data);
  }

  private handle(id: number, instr: number, params: number[]): void {
    const broadcast = id === BROADCAST_ID;
    const out: Uint8Array[] = [];
    switch (instr) {
      case INST_PING:
        for (const s of this.targets(id)) out.push(statusPacket(s.id, s.replyError));
        break;
      case INST_READ: {
        const [address = 0, length = 0] = params;
        for (const s of this.targets(id)) if (!broadcast) out.push(statusPacket(s.id, s.replyError, Array.from(s.regs.subarray(address, address + length))));
        break;
      }
      case INST_WRITE:
        for (const s of this.targets(id)) {
          const error = this.record(s, instr, params[0] ?? 0, params.slice(1));
          if (!broadcast) out.push(statusPacket(s.id, error | s.forcedError));
        }
        break;
      case INST_REG_WRITE:
        for (const s of this.targets(id)) {
          s.staged = { address: params[0] ?? 0, data: params.slice(1) };
          if (!broadcast) out.push(statusPacket(s.id, s.forcedError));
        }
        break;
      case INST_ACTION:
        for (const s of this.targets(id)) {
          if (!s.staged) continue;
          const { address, data } = s.staged;
          s.staged = null;
          this.record(s, instr, address, data);
        }
        break;
      case INST_SYNC_READ: {
        const [address = 0, length = 0, ...ids] = params;
        for (const sid of ids) {
          const s = this.servos.get(sid);
          if (s?.online) out.push(statusPacket(s.id, s.replyError, Array.from(s.regs.subarray(address, address + length))));
        }
        break;
      }
      case INST_SYNC_WRITE: {
        const [address = 0, length = 0] = params;
        for (let k = 2; k + 1 + length <= params.length; k += 1 + length) {
          const s = this.servos.get(params[k] as number);
          if (s?.online) this.record(s, instr, address, params.slice(k + 1, k + 1 + length));
        }
        break;
      }
    }
    this.reply(out);
  }
}

// ---------- A SerialPort in front of the bus ----------

/** A SerialPort-shaped handle (open, readable, writable, close, getInfo; Web Serial's rule that a locked stream blocks
close) wired to a SimFeetechBus. Hooks let a test act before each host write (`beforeWrite`) or swallow one (`drop`). */
export class SimSerialPort {
  readable: ReadableStream<Uint8Array> | null = null;
  writable: WritableStream<Uint8Array> | null = null;
  baudRate: number | null = null;
  opens = 0;
  closes = 0;
  /** Thrown by the next open (a port another program holds). */
  failOpen: Error | null = null;
  /** Called before each host write reaches the bus. */
  beforeWrite: ((bytes: Uint8Array) => void) | null = null;
  /** Host writes for which this returns true never reach the bus. */
  drop: ((bytes: Uint8Array) => boolean) | null = null;
  /** Every host write, in order. */
  readonly written: Uint8Array[] = [];
  private controller: ReadableStreamDefaultController<Uint8Array> | null = null;

  constructor(
    readonly bus: SimFeetechBus,
    readonly label = "usb:1A86:55D3",
    private readonly info: { usbVendorId?: number; usbProductId?: number } = { usbVendorId: 0x1a86, usbProductId: 0x55d3 },
  ) {
    bus.uart.feed = (bytes: Uint8Array) => this.feed(bytes);
  }

  get isOpen(): boolean {
    return this.readable !== null;
  }

  getInfo(): { usbVendorId?: number; usbProductId?: number } {
    return { ...this.info };
  }

  async open({ baudRate }: { baudRate: number }): Promise<void> {
    if (this.failOpen) throw this.failOpen;
    if (this.readable) throw new DOMException("The port is already open.", "InvalidStateError");
    this.opens++;
    this.baudRate = baudRate;
    this.readable = new ReadableStream<Uint8Array>({
      start: (controller) => {
        this.controller = controller;
      },
      cancel: () => {
        this.controller = null;
      },
    });
    this.writable = new WritableStream<Uint8Array>({
      write: async (chunk) => {
        const bytes = Uint8Array.from(chunk);
        this.written.push(bytes);
        this.beforeWrite?.(bytes);
        if (this.drop?.(bytes)) return;
        await this.bus.write(bytes);
      },
    });
  }

  async close(): Promise<void> {
    if (!this.readable || !this.writable) throw new DOMException("The port is already closed.", "InvalidStateError");
    if (this.readable.locked || this.writable.locked) throw new TypeError("Cannot close a port while a stream is locked");
    this.readable = null;
    this.writable = null;
    this.controller = null;
    this.closes++;
  }

  /** Bytes from the device to the host. */
  feed(bytes: Uint8Array): void {
    try {
      this.controller?.enqueue(bytes);
    } catch {
      // stream already cancelled
    }
  }

  /** The adapter is pulled out: the host's read fails. */
  unplug(): void {
    this.controller?.error(new DOMException("The device has been lost.", "NetworkError"));
    this.controller = null;
  }
}

const mod4096 = (a: number) => ((a % 4096) + 4096) % 4096;

/** How simulatedSo101 builds its arm. */
export interface SimulatedSo101Options {
  /** The arm's hard stops are where this calibration says (actual = present + homing_offset). */
  calibration: So101Calibration;
  /** The servos hold `calibration` (default true); false leaves them at factory settings. */
  hold?: boolean;
  /** Each joint's start, as a fraction of its calibrated range (default 0.5, a little off centre). */
  start?: Partial<Record<(typeof SO101_MOTORS)[number], number>>;
  label?: string;
}

/** A simulated SO-101 on a SerialPort-shaped handle: hard stops from `calibration` (the wrist roll turns freely), each
joint mid-range, and (with `hold`) the servos holding that calibration. */
export function simulatedSo101(clock: Clock, options: SimulatedSo101Options): { sim: SimFeetechBus; port: SimSerialPort } {
  const sim = new SimFeetechBus(clock);
  const port = new SimSerialPort(sim, options.label);
  for (const m of SO101_MOTORS) {
    const c = options.calibration[m];
    const lo = mod4096(c.homing_offset + c.range_min);
    const hi = mod4096(c.homing_offset + c.range_max);
    if (m !== "wrist_roll") sim.setHardStops(c.id, lo, hi);
    const fraction = options.start?.[m];
    const present = fraction === undefined ? Math.round((c.range_min + c.range_max) / 2) + 37 : Math.round(c.range_min + fraction * (c.range_max - c.range_min));
    sim.place(c.id, mod4096(c.homing_offset + present));
    if (options.hold ?? true) {
      const s = sim.servo(c.id);
      s.setReg("Homing_Offset", c.homing_offset);
      s.setReg("Min_Position_Limit", c.range_min);
      s.setReg("Max_Position_Limit", c.range_max);
      s.eeprom.set(s.regs.subarray(0, 40));
      s.refresh();
    }
  }
  return { sim, port };
}
