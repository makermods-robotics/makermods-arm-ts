/** The transaction layer: a Web Serial port (or anything SerialPort-shaped) opened at 1 Mbaud, the UART that buffers
what it reads, the Feetech bus that turns a request into its replies, and the per-port Arbiter that serializes
transactions. Lifted from the console (hardware/feetech.ts FeetechBus, fashionstar.ts FedUart, arbiter.ts,
identify.ts openPortLink, so101.ts openSo101Link).

Getting a port: requestPort() shows the browser's chooser filtered to the CH343 adapter (call it from a user
gesture); grantedPorts() lists the CH343 ports this origin was already granted. */

import type { Clock } from "@makermods/arm-core";
import { ArmError } from "@makermods/arm-core";
import { msClock, type MsClock } from "./clock-ms.js";
import {
  EEPROM_END,
  EEPROM_REPLY_TIMEOUT_MS,
  FEETECH_BAUDRATE,
  FeetechError,
  FeetechServoError,
  INST_ACTION,
  INST_READ,
  INST_REG_WRITE,
  INST_SYNC_READ,
  INST_SYNC_WRITE,
  INST_WRITE,
  BROADCAST_ID,
  READ_SLICE_MS,
  REG,
  REPLY_TIMEOUT_MS,
  STATE_ADDRESS,
  STATE_LENGTH,
  bytesOf,
  decodeValue,
  encodeValue,
  packet,
  parseStatus,
  pingPacket,
  readPacket,
  syncReadPacket,
  syncWritePacket,
  valueOf,
  writePacket,
  type RegName,
  type ServoState,
  type StatusPacket,
} from "./protocol.js";

// ---------- Getting a port ----------

/** The SO-101's USB serial adapter: WCH CH343 (1a86:55d3). */
export const CH343_FILTER = { usbVendorId: 0x1a86, usbProductId: 0x55d3 } as const;

/** The parts of a Web Serial SerialPort this package uses. */
export interface SerialPortLike {
  open(options: { baudRate: number }): Promise<void>;
  readonly readable: ReadableStream<Uint8Array> | null;
  readonly writable: WritableStream<Uint8Array> | null;
  close(): Promise<void>;
  getInfo?(): { usbVendorId?: number; usbProductId?: number };
}

/** The parts of navigator.serial requestPort / grantedPorts use. */
export interface SerialLike {
  requestPort(options?: { filters?: Array<{ usbVendorId?: number; usbProductId?: number }> }): Promise<SerialPort>;
  getPorts(): Promise<SerialPort[]>;
}

function navigatorSerial(): SerialLike {
  const serial = (globalThis as { navigator?: { serial?: SerialLike } }).navigator?.serial;
  if (!serial) throw new ArmError("Web Serial is not available here (it needs Chrome or Edge, on a secure page)");
  return serial;
}

/** Shows the browser's port chooser filtered to the CH343 adapter. Must be called from a user gesture (a click); in a
Web Worker, use grantedPorts(). Resolves to null when the chooser is dismissed. */
export async function requestPort(serial: SerialLike = navigatorSerial()): Promise<SerialPort | null> {
  try {
    return await serial.requestPort({ filters: [{ ...CH343_FILTER }] });
  } catch (exc) {
    if ((exc as { name?: string }).name === "NotFoundError") return null;
    throw exc;
  }
}

/** The CH343 ports this origin was already granted (no prompt; works in a Web Worker). */
export async function grantedPorts(serial: SerialLike = navigatorSerial()): Promise<SerialPort[]> {
  const ports = await serial.getPorts();
  return ports.filter((port) => {
    const info = port.getInfo();
    return info.usbVendorId === CH343_FILTER.usbVendorId && info.usbProductId === CH343_FILTER.usbProductId;
  });
}

/** A short name for a port in messages: its USB ids when it reports them. */
export function portLabel(port: SerialPortLike): string {
  const info = port.getInfo?.();
  if (info?.usbVendorId === undefined || info.usbProductId === undefined) return "serial port";
  const hex = (n: number) => n.toString(16).toUpperCase().padStart(4, "0");
  return `usb:${hex(info.usbVendorId)}:${hex(info.usbProductId)}`;
}

// ---------- Port plumbing ----------

/** An open port's writer, what it reads, and close(). */
export interface PortLink {
  readonly writer: { write(bytes: Uint8Array): Promise<void> };
  /** What the port reads, until close; a read error (unplug) is thrown from the iterator. */
  readonly chunks: AsyncIterable<Uint8Array>;
  close(): Promise<void>;
}

/** Opens the port and takes its reader and writer. close() cancels the read, releases both locks and closes the
port (SerialPort.close refuses while a stream is locked). */
export async function openPortLink(port: SerialPortLike, baudRate: number): Promise<PortLink> {
  await port.open({ baudRate });
  if (!port.readable || !port.writable) {
    await port.close().catch(() => undefined);
    throw new ArmError("The port opened without streams");
  }
  const reader = port.readable.getReader();
  const writer = port.writable.getWriter();
  let closing: Promise<void> | null = null;
  async function* chunks(): AsyncGenerator<Uint8Array> {
    for (;;) {
      const { value, done } = await reader.read();
      if (done) return;
      if (value?.length) yield value;
    }
  }
  return {
    writer: { write: (bytes) => writer.write(bytes) },
    chunks: chunks(),
    close() {
      closing ??= (async () => {
        await reader.cancel().catch(() => undefined);
        reader.releaseLock();
        writer.releaseLock();
        await port.close();
      })();
      return closing;
    },
  };
}

// ---------- UART ----------

/** A byte pipe with a timed read: what FeetechBus talks to. */
export interface UartPort {
  write(bytes: Uint8Array): Promise<void>;
  read(timeoutMs: number): Promise<Uint8Array>;
  clear(): void;
}

/** A UartPort over a byte stream that is fed as data arrives (the reader loop calls `feed`). */
export class FedUart implements UartPort {
  private buffer: number[] = [];
  private waiter: (() => void) | null = null;

  constructor(
    private readonly writer: { write(bytes: Uint8Array): Promise<void> },
    private readonly clock: MsClock,
  ) {}

  feed(bytes: Uint8Array): void {
    this.buffer.push(...bytes);
    const w = this.waiter;
    this.waiter = null;
    w?.();
  }

  write(bytes: Uint8Array): Promise<void> {
    return this.writer.write(bytes);
  }

  clear(): void {
    this.buffer = [];
  }

  async read(timeoutMs: number): Promise<Uint8Array> {
    if (!this.buffer.length) {
      const abort = new AbortController();
      await Promise.race([new Promise<void>((resolve) => (this.waiter = resolve)), this.clock.sleep(timeoutMs, abort.signal)]);
      abort.abort();
      this.waiter = null;
    }
    const out = Uint8Array.from(this.buffer);
    this.buffer = [];
    return out;
  }
}

// ---------- Arbiter ----------

/** A transaction dropped because a stop moved the arbiter's epoch on after it was queued. */
export class StaleEpochError extends ArmError {}

/** The arbiter refuses work once its port closed. */
export class ArbiterClosedError extends ArmError {}

/** How a transaction is queued. */
export interface TransactionOptions {
  /** Stop/disable: dispatched before any normal transaction already queued. */
  priority?: boolean;
  /** Dropped (StaleEpochError) if the arbiter's epoch differs when its turn comes. */
  epoch?: number;
  /** For diagnostics only. */
  label?: string;
}

interface Entry {
  run: () => Promise<unknown>;
  resolve: (value: unknown) => void;
  reject: (error: unknown) => void;
  options: TransactionOptions;
}

/** Per-port transaction queue: one transaction (a request and its reply, or a short sequence that must not
interleave) at a time. Stop/disable transactions jump ahead of everything queued, though never into the middle of
the one in flight. A transaction queued with an epoch is dropped at dispatch if the epoch moved on, so once a stop
has bumped it an old target can never be written. */
export class Arbiter {
  private queue: Entry[] = [];
  private busy = false;
  private current: Promise<unknown> | null = null;
  private closedReason: string | null = null;
  private epochValue = 0;
  /** Transactions dropped at dispatch because their epoch was stale. */
  dropped = 0;

  get epoch(): number {
    return this.epochValue;
  }

  /** A stop: every queued transaction tagged with an older epoch will be dropped. */
  bumpEpoch(): number {
    return ++this.epochValue;
  }

  /** Inside a transaction, right before each write: a stop that landed while it was running ends it here, so no
  frame of the old command goes out after the stop. */
  check(epoch: number | undefined, label = "write"): void {
    if (epoch !== undefined && epoch !== this.epochValue) throw new StaleEpochError(`Dropped ${label} from epoch ${epoch}`);
  }

  /** Resolves once the transaction in flight now (if any) has finished. */
  idle(): Promise<void> {
    const current = this.current;
    return current ? current.then(() => undefined, () => undefined) : Promise.resolve();
  }

  get pending(): number {
    return this.queue.length + (this.busy ? 1 : 0);
  }

  run<T>(fn: () => Promise<T>, options: TransactionOptions = {}): Promise<T> {
    if (this.closedReason) return Promise.reject(new ArbiterClosedError(this.closedReason));
    return new Promise<T>((resolve, reject) => {
      const entry: Entry = { run: fn, resolve: resolve as (v: unknown) => void, reject, options };
      if (options.priority) {
        // After other priority entries already waiting, ahead of every normal one.
        const index = this.queue.findIndex((e) => !e.options.priority);
        this.queue.splice(index < 0 ? this.queue.length : index, 0, entry);
      } else {
        this.queue.push(entry);
      }
      void this.dispatch();
    });
  }

  /** Refuse new work and fail everything still queued; the transaction in flight finishes. */
  close(reason = "Port closed"): void {
    this.closedReason = reason;
    const queued = this.queue;
    this.queue = [];
    for (const entry of queued) entry.reject(new ArbiterClosedError(reason));
  }

  private async dispatch(): Promise<void> {
    if (this.busy) return;
    this.busy = true;
    try {
      for (let entry = this.queue.shift(); entry; entry = this.queue.shift()) {
        const { epoch } = entry.options;
        if (epoch !== undefined && epoch !== this.epochValue) {
          this.dropped++;
          entry.reject(new StaleEpochError(`Dropped ${entry.options.label ?? "transaction"} from epoch ${epoch}`));
          continue;
        }
        let running: Promise<unknown>;
        try {
          running = entry.run();
        } catch (exc) {
          running = Promise.reject(exc);
        }
        this.current = running;
        try {
          entry.resolve(await running);
        } catch (exc) {
          entry.reject(exc);
        } finally {
          this.current = null;
        }
      }
    } finally {
      this.busy = false;
    }
  }
}

// ---------- Feetech bus ----------

/** One Feetech bus on one UART. Every method is one exchange; the caller serializes them (the port's Arbiter). */
export class FeetechBus {
  /** Exchanges that timed out, for diagnostics. */
  timeouts = 0;
  /** Each servo's error byte in its last reply (0: none), sync reads included. */
  readonly lastErrors = new Map<number, number>();

  constructor(
    private readonly port: UartPort,
    private readonly clock: MsClock,
    readonly timeoutMs = REPLY_TIMEOUT_MS,
  ) {}

  /** Writes `request` and reads until `want` status packets parsed or the window ends (the error bytes unchecked). */
  async request(request: Uint8Array, want: number, timeoutMs = this.timeoutMs): Promise<StatusPacket[]> {
    const instr = request[4];
    // Torque_Enable 128 (the current position becomes the middle) also saves a homing offset before answering.
    const recentre = instr === INST_WRITE && request[5] === REG.Torque_Enable[0] && request[6] === 128;
    const eeprom = ((instr === INST_WRITE || instr === INST_REG_WRITE) && (request[5] as number) < EEPROM_END) || recentre;
    const packets = await this.exchange(request, want, eeprom ? Math.max(timeoutMs, EEPROM_REPLY_TIMEOUT_MS) : timeoutMs);
    for (const p of packets) this.lastErrors.set(p.id, p.error);
    return packets;
  }

  private checked(reply: StatusPacket, what: string): StatusPacket {
    if (reply.error) throw new FeetechServoError(reply.id, reply.error, what);
    return reply;
  }

  private async exchange(request: Uint8Array, want: number, timeoutMs = this.timeoutMs): Promise<StatusPacket[]> {
    this.port.clear();
    await this.port.write(request);
    if (!want) return [];
    const started = this.clock.now();
    let buffer = new Uint8Array(0);
    let packets: StatusPacket[] = [];
    while (this.clock.now() - started < timeoutMs) {
      const chunk = await this.port.read(Math.min(READ_SLICE_MS, Math.max(1, timeoutMs - (this.clock.now() - started))));
      if (!chunk.length) continue;
      const next = new Uint8Array(buffer.length + chunk.length);
      next.set(buffer);
      next.set(chunk, buffer.length);
      buffer = next;
      // The request itself, if the adapter echoes it (half-duplex boards without echo cancellation), parses as a
      // packet from the target id: dropped by matching the instruction byte. Only replies this request can have: a
      // late reply to an earlier one (an EEPROM write that answered past its window) is not taken for this one's.
      packets = parseStatus(buffer).packets.filter((p) => !isEcho(p, request) && answers(p, request));
      if (packets.length >= want) break;
    }
    if (packets.length < want) this.timeouts += 1;
    return packets;
  }

  async ping(id: number, timeoutMs = this.timeoutMs): Promise<boolean> {
    const replies = await this.request(pingPacket(id), 1, timeoutMs);
    return replies.some((p) => p.id === id);
  }

  /** One register of one servo, decoded. A reply with its error byte set throws FeetechServoError. */
  async read(id: number, reg: RegName, timeoutMs = this.timeoutMs): Promise<number> {
    const [address, length] = REG[reg];
    const reply = (await this.request(readPacket(id, address, length), 1, timeoutMs)).find((p) => p.id === id);
    if (!reply) throw new FeetechError(`No reply from servo ${id} reading ${reg}`);
    this.checked(reply, `reading ${reg}`);
    if (reply.params.length < length) throw new FeetechError(`Short reply from servo ${id} reading ${reg}`);
    return decodeValue(reg, valueOf(reply.params, 0, length));
  }

  /** One register written, with the servo's status reply awaited (Response_Status_Level 1, the STS default). A reply
  with its error byte set throws FeetechServoError (the write may still have landed: read it back to know). */
  async write(id: number, reg: RegName, value: number, timeoutMs = this.timeoutMs): Promise<void> {
    const [address, length] = REG[reg];
    const reply = (await this.request(writePacket(id, address, bytesOf(encodeValue(reg, value), length)), 1, timeoutMs)).find((p) => p.id === id);
    if (!reply) throw new FeetechError(`No reply from servo ${id} writing ${reg}`);
    this.checked(reply, `writing ${reg}`);
  }

  /** One register of every servo in `ids`; a servo that did not answer is missing from the map. */
  async syncRead(ids: readonly number[], reg: RegName, timeoutMs = this.timeoutMs): Promise<Map<number, number>> {
    const [address, length] = REG[reg];
    const replies = await this.request(syncReadPacket(ids, address, length), ids.length, timeoutMs);
    const out = new Map<number, number>();
    for (const p of replies) {
      if (ids.includes(p.id) && p.params.length >= length) out.set(p.id, decodeValue(reg, valueOf(p.params, 0, length)));
    }
    return out;
  }

  /** Every servo's live state in one sync read (Torque_Enable at 40 through Moving at 66, 27 bytes): torque, position,
  velocity, load, voltage, temperature, status and the reply's error byte. A servo that did not answer is missing. */
  async syncReadState(ids: readonly number[], timeoutMs = this.timeoutMs): Promise<Map<number, ServoState>> {
    const replies = await this.request(syncReadPacket(ids, STATE_ADDRESS, STATE_LENGTH), ids.length, timeoutMs);
    const out = new Map<number, ServoState>();
    for (const p of replies) {
      if (!ids.includes(p.id) || p.params.length < STATE_LENGTH) continue;
      const at = (reg: RegName) => decodeValue(reg, valueOf(p.params, REG[reg][0] - STATE_ADDRESS, REG[reg][1]));
      out.set(p.id, {
        torque: at("Torque_Enable"),
        position: at("Present_Position"),
        load: at("Present_Load"),
        voltage: at("Present_Voltage"),
        temperatureC: at("Present_Temperature"),
        status: at("Status"),
        error: p.error,
        velocity: at("Present_Velocity"),
      });
    }
    return out;
  }

  /** The same register on several servos, one broadcast packet, no reply. */
  async syncWrite(reg: RegName, values: ReadonlyArray<readonly [number, number]>): Promise<void> {
    const [address, length] = REG[reg];
    await this.request(syncWritePacket(address, length, values.map(([id, v]) => [id, encodeValue(reg, v)] as const)), 0);
  }

  /** Raw bytes from `address` (several consecutive registers in one exchange). */
  async readBytes(id: number, address: number, length: number, timeoutMs = this.timeoutMs): Promise<Uint8Array> {
    const reply = (await this.request(readPacket(id, address, length), 1, timeoutMs)).find((p) => p.id === id);
    if (!reply || reply.params.length < length) throw new FeetechError(`No reply from servo ${id} reading ${length} bytes at ${address}`);
    return this.checked(reply, `reading ${length} bytes at ${address}`).params.slice(0, length);
  }

  /** Raw bytes written from `address`, the status reply awaited. */
  async writeBytes(id: number, address: number, data: readonly number[], timeoutMs = this.timeoutMs): Promise<void> {
    const reply = (await this.request(writePacket(id, address, data), 1, timeoutMs)).find((p) => p.id === id);
    if (!reply) throw new FeetechError(`No reply from servo ${id} writing ${data.length} bytes at ${address}`);
    this.checked(reply, `writing ${data.length} bytes at ${address}`);
  }

  /** REG_WRITE: staged until ACTION (scservo_sdk regWriteTxRx); the status reply awaited. */
  async regWriteBytes(id: number, address: number, data: readonly number[], timeoutMs = this.timeoutMs): Promise<void> {
    const reply = (await this.request(packet(id, INST_REG_WRITE, [address, ...data]), 1, timeoutMs)).find((p) => p.id === id);
    if (!reply) throw new FeetechError(`No reply from servo ${id} to REG_WRITE at ${address}`);
    this.checked(reply, `REG_WRITE at ${address}`);
  }

  /** ACTION to every servo (broadcast, no reply): staged REG_WRITEs take effect together. */
  async action(): Promise<void> {
    await this.request(packet(BROADCAST_ID, INST_ACTION), 0);
  }

  /** Raw SYNC_WRITE of `length` bytes per servo from `address`, no reply. */
  async syncWriteBytes(address: number, length: number, values: ReadonlyArray<readonly [number, readonly number[]]>): Promise<void> {
    const params = [address, length];
    for (const [id, data] of values) {
      if (data.length !== length) throw new RangeError(`syncWriteBytes: servo ${id} needs ${length} bytes`);
      params.push(id, ...data);
    }
    await this.request(packet(BROADCAST_ID, INST_SYNC_WRITE, params), 0);
  }
}

/** Whether a status packet can be the reply to `request`: from its servo (any, for a sync read of several), with the
parameter bytes that instruction answers with (a READ's length, none for PING / WRITE / REG_WRITE). */
function answers(p: StatusPacket, request: Uint8Array): boolean {
  const id = request[2] as number;
  const instr = request[4] as number;
  if (instr === INST_SYNC_READ) return p.params.length === request[6] && Array.from(request.subarray(7, request.length - 1)).includes(p.id);
  if (p.id !== id) return false;
  if (instr === INST_READ) return p.params.length === request[6];
  return p.params.length === 0;
}

function isEcho(p: StatusPacket, request: Uint8Array): boolean {
  // An echoed instruction packet parses with `error` = the instruction byte and the request's own checksum.
  return request.length === p.params.length + 6 && p.id === request[2] && p.error === request[4];
}

// ---------- One SO-101's bus on a port ----------

/** One SO-101's servos on an open port: the bus, its transaction queue, and close(). */
export interface So101Link {
  readonly bus: FeetechBus;
  readonly arbiter: Arbiter;
  /** The clock the bus runs on (milliseconds). */
  readonly clock: MsClock;
  /** The same clock in seconds, as injected. */
  readonly seconds: Clock;
  close(): Promise<void>;
}

/** Opens `port` at 1 Mbaud with a reader feeding the bus's UART. */
export async function openSo101Link(port: SerialPortLike, clock: Clock): Promise<So101Link> {
  const ms = msClock(clock);
  const link = await openPortLink(port, FEETECH_BAUDRATE);
  const uart = new FedUart(link.writer, ms);
  const reading = (async () => {
    try {
      for await (const chunk of link.chunks) uart.feed(chunk);
    } catch {
      // unplugged: every later exchange times out
    }
  })();
  const arbiter = new Arbiter();
  let closing: Promise<void> | null = null;
  return {
    bus: new FeetechBus(uart, ms),
    arbiter,
    clock: ms,
    seconds: clock,
    close() {
      closing ??= (async () => {
        arbiter.close("Port closed");
        await link.close().catch(() => undefined);
        await reading;
      })();
      return closing;
    },
  };
}
