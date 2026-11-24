/**
 * candleLight / gs_usb CAN adapters over WebUSB, in a page, a Web Worker (Chromium) or Node (through
 * the `usb` package's WebUSB class, see "@makermods/arm-core/node").
 *
 * Ports are `gs_usb:<USB serial>`. The adapter firmware wedges when the host USB-resets it or keeps
 * writing to a bus where nothing ACKs, so this backend never issues a USB reset, bounds every
 * transfer with a timeout, caps the frames awaiting their TX echo (unacknowledged frames are never
 * echoed) and throws a clear error instead of writing more.
 *
 * Wire format (Linux drivers/net/can/usb/gs_usb.c; candleLight_fw): vendor control requests on
 * interface 0 (1 BITTIMING, 2 MODE, 4 BT_CONST), bulk OUT 0x02 / IN 0x81 carrying 20-byte host frames
 * `<echo_id u32, can_id u32, dlc u8, channel u8, flags u8, reserved u8, data[8]>`; received frames
 * have echo_id 0xFFFFFFFF, a transmitted frame comes back with the echo_id it was sent with once it
 * was acknowledged on the bus. Bit timing at an 87.5 % sample point, prop_seg 1.
 */

import { type Clock, RealClock, Mutex, Signal } from "../clock.js";
import { ConnectError, TimeoutError, ValueError } from "../errors.js";
import { CanBackend } from "./base.js";

/** Vendor/product ids the Linux gs_usb driver binds (candleLight and compatibles), as WebUSB filters. */
export const GS_USB_FILTERS: readonly { readonly vendorId: number; readonly productId: number }[] = [
  { vendorId: 0x1d50, productId: 0x606f },
  { vendorId: 0x1209, productId: 0x2323 },
  { vendorId: 0x1cd2, productId: 0x606f },
  { vendorId: 0x16d0, productId: 0x10b8 },
  { vendorId: 0x1209, productId: 0xca01 },
];

/** Vendor control requests on interface 0. */
export const BREQ_BITTIMING = 1;
export const BREQ_MODE = 2;
export const BREQ_BT_CONST = 4;
/** MODE request words: stop or start the CAN channel. */
export const MODE_RESET = 0;
export const MODE_START = 1;
/** Bulk OUT 0x02 and bulk IN 0x81, as WebUSB endpoint numbers. */
export const EP_OUT = 2;
export const EP_IN = 1;
/** The adapter's only interface. */
export const INTERFACE = 0;
/** echo_id of a frame received from the bus (anything else is the echo of a frame sent). */
export const ECHO_RX = 0xffffffff;
/** can_id flags (Linux can.h): extended id, remote request, error frame; and the 29-bit id mask. */
export const CAN_EFF_FLAG = 0x80000000;
export const CAN_RTR_FLAG = 0x40000000;
export const CAN_ERR_FLAG = 0x20000000;
export const CAN_EFF_MASK = 0x1fffffff;
/** The adapter dropped received frames before this one. */
export const FLAG_OVERFLOW = 0x01;
/** Bit timing sample point. */
export const SAMPLE_POINT = 0.875;
/** Frames sent but not yet echoed (the Linux driver's GS_MAX_TX_URBS). */
export const MAX_IN_FLIGHT = 10;
/** A host frame without hardware timestamp. */
export const FRAME_SIZE = 20;
/** The BT_CONST answer (`<10I`). */
export const BT_CONST_SIZE = 40;
/** Bytes asked of each bulk IN transfer (the adapter sends one frame per transfer). */
export const READ_SIZE = 64;
/** Bulk IN transfers kept queued, so a burst of replies is not paced by one round trip each. */
export const READS_IN_FLIGHT = 4;

/** How to give a page, a worker or a Node process access to an adapter. */
export const ACCESS_HINT =
  "In a browser, grant the adapter to this site first (requestAdapter() from a click). On Linux without root, " +
  'allow it with a udev rule such as SUBSYSTEM=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", MODE="0666".';

type TransferStatus = "ok" | "stall" | "babble";

/** A WebUSB control transfer setup. */
export interface UsbControlSetup {
  requestType: "standard" | "class" | "vendor";
  recipient: "device" | "interface" | "endpoint" | "other";
  request: number;
  value: number;
  index: number;
}

/**
 * The parts of WebUSB's USBDevice this transport uses. A browser USBDevice, the `usb` package's
 * WebUSBDevice and FakeGsUsbDevice all fit.
 */
export interface UsbDeviceLike {
  readonly vendorId: number;
  readonly productId: number;
  readonly serialNumber?: string | null | undefined;
  readonly productName?: string | null | undefined;
  readonly manufacturerName?: string | null | undefined;
  readonly opened: boolean;
  readonly configuration: unknown;
  readonly configurations?: readonly { readonly configurationValue: number }[];
  /**
   * True when bulk IN reads end on their own within the I/O timeout (a timed-out read answers ok
   * with no data). close() then waits for them before releasing the interface. WebUSB reads have
   * no timeout; releasing the interface ends them.
   */
  readonly readsTimeOut?: boolean;
  open(): Promise<void>;
  close(): Promise<void>;
  selectConfiguration(configurationValue: number): Promise<void>;
  claimInterface(interfaceNumber: number): Promise<void>;
  releaseInterface(interfaceNumber: number): Promise<void>;
  controlTransferIn(setup: UsbControlSetup, length: number): Promise<{ status: TransferStatus; data?: DataView | undefined }>;
  controlTransferOut(setup: UsbControlSetup, data?: BufferSource): Promise<{ status: TransferStatus; bytesWritten: number }>;
  transferIn(endpointNumber: number, length: number): Promise<{ status: TransferStatus; data?: DataView | undefined }>;
  transferOut(endpointNumber: number, data: BufferSource): Promise<{ status: TransferStatus; bytesWritten: number }>;
}

/** Where warnings and errors go (default: the console). */
export interface TransportLogger {
  warn(message: string, error?: unknown): void;
  error(message: string, error?: unknown): void;
}

// ---------- Ports and selection ----------

/** True for the vendor/product pairs in GS_USB_FILTERS. */
export function isGsUsbDevice(device: { vendorId: number; productId: number }): boolean {
  return GS_USB_FILTERS.some((f) => f.vendorId === device.vendorId && f.productId === device.productId);
}

/** Python's repr() of a string, for error messages that match the Python SDK's. */
function pyRepr(s: string): string {
  const quote = s.includes("'") && !s.includes('"') ? '"' : "'";
  let out = "";
  for (const c of s) {
    if (c === "\\") out += "\\\\";
    else if (c === quote) out += "\\" + c;
    else if (c === "\t") out += "\\t";
    else if (c === "\n") out += "\\n";
    else if (c === "\r") out += "\\r";
    else out += c;
  }
  return quote + out + quote;
}

/** The USB serial a port names (`gs_usb:<serial>` or a bare serial), or null for "the only one". */
export function parsePort(port: string | null | undefined): string | null {
  if (port === null || port === undefined || port === "" || port === "gs_usb" || port === "gs_usb:") return null;
  const serial = port.startsWith("gs_usb:") ? port.slice("gs_usb:".length) : port;
  if (!serial || /\s/.test(serial)) throw new ValueError(`expected gs_usb:<USB serial>, got ${pyRepr(port)}`);
  return serial;
}

/** `gs_usb:<serial>`, or `gs_usb` for a device without a serial number. */
export function adapterPort(device: { serialNumber?: string | null | undefined }): string {
  return device.serialNumber ? `gs_usb:${device.serialNumber}` : "gs_usb";
}

/**
 * The adapter a port names among `devices` (others than gs_usb adapters are ignored): with no
 * serial exactly one adapter must be present, otherwise the one with that USB serial.
 */
export function selectAdapter<D extends { vendorId: number; productId: number; serialNumber?: string | null | undefined }>(devices: readonly D[], port?: string | null): D {
  const serial = parsePort(port);
  const adapters = devices.filter(isGsUsbDevice);
  const matches = adapters.filter((d) => serial === null || d.serialNumber === serial);
  if (matches.length === 1) return matches[0]!;
  if (serial === null) {
    throw new ConnectError(
      `expected exactly one gs_usb adapter, found ${matches.length}; name one as gs_usb:<serial> (see the ports listing)`,
    );
  }
  throw new ConnectError(
    `no gs_usb adapter with serial ${pyRepr(serial)} (found ${adapters.length} adapter(s)); ` +
      `check the cable and USB permissions. ${ACCESS_HINT}`,
  );
}

// ---------- Browser helpers (page or Web Worker) ----------

interface UsbApi {
  getDevices(): Promise<USBDevice[]>;
  requestDevice?(options: USBDeviceRequestOptions): Promise<USBDevice>;
}

function usbApi(): UsbApi | null {
  const nav = (globalThis as { navigator?: { usb?: UsbApi } }).navigator;
  return nav?.usb ?? null;
}

const NO_WEBUSB =
  "WebUSB is not available here: use a Chromium-based browser (Chrome, Edge) on https or localhost, " +
  "or @makermods/arm-core/node in Node";

/**
 * Shows the browser's device chooser for gs_usb adapters. Must be called on the page from a user
 * gesture (a click). Resolves to the chosen adapter, or null when the chooser was dismissed. Once
 * granted, the adapter is also listed by grantedAdapters(), in the page and in its workers.
 */
export async function requestAdapter(): Promise<USBDevice | null> {
  const usb = usbApi();
  if (!usb) throw new ConnectError(NO_WEBUSB);
  if (typeof usb.requestDevice !== "function") {
    throw new ConnectError("requestAdapter() needs the page (a Web Worker cannot show the chooser); use grantedAdapters() in the worker");
  }
  try {
    return await usb.requestDevice({ filters: GS_USB_FILTERS.map((f) => ({ ...f })) });
  } catch (e) {
    if ((e as { name?: string } | null)?.name === "NotFoundError") return null;
    throw e;
  }
}

/** The connected gs_usb adapters this site was already granted (works in a page and in a Web Worker). */
export async function grantedAdapters(): Promise<USBDevice[]> {
  const usb = usbApi();
  if (!usb) throw new ConnectError(NO_WEBUSB);
  return (await usb.getDevices()).filter(isGsUsbDevice);
}

// ---------- Bit timing and frames ----------

/** Python's round(): half to even. */
function pyRound(x: number): number {
  const r = Math.round(x);
  return Math.abs(x % 1) === 0.5 && r % 2 !== 0 ? r - 1 : r;
}

/** The adapter's BT_CONST answer (`<10I`). */
export interface BtConst {
  feature: number;
  fclk: number;
  tseg1Min: number;
  tseg1Max: number;
  tseg2Min: number;
  tseg2Max: number;
  sjwMax: number;
  brpMin: number;
  brpMax: number;
  brpInc: number;
}

/** Parses BT_CONST; throws when the answer is shorter than 40 bytes. */
export function parseBtConst(bytes: Uint8Array): BtConst {
  if (bytes.length < BT_CONST_SIZE) throw new Error(`BT_CONST answered ${bytes.length} bytes, expected ${BT_CONST_SIZE}`);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const u = (i: number) => view.getUint32(4 * i, true);
  return {
    feature: u(0), fclk: u(1), tseg1Min: u(2), tseg1Max: u(3), tseg2Min: u(4),
    tseg2Max: u(5), sjwMax: u(6), brpMin: u(7), brpMax: u(8), brpInc: u(9),
  };
}

/**
 * [propSeg, phaseSeg1, phaseSeg2, sjw, brp] for `bitrate` at an 87.5 % sample point, using the most
 * time quanta the adapter allows. Throws ConnectError when the clock cannot produce the bitrate.
 */
export function bitTiming(
  fclk: number, bitrate: number, tseg1Min: number, tseg1Max: number, tseg2Min: number,
  tseg2Max: number, sjwMax: number, brpMin: number, brpMax: number, brpInc: number,
): [number, number, number, number, number] {
  void sjwMax;
  const step = Math.max(1, brpInc);
  for (let brp = Math.max(1, brpMin); brp < brpMax + 1; brp += step) {
    const quanta = Math.floor(fclk / (brp * bitrate));
    const rest = fclk - quanta * brp * bitrate;
    if (rest) continue;
    const tseg2 = Math.max(tseg2Min, pyRound(quanta * (1 - SAMPLE_POINT)));
    const tseg1 = quanta - 1 - tseg2;
    if (tseg2 <= tseg2Max && tseg1Min <= tseg1 && tseg1 <= tseg1Max && tseg1 >= 2) return [1, tseg1 - 1, tseg2, 1, brp];
  }
  throw new ConnectError(`adapter clock ${fclk} Hz cannot produce ${bitrate} bit/s`);
}

function u32s(...values: number[]): Uint8Array<ArrayBuffer> {
  const out = new Uint8Array(4 * values.length);
  const view = new DataView(out.buffer);
  values.forEach((v, i) => view.setUint32(4 * i, v >>> 0, true));
  return out;
}

/** The 20-byte host frame: echo_id, can_id (flags included), dlc, channel 0, flags 0, reserved, data[8]. */
export function encodeFrame(echoId: number, ident: number, data: Uint8Array): Uint8Array<ArrayBuffer> {
  const out = new Uint8Array(FRAME_SIZE);
  const view = new DataView(out.buffer);
  view.setUint32(0, echoId >>> 0, true);
  view.setUint32(4, ident >>> 0, true);
  out[8] = data.length;
  out.set(data, 12);
  return out;
}

const viewBytes = (data: DataView | undefined) =>
  data ? new Uint8Array(data.buffer, data.byteOffset, data.byteLength) : new Uint8Array(0);

const reason = (e: unknown) => (e instanceof Error ? e.message : String(e));

const vendorSetup = (request: number): UsbControlSetup => ({
  requestType: "vendor", recipient: "interface", request, value: 0, index: INTERFACE,
});

// ---------- Backend ----------

/** GsUsbBackend settings. */
export interface GsUsbBackendOptions {
  /** CAN bitrate in bit/s (default 1 000 000). */
  bitrate?: number;
  /** Bound on every USB transfer and on the wait for a free echo slot, in seconds (default 0.1). */
  ioTimeout?: number;
  clock?: Clock;
  logger?: TransportLogger;
}

type InResult = { status: TransferStatus; data?: DataView | undefined };
type OutResult = { status: TransferStatus; bytesWritten: number };

/** An open channel: what send() writes to. A send accepted in one session never writes in another. */
interface Session {
  readonly dev: UsbDeviceLike;
  /** Echo ids reserved by this session's frames (written or being written, echo not seen). */
  readonly pending: Set<number>;
  /** This session's bulk OUT transfers that have not ended yet. */
  readonly unresolved: Set<Promise<unknown>>;
}

/** Devices a GsUsbBackend has open (or is opening) in this context: one backend per device. */
const devicesInUse = new WeakSet<object>();

/** A transfer that outlived its time bound (the transfer itself may still complete). */
class BoundExpired extends Error {}

/** A CAN backend on one gs_usb adapter. The device is opened by open() and closed by close(). */
export class GsUsbBackend extends CanBackend {
  readonly device: UsbDeviceLike;
  readonly bitrate: number;
  readonly ioTimeout: number;
  readonly clock: Clock;
  /** Why receiving stopped (a read failure or an unplug), or null. Sends still go out within the echo budget. */
  error: string | null = null;
  /**
   * What went wrong in the last close (or in the cleanup of a failed open): a channel stop or a
   * device close that failed, so the adapter's channel may still be running. Null when clean; reset by open().
   */
  closeError: string | null = null;
  /** Error frames the adapter reported. */
  errorFrames = 0;
  /** Received frames flagged as following frames the adapter dropped. */
  rxOverflows = 0;

  #log: TransportLogger;
  /** The device open() or close() is working on, from open() until close() is done with it. */
  #usb: UsbDeviceLike | null = null;
  /** Published once the channel has started; null when closed. */
  #session: Session | null = null;
  #claimed = false;
  #started = false;
  #running = false;
  #lifecycle = new Mutex();
  #txLock = new Mutex();
  #inFlight = new Signal();
  #nextEcho = 0;
  /** A closed session whose close failed with OUT transfers unresolved, and why the close failed. */
  #leftover: { session: Session; reason: string } | null = null;
  #reads: Promise<InResult>[] = [];
  #rxGen = 0;

  constructor(device: UsbDeviceLike, options: GsUsbBackendOptions = {}) {
    super();
    this.device = device;
    this.bitrate = options.bitrate ?? 1_000_000;
    this.ioTimeout = options.ioTimeout ?? 0.1;
    this.clock = options.clock ?? new RealClock();
    this.#log = options.logger ?? console;
  }

  /** `gs_usb:<USB serial>`. */
  get port(): string {
    return adapterPort(this.device);
  }

  /** Frames written (or still being written) whose echo has not come back yet. */
  get inFlight(): number {
    return this.#session?.pending.size ?? 0;
  }

  /** True between a successful open() and close(). */
  get isOpen(): boolean {
    return this.#session !== null;
  }

  /** A USB transfer bound in time like the Python SDK's (at least 1 ms). WebUSB cannot cancel the transfer itself. */
  #bounded<T>(work: Promise<T>, seconds: number, what: string): Promise<T> {
    const ms = Math.max(1, Math.ceil(seconds * 1000));
    return new Promise<T>((resolve, reject) => {
      const cancel = this.clock.timer(ms / 1000, () => reject(new BoundExpired(`${what} timed out after ${ms} ms`)));
      work.then(
        (v) => (cancel(), resolve(v)),
        (e) => (cancel(), reject(e)),
      );
    });
  }

  async #control(dev: UsbDeviceLike, request: number, data: Uint8Array<ArrayBuffer>, what: string): Promise<void> {
    const result = await this.#bounded(dev.controlTransferOut(vendorSetup(request), data), this.ioTimeout, what);
    if (result.status !== "ok") throw new Error(`${what} failed (${result.status})`);
  }

  /** Opens the device, claims it, sets the bit timing and starts the channel. Serialised with close(). */
  open(): Promise<void> {
    return this.#lifecycle.run(() => this.#open());
  }

  async #open(): Promise<void> {
    if (this.#session) return;
    const dev = this.device;
    if (devicesInUse.has(dev)) throw new ConnectError(`${this.port} is already open in another GsUsbBackend; close that one first`);
    await this.#awaitLeftover();
    devicesInUse.add(dev);
    this.#usb = dev;
    this.closeError = null;
    try {
      if (!dev.opened) await dev.open();
      if (dev.configuration === null || dev.configuration === undefined) {
        await dev.selectConfiguration(dev.configurations?.[0]?.configurationValue ?? 1);
      }
      try {
        await dev.claimInterface(INTERFACE);
      } catch (e) {
        throw new Error(
          `cannot claim the adapter's interface (${reason(e)}): another program or tab is using it, or on Linux the ` +
            "kernel gs_usb driver holds it (a browser cannot detach it: run `sudo modprobe -r gs_usb` or blacklist the module)",
          { cause: e },
        );
      }
      this.#claimed = true;
      const reply = await this.#bounded(dev.controlTransferIn(vendorSetup(BREQ_BT_CONST), BT_CONST_SIZE), this.ioTimeout, "BT_CONST");
      if (reply.status !== "ok") throw new Error(`BT_CONST failed (${reply.status})`);
      const c = parseBtConst(viewBytes(reply.data));
      const timing = bitTiming(c.fclk, this.bitrate, c.tseg1Min, c.tseg1Max, c.tseg2Min, c.tseg2Max, c.sjwMax, c.brpMin, c.brpMax, c.brpInc);
      await this.#control(dev, BREQ_BITTIMING, u32s(...timing), "BITTIMING");
      this.#started = true; // stop on close even if the start transfer fails partway
      // Channel start only; never a USB reset (it wedges these adapters).
      await this.#control(dev, BREQ_MODE, u32s(MODE_START, 0), "MODE start");
    } catch (e) {
      await this.#shutdownUsb();
      if (e instanceof ConnectError) throw e;
      throw new ConnectError(
        `cannot open ${this.port}: ${reason(e)}. Close other CAN software; unplug and replug the adapter if it ` +
          `stopped answering. ${ACCESS_HINT}`,
        { cause: e },
      );
    }
    this.error = null;
    this.#running = true;
    const session: Session = { dev, pending: new Set(), unresolved: new Set() };
    void this.#receive(session, ++this.#rxGen);
    this.#session = session;
  }

  /**
   * Stops receiving, stops the channel (MODE reset), releases the interface and closes the device.
   * Never throws for a failed stop or close: see closeError. Idempotent; serialised with open().
   */
  close(): Promise<void> {
    return this.#lifecycle.run(async () => {
      const session = this.#session;
      this.#session = null;
      this.#running = false;
      this.#rxGen++;
      this.#inFlight.notify();
      try {
        await this.#shutdownUsb();
      } finally {
        // A clean close ended every transfer. After a failed one, OUT transfers may still be
        // queued at the adapter: the next open() waits for them before it admits new frames.
        if (session && this.closeError !== null && session.unresolved.size > 0) {
          this.#leftover = { session, reason: this.closeError };
        }
        this.#inFlight.notify();
      }
    });
  }

  /** Waits (at most 1 s + ioTimeout) for OUT transfers a failed close left unresolved. */
  async #awaitLeftover(): Promise<void> {
    const leftover = this.#leftover;
    if (!leftover) return;
    const { unresolved } = leftover.session;
    if (unresolved.size) {
      let cancel = () => {};
      await Promise.race([
        Promise.allSettled([...unresolved]),
        new Promise<void>((resolve) => (cancel = this.clock.timer(1.0 + this.ioTimeout, resolve))),
      ]);
      cancel();
    }
    if (unresolved.size) {
      throw new ConnectError(
        `cannot open ${this.port}: ${unresolved.size} USB write(s) from the previous session have not ended ` +
          `(${leftover.reason}); unplug and replug the adapter`,
      );
    }
    this.#leftover = null;
  }

  #closeProblem(message: string, error: unknown): void {
    this.closeError = this.closeError ? `${this.closeError}; ${message}` : message;
    this.#log.warn(`${message}; the adapter's channel may not have stopped cleanly`, error);
  }

  async #shutdownUsb(): Promise<void> {
    const dev = this.#usb;
    this.#usb = null;
    if (dev === null) return;
    try {
      await this.#shutdownDevice(dev);
    } finally {
      devicesInUse.delete(dev);
    }
  }

  async #shutdownDevice(dev: UsbDeviceLike): Promise<void> {
    const readsTimeOut = dev.readsTimeOut === true;
    // Reads that time out on their own end within ioTimeout (the Python receive thread's join).
    if (readsTimeOut) await this.#settleReads();
    try {
      if (this.#started) await this.#control(dev, BREQ_MODE, u32s(MODE_RESET, 0), "MODE reset");
    } catch (e) {
      this.#closeProblem(`gs_usb channel stop failed on ${this.port}: ${reason(e)}`, e);
    }
    this.#started = false;
    if (this.#claimed) {
      try {
        await dev.releaseInterface(INTERFACE);
      } catch {
        // already released (an unplugged or closed device)
      }
      this.#claimed = false;
    }
    // WebUSB reads have no timeout: releasing the interface (or closing the device) ends them.
    if (!readsTimeOut) await this.#settleReads();
    try {
      await dev.close();
    } catch (e) {
      // An unplugged device cannot be closed and needs nothing more.
      if (dev.opened) this.#closeProblem(`gs_usb device close failed on ${this.port}: ${reason(e)}`, e);
    }
  }

  /** Waits, at most 1 s + ioTimeout, for the queued bulk IN transfers to end. */
  async #settleReads(): Promise<void> {
    const reads = this.#reads;
    this.#reads = [];
    if (!reads.length) return;
    let cancel = () => {};
    await Promise.race([
      Promise.allSettled(reads),
      new Promise<void>((resolve) => (cancel = this.clock.timer(1.0 + this.ioTimeout, resolve))),
    ]);
    cancel();
  }

  async #receive(session: Session, gen: number): Promise<void> {
    const dev = session.dev;
    const reads = this.#reads;
    const current = () => this.#running && this.#rxGen === gen;
    while (current()) {
      while (reads.length < READS_IN_FLIGHT) {
        let read: Promise<InResult>;
        try {
          read = dev.transferIn(EP_IN, READ_SIZE);
        } catch (e) {
          read = Promise.reject(e);
        }
        read.catch(() => undefined); // awaited in order below
        reads.push(read);
      }
      let result: InResult;
      try {
        result = await reads[0]!;
      } catch (e) {
        if (current()) this.#fail(`USB read failed: ${reason(e)}`, e);
        break;
      }
      reads.shift();
      if (!current()) break;
      if (result.status !== "ok") {
        this.#fail(`USB read failed (${result.status})`);
        break;
      }
      try {
        this.#handle(session, viewBytes(result.data));
      } catch (e) {
        this.#log.error("gs_usb RX frame-processing exception (continuing to receive frames)", e);
      }
    }
    if (this.#rxGen === gen) this.#running = false;
    this.#inFlight.notify();
  }

  #fail(message: string, cause?: unknown): void {
    this.error = message;
    this.#log.error(`${this.port}: ${message}; receive stopped (unplug and replug the adapter)`, cause);
  }

  /** Processes one bulk IN transfer: echoes free their slot, the rest is filtered and delivered. */
  #handle(session: Session, raw: Uint8Array): void {
    if (raw.length < FRAME_SIZE) return;
    const view = new DataView(raw.buffer, raw.byteOffset, FRAME_SIZE);
    const echo = view.getUint32(0, true);
    const ident = view.getUint32(4, true);
    const dlc = raw[8]!;
    const channel = raw[9]!;
    const flags = raw[10]!;
    if (echo !== ECHO_RX) {
      this.#forget(session, echo); // our frame was acknowledged on the bus
      return;
    }
    if (flags & FLAG_OVERFLOW) this.rxOverflows += 1;
    if (channel !== 0 || flags & ~FLAG_OVERFLOW || ident & CAN_RTR_FLAG) return;
    if (ident & CAN_ERR_FLAG) {
      this.errorFrames += 1;
      return;
    }
    const cb = this.cb;
    if (cb !== null) {
      const extended = (ident & CAN_EFF_FLAG) !== 0;
      cb(ident & (extended ? CAN_EFF_MASK : 0x7ff), raw.slice(12, 12 + Math.min(dlc, 8)), extended);
    }
  }

  #forget(session: Session, echo: number): void {
    session.pending.delete(echo);
    this.#inFlight.notify();
  }

  #notOpen(): ConnectError {
    return new ConnectError(`${this.port} is not open` + (this.error ? ` (${this.error})` : ""));
  }

  #capReached(session: Session): TimeoutError {
    return new TimeoutError(
      `${this.port}: ${session.pending.size} frames were never acknowledged on the CAN bus; ` +
        "stopped writing to protect the adapter. Check that the motors are powered and " +
        "the CAN wiring and bitrate are right, then reconnect" +
        (this.error ? ` (receive had stopped: ${this.error})` : ""),
    );
  }

  /**
   * Writes one frame. Refused only when the backend is not open: after a receive failure frames
   * still go out (a torque-off must reach the bus) until MAX_IN_FLIGHT of them await an echo.
   */
  async send(canId: number, data: Uint8Array, extended = true): Promise<void> {
    const payload = Uint8Array.from(data);
    if (payload.length > 8) throw new ValueError("classic CAN carries at most 8 data bytes");
    const session = this.#session;
    if (session === null) throw this.#notOpen();
    const ident = extended ? ((canId & CAN_EFF_MASK) | CAN_EFF_FLAG) >>> 0 : canId & 0x7ff;
    const deadline = this.clock.now() + this.ioTimeout;
    await this.#txLock.run(async () => {
      for (;;) {
        if (this.#session !== session) throw this.#notOpen();
        if (session.pending.size < MAX_IN_FLIGHT) break;
        // With receive stopped no echo can free a slot.
        if (deadline - this.clock.now() <= 0 || this.error !== null) throw this.#capReached(session);
        await this.#inFlight.wait(
          () => session.pending.size < MAX_IN_FLIGHT || this.#session !== session || this.error !== null,
          deadline - this.clock.now(),
          this.clock,
        );
      }
      const dev = session.dev;
      // The next echo id not still reserved (an id whose echo was lost stays reserved).
      let echo = this.#nextEcho;
      while (session.pending.has(echo) || this.#leftover?.session.pending.has(echo)) echo = (echo + 1) % 0x10000;
      this.#nextEcho = (echo + 1) % 0x10000;
      session.pending.add(echo);
      const packet = encodeFrame(echo, ident, payload);
      let transfer: Promise<OutResult>;
      try {
        transfer = dev.transferOut(EP_OUT, packet);
      } catch (e) {
        transfer = Promise.reject(e);
      }
      // The slot stays reserved until the echo comes back, or the transfer itself fails; a time
      // bound that expires first does not free it, since the transfer may still be queued.
      const outcome = transfer.then(
        (r) => {
          if (r.status !== "ok" || r.bytesWritten !== packet.length) this.#forget(session, echo);
          return r;
        },
        (e: unknown) => {
          this.#forget(session, echo);
          throw e;
        },
      );
      outcome.catch(() => undefined);
      session.unresolved.add(outcome);
      void outcome.then(
        () => session.unresolved.delete(outcome),
        () => session.unresolved.delete(outcome),
      );
      let written: OutResult;
      try {
        written = await this.#bounded(outcome, Math.max(0, deadline - this.clock.now()), "USB write");
      } catch (e) {
        throw new TimeoutError(`${this.port}: USB write failed (${reason(e)}); the adapter may need unplugging and replugging`, { cause: e });
      }
      if (written.status !== "ok") {
        throw new TimeoutError(`${this.port}: USB write failed (${written.status}); the adapter may need unplugging and replugging`);
      }
      if (written.bytesWritten !== packet.length) {
        throw new TimeoutError(`${this.port}: incomplete USB write (${written.bytesWritten}/${packet.length} bytes)`);
      }
    });
  }
}
