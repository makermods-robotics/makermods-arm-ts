/**
 * Node serial ports through the optional `serialport` package, shaped like Web Serial ports so the
 * SLCAN backend (and anything else taking a SerialPortLike) runs unchanged. Imported only from
 * "@makermods/arm-core/node", so browser bundles never load `serialport`.
 */

import { boundAfterIo, type Clock, RealClock } from "../clock.js";
import { ConnectError } from "../errors.js";
import type { SerialPortLike } from "./serial.js";
import { SlcanSerialBackend, type SlcanSerialBackendOptions } from "./slcan-serial.js";

/** A serial port as listSerialPorts() describes it (no port is opened to list it). */
export interface SerialPortInfo {
  path: string;
  manufacturer: string;
  serialNumber: string;
  /** USB vendor and product ids, or null for a port that is not on USB. */
  vendorId: number | null;
  productId: number | null;
  /** `vvvv:pppp`, or "" for a port that is not on USB. */
  vidPid: string;
  pnpId: string;
  locationId: string;
}

/** The `serialport` SerialPort members used here. A real SerialPort fits. */
export interface RawSerialPort {
  readonly isOpen: boolean;
  open(callback: (error: Error | null) => void): void;
  write(data: Uint8Array, callback: (error: Error | null | undefined) => void): boolean;
  close(callback: (error: Error | null) => void): void;
  on(event: "data", listener: (chunk: Uint8Array) => void): this;
  on(event: "close", listener: (error?: (Error & { disconnected?: boolean }) | null) => void): this;
  on(event: "error", listener: (error: Error) => void): this;
  off(event: string, listener: (...args: never[]) => void): this;
}

/** The parts of the `serialport` module used here. */
export interface SerialportModule {
  SerialPort: {
    new (options: { path: string; baudRate: number; autoOpen: false; rtscts?: boolean }): unknown;
    list(): Promise<{
      path: string; manufacturer?: string | undefined; serialNumber?: string | undefined; pnpId?: string | undefined;
      locationId?: string | undefined; vendorId?: string | undefined; productId?: string | undefined;
    }[]>;
  };
}

const NEEDS_SERIALPORT = "SLCAN adapters in Node need the optional 'serialport' package (npm install serialport)";

let loading: Promise<SerialportModule> | null = null;

/** The `serialport` package, loaded on first use. Throws ConnectError when it is not installed. */
export function loadSerialport(): Promise<SerialportModule> {
  loading ??= (import("serialport") as Promise<unknown> as Promise<SerialportModule>).catch((e: unknown) => {
    loading = null;
    throw new ConnectError(`${NEEDS_SERIALPORT}: ${e instanceof Error ? e.message : String(e)}`, { cause: e });
  });
  return loading;
}

const parseId = (s: string | undefined): number | null => {
  if (!s || !/^[0-9a-fA-F]{1,4}$/.test(s)) return null;
  return parseInt(s, 16);
};

const hex4 = (n: number) => n.toString(16).padStart(4, "0");

/** Every serial port `serialport` finds, sorted by path. Never opens a port. */
export async function listSerialPorts(): Promise<SerialPortInfo[]> {
  const sp = await loadSerialport();
  const ports = await sp.SerialPort.list();
  return ports
    .map((p) => {
      const vendorId = parseId(p.vendorId), productId = parseId(p.productId);
      return {
        path: p.path,
        manufacturer: p.manufacturer ?? "",
        serialNumber: p.serialNumber ?? "",
        vendorId,
        productId,
        vidPid: vendorId !== null && productId !== null ? `${hex4(vendorId)}:${hex4(productId)}` : "",
        pnpId: p.pnpId ?? "",
        locationId: p.locationId ?? "",
      };
    })
    .sort((a, b) => (a.path < b.path ? -1 : a.path > b.path ? 1 : 0));
}

/**
 * The ports that may be a USB-SLCAN adapter (or a Star leader), as the Python SDK picks them:
 * /dev/ttyACM* and /dev/ttyUSB* on Linux, /dev/cu.usbmodem* on macOS, every port on Windows.
 */
export function slcanCandidates(ports: readonly SerialPortInfo[], platform: string = process.platform): SerialPortInfo[] {
  if (platform === "win32") return [...ports];
  const prefixes = platform === "darwin" ? ["/dev/cu.usbmodem"] : ["/dev/ttyACM", "/dev/ttyUSB"];
  return ports.filter((p) => prefixes.some((prefix) => p.path.startsWith(prefix)));
}

/** NodeSerialPort settings. */
export interface NodeSerialPortOptions {
  /** RTS/CTS hardware flow control (default false). */
  rtscts?: boolean;
  /** USB ids getInfo() reports. */
  info?: { usbVendorId?: number; usbProductId?: number };
  /** Bound on close(), in seconds (default 2.0). */
  closeTimeout?: number;
  clock?: Clock;
}

/**
 * A `serialport` port with the Web Serial SerialPort interface: open({ baudRate }) opens the device,
 * `readable` and `writable` are the port's streams while it is open (a write resolves once
 * `serialport` has written the bytes), close() refuses while a stream is locked, as in a browser.
 * An unplug errors the readable stream and fails every pending and later write (`serialport` itself
 * would hold a write on a closed port until it reopens). close() closes the native port first, so a
 * native write stuck on the device is interrupted, and is bounded by closeTimeout; the port stays
 * owned (open() refuses) until the native close has finished.
 */
export class NodeSerialPort implements SerialPortLike {
  readonly path: string;
  readonly #module: SerialportModule;
  readonly #rtscts: boolean;
  readonly #info: { usbVendorId?: number; usbProductId?: number };
  #raw: RawSerialPort | null = null;
  #readable: ReadableStream<Uint8Array> | null = null;
  #writable: WritableStream<Uint8Array> | null = null;
  #detach: () => void = () => {};
  /** The raw port closed or failed: writes are refused from here on. */
  #dead = false;
  #closing: Promise<void> | null = null;
  /** The sink write waiting on `serialport`, failed by close or a disconnect. */
  #pendingWrite: ((error: Error) => void) | null = null;
  readonly #closeTimeout: number;
  readonly #clock: Clock;

  constructor(path: string, module: SerialportModule, options: NodeSerialPortOptions = {}) {
    this.path = path;
    this.#module = module;
    this.#rtscts = options.rtscts ?? false;
    this.#info = { ...options.info };
    this.#closeTimeout = options.closeTimeout ?? 2.0;
    this.#clock = options.clock ?? new RealClock();
  }

  /** Marks the raw port gone and fails the write waiting on it. */
  #kill(error: Error): void {
    this.#dead = true;
    const fail = this.#pendingWrite;
    this.#pendingWrite = null;
    fail?.(error);
  }

  getInfo(): { usbVendorId?: number; usbProductId?: number } {
    return { ...this.#info };
  }

  get readable(): ReadableStream<Uint8Array> | null {
    return this.#readable;
  }

  get writable(): WritableStream<Uint8Array> | null {
    return this.#writable;
  }

  async open(options: { baudRate: number }): Promise<void> {
    if (this.#raw) throw new Error(`${this.path} is already open`);
    const raw = new this.#module.SerialPort({
      path: this.path, baudRate: options.baudRate, autoOpen: false, rtscts: this.#rtscts,
    }) as RawSerialPort;
    await new Promise<void>((resolve, reject) => raw.open((e) => (e ? reject(e) : resolve())));
    this.#raw = raw;
    this.#dead = false;
    let controller: ReadableStreamDefaultController<Uint8Array> | null = null;
    const onData = (chunk: Uint8Array) => {
      try {
        controller?.enqueue(Uint8Array.from(chunk));
      } catch {
        // the stream was cancelled or errored
      }
    };
    const onClose = (error?: (Error & { disconnected?: boolean }) | null) => {
      this.#kill(error?.disconnected ? new Error(`${this.path} was disconnected`) : new Error(`${this.path} was closed`));
      const c = controller;
      controller = null;
      try {
        if (error) c?.error(error.disconnected ? new Error(`${this.path} was disconnected`) : error);
        else c?.close();
      } catch {
        // already closed
      }
    };
    const onError = (error: Error) => {
      this.#kill(error);
      const c = controller;
      controller = null;
      try {
        c?.error(error);
      } catch {
        // already closed
      }
    };
    raw.on("data", onData);
    raw.on("close", onClose);
    raw.on("error", onError);
    this.#detach = () => {
      raw.off("data", onData);
      raw.off("close", onClose);
      raw.off("error", onError);
    };
    this.#readable = new ReadableStream<Uint8Array>({
      start: (c) => {
        controller = c;
      },
      cancel: () => {
        controller = null;
      },
    });
    this.#writable = new WritableStream<Uint8Array>({
      write: (chunk) =>
        new Promise<void>((resolve, reject) => {
          if (this.#dead || !raw.isOpen) {
            reject(new Error(`${this.path} is closed`));
            return;
          }
          let settled = false;
          const fail = (e: Error) => {
            if (settled) return;
            settled = true;
            this.#pendingWrite = null;
            reject(e);
          };
          this.#pendingWrite = fail;
          raw.write(Uint8Array.from(chunk), (e) => {
            if (e) fail(e);
            else if (!settled) {
              settled = true;
              this.#pendingWrite = null;
              resolve();
            }
          });
        }),
    });
  }

  async close(): Promise<void> {
    const raw = this.#raw;
    if (!raw) throw new Error(`${this.path} is not open`);
    if (this.#closing) return this.#closing;
    if (this.#readable?.locked || this.#writable?.locked) throw new TypeError("Cannot close a port with a locked stream");
    const readable = this.#readable, writable = this.#writable;
    this.#readable = this.#writable = null;
    this.#kill(new Error(`${this.path} was closed`));
    // The native close first: it interrupts a native write stuck on the device.
    const native = raw.isOpen
      ? new Promise<void>((resolve, reject) => raw.close((e) => (e ? reject(e) : resolve())))
      : Promise.resolve();
    const done = native.finally(() => {
      this.#detach();
      if (this.#raw === raw) this.#raw = null;
      this.#closing = null;
    });
    done.catch(() => undefined);
    void readable?.cancel().catch(() => undefined);
    void writable?.abort().catch(() => undefined);
    // Checked after pending I/O is delivered: a close that completed during a stall is not late.
    this.#closing = boundAfterIo(this.#clock, done, this.#closeTimeout,
      () => new Error(`closing ${this.path} timed out after ${this.#closeTimeout} s`));
    return this.#closing;
  }
}

/**
 * A serial port at `path` as a SerialPortLike, not yet opened (open it with `open({ baudRate })`,
 * or hand it to SlcanSerialBackend). Throws ConnectError when `serialport` is not installed.
 */
export async function openSerialPort(path: string, options: NodeSerialPortOptions = {}): Promise<SerialPortLike> {
  const module = await loadSerialport();
  let info = options.info;
  if (!info) {
    const found = (await listSerialPorts()).find((p) => p.path === path);
    info = {};
    if (found?.vendorId != null) info.usbVendorId = found.vendorId;
    if (found?.productId != null) info.usbProductId = found.productId;
  }
  return new NodeSerialPort(path, module, { ...options, info });
}

/** openSlcanAdapter settings: the backend's, and the port's flow control. */
export interface OpenSlcanAdapterOptions extends SlcanSerialBackendOptions {
  rtscts?: boolean;
}

/**
 * The SLCAN adapter at `path` (omitted: the only candidate port, as the Python SDK picks it), as an
 * unopened SlcanSerialBackend named after its path. Throws ConnectError when no single candidate is
 * found or `serialport` is not installed.
 */
export async function openSlcanAdapter(path?: string | null, options: OpenSlcanAdapterOptions = {}): Promise<SlcanSerialBackend> {
  const { rtscts, ...backendOptions } = options;
  let chosen = path ?? null;
  if (!chosen) {
    const found = slcanCandidates(await listSerialPorts());
    if (found.length === 1) chosen = found[0]!.path;
    else if (!found.length) {
      throw new ConnectError("no USB CAN adapter found; plug it in or pass its serial port path");
    } else {
      throw new ConnectError(`several serial devices found (${found.map((p) => p.path).join(", ")}); pass the port path`);
    }
  }
  const portOptions: NodeSerialPortOptions = {};
  if (rtscts !== undefined) portOptions.rtscts = rtscts;
  if (backendOptions.clock) portOptions.clock = backendOptions.clock;
  const port = await openSerialPort(chosen, portOptions);
  return new SlcanSerialBackend(port, { name: chosen, ...backendOptions });
}
