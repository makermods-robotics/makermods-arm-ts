/**
 * Node helpers: gs_usb adapters through the optional `usb` package (node-usb), whose WebUSBDevice
 * gives the same device interface the browser has. Imported only from "@makermods/arm-core/node",
 * so browser bundles never load `usb`.
 */

import type { Device, Endpoint, InEndpoint, Interface } from "usb";
import { ConnectError } from "./errors.js";
import {
  EP_IN, EP_OUT, GsUsbBackend, type GsUsbBackendOptions, INTERFACE, type UsbControlSetup, type UsbDeviceLike,
  adapterPort, isGsUsbDevice, selectAdapter,
} from "./transport/gs-usb.js";

type UsbModule = typeof import("usb");

/** An adapter as listAdapters() describes it, from descriptors only. */
export interface AdapterInfo {
  serial: string;
  /** `gs_usb:<serial>`, or "" when the adapter has no serial number. */
  port: string;
  /** `<bus>-<port path>` (or `<bus>:<address>`), as the Python SDK's ports listing shows it. */
  usbPath: string;
  product: string;
  manufacturer: string;
  vidPid: string;
}

/** The node-usb Endpoint members used here (an InEndpoint also has transferAsync). */
export type RawEndpoint = Pick<Endpoint, "timeout" | "direction"> & Partial<Pick<InEndpoint, "transferAsync">>;

/** The node-usb Interface members used here: the kernel-driver calls live on the interface. */
export interface RawInterface extends Pick<Interface, "isKernelDriverActive" | "detachKernelDriver"> {
  endpoint(address: number): RawEndpoint | undefined;
}

/** The node-usb Device members NodeGsUsbDevice uses. A real `usb.Device` fits. */
export interface RawDevice extends Pick<Device, "timeout"> {
  readonly interfaces: readonly unknown[] | undefined;
  interface(n: number): RawInterface;
}

/** libusb transfer status codes (node-usb's usb.LIBUSB_TRANSFER_*). */
export interface LibusbTransferCodes {
  timedOut: number;
  stall: number;
  overflow: number;
}

const LIBUSB_CODES: LibusbTransferCodes = { timedOut: 2, stall: 4, overflow: 6 };

const NEEDS_USB =
  "gs_usb in Node needs the optional 'usb' package (npm install usb; on Linux it needs libudev, " +
  "e.g. apt install libudev-dev when no prebuilt binary fits)";

let loading: Promise<UsbModule> | null = null;

/** The `usb` package, loaded on first use. Throws ConnectError when it is not installed. */
export function loadUsb(): Promise<UsbModule> {
  loading ??= (import("usb") as Promise<UsbModule>).catch((e: unknown) => {
    loading = null;
    throw new ConnectError(`${NEEDS_USB}: ${e instanceof Error ? e.message : String(e)}`, { cause: e });
  });
  return loading;
}

const ms = (seconds: number) => Math.max(1, Math.ceil(seconds * 1000));

/**
 * Raw node-usb devices this process has opened through a NodeGsUsbDevice, with their description.
 * node-usb hands enumeration the same Device object as the open one, and its `open()` resets the
 * object's interfaces (and `close()` would close the shared handle), so discovery never touches
 * these: it reports the cached description instead.
 */
const owned = new Map<object, { owner: NodeGsUsbDevice; info: AdapterInfo }>();

/**
 * A node-usb WebUSBDevice with the Python SDK's USB handling, on the raw node-usb Device behind it:
 * on Linux the kernel gs_usb driver is detached from interface 0 at open (and left detached on
 * close, so a quick reopen cannot race a rebind); control and bulk OUT transfers time out after
 * `ioTimeout` in libusb itself; bulk IN reads time out after `ioTimeout` too, and a timed-out read
 * answers ok with no data (Python's "nothing yet"), so close() can wait for every read to end
 * before closing the device (node-usb refuses to close a device with a pending transfer).
 */
export class NodeGsUsbDevice implements UsbDeviceLike {
  readonly readsTimeOut = true;
  readonly #web: UsbDeviceLike;

  /** libusb applies the endpoint timeout to every bulk transfer (set in claimInterface()). */
  get nativeTransferTimeoutMs(): number {
    return this.#timeoutMs;
  }
  readonly #raw: RawDevice;
  readonly #platform: string;
  readonly #timeoutMs: number;
  readonly #codes: LibusbTransferCodes;
  readonly #info: AdapterInfo;

  constructor(
    web: UsbDeviceLike,
    raw: RawDevice,
    options: { ioTimeout?: number; platform?: string; codes?: LibusbTransferCodes; info?: AdapterInfo } = {},
  ) {
    this.#web = web;
    this.#raw = raw;
    this.#timeoutMs = ms(options.ioTimeout ?? 0.1);
    this.#platform = options.platform ?? process.platform;
    this.#codes = options.codes ?? LIBUSB_CODES;
    const hex4 = (n: number) => n.toString(16).padStart(4, "0");
    this.#info = options.info ?? {
      serial: web.serialNumber ?? "",
      port: web.serialNumber ? adapterPort(web) : "",
      usbPath: "",
      product: web.productName ?? "",
      manufacturer: web.manufacturerName ?? "",
      vidPid: `${hex4(web.vendorId)}:${hex4(web.productId)}`,
    };
  }

  /** True while this process has the raw device open through a NodeGsUsbDevice. */
  static owns(raw: object): boolean {
    return owned.has(raw);
  }

  get vendorId() { return this.#web.vendorId; }
  get productId() { return this.#web.productId; }
  get serialNumber() { return this.#web.serialNumber; }
  get productName() { return this.#web.productName; }
  get manufacturerName() { return this.#web.manufacturerName; }
  /** Open through this wrapper (another wrapper's handle on the same device does not count). */
  get opened() { return this.#owner && this.#web.opened; }
  get configuration() { return this.#web.configuration; }
  get configurations() { return this.#web.configurations; }

  async open(): Promise<void> {
    const holder = owned.get(this.#raw);
    if (holder && holder.owner !== this) throw new ConnectError(`${adapterPort(this)} is already open in this process`);
    owned.set(this.#raw, { owner: this, info: this.#info });
    try {
      await this.#web.open();
      this.#raw.timeout = this.#timeoutMs;
      if (this.#platform === "linux") {
        const iface = this.#raw.interface(INTERFACE);
        if (iface.isKernelDriverActive()) iface.detachKernelDriver();
      }
    } catch (e) {
      if (!this.#web.opened) this.#disown();
      throw e;
    }
  }

  async close(): Promise<void> {
    if (!this.#owner) return; // never close a handle another wrapper owns
    try {
      await this.#web.close();
    } finally {
      // A device node-usb could not close stays owned: discovery must still leave it alone.
      if (!this.#web.opened) this.#disown();
    }
  }

  get #owner(): boolean {
    return owned.get(this.#raw)?.owner === this;
  }

  #disown(): void {
    if (owned.get(this.#raw)?.owner === this) owned.delete(this.#raw);
  }
  selectConfiguration(value: number): Promise<void> { return this.#web.selectConfiguration(value); }

  async claimInterface(n: number): Promise<void> {
    await this.#web.claimInterface(n);
    const iface = this.#raw.interface(n);
    for (const address of [EP_OUT, 0x80 | EP_IN]) {
      const endpoint = iface.endpoint(address);
      if (endpoint) endpoint.timeout = this.#timeoutMs;
    }
  }

  releaseInterface(n: number): Promise<void> { return this.#web.releaseInterface(n); }
  controlTransferIn(setup: UsbControlSetup, length: number) { return this.#web.controlTransferIn(setup, length); }
  controlTransferOut(setup: UsbControlSetup, data?: BufferSource) { return this.#web.controlTransferOut(setup, data); }
  transferOut(endpoint: number, data: BufferSource) { return this.#web.transferOut(endpoint, data); }

  async transferIn(endpointNumber: number, length: number): Promise<{ status: "ok" | "stall" | "babble"; data?: DataView }> {
    if (!this.#raw.interfaces) throw new Error("transferIn error: the device is not open");
    const endpoint = this.#raw.interface(INTERFACE).endpoint(0x80 | endpointNumber);
    if (!endpoint?.transferAsync || endpoint.direction !== "in") throw new Error(`transferIn error: no IN endpoint ${endpointNumber}`);
    try {
      const buffer = await endpoint.transferAsync(length);
      const bytes = buffer ? Uint8Array.from(buffer) : new Uint8Array(0);
      return { status: "ok", data: new DataView(bytes.buffer) };
    } catch (e) {
      const errno = (e as { errno?: number } | null)?.errno;
      if (errno === this.#codes.timedOut) return { status: "ok", data: new DataView(new ArrayBuffer(0)) };
      if (errno === this.#codes.stall) return { status: "stall" };
      if (errno === this.#codes.overflow) return { status: "babble" };
      throw new Error(`transferIn error: ${e instanceof Error ? e.message : String(e)}`, { cause: e });
    }
  }
}

function getString(raw: Device, index: number): Promise<string> {
  if (!index) return Promise.resolve("");
  return new Promise((resolve) => raw.getStringDescriptor(index, (error, value) => resolve(error ? "" : (value ?? ""))));
}

interface Found {
  raw: Device;
  info: AdapterInfo;
}

/**
 * The connected gs_usb devices, matched on their raw descriptors; each is opened only to read its
 * string descriptors (no configuration change, no claim, no transfer to the CAN side).
 */
async function findAdapters(): Promise<Found[]> {
  const usb = await loadUsb();
  const found: Found[] = [];
  for (const raw of usb.getDeviceList()) {
    const d = raw.deviceDescriptor;
    if (!isGsUsbDevice({ vendorId: d.idVendor, productId: d.idProduct })) continue;
    const holder = owned.get(raw);
    if (holder) {
      found.push({ raw, info: holder.info });
      continue;
    }
    const strings = { serial: "", product: "", manufacturer: "" };
    // A handle someone else in this process opened is read but left as it is.
    const openedHere = !raw.interfaces;
    try {
      if (openedHere) raw.open(false);
      try {
        strings.serial = await getString(raw, d.iSerialNumber);
        strings.product = await getString(raw, d.iProduct);
        strings.manufacturer = await getString(raw, d.iManufacturer);
      } finally {
        if (openedHere) raw.close();
      }
    } catch {
      // no permission or busy: described without its strings, as the Python listing does
    }
    const ports = raw.portNumbers ?? [];
    const hex4 = (n: number) => n.toString(16).padStart(4, "0");
    found.push({
      raw,
      info: {
        serial: strings.serial,
        port: strings.serial ? adapterPort({ serialNumber: strings.serial }) : "",
        usbPath: ports.length ? `${raw.busNumber}-${ports.join(".")}` : `${raw.busNumber}:${raw.deviceAddress}`,
        product: strings.product,
        manufacturer: strings.manufacturer,
        vidPid: `${hex4(d.idVendor)}:${hex4(d.idProduct)}`,
      },
    });
  }
  return found;
}

/**
 * Every gs_usb adapter: serial, port, USB path, product. Never opens the CAN bus. Devices are
 * matched on raw descriptors and read through node-usb directly, so discovery never reconfigures a
 * device (node-usb's WebUSB class sets configuration 1 on vendor-class devices on macOS while
 * listing them).
 */
export async function listAdapters(): Promise<AdapterInfo[]> {
  const out = (await findAdapters()).map((f) => f.info);
  return out.sort((a, b) =>
    a.serial < b.serial ? -1 : a.serial > b.serial ? 1 : a.usbPath < b.usbPath ? -1 : a.usbPath > b.usbPath ? 1 : 0,
  );
}

/**
 * The adapter a port names (`gs_usb:<serial>`; omitted: the only one connected), as an unopened
 * GsUsbBackend. Throws ConnectError when it is not found or `usb` is not installed. Only the chosen
 * adapter is wrapped in node-usb's WebUSBDevice (which, on macOS, sets configuration 1 on a
 * vendor-class device; candleLight reports device class 0 and is not affected).
 */
export async function openAdapter(port?: string | null, options: GsUsbBackendOptions = {}): Promise<GsUsbBackend> {
  const ioTimeout = options.ioTimeout ?? 0.1;
  const found = await findAdapters();
  const candidates = found.map((f) => ({
    vendorId: f.raw.deviceDescriptor.idVendor,
    productId: f.raw.deviceDescriptor.idProduct,
    serialNumber: f.info.serial,
    found: f,
  }));
  const chosen = selectAdapter(candidates, port).found;
  // Wrapping an open device would close its handle (WebUSBDevice.createInstance closes what it finds open).
  const alreadyOpen = () => new ConnectError(`${chosen.info.port || "gs_usb"} is already open in this process; close its backend first`);
  if (owned.has(chosen.raw)) throw alreadyOpen();
  const usb = await loadUsb();
  const web = await usb.WebUSBDevice.createInstance(chosen.raw, false);
  if (owned.has(chosen.raw)) throw alreadyOpen();
  const codes = { timedOut: usb.usb.LIBUSB_TRANSFER_TIMED_OUT, stall: usb.usb.LIBUSB_TRANSFER_STALL, overflow: usb.usb.LIBUSB_TRANSFER_OVERFLOW };
  return new GsUsbBackend(new NodeGsUsbDevice(web, chosen.raw, { ioTimeout, codes, info: chosen.info }), options);
}

export * from "./transport/node-serial.js";
