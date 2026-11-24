/**
 * A stand-in for `navigator.usb` (page or worker) over fake candleLight adapters: `requestDevice` plays the browser's
 * chooser (filters honoured, a user gesture required, "dismiss" answers NotFoundError as Chrome does), `getDevices`
 * lists the granted adapters still plugged in, and unplugging one fires `disconnect` as Chrome does.
 */

import type { FakeGsUsbDevice } from "@makermods/arm-core/testing";

export type ChooserMode = "pick" | "dismiss";

type Filter = { vendorId?: number; productId?: number };

const matches = (device: FakeGsUsbDevice, filters: readonly Filter[]) =>
  filters.some((f) => (f.vendorId === undefined || f.vendorId === device.vendorId) && (f.productId === undefined || f.productId === device.productId));

/** An event like USBConnectionEvent: `device` is the adapter that came or went. */
function connectionEvent(type: "connect" | "disconnect", device: FakeGsUsbDevice): Event {
  const event = new Event(type);
  Object.defineProperty(event, "device", { value: device });
  return event;
}

export class FakeUsb extends EventTarget {
  /** Adapters plugged in, in plug order. */
  readonly plugged: FakeGsUsbDevice[] = [];
  readonly granted = new Set<FakeGsUsbDevice>();
  chooser: ChooserMode = "pick";
  /** How often the chooser was shown. */
  choosers = 0;
  onconnect: ((event: Event) => void) | null = null;
  ondisconnect: ((event: Event) => void) | null = null;

  /**
   * @param grantedElsewhere Serials granted by the page, for a worker (whose own chooser does not exist); the page
   *   passes nothing.
   */
  constructor(private readonly grantedElsewhere: (() => Promise<string[]>) | null = null) {
    super();
  }

  plug(device: FakeGsUsbDevice): void {
    this.plugged.push(device);
    const event = connectionEvent("connect", device);
    this.dispatchEvent(event);
    this.onconnect?.(event);
  }

  unplug(device: FakeGsUsbDevice): void {
    const i = this.plugged.indexOf(device);
    if (i < 0) return;
    this.plugged.splice(i, 1);
    this.granted.delete(device);
    device.unplug();
    const event = connectionEvent("disconnect", device);
    this.dispatchEvent(event);
    this.ondisconnect?.(event);
  }

  async requestDevice(options: { filters: Filter[] }): Promise<FakeGsUsbDevice> {
    if (this.grantedElsewhere) throw new DOMException("requestDevice is not available in a worker", "SecurityError");
    const activation = (navigator as Navigator & { userActivation?: { isActive: boolean } }).userActivation;
    if (activation && !activation.isActive) {
      throw new DOMException("Must be handling a user gesture to show a permission request.", "SecurityError");
    }
    if (!options || !Array.isArray(options.filters)) throw new TypeError("requestDevice needs { filters }");
    this.choosers++;
    const candidates = this.plugged.filter((d) => matches(d, options.filters));
    if (this.chooser === "dismiss" || candidates.length === 0) throw new DOMException("No device selected.", "NotFoundError");
    // The person picks an adapter not granted yet when there is one (a second Connect for a second arm).
    const device = candidates.find((d) => !this.granted.has(d)) ?? candidates[candidates.length - 1]!;
    this.granted.add(device);
    return device;
  }

  async getDevices(): Promise<FakeGsUsbDevice[]> {
    if (this.grantedElsewhere) {
      const serials = await this.grantedElsewhere();
      return this.plugged.filter((d) => serials.includes(d.serialNumber ?? ""));
    }
    return this.plugged.filter((d) => this.granted.has(d));
  }
}

/** Puts `usb` where the SDK looks for WebUSB (`navigator.usb`), in a page or a worker. */
export function installUsb(usb: FakeUsb): void {
  Object.defineProperty(navigator, "usb", { value: usb, configurable: true });
}
