/**
 * Serial ports, as the Web Serial API shapes them. Everything that talks over a serial line (an
 * SLCAN adapter, a leader arm's servo bus) takes a SerialPortLike, so a browser's SerialPort, the
 * Node adapter from "@makermods/arm-core/node" and a fake port in tests all fit.
 */

import { ConnectError } from "../errors.js";

/** The parts of a Web Serial SerialPort the SDK uses. */
export interface SerialPortLike {
  open(options: { baudRate: number }): Promise<void>;
  readonly readable: ReadableStream<Uint8Array> | null;
  readonly writable: WritableStream<Uint8Array> | null;
  close(): Promise<void>;
  getInfo?(): { usbVendorId?: number; usbProductId?: number };
}

export interface SerialFilter {
  usbVendorId?: number;
  usbProductId?: number;
}

/** The parts of navigator.serial used here. */
export interface SerialLike {
  requestPort(options?: { filters?: SerialFilter[] }): Promise<SerialPortLike>;
  getPorts(): Promise<SerialPortLike[]>;
}

function navigatorSerial(): SerialLike {
  const serial = (globalThis as { navigator?: { serial?: SerialLike } }).navigator?.serial;
  if (!serial) throw new ConnectError("Web Serial is not available here (it needs Chrome or Edge, on a secure page)");
  return serial;
}

const matches = (port: SerialPortLike, filters: readonly SerialFilter[]): boolean => {
  if (!filters.length) return true;
  const info = port.getInfo?.() ?? {};
  return filters.some((f) =>
    (f.usbVendorId === undefined || f.usbVendorId === info.usbVendorId)
    && (f.usbProductId === undefined || f.usbProductId === info.usbProductId));
};

/**
 * Shows the browser's serial port chooser, limited to `filters`. Must be called from a user
 * gesture (a click); in a Web Worker use grantedSerialPorts(). Resolves to null when the chooser
 * is dismissed.
 */
export async function requestSerialPort(
  filters: readonly SerialFilter[] = [],
  serial: SerialLike = navigatorSerial(),
): Promise<SerialPortLike | null> {
  try {
    return await serial.requestPort(filters.length ? { filters: [...filters] } : undefined);
  } catch (e) {
    if (e instanceof Error && e.name === "NotFoundError") return null;
    throw e;
  }
}

/** The serial ports this page was already granted, limited to `filters`. */
export async function grantedSerialPorts(
  filters: readonly SerialFilter[] = [],
  serial: SerialLike = navigatorSerial(),
): Promise<SerialPortLike[]> {
  return (await serial.getPorts()).filter((port) => matches(port, filters));
}
