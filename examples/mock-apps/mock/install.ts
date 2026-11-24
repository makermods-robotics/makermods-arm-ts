/**
 * Mock hardware for the apps, installed before an app runs when its URL has `?mock=<what>`:
 *
 *   ?mock=metal / ?mock=maker   one fake candleLight adapter with that arm's seven simulated motors behind it, as
 *                               `navigator.usb` (and in every module worker the page starts)
 *   ?mock=so101                 one simulated SO-101 on a fake CH343 serial port, as `navigator.serial`
 *
 * The SDK runs unchanged on top: the chooser, the USB or serial transport and the arm. `window.__mock` holds the
 * test hooks (what a person would do to the hardware: pull a cable, heat a motor, move a limp joint by hand).
 */

import { RealClock } from "@makermods/arm-core";
import { referenceRanges, SO101_IDS, SO101_MOTORS } from "@makermods/so101";
import { SAMPLE_CALIBRATION, simulatedSo101, type SimFeetechBus, type SimSerialPort } from "@makermods/so101/testing";
import { PRODUCTS, type Product, simulatedSlcanArm } from "./can-arm.ts";
import { type ChooserMode, FakeUsb, installUsb } from "./fake-usb.ts";
import { Bench, CHANNEL, callOp, type HubMessage } from "./hub.ts";

const what = new URLSearchParams(location.search).get("mock") ?? "";

// ---------- CAN arms over WebUSB ----------

const channel = new BroadcastChannel(CHANNEL);
const bench = new Bench();
const usb = new FakeUsb();
let plugs = 0;
let nextId = 1;
const results = new Map<number, (m: Extract<HubMessage, { kind: "result" }>) => void>();

function plug(product: Product): string {
  const serial = `FAKE-${product.toUpperCase()}-${++plugs}`;
  usb.plug(bench.add(product, serial).device);
  channel.postMessage({ kind: "plug", product, serial } satisfies HubMessage);
  return serial;
}

channel.onmessage = (event: MessageEvent<HubMessage>) => {
  const m = event.data;
  if (m.kind === "granted?") {
    const serials = [...usb.granted].filter((d) => usb.plugged.includes(d)).map((d) => d.serialNumber ?? "");
    channel.postMessage({ kind: "granted", id: m.id, serials } satisfies HubMessage);
  } else if (m.kind === "result") {
    results.get(m.id)?.(m);
    results.delete(m.id);
  }
};

/** A hook call on the simulated arm whose adapter was opened (the most recent, or the one with `serial`): here, or
 * in a worker the page started. */
function callOn(serial: string | null, op: string, ...args: unknown[]): Promise<unknown> {
  const local = bench.active(serial);
  if (local) return Promise.resolve(callOp(local, op, args));
  return new Promise((resolve, reject) => {
    const id = nextId++;
    const timer = setTimeout(() => {
      results.delete(id);
      reject(new Error(`mock: no page or worker has opened an adapter (${op})`));
    }, 2000);
    results.set(id, (m) => {
      clearTimeout(timer);
      if (m.error) reject(new Error(m.error));
      else resolve(m.value);
    });
    channel.postMessage({ kind: "call", id, op, args, serial } satisfies HubMessage);
  });
}

const call = (op: string, ...args: unknown[]) => callOn(null, op, ...args);

/** Module workers the page starts load the worker-side mock first (worker-entry.ts), then their own module. */
function wrapWorkers(): void {
  const RealWorker = window.Worker;
  const entry = new URL("./worker-entry.ts", import.meta.url);
  window.Worker = class extends RealWorker {
    constructor(url: string | URL, options?: WorkerOptions) {
      if (options?.type !== "module") {
        super(url, options);
        return;
      }
      const plugged = usb.plugged.map((d) => [bench.arms.get(d.serialNumber ?? "")!.product, d.serialNumber]);
      const first = new URL(entry);
      first.searchParams.set("plugged", JSON.stringify(plugged));
      const app = new URL(String(url), location.href).href;
      const code = `import ${JSON.stringify(first.href)};\nimport ${JSON.stringify(app)};\n`;
      super(URL.createObjectURL(new Blob([code], { type: "text/javascript" })), options);
    }
  };
}

// ---------- SO-101 over Web Serial ----------

interface So101Rig {
  sim: SimFeetechBus;
  port: SimSerialPort;
}

type InfoPort = { getInfo(): { usbVendorId?: number; usbProductId?: number } };

/** `navigator.serial` over fake ports: the chooser (filters honoured, a user gesture required) and the granted list. */
class FakeSerial<P extends InfoPort> extends EventTarget {
  chooser: ChooserMode = "pick";
  readonly granted = new Set<P>();
  constructor(readonly ports: P[]) {
    super();
  }

  async requestPort(options?: { filters?: { usbVendorId?: number; usbProductId?: number }[] }): Promise<P> {
    const activation = (navigator as Navigator & { userActivation?: { isActive: boolean } }).userActivation;
    if (activation && !activation.isActive) {
      throw new DOMException("Must be handling a user gesture to show a permission request.", "SecurityError");
    }
    const filters = options?.filters ?? [];
    const candidates = this.ports.filter((p) => {
      const info = p.getInfo();
      return !filters.length || filters.some((f) =>
        (f.usbVendorId === undefined || f.usbVendorId === info.usbVendorId) && (f.usbProductId === undefined || f.usbProductId === info.usbProductId));
    });
    if (this.chooser === "dismiss" || !candidates.length) throw new DOMException("No port selected by the user.", "NotFoundError");
    this.granted.add(candidates[0]!);
    return candidates[0]!;
  }

  async getPorts(): Promise<P[]> {
    return this.ports.filter((p) => this.granted.has(p));
  }
}

const sleep = (ms: number) => new Promise((resolve) => setTimeout(resolve, ms));

/** Holds still until two more position sync reads (instruction 0x82) reached the bus, so the calibration's recording
 * has seen the pose however late its reads run, as a person holds a joint at its stop (at most 2 s). */
async function held(rig: So101Rig): Promise<void> {
  const from = rig.sim.requests.length;
  const until = performance.now() + 2000;
  while (rig.sim.requests.slice(from).filter((r) => r[4] === 0x82).length < 2 && performance.now() < until) await sleep(10);
}

/** By hand: every joint taken end to end through Dream's reference range and back to the middle, as the
 * calibration asks a person to (the servos' homing is read when the recording has started). */
async function sweep(rig: So101Rig): Promise<void> {
  const ref = referenceRanges("left");
  for (const [i, m] of SO101_MOTORS.entries()) {
    const id = SO101_IDS[i]!;
    const homing = rig.sim.reg(id, "Homing_Offset");
    const r = ref[m];
    const ends = r ? [Math.round(r.low) + 30, Math.round(r.high) + 30] : m === "gripper" ? [1700, 2400] : [2047, 2047];
    for (const target of [...ends, 2047]) {
      const from = rig.sim.reg(id, "Present_Position");
      const steps = Math.ceil(Math.abs(target - from) / 300) || 1;
      for (let k = 1; k <= steps; k++) {
        rig.sim.moveByHand(id, homing + Math.round(from + ((target - from) * k) / steps));
        await sleep(60);
      }
      await held(rig);
    }
  }
}

function so101Hooks(rig: So101Rig, serial: FakeSerial<SimSerialPort>) {
  const { sim, port } = rig;
  return {
    setChooser: (mode: ChooserMode) => void (serial.chooser = mode),
    sweep: () => sweep(rig),
    /** Joint positions as the SDK reads them, rad. */
    positions: () => SO101_IDS.map((id) => ((sim.reg(id, "Present_Position") - 2048) * 2 * Math.PI) / 4096),
    torque: () => SO101_IDS.map((id) => sim.reg(id, "Torque_Enable")),
    /** OR `bits` into every status reply of servo `id` (0x04: overheat). */
    servoError: (id: number, bits: number) => void (sim.servo(id).forcedError = bits),
    /** Goal_Position values written to servo `id`, in order. */
    goals: (id: number) => sim.writesTo(id, "Goal_Position"),
    isOpen: () => port.isOpen,
    unplug: () => port.unplug(),
  };
}

// ---------- install ----------

const hooks: Record<string, unknown> = { what };
const params = new URLSearchParams(location.search);

if ((PRODUCTS as readonly string[]).includes(what) && params.get("link") === "slcan") {
  // ?mock=maker&link=slcan: the arm behind an SLCAN adapter on a serial port instead.
  const arm = simulatedSlcanArm(what as Product);
  const serial = new FakeSerial([arm.device]);
  Object.defineProperty(navigator, "serial", { value: serial, configurable: true });
  const op = (name: string, ...args: unknown[]) => Promise.resolve(callOp(arm, name, args));
  Object.assign(hooks, {
    setChooser: (mode: ChooserMode) => void (serial.chooser = mode),
    call: op,
    positions: () => op("positions"),
    enabled: () => op("enabled"),
    jointNames: () => op("jointNames"),
    fault: (joint: string, on = true) => op("fault", joint, on),
    protocolErrors: () => op("protocolErrors"),
    frames: () => op("frames"),
    /** The SLCAN commands the adapter took (C, S8, A0, O, ...). */
    commands: () => [...arm.device.commands],
    isOpen: () => arm.device.opened,
  });
} else if ((PRODUCTS as readonly string[]).includes(what)) {
  // ?mock=metal&adapters=2: that many adapters plugged in, each with its own arm.
  const product = what as Product;
  for (let i = 0; i < Number(params.get("adapters") ?? 1); i++) plug(product);
  installUsb(usb);
  wrapWorkers();
  Object.assign(hooks, {
    setChooser: (mode: ChooserMode) => void (usb.chooser = mode),
    choosers: () => usb.choosers,
    call,
    /** A hook on the arm behind the adapter with this serial. */
    callOn,
    /** The adapters plugged in, by serial. */
    adapters: () => usb.plugged.map((d) => d.serialNumber),
    /** Frames put on each adapter's bus, by serial (here, not in workers). */
    framesBySerial: () => Object.fromEntries([...bench.arms].map(([s, a]) => [s, a.device.writes.length])),
    opensBySerial: () => Object.fromEntries([...bench.arms].map(([s, a]) => [s, a.device.opens])),
    positions: () => call("positions"),
    enabled: () => call("enabled"),
    jointNames: () => call("jointNames"),
    fault: (joint: string, on = true) => call("fault", joint, on),
    offline: (joint: string, on = true) => call("offline", joint, on),
    setTemperature: (joint: string, celsius: number) => call("setTemperature", joint, celsius),
    protocolErrors: () => call("protocolErrors"),
    /** Pulls out every adapter (here and in the workers). */
    unplug: () => {
      for (const device of [...usb.plugged]) {
        usb.unplug(device);
        channel.postMessage({ kind: "unplug", serial: device.serialNumber ?? "" } satisfies HubMessage);
      }
    },
    /** Plugs in a fresh adapter with fresh motors (a pulled-out adapter stays dead, as a real one's handle does). */
    replug: () => plug(product),
  });
} else if (what === "so101") {
  const rig = simulatedSo101(new RealClock(), { calibration: SAMPLE_CALIBRATION, hold: false });
  const serial = new FakeSerial([rig.port]);
  Object.defineProperty(navigator, "serial", { value: serial, configurable: true });
  Object.assign(hooks, { so101: so101Hooks(rig, serial) });
} else if (what) {
  throw new Error(`?mock=${what}: expected one of ${[...PRODUCTS, "so101"].join(", ")}`);
}

(window as unknown as { __mock: unknown }).__mock = hooks;
