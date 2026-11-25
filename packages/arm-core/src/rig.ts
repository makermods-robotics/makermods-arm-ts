/**
 * A bench described once: which adapter drives which arm, the cameras, and the teleop leaders.
 *
 *     const rig = Rig.fromRaw(raw, product);           // the rig file's contents, as an object or JSON text
 *     const arms = await rig.group(["left", "right"]);
 *     await arms.open();                               // connect every arm
 *     try {
 *       await arms.enable();                           // both arms, concurrently
 *       await arms.get("left").moveTo(q);
 *     } finally {
 *       await arms.close();                            // every arm back to rest and released
 *     }
 *
 * The description has the keys of the Python SDK's rig YAML:
 *
 *     arms:
 *       left:  {backend: gs_usb, port: "gs_usb:<serial>"}   # profile: shipped default
 *       right: {backend: gs_usb, port: "gs_usb:<serial>", profile: my_profile.yaml}
 *     cameras: {top: "device:<deviceId>", left_wrist: "label:Wrist Cam", right_wrist: "index:2"}
 *     leaders:
 *       left:  {type: star, port: "serial:1a86:7523"}   # map: Star mapping JSON (default: shipped)
 *       right: {type: arm, arm: spare}                 # another arm entry, floated by hand
 *
 * A leader entry is keyed by the follower arm it drives. There is no filesystem here: a profile or
 * map named by path is loaded through `RigOptions.files`, which gets the path as written (resolve
 * it against the rig file's folder there); `profile` may also be the profile itself (an object with
 * the YAML's keys). Nothing here opens a device until an arm connects or a camera opens.
 *
 * CAN backends come from a resolver, because getting one is asynchronous and differs by
 * environment. The built-in one (browserBackends) works in a page or a Web Worker: `gs_usb` takes an
 * adapter already granted to the site (grantedAdapters() + selectAdapter()), `mock` builds a
 * MockBackend, `slcan` takes a granted serial port and the `RigOptions.slcan` factory. In Node pass
 * `backends`, e.g. `(spec) => spec.backend === "gs_usb" ? openAdapter(spec.port) : null` with
 * openAdapter from the core package's "/node" entry; returning null leaves the entry to the built-in one.
 */

import { Arm, type ArmOptions } from "./arm.js";
import { Camera, type CameraOptions, isCameraId } from "./cameras.js";
import { ArmConfig, type RawProfile } from "./config.js";
import { ArmError, ConnectError } from "./errors.js";
import { ArmLeader, StarLeader } from "./leaders/index.js";
import type { Logger } from "./logger.js";
import type { StarMapping } from "./mapping.js";
import type { Product } from "./motor.js";
import { ArmState } from "./state.js";
import { CanBackend } from "./transport/base.js";
import { adapterPort, grantedAdapters, GsUsbBackend, type GsUsbBackendOptions, parsePort, selectAdapter } from "./transport/gs-usb.js";
import { MockBackend } from "./transport/mock.js";
import { grantedSerialPorts, type SerialPortLike } from "./transport/serial.js";
import { SlcanSerialBackend } from "./transport/slcan-serial.js";

/** Backends a rig entry may name. `mock` (no hardware) is accepted here as Python's create_backend accepts it. */
export const BACKENDS = ["socketcan", "slcan", "gs_usb", "mock"] as const;
export const LEADER_TYPES = ["star", "arm"] as const;
export const DEFAULT_CHANNEL = "can0";
export const SLCAN_BAUDRATE = 115200;
/** s, waited after opening an SLCAN serial port. */
export const SLCAN_STARTUP_DELAY = 2.5;
/** Camera id prefixes passed through as ids by Rig.cameraId(). */
export const CAMERA_ID_PREFIXES = ["usb", "path", "index", "device", "label"] as const;

/** The rig description is invalid, or names something it does not contain. */
export class RigError extends ArmError {}

/** Python's repr() of a JSON-like value, so messages read as the Python SDK's do. */
function pyRepr(value: unknown): string {
  if (value === null || value === undefined) return "None";
  if (value === true) return "True";
  if (value === false) return "False";
  if (typeof value === "string") {
    const quote = value.includes("'") && !value.includes('"') ? '"' : "'";
    let out = "";
    for (const c of value) {
      if (c === "\\") out += "\\\\";
      else if (c === quote) out += "\\" + c;
      else if (c === "\n") out += "\\n";
      else if (c === "\t") out += "\\t";
      else if (c === "\r") out += "\\r";
      else out += c;
    }
    return quote + out + quote;
  }
  if (Array.isArray(value)) return `[${value.map(pyRepr).join(", ")}]`;
  if (typeof value === "object") {
    return `{${Object.entries(value as Record<string, unknown>).map(([k, v]) => `${pyRepr(k)}: ${pyRepr(v)}`).join(", ")}}`;
  }
  return String(value);
}

const isMapping = (raw: unknown): raw is Record<string, unknown> =>
  raw !== null && typeof raw === "object" && !Array.isArray(raw) && !(raw instanceof Map);

/** A profile as an entry may give it: a path (loaded through RigOptions.files), the profile's keys, or a built config. */
export type ProfileSource = string | RawProfile | ArmConfig;

/** ArmSpec fields other than the backend. */
export interface ArmSpecInit {
  /** socketcan interface. */
  channel?: string | null;
  /** slcan serial port, or gs_usb:<serial> (null: the only one). */
  port?: string | null;
  /** null: the shipped profile. */
  profile?: string | RawProfile | null;
  /** slcan serial baud rate. */
  baudrate?: number | null;
  /** slcan: seconds to wait after opening the port. */
  startupDelay?: number | null;
}

/** What an slcan backend is built with (the Python backend's keyword arguments). */
export interface SlcanBackendOptions {
  rtscts: false;
  baudrate: number;
  startupDelay: number;
}

/** ArmSpec.backendKwargs(): what the entry's backend is built with. */
export type BackendKwargs =
  | { channel: string }
  | { port: string | null }
  | ({ port: string | null } & SlcanBackendOptions);

const ARM_SPEC_KEYS: Record<string, keyof ArmSpecInit | "backend"> = {
  backend: "backend", channel: "channel", port: "port", profile: "profile", baudrate: "baudrate", startup_delay: "startupDelay",
};
const LEADER_SPEC_KEYS: Record<string, keyof LeaderSpecInit | "type"> = { type: "type", port: "port", map: "map", arm: "arm" };

function fields(raw: unknown, known: Record<string, string>, where: string): Record<string, unknown> {
  if (!isMapping(raw)) throw new RigError(`${where}: expected a mapping, got ${pyRepr(raw)}`);
  const unknown = Object.keys(raw).filter((k) => !Object.hasOwn(known, k)).sort();
  if (unknown.length) {
    throw new RigError(`${where}: unknown keys ${pyRepr(unknown)} (known: ${pyRepr(Object.keys(known).sort())})`);
  }
  return raw;
}

/** How to reach one arm: a CAN backend and the profile describing it. */
export class ArmSpec {
  backend: string;
  channel: string | null;
  port: string | null;
  profile: string | RawProfile | null;
  baudrate: number | null;
  startupDelay: number | null;

  constructor(backend: string, init: ArmSpecInit = {}) {
    if (!(BACKENDS as readonly string[]).includes(backend)) {
      throw new RigError(`backend must be one of ${BACKENDS.join(", ")}, got ${pyRepr(backend)}`);
    }
    this.backend = backend;
    this.channel = init.channel ?? null;
    this.port = init.port ?? null;
    this.profile = init.profile ?? null;
    this.baudrate = init.baudrate ?? null;
    this.startupDelay = init.startupDelay ?? null;
  }

  /** An entry from the rig description's keys (snake_case, as in the YAML). */
  static fromRaw(raw: unknown, where = "arm"): ArmSpec {
    const given = fields(raw, ARM_SPEC_KEYS, where);
    if (given.backend === undefined) throw new RigError(`${where}: missing 1 required key: 'backend'`);
    const init: Record<string, unknown> = {};
    for (const [key, value] of Object.entries(given)) if (key !== "backend") init[ARM_SPEC_KEYS[key]!] = value;
    return new ArmSpec(given.backend as string, init as ArmSpecInit);
  }

  /** Where the bus is: "can1", "/dev/ttyACM0", "gs_usb:<serial>" ("gs_usb" = the only adapter). */
  get address(): string {
    if (this.backend === "socketcan") return this.channel || DEFAULT_CHANNEL;
    if (this.backend === "gs_usb") return canonicalGsUsb(this.port);
    if (this.backend === "mock") return this.port || "mock";
    return canonicalSerial(this.port) || "slcan";
  }

  /**
   * The entry's profile (`profile` overrides it): the shipped one when none is named, a path
   * through `files`, or the profile itself.
   */
  async config(product: Product, files?: FileLoader | null, profile?: ProfileSource | null): Promise<ArmConfig> {
    return loadProfile(profile ?? this.profile, product, files);
  }

  /** A new (closed) backend for this arm's bus: `options.backends` first, then the built-in resolver. */
  async openBackend(options: RigOptions = {}, name = "arm"): Promise<CanBackend> {
    const custom = await options.backends?.(this, name);
    return custom ?? (await browserBackends(options)(this, name))!;
  }

  /** An Arm for this entry, not connected. `profile` overrides the entry's. */
  async build(product: Product, options: RigOptions = {}, profile?: ProfileSource | null, name = "arm"): Promise<Arm> {
    const config = await this.config(product, options.files, profile);
    return new Arm(config, await this.openBackend(options, name), options.armOptions);
  }

  /** What the entry's backend is built with. A missing slcan port stays null: the serial-port resolver picks it. */
  backendKwargs(): BackendKwargs {
    if (this.backend === "socketcan") return { channel: this.channel || DEFAULT_CHANNEL };
    if (this.backend === "gs_usb" || this.backend === "mock") return { port: this.port };
    return {
      port: this.port, rtscts: false, baudrate: this.baudrate || SLCAN_BAUDRATE,
      startupDelay: this.startupDelay === null ? SLCAN_STARTUP_DELAY : this.startupDelay,
    };
  }

  /** The entry as the rig description writes it: snake_case keys, unset ones left out. */
  toRaw(): Record<string, unknown> {
    const out: Record<string, unknown> = {};
    const entries: Array<[string, unknown]> = [
      ["backend", this.backend], ["channel", this.channel], ["port", this.port], ["profile", this.profile],
      ["baudrate", this.baudrate], ["startup_delay", this.startupDelay],
    ];
    for (const [k, v] of entries) if (v !== null && v !== undefined) out[k] = v;
    return out;
  }
}

/** LeaderSpec fields other than the type. */
export interface LeaderSpecInit {
  /** star: serial port. */
  port?: string | null;
  /** star: mapping JSON path, loaded through RigOptions.files (null: the shipped mapping). */
  map?: string | null;
  /** arm: name of the arm entry used as leader. */
  arm?: string | null;
}

/** The leader that drives one follower arm. */
export class LeaderSpec {
  type: string;
  port: string | null;
  map: string | null;
  arm: string | null;

  constructor(type: string, init: LeaderSpecInit = {}) {
    if (!(LEADER_TYPES as readonly string[]).includes(type)) {
      throw new RigError(`leader type must be one of ${LEADER_TYPES.join(", ")}, got ${pyRepr(type)}`);
    }
    this.type = type;
    this.port = init.port ?? null;
    this.map = init.map ?? null;
    this.arm = init.arm ?? null;
    if (type === "star" && !this.port) throw new RigError("a star leader needs a port");
    if (type === "arm" && !this.arm) throw new RigError("an arm leader needs arm: <name of another arm entry>");
  }

  static fromRaw(raw: unknown, where = "leader"): LeaderSpec {
    const given = fields(raw, LEADER_SPEC_KEYS, where);
    if (given.type === undefined) throw new RigError(`${where}: missing 1 required key: 'type'`);
    const { type, ...init } = given;
    return new LeaderSpec(type as string, init as LeaderSpecInit);
  }

  toRaw(): Record<string, unknown> {
    const out: Record<string, unknown> = {};
    const entries: Array<[string, unknown]> = [["type", this.type], ["port", this.port], ["map", this.map], ["arm", this.arm]];
    for (const [k, v] of entries) if (v !== null && v !== undefined) out[k] = v;
    return out;
  }
}

// ── resolvers ──

/** Loads what a rig entry names by path (a profile, a Star mapping): the parsed contents, given the path as written. */
export type FileLoader = (path: string) => unknown;

/**
 * A CAN backend (closed) for the arm entry `spec` named `name`. Returning null or undefined leaves
 * the entry to the built-in resolver.
 */
export type BackendResolver = (spec: ArmSpec, name: string) => CanBackend | null | undefined | Promise<CanBackend | null | undefined>;

/** The serial port a rig entry names (null: the only one), for an slcan arm or a Star leader. Not opened. */
export type SerialPortResolver = (port: string | null, purpose: "slcan" | "star") => Promise<SerialPortLike>;

/** Builds an slcan backend (closed) on a serial port, e.g. `(port, o) => new SlcanSerialBackend(port, o)`. */
export type SlcanFactory = (port: SerialPortLike, options: SlcanBackendOptions) => CanBackend;

export interface RigOptions {
  /** Where the description came from, for messages (Python: the rig file's path). */
  path?: string | null;
  /** Loads profiles and Star mappings named by path. */
  files?: FileLoader | null;
  /** Tried first for every arm entry (Node: gs_usb through openAdapter()). */
  backends?: BackendResolver | null;
  /** Default: the page's granted serial ports (browserSerialPorts). */
  serialPorts?: SerialPortResolver | null;
  /** How the built-in resolver builds an slcan backend. */
  slcan?: SlcanFactory | null;
  /** For gs_usb backends built by the built-in resolver. */
  gsUsb?: GsUsbBackendOptions;
  /** For every Arm built (clock, logger, frame spacing). */
  armOptions?: ArmOptions;
  /** Where ArmGroups report (default: console). */
  logger?: Logger;
}

/**
 * The serial ports granted to this page: a null port is the only one granted; `serial:<vid>:<pid>`
 * (hex USB ids) the only one with those ids. A browser cannot see device paths (/dev/ttyUSB0,
 * COM3): a rig naming one needs RigOptions.serialPorts.
 */
export function browserSerialPorts(list: () => Promise<SerialPortLike[]> = () => grantedSerialPorts()): SerialPortResolver {
  return async (port, purpose) => {
    const what = purpose === "slcan" ? "USB CAN adapter" : "Star leader";
    let ports = await list();
    if (port !== null) {
      const m = /^serial:([0-9a-fA-F]{1,4}):([0-9a-fA-F]{1,4})$/.exec(port);
      if (!m) {
        throw new RigError(`a browser cannot find serial port ${pyRepr(port)} by name: name it serial:<usb vendor id>:<usb product id> ` +
          "(hex) or pass RigOptions.serialPorts");
      }
      const [vid, pid] = [parseInt(m[1]!, 16), parseInt(m[2]!, 16)];
      ports = ports.filter((p) => {
        const info = p.getInfo?.() ?? {};
        return info.usbVendorId === vid && info.usbProductId === pid;
      });
    }
    if (ports.length === 1) return ports[0]!;
    if (!ports.length) {
      throw new RigError(`no ${what} found${port === null ? "" : ` at ${port}`}: plug it in and grant it to this page ` +
        "(requestSerialPort() from a click)");
    }
    throw new RigError(`several serial ports found (${ports.length})${port === null ? "" : ` at ${port}`}; name the ${what}'s ` +
      "port or pass RigOptions.serialPorts");
  };
}

/**
 * The built-in backend resolver, for a page or a Web Worker: gs_usb on an adapter already granted
 * to the site, mock as a MockBackend, slcan on a granted serial port through `options.slcan`.
 * socketcan is a Linux kernel interface no browser reaches.
 */
export function browserBackends(options: Pick<RigOptions, "serialPorts" | "slcan" | "gsUsb"> = {}): BackendResolver {
  return async (spec, name) => {
    if (spec.backend === "mock") return new MockBackend();
    if (spec.backend === "gs_usb") return new GsUsbBackend(selectAdapter(await grantedAdapters(), spec.port), options.gsUsb);
    if (spec.backend === "slcan") {
      const kwargs = spec.backendKwargs() as { port: string | null } & SlcanBackendOptions;
      const port = await (options.serialPorts ?? browserSerialPorts())(kwargs.port, "slcan");
      const { port: _, ...rest } = kwargs;
      if (options.slcan) return options.slcan(port, rest);
      return new SlcanSerialBackend(port, { baudrate: rest.baudrate, startupDelay: rest.startupDelay });
    }
    throw new ConnectError(`arms.${name}: socketcan is a Linux kernel interface a browser cannot reach; ` +
      "use gs_usb, or pass RigOptions.backends");
  };
}

/** `gs_usb:<serial>` for any spelling of one adapter (`ABC`, `gs_usb:ABC`), `gs_usb` for "the only one". */
function canonicalGsUsb(port: string | null): string {
  try {
    const serial = parsePort(port);
    return serial === null ? "gs_usb" : `gs_usb:${serial}`;
  } catch {
    return port ?? "gs_usb"; // malformed: kept as written (opening it reports the error)
  }
}

/** `serial:<vid>:<pid>` in lower-case hex without leading zeros; other ports as written. */
function canonicalSerial(port: string | null): string | null {
  const m = port ? /^serial:([0-9a-fA-F]{1,4}):([0-9a-fA-F]{1,4})$/.exec(port) : null;
  return m ? `serial:${parseInt(m[1]!, 16).toString(16)}:${parseInt(m[2]!, 16).toString(16)}` : port;
}

/**
 * A validated copy of `config` that no other arm shares (an Arm and floatMode() change their
 * config's joints in place). Refuses a config of another product.
 */
function ownConfig(config: ArmConfig, product: Product, what: string): ArmConfig {
  if (config.product !== product && config.product.packageName !== product.packageName) {
    throw new RigError(`${what} is a ${config.product.displayName} profile; this rig's arms are ${product.displayName}`);
  }
  const changes: Record<string, unknown> = {};
  for (const [key, value] of Object.entries(config)) {
    if (key === "product") continue;
    if (Array.isArray(value)) changes[key] = [...value];
    else if (value !== null && typeof value === "object") changes[key] = structuredClone(value);
  }
  return config.replace(changes);
}

async function loadProfile(source: ProfileSource | null | undefined, product: Product, files?: FileLoader | null): Promise<ArmConfig> {
  if (source === null || source === undefined) return ArmConfig.fromRaw(product.defaultProfile, product);
  if (source instanceof ArmConfig) return ownConfig(source, product, "the profile");
  if (typeof source !== "string") return ArmConfig.fromRaw(source, product);
  if (!files) throw new RigError(`profile ${pyRepr(source)} is a path: pass RigOptions.files to load it`);
  const loaded = await files(source);
  if (loaded instanceof ArmConfig) return ownConfig(loaded, product, `profile ${pyRepr(source)}`);
  if (!isMapping(loaded)) throw new RigError(`profile ${pyRepr(source)}: expected a mapping, got ${pyRepr(loaded)}`);
  return ArmConfig.fromRaw(loaded, product);
}

/** What makes a bus physical: the backend, and the USB device or serial port under it. */
function physicalIds(x: CanBackend | SerialPortLike): unknown[] {
  const ids: unknown[] = [x];
  if (x instanceof GsUsbBackend) {
    ids.push(x.device);
    if (x.device.serialNumber) ids.push(adapterPort(x.device));
  } else if (x instanceof SlcanSerialBackend) {
    ids.push(x.serial);
  }
  return ids;
}

function describeBus(x: CanBackend | SerialPortLike): string {
  if (x instanceof GsUsbBackend) return `adapter ${adapterPort(x.device)}`;
  if (x instanceof SlcanSerialBackend) return "serial port";
  if (x instanceof CanBackend) return "backend";
  return "serial port";
}

/** Records who owns each physical bus; throws RigError when two owners resolve to the same one. */
function claimBus(owners: Map<unknown, string>, who: string, x: CanBackend | SerialPortLike): void {
  const ids = physicalIds(x);
  for (const id of ids) {
    const other = owners.get(id);
    if (other !== undefined && other !== who) {
      throw new RigError(`${other} and ${who} resolve to the same ${describeBus(x)}; give each its own adapter or port`);
    }
  }
  for (const id of ids) owners.set(id, who);
}

/** What `new Rig()` takes: the three sections, as Maps or plain objects. */
export interface RigInit {
  arms?: ReadonlyMap<string, ArmSpec> | Readonly<Record<string, ArmSpec>>;
  cameras?: ReadonlyMap<string, string> | Readonly<Record<string, string>>;
  leaders?: ReadonlyMap<string, LeaderSpec> | Readonly<Record<string, LeaderSpec>>;
}

const toMap = <V>(from: ReadonlyMap<string, V> | Readonly<Record<string, V>> | undefined): Map<string, V> =>
  from instanceof Map ? new Map(from) : new Map(Object.entries((from ?? {}) as Record<string, V>));

/**
 * A rig description: arms, cameras and leaders by name. `Rig.load()` (a path) has no browser
 * equivalent: read the file yourself and pass its parsed contents to Rig.fromRaw(); there is no
 * save() for the same reason (toRaw() gives what to write).
 */
export class Rig {
  arms: Map<string, ArmSpec>;
  cameras: Map<string, string>;
  leaders: Map<string, LeaderSpec>;
  readonly product: Product;
  options: RigOptions;

  constructor(init: RigInit, product: Product, options: RigOptions = {}) {
    this.arms = toMap(init.arms);
    this.cameras = toMap(init.cameras);
    this.leaders = toMap(init.leaders);
    this.product = product;
    this.options = options;
  }

  /** Python: the rig file's path; here only a name for messages. */
  get path(): string | null {
    return this.options.path ?? null;
  }

  #where(): string {
    return this.path ?? "the rig";
  }

  /**
   * A rig from its description: the parsed rig file (keys arms, cameras, leaders), its JSON text,
   * or null (an empty rig). Validated as Rig.validate() does.
   */
  static fromRaw(raw: unknown, product: Product, options: RigOptions = {}): Rig {
    const where = options.path || "rig";
    if (typeof raw === "string") {
      try {
        raw = JSON.parse(raw) as unknown;
      } catch (e) {
        throw new RigError(`${where}: not valid JSON (${e instanceof Error ? e.message : String(e)})`, { cause: e });
      }
    }
    raw ??= {};
    if (!isMapping(raw) || Object.keys(raw).some((k) => !["arms", "cameras", "leaders"].includes(k))) {
      throw new RigError(`${where}: expected a mapping with arms, cameras and leaders`);
    }
    for (const key of ["arms", "cameras", "leaders"]) {
      const section = raw[key] || {};
      if (!isMapping(section)) throw new RigError(`${where}: ${key}: expected a mapping of names, got ${pyRepr(raw[key])}`);
    }
    const section = (key: string) => Object.entries((raw as Record<string, unknown>)[key] || {});
    const arms = new Map(section("arms").map(([n, v]) => [n, ArmSpec.fromRaw(v, `${where}: arms.${n}`)] as const));
    const leaders = new Map(section("leaders").map(([n, v]) => [n, LeaderSpec.fromRaw(v, `${where}: leaders.${n}`)] as const));
    const cameras = new Map(section("cameras").map(([n, v]) => [n, String(v)] as const));
    const rig = new Rig({ arms, cameras, leaders }, product, options);
    rig.validate();
    return rig;
  }

  /** The description as the rig file writes it (Python: to_dict()). */
  toRaw(): { arms: Record<string, Record<string, unknown>>; cameras: Record<string, string>; leaders: Record<string, Record<string, unknown>> } {
    return {
      arms: Object.fromEntries([...this.arms].map(([n, s]) => [n, s.toRaw()])),
      cameras: Object.fromEntries(this.cameras),
      leaders: Object.fromEntries([...this.leaders].map(([n, s]) => [n, s.toRaw()])),
    };
  }

  validate(): void {
    for (const name of this.arms.keys()) {
      if (name === "all" || !name) throw new RigError(`${pyRepr(name)} cannot name an arm`);
    }
    const seen = new Map<string, string>();
    for (const [name, spec] of this.arms) {
      const key = `${spec.backend}\u0000${spec.address}`;
      if (seen.has(key) && !["gs_usb", "slcan", "mock"].includes(spec.address)) {
        throw new RigError(`arms ${pyRepr(seen.get(key))} and ${pyRepr(name)} use the same bus ${spec.address}`);
      }
      seen.set(key, name);
    }
    for (const [name, spec] of this.leaders) {
      if (!this.arms.has(name)) throw new RigError(`leaders.${name}: no arm named ${pyRepr(name)} to follow it`);
      if (spec.type === "arm") {
        if (!this.arms.has(spec.arm!)) throw new RigError(`leaders.${name}: no arm named ${pyRepr(spec.arm)}`);
        if (spec.arm === name) throw new RigError(`leaders.${name}: an arm cannot lead itself`);
      }
    }
  }

  // ── lookups ──
  get armNames(): string[] {
    return [...this.arms.keys()];
  }

  spec(name: string): ArmSpec {
    const spec = this.arms.get(name);
    if (!spec) {
      const known = this.armNames.join(", ") || "none";
      throw new RigError(`no arm named ${pyRepr(name)} in ${this.#where()} (arms: ${known})`);
    }
    return spec;
  }

  /** Expand `all` and check every name; order kept, duplicates dropped. */
  names(selected: Iterable<string>): string[] {
    const out: string[] = [];
    for (const name of selected) {
      for (const n of name === "all" ? this.armNames : [name]) {
        this.spec(n);
        if (!out.includes(n)) out.push(n);
      }
    }
    if (!out.length) throw new RigError(`no arms in ${this.#where()}`);
    return out;
  }

  /** The entry's profile (`profile` overrides it). */
  config(name: string, profile?: ProfileSource | null): Promise<ArmConfig> {
    return this.spec(name).config(this.product, this.options.files, profile);
  }

  /** A new (closed) backend for the named arm's bus: RigOptions.backends first, then the built-in resolver. */
  openBackend(name: string): Promise<CanBackend> {
    return this.spec(name).openBackend(this.options, name);
  }

  /** The named arm, not connected. `profile` overrides the entry's. */
  arm(name: string, profile?: ProfileSource | null): Promise<Arm> {
    return this.spec(name).build(this.product, this.options, profile, name);
  }

  /**
   * A rig camera name's id; an id (`usb:`, `path:`, `index:`, `device:`, `label:`,
   * `url:http...`) passes through.
   */
  cameraId(name: string): string {
    const id = this.cameras.get(name);
    if (id !== undefined) return id;
    if (name.includes(":") && isCameraId(name)) return name;
    const known = [...this.cameras.keys()].join(", ") || "none";
    throw new RigError(`no camera named ${pyRepr(name)} in ${this.#where()} (cameras: ${known})`);
  }

  /** The named camera (or an id), not opened. */
  camera(name: string, options: CameraOptions = {}): Camera {
    return new Camera(this.cameraId(name), options);
  }

  /**
   * The leader configured for the follower `name`. An arm leader reuses `arms[leader]` when given
   * (so one Arm object owns each bus), else builds it. A star leader gets its serial port from
   * RigOptions.serialPorts (not opened) and its mapping from RigOptions.files.
   */
  async leader(name: string, followerArm: Arm,
               arms?: ReadonlyMap<string, Arm> | Readonly<Record<string, Arm>> | null): Promise<RigLeader> {
    const spec = this.leaders.get(name);
    if (!spec) throw new RigError(`no leader for arm ${pyRepr(name)} in ${this.#where()}`);
    if (spec.type === "star") {
      let mapping: StarMapping | null = null;
      if (spec.map) {
        if (!this.options.files) throw new RigError(`leaders.${name}: map ${pyRepr(spec.map)} is a path: pass RigOptions.files to load it`);
        const raw = await this.options.files(spec.map);
        if (!isMapping(raw)) throw new RigError(`leaders.${name}: map ${pyRepr(spec.map)}: expected a mapping, got ${pyRepr(raw)}`);
        mapping = raw as unknown as StarMapping;
      }
      const port = await (this.options.serialPorts ?? browserSerialPorts())(spec.port, "star");
      const owners = new Map<unknown, string>();
      claimBus(owners, `arm ${pyRepr(name)} (follower)`, followerArm.backend);
      const others = arms instanceof Map ? [...arms] : Object.entries(arms ?? {});
      for (const [n, a] of others) if (a !== followerArm) claimBus(owners, `arm ${pyRepr(n)}`, a.backend);
      claimBus(owners, `leaders.${name} (star)`, port);
      return new StarLeader(port, {
        mapping, product: this.product, clock: this.options.armOptions?.clock, logger: this.options.armOptions?.logger,
      });
    }
    const given = arms instanceof Map ? arms.get(spec.arm!) : (arms as Record<string, Arm> | null | undefined)?.[spec.arm!];
    const leaderArm = given ?? (await this.arm(spec.arm!));
    const owners = new Map<unknown, string>();
    claimBus(owners, `arm ${pyRepr(name)} (follower)`, followerArm.backend);
    claimBus(owners, `arm ${pyRepr(spec.arm)} (leader of ${pyRepr(name)})`, leaderArm.backend);
    return new ArmLeader(leaderArm, followerArm);
  }

  /** The named arms (default all) as an ArmGroup, concurrent when each has its own bus. */
  async group(names: Iterable<string> = ["all"], profile?: ProfileSource | null): Promise<ArmGroup> {
    const selected = this.names(names);
    const buses = selected.map((n) => `${this.arms.get(n)!.backend}\u0000${this.arms.get(n)!.address}`);
    const arms = new Map<string, Arm>();
    const owners = new Map<unknown, string>();
    for (const n of selected) {
      const arm = await this.arm(n, profile);
      claimBus(owners, `arm ${pyRepr(n)}`, arm.backend); // nothing is opened yet
      arms.set(n, arm);
    }
    return new ArmGroup(arms, new Set(buses).size === buses.length, this.options.logger);
  }
}

/** What Rig.leader() builds. */
export type RigLeader = StarLeader | ArmLeader;

const errorText = (e: unknown): string => (e instanceof Error ? e.message : String(e));

/** close() outcome per arm (see ArmGroup.close()). */
export type CloseOutcome = "released" | "energized" | "unknown" | "detached" | "holding" | `error: ${string}`;

/**
 * Several arms driven together: connect, enable and stop all of them, concurrently when each has
 * its own bus. If one arm fails to enable, the arms already enabled are stopped again.
 *
 * close() (also `await using group = ...`) stops every enabled arm (verified return to rest, then
 * release) and disconnects; it says per arm whether torque off was confirmed. An arm whose return
 * is not verified (or is interrupted) is never released: it is detached holding its pose when no
 * stiff joint has a motor watchdog armed, else left ENABLED with its control loop holding while
 * this page or process runs (stop() or disable() it).
 */
export class ArmGroup {
  readonly arms: Map<string, Arm>;
  /** Run calls on the arms concurrently (false: one arm after the other, e.g. when they share a bus). */
  parallel: boolean;
  #log: Logger;

  constructor(arms: ReadonlyMap<string, Arm> | Readonly<Record<string, Arm>> | Iterable<Arm>, parallel = true, logger: Logger = console) {
    if (arms instanceof Map) this.arms = new Map(arms);
    else if (typeof (arms as Iterable<Arm>)[Symbol.iterator] === "function") {
      this.arms = new Map([...(arms as Iterable<Arm>)].map((a, i) => [String(i), a]));
    } else this.arms = new Map(Object.entries(arms as Record<string, Arm>));
    this.parallel = parallel;
    this.#log = logger;
  }

  /** The named arm. */
  get(name: string): Arm {
    const arm = this.arms.get(name);
    if (!arm) throw new RangeError(`no arm named ${pyRepr(name)} (arms: ${[...this.arms.keys()].join(", ")})`);
    return arm;
  }

  /** [name, arm] pairs. */
  [Symbol.iterator](): IterableIterator<[string, Arm]> {
    return this.arms.entries();
  }

  get length(): number {
    return this.arms.size;
  }

  /**
   * `fn(arm)` for every arm (concurrently when allowed). Every call runs to completion; then the
   * first error (the first to fail), if any, is thrown, wrapped in ArmError unless it is one.
   */
  async each<T>(fn: (arm: Arm) => T | Promise<T>, names?: Iterable<string> | null): Promise<Record<string, T | null>> {
    const list = names ? [...names] : [...this.arms.keys()];
    const results = new Map<string, T>();
    const errors = new Map<string, unknown>();
    const call = async (name: string): Promise<void> => {
      try {
        results.set(name, await fn(this.get(name)));
      } catch (e) { // reported below, after every arm has finished
        errors.set(name, e);
      }
    };
    if (this.parallel && list.length > 1) await Promise.all(list.map(call));
    else for (const n of list) await call(n);
    if (errors.size) {
      const [name, error] = errors.entries().next().value!;
      if (errors.size > 1) {
        this.#log.error(`several arms failed: ${pyRepr(Object.fromEntries([...errors].map(([n, e]) => [n, errorText(e)])))}`);
      }
      throw error instanceof ArmError ? error : new ArmError(`${name}: ${errorText(error)}`, { cause: error });
    }
    const out: Record<string, T | null> = {};
    for (const n of list) out[n] = results.has(n) ? results.get(n)! : null;
    return out;
  }

  /** Connect every arm; if one fails, every arm is disconnected and the error thrown. */
  async connect(clearFaults = false, clearReleased = false): Promise<void> {
    try {
      await this.each((a) => a.connect(clearFaults, clearReleased));
    } catch (e) {
      await this.disconnect();
      throw e;
    }
  }

  /** Enable every arm (`signal` aborts each enable, as in Arm.enable); if one fails, every arm is stopped and the error thrown. */
  async enable(signal?: AbortSignal): Promise<void> {
    try {
      await this.each((a) => a.enable(signal ? { signal } : {}));
    } catch (e) {
      await this.stop();
      throw e;
    }
  }

  /**
   * Return every enabled arm to rest and release it; false where the return was not verified (that
   * arm keeps holding).
   */
  stop(): Promise<Record<string, boolean | null>> {
    return this.each((a) => a.stop());
  }

  async disable(): Promise<void> {
    await this.each((a) => a.disable());
  }

  /**
   * Torque off on every arm now. Every arm's e-stop starts before any is awaited (each sets its
   * stop flags synchronously); the first error is thrown once all have finished.
   */
  async estop(): Promise<void> {
    const pending = [...this.arms.values()].map((arm) => arm.estop());
    const settled = await Promise.allSettled(pending);
    const failed = settled.find((s): s is PromiseRejectedResult => s.status === "rejected");
    if (failed) throw failed.reason;
  }

  async disconnect(): Promise<void> {
    for (const arm of this.arms.values()) {
      try {
        await arm.disconnect();
      } catch (e) {
        this.#log.error(`disconnect failed: ${errorText(e)}`);
      }
    }
  }

  /** Connect every arm (Python: entering a `with` block). Returns the group. */
  async open(): Promise<this> {
    await this.connect();
    return this;
  }

  /** close(), for `await using`. */
  async [Symbol.asyncDispose](): Promise<void> {
    await this.close();
  }

  /**
   * Stop every arm and disconnect. Outcome per arm: "released" (torque off confirmed by every
   * motor), "energized" (not enabled here but its motors report torque on, e.g. left holding by an
   * earlier process: bus closed, still on), "unknown" (torque off not confirmed: a motor did not
   * answer, reports no mode, or the arm was never connected), "detached" (return to rest not
   * verified: bus closed with torque on, holding its pose; joints with a motor watchdog release
   * when it expires), "holding" (return not verified and a stiff joint would drop on its
   * watchdog: left ENABLED, control loop holding) or "error: <reason>". Only "released" means the
   * arm is off.
   */
  async close(): Promise<Record<string, CloseOutcome>> {
    const isOn = (arm: Arm) => arm.state === ArmState.ENABLED || arm.state === ArmState.FAULT;
    const enabled = new Set([...this.arms].filter(([, a]) => isOn(a)).map(([n]) => n));
    await this.each(async (arm) => {
      try {
        await arm.stop();
      } catch (e) { // the arm is handled below by the state it is left in
        this.#log.error(`stop failed: ${errorText(e)}`);
      }
    });
    const outcomes: Record<string, CloseOutcome> = {};
    for (const [name, arm] of this.arms) {
      try {
        if (!isOn(arm)) {
          if (enabled.has(name)) outcomes[name] = arm.unconfirmedDisable.length ? "unknown" : "released";
          else outcomes[name] = await reportedTorque(arm, this.#log);
          await arm.disconnect();
          if (outcomes[name] !== "released") this.#log.error(`${name}: torque off not confirmed (${outcomes[name]})`);
          continue;
        }
        arm.holdCurrentPosition();
        const unsafe = arm.detachUnsafe();
        if (unsafe.length) {
          outcomes[name] = "holding";
          this.#log.error(`${name}: return to rest not verified; still holding with torque on (control loop running): ` +
            `${unsafe.join(", ")} would drop on their watchdog if commands stopped`);
        } else {
          const releasing = await arm.detach();
          outcomes[name] = "detached";
          this.#log.error(`${name}: return to rest not verified; left holding with torque on ` +
            `(${Object.keys(releasing).join(", ") || "no joints"} release on their watchdog). Support it, then release it`);
        }
      } catch (e) {
        outcomes[name] = `error: ${errorText(e)}`;
        this.#log.error(`${name}: closing failed: ${errorText(e)}`);
      }
    }
    return outcomes;
  }
}

/** "released", "energized" or "unknown" from fresh replies of an arm that is not enabled. */
async function reportedTorque(arm: Arm, log: Logger): Promise<CloseOutcome> {
  if (arm.state !== ArmState.CONNECTED) return "unknown";
  let fresh: boolean[] | null;
  try {
    fresh = await arm.refresh();
  } catch (e) {
    log.error(`torque state not read: ${errorText(e)}`);
    return "unknown";
  }
  if (!fresh || !fresh.length || !fresh.every(Boolean)) return "unknown";
  const reported = Object.values(arm.status().joints).map((j) => j.enabled);
  if (reported.some((on) => on === true)) return "energized";
  return reported.includes(null) ? "unknown" : "released";
}
