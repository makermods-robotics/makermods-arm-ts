/**
 * Star Arm 102 leader: FashionStar UART smart servos on a serial port (FashionStarBus).
 *
 * Connect pings every servo, unlocks it (back-drivable) and resets its multi-turn counter. Each read
 * is one sync monitor of all servos; a servo that misses a reply keeps its last target
 * (JointMapper), and one that misses `missLimit` reads in a row ends the session with LeaderError.
 */

import { type Clock, RealClock } from "../clock.js";
import { ValueError } from "../errors.js";
import type { Logger } from "../logger.js";
import { JointMapper, type StarMapping } from "../mapping.js";
import type { Product } from "../motor.js";
import type { SerialPortLike } from "../transport/serial.js";
import { type Leader, LeaderError } from "./base.js";
import { FashionStarBus } from "./fashionstar.js";

/** s, between per-servo unlocks. */
export const SETTLE_S = 0.01;
/** The Star Arm 102's UART rate. */
export const BAUDRATE = 1_000_000;

/** The servo bus calls StarLeader makes (FashionStarBus has them). */
export interface ServoBus {
  ping(id: number, signal?: AbortSignal): Promise<boolean>;
  unlock(id: number, signal?: AbortSignal): Promise<void>;
  resetMultiTurn(id: number, signal?: AbortSignal): Promise<void>;
  setOriginPoint(id: number, signal?: AbortSignal): Promise<void>;
  /** Servo id -> its monitor (`angleDeg`, multi-turn degrees), null or absent when it has none. */
  syncMonitor(ids: readonly number[], signal?: AbortSignal):
    Promise<ReadonlyMap<number, { readonly angleDeg: number } | null | undefined>>;
  close(): Promise<void>;
}

/** Opens the bus on `port` (default: FashionStarBus.open). */
export type ServoBusFactory = (port: SerialPortLike, baudrate: number) => ServoBus | Promise<ServoBus>;

/** Options of the StarLeader constructor. */
export interface StarLeaderOptions {
  /** The Star mapping (default: `product.defaultStarMapping`). */
  mapping?: JointMapper | StarMapping | null;
  /**
   * The follower's product: supplies the default mapping, and a mapping's joint names must be its
   * profile's joint names, in order.
   */
  product?: Product | null;
  /** Default BAUDRATE. */
  baudrate?: number;
  /** Consecutive missed reads of one servo that end the session (default 50). */
  missLimit?: number;
  busFactory?: ServoBusFactory | null;
  /** Default: a RealClock. */
  clock?: Clock;
  /** Default: console. */
  logger?: Logger;
}

const errorText = (e: unknown): string => (e instanceof Error ? e.message : String(e));
const list = (ids: readonly number[]): string => `[${ids.join(", ")}]`;

/** A port in messages: its USB ids when it reports them. */
function portLabel(port: SerialPortLike): string {
  const info = port.getInfo?.();
  if (info?.usbVendorId === undefined || info.usbProductId === undefined) return "the serial port";
  const hex = (n: number): string => n.toString(16).toUpperCase().padStart(4, "0");
  return `usb:${hex(info.usbVendorId)}:${hex(info.usbProductId)}`;
}

/** The Star Arm 102 as a leader. */
export class StarLeader implements Leader {
  readonly name = "star";
  readonly mapper: JointMapper;
  readonly port: SerialPortLike;
  readonly baudrate: number;
  readonly missLimit: number;
  readonly clock: Clock;
  bus: ServoBus | null = null;
  /** Servo id -> consecutive missed reads. */
  misses: Map<number, number>;
  /** The last bus error a read swallowed, or null. */
  lastError: string | null = null;
  #factory: ServoBusFactory;
  #log: Logger;

  /**
   * Throws ValueError without a mapping or a product, for an invalid mapping, for a mapping whose
   * joints are not the product profile's (names, in order), or when missLimit < 1.
   */
  constructor(port: SerialPortLike, options: StarLeaderOptions = {}) {
    let mapping = options.mapping ?? null;
    if (mapping === null) {
      if (!options.product) throw new ValueError("StarLeader needs a mapping or the follower's product");
      mapping = options.product.defaultStarMapping;
    }
    this.mapper = mapping instanceof JointMapper ? mapping : JointMapper.fromJson(mapping);
    if (options.product) {
      const joints = (options.product.defaultProfile as { joints?: Array<{ name?: unknown }> }).joints ?? [];
      const expected = joints.map((j) => String(j.name ?? ""));
      const names = this.mapper.names;
      if (names.length !== expected.length || names.some((n, i) => n !== expected[i])) {
        throw new ValueError(`the Star mapping's joints [${names.join(", ")}] are not the ${options.product.displayName}'s `
          + `[${expected.join(", ")}]`);
      }
    }
    this.port = port;
    this.baudrate = options.baudrate ?? BAUDRATE;
    const missLimit = options.missLimit ?? 50;
    if (missLimit < 1) throw new ValueError("miss_limit must be >= 1");
    this.missLimit = missLimit;
    this.clock = options.clock ?? new RealClock();
    this.#log = options.logger ?? console;
    this.#factory = options.busFactory ?? ((p, baudrate) => FashionStarBus.open(p, { baudrate, clock: this.clock }));
    this.misses = new Map(this.mapper.servoIds.map((sid) => [sid, 0]));
  }

  get servoIds(): number[] {
    return this.mapper.servoIds;
  }

  // ── lifecycle ──
  /**
   * Open the port and unlock the servos (nothing here energizes anything; `signal` cuts the
   * connect short). Throws LeaderError when a servo does not answer, after closing the port.
   */
  async connect(signal?: AbortSignal): Promise<void> {
    if (this.bus !== null) return;
    const bus = await this.#factory(this.port, this.baudrate);
    try {
      const silent: number[] = [];
      for (const sid of this.servoIds) if (!(await bus.ping(sid, signal))) silent.push(sid);
      if (silent.length) {
        throw new LeaderError(`Star leader servos ${list(silent)} did not answer on ${portLabel(this.port)} -- check the port and ids`);
      }
      for (const sid of this.servoIds) {
        await bus.unlock(sid, signal);
        await this.clock.sleep(SETTLE_S);
      }
      for (const sid of this.servoIds) await bus.resetMultiTurn(sid, signal);
    } catch (e) {
      await bus.close();
      throw e;
    }
    this.bus = bus;
    this.mapper.reset();
    this.misses = new Map(this.servoIds.map((sid) => [sid, 0]));
  }

  async close(): Promise<void> {
    const bus = this.bus;
    this.bus = null;
    if (bus !== null) {
      try {
        await bus.close();
      } catch (e) { // a USB dropout makes close() fail; the port is gone either way
        this.#log.warn(`closing the Star leader port failed: ${errorText(e)}`);
      }
    }
  }

  // ── reading ──
  /** One sync read: servo id -> multi-turn degrees, for the servos that replied. `signal` cuts it short. */
  async readRaw(signal?: AbortSignal): Promise<Map<number, number>> {
    if (this.bus === null) throw new LeaderError("Star leader is not connected");
    const data = await this.bus.syncMonitor(this.servoIds, signal);
    const out = new Map<number, number>();
    for (const sid of this.servoIds) {
      const m = data.get(sid);
      if (m !== null && m !== undefined && Number.isFinite(m.angleDeg)) out.set(sid, Number(m.angleDeg));
    }
    return out;
  }

  /**
   * Follower targets (rad), or null when no servo replied or a joint has never had a reading. An
   * abort of `signal` ends the read with an error and counts no miss.
   */
  async read(signal?: AbortSignal): Promise<number[] | null> {
    let raw: Map<number, number>;
    try {
      raw = await this.readRaw(signal);
    } catch (e) {
      if (e instanceof LeaderError || signal?.aborted) throw e;
      this.lastError = errorText(e);
      raw = new Map();
    }
    for (const sid of this.servoIds) this.misses.set(sid, raw.has(sid) ? 0 : this.misses.get(sid)! + 1);
    const lost = [...this.misses].filter(([, count]) => count >= this.missLimit).map(([sid]) => sid);
    if (lost.length) {
      throw new LeaderError(`Star leader servos ${list(lost)} missed ${this.missLimit} reads in a row`
        + (this.lastError ? ` (last error: ${this.lastError})` : ""));
    }
    if (!raw.size) return null;
    const targets = this.mapper.map(raw);
    return targets.every((t) => Number.isFinite(t)) ? targets : null;
  }

  // ── calibration ──
  /** Make the present pose every servo's origin (the leader must be at the zero pose). */
  async zero(): Promise<void> {
    if (this.bus === null) throw new LeaderError("Star leader is not connected");
    for (const sid of this.servoIds) {
      await this.bus.unlock(sid);
      await this.clock.sleep(SETTLE_S);
      await this.bus.setOriginPoint(sid);
    }
    for (const sid of this.servoIds) await this.bus.resetMultiTurn(sid);
    this.mapper.reset();
  }
}
