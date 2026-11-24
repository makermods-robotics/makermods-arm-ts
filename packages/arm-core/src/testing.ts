/**
 * Test support: virtual time, and the product-neutral parts of simulated arms. Each arm package's
 * own "/testing" entry supplies its simulated motors.
 */

import { Arm, type Logger } from "./arm.js";
import type { Clock } from "./clock.js";
import type { ArmConfig, JointConfig } from "./config.js";
import { ValueError } from "./errors.js";
import { ArmGroup, Rig, type RigOptions } from "./rig.js";
import type { CanBackend } from "./transport/base.js";

interface Timer {
  at: number;
  order: number;
  fn: () => void;
}

const later: (fn: () => void) => void =
  typeof (globalThis as { setImmediate?: unknown }).setImmediate === "function"
    ? (fn) => (globalThis as unknown as { setImmediate: (f: () => void) => void }).setImmediate(fn)
    : (fn) => setTimeout(fn, 0);

export interface VirtualClockOptions {
  start?: number;
  /**
   * true (default): whenever every task is waiting, time jumps to the earliest timer, so async
   * code runs to completion without real delays. false: time moves only in advance().
   */
  auto?: boolean;
}

/**
 * A clock that never waits for real time. Timers fire in deadline order, one per event-loop turn,
 * so everything a timer wakes has run before the next one fires. A sleep of x seconds moves the
 * time by exactly x.
 */
export class VirtualClock implements Clock {
  #now: number;
  #auto: boolean;
  #timers: Timer[] = [];
  #order = 0;
  #pumping = false;

  constructor(options: VirtualClockOptions = {}) {
    this.#now = options.start ?? 0;
    this.#auto = options.auto ?? true;
  }

  now(): number {
    return this.#now;
  }

  timer(seconds: number, fn: () => void): () => void {
    const timer: Timer = { at: this.#now + Math.max(0, seconds), order: this.#order++, fn };
    this.#timers.push(timer);
    if (this.#auto) this.#pump();
    return () => {
      const i = this.#timers.indexOf(timer);
      if (i >= 0) this.#timers.splice(i, 1);
    };
  }

  sleep(seconds: number): Promise<void> {
    return new Promise<void>((resolve) => this.timer(seconds, resolve));
  }

  /** Timers still waiting. */
  get pending(): number {
    return this.#timers.length;
  }

  /** Move time forward by `seconds`, firing the timers that come due in order. */
  async advance(seconds: number): Promise<void> {
    const end = this.#now + seconds;
    for (;;) {
      await new Promise<void>((resolve) => later(resolve));
      const next = this.#earliest();
      if (!next || next.at > end) break;
      this.#fire(next);
    }
    this.#now = Math.max(this.#now, end);
    await new Promise<void>((resolve) => later(resolve));
  }

  #earliest(): Timer | undefined {
    let best: Timer | undefined;
    for (const t of this.#timers) {
      if (!best || t.at < best.at || (t.at === best.at && t.order < best.order)) best = t;
    }
    return best;
  }

  #fire(timer: Timer): void {
    this.#timers.splice(this.#timers.indexOf(timer), 1);
    this.#now = Math.max(this.#now, timer.at);
    timer.fn();
  }

  #pump(): void {
    if (this.#pumping) return;
    this.#pumping = true;
    later(() => {
      this.#pumping = false;
      const next = this.#earliest();
      if (next) this.#fire(next);
      if (this.#timers.length) this.#pump();
    });
  }
}

// ── Simulated arms (product-neutral; each arm package supplies its Simulation) ──

/** s, host-side feedback deadline for simulated motors (at least). */
export const SIM_FEEDBACK_TIMEOUT = 1.0;
/** Control loop rate for simulated motors (at most). */
export const SIM_CONTROL_RATE_HZ = 50.0;
/** A late tick of a simulated arm advances by at most this many periods. */
export const SIM_MAX_TICK_PERIODS = 4;

/** What the core needs to see of one simulated motor. */
export interface SimulatedMotor {
  /** Motor coordinates, rad. */
  position: number;
  temperature: number;
  online: boolean;
  enabled: boolean;
  /** Reports a fault (false: clear it). */
  faulted: boolean;
}

/**
 * What an arm package implements so the core can build simulated arms of its product: simulated
 * motors that answer its protocol on a MockBackend.
 */
export interface Simulation<M extends SimulatedMotor = SimulatedMotor, O = Record<string, unknown>> {
  /** The product's shipped profile, validated. */
  defaultConfig(): ArmConfig;
  /** A simulated motor answering like the motor of `joint` at `position` (motor rad); `options` from the caller. */
  forJoint(joint: JointConfig, hostId: number, position: number, options: O): M;
  /** A backend whose replies come from `motors`. */
  bus(motors: M[]): CanBackend;
  /** What the motor's own watchdog does once commands stop (torque off, and the state the protocol reports for it). */
  watchdogTimeout(motor: M): void;
}

/** The simulated motors behind one arm, addressed by joint name or index. */
export class SimulatedMotors<M extends SimulatedMotor = SimulatedMotor> {
  readonly config: ArmConfig;
  /** Joint name -> simulated motor. */
  readonly motors: Map<string, M>;
  #watchdogTimeout: (motor: M) => void;

  constructor(config: ArmConfig, motors: M[], watchdogTimeout: (motor: M) => void) {
    this.config = config;
    this.motors = new Map(config.joints.map((j, i) => [j.label, motors[i]!]));
    this.#watchdogTimeout = watchdogTimeout;
  }

  motor(joint: string | number): M {
    if (typeof joint === "number") {
      const m = [...this.motors.values()][joint];
      if (m === undefined) throw new RangeError(`no joint ${joint} (joints: ${[...this.motors.keys()].join(", ")})`);
      return m;
    }
    const m = this.motors.get(joint);
    if (m === undefined) throw new RangeError(`no joint '${joint}' (joints: ${[...this.motors.keys()].join(", ")})`);
    return m;
  }

  /** Joint positions of the simulated motors, rad. */
  get positions(): number[] {
    const motors = [...this.motors.values()];
    return this.config.joints.map((j, i) => (motors[i]!.position - j.offset) * j.direction);
  }

  /** Make the motor report a fault (false: clear it). */
  fault(joint: string | number, faulted = true): void {
    this.motor(joint).faulted = faulted;
  }

  setTemperature(joint: string | number, celsius: number): void {
    this.motor(joint).temperature = celsius;
  }

  /** Stop the motor answering (false: answer again). */
  offline(joint: string | number, offline = true): void {
    this.motor(joint).online = !offline;
  }

  /** Do what the motor's watchdog does once commands stop (torque off, and the state the protocol reports for it). */
  watchdogTimeout(joint: string | number): void {
    this.#watchdogTimeout(this.motor(joint));
  }
}

/**
 * A copy of `config` whose feedback deadline is at least SIM_FEEDBACK_TIMEOUT and control rate at
 * most SIM_CONTROL_RATE_HZ (the per-tick startup step scaled to keep its speed), for simulated
 * motors on a machine that may starve the control loop.
 */
export function simulationTiming(config: ArmConfig): ArmConfig {
  const rate = Math.min(config.controlRateHz, SIM_CONTROL_RATE_HZ);
  return config.replace({
    feedbackTimeout: Math.max(config.feedbackTimeout, SIM_FEEDBACK_TIMEOUT),
    controlRateHz: rate,
    startupStep: config.startupStep * config.controlRateHz / rate, // same rad/s
  });
}

export interface SimulatedBackendOptions<O> {
  /** Default: the product's shipped profile. */
  config?: ArmConfig | null;
  /** rad per joint (default: zero clipped into each joint's limits). */
  positions?: readonly number[] | null;
  /** Passed to every simulated motor (e.g. a temperature). */
  motor?: O;
}

export interface SimulatedArmOptions<O> extends SimulatedBackendOptions<O> {
  /** Default: a RealClock. */
  clock?: Clock;
  logger?: Logger;
}

/**
 * [backend, SimulatedMotors]: a backend with one simulated motor per joint of `options.config` at
 * `options.positions`, its timing relaxed by simulationTiming().
 */
export function simulatedBackend<M extends SimulatedMotor, O>(simulation: Simulation<M, O>,
                                                            options: SimulatedBackendOptions<O> = {}):
    [CanBackend, SimulatedMotors<M>] {
  const config = simulationTiming(options.config ?? simulation.defaultConfig());
  const joints = config.joints;
  const positions = options.positions ?? joints.map((j) => Math.min(j.hi, Math.max(j.lo, 0.0)));
  if (positions.length !== joints.length) {
    throw new ValueError(`positions needs one value per joint (${joints.length}), got ${positions.length}`);
  }
  const motors = joints.map((j, i) => simulation.forJoint(j, config.hostId, positions[i]! * j.direction + j.offset,
    options.motor ?? ({} as O)));
  return [simulation.bus(motors), new SimulatedMotors(config, motors, (m) => simulation.watchdogTimeout(m))];
}

/** [arm, sim]: a real Arm, not connected, on simulatedBackend(simulation, options). */
export function simulatedArm<M extends SimulatedMotor, O>(simulation: Simulation<M, O>, options: SimulatedArmOptions<O> = {}):
    [Arm, SimulatedMotors<M>] {
  const [backend, sim] = simulatedBackend(simulation, options);
  // Frame spacing is for real buses.
  const arm = new Arm(sim.config, backend, { clock: options.clock, logger: options.logger, frameSpacing: 0 });
  followTheClock(arm);
  return [arm, sim];
}

export interface SimulatedGroupOptions<O> {
  /** Arm names (default: all). */
  names?: Iterable<string>;
  /** Starting positions (rad) by arm name. */
  positions?: Readonly<Record<string, readonly number[]>> | ReadonlyMap<string, readonly number[]> | null;
  /** Passed to every simulated motor. */
  motor?: O;
  /** For every arm (default: a RealClock each). */
  clock?: Clock;
  logger?: Logger;
  /** How a rig description (not a Rig) is read: its name for messages, and `files` for profiles named by path. */
  rigOptions?: RigOptions;
}

/**
 * [group, {name: sim}]: the rig's arms (`rig.group(names)`) each with its profile on its own
 * simulated bus; the rig's backends and addresses are ignored. `rig` is a Rig or the contents of a
 * rig file (an object or JSON text); `options.positions` maps arm names to starting positions.
 */
export async function simulatedGroup<M extends SimulatedMotor, O>(simulation: Simulation<M, O>, rig: Rig | unknown,
                                                                  options: SimulatedGroupOptions<O> = {}):
    Promise<[ArmGroup, Map<string, SimulatedMotors<M>>]> {
  const theRig = rig instanceof Rig ? rig : Rig.fromRaw(rig, simulation.defaultConfig().product, options.rigOptions);
  const positions = options.positions;
  const arms = new Map<string, Arm>();
  const sims = new Map<string, SimulatedMotors<M>>();
  for (const name of theRig.names(options.names ?? ["all"])) {
    const start = positions instanceof Map ? positions.get(name) : (positions as Record<string, readonly number[]> | null | undefined)?.[name];
    const [arm, sim] = simulatedArm(simulation, {
      config: await theRig.config(name), positions: start ?? null, motor: options.motor, clock: options.clock, logger: options.logger,
    });
    arms.set(name, arm);
    sims.set(name, sim);
  }
  return [new ArmGroup(arms, true, options.logger), sims];
}

/**
 * Each control tick of `arm` advances its commands by the time since the previous tick, between
 * one and SIM_MAX_TICK_PERIODS periods.
 */
function followTheClock(arm: Arm): void {
  const tick = arm["tick"].bind(arm);
  let last: number | null = null;
  arm["tick"] = (dt: number, session: number | null = null): Promise<void> => {
    const now = arm.clock.now();
    const elapsed = last === null ? dt : now - last;
    last = now;
    return tick(Math.min(Math.max(dt, elapsed), SIM_MAX_TICK_PERIODS * dt), session);
  };
}

export {
  type FakeBusFrame, type FakeBusResponder, type FakeControl, FakeGsUsbDevice, type FakeGsUsbDeviceOptions, type FakeWrite,
} from "./transport/fake-gs-usb.js";
export {
  FakeSerialPort, type FakeSerialPortOptions, type FakeSerialWrite, type FakeSlcanFrame,
} from "./transport/fake-serial.js";
export {
  SimulatedServo, simulatedStarLeader, type SimulatedStarLeaderOptions, SimulatedStarPort,
} from "./leaders/sim.js";
