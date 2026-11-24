/**
 * What the shared arm test suites need from an arm package's test fakes (the Python SDKs'
 * tests/fakes.py, whose shared tests are byte-identical in both repos): simulated motors and their
 * bus, the two-joint test profile, makeArm(), the capability flags, and the package's simulated
 * arm. Each package's test/fakes.ts builds one and hands it to the suites.
 */
import type { Arm, ArmConfig, ArmConfigInit, JointConfig, Logger, MockBackend, Product } from "@makermods/arm-core";
import type { SimulatedArmOptions, SimulatedMotor, SimulatedMotors, VirtualClock } from "@makermods/arm-core/testing";

/** The generic surface of a simulated motor that the shared tests use. */
export interface FakeMotorLike extends SimulatedMotor {
  motorId: number;
  track: boolean;
  warning: boolean;
  lastKp: number;
  /** "status_query", "clear", "enable", "disable", "zero", ["cmd", kp], ... */
  log: Array<string | [string, number]>;
  /** [position, kp, tau] of every MIT command. */
  frames: Array<[number, number, number]>;
  handle(canId: number, data: Uint8Array): Array<[number, Uint8Array]>;
}

/** Options every product's simulated motor accepts. */
export interface FakeMotorOptions {
  position?: number;
  temperature?: number;
  enabled?: boolean;
  online?: boolean;
  track?: boolean;
  faulted?: boolean;
  warning?: boolean;
  paramsSupported?: boolean;
}

export interface Made<M extends FakeMotorLike = FakeMotorLike> {
  arm: Arm;
  bus: MockBackend;
  /** motor id -> simulated motor. */
  fakes: Record<number, M>;
  clock: VirtualClock;
  logger: RecordingLogger;
}

export interface ArmFakes<M extends FakeMotorLike = FakeMotorLike> {
  /** Label for the suites ("maker", "metal"). */
  name: string;
  product: Product;
  /** new FakeMotor(motorId, model?, options?) */
  FakeMotor: new (motorId: number, model?: string, options?: FakeMotorOptions) => M;
  FakeBus: new (motors: M[]) => MockBackend;
  classify(canId: number, data: Uint8Array): [string, number | null];
  FAULT_CODE: number;
  FAULT_TEXT: string;
  FULL_TURN_SUPPORTED: boolean;
  WARNING_SUPPORTED: boolean;
  twoJointConfig(overrides?: Partial<ArmConfigInit>): ArmConfig;
  makeArm(motors?: M[] | null, overrides?: Partial<ArmConfigInit>, clock?: VirtualClock): Made<M>;
  /** The shipped profile, validated. */
  defaultConfig(): ArmConfig;
  simulatedArm(options?: SimulatedArmOptions<FakeMotorOptions>): [Arm, SimulatedMotors<M>];
  /** Source files of the package's and the core's testing entries (must not use the test framework). */
  testingSources: URL[];
}

/** Records what the arm logs; tests read `messages`. */
export class RecordingLogger implements Logger {
  messages: Array<[string, string]> = [];
  info(message: string): void {
    this.messages.push(["info", message]);
  }
  warn(message: string): void {
    this.messages.push(["warn", message]);
  }
  error(message: string): void {
    this.messages.push(["error", message]);
  }
}

/** Polls `predicate` every 5 ms of clock time for up to `seconds`. */
export async function waitFor(predicate: () => boolean, clock: VirtualClock, seconds = 2.0): Promise<boolean> {
  const deadline = clock.now() + seconds;
  while (clock.now() < deadline && !predicate()) await clock.sleep(0.005);
  return predicate();
}

/** The ["cmd", kp] entries of a simulated motor's log. */
export function commands(fake: FakeMotorLike): Array<[string, number]> {
  return fake.log.filter((e): e is [string, number] => Array.isArray(e) && e[0] === "cmd");
}

export type { JointConfig };
