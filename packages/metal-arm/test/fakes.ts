// Shared test arm (the Python SDK's tests/fakes.py): simulated Damiao motors on a SimBus, the
// two-joint test profile, makeArm(), and the Damiao helpers makeJoint() / wired(). Everything runs
// on a VirtualClock.
import {
  Arm, ArmConfig, type ArmConfigInit, DamiaoMotor, defaultConfig, JointConfig, type JointConfigInit, METAL_ARM, protocol,
  route,
} from "@makermods/metal-arm";
import {
  classify, FAULT_CODE, FAULT_TEXT, FULL_TURN_SUPPORTED, SimBus, SimMotor, simulatedArm, VirtualClock, WARNING_SUPPORTED,
} from "@makermods/metal-arm/testing";
import { type ArmFakes, RecordingLogger } from "../../arm-core/test/shared/fakes.js";

export { commands, RecordingLogger, waitFor } from "../../arm-core/test/shared/fakes.js";

export {
  classify, FAULT_CODE, FAULT_TEXT, feedbackBytes, FULL_TURN_SUPPORTED, registerReply, respond, SimBus as FakeBus,
  SimMotor as FakeMotor, WARNING_SUPPORTED,
} from "@makermods/metal-arm/testing";

/** A JointConfig for one motor. */
export function makeJoint(motorId: number, model = "jlo", name = "", extra: Partial<JointConfigInit> = {}): JointConfig {
  return new JointConfig({ motorId, model, name, lo: -1.0, hi: 1.0, kp: 20.0, kd: 0.6, ...extra });
}

/**
 * SimBus with these motors and a Motor for each, dispatched the way the arm does it (route then
 * handleFrame), on a VirtualClock. `motor` is the first fake's Motor.
 */
export function wired(fakes: SimMotor[], hostId: number = protocol.DEFAULT_MASTER_OFFSET, joint: Partial<JointConfigInit> = {}) {
  const clock = new VirtualClock();
  const bus = new SimBus(fakes);
  const motors = new Map<number, DamiaoMotor>();
  for (const f of fakes) motors.set(f.motorId, new DamiaoMotor(makeJoint(f.motorId, f.model, "", joint), bus, hostId, clock));
  bus.setRecvCallback((canId, data, extended) => {
    const id = route(canId, data, hostId, extended);
    if (id !== null) motors.get(id)?.handleFrame(canId, data, extended);
  });
  return { motors, bus, clock, motor: motors.get(fakes[0]!.motorId)! };
}

/**
 * The shared two-joint test arm (same limits/gains/protection as the other SDK's), both joints on
 * the default model so new FakeMotor(n) scales like the joint.
 */
export function twoJointConfig(overrides: Partial<ArmConfigInit> = {}): ArmConfig {
  return new ArmConfig({
    product: METAL_ARM, controlRateHz: 200, maxVelocity: 50.0,
    joints: [
      new JointConfig({ motorId: 1, name: "j1", lo: -2.0, hi: 2.0, kp: 60, kd: 2.0, model: METAL_ARM.defaultModel }),
      new JointConfig({
        motorId: 2, name: "gripper", lo: -1.6, hi: -0.04, kp: 20, kd: 0.5, model: METAL_ARM.defaultModel, canTimeoutMs: 500,
        maxTemperatureC: 55, enableTemperatureC: 45, returnToRest: false, releaseOnFault: true,
      }),
    ],
    ...overrides,
  });
}

export interface Made {
  arm: Arm;
  bus: SimBus;
  /** motor id -> simulated motor. */
  fakes: Record<number, SimMotor>;
  clock: VirtualClock;
  logger: RecordingLogger;
}

/**
 * The arm, its bus and simulated motors for twoJointConfig(overrides); default motors sit at 0.3 and
 * -0.5 rad. Frame spacing is off and PARAM_TIMEOUT is 0.005 s, as in the Python tests.
 */
export function makeArm(motors?: SimMotor[] | null, overrides: Partial<ArmConfigInit> = {},
                        clock: VirtualClock = new VirtualClock()): Made {
  const list = motors ?? [new SimMotor(1, "jlo", { position: 0.3 }), new SimMotor(2, "jlo", { position: -0.5 })];
  const bus = new SimBus(list);
  const logger = new RecordingLogger();
  const arm = new Arm(twoJointConfig(overrides), bus, { clock, logger, frameSpacing: 0 });
  arm.PARAM_TIMEOUT = 0.005;
  return { arm, bus, fakes: Object.fromEntries(list.map((m) => [m.motorId, m])), clock, logger };
}

/** The Metal Arm's fakes for the shared arm suites (packages/arm-core/test/shared/). */
export const FAKES: ArmFakes<SimMotor> = {
  name: "metal",
  product: METAL_ARM,
  FakeMotor: SimMotor,
  FakeBus: SimBus,
  classify,
  FAULT_CODE,
  FAULT_TEXT,
  FULL_TURN_SUPPORTED,
  WARNING_SUPPORTED,
  twoJointConfig,
  makeArm,
  defaultConfig: () => defaultConfig(),
  simulatedArm: (options = {}) => simulatedArm(options),
  testingSources: [new URL("../src/testing.ts", import.meta.url), new URL("../../arm-core/src/testing.ts", import.meta.url)],
};
