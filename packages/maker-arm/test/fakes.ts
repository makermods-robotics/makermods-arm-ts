// Shared test arm (the Python SDK's tests/fakes.py): simulated RobStride motors on a SimBus, the
// two-joint test profile, and make_arm(). Everything runs on a VirtualClock.
import { Arm, ArmConfig, type ArmConfigInit, JointConfig, MAKER_ARM, defaultConfig } from "@makermods/maker-arm";
import {
  classify, FAULT_CODE, FAULT_TEXT, FULL_TURN_SUPPORTED, SimBus, SimMotor, simulatedArm, VirtualClock, WARNING_SUPPORTED,
} from "@makermods/maker-arm/testing";
import { type ArmFakes, RecordingLogger } from "../../arm-core/test/shared/fakes.js";

export { commands, RecordingLogger, waitFor } from "../../arm-core/test/shared/fakes.js";

export {
  classify, FAULT_CODE, FAULT_TEXT, feedbackBytes, FULL_TURN_SUPPORTED, PREFIX, respond, SimBus as FakeBus, SimMotor as FakeMotor,
  WARNING_SUPPORTED,
} from "@makermods/maker-arm/testing";

export function twoJointConfig(overrides: Partial<ArmConfigInit> = {}): ArmConfig {
  return new ArmConfig({
    product: MAKER_ARM, controlRateHz: 200, maxVelocity: 50.0,
    joints: [
      new JointConfig({ motorId: 1, name: "j1", lo: -2.0, hi: 2.0, kp: 60, kd: 2.0, model: "RS00" }),
      new JointConfig({
        motorId: 2, name: "gripper", lo: -1.6, hi: -0.04, kp: 20, kd: 0.5, model: "RS00", canTimeoutMs: 500,
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
  const list = motors ?? [new SimMotor(1, "RS00", { position: 0.3 }), new SimMotor(2, "RS00", { position: -0.5 })];
  const bus = new SimBus(list);
  const logger = new RecordingLogger();
  const arm = new Arm(twoJointConfig(overrides), bus, { clock, logger, frameSpacing: 0 });
  arm.PARAM_TIMEOUT = 0.005;
  return { arm, bus, fakes: Object.fromEntries(list.map((m) => [m.motorId, m])), clock, logger };
}

/** The Maker Arm's fakes for the shared arm suites (packages/arm-core/test/shared/). */
export const FAKES: ArmFakes<SimMotor> = {
  name: "maker",
  product: MAKER_ARM,
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
