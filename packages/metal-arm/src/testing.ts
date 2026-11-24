/**
 * Simulated Metal Arms for application tests, no hardware: a real Arm on a MockBackend whose
 * simulated Damiao motors track MIT commands, report status and temperatures, and take injected
 * faults.
 *
 *     const [arm, sim] = simulatedArm();     // shipped profile, each joint at zero clipped into its limits
 *     await arm.connect();
 *     await arm.enable();
 *     await arm.moveTo(q);
 *     sim.positions;                          // where the simulated joints are (rad)
 *     sim.fault("gripper"); sim.setTemperature("elbow_flex", 80.0); sim.offline("wrist_roll");
 *
 * Simulated arms use simulationTiming(config): the same profile with a longer host-side feedback
 * deadline and a lower control rate, and each tick advances its commands by the time that passed
 * (up to a few periods) rather than one period, so late wake-ups do not leave it behind its
 * targets. Pass a VirtualClock as `clock` to run without real time.
 */

import { type Arm, ArmConfig, type CanBackend, type JointConfig } from "@makermods/arm-core";
import {
  SIM_CONTROL_RATE_HZ, SIM_FEEDBACK_TIMEOUT, SIM_MAX_TICK_PERIODS, type SimulatedArmOptions, type SimulatedBackendOptions,
  type Simulation, SimulatedMotors, simulatedArm as coreSimulatedArm, simulatedBackend as coreSimulatedBackend,
  type SimulatedGroupOptions, simulatedGroup as coreSimulatedGroup, simulationTiming, VirtualClock,
} from "@makermods/arm-core/testing";
import { METAL_ARM } from "./product.js";
import { forJoint, SimBus, SimMotor, type SimMotorOptions, watchdogTimeout } from "./sim-motor.js";

export {
  SIM_CONTROL_RATE_HZ, SIM_FEEDBACK_TIMEOUT, SIM_MAX_TICK_PERIODS, SimulatedMotors, simulationTiming, VirtualClock,
};
export {
  classify, FAULT_CODE, FAULT_TEXT, feedbackBytes, type FeedbackBytesOptions, forJoint, type FrameKind,
  FULL_TURN_SUPPORTED, registerReply, respond, SimBus, type SimLogEntry, SimMotor, type SimMotorOptions,
  WARNING_SUPPORTED, watchdogTimeout,
} from "./sim-motor.js";

/** The Metal Arm's simulated motors, for the core's simulated-arm helpers. */
export const SIMULATION: Simulation<SimMotor, SimMotorOptions> = {
  defaultConfig: () => ArmConfig.fromRaw(METAL_ARM.defaultProfile, METAL_ARM),
  forJoint: (joint: JointConfig, hostId: number, position: number, options: SimMotorOptions) =>
    forJoint(joint, hostId, position, options),
  bus: (motors: SimMotor[]): CanBackend => new SimBus(motors),
  watchdogTimeout,
};

/**
 * [backend, SimulatedMotors]: a SimBus with one simulated motor per joint of `options.config`
 * (default: the shipped profile) at `options.positions` (rad, default zero clipped into each
 * joint's limits), its timing relaxed by simulationTiming(). `options.motor` goes to every
 * simulated motor (e.g. `{ temperature: 40.0 }`).
 */
export function simulatedBackend(options: SimulatedBackendOptions<SimMotorOptions> = {}): [SimBus, SimulatedMotors<SimMotor>] {
  const [backend, sim] = coreSimulatedBackend(SIMULATION, options);
  return [backend as SimBus, sim];
}

/** [arm, sim]: a real Arm, not connected, on simulatedBackend(options). */
export function simulatedArm(options: SimulatedArmOptions<SimMotorOptions> = {}): [Arm, SimulatedMotors<SimMotor>] {
  return coreSimulatedArm(SIMULATION, options);
}

/**
 * [group, Map(name -> sim)]: the rig's arms each with its profile on its own simulated bus; the
 * rig's backends and addresses are ignored. `rig` is a Rig or the contents of a rig file (an object
 * or JSON text).
 *
 *     const [group, sims] = await simulatedGroup({ arms: { left: { backend: "gs_usb" }, right: { backend: "gs_usb" } } });
 *     await group.open();
 *     try { await group.enable(); } finally { await group.close(); }
 */
export function simulatedGroup(rig: unknown, options: SimulatedGroupOptions<SimMotorOptions> = {}) {
  return coreSimulatedGroup(SIMULATION, rig, options);
}
