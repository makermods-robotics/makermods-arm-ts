/** TypeScript SDK for the MakerMods Metal Arm: 7 Damiao motors on the MIT protocol over CAN. */

import { Arm, type ArmOptions, ArmConfig, type CanBackend, type RawProfile } from "@makermods/arm-core";
import { METAL_ARM } from "./product.js";

export * from "@makermods/arm-core";
/** The Damiao wire codec. */
export * as protocol from "./protocol.js";
export { DREAM_EMBODIMENT, DREAM_JOINT_OFFSET, DREAM_JOINT_SIGN, METAL_ARM, ZERO_POSE } from "./product.js";
export {
  DEFAULT_HOST_ID, DEFAULT_MODEL, FLUSH_AFTER_S, FLUSHES_PER_ATTEMPT, ID_RANGE, KD_RANGE, KP_RANGE, MODELS,
  Motor as DamiaoMotor, type MotorFeedback as DamiaoFeedback, route, SAVE_SETTLE_S,
} from "./motor.js";

/**
 * The shipped profile as a validated ArmConfig. `overrides` are raw profile keys (snake_case, as
 * in the YAML profiles, `_deg` forms allowed) merged over the shipped profile's top level.
 */
export function defaultConfig(overrides: RawProfile = {}): ArmConfig {
  return ArmConfig.fromRaw({ ...METAL_ARM.defaultProfile, ...overrides }, METAL_ARM);
}

/** An Arm with the shipped profile on `backend`. */
export function createArm(backend: CanBackend, options: ArmOptions & { config?: ArmConfig } = {}): Arm {
  const { config, ...rest } = options;
  return new Arm(config ?? defaultConfig(), backend, rest);
}
