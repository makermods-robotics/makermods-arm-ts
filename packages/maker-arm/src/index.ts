/** TypeScript SDK for the MakerMods Maker Arm: 7 RobStride motors on the MIT protocol over CAN. */

import { Arm, type ArmOptions, ArmConfig, type CanBackend, type RawProfile } from "@makermods/arm-core";
import { MAKER_ARM } from "./product.js";

export * from "@makermods/arm-core";
export * as protocol from "./protocol.js";
export { MAKER_ARM } from "./product.js";
export {
  DEFAULT_HOST_ID, DEFAULT_MODEL, ID_RANGE, ID_SETTLE_S, KD_RANGE, KP_RANGE, MODELS, PARAMS, type RobStrideFeedback,
  RobStrideMotor, route, SET_ID_MAX_TORQUE_NM, SET_ID_MAX_VELOCITY,
} from "./motor.js";

/**
 * The shipped profile as a validated ArmConfig. `overrides` are raw profile keys (snake_case, as
 * in the YAML profiles, `_deg` forms allowed) merged over the shipped profile's top level.
 */
export function defaultConfig(overrides: RawProfile = {}): ArmConfig {
  return ArmConfig.fromRaw({ ...MAKER_ARM.defaultProfile, ...overrides }, MAKER_ARM);
}

/** An Arm with the shipped profile on `backend`. */
export function createArm(backend: CanBackend, options: ArmOptions & { config?: ArmConfig } = {}): Arm {
  const { config, ...rest } = options;
  return new Arm(config ?? defaultConfig(), backend, rest);
}
