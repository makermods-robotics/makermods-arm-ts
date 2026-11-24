/** Product identity: names, shipped profile and URDF of this arm, and its motor implementation. */

import type { Product } from "@makermods/arm-core";
import { PROFILE } from "./generated/profile.js";
import { STAR_MAPPING } from "./generated/star-mapping.js";
import { URDF } from "./generated/urdf.js";
import { DEFAULT_HOST_ID, DEFAULT_MODEL, ID_RANGE, KD_RANGE, KP_RANGE, MODELS, Motor, route } from "./motor.js";

/** The pose the motors' zero is set at. */
export const ZERO_POSE = "folded at rest, as the arm sits unpowered, gripper fully closed";

/**
 * dream_ik embodiment of this arm and its joints from this package's:
 * embodiment joint = joint * sign + offset (rad). Its URDF takes the joints as the arm reports them.
 */
export const DREAM_EMBODIMENT = "metal";
export const DREAM_JOINT_SIGN: readonly number[] = Object.freeze([1, 1, 1, 1, 1, 1]);
export const DREAM_JOINT_OFFSET: readonly number[] = Object.freeze([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

/** The MakerMods Metal Arm (Damiao motors). */
export const METAL_ARM: Product = Object.freeze({
  packageName: "@makermods/metal-arm",
  displayName: "Metal Arm",
  motorVendor: "Damiao",
  zeroPose: ZERO_POSE,
  models: MODELS,
  defaultModel: DEFAULT_MODEL,
  kpRange: KP_RANGE,
  kdRange: KD_RANGE,
  idRange: ID_RANGE,
  defaultHostId: DEFAULT_HOST_ID,
  route: (canId: number, data: Uint8Array, hostId: number, extended: boolean) => route(canId, data, hostId, extended),
  createMotor: (joint, backend, hostId, clock, logger) => new Motor(joint, backend, hostId, clock, logger),
  defaultProfile: Object.freeze({ ...PROFILE, urdf: URDF }),
  defaultStarMapping: STAR_MAPPING,
  dreamEmbodiment: DREAM_EMBODIMENT,
  dreamJointSign: DREAM_JOINT_SIGN,
  dreamJointOffset: DREAM_JOINT_OFFSET,
} satisfies Product);
