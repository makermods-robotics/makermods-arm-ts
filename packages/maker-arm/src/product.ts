/** Product identity: names, protocol limits, motor implementation and shipped data of the Maker Arm. */

import type { Product } from "@makermods/arm-core";
import { PROFILE } from "./generated/profile.js";
import { STAR_MAPPING } from "./generated/star-mapping.js";
import { URDF } from "./generated/urdf.js";
import { DEFAULT_HOST_ID, DEFAULT_MODEL, ID_RANGE, KD_RANGE, KP_RANGE, MODELS, RobStrideMotor, route } from "./motor.js";

/** The Maker Arm: 7 RobStride motors on the MIT protocol, classic CAN 1 Mbit/s. */
export const MAKER_ARM: Product = Object.freeze({
  packageName: "@makermods/maker-arm",
  displayName: "Maker Arm",
  motorVendor: "RobStride",
  zeroPose: "folded against the base, gripper fully closed",

  models: MODELS,
  defaultModel: DEFAULT_MODEL,
  kpRange: KP_RANGE,
  kdRange: KD_RANGE,
  idRange: ID_RANGE,
  defaultHostId: DEFAULT_HOST_ID,

  route,
  createMotor: (joint, backend, hostId, clock, logger) => new RobStrideMotor(joint, backend, hostId, clock, logger),

  defaultProfile: Object.freeze({ ...PROFILE, urdf: URDF }),
  defaultStarMapping: STAR_MAPPING,

  // dream_ik embodiment of this arm and its joints from this package's: embodiment joint = joint * sign + offset
  // (rad). Its URDF takes the joints as the arm reports them.
  dreamEmbodiment: "makers",
  dreamJointSign: [1, 1, 1, 1, 1, 1],
  dreamJointOffset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
} satisfies Product);
