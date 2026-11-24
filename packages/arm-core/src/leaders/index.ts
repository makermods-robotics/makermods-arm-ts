/**
 * Teleop leaders: anything that yields follower joint targets (Leader).
 *
 * StarLeader reads a Star Arm 102 (FashionStar servos on a serial port); ArmLeader floats a second
 * arm of the same kind so it can be moved by hand.
 */

export {
  ArmLeader, DEFAULT_DEADZONE, Friction, type FrictionConfig, type FrictionJoint, type LeaderArm, type LimitsSource,
} from "./arm.js";
export { isLeader, type Leader, LeaderError } from "./base.js";
export {
  BAUDRATE, type ServoBus, type ServoBusFactory, SETTLE_S, StarLeader, type StarLeaderOptions,
} from "./star.js";
