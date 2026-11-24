/** @makermods/so101: the SO-101 arm (six Feetech STS3215 servos on a CH343 USB serial adapter) from a browser page or
Web Worker over Web Serial, with the same calls as the other MakerMods arms. */

export { So101Arm, createArm, type MoveToOptions, type So101ArmOptions, type So101JointStatus, type So101Status } from "./arm.js";
export {
  Arbiter,
  ArbiterClosedError,
  CH343_FILTER,
  FedUart,
  FeetechBus,
  StaleEpochError,
  grantedPorts,
  openPortLink,
  openSo101Link,
  portLabel,
  requestPort,
  type PortLink,
  type SerialLike,
  type SerialPortLike,
  type So101Link,
  type TransactionOptions,
  type UartPort,
} from "./bus.js";
export {
  LocalStorageCalibrationStore,
  MemoryCalibrationStore,
  So101CalibrationSession,
  TorqueOnError,
  calibrateSo101,
  type CalibrationStatus,
  type CalibrationStore,
  type So101CalibrationOptions,
  type So101Method,
  type StorageLike,
} from "./calibration.js";
export {
  AutoCalStopped,
  DEFAULT_TORQUE_LIMIT,
  ServoFaultError,
  autoCalibrateSo101,
  clampTorqueLimit,
  nearestWrapTarget,
  rangeFromLimits,
  type AutoCalOptions,
  type MotorCalibration,
} from "./autocal.js";
export { SO101_CHAINS, type So101Chain, type UrdfJoint } from "./chain.js";
export { So101Follower, type So101FollowerOptions } from "./follower.js";
export {
  identifySo101,
  planWiggle,
  wiggleGripper,
  wiggleSo101,
  type IdentifyOptions,
  type IdentifyResult,
  type PortCandidate,
  type WiggleResult,
} from "./identify.js";
export * as limits from "./limits.js";
export {
  SO101_LIMIT_TOLERANCE_DEG,
  SO101_SIDE_LIMITS_DEG,
  SO101_VENDOR_LIMITS_DEG,
  armConfigOf,
  defaultSo101Profile,
  so101Profile,
  type Range,
  type So101ArmConfig,
  type So101Profile,
} from "./profile.js";
export * as protocol from "./protocol.js";
export { FeetechError, FeetechServoError, type FeetechCalibration, type ServoState } from "./protocol.js";
export {
  JOINT_WORDS,
  MissedReadError,
  REFERENCE_UNITS,
  ReleaseError,
  SO101_IDS,
  SO101_MOTORS,
  So101Error,
  So101FaultError,
  StartCancelledError,
  alignToReference,
  calibrationDifferences,
  degToTicks,
  parseSo101Calibration,
  referenceRanges,
  shortJoints,
  ticksToDeg,
  ticksToVector,
  vectorToTicks,
  type So101Calibration,
  type So101Motor,
} from "./so101.js";
