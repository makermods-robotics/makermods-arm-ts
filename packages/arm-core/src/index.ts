export { type Clock, Mutex, RealClock, type RealClockOptions, Signal } from "./clock.js";
export {
  ArmConfig, type ArmConfigInit, checkGains, degrees, JointConfig, type JointConfigInit, radians, type RawProfile,
} from "./config.js";
export { ArmError, ConnectError, FaultLatched, ParamTimeout, StateError, TimeoutError, ValueError } from "./errors.js";
export { ArmState } from "./state.js";
export type { Motor, MotorFeedback, MotorStatus, ParamSpec, Product } from "./motor.js";
export { CanBackend, type RecvCallback } from "./transport/base.js";
export { MockBackend, type Responder } from "./transport/mock.js";
export {
  Arm, type ArmOptions, type ArmStatus, EE_ANGLE_TOL, EE_MAX_SECONDS, EE_MIN_SECONDS, EE_POSITION_TOL, type EnableOptions, type ExtraTorque,
  FRAME_SPACING, type JointStatus, type Logger, type MoveEeOptions, type MoveOptions,
} from "./arm.js";
export {
  Bus, type BusRow, jointPosition, release, RELEASE_ATTEMPTS, RELEASE_REPLY_S, releaseMotor, type ReleaseResult,
} from "./bus.js";
export { CAPPED_CLOSING_KD, GripperHold, pySum } from "./gripper.js";
export * as kinematics from "./kinematics.js";
export * as trajectoryIk from "./trajectory-ik.js";
export * as gravity from "./gravity.js";
export {
  Chain, chainFromConfig, type FromUrdfOptions, type IKOptions, type IKResult, matrixToRpy, poseToXyzRpy, rotationError,
  rpyToMatrix, xyzRpyToPose,
} from "./kinematics.js";
export {
  extendPoses, IKStream, IKWeights, type IKResult as TrajectoryIKResult, poseResidual, savgol, seamBlend, smoothPoses, so3Exp,
  so3Log, solvePath, solvePathAsync, type SolveOptions, straightLine, toolErrors, TrajectoryIK, type TrajectoryIKOptions,
} from "./trajectory-ik.js";
export { type CooperativeOptions, macrotask } from "./cooperate.js";
export { GravityFeedforward, type GravityFeedforwardOptions } from "./gravity.js";
export {
  ACCESS_HINT, adapterPort, bitTiming, type BtConst, grantedAdapters, GS_USB_FILTERS, GsUsbBackend, type GsUsbBackendOptions,
  isGsUsbDevice, MAX_IN_FLIGHT, parseBtConst, parsePort, requestAdapter, selectAdapter, type TransportLogger,
  type UsbControlSetup, type UsbDeviceLike,
} from "./transport/gs-usb.js";
export {
  grantedSerialPorts, requestSerialPort, type SerialFilter, type SerialLike, type SerialPortLike,
} from "./transport/serial.js";
export {
  BITRATE_COMMANDS, grantedSlcanPorts, requestSlcanPort, type SlcanFrame, SlcanFrameParser, SlcanSerialBackend,
  type SlcanSerialBackendOptions, slcanEncode,
} from "./transport/slcan-serial.js";
export {
  MAX_PENDING_HOOKS, MAX_STEP_S, type OnStep, runMany, SpeedCap, type StallHook, Teleop, type TeleopFollower, type TeleopOptions,
  TeleopStep,
} from "./teleop.js";
export { JointMapper, type RawDegrees, type StarMapping, type StarMappingJoint } from "./mapping.js";
export {
  defaultStallLogPath, MotorStateRecorder, type MotorStateRecorderOptions, type RecordedArm, type RecordSink, StallDetector,
  StallEvent, type StallSample, type StallSummary, StallThresholds,
} from "./stall.js";
export {
  ArmLeader, DEFAULT_DEADZONE, Friction, type FrictionConfig, type FrictionJoint, isLeader, type Leader, type LeaderArm,
  LeaderError, type LimitsSource, type ServoBus, type ServoBusFactory, BAUDRATE as STAR_BAUDRATE, SETTLE_S as STAR_SETTLE_S,
  StarLeader, type StarLeaderOptions,
} from "./leaders/index.js";
export {
  AngleReliability, FashionStarBus, type FashionStarBusOptions, LossTracker, RESYNC_ROOM, ServoBusError, type ServoMonitor,
  WRITE_TIMEOUT as SERVO_WRITE_TIMEOUT,
} from "./leaders/fashionstar.js";
export {
  ArmGroup, ArmSpec, type ArmSpecInit, type BackendKwargs, type BackendResolver, BACKENDS, browserBackends, browserSerialPorts,
  CAMERA_ID_PREFIXES, type CloseOutcome, DEFAULT_CHANNEL, type FileLoader, LEADER_TYPES, LeaderSpec, type LeaderSpecInit,
  type ProfileSource, Rig, RigError, type RigInit, type RigLeader, type RigOptions, type SerialPortResolver, SLCAN_BAUDRATE,
  SLCAN_STARTUP_DELAY, type SlcanBackendOptions, type SlcanFactory,
} from "./rig.js";
export {
  browserFrames, Camera, type CameraId, type CameraInfo, type CameraOptions, cameraConstraints, cameraStartError,
  type CanvasFactory, type CanvasLike, type CaptureMode, captureModes, encodeJpeg, FALLBACK_MODES, Frame, type FrameReader,
  listCameras, type MediaDeviceInfoLike, type MediaDevicesLike, type MediaStreamLike, type MediaStreamTrackLike, parseCameraId,
  READ_FAILURE_S, REOPEN_S, resolveCamera, type VideoFrameLike, WATCH_S,
} from "./cameras.js";
