// The Maker Arm's frame fixture (tools/gen_arm_frames.py --product maker), replayed by the shared helper.
import { defaultConfig } from "@makermods/maker-arm";
import { forJoint, SimBus, type SimMotor, watchdogTimeout } from "@makermods/maker-arm/testing";
import { replayArmFrames } from "../../arm-core/test/helpers/arm-frames.js";

replayArmFrames<SimMotor>({
  fixture: new URL("../../../contract/fixtures/arm_frames_maker.json", import.meta.url),
  defaultConfig,
  forJoint: (joint, hostId, position, options) => forJoint(joint, hostId, position, options),
  bus: (motors) => new SimBus(motors),
  watchdogTimeout,
});
