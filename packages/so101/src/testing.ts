/** Test support: the simulated SO-101 bus and port, a sample calibration, and a simulated arm in one call. Virtual time
comes from "@makermods/arm-core/testing". */

import type { Clock } from "@makermods/arm-core";
import { So101Arm, type So101ArmOptions } from "./arm.js";
import { MemoryCalibrationStore } from "./calibration.js";
import { simulatedSo101, type SimFeetechBus, type SimSerialPort, type SimulatedSo101Options } from "./sim.js";
import type { So101Calibration } from "./so101.js";

export { SimFeetechBus, SimSerialPort, SimStsServo, simulatedSo101, type SimWrite, type SimulatedSo101Options } from "./sim.js";

/** A real SO-101 follower's calibration (the console's dlabs_xlerobot_left.json; every range across the encoder
wrap). */
export const SAMPLE_CALIBRATION: So101Calibration = {
  shoulder_pan: { id: 1, drive_mode: 0, homing_offset: 1971, range_min: 692, range_max: 3426 },
  shoulder_lift: { id: 2, drive_mode: 0, homing_offset: 1099, range_min: 836, range_max: 3236 },
  elbow_flex: { id: 3, drive_mode: 0, homing_offset: -1365, range_min: 925, range_max: 3204 },
  wrist_flex: { id: 4, drive_mode: 0, homing_offset: 1823, range_min: 862, range_max: 3277 },
  wrist_roll: { id: 5, drive_mode: 0, homing_offset: -1281, range_min: 0, range_max: 4095 },
  gripper: { id: 6, drive_mode: 0, homing_offset: 1946, range_min: 1278, range_max: 2842 },
};

/** An So101Arm on a simulated SO-101 whose servos hold `calibration` (default SAMPLE_CALIBRATION), with that
calibration in an in-memory store. */
export function simulatedArm(
  clock: Clock,
  options: Partial<SimulatedSo101Options> & { arm?: So101ArmOptions } = {},
): { arm: So101Arm; sim: SimFeetechBus; port: SimSerialPort } {
  const calibration = options.calibration ?? SAMPLE_CALIBRATION;
  const { sim, port } = simulatedSo101(clock, { ...options, calibration });
  const side = options.arm?.profile?.side ?? "left";
  const store = options.arm?.calibrationStore ?? new MemoryCalibrationStore({ [`so101_${side}`]: calibration });
  const arm = new So101Arm(port, { clock, calibrationStore: store, ...options.arm });
  return { arm, sim, port };
}
