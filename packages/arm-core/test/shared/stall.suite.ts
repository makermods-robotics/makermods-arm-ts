// Port of tests/unit/test_stall.py (identical in both Python SDKs).
import { describe, expect, it } from "vitest";
import {
  defaultStallLogPath, MotorStateRecorder, pyFixed, type RecordedArm, StallDetector, type StallSample, StallThresholds,
} from "../../src/stall.js";
import type { ArmFakes } from "./fakes.js";

function sample(detector: StallDetector, now: number, motorId = 1, overrides: Partial<StallSample> = {}): boolean {
  return detector.update(now, motorId, {
    error: 0.2, velocity: 0.01, torque: 2.0, feedbackAge: 0.01, faulted: false, ...overrides,
  });
}

/** Registers the tests (product-neutral; run once per arm package). */
export function stallSuite(fk: Pick<ArmFakes, "name">): void {
  describe(`${fk.name}: stall detection`, () => {
    it("a persistent candidate confirms once", () => {
      const detector = new StallDetector([1], new StallThresholds({ persistenceS: 0.5 }));
      expect(sample(detector, 1.0)).toBe(true);
      expect(sample(detector, 1.49)).toBe(true);
      expect(detector.events).toEqual([]);
      expect(sample(detector, 1.5)).toBe(true);
      expect(detector.events.length).toBe(1);
      expect(detector.events[0]!.motorId).toBe(1);
      expect(detector.events[0]!.candidateSinceS).toBe(1.0);
      sample(detector, 2.0);
      expect(detector.events.length).toBe(1);
    });

    it("the candidate timer resets when the motor moves again", () => {
      const detector = new StallDetector([1], new StallThresholds({ persistenceS: 0.5 }));
      sample(detector, 0.0);
      expect(sample(detector, 0.3, 1, { velocity: 0.2 })).toBe(false);
      sample(detector, 0.4);
      sample(detector, 0.8);
      expect(detector.events).toEqual([]);
      sample(detector, 0.9);
      expect(detector.events.length).toBe(1);
      expect(detector.events[0]!.candidateSinceS).toBe(0.4);
    });

    it("a candidate needs error, low speed, torque and fresh healthy feedback", () => {
      const detector = new StallDetector([1], new StallThresholds());
      const cases: Array<Partial<StallSample>> = [
        { error: 0.01 }, { velocity: 0.2 }, { torque: 0.1 }, { feedbackAge: 0.3 }, { faulted: true }, { error: NaN },
      ];
      cases.forEach((overrides, index) => expect(sample(detector, index, 1, overrides)).toBe(false));
    });

    it("independent motors keep their confirmation order", () => {
      const detector = new StallDetector([1, 2], new StallThresholds({ persistenceS: 0.5 }));
      sample(detector, 0.0, 2);
      sample(detector, 0.2, 1);
      sample(detector, 0.5, 2);
      sample(detector, 0.7, 1);
      expect(detector.events.map((e) => e.motorId)).toEqual([2, 1]);
    });

    it("the recorder writes all motors and the summary", () => {
      const feedback = { enabled: true };
      const healthy = {
        feedback, feedbackAge: 0.01, statusAge: 0.01, status: { code: 0, hasFault: false, text: "" }, faultLatched: null,
      };
      const arm: RecordedArm = {
        state: "enabled",
        config: { joints: [{ motorId: 1, model: "A" }, { motorId: 2, model: "B" }] },
        motors: [{ ...healthy, torqueLimit: 14.0 }, { ...healthy, torqueLimit: 17.0 }],
        getCommandedPositions: () => [1.0, 0.0],
        getJointPositions: () => [0.7, 0.0],
        getJointVelocities: () => [0.01, 0.2],
        getJointTorques: () => [2.0, 0.1],
        getTemperatures: () => [35.0, 36.0],
      };
      let csv = "";
      let summaryJson = "";
      const recorder = new MotorStateRecorder({
        write: (text) => (csv += text), summary: (json) => (summaryJson = json),
      }, 1.5, arm, new StallThresholds({ persistenceS: 0.5 }), { name: "states.csv" });
      expect(recorder.record(0.0, [1.2, 0.0])).toEqual([]);
      const events = recorder.record(0.5, [1.2, 0.0]);
      expect(events.map((e) => e.motorId)).toEqual([1]);
      const summary = recorder.close("stall_CAN1");

      const lines = csv.split("\r\n").filter((l) => l);
      expect(lines.length).toBe(5); // header + 2 motors × 2 ticks
      expect(lines[0]).toBe(MotorStateRecorder.FIELDS.join(","));
      const row = Object.fromEntries(MotorStateRecorder.FIELDS.map((f, i) => [f, lines[3]!.split(",")[i]]));
      expect(row).toMatchObject({
        elapsed_s: "0.500000", payload_kg: "1.5", joint: "1", motor_id: "1", model: "A", leader_target_rad: "1.2000000",
        error_rad: "0.3000000", torque_fraction: "0.142857", temperature_c: "35.00", enabled: "1", status_code: "0x00000000",
        stall_candidate: "1", stall_confirmed: "1",
      });
      const parsed = JSON.parse(summaryJson);
      expect(parsed).toEqual(summary);
      expect(parsed.payload_kg).toBe(1.5);
      expect(parsed.first_stall.motor_id).toBe(1);
      expect(parsed.motor_peaks["1"].max_abs_torque_nm).toBe(2.0);
      expect(parsed.csv).toBe("states.csv");
      expect(recorder.close("again")).toBe(summary);
    });

    it("a stall needs an enabled, unfaulted arm", () => {
      const detector = new StallDetector([1], new StallThresholds());
      const base = { error: 0.2, velocity: 0.01, torque: 2, feedbackAge: 0.01 };
      expect(detector.isCandidate({ ...base, faulted: false, enabled: false })).toBe(false);
      expect(detector.isCandidate({ ...base, faulted: true, enabled: true })).toBe(false);
      expect(detector.isCandidate({ ...base, faulted: false, enabled: true })).toBe(true);
    });

    it("numbers are formatted as Python formats them", () => {
      expect(pyFixed(35.125, 2)).toBe("35.12"); // an exact tie rounds to even
      expect(pyFixed(35.375, 2)).toBe("35.38");
      expect(pyFixed(0.0078125, 6)).toBe("0.007812");
      expect(pyFixed(-0.0, 6)).toBe("-0.000000");
      expect(pyFixed(-1e-9, 6)).toBe("-0.000000");
      expect(pyFixed(1.2, 7)).toBe("1.2000000");
      expect(pyFixed(NaN, 2)).toBe("nan");
      expect(defaultStallLogPath(1.5, "logs", new Date(2026, 10, 23, 7, 5, 9))).toBe("logs/teleop_motor_states_20261123_070509_1.5kg.csv");
      expect(defaultStallLogPath(2.0, "x", new Date(2026, 0, 2, 3, 4, 5))).toBe("x/teleop_motor_states_20260102_030405_2kg.csv");
    });
  });
}
