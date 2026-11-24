// Swing to find and Wiggle (MakerLab identify.py, wiggle.py) on simulated SO-101s. Ported from the console's
// test/so101.test.ts.
import { VirtualClock } from "@makermods/arm-core/testing";
import { describe, expect, it } from "vitest";
import { identifySo101, planWiggle, wiggleSo101 } from "../src/identify.js";
import { simulatedSo101 } from "../src/sim.js";
import type { So101Calibration } from "../src/so101.js";
import fixtures from "./fixtures/calibrations.json" with { type: "json" };

const LEFT_FILE = fixtures.follower_left as So101Calibration;

describe("Swing to find and Wiggle (MakerLab)", () => {
  it("finds the port whose base swung 120 ticks both ways, never touching torque", async () => {
    const clock = new VirtualClock();
    const a = simulatedSo101(clock, { calibration: LEFT_FILE, label: "usb:1A86:55D3#1" });
    const b = simulatedSo101(clock, { calibration: LEFT_FILE, label: "usb:1A86:55D3#2" });
    const base = a.sim.actual(1);
    // Hand moves on the virtual timeline: a small bump on the other arm (not a swing), then a swing both ways.
    void clock.sleep(0.6).then(() => {
      b.sim.moveByHand(1, base + 60);
      a.sim.moveByHand(1, base + 150);
    });
    void clock.sleep(0.9).then(() => a.sim.moveByHand(1, base - 150));
    const result = await identifySo101([a.port, b.port], { clock });
    expect(result).toMatchObject({ success: true, index: 0, message: "Found the arm on usb:1A86:55D3#1." });
    expect(result.port).toBe(a.port);
    for (const sim of [a.sim, b.sim]) expect(sim.writes).toEqual([]);
    expect(a.port.isOpen || b.port.isOpen).toBe(false);
  });

  it("times out when no base moves", async () => {
    const clock = new VirtualClock();
    const a = simulatedSo101(clock, { calibration: LEFT_FILE });
    const result = await identifySo101([a.port], { clock });
    expect(result.success).toBe(false);
    expect(result.message).toMatch(/No motion detected within 20s/);
    expect(clock.now()).toBeGreaterThanOrEqual(20);
  });

  it("stops when aborted, and refuses when nothing answers", async () => {
    const clock = new VirtualClock();
    const a = simulatedSo101(clock, { calibration: LEFT_FILE });
    const abort = new AbortController();
    void clock.sleep(1).then(() => abort.abort());
    expect(await identifySo101([a.port], { clock, signal: abort.signal })).toEqual({ success: false, message: "Stopped." });
    expect(clock.now()).toBeLessThan(2);
    a.sim.setOnline(1, false);
    await expect(identifySo101([a.port], { clock })).rejects.toThrow(/No SO-101 answered/);
    expect(a.port.isOpen).toBe(false);
  });

  it("wiggles only the gripper inside its limits, then returns it and switches it off", async () => {
    const clock = new VirtualClock();
    const { sim, port } = simulatedSo101(clock, { calibration: LEFT_FILE });
    const start = sim.reg(6, "Present_Position");
    const result = await wiggleSo101(port, { clock });
    expect(result.success).toBe(true);
    const goals = sim.writesTo(6, "Goal_Position");
    expect(goals[0]).toBe(start);
    expect(goals).toContain(Math.min(start + 200, LEFT_FILE.gripper.range_max));
    expect(goals).toContain(Math.max(start - 200, LEFT_FILE.gripper.range_min));
    expect(goals.at(-1)).toBe(start);
    expect(Math.abs(sim.reg(6, "Present_Position") - start)).toBeLessThanOrEqual(10);
    expect(sim.reg(6, "Torque_Enable")).toBe(0);
    for (const id of [1, 2, 3, 4, 5]) expect(sim.writesTo(id, "Torque_Enable")).toEqual([]);
    expect(port.isOpen).toBe(false);
  });

  it("plans the jog inside the EEPROM limits", () => {
    expect(planWiggle(2000, 1278, 2842)).toEqual({ high: 2200, low: 1800, rest: 2000 });
    expect(planWiggle(1300, 1278, 2842)).toEqual({ high: 1500, low: 1278, rest: 1300 });
    expect(() => planWiggle(2000, 1900, 2100)).toThrow(/too narrow/);
    expect(() => planWiggle(3000, 1278, 2842)).toThrow(/outside its limits/);
  });
});
