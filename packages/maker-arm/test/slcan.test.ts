// A real Maker Arm over the SLCAN serial transport: simulated RobStride motors behind a fake SLCAN
// adapter on a fake serial port, on virtual time. Proves feedback-paced control end to end: connect,
// enable, a short move and stop, with every frame spaced as canable-fw needs.
import { describe, expect, it } from "vitest";
import { Arm, ArmState } from "@makermods/maker-arm";
import { simulatedBackend, VirtualClock } from "@makermods/maker-arm/testing";
import { FakeSerialPort } from "@makermods/arm-core/transport/fake-serial";
import { SlcanSerialBackend } from "@makermods/arm-core/transport/slcan-serial";

const quiet = { info() {}, warn() {}, error() {} };

describe("Maker Arm over SLCAN", () => {
  it("connects, enables, moves and stops with feedback-paced control", async () => {
    const clock = new VirtualClock();
    const [bus, sim] = simulatedBackend();
    const port = new FakeSerialPort({
      clock, minFrameInterval: 0.0015, responder: (canId, data, extended) => bus.responder!(canId, data, extended) ?? [],
    });
    port.replyDelay = 0.002; // each motor answers 2 ms after its frame reached the adapter
    const backend = new SlcanSerialBackend(port, { clock, logger: quiet });
    expect(backend.feedbackPacedControl).toBe(true);
    const arm = new Arm(sim.config, backend, { clock, logger: quiet });
    await arm.connect();
    expect(clock.now()).toBeGreaterThanOrEqual(2.5); // the adapter's startup delay
    await arm.enable();
    expect(arm.state).toBe(ArmState.ENABLED);
    const moveFrom = port.frames.length;
    const joints = arm.config.joints;
    const start = arm.getJointPositions();
    const goal = start.map((q, i) => Math.min(joints[i]!.hi - 0.1, Math.max(joints[i]!.lo + 0.1, q + 0.5)));
    let moved = 0;
    try {
      expect(await arm.moveTo(goal, { timeout: 10.0 })).toBe(true);
      moved = port.frames.length;
      sim.positions.forEach((q, i) => expect(Math.abs(q - goal[i]!)).toBeLessThanOrEqual(0.05));
    } finally {
      expect(await arm.stop()).toBe(true);
      await arm.disconnect();
    }
    expect([...sim.motors.values()].some((m) => m.enabled)).toBe(false);
    expect(arm.state).toBe(ArmState.DISCONNECTED);
    expect(backend.error).toBe(null);
    expect(backend.closeError).toBe(null);
    expect(port.protocolErrors).toEqual([]);
    expect(port.commands[0]).toBe("C");
    expect(port.commands.slice(1, 4)).toEqual(["S8", "A0", "O"]);
    expect(port.commands.at(-1)).toBe("C");
    expect(port.opened).toBe(false);
    expect(port.frames.length).toBeGreaterThan(100);
    expect(port.frames.every((f) => !f.extended)).toBe(true);
    // Paced: during the move each motor's command waits for the previous motor's reply (2 ms), where
    // an unpaced loop would space them by the 1.5 ms transmit interval only.
    const moving = port.frames.slice(moveFrom, moved);
    const pairs = moving.slice(1).map((f, i) => [moving[i]!, f] as const)
      .filter(([a, b]) => b.canId === a.canId + 1 && a.data.length === 8 && b.data.length === 8);
    expect(pairs.length).toBeGreaterThan(30);
    for (const [a, b] of pairs) expect(b.at! - a.at!).toBeGreaterThanOrEqual(0.002 - 1e-9);
  });
});
