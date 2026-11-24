// The motor / late-gripper case of the Metal SDK's tests/unit/test_gs_usb.py: Damiao motors behind a
// fake candleLight adapter, the gripper holding each reply until the next frame on the bus.
import { describe, expect, it } from "vitest";
import { DamiaoMotor, route } from "@makermods/metal-arm";
import { FakeGsUsbDevice, VirtualClock } from "@makermods/arm-core/testing";
import { GsUsbBackend } from "@makermods/arm-core/transport/gs-usb";
import { FakeBus, FakeMotor, makeJoint } from "./fakes.js";

describe("gs_usb", () => {
  it("a motor over gs_usb with a late gripper", async () => {
    const clock = new VirtualClock();
    const fakes = [new FakeMotor(6, "jhi", { position: -0.3 }), new FakeMotor(7, "jhi", { position: 0.8, late: true })];
    const bus = new FakeBus(fakes);
    const dev = new FakeGsUsbDevice({ serialNumber: "ABC123", responder: (canId, data, extended) => bus.responder!(canId, data, extended) ?? [] });
    const backend = new GsUsbBackend(dev, { clock, logger: { warn() {}, error() {} } });
    const motors = new Map(fakes.map((f) => [f.motorId, new DamiaoMotor(makeJoint(f.motorId, f.model), backend, 0x10, clock)]));
    backend.setRecvCallback((canId, data, extended) => {
      const id = route(canId, data, 0x10, extended);
      if (id !== null) motors.get(id)?.handleFrame(canId, data, extended);
    });
    await backend.open();
    try {
      expect(Math.abs((await motors.get(7)!.readState(0.1)).position - 0.8)).toBeLessThanOrEqual(1e-3);
      await motors.get(7)!.validate();
      const token = await motors.get(7)!.armWatchdog(500);
      expect(fakes[1]!.watchdogTicks).toBe(10_000);
      await motors.get(7)!.restoreWatchdog(token);
      expect(Math.abs((await motors.get(6)!.readState()).position + 0.3)).toBeLessThanOrEqual(1e-3);
    } finally {
      await backend.close();
    }
  });
});
