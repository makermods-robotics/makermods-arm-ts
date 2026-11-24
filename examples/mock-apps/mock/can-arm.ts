/**
 * A simulated CAN arm behind a fake candleLight adapter: the arm package's simulated motors answer every frame the
 * adapter puts on the bus, so the real stack (WebUSB transport, GsUsbBackend, Arm) runs unchanged on top of it.
 */

import { type ArmConfig } from "@makermods/arm-core";
import { RealClock } from "@makermods/arm-core";
import { FakeGsUsbDevice, FakeSerialPort, SimulatedMotors, type SimulatedMotor } from "@makermods/arm-core/testing";
import { defaultConfig as makerConfig } from "@makermods/maker-arm";
import * as makerSim from "@makermods/maker-arm/testing";
import { defaultConfig as metalConfig } from "@makermods/metal-arm";
import * as metalSim from "@makermods/metal-arm/testing";

export type Product = "metal" | "maker";

export const PRODUCTS: readonly Product[] = ["metal", "maker"];

/** Each kind of frame the host sent, counted per motor, for the timing bench. */
export type FrameObserver = (kind: string, motorId: number | null) => void;

export interface SimulatedCanArm {
  product: Product;
  /** The adapter: a candleLight on WebUSB, or (simulatedSlcanArm) an SLCAN adapter on a serial port. */
  device: FakeGsUsbDevice | FakeSerialPort;
  sim: SimulatedMotors<SimulatedMotor>;
  config: ArmConfig;
  /** Called for every frame the host puts on the bus (set it to watch the traffic). */
  observe: FrameObserver | null;
}

/** The product a fake adapter's serial names (`FAKE-METAL-1`), or null for another adapter. */
export function productOfSerial(serial: string | null | undefined): Product | null {
  const m = /^FAKE-(METAL|MAKER)-/.exec(serial ?? "");
  return m ? (m[1]!.toLowerCase() as Product) : null;
}

/**
 * A fake adapter with `product`'s seven simulated motors behind it, each joint at zero clipped into its limits.
 * `serial` defaults to `FAKE-<PRODUCT>-1`.
 */
export function simulatedCanArm(product: Product, serial = `FAKE-${product.toUpperCase()}-1`): SimulatedCanArm & { device: FakeGsUsbDevice } {
  const arm = simulatedMotors(product);
  arm.device = new FakeGsUsbDevice({ serialNumber: serial, responder: arm.responder });
  return arm as SimulatedCanArm & { device: FakeGsUsbDevice };
}

/** The same arm behind a fake SLCAN adapter (a CANable on a serial port) that checks the frame spacing it needs. */
export function simulatedSlcanArm(product: Product): SimulatedCanArm & { device: FakeSerialPort } {
  const arm = simulatedMotors(product);
  arm.device = new FakeSerialPort({ responder: arm.responder, clock: new RealClock(), minFrameInterval: 0.0015 });
  return arm as SimulatedCanArm & { device: FakeSerialPort };
}

function simulatedMotors(product: Product): SimulatedCanArm & { responder: (canId: number, data: Uint8Array, extended: boolean) => [number, Uint8Array][] } {
  const config = product === "maker" ? makerConfig() : metalConfig();
  const s = product === "maker" ? makerSim : metalSim;
  const motors = config.joints.map((j) =>
    s.forJoint(j as never, config.hostId, Math.min(j.hi, Math.max(j.lo, 0)) * j.direction + j.offset),
  );
  const byId = new Map(motors.map((m) => [m.motorId, m]));
  const sim = new SimulatedMotors<SimulatedMotor>(config, motors as SimulatedMotor[], (m) => s.watchdogTimeout(m as never));
  const arm = {
    product, sim, config, observe: null, device: null as never,
    responder: (canId: number, data: Uint8Array, extended: boolean) => {
      if (arm.observe) {
        const [kind, id] = s.classify(canId, data);
        arm.observe(kind, id);
      }
      return s.respond(byId as never, canId, data, extended) as [number, Uint8Array][];
    },
  } as SimulatedCanArm & { responder: (canId: number, data: Uint8Array, extended: boolean) => [number, Uint8Array][] };
  return arm;
}
