import { expect, it } from "vitest";
import * as core from "@makermods/arm-core";
import { MockBackend } from "@makermods/arm-core";
import * as maker from "@makermods/maker-arm";
import * as testing from "@makermods/maker-arm/testing";
import * as gsUsb from "../../arm-core/src/transport/gs-usb.js";
import { missingFromTypeScript } from "../../arm-core/test/helpers/api-manifest.js";

it("has every public name of the Python maker_arm SDK", () => {
  const config = maker.defaultConfig();
  const arm = maker.createArm(new MockBackend(), { config });
  const { missing, unusedExceptions } = missingFromTypeScript("maker_arm", {
    arm: { exports: core, instances: { Arm: arm } },
    bus: { exports: core },
    config: { exports: core, instances: { ArmConfig: config, JointConfig: config.joints[0]! } },
    errors: { exports: core },
    gravity: { exports: core.gravity },
    gripper: { exports: core },
    kinematics: { exports: core.kinematics },
    trajectory_ik: { exports: core.trajectoryIk },
    testing: { exports: testing },
    teleop: { exports: core },
    mapping: { exports: core },
    stall: { exports: core },
    rig: { exports: core },
    cameras: { exports: core },
    "leaders.base": { exports: core },
    "leaders.star": { exports: core },
    "leaders.arm": { exports: core },
    "transport.base": { exports: core },
    "transport.mock": { exports: core },
    "transport.gs_usb": { exports: { ...core, ...gsUsb } },
    "transport.slcan_serial": { exports: core },
    protocol: { exports: maker.protocol },
    motor: { exports: maker, rename: { Motor: "RobStrideMotor" } },
    _sim_motor: { exports: testing },
    _product: { exports: maker.MAKER_ARM as unknown as Record<string, unknown>, constantsAsFields: true },
  });
  expect(missing).toEqual([]);
  expect(unusedExceptions).toEqual([]);
});
