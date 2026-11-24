// Built against the packages' published entry points (dist), the way an application's bundler sees them.
import { ArmGroup, Camera, GsUsbBackend, requestAdapter, Rig, SlcanSerialBackend, StarLeader, Teleop } from "@makermods/arm-core";
import { simulatedStarLeader, VirtualClock } from "@makermods/arm-core/testing";
import { createArm as createMaker, MAKER_ARM } from "@makermods/maker-arm";
import { simulatedArm as simulatedMaker } from "@makermods/maker-arm/testing";
import { createArm as createMetal, METAL_ARM } from "@makermods/metal-arm";
import { simulatedArm as simulatedMetal } from "@makermods/metal-arm/testing";
import { createArm as createSo101, requestPort } from "@makermods/so101";

const report: Record<string, unknown> = {
  exports: [ArmGroup, Camera, GsUsbBackend, requestAdapter, Rig, SlcanSerialBackend, StarLeader, Teleop, simulatedStarLeader,
    createMaker, createMetal, createSo101, requestPort].map((x) => typeof x),
  products: [MAKER_ARM.packageName, METAL_ARM.packageName],
};
for (const [name, make] of [["maker", simulatedMaker], ["metal", simulatedMetal]] as const) {
  const [arm] = make({ clock: new VirtualClock() });
  await arm.connect();
  await arm.enable();
  const ok = await arm.stop();
  await arm.disconnect();
  report[name] = ok;
}
(globalThis as { distCheck?: unknown }).distCheck = report;
console.log(JSON.stringify(report));
