/**
 * Read every motor of one arm: position, temperature, fault status. Never enables or releases
 * torque. Each motor first gets a non-clearing fault query; only fault-free motors are then asked
 * for their position, so fault evidence is kept.
 *
 *     node examples/node/scan.ts --product metal --port gs_usb:<serial> [--frames sent.jsonl]
 *
 * The adapter must be listed for that product in the rig file (see common.ts).
 */

import { Bus } from "@makermods/arm-core";
import { COMMON_OPTIONS, deg, openTarget, parseArgs, recordSends, writeFrames } from "./common.ts";

const args = parseArgs(process.argv.slice(2), COMMON_OPTIONS);
const { config, backend } = await openTarget(args);
const frames = typeof args.frames === "string" ? recordSends(backend) : null;
const bus = new Bus(backend, config);
let ok = false;
try {
  await bus.open();
  const rows = await bus.readAll();
  const missing = bus.missing(rows);
  console.log(`${args.product} (${backend.port}):`);
  if (!rows.length) {
    console.log("  no motors found -- check power, wiring, termination, 1 Mbps, and that no other program holds the adapter");
  } else {
    console.log("   ID joint            position_deg  temp_C  enabled  status");
    for (const r of rows) {
      const enabled = r.enabled === null ? "-" : r.enabled ? "yes" : "no";
      const position = r.position === null ? "-" : (deg(r.position) >= 0 ? "+" : "") + deg(r.position).toFixed(1);
      const temperature = r.temperature === null ? "-" : r.temperature.toFixed(1);
      const status = r.fault ? `FAULT 0x${r.faultCode.toString(16).toUpperCase().padStart(8, "0")}: ${r.fault}`
        : r.outsideLimits ? "ok -- outside limits, re-zero needed" : "ok";
      console.log(`  ${String(r.motorId).padStart(3)} ${(r.joint ?? "(not in profile)").padEnd(16)} ${position.padStart(12)}  ${temperature.padStart(6)}  ${enabled.padStart(7)}  ${status}`);
    }
  }
  if (missing.length) console.log(`  in the profile but not found: ${JSON.stringify(missing)}`);
  ok = rows.length > 0 && !missing.length;
} finally {
  await bus.close();
  if (backend.closeError) console.error(`adapter close: ${backend.closeError}`);
  if (frames && typeof args.frames === "string") writeFrames(args.frames, frames);
}
process.exit(ok ? 0 : 1);
