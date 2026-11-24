/** Shared by the Node examples: arguments, the rig file, opening an arm's adapter, frame logs. */

import { existsSync, readFileSync, writeFileSync } from "node:fs";
import { homedir } from "node:os";
import { join } from "node:path";
import type { ArmConfig, CanBackend, GsUsbBackend, RawProfile } from "@makermods/arm-core";
import { openAdapter } from "@makermods/arm-core/node";

export type Args = Record<string, string | boolean | undefined>;

/**
 * `--name value` and `--flag` into an object. `spec` names every option and whether it takes a
 * value; anything else, or a missing value, is an error before any device is opened.
 */
export function parseArgs(argv: string[], spec: Record<string, "value" | "flag">): Args {
  const args: Args = {};
  for (let i = 0; i < argv.length; i++) {
    const item = argv[i]!;
    const name = item.startsWith("--") ? item.slice(2) : "";
    const kind = spec[name];
    if (!kind) throw new Error(`unknown argument ${item} (options: ${Object.keys(spec).map((k) => `--${k}`).join(" ")})`);
    if (kind === "flag") {
      args[name] = true;
      continue;
    }
    const value = argv[++i];
    if (value === undefined || value.startsWith("--")) throw new Error(`${item} needs a value`);
    args[name] = value;
  }
  return args;
}

export const COMMON_OPTIONS = { product: "value", port: "value", frames: "value" } as const;

export type ProductName = "maker" | "metal";

/**
 * The rig file says which arm is on which adapter: `{"gs_usb:<serial>": "metal", ...}` in
 * $MAKERMODS_RIG or ~/.config/makermods-arm/rig.json. The examples refuse a product/adapter pair it
 * does not list: one arm's frames mean something else to the other arm's motors, and a frame sent
 * to the wrong arm can move it.
 */
export function checkRig(product: string, port: string | undefined): asserts product is ProductName {
  if (product !== "maker" && product !== "metal") throw new Error(`--product must be maker or metal, got ${product}`);
  if (!port) throw new Error("--port gs_usb:<serial> is required");
  const path = process.env.MAKERMODS_RIG ?? join(homedir(), ".config", "makermods-arm", "rig.json");
  if (!existsSync(path)) {
    throw new Error(`no rig file at ${path}: create it with {"${port}": "${product}"} once you are sure which arm is on that adapter`);
  }
  const rig = JSON.parse(readFileSync(path, "utf8")) as Record<string, string>;
  if (rig[port] !== product) {
    throw new Error(`${path} lists ${port} as ${rig[port] ?? "nothing"}, not a ${product} arm; refusing to send it ${product} frames`);
  }
}

export async function defaultConfig(product: ProductName, overrides: RawProfile = {}): Promise<ArmConfig> {
  if (product === "maker") return (await import("@makermods/maker-arm")).defaultConfig(overrides);
  return (await import("@makermods/metal-arm")).defaultConfig(overrides);
}

/** The profile of `product` and its adapter (not opened), after the rig check. */
export async function openTarget(args: Args, overrides: RawProfile = {}): Promise<{ config: ArmConfig; backend: GsUsbBackend }> {
  const product = String(args.product ?? "");
  const port = typeof args.port === "string" ? args.port : undefined;
  checkRig(product, port);
  return { config: await defaultConfig(product, overrides), backend: await openAdapter(port) };
}

export interface SentFrame {
  t: number;
  id: number;
  data: string;
  extended: boolean;
}

/** Record every frame handed to `backend.send`, in order (only when a log was asked for). */
export function recordSends(backend: CanBackend): SentFrame[] {
  const frames: SentFrame[] = [];
  const send = backend.send.bind(backend);
  backend.send = (canId: number, data: Uint8Array, extended = true) => {
    frames.push({ t: performance.now() / 1000, id: canId, data: Buffer.from(data).toString("hex"), extended });
    return send(canId, data, extended);
  };
  return frames;
}

export function writeFrames(path: string, frames: SentFrame[]): void {
  writeFileSync(path, frames.map((f) => JSON.stringify(f)).join("\n") + "\n");
}

export const deg = (rad: number): number => rad * (180 / Math.PI);
export const rad = (degrees: number): number => degrees * (Math.PI / 180);
