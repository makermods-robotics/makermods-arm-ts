/**
 * The mock hardware inside a Web Worker the page started: loaded before the app's worker module (see install.ts),
 * it gives the worker a `navigator.usb` whose `getDevices()` lists the adapters the page granted, each with its own
 * simulated motors here, where the worker opens it.
 */

import type { Product } from "./can-arm.ts";
import { FakeUsb, installUsb } from "./fake-usb.ts";
import { Bench, CHANNEL, callOp, type HubMessage } from "./hub.ts";

const plugged = JSON.parse(new URL(import.meta.url).searchParams.get("plugged") ?? "[]") as [Product, string][];
const channel = new BroadcastChannel(CHANNEL);
const bench = new Bench();
let nextId = 1;
const waiting = new Map<number, (serials: string[]) => void>();

const usb = new FakeUsb(
  () =>
    new Promise<string[]>((resolve) => {
      const id = nextId++;
      waiting.set(id, resolve);
      channel.postMessage({ kind: "granted?", id } satisfies HubMessage);
    }),
);
for (const [product, serial] of plugged) usb.plug(bench.add(product, serial).device);
installUsb(usb);

channel.onmessage = (event: MessageEvent<HubMessage>) => {
  const m = event.data;
  if (m.kind === "granted") {
    waiting.get(m.id)?.(m.serials);
    waiting.delete(m.id);
  } else if (m.kind === "plug") {
    usb.plug(bench.add(m.product, m.serial).device);
  } else if (m.kind === "unplug") {
    const arm = bench.arms.get(m.serial);
    if (arm) usb.unplug(arm.device);
  } else if (m.kind === "call") {
    const arm = bench.active(m.serial);
    if (!arm) return; // not opened here
    try {
      channel.postMessage({ kind: "result", id: m.id, value: callOp(arm, m.op, m.args) } satisfies HubMessage);
    } catch (e) {
      channel.postMessage({ kind: "result", id: m.id, error: String(e) } satisfies HubMessage);
    }
  }
};
